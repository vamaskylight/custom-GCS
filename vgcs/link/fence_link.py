"""The geofence part of the link thread: circle fence, polygon fence, switching it off, reading it.

A mixin of MavlinkThread (vgcs/link/mavlink_thread.py). What a polygon is and
when it can be used is in vgcs/mission/fence.py, the exchange with the drone
in vgcs/link/fence_transfer.py. This file ties the two to the link.

A polygon fence goes to the drone in these steps, and each one has to work
before the next:

1. The shape is checked (corners in order, no side crossing another).
2. It has to hold the drone: 5 m or more inside. For an armed drone that is a
   refusal, also for its home point and its height: a drone outside its fence
   turns back or lands the moment the fence is on (simulator, 2026-10-14).
   On the ground the operator is asked, since the drone only refuses to arm.
3. The fence the drone holds now is read. Other areas in it (another ground
   station's) would be removed by the upload, so the operator is asked.
4. The corners are sent, and the drone acknowledges them.
5. The fence is read back and compared, to the last digit.
6. The settings are written and echoed: height limit and polygon on, the
   action, FENCE_ENABLE 1.

Only then VGCS says that the fence is on. Nothing here waits for the drone:
the link's loop feeds in what it hears (see fence_transfer.py).
"""

from __future__ import annotations

import time

from vgcs.link.fence_transfer import (
    CHECK_TRIES,
    CHECK_WAIT_S,
    FenceJob,
    FenceRead,
    FenceUpload,
    MessageAsk,
)
from vgcs.mission.fence import (
    EDGE_MIN_M,
    HEIGHT_MIN_M,
    TYPE_ALT_MAX,
    TYPE_CIRCLE,
    TYPE_POLYGON,
    action_words,
    check_polygon,
    distance_inside_m,
    distance_m,
    item_from_row,
    kinds_words,
    outside_words,
    polygon_items,
    read_fence,
    same_items,
)

MISSION_TYPE_FENCE = 1
MAV_FRAME_GLOBAL = 0
MAV_CMD_REQUEST_MESSAGE = 512
MSG_ID_HOME_POSITION = 242

# A position older than this is not where the drone is.
POSITION_MAX_AGE_S = 5.0
# The first read after an upload: the drone gets a moment to list what it stored.
FIRST_CHECK_WAIT_S = 0.2

FENCE_SETTINGS = ("FENCE_ENABLE", "FENCE_TYPE", "FENCE_ALT_MAX", "FENCE_ACTION", "FENCE_RADIUS")

_AT_ONCE = "A flying drone would turn back or land at once. Not sent."


def _sentence(text: str) -> str:
    text = str(text or "").strip()
    return text[:1].upper() + text[1:] if text else text


class FenceLinkMixin:
    """Geofence commands of the link thread. See the module docstring."""

    # --- what the thread knows ----------------------------------------------

    def _fence_job_open(self) -> bool:
        return getattr(self, "_fence_job", None) is not None

    def _note_position(self, lat: float, lon: float, relative_alt_m: float) -> None:
        """The flight controller's own position, kept for the fence checks."""
        if abs(lat) < 1e-9 and abs(lon) < 1e-9:
            return                 # no position yet
        self._fence_last_position = (float(lat), float(lon), float(relative_alt_m), time.monotonic())

    def _fence_position(self) -> tuple[float, float, float] | None:
        """(lat, lon, height above home) when it is fresh, else None."""
        last = getattr(self, "_fence_last_position", None)
        if last is None or time.monotonic() - last[3] > POSITION_MAX_AGE_S:
            return None
        return last[0], last[1], last[2]

    def _link_is_mavlink2(self) -> bool:
        return str(getattr(self._master, "WIRE_PROTOCOL_VERSION", "")) == "2.0"

    def _is_fence_answer_for_us(self, msg) -> bool:
        """From the flight controller, about the fence, and sent to this station."""
        if not self._is_primary_source(msg):
            return False
        if int(getattr(msg, "mission_type", 0) or 0) != MISSION_TYPE_FENCE:
            return False
        target = int(getattr(msg, "target_system", 0) or 0)
        mine = getattr(getattr(self._master, "mav", None), "srcSystem", None)
        return target == 0 or mine is None or int(mine) == target

    # --- sending ------------------------------------------------------------

    def _send_fence_count(self, count: int) -> None:
        self._master.mav.mission_count_send(self._target_sysid, self._target_compid, int(count), MISSION_TYPE_FENCE)

    def _send_fence_item(self, seq: int, item) -> None:
        self._master.mav.mission_item_int_send(
            self._target_sysid, self._target_compid, int(seq), MAV_FRAME_GLOBAL, int(item.command), 0, 1,
            float(item.p1), 0.0, 0.0, 0.0, int(item.lat_e7), int(item.lon_e7), 0.0, MISSION_TYPE_FENCE)

    def _send_fence_list_request(self) -> None:
        self._master.mav.mission_request_list_send(self._target_sysid, self._target_compid, MISSION_TYPE_FENCE)

    def _send_fence_item_request(self, seq: int) -> None:
        self._master.mav.mission_request_int_send(self._target_sysid, self._target_compid, int(seq), MISSION_TYPE_FENCE)

    def _send_fence_read_ack(self) -> None:
        self._master.mav.mission_ack_send(self._target_sysid, self._target_compid, 0, MISSION_TYPE_FENCE)

    def _send_home_request(self) -> None:
        self._send_command_long(MAV_CMD_REQUEST_MESSAGE, p1=float(MSG_ID_HOME_POSITION))

    def _fence_try(self, send, *args) -> None:
        """A send that fails is tried again by the step's own timer, and reported when that runs out."""
        try:
            send(*args)
        except Exception as e:
            self.log_line.emit(f"Geofence: a message could not be sent ({e})")

    # --- results ------------------------------------------------------------

    def _fence_fail(self, text: str) -> None:
        """End the open request, and say why."""
        self._fence_job = None
        self.geofence_result.emit(False, text)
        self.error.emit(f"Geofence: {text}")

    def _fence_refuse(self, text: str) -> None:
        """A request that is not started (another one may be open: it is left alone)."""
        self.geofence_result.emit(False, text)
        self.error.emit(f"Geofence: {text}")

    def _fence_written(self, results: dict, ok_text: str, state: dict | None = None) -> None:
        """Report a fence only once the drone has echoed every one of its settings."""
        failed = [f"{name}: {detail}" for name, (ok, detail) in results.items() if not ok]
        if failed:
            self.geofence_result.emit(False, "; ".join(failed))
            self.error.emit("Geofence not set: " + "; ".join(failed))
            return
        if state is not None:
            self.fence_state.emit(dict(state))
        self.geofence_result.emit(True, ok_text)
        self.log_line.emit(f"Geofence confirmed by the drone: {ok_text}")

    # --- the commands -------------------------------------------------------

    def _geofence_upload(self, cfg: dict) -> None:
        if self._master is None:
            self.geofence_result.emit(False, "Link not ready")
            return
        if bool(cfg.get("disable")):
            self._write_params(
                {"FENCE_ENABLE": 0.0},
                lambda results: self._fence_written(results, "Fence disabled", {"source": "off", "enabled": False}),
            )
            return
        if cfg.get("points") is not None:
            # A polygon was asked for. One that cannot be sent is said so: it
            # is never replaced by a circle the operator did not ask for.
            self._polygon_fence_start(cfg)
            return
        self._circle_fence_start(cfg)

    def _geofence_download(self) -> None:
        """Read the fence the drone holds: its corners and its settings."""
        if self._master is None:
            self.geofence_result.emit(False, "Link not ready")
            return
        if self._fence_job_open():
            self._fence_refuse("Another fence request is still running. Try again in a moment.")
            return
        if not self._link_is_mavlink2():
            self._fence_refuse(self._mavlink1_words())
            return
        job = FenceJob("read", {}, time.monotonic())
        self._fence_job = job
        self._fence_read(job, "show")

    @staticmethod
    def _mavlink1_words() -> str:
        return ("This link speaks MAVLink 1, which cannot carry a polygon fence. "
                "Set the drone's telemetry port to MAVLink 2 (SERIALx_PROTOCOL 2).")

    # --- the circle fence around home -----------------------------------------

    def _circle_fence_start(self, cfg: dict) -> None:
        radius_m = max(10.0, float(cfg.get("radius_m", 80.0)))
        alt_max_m = max(5.0, float(cfg.get("alt_max_m", 120.0)))
        # Common ArduPilot meanings: 0=None, 1=RTL, 2=Land. Default to RTL, so
        # there is an actual restriction on breach.
        action = float(cfg.get("action", 1.0))
        if not bool(getattr(self, "_vehicle_armed", False)):
            # On the ground the drone itself refuses to arm outside its fence.
            self._circle_fence_write(radius_m, alt_max_m, action)
            return
        if self._fence_job_open():
            self._fence_refuse("Another fence request is still running. Try again in a moment.")
            return
        position = self._fence_position()
        if position is None:
            self._fence_refuse("The drone is armed and VGCS does not know its position, so it cannot tell "
                               "whether this fence holds the drone. Not sent.")
            return
        problem = self._height_problem(position[2], alt_max_m)
        if problem:
            self._fence_refuse(problem)
            return
        job = FenceJob("circle", cfg, time.monotonic())
        job.radius_m, job.alt_max_m, job.action = radius_m, alt_max_m, action
        self._fence_job = job
        self._fence_ask_home(job)

    def _circle_fence_write(self, radius_m: float, alt_max_m: float, action: float) -> None:
        # Bit 1 is the height limit, bit 2 the circle. This was 2 (the circle
        # alone), so FENCE_ALT_MAX was written but never enforced: the drone
        # could climb through the operator's height limit (found by the
        # simulator test, 2026-10-07).
        kinds = TYPE_ALT_MAX | TYPE_CIRCLE
        state = {"source": "circle", "enabled": True, "kinds": kinds, "radius_m": radius_m,
                 "alt_max_m": alt_max_m, "action": action}
        self._write_params(
            {
                "FENCE_ENABLE": 1.0,
                "FENCE_ACTION": action,
                "FENCE_TYPE": float(kinds),
                "FENCE_RADIUS": radius_m,
                "FENCE_ALT_MAX": alt_max_m,
            },
            lambda results: self._fence_written(
                results, f"Fence enabled: radius={radius_m:.0f}m alt_max={alt_max_m:.0f}m", state
            ),
        )

    @staticmethod
    def _height_problem(height_m: float, alt_max_m: float) -> str:
        if height_m <= alt_max_m - HEIGHT_MIN_M:
            return ""
        return (f"The drone is at {height_m:.0f} m. This fence's max height is {alt_max_m:.0f} m, "
                f"and the drone has to be {HEIGHT_MIN_M:.0f} m or more below it. {_AT_ONCE}")

    # --- the polygon fence ----------------------------------------------------

    def _polygon_fence_start(self, cfg: dict) -> None:
        if self._fence_job_open():
            self._fence_refuse("Another fence request is still running. Try again in a moment.")
            return
        corners, problem = check_polygon(cfg.get("points"))
        if problem:
            self._fence_refuse(f"{problem} Not sent.")
            return
        if not self._link_is_mavlink2():
            self._fence_refuse(self._mavlink1_words())
            return
        alt_max_m = max(5.0, float(cfg.get("alt_max_m", 120.0)))
        action = float(cfg.get("action", 1.0))
        job = FenceJob("polygon", cfg, time.monotonic())
        job.corners = corners
        job.items = polygon_items(corners)
        job.alt_max_m, job.action = alt_max_m, action
        position = self._fence_position()

        if bool(getattr(self, "_vehicle_armed", False)):
            if position is None:
                self._fence_refuse("The drone is armed and VGCS does not know its position, so it cannot tell "
                                   "whether this fence holds the drone. Not sent.")
                return
            inside = distance_inside_m(corners, position[0], position[1])
            if inside < EDGE_MIN_M:
                self._fence_refuse(f"{outside_words('The drone', inside)} {_AT_ONCE}")
                return
            problem = self._height_problem(position[2], alt_max_m)
            if problem:
                self._fence_refuse(problem)
                return
            job.checked_armed = True
            self._fence_job = job
            self._fence_ask_home(job)
            return

        # On the ground: the drone refuses to arm outside its fence, so the
        # operator may send one that does not hold it (the fence of a field
        # the drone is not standing in yet). It is their choice, not a default.
        if position is None:
            job.notes.append("The drone's position is not known, so VGCS did not check that the fence holds it")
        else:
            inside = distance_inside_m(corners, position[0], position[1])
            if inside < EDGE_MIN_M:
                if not bool(cfg.get("allow_outside")):
                    self._fence_ask_operator(
                        "outside", cfg, "allow_outside",
                        f"{outside_words('The drone', inside)} It will not arm until it stands inside the fence."
                        "\n\nSend this fence anyway?")
                    return
                job.notes.append("The drone is not inside this fence: it will not arm until it is")
        self._fence_job = job
        self._fence_read(job, "before")

    def _fence_ask_operator(self, kind: str, cfg: dict, flag: str, text: str) -> None:
        """Nothing is sent. The window asks, and sends the request again with the answer in it."""
        again = dict(cfg)
        again[flag] = True
        self.log_line.emit(f"Geofence: not sent yet, the operator is asked ({kind})")
        self.fence_question.emit({"kind": kind, "cfg": again, "text": text})

    # --- step: the home point (an armed drone) --------------------------------

    def _fence_ask_home(self, job: FenceJob) -> None:
        job.home = MessageAsk(time.monotonic(), lambda payload: self._fence_home_heard(job, payload))
        self._fence_try(self._send_home_request)

    def _hear_home_position(self, msg) -> None:
        job = getattr(self, "_fence_job", None)
        if job is None or job.home is None or not self._is_primary_source(msg):
            return
        try:
            lat = float(getattr(msg, "latitude", 0) or 0) / 1e7
            lon = float(getattr(msg, "longitude", 0) or 0) / 1e7
        except (TypeError, ValueError):
            return
        if abs(lat) < 1e-9 and abs(lon) < 1e-9:
            return
        job.home.heard((lat, lon))

    def _fence_home_heard(self, job: FenceJob, home) -> None:
        if getattr(self, "_fence_job", None) is not job:
            return
        job.home = None
        if home is None:
            self._fence_fail("The drone did not tell its home point, so VGCS cannot tell whether this fence "
                             "holds it. Not sent.")
            return
        if job.kind == "polygon":
            inside = distance_inside_m(job.corners, home[0], home[1])
            if inside < EDGE_MIN_M:
                words = outside_words("The drone's home point", inside)
                self._fence_fail(f"{words} The drone could not return home inside this fence. Not sent.")
                return
            self._fence_read(job, "before")
            return
        # The circle is around home.
        position = self._fence_position()
        if position is None:
            self._fence_fail("VGCS lost the drone's position, so it cannot tell whether this fence holds "
                             "the drone. Not sent.")
            return
        away = distance_m(home, (position[0], position[1]))
        if away > job.radius_m - EDGE_MIN_M:
            self._fence_fail(f"The drone is {away:.0f} m from its home point. A fence of {job.radius_m:.0f} m "
                             f"around home has to leave it {EDGE_MIN_M:.0f} m or more. {_AT_ONCE}")
            return
        self._fence_job = None
        self._circle_fence_write(job.radius_m, job.alt_max_m, job.action)

    # --- step: reading the drone's fence ---------------------------------------

    def _fence_read(self, job: FenceJob, purpose: str) -> None:
        job.read_purpose = purpose
        job.check_again_at = None
        job.read = FenceRead(time.monotonic(), lambda rows, detail: self._fence_read_done(job, rows, detail))
        self._fence_try(self._send_fence_list_request)

    def _hear_fence_count(self, msg) -> None:
        job = getattr(self, "_fence_job", None)
        if job is None or job.read is None or not self._is_fence_answer_for_us(msg):
            return
        ask = job.read.heard_count(int(getattr(msg, "count", 0) or 0), time.monotonic())
        if ask is not None:
            self._fence_try(self._send_fence_item_request, ask)

    def _hear_fence_item(self, msg) -> bool:
        """True when the item was the fence's (so it is not the mission's)."""
        if int(getattr(msg, "mission_type", 0) or 0) != MISSION_TYPE_FENCE:
            return False
        job = getattr(self, "_fence_job", None)
        if job is None or job.read is None or not self._is_fence_answer_for_us(msg):
            return True
        if msg.get_type() == "MISSION_ITEM_INT":
            lat_e7, lon_e7 = int(getattr(msg, "x", 0) or 0), int(getattr(msg, "y", 0) or 0)
        else:
            lat_e7 = int(round(float(getattr(msg, "x", 0.0) or 0.0) * 1e7))
            lon_e7 = int(round(float(getattr(msg, "y", 0.0) or 0.0) * 1e7))
        row = {"command": int(getattr(msg, "command", 0) or 0), "p1": float(getattr(msg, "param1", 0.0) or 0.0),
               "lat_e7": lat_e7, "lon_e7": lon_e7}
        ask = job.read.heard_item(int(getattr(msg, "seq", 0) or 0), row, time.monotonic())
        if ask is not None:
            self._fence_try(self._send_fence_item_request, ask)
        return True

    def _hear_fence_ack(self, msg) -> bool:
        """True when the MISSION_ACK was about the fence."""
        if int(getattr(msg, "mission_type", 0) or 0) != MISSION_TYPE_FENCE:
            return False
        job = getattr(self, "_fence_job", None)
        if job is None or not self._is_fence_answer_for_us(msg):
            return True
        result = int(getattr(msg, "type", 0) or 0)
        now = time.monotonic()
        if job.upload is not None:
            job.upload.heard_ack(result, now)
        elif job.read is not None:
            job.read.heard_ack(result, now)
        return True

    def _fence_read_done(self, job: FenceJob, rows, detail: str) -> None:
        if getattr(self, "_fence_job", None) is not job:
            return
        purpose = job.read_purpose
        job.read = None
        if rows is not None:
            # The drone waits for this before it takes the next request.
            self._fence_try(self._send_fence_read_ack)
        items = None if rows is None else [item_from_row(row) for row in rows]

        if purpose == "check":
            if items is not None and same_items(job.items, items):
                self._polygon_fence_settings(job)
                return
            job.check_tries += 1
            if job.check_tries < CHECK_TRIES:
                job.check_again_at = time.monotonic() + CHECK_WAIT_S
                return
            if items is None:
                self._fence_fail(f"The drone took the fence, but VGCS could not read it back ({detail}). "
                                 "Press Read fence from drone to see what the drone holds now.")
            else:
                self._fence_fail("The drone took the fence, but what it gives back is not what VGCS sent "
                                 f"({len(items)} items instead of {len(job.items)} corners, or other places). "
                                 "Switch the fence off, or upload it again.")
            return

        if items is None:
            what = "the fence the drone holds now" if purpose == "before" else "the drone's fence"
            self._fence_fail(f"VGCS could not read {what} ({detail})."
                             + (" Nothing was sent." if purpose == "before" else ""))
            return
        on_drone = read_fence(items)
        if purpose == "before":
            if on_drone.other and not bool(job.cfg.get("replace_other")):
                self._fence_job = None
                self._fence_ask_operator(
                    "other", job.cfg, "replace_other",
                    "The fence on the drone also holds: " + "; ".join(on_drone.other) + "."
                    "\nVGCS does not show these, and this upload removes them."
                    "\n\nSend this fence anyway?")
                return
            self._fence_send(job)
            return
        # "show": the settings belong to the picture.
        job.on_drone = on_drone
        job.settings_open = True
        names = self._param_reads.start(
            list(FENCE_SETTINGS), time.monotonic(),
            lambda values, missing: self._fence_settings_read(job, values, missing))
        for name in names:
            self._send_param_request(name)

    # --- step: sending the corners ---------------------------------------------

    def _fence_send(self, job: FenceJob) -> None:
        job.upload = FenceUpload(job.items, time.monotonic(),
                                 lambda ok, detail: self._fence_upload_done(job, ok, detail))
        self.log_line.emit(f"Geofence: sending a polygon of {len(job.items)} corners")
        self._fence_try(self._send_fence_count, len(job.items))

    def _hear_fence_request(self, msg) -> None:
        job = getattr(self, "_fence_job", None)
        if job is None or job.upload is None or not self._is_fence_answer_for_us(msg):
            return
        seq = int(getattr(msg, "seq", 0) or 0)
        item = job.upload.heard_request(seq, time.monotonic())
        if item is not None:
            self._fence_try(self._send_fence_item, seq, item)

    def _hear_fence_text(self, text: str) -> None:
        job = getattr(self, "_fence_job", None)
        if job is not None and job.upload is not None:
            job.upload.heard_text(text)

    def _fence_upload_done(self, job: FenceJob, ok: bool, detail: str) -> None:
        if getattr(self, "_fence_job", None) is not job:
            return
        job.upload = None
        if not ok:
            refused = "did not take the fence" in detail
            self._fence_fail(_sentence(detail) + ("" if detail.endswith((".", '"')) else ".")
                             + (" It keeps the fence it had." if refused
                                else " Press Read fence from drone to see what the drone holds now."))
            return
        # Acknowledged is not yet stored: read it back and compare.
        job.read_purpose = "check"
        job.check_tries = 0
        job.check_again_at = time.monotonic() + FIRST_CHECK_WAIT_S

    # --- step: the settings ----------------------------------------------------

    def _polygon_fence_settings(self, job: FenceJob) -> None:
        if bool(getattr(self, "_vehicle_armed", False)) and not job.checked_armed:
            # The checks were those for a drone on the ground (it may stand
            # outside the fence, its home point was not asked for). Armed
            # since, it gets no fence switched on that was never checked for
            # flight.
            self._fence_fail("The drone was armed while the fence was on its way. It holds the new polygon, "
                             "and its fence settings were left as they were. Press Read fence from drone to "
                             "see whether its fence is on, or upload the fence again.")
            return
        job.settings_open = True
        kinds = TYPE_ALT_MAX | TYPE_POLYGON
        self._write_params(
            {
                "FENCE_ALT_MAX": job.alt_max_m,
                "FENCE_ACTION": job.action,
                "FENCE_TYPE": float(kinds),
                # Last: the fence is switched on once it is the right one.
                "FENCE_ENABLE": 1.0,
            },
            lambda results: self._polygon_fence_written(job, results),
        )

    def _polygon_fence_written(self, job: FenceJob, results: dict) -> None:
        if getattr(self, "_fence_job", None) is job:
            self._fence_job = None
        failed = [f"{name}: {detail}" for name, (ok, detail) in results.items() if not ok]
        if failed:
            text = ("The drone holds the polygon, but these settings were not confirmed: " + "; ".join(failed)
                    + ". The fence may not be switched on. Upload it again.")
            self.geofence_result.emit(False, text)
            self.error.emit(f"Geofence: {text}")
            return
        count = len(job.corners)
        text = (f"Polygon fence on: {count} corners, max height {job.alt_max_m:.0f} m, "
                f"on breach {action_words(job.action)}")
        if job.notes:
            text += ". " + ". ".join(job.notes)
        self.fence_state.emit({
            "source": "upload", "enabled": True, "kinds": TYPE_ALT_MAX | TYPE_POLYGON,
            "alt_max_m": job.alt_max_m, "action": job.action,
            "polygon": [[lat, lon] for lat, lon in job.corners], "other": [], "problem": "",
        })
        self.geofence_result.emit(True, text)
        self.log_line.emit(f"Geofence confirmed by the drone: {text}")

    def _fence_settings_read(self, job: FenceJob, values: dict, missing: list) -> None:
        """The last step of "Read fence from drone"."""
        if getattr(self, "_fence_job", None) is job:
            self._fence_job = None
        on_drone = job.on_drone
        enabled = None if "FENCE_ENABLE" not in values else bool(round(float(values["FENCE_ENABLE"])))
        kinds = None if "FENCE_TYPE" not in values else int(round(float(values["FENCE_TYPE"])))
        state = {
            "source": "download", "enabled": enabled, "kinds": kinds,
            "alt_max_m": values.get("FENCE_ALT_MAX"), "action": values.get("FENCE_ACTION"),
            "radius_m": values.get("FENCE_RADIUS"),
            "polygon": [[lat, lon] for lat, lon in on_drone.polygon],
            "other": list(on_drone.other), "problem": on_drone.problem,
        }
        self.fence_state.emit(state)
        self.geofence_result.emit(True, describe_fence(state))
        self.log_line.emit(f"Geofence read from the drone: {on_drone.item_count} items, "
                           f"settings {', '.join(f'{k}={v:g}' for k, v in values.items()) or 'none'}"
                           + (f"; not answered: {', '.join(missing)}" if missing else ""))

    # --- the loop --------------------------------------------------------------

    def _tick_fence(self, now: float) -> None:
        """Send again what got no answer, and settle what ran out of time."""
        job = getattr(self, "_fence_job", None)
        if job is None:
            return
        if job.home is not None and job.home.tick(now):
            self._fence_try(self._send_home_request)
        if getattr(self, "_fence_job", None) is not job:
            return
        if job.upload is not None and job.upload.tick(now):
            self._fence_try(self._send_fence_count, len(job.items))
        if getattr(self, "_fence_job", None) is not job:
            return
        if job.read is not None:
            again = job.read.tick(now)
            if again == "list":
                self._fence_try(self._send_fence_list_request)
            elif again is not None:
                self._fence_try(self._send_fence_item_request, again)
        elif job.check_again_at is not None and now >= job.check_again_at:
            self._fence_read(job, "check")

    def _cancel_fence(self, reason: str) -> None:
        """The link is gone. The settings step, when open, has reported already."""
        job = getattr(self, "_fence_job", None)
        if job is None:
            return
        # Its open step is heard no more: every answer of the drone goes
        # through the request that is open, and none is.
        self._fence_job = None
        self.geofence_result.emit(False, f"The fence request was not finished: {reason}.")


def describe_fence(state: dict) -> str:
    """What the drone's fence is, in a sentence or two for the operator."""
    enabled = state.get("enabled")
    kinds = state.get("kinds")
    polygon = state.get("polygon") or []
    parts: list[str] = []
    if enabled is None:
        parts.append("The drone did not tell whether its fence is switched on")
    elif not enabled:
        parts.append("The drone's fence is switched off")
    else:
        limits: list[str] = []
        bits = int(kinds or 0)
        if bits & TYPE_ALT_MAX and state.get("alt_max_m") is not None:
            limits.append(f"max height {float(state['alt_max_m']):.0f} m")
        if bits & TYPE_CIRCLE and state.get("radius_m") is not None:
            limits.append(f"circle of {float(state['radius_m']):.0f} m around home")
        if bits & TYPE_POLYGON:
            limits.append(f"polygon of {len(polygon)} corners" if polygon else "polygon (but the drone holds none)")
        if not limits:
            limits.append(f"no limit chosen ({kinds_words(kinds)})")
        parts.append("The drone's fence is switched on: " + ", ".join(limits)
                     + f", on breach {action_words(state.get('action'))}")
    if polygon and not (enabled and int(kinds or 0) & TYPE_POLYGON):
        parts.append(f"It holds a polygon of {len(polygon)} corners that is not in use")
    if state.get("problem"):
        parts.append(str(state["problem"]).rstrip("."))
    if state.get("other"):
        parts.append("It also holds: " + "; ".join(state["other"]) + " (VGCS does not show these)")
    return ". ".join(parts) + "."
