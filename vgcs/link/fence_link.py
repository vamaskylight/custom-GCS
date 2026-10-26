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

Areas to stay out of (2026-10-21) go with either fence as fence items of their
own: with the polygon, or with the circle around home (then the circle and the
items are both on, FENCE_TYPE 7). The drone has to be 5 m or more outside each
of them, as it has to be inside the polygon, and in flight its home point too.
A circle without such areas is only settings, as before.

They can also go alone (2026-10-26, client feedback of 2026-10-22: there was no
way to send them without the circle): the items are the areas only, and
FENCE_TYPE is the max height and the items (5), so a circle fence the drone had
is off afterwards.

More areas to stay inside (polygons and circles) and a return point go with the
polygon, or without it (2026-10-27, DGCA FEN-12). The drone has to be inside
them as the drone takes them together: inside all of them at once (their
overlap, ArduPilot's default), or inside any of them (FENCE_OPTIONS bit 1,
ArduCopter 4.6 or newer). That bit is set before the upload and cleared after
it, so the drone never takes the new areas more tightly than VGCS checked them.
A multicopter keeps the return point, but does not fly to it (only ArduPlane
does).
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
    OPTION_UNION,
    TYPE_ALT_MAX,
    TYPE_CIRCLE,
    TYPE_POLYGON,
    NoFly,
    action_words,
    check_keepin,
    check_nofly,
    check_polygon,
    check_return_point,
    distance_m,
    fence_items,
    fence_total_points,
    inside_margin_m,
    item_from_row,
    keepin_from_rows,
    kinds_words,
    nofly_from_rows,
    nofly_outside_m,
    outside_words,
    read_fence,
    same_items,
    same_nofly,
    union_words,
)

MISSION_TYPE_FENCE = 1
MAV_FRAME_GLOBAL = 0
MAV_CMD_REQUEST_MESSAGE = 512
MSG_ID_HOME_POSITION = 242

# A position older than this is not where the drone is.
POSITION_MAX_AGE_S = 5.0
# The first read after an upload: the drone gets a moment to list what it stored.
FIRST_CHECK_WAIT_S = 0.2
# A fence of fewer than this many points (corners and circles) makes ArduPilot set FENCE_TOTAL to 0,
# and after FENCE_TOTAL is set to 0 the drone gets this long to clear its old fence (it looks 10
# times a second). See _fence_clear_then_send.
FENCE_TOTAL_MIN_POINTS = 3
TOTAL_CLEAR_WAIT_S = 0.5

# FENCE_OPTIONS is not on ArduCopter before 4.6 (then it is not answered): not last, so it is given up sooner.
FENCE_SETTINGS = ("FENCE_ENABLE", "FENCE_TYPE", "FENCE_OPTIONS", "FENCE_ALT_MAX", "FENCE_ACTION", "FENCE_RADIUS")

_AT_ONCE = "A flying drone would turn back or land at once. Not sent."
_UNION_LEFT = ("Its FENCE_OPTIONS was set before the upload, so the drone now takes several areas to stay inside "
               "as inside any of them.")


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
        job = getattr(self, "_fence_job", None)
        self._fence_job = None
        if job is not None and job.union_set_before:
            text += f" {_UNION_LEFT}"
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
        if cfg.get("points") is not None or cfg.get("keepin"):
            # A polygon was asked for, or areas to stay inside. One that cannot
            # be sent is said so: it is never replaced by a circle the operator
            # did not ask for.
            self._items_fence_start(cfg, "polygon")
            return
        if cfg.get("areas_only"):
            # The areas to stay out of alone: no circle, no polygon.
            self._items_fence_start(cfg, "nofly")
            return
        if cfg.get("nofly"):
            # The circle around home, with areas to stay out of: they are items.
            self._items_fence_start(cfg, "circle_items")
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
        return ("This link speaks MAVLink 1, which cannot carry a polygon fence or areas to stay out of. "
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

    def _items_fence_start(self, cfg: dict, kind: str) -> None:
        """A fence of items: the polygon ("polygon") or the circle ("circle_items"), with areas to stay out
        of, or those areas alone ("nofly").

        "polygon" also takes further areas to stay inside (cfg "keepin", then the polygon itself may be
        left out), whether the drone may be in any of them (cfg "union"), and a return point (cfg
        "return_point"). DGCA FEN-12, 2026-10-27.
        """
        if self._fence_job_open():
            self._fence_refuse("Another fence request is still running. Try again in a moment.")
            return
        corners: list = []
        keepin: list = []
        return_point = None
        if kind == "polygon":
            keepin, problem = check_keepin(cfg.get("keepin") or [])
            if problem:
                self._fence_refuse(f"{problem} Not sent.")
                return
            if cfg.get("points") or not keepin:
                corners, problem = check_polygon(cfg.get("points"))
                if problem:
                    self._fence_refuse(f"{problem} Not sent.")
                    return
            return_point, problem = check_return_point(cfg.get("return_point"))
            if problem:
                self._fence_refuse(f"{problem} Not sent.")
                return
        nofly, problem = check_nofly(cfg.get("nofly") or [])
        if problem:
            self._fence_refuse(f"{problem} Not sent.")
            return
        if kind == "nofly" and not nofly:
            self._fence_refuse("There is no area to stay out of on the map. Not sent.")
            return
        union = bool(cfg.get("union"))
        inside_areas = ([NoFly(corners=tuple(corners), inside=True)] if corners else []) + list(keepin)
        if return_point is not None and inside_margin_m(inside_areas, return_point[0], return_point[1], union) < 0.0:
            self._fence_refuse("The return point is outside the areas to stay inside. Move it inside. Not sent.")
            return
        if not self._link_is_mavlink2():
            self._fence_refuse(self._mavlink1_words())
            return
        alt_max_m = max(5.0, float(cfg.get("alt_max_m", 120.0)))
        action = float(cfg.get("action", 1.0))
        job = FenceJob(kind, cfg, time.monotonic())
        job.corners = corners
        job.nofly = nofly
        job.keepin = list(keepin)
        job.inside_areas = inside_areas
        job.return_point = return_point
        job.union = union
        job.items = fence_items(corners, nofly, keepin, return_point)
        job.alt_max_m, job.action = alt_max_m, action
        if kind == "circle_items":
            job.radius_m = max(10.0, float(cfg.get("radius_m", 80.0)))
        position = self._fence_position()

        if bool(getattr(self, "_vehicle_armed", False)):
            if position is None:
                self._fence_refuse("The drone is armed and VGCS does not know its position, so it cannot tell "
                                   "whether this fence holds the drone. Not sent.")
                return
            if inside_areas:
                inside = inside_margin_m(inside_areas, position[0], position[1], union)
                if inside < EDGE_MIN_M:
                    self._fence_refuse(f"{outside_words('The drone', inside)} {_AT_ONCE}")
                    return
            problem = _nofly_problem(nofly, position[0], position[1], "The drone")
            if problem:
                self._fence_refuse(f"{problem} {_AT_ONCE}")
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
            if inside_areas:
                inside = inside_margin_m(inside_areas, position[0], position[1], union)
                if inside < EDGE_MIN_M:
                    if not bool(cfg.get("allow_outside")):
                        self._fence_ask_operator(
                            "outside", cfg, "allow_outside",
                            f"{outside_words('The drone', inside)} It will not arm until it stands inside the fence."
                            "\n\nSend this fence anyway?")
                        return
                    job.notes.append("The drone is not inside this fence: it will not arm until it is")
            problem = _nofly_problem(nofly, position[0], position[1], "The drone")
            if problem:
                if not bool(cfg.get("allow_nofly")):
                    self._fence_ask_operator(
                        "nofly", cfg, "allow_nofly",
                        f"{problem} It will not arm in an area to stay out of.\n\nSend this fence anyway?")
                    return
                job.notes.append("The drone stands in or near an area to stay out of: it will not arm inside one")
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
        if job.inside_areas:
            inside = inside_margin_m(job.inside_areas, home[0], home[1], job.union)
            if inside < EDGE_MIN_M:
                words = outside_words("The drone's home point", inside)
                self._fence_fail(f"{words} The drone could not return home inside this fence. Not sent.")
                return
        problem = _nofly_problem(getattr(job, "nofly", []), home[0], home[1], "The drone's home point")
        if problem:
            self._fence_fail(f"{problem} The drone could not return home. Not sent.")
            return
        if job.kind in ("polygon", "nofly"):
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
        if job.kind == "circle_items":
            self._fence_read(job, "before")
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
                                 f"({len(items)} items instead of {len(job.items)}, or other places). "
                                 "Switch the fence off, or upload it again.")
            return

        if items is None:
            what = "the fence the drone holds now" if purpose == "before" else "the drone's fence"
            self._fence_fail(f"VGCS could not read {what} ({detail})."
                             + (" Nothing was sent." if purpose == "before" else ""))
            return
        on_drone = read_fence(items)
        if purpose == "before":
            # Its areas (to stay inside after its polygon, and out of) and its return point that this
            # upload does not hold again, and what VGCS does not show. Its polygon is the one replaced.
            gone = [area.drone_words() for area in on_drone.nofly
                    if not any(same_nofly(area, kept) for kept in job.nofly)]
            gone += [area.drone_words() for area in on_drone.keepin
                     if not any(same_nofly(area, kept) for kept in job.inside_areas)]
            if on_drone.return_point is not None and job.return_point is None:
                gone.append("a return point")      # one on the map replaces it, as the polygon does
            removed = gone + list(on_drone.other)
            if removed and not bool(job.cfg.get("replace_other")):
                self._fence_job = None
                self._fence_ask_operator(
                    "other", job.cfg, "replace_other",
                    "The fence on the drone also holds: " + "; ".join(removed) + "."
                    "\nThey are not on the map, and this upload removes them."
                    "\n\nSend this fence anyway?")
                return
            self._fence_options_then_send(job)
            return
        # "show": the settings belong to the picture.
        job.on_drone = on_drone
        job.settings_open = True
        names = self._param_reads.start(
            list(FENCE_SETTINGS), time.monotonic(),
            lambda values, missing: self._fence_settings_read(job, values, missing))
        for name in names:
            self._send_param_request(name)

    # --- step: how the areas to stay inside go together --------------------------

    def _fence_options_then_send(self, job: FenceJob) -> None:
        """With two or more areas to stay inside: read FENCE_OPTIONS, and set its union bit now when the
        drone may be inside any of them.

        Set before the upload, so the drone never takes the new areas all at once while VGCS checked
        them as "inside any" (a flying drone would turn back or land). With the bit, the old fence only
        grows: a union holds all that the overlap holds. Cleared only after the upload (the settings
        step), for the same reason. ArduCopter has FENCE_OPTIONS since 4.6.0 (in 4.4 and 4.5 it is
        ArduPlane's only, read in their source 2026-10-27): there a drone takes its areas all at once.
        """
        if len(job.inside_areas) < 2:
            self._fence_clear_then_send(job)
            return
        names = self._param_reads.start(
            ["FENCE_OPTIONS"], time.monotonic(), lambda values, missing: self._fence_options_read(job, values))
        for name in names:
            self._send_param_request(name)

    def _fence_options_read(self, job: FenceJob, values: dict) -> None:
        if getattr(self, "_fence_job", None) is not job or self._master is None:
            return                 # the link closed (its reads end with no values): _cancel_fence answers
        if "FENCE_OPTIONS" not in values:
            if job.union:
                self._fence_fail("The drone did not give FENCE_OPTIONS, so it cannot be told that it may be inside "
                                 "any of the areas to stay inside (ArduCopter has it since 4.6). Choose inside all "
                                 "of them at once, or update the firmware. Nothing was sent.")
                return
            # A firmware without it takes the areas all at once, as asked.
            self._fence_clear_then_send(job)
            return
        job.options = int(round(float(values["FENCE_OPTIONS"])))
        if not job.union or job.options & OPTION_UNION:
            self._fence_clear_then_send(job)
            return
        self._write_params({"FENCE_OPTIONS": float(job.options | OPTION_UNION)},
                           lambda results: self._fence_union_set(job, results))

    def _fence_union_set(self, job: FenceJob, results: dict) -> None:
        if getattr(self, "_fence_job", None) is not job:
            return
        failed = [f"{name}: {detail}" for name, (ok, detail) in results.items() if not ok]
        if failed:
            self._fence_fail("The drone did not confirm FENCE_OPTIONS (" + "; ".join(failed) + "). "
                             "The fence was not sent.")
            return
        job.union_set_before = True
        self._fence_clear_then_send(job)

    # --- step: sending the corners ---------------------------------------------

    def _fence_clear_then_send(self, job: FenceJob) -> None:
        """Send the items, first setting FENCE_TOTAL to 0 when the fence has fewer than 3 points.

        ArduPilot's write_fence sets FENCE_TOTAL to 0 for a fence of fewer than 3 points (corners
        and circles), and a firmware with the old fence point protocol (4.6, and every simulator
        build) erases its whole fence store when FENCE_TOTAL goes from a number to 0: that was how an
        old ground station cleared its fence. So one or two circles sent after a larger fence were
        stored and erased at once (simulator, 4.6.2 and 4.7.0, 2026-10-26: "0 items instead of 1").
        With FENCE_TOTAL set to 0 first, the drone clears the old fence now (the operator was asked
        about what it holds, see "before"), and the new fence is then no change to FENCE_TOTAL. Where
        FENCE_TOTAL is 0 already, or the firmware has no such protocol (4.7 on a flight controller),
        this changes nothing.
        """
        if fence_total_points(job.items) >= FENCE_TOTAL_MIN_POINTS:
            self._fence_send(job)
            return
        self.log_line.emit("Geofence: FENCE_TOTAL 0 first, so the drone keeps a fence of fewer than 3 points")
        self._write_params({"FENCE_TOTAL": 0.0}, lambda results: self._fence_total_cleared(job, results))

    def _fence_total_cleared(self, job: FenceJob, results: dict) -> None:
        if getattr(self, "_fence_job", None) is not job:
            return
        # Not confirmed (a firmware without FENCE_TOTAL): the items go anyway, and the read back decides.
        job.send_at = time.monotonic() + TOTAL_CLEAR_WAIT_S

    def _fence_send(self, job: FenceJob) -> None:
        job.upload = FenceUpload(job.items, time.monotonic(),
                                 lambda ok, detail: self._fence_upload_done(job, ok, detail))
        self.log_line.emit(f"Geofence: sending {len(job.items)} fence items"
                           + (f" with {len(job.keepin)} more area(s) to stay inside" if job.keepin else "")
                           + (f" with {len(job.nofly)} area(s) to stay out of" if job.nofly else "")
                           + (" and a return point" if job.return_point is not None else ""))
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
            self._fence_fail(f"The drone was armed while the fence was on its way. It holds the new {_job_words(job)}, "
                             "and its fence settings were left as they were. Press Read fence from drone to "
                             "see whether its fence is on, or upload the fence again.")
            return
        job.settings_open = True
        settings = {"FENCE_ALT_MAX": job.alt_max_m, "FENCE_ACTION": job.action}
        if job.kind == "circle_items":
            # The circle around home, and the items (the areas to stay out of) too.
            settings["FENCE_RADIUS"] = job.radius_m
            settings["FENCE_TYPE"] = float(TYPE_ALT_MAX | TYPE_CIRCLE | TYPE_POLYGON)
        else:
            settings["FENCE_TYPE"] = float(TYPE_ALT_MAX | TYPE_POLYGON)
        if job.options is not None and not job.union and job.options & OPTION_UNION:
            # Cleared only now that the new areas are on the drone (see _fence_options_then_send).
            settings["FENCE_OPTIONS"] = float(job.options & ~OPTION_UNION)
        # Last: the fence is switched on once it is the right one.
        settings["FENCE_ENABLE"] = 1.0
        self._write_params(settings, lambda results: self._polygon_fence_written(job, results))

    def _polygon_fence_written(self, job: FenceJob, results: dict) -> None:
        if getattr(self, "_fence_job", None) is job:
            self._fence_job = None
        failed = [f"{name}: {detail}" for name, (ok, detail) in results.items() if not ok]
        if failed:
            text = (f"The drone holds the {_job_words(job)}, but these settings were not confirmed: "
                    + "; ".join(failed) + ". The fence may not be switched on. Upload it again.")
            self.geofence_result.emit(False, text)
            self.error.emit(f"Geofence: {text}")
            return
        nofly = _nofly_count_words(len(job.nofly))
        if job.kind == "polygon":
            parts = [f"{len(job.corners)} corners"] if job.corners else []
            if job.keepin:
                count = len(job.keepin)
                parts.append(f"{count} {'more ' if job.corners else ''}area{'' if count == 1 else 's'} to stay inside")
            if len(job.inside_areas) >= 2:
                parts.append(f"the drone has to be {union_words(job.union)}")
            text = (f"{'Polygon fence' if job.corners else 'Fence'} on: {', '.join(parts)}{nofly}"
                    + (", a return point" if job.return_point is not None else "")
                    + f", max height {job.alt_max_m:.0f} m, on breach {action_words(job.action)}")
            kinds = TYPE_ALT_MAX | TYPE_POLYGON
        elif job.kind == "nofly":
            count = len(job.nofly)
            text = (f"Areas to stay out of on: {count} area{'' if count == 1 else 's'}, max height "
                    f"{job.alt_max_m:.0f} m, on breach {action_words(job.action)}. No circle and no polygon fence")
            kinds = TYPE_ALT_MAX | TYPE_POLYGON
        else:
            text = (f"Circle fence on: {job.radius_m:.0f} m around home{nofly}, max height {job.alt_max_m:.0f} m, "
                    f"on breach {action_words(job.action)}")
            kinds = TYPE_ALT_MAX | TYPE_CIRCLE | TYPE_POLYGON
        if job.notes:
            text += ". " + ". ".join(job.notes)
        state = {
            "source": "upload", "enabled": True, "kinds": kinds,
            "alt_max_m": job.alt_max_m, "action": job.action,
            "polygon": [[lat, lon] for lat, lon in job.corners],
            "nofly": [area.row() for area in job.nofly], "other": [], "problem": "",
            "keepin": [area.row() for area in job.keepin],
            "return_point": None if job.return_point is None else [job.return_point[0], job.return_point[1]],
            # How the drone takes its areas to stay inside, when that matters (two or more).
            "union": job.union if len(job.inside_areas) >= 2 else None,
        }
        if job.kind == "circle_items":
            state["radius_m"] = job.radius_m
        self.fence_state.emit(state)
        self.geofence_result.emit(True, text)
        self.log_line.emit(f"Geofence confirmed by the drone: {text}")

    def _fence_settings_read(self, job: FenceJob, values: dict, missing: list) -> None:
        """The last step of "Read fence from drone"."""
        if getattr(self, "_fence_job", None) is job:
            self._fence_job = None
        on_drone = job.on_drone
        enabled = None if "FENCE_ENABLE" not in values else bool(round(float(values["FENCE_ENABLE"])))
        kinds = None if "FENCE_TYPE" not in values else int(round(float(values["FENCE_TYPE"])))
        inside_count = (1 if on_drone.polygon else 0) + len(on_drone.keepin)
        if inside_count < 2:
            union = None           # one area or none: all at once and any of them are the same
        elif "FENCE_OPTIONS" in values:
            union = bool(int(round(float(values["FENCE_OPTIONS"]))) & OPTION_UNION)
        else:
            union = None           # not answered (ArduCopter before 4.6 has no such setting): see describe_fence
        state = {
            "source": "download", "enabled": enabled, "kinds": kinds,
            "alt_max_m": values.get("FENCE_ALT_MAX"), "action": values.get("FENCE_ACTION"),
            "radius_m": values.get("FENCE_RADIUS"),
            "polygon": [[lat, lon] for lat, lon in on_drone.polygon],
            "nofly": [area.row() for area in on_drone.nofly],
            "keepin": [area.row() for area in on_drone.keepin],
            "return_point": (None if on_drone.return_point is None
                             else [on_drone.return_point[0], on_drone.return_point[1]]),
            "union": union,
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
        if job.send_at is not None and now >= job.send_at:
            job.send_at = None
            self._fence_send(job)
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
        self.geofence_result.emit(False, f"The fence request was not finished: {reason}."
                                  + (f" {_UNION_LEFT}" if job.union_set_before else ""))


def _nofly_problem(nofly, lat: float, lon: float, what: str) -> str:
    """Why a position is in, or too near to, an area to stay out of. "" when it is 5 m or more outside each."""
    for number, area in enumerate(nofly, start=1):
        outside = nofly_outside_m(area, lat, lon)
        if outside < 0.0:
            return f"{what} is inside no-fly area {number} ({area.words()})."
        if outside < EDGE_MIN_M:
            return (f"{what} is only {outside:.0f} m from no-fly area {number}. Its edge has to be "
                    f"{EDGE_MIN_M:.0f} m away or more.")
    return ""


def _job_words(job: FenceJob) -> str:
    """What an items fence holds, in a few words: "polygon", "areas to stay inside", "areas to stay out of"."""
    if job.kind != "polygon":
        return "areas to stay out of"
    return "polygon" if job.corners else "areas to stay inside"


def _nofly_count_words(count: int) -> str:
    if count <= 0:
        return ""
    return f", {count} area{'' if count == 1 else 's'} to stay out of"


def describe_fence(state: dict) -> str:
    """What the drone's fence is, in a sentence or two for the operator."""
    enabled = state.get("enabled")
    kinds = state.get("kinds")
    polygon = state.get("polygon") or []
    nofly = nofly_from_rows(state.get("nofly"))
    keepin = keepin_from_rows(state.get("keepin"))
    return_point = state.get("return_point")
    in_use = bool(enabled and int(kinds or 0) & TYPE_POLYGON)
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
            if polygon:
                limits.append(f"polygon of {len(polygon)} corners")
            elif not nofly and not keepin:
                limits.append("polygon (but the drone holds none)")
            if keepin:
                limits.append(f"{len(keepin)} {'more ' if polygon else ''}area{'' if len(keepin) == 1 else 's'} "
                              "to stay inside")
            if (1 if polygon else 0) + len(keepin) >= 2:
                union = state.get("union")
                limits.append("it did not give FENCE_OPTIONS (ArduCopter before 4.6 has none, and has to be inside "
                              "all of them at once)" if union is None else f"it has to be {union_words(bool(union))}")
            if nofly:
                limits.append(f"{len(nofly)} area{'' if len(nofly) == 1 else 's'} to stay out of")
            if return_point:
                limits.append("a return point")
        if not limits:
            limits.append(f"no limit chosen ({kinds_words(kinds)})")
        parts.append("The drone's fence is switched on: " + ", ".join(limits)
                     + f", on breach {action_words(state.get('action'))}")
    if polygon and not in_use:
        parts.append(f"It holds a polygon of {len(polygon)} corners that is not in use")
    if keepin and not in_use:
        parts.append(f"It holds {len(keepin)} {'more ' if polygon else ''}area{'' if len(keepin) == 1 else 's'} to "
                     f"stay inside that {'is' if len(keepin) == 1 else 'are'} not in use")
    if nofly and not in_use:
        parts.append(f"It holds {len(nofly)} area{'' if len(nofly) == 1 else 's'} to stay out of that "
                     f"{'is' if len(nofly) == 1 else 'are'} not in use")
    if return_point and not in_use:
        parts.append("It holds a return point that is not in use")
    if state.get("problem"):
        parts.append(str(state["problem"]).rstrip("."))
    if state.get("other"):
        parts.append("It also holds: " + "; ".join(state["other"]) + " (VGCS does not show these)")
    return ". ".join(parts) + "."
