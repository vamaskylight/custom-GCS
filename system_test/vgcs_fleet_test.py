"""Three drones at once in the VGCS window (milestone M15), against three real ArduCopter simulators.

Usage, from the repo root:

    py system_test/vgcs_fleet_test.py                   ArduCopter 4.7.0
    py system_test/vgcs_fleet_test.py 4.6.2 --report out.md

The acceptance test of client requirement 14 (FR-SWARM):

1. N simulators connected, and switching the active drone does not drop the
   others.
2. The coordination behaviour: a mission goes to the selected drone only, and
   hold, return home and land go to one drone or to all.

Three simulators run side by side (instances 0, 1 and 2, system ids 1, 2 and
3, homes 40 m apart). The real VGCS window connects to each through its own
connection, the first with the Connect button and the others the way the
Fleet panel adds them. One checking connection per simulator, in its own
thread, says what each drone really did.

Like vgcs_window_test.py: the user's settings are never touched (every
QSettings goes to a temporary INI file), and the map draws from its own
offline tile folder, with a check that it asks the internet for nothing.
"""

from __future__ import annotations

import math
import os
import pathlib
import re
import shutil
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import vgcs_window_test as wt  # noqa: E402  (sends QSettings to a temp INI before VGCS loads)

from PySide6 import QtCore  # noqa: E402
from PySide6.QtWidgets import QApplication, QMessageBox  # noqa: E402

from sitl_session import Sitl  # noqa: E402
from pymavlink import mavutil  # noqa: E402

HOME = (20.4347, 72.8696)
SPACING_M = 40.0
DRONES = 3


def home_of(index: int) -> str:
    east = index * SPACING_M
    lon = HOME[1] + east / (111_320.0 * math.cos(math.radians(HOME[0])))
    return f"{HOME[0]},{lon:.7f},30,0"


def watch_mission_count(sim: Sitl) -> None:
    """Let the checking connection remember the MISSION_COUNT answers it gets."""
    sim.mission_count = None
    take = sim._take

    def taking(msg) -> None:
        if msg.get_type() == "MISSION_COUNT":
            sim.mission_count = int(msg.count)
        take(msg)

    sim._take = taking


def ask_mission_count(sim: Sitl) -> None:
    sim.mission_count = None
    sim.mav.mav.mission_request_list_send(sim.mav.target_system, sim.mav.target_component)


def session(version: str, r: wt.Report) -> None:
    app = QApplication.instance() or QApplication(sys.argv[:1])
    import vgcs.app.flight_action_dialogs as dialogs
    from vgcs.app.main_window import MainWindow
    from vgcs.map import native_tile_map
    from vgcs.map.app_settings import QS_APP, QS_ORG
    from vgcs.map.surface.settings_keys import _KEY_MAP_OFFLINE_TILE_ROOT, _KEY_MAP_TILE_MODE
    from vgcs.mission import Waypoint
    from vgcs.link.fleet import LINK_LOST, LINK_UP

    dialogs.ask_takeoff_altitude = lambda _parent, alt: 15.0
    dialogs.confirm_land = lambda _parent: True
    popups: list[str] = []
    QMessageBox.warning = staticmethod(lambda _p, title, text, *a, **k: popups.append(f"{title}: {text}") or QMessageBox.StandardButton.Ok)
    QMessageBox.information = staticmethod(lambda _p, title, text, *a, **k: popups.append(f"{title}: {text}") or QMessageBox.StandardButton.Ok)
    QMessageBox.question = staticmethod(lambda *a, **k: QMessageBox.StandardButton.Yes)

    tiles = wt.SETTINGS_DIR / "tiles"
    wt.make_offline_tiles(tiles)
    store = QtCore.QSettings(QS_ORG, QS_APP)
    store.setValue(_KEY_MAP_TILE_MODE, "offline")
    store.setValue(_KEY_MAP_OFFLINE_TILE_ROOT, str(tiles))
    store.sync()
    internet: list[str] = []
    real_request = native_tile_map._NativeTileLoader.request

    def watched_request(self, z, x, y, url, **kwargs):
        if str(url).lower().startswith(("http://", "https://")):
            internet.append(str(url))
        return real_request(self, z, x, y, url, **kwargs)

    native_tile_map._NativeTileLoader.request = watched_request

    sims: list[Sitl] = []
    observers: list[wt.Observer] = []
    w = None
    try:
        for i in range(DRONES):
            sim = Sitl(version, {}, {"RC_OVERRIDE_TIME": -1, "SIM_WIND_SPD": 0}, speedup=5,
                       home=home_of(i), instance=i, sysid=i + 1)
            watch_mission_count(sim)
            sims.append(sim)
            observers.append(wt.Observer(sim))
        r.check(all(s.mav.target_system == i + 1 for i, s in enumerate(sims)),
                f"three simulators run, system ids {[s.mav.target_system for s in sims]}")

        w = MainWindow()
        w.resize(1600, 900)
        w.show()
        logs: list[str] = []
        real_append = w._append_log
        w._append_log = lambda line: (logs.append(str(line)), real_append(line))[1]
        wt.wait_qt(lambda: False, 3.0)

        # Drone 1 with the Connect button, drones 2 and 3 the way the Fleet panel adds them.
        w._conn_edit.setText("tcp:127.0.0.1:5762")
        w._btn_connect.click()
        for i in (1, 2):
            why = w._fleet_add_vehicle(f"tcp:127.0.0.1:{5762 + 10 * i}")
            r.check(why == "", f"drone {i + 1} added in the background ({why or 'ok'})")
        fleet = w._fleet
        vehicles = fleet.vehicles()
        r.check(len(vehicles) == 3, f"the fleet holds three drones ({[v.name for v in vehicles]})")
        r.check(wt.wait_qt(lambda: all(v.summary.link == LINK_UP for v in vehicles), 40),
                f"all three links are up ({[v.summary.link for v in vehicles]})")
        r.check([v.summary.sysid for v in vehicles] == [1, 2, 3],
                f"each link reads its own drone (system ids {[v.summary.sysid for v in vehicles]})")
        r.check(w._thread is vehicles[0].thread, "drone 1 is on screen")
        w._close_preflight_dialog()

        r.check(wt.wait_qt(lambda: all((s.ekf_flags & 0x18) == 0x18 for s in sims), 120), "the three drones have their position")
        r.check(wt.wait_qt(lambda: all(v.summary.lat is not None for v in vehicles), 20), "VGCS has the position of each")
        lons = [v.summary.lon for v in vehicles]
        gaps = [abs(lons[i + 1] - lons[i]) * 111_320.0 * math.cos(math.radians(HOME[0])) for i in range(2)]
        r.check(all(30.0 < g < 50.0 for g in gaps), f"and they stand 40 m apart, as placed ({gaps[0]:.0f} m, {gaps[1]:.0f} m)")

        w._refresh_fleet_view()
        items = w._fleet_map_items()
        r.check(len(items) == 3 and [i["active"] for i in items] == [True, False, False],
                "the map draws all three, drone 1 as the one on screen")
        r.check("fleet of 3" in w.windowTitle(), f"the title says so ({w.windowTitle()!r})")
        w._show_fleet_dialog()
        rows = [[w._fleet_table.item(row, col).text() for col in range(3)] for row in range(w._fleet_table.rowCount())]
        r.check(len(rows) == 3 and rows[0][0].endswith("(on screen)") and all(row[1].startswith("OK") for row in rows),
                f"the Fleet panel lists them: {rows}")
        w._fleet_dialog.close()

        # 1. Switching does not drop the others.
        switched_at = time.monotonic()
        r.check(w._switch_active_vehicle(vehicles[1].vid), "drone 2 put on screen")
        r.check(w._thread is vehicles[1].thread, "the window's buttons now command drone 2")
        r.check(wt.wait_qt(lambda: "sys 2" in w._hb.text(), 10), f"the header shows drone 2 ({w._hb.text()!r})")
        r.check(wt.wait_qt(lambda: all(v.summary.last_heard_mono > switched_at + 2.0 for v in vehicles), 10),
                "drones 1 and 3 are still heard after the switch")
        r.check(all(v.is_running() for v in vehicles), "no link was dropped")
        r.check(w._preflight_dialog is None, "no pre-flight popup for a switch")

        # 2. The window's buttons command the drone on screen only.
        w._on_takeoff()
        r.check(wt.wait_qt(lambda: sims[1].armed, 40), "Take-off from the window: drone 2 armed")
        r.check(wt.wait_qt(lambda: sims[1].alt > 13.0, 90), f"drone 2 climbed to 15 m ({sims[1].alt:.1f} m)")
        r.check(not sims[0].armed and not sims[2].armed, "drones 1 and 3 stayed on the ground, disarmed")
        r.check(wt.wait_qt(lambda: w._fields["alt_rel"].text().startswith(("14", "15")), 10),
                f"the window shows drone 2's height ({w._fields['alt_rel'].text()!r})")

        # 3. A mission goes to the selected drone only.
        r.check(w._switch_active_vehicle(vehicles[2].vid), "drone 3 put on screen")
        r.check(wt.wait_qt(lambda: "sys 3" in w._hb.text(), 10), f"the header shows drone 3 ({w._hb.text()!r})")
        north = 50.0 / 111_320.0
        lat3 = vehicles[2].summary.lat
        lon3 = vehicles[2].summary.lon
        plan = [Waypoint(lat=lat3 + north, lon=lon3, alt_m=20.0, speed_mps=5.0),
                Waypoint(lat=lat3 + 2 * north, lon=lon3, alt_m=20.0, speed_mps=5.0),
                Waypoint(lat=lat3 + north, lon=lon3, alt_m=20.0, speed_mps=5.0)]
        w._on_mission_upload_requested(plan)
        r.check(wt.wait_qt(lambda: any("Mission upload success" in line for line in logs), 30),
                "Upload from the window: VGCS reports the mission uploaded")
        counts = []
        for sim in sims:
            ask_mission_count(sim)
            wt.wait_qt(lambda s=sim: s.mission_count is not None, 10)
            counts.append(sim.mission_count)
            wt.wait_qt(lambda: False, 1.0)   # let that drone's mission transfer end before asking the next
        # An empty mission counts 0 items on 4.6.2 and 1 (home) on 4.7.0.
        r.check(counts[2] is not None and counts[2] >= 4 and counts[0] in (0, 1) and counts[1] in (0, 1),
                f"only drone 3 holds the mission (items per drone: {counts})")

        # 4. One drone, then all.
        sent = w._fleet_command("rtl", vehicles[1].vid)
        r.check(sent == ["Drone 2"], f"Return home sent to drone 2 alone ({sent})")
        r.check(wt.wait_qt(lambda: sims[1].mode == "RTL", 20), f"drone 2 returns home ({sims[1].mode})")
        r.check(sims[0].mode != "RTL" and sims[2].mode != "RTL", f"the others did not ({sims[0].mode}, {sims[2].mode})")
        sent = w._fleet_command("land")
        r.check(sent == ["Drone 1", "Drone 2", "Drone 3"], f"Land sent to all ({sent})")
        r.check(wt.wait_qt(lambda: all(s.mode == "LAND" for s in sims), 20), f"all three in LAND ({[s.mode for s in sims]})")
        r.check(wt.wait_qt(lambda: not sims[1].armed, 120), "drone 2 landed and disarmed")

        # 5. One drone goes silent: only its link is lost, and the operator hears it.
        sims[0].freeze()
        r.check(wt.wait_qt(lambda: vehicles[0].summary.link == LINK_LOST, 15), "drone 1 frozen: its link is lost")
        r.check(vehicles[1].summary.link == LINK_UP and vehicles[2].summary.link == LINK_UP, "drones 2 and 3 are still up")
        r.check(any(line == "[Fleet] Drone 1: link lost" for line in logs), "the window says so, with the drone's name")
        sims[0].unfreeze()
        r.check(wt.wait_qt(lambda: vehicles[0].summary.link == LINK_UP, 20), "drone 1 back")

        # 6. Disconnect one, the others stay.
        w._fleet_remove_vehicle(vehicles[0].vid)
        r.check(wt.wait_qt(lambda: not vehicles[0].is_running(), 10), "drone 1 disconnected")
        r.check(all(v.is_running() and v.summary.link == LINK_UP for v in vehicles[1:]), "drones 2 and 3 still connected")
        r.check(len(fleet.vehicles()) == 2, "the fleet lists two")

        unexpected = [p for p in popups if not p.startswith("Mission Upload")]
        r.check(not unexpected, f"no unexpected popup {unexpected[:2]}")
        r.check(not internet, f"the map asked the internet for nothing ({len(internet)} requests)")
    finally:
        if w is not None:
            threads = [v.thread for v in w._fleet.vehicles()]
            w.close()
            app.processEvents()
            r.check(all(not t.isRunning() for t in threads), "closing the window stopped every link")
        for obs in observers:
            obs.stop()
        for sim in reversed(sims):
            sim.close()
        app.processEvents()


def main(argv: list[str]) -> int:
    version = next((a for a in argv if re.fullmatch(r"\d+\.\d+\.\d+", a)), "4.7.0")
    r = wt.Report()
    print(f"\n===== VGCS fleet of {DRONES}, ArduCopter {version}")
    began = time.monotonic()
    try:
        session(version, r)
    except Exception as exc:
        import traceback

        traceback.print_exc()
        r.check(False, f"the session stopped: {type(exc).__name__}: {exc}")
    print(f"\n{len(r.lines) - r.failed} of {len(r.lines)} checks passed in {time.monotonic() - began:.0f} s")
    if "--report" in argv:
        target = pathlib.Path(argv[argv.index("--report") + 1])
        lines = ["# VGCS fleet test results", "", f"Made by `system_test/vgcs_fleet_test.py`, three ArduCopter {version} simulators.", ""]
        lines += [f"- {'ok' if ok else '**FAIL**'}: {text}" for ok, text in r.lines]
        target.write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")
        print(f"report written to {target}")
    shutil.rmtree(wt.SETTINGS_DIR, ignore_errors=True)
    return 1 if r.failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
