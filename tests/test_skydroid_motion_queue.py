"""Skydroid C13 motion command queue (PTZ hold / stop coalescing)."""

import queue

from vgcs.skydroid.adapter import SkydroidTopUdpAdapter


def test_motion_stop_command_detection():
    assert SkydroidTopUdpAdapter._is_motion_stop_command(["PT_STOP"], {})
    assert SkydroidTopUdpAdapter._is_motion_stop_command(["PTZ_STOP"], {})
    assert not SkydroidTopUdpAdapter._is_motion_stop_command(["PT_RIGHT"], {})
    assert SkydroidTopUdpAdapter._is_motion_stop_command(
        ["GSM"], {"yaw": 0.0, "pitch": 0.0}
    )
    assert not SkydroidTopUdpAdapter._is_motion_stop_command(
        ["GSM"], {"yaw": 5.0, "pitch": 0.0}
    )


def test_drop_pending_motion_preserves_stop():
    adapter = SkydroidTopUdpAdapter(host="127.0.0.1")
    adapter._running = True
    adapter._queue.put_nowait((["PT_STOP"], {}, False))
    adapter._queue.put_nowait((["PT_RIGHT"], {}, False))
    adapter._drop_pending_motion_commands()
    adapter._queue.put_nowait((["PT_LEFT"], {}, False))
    items: list[tuple[list[str], dict, bool]] = []
    while True:
        try:
            items.append(adapter._queue.get_nowait())
        except queue.Empty:
            break
    tags = [str(cmds[0]) for cmds, _p, _e in items]
    assert "PT_STOP" in tags
    assert "PT_RIGHT" not in tags
    assert "PT_LEFT" in tags


def test_ptz_stop_burst_enqueues_multiple_stops():
    adapter = SkydroidTopUdpAdapter(host="127.0.0.1")
    adapter._running = True
    adapter.ptz_stop_burst(count=3)
    stops = 0
    while True:
        try:
            cmds, _p, _e = adapter._queue.get_nowait()
        except queue.Empty:
            break
        if cmds and str(cmds[0]).upper() == "PT_STOP":
            stops += 1
    assert stops == 3
