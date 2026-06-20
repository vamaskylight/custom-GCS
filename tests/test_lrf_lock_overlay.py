from vgcs.skydroid.adapter import SkydroidTopUdpAdapter


def test_lrf_lock_video_norm_from_pixels():
    adapter = SkydroidTopUdpAdapter.__new__(SkydroidTopUdpAdapter)
    adapter._status_lock = __import__("threading").Lock()
    adapter._lrf_locked = True
    adapter._lrf_lock_external = False
    adapter._lrf_lock_x = 640
    adapter._lrf_lock_y = 360

    norm = adapter.get_lrf_lock_video_norm()
    assert norm is not None
    assert abs(norm[0] - 0.5) < 0.001
    assert abs(norm[1] - 0.5) < 0.001

    adapter._lrf_locked = False
    assert adapter.get_lrf_lock_video_norm() is None
