from types import SimpleNamespace

import numpy as np

from yolo_tracking.hardening import FrameGuardMixin


class GuardHarness(FrameGuardMixin):
    def __init__(self):
        self.frame_id = 8
        self.tracked_stracks = [
            SimpleNamespace(is_activated=True, frame_id=7, result=[0, 0, 1, 1, 1, 0.9, 0, 0]),
            SimpleNamespace(is_activated=False, frame_id=8, result=[0, 0, 1, 1, 2, 0.9, 0, 0]),
            SimpleNamespace(is_activated=True, frame_id=8, result=[0, 0, 1, 1, 3, 0.9, 0, 0]),
        ]


def test_frame_guard_suppresses_stale_and_unactivated_tracks():
    rows = GuardHarness()._format_output()
    assert rows.dtype == np.float32
    np.testing.assert_allclose(rows, [[0, 0, 1, 1, 3, 0.9, 0, 0]])
