import numpy as np

from yolo_tracking.boxes_shim import BoxesShim
from yolo_tracking.factory import make_tracker


class FakeParameter:
    def __init__(self, value):
        self.value = value


class FakeNode:
    def declare_parameter(self, name, default):
        return FakeParameter("bytetrack" if name == "tracker_type" else default)


def _boxes(rectangles):
    xyxy = np.asarray(rectangles, dtype=np.float32).reshape(-1, 4)
    if len(xyxy):
        widths = xyxy[:, 2] - xyxy[:, 0]
        heights = xyxy[:, 3] - xyxy[:, 1]
        xywh = np.column_stack(
            (
                xyxy[:, 0] + widths / 2,
                xyxy[:, 1] + heights / 2,
                widths,
                heights,
            )
        )
    else:
        xywh = np.empty((0, 4), dtype=np.float32)
    return BoxesShim(
        conf=np.full(len(xyxy), 0.9, dtype=np.float32),
        cls=np.zeros(len(xyxy), dtype=np.float32),
        xywh=xywh,
        xyxy=xyxy,
    )


def test_two_object_crossing_keeps_ids_and_current_frame_subset():
    tracker = make_tracker(FakeNode())

    assert len(tracker.update(_boxes([]))) == 0
    assert len(tracker.update(_boxes([[10, 10, 26, 30], [70, 40, 86, 60]]))) == 0

    stable_ids = None
    for step in range(1, 15):
        left_to_right = 10 + 4 * step
        right_to_left = 70 - 4 * step
        rows = tracker.update(
            _boxes(
                [
                    [left_to_right, 10, left_to_right + 16, 30],
                    [right_to_left, 40, right_to_left + 16, 60],
                ]
            )
        )
        assert len(rows) <= 2
        assert set(rows[:, -1].astype(int)) <= {0, 1}
        ids_by_input = {int(row[-1]): int(row[4]) for row in rows}
        assert set(ids_by_input) == {0, 1}
        if stable_ids is None:
            stable_ids = ids_by_input
        else:
            assert ids_by_input == stable_ids

    assert len(tracker.update(_boxes([]))) == 0
