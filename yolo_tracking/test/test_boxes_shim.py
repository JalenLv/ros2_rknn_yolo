import numpy as np
import pytest

from yolo_tracking.boxes_shim import BoxesShim


def _boxes():
    return BoxesShim(
        conf=np.array([0.9, 0.2, 0.8]),
        cls=np.array([1, 2, 3]),
        xywh=np.array([[5, 5, 4, 4], [15, 5, 4, 4], [25, 5, 4, 4]]),
        xyxy=np.array([[3, 3, 7, 7], [13, 3, 17, 7], [23, 3, 27, 7]]),
    )


def test_boolean_mask_returns_same_duck_type():
    boxes = _boxes()
    selected = boxes[np.array([True, False, True])]
    assert isinstance(selected, BoxesShim)
    assert len(selected) == 2
    np.testing.assert_allclose(selected.conf, [0.9, 0.8])
    np.testing.assert_array_equal(selected.cls, [1, 3])
    np.testing.assert_array_equal(selected.xyxy, [[3, 3, 7, 7], [23, 3, 27, 7]])
    assert not hasattr(selected, "xywhr")


def test_non_boolean_or_wrong_shape_index_is_rejected():
    boxes = _boxes()
    with pytest.raises(TypeError):
        boxes[np.array([0, 2])]
    with pytest.raises(TypeError):
        boxes[np.array([True, False])]
