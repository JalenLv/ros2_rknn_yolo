import numpy as np
import pytest
from yolo_msgs.msg import Detection, Detections, Keypoint

from yolo_tracking.adapter import detections_to_boxes, tracks_to_detections


def _message(width=20, height=20):
    message = Detections()
    message.header.frame_id = "camera"
    message.image_meta.header.frame_id = "camera"
    message.image_meta.header.stamp.sec = 12
    message.image_meta.header.stamp.nanosec = 34
    message.image_meta.width = width
    message.image_meta.height = height

    detection = Detection()
    detection.class_id = 4
    detection.class_name = "person"
    detection.confidence = 0.6
    detection.id = Detection.UNCHECKED
    detection.bbox.xmin = 2
    detection.bbox.ymin = 3
    detection.bbox.xmax = 10
    detection.bbox.ymax = 15
    keypoint = Keypoint()
    keypoint.id = Keypoint.NOSE
    keypoint.x = 4.5
    keypoint.y = 5.5
    keypoint.confidence = 0.9
    detection.keypoints = [keypoint]
    detection.embedding = [0.25, 0.5]
    message.detections = [detection]
    return message


def test_message_constant_is_uint32_sentinel():
    assert Detection.UNCHECKED == 4294967295


def test_input_reconstructs_inverted_and_degenerate_boxes():
    message = _message()
    detection = message.detections[0]
    detection.bbox.xmin = 10
    detection.bbox.xmax = 4
    detection.bbox.ymin = 7
    detection.bbox.ymax = 7

    boxes = detections_to_boxes(message)
    np.testing.assert_array_equal(boxes.xywh, [[10.5, 7.5, 1.0, 1.0]])
    np.testing.assert_array_equal(boxes.xyxy, [[10.0, 7.0, 11.0, 8.0]])
    np.testing.assert_allclose(boxes.conf, [0.6])
    np.testing.assert_array_equal(boxes.cls, [4.0])


def test_output_reattaches_raw_fields_clamps_slivers_and_clears_embedding():
    message = _message()
    rows = np.array([[18.6, -4.0, -10.0, 40.0, 17, 0.875, 999, 0]], dtype=np.float32)

    output = tracks_to_detections(message, rows)

    assert output.image_meta == message.image_meta
    assert len(output.detections) == 1
    tracked = output.detections[0]
    assert tracked.id == 17
    assert tracked.confidence == pytest.approx(0.875)
    assert tracked.class_id == 4
    assert tracked.class_name == "person"
    assert len(tracked.keypoints) == 1
    assert tracked.keypoints[0] == message.detections[0].keypoints[0]
    assert len(tracked.embedding) == 0
    assert (tracked.bbox.xmin, tracked.bbox.ymin, tracked.bbox.xmax, tracked.bbox.ymax) == (19, 0, 19, 19)
    assert message.detections[0].id == Detection.UNCHECKED
    assert list(message.detections[0].embedding) == pytest.approx([0.25, 0.5])


@pytest.mark.parametrize("idx", [-1.0, 1.0, 0.25, np.nan])
def test_invalid_tracker_index_drops_the_conversion(idx):
    message = _message()
    rows = np.array([[0, 0, 1, 1, 1, 0.5, 4, idx]], dtype=np.float32)
    with pytest.raises((IndexError, ValueError)):
        tracks_to_detections(message, rows)


def test_zero_image_dimensions_are_rejected_even_for_empty_output():
    message = _message(width=0, height=20)
    with pytest.raises(ValueError, match="image dimensions"):
        tracks_to_detections(message, np.empty((0, 8), dtype=np.float32))
