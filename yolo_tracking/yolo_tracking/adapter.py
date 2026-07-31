"""Conversions between yolo_msgs detections and tracker NumPy arrays."""

from copy import deepcopy
import math

import numpy as np

from .boxes_shim import BoxesShim


def detections_to_boxes(message) -> BoxesShim:
    """Build a consistent BoxesShim from a yolo_msgs/Detections message."""
    count = len(message.detections)
    conf = np.empty(count, dtype=np.float32)
    classes = np.empty(count, dtype=np.float32)
    xywh = np.empty((count, 4), dtype=np.float32)
    xyxy = np.empty((count, 4), dtype=np.float32)

    for index, detection in enumerate(message.detections):
        xmin = float(detection.bbox.xmin)
        ymin = float(detection.bbox.ymin)
        width = max(float(detection.bbox.xmax) - xmin, 1.0)
        height = max(float(detection.bbox.ymax) - ymin, 1.0)
        conf[index] = float(detection.confidence)
        classes[index] = float(detection.class_id)
        xywh[index] = (xmin + width / 2.0, ymin + height / 2.0, width, height)
        xyxy[index] = (xmin, ymin, xmin + width, ymin + height)

    return BoxesShim(conf=conf, cls=classes, xywh=xywh, xyxy=xyxy)


def tracks_to_detections(message, tracks: np.ndarray):
    """Re-attach tracker rows to source detections and clamp their output boxes."""
    width = int(message.image_meta.width)
    height = int(message.image_meta.height)
    if width <= 0 or height <= 0:
        raise ValueError(f"image dimensions must be positive, got {width}x{height}")

    output = deepcopy(message)
    output.detections = []
    rows = np.asarray(tracks)
    if rows.size == 0:
        return output
    if rows.ndim != 2 or rows.shape[1] != 8:
        raise ValueError(f"axis-aligned tracker output must have shape (N, 8), got {rows.shape}")

    for row in rows:
        raw_index = float(row[7])
        if not math.isfinite(raw_index) or not raw_index.is_integer():
            raise ValueError(f"tracker output idx must be integral, got {raw_index}")
        index = int(raw_index)
        if index < 0 or index >= len(message.detections):
            raise IndexError(
                f"tracker output idx {index} is outside detection range [0, {len(message.detections)})"
            )

        detection = deepcopy(message.detections[index])
        detection.id = int(row[4])
        detection.confidence = float(row[5])
        xmin = _round_and_clamp(row[0], width - 1)
        ymin = _round_and_clamp(row[1], height - 1)
        xmax = max(_round_and_clamp(row[2], width - 1), xmin)
        ymax = max(_round_and_clamp(row[3], height - 1), ymin)
        detection.bbox.xmin = xmin
        detection.bbox.ymin = ymin
        detection.bbox.xmax = xmax
        detection.bbox.ymax = ymax
        detection.embedding = []
        output.detections.append(detection)

    return output


def _round_and_clamp(value: float, upper: int) -> int:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"tracker coordinate must be finite, got {value}")
    return min(max(int(round(value)), 0), upper)
