# Ultralytics 🚀 AGPL-3.0 License - https://ultralytics.com/license
"""Local compatibility helpers for the vendored tracker subtree."""

import logging

import numpy as np


LOGGER = logging.getLogger("yolo_tracking")


def xywh2ltwh(x):
    """Convert bounding box format from [x, y, w, h] to [x1, y1, w, h] where x1, y1 are top-left coordinates.

    Args:
        x (np.ndarray): Input bounding box coordinates in xywh format.

    Returns:
        (np.ndarray): Bounding box coordinates in ltwh format.
    """
    y = np.copy(x)
    y[..., 0] = x[..., 0] - x[..., 2] / 2  # top left x
    y[..., 1] = x[..., 1] - x[..., 3] / 2  # top left y
    return y


def xyxy2ltwh(x):
    """Convert bounding boxes from [x1, y1, x2, y2] to [x1, y1, w, h] format.

    Args:
        x (np.ndarray): Input bounding box coordinates in xyxy format.

    Returns:
        (np.ndarray): Bounding box coordinates in ltwh format.
    """
    y = np.copy(x)
    y[..., 2] = x[..., 2] - x[..., 0]  # width
    y[..., 3] = x[..., 3] - x[..., 1]  # height
    return y


def bbox_ioa(box1: np.ndarray, box2: np.ndarray, iou: bool = False, eps: float = 1e-7) -> np.ndarray:
    """Calculate the intersection over box2 area given box1 and box2.

    Args:
        box1 (np.ndarray): A numpy array of shape (N, 4) representing N bounding boxes in x1y1x2y2 format.
        box2 (np.ndarray): A numpy array of shape (M, 4) representing M bounding boxes in x1y1x2y2 format.
        iou (bool, optional): Calculate the standard IoU if True else return inter_area/box2_area.
        eps (float, optional): A small value to avoid division by zero.

    Returns:
        (np.ndarray): A numpy array of shape (N, M) representing the intersection over box2 area.
    """
    # Get the coordinates of bounding boxes
    b1_x1, b1_y1, b1_x2, b1_y2 = box1.T
    b2_x1, b2_y1, b2_x2, b2_y2 = box2.T

    # Intersection area
    inter_area = (np.minimum(b1_x2[:, None], b2_x2) - np.maximum(b1_x1[:, None], b2_x1)).clip(0) * (
        np.minimum(b1_y2[:, None], b2_y2) - np.maximum(b1_y1[:, None], b2_y1)
    ).clip(0)

    # Box2 area
    area = (b2_x2 - b2_x1) * (b2_y2 - b2_y1)
    if iou:
        box1_area = (b1_x2 - b1_x1) * (b1_y2 - b1_y1)
        area = area + box1_area[:, None] - inter_area

    # Intersection over box2 area
    return inter_area / (area + eps)


def lapjv_cost_limit(cost: np.ndarray, thresh: float):
    """Reproduce lap.lapjv extended cost-limit assignment with SciPy."""
    from scipy.optimize import linear_sum_assignment

    cost = np.asarray(cost, dtype=np.float64)
    if cost.ndim != 2:
        raise ValueError(f"cost matrix must be two-dimensional, got shape {cost.shape}")
    n, m = cost.shape
    if n == 0 or m == 0:
        return np.empty((0, 2), dtype=int), np.arange(n), np.arange(m)

    extended = np.full((n + m, n + m), float(thresh) / 2.0, dtype=np.float64)
    extended[n:, m:] = 0.0
    extended[:n, :m] = cost
    rows, cols = linear_sum_assignment(extended)

    matches = []
    unmatched_a = []
    matched_cols = set()
    for row, col in zip(rows, cols):
        if row < n:
            if col < m:
                matches.append([row, col])
                matched_cols.add(col)
            else:
                unmatched_a.append(row)
    unmatched_b = [col for col in range(m) if col not in matched_cols]
    return (
        np.asarray(matches, dtype=int).reshape(-1, 2),
        np.asarray(unmatched_a, dtype=int),
        np.asarray(unmatched_b, dtype=int),
    )
