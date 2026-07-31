"""ROS parameter declaration and tracker construction."""

import math
from types import SimpleNamespace

from .hardening import ByteTrack


AVAILABLE_TRACKERS = ("bytetrack", "botsort", "ocsort", "fasttrack")


def make_tracker(node):
    """Declare tracker parameters and construct the selected hardened backend."""
    tracker_type = _declare(node, "tracker_type", "")
    values = {
        "track_high_thresh": _declare(node, "track_high_thresh", 0.25),
        "track_low_thresh": _declare(node, "track_low_thresh", 0.1),
        "new_track_thresh": _declare(node, "new_track_thresh", 0.25),
        "track_buffer": _declare(node, "track_buffer", 30),
        "match_thresh": _declare(node, "match_thresh", 0.8),
        "fuse_score": _declare(node, "fuse_score", True),
    }

    if not tracker_type:
        raise RuntimeError(
            "tracker_type is empty; load a yolo_tracking config or set "
            "--ros-args -p tracker_type:=bytetrack"
        )
    if tracker_type != "bytetrack":
        choices = ", ".join(AVAILABLE_TRACKERS)
        raise RuntimeError(f"unknown tracker_type '{tracker_type}'; expected one of: {choices}")

    _validate(values)
    tracker = ByteTrack(SimpleNamespace(tracker_type=tracker_type, **values))
    tracker.needs_image = False
    return tracker


def _declare(node, name, default):
    return node.declare_parameter(name, default).value


def _validate(values) -> None:
    for name in (
        "track_high_thresh",
        "track_low_thresh",
        "new_track_thresh",
        "match_thresh",
    ):
        value = values[name]
        if isinstance(value, bool) or not math.isfinite(float(value)) or not 0.0 <= float(value) <= 1.0:
            raise RuntimeError(f"{name} must be a finite number in [0, 1], got {value!r}")
    if values["track_low_thresh"] > values["track_high_thresh"]:
        raise RuntimeError("track_low_thresh must not exceed track_high_thresh")
    if isinstance(values["track_buffer"], bool) or not isinstance(values["track_buffer"], int):
        raise RuntimeError("track_buffer must be an integer number of frames")
    if values["track_buffer"] < 0:
        raise RuntimeError("track_buffer must be non-negative")
    if not isinstance(values["fuse_score"], bool):
        raise RuntimeError("fuse_score must be boolean")
