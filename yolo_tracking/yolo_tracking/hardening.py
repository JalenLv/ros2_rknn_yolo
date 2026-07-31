"""Explicit safety divergences layered over the vendored tracker classes."""

import numpy as np

from .trackers import BYTETracker


class FrameGuardMixin:
    """Emit only tracks that were activated by a detection in the current frame."""

    def _format_output(self) -> np.ndarray:
        return np.asarray(
            [
                track.result
                for track in self.tracked_stracks
                if track.is_activated and track.frame_id == self.frame_id
            ],
            dtype=np.float32,
        )


class ByteTrack(FrameGuardMixin, BYTETracker):
    """ByteTrack with current-frame output guarding for safe message re-attachment."""
