import pytest

from yolo_tracking.factory import make_tracker
from yolo_tracking.hardening import ByteTrack


class FakeParameter:
    def __init__(self, value):
        self.value = value


class FakeNode:
    def __init__(self, **overrides):
        self.overrides = overrides
        self.declared = {}

    def declare_parameter(self, name, default):
        value = self.overrides.get(name, default)
        self.declared[name] = value
        return FakeParameter(value)


def test_empty_tracker_type_has_remediation_hint():
    with pytest.raises(RuntimeError, match="tracker_type is empty.*bytetrack"):
        make_tracker(FakeNode())


def test_unknown_tracker_lists_all_planned_backends():
    with pytest.raises(RuntimeError) as error:
        make_tracker(FakeNode(tracker_type="mystery"))
    for name in ("bytetrack", "botsort", "ocsort", "fasttrack"):
        assert name in str(error.value)


def test_bytetrack_constructs_from_declared_defaults():
    tracker = make_tracker(FakeNode(tracker_type="bytetrack"))
    assert isinstance(tracker, ByteTrack)
    assert tracker.args.track_high_thresh == pytest.approx(0.25)
    assert tracker.args.track_buffer == 30
    assert tracker.needs_image is False


def test_invalid_parameter_fails_before_frames_are_processed():
    with pytest.raises(RuntimeError, match="track_low_thresh"):
        make_tracker(
            FakeNode(
                tracker_type="bytetrack",
                track_low_thresh=0.7,
                track_high_thresh=0.2,
            )
        )
