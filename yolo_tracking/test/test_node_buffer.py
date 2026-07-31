from types import SimpleNamespace

import pytest

from yolo_tracking.node import ImageBuffer


def _image(sec, nanosec=0):
    stamp = SimpleNamespace(sec=sec, nanosec=nanosec)
    return SimpleNamespace(header=SimpleNamespace(stamp=stamp))


def test_exact_hit_prunes_only_images_older_than_match():
    buffer = ImageBuffer(4)
    images = [_image(1), _image(2), _image(3)]
    for image in images:
        buffer.append(image)

    assert buffer.find(images[1].header.stamp) is images[1]
    assert len(buffer) == 2
    assert buffer.find(images[1].header.stamp) is images[1]


def test_miss_does_not_prune_and_capacity_evicts_oldest():
    buffer = ImageBuffer(2)
    first, second, third = _image(1), _image(2), _image(3)
    buffer.append(first)
    buffer.append(second)
    assert buffer.find(third.header.stamp) is None
    assert len(buffer) == 2
    buffer.append(third)
    assert buffer.find(first.header.stamp) is None
    assert buffer.find(third.header.stamp) is third


def test_invalid_capacity_is_rejected():
    with pytest.raises(ValueError):
        ImageBuffer(0)
