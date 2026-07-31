"""ROS 2 node that assigns tracking IDs to YOLO detections."""

from collections import deque

import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from yolo_msgs.msg import Detections

from .adapter import detections_to_boxes, tracks_to_detections
from .factory import make_tracker


class ImageBuffer:
    """A bounded, timestamp-keyed buffer of source image messages."""

    def __init__(self, capacity: int) -> None:
        if capacity <= 0:
            raise ValueError("image buffer capacity must be positive")
        self._images = deque(maxlen=capacity)

    def append(self, image) -> None:
        """Append an image, evicting the oldest image when the buffer is full."""
        self._images.append(image)

    def find(self, stamp):
        """Return an exact stamp match and prune images older than that match."""
        target = _stamp_key(stamp)
        match = next(
            (image for image in self._images if _stamp_key(image.header.stamp) == target),
            None,
        )
        if match is None:
            return None
        while self._images and _stamp_key(self._images[0].header.stamp) < target:
            self._images.popleft()
        return match

    def __len__(self) -> int:
        return len(self._images)


class TrackingNode(Node):
    """Consume detections in order and publish the current-frame tracked subset."""

    def __init__(self, node_name: str = "tracking_node") -> None:
        super().__init__(node_name)
        self._tracker = make_tracker(self)

        image_policy = self.declare_parameter("image_policy", "auto").value
        if image_policy not in {"auto", "on", "off"}:
            raise RuntimeError(f"image_policy must be one of auto, on, off; got {image_policy!r}")
        image_buffer_size = self.declare_parameter("image_buffer_size", 16).value
        if isinstance(image_buffer_size, bool) or not isinstance(image_buffer_size, int) or image_buffer_size <= 0:
            raise RuntimeError(f"image_buffer_size must be a positive integer, got {image_buffer_size!r}")

        self._image_enabled = image_policy == "on" or (
            image_policy == "auto" and self._tracker.needs_image
        )
        self._image_buffer = ImageBuffer(image_buffer_size)
        self._bridge = CvBridge() if self._image_enabled else None

        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._tracked_publisher = self.create_publisher(Detections, "tracked", reliable_qos)
        self._detections_subscription = self.create_subscription(
            Detections,
            "detections",
            self._detections_callback,
            reliable_qos,
        )
        self._image_subscription = None
        if self._image_enabled:
            self._image_subscription = self.create_subscription(
                Image,
                "image_raw",
                self._image_callback,
                qos_profile_sensor_data,
            )

    def _image_callback(self, message: Image) -> None:
        self._image_buffer.append(message)

    def _detections_callback(self, message: Detections) -> None:
        try:
            image = self._matched_image(message) if self._image_enabled else None
            boxes = detections_to_boxes(message)
            rows = self._tracker.update(boxes, image)
            output = tracks_to_detections(message, rows)
            output.header.stamp = self.get_clock().now().to_msg()
            output.header.frame_id = message.header.frame_id
            self._tracked_publisher.publish(output)
        except Exception as error:
            self.get_logger().error(
                f"tracking frame failed; dropping detections frame: {error}",
                throttle_duration_sec=2.0,
            )

    def _matched_image(self, message: Detections):
        image_message = self._image_buffer.find(message.image_meta.header.stamp)
        if image_message is None:
            stamp = message.image_meta.header.stamp
            self.get_logger().warning(
                f"no image matched detections stamp {stamp.sec}.{stamp.nanosec:09d}; continuing without image",
                throttle_duration_sec=2.0,
            )
            return None
        try:
            return self._bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")
        except Exception as error:
            self.get_logger().warning(
                f"failed to convert matched image; continuing without image: {error}",
                throttle_duration_sec=2.0,
            )
            return None


def _stamp_key(stamp) -> tuple[int, int]:
    return int(stamp.sec), int(stamp.nanosec)


def main(args=None) -> None:
    """Run the tracking node and convert initialization failures to exit status 1."""
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = TrackingNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        get_logger("tracking_node").fatal(str(error))
        exit_code = 1
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass
    if exit_code:
        raise SystemExit(exit_code)
