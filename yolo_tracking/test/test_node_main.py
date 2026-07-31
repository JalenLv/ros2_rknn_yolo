from rclpy.executors import ExternalShutdownException

import yolo_tracking.node as tracking_node


def test_external_shutdown_exception_exits_cleanly(monkeypatch):
    destroyed = []

    class DummyNode:
        def destroy_node(self):
            destroyed.append(True)

    def raise_external_shutdown(_node):
        raise ExternalShutdownException()

    monkeypatch.setattr(tracking_node.rclpy, "init", lambda args=None: None)
    monkeypatch.setattr(tracking_node.rclpy, "spin", raise_external_shutdown)
    monkeypatch.setattr(tracking_node.rclpy, "ok", lambda: False)
    monkeypatch.setattr(tracking_node, "TrackingNode", DummyNode)

    tracking_node.main()

    assert destroyed == [True]
