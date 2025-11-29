from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "image_topic",
            default_value="/image_raw",
            description="Topic to subscribe for input images"
        ),
        DeclareLaunchArgument(
            "bbox_kpoints_topic",
            default_value="bounding_boxes_keypoints",
            description="Topic to publish bounding boxes with keypoints"
        ),
        DeclareLaunchArgument(
            "fps",
            default_value="15",
            description="Frame rate for the YOLO node",
        ),
        Node(
            package="rknn_yolo",
            executable="yolo_node",
            parameters=[
                {"image_topic": LaunchConfiguration("image_topic")},
                {"bbox_kpoints_topic": LaunchConfiguration("bbox_kpoints_topic")},
                {"fps": LaunchConfiguration("fps")},
            ],
        ),
    ])
