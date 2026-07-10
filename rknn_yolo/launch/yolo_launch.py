from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    model_path = PathJoinSubstitution(
        [FindPackageShare("librknn_yolov8_pose"), "model", "yolov8_pose.rknn"]
    )
    label_path = PathJoinSubstitution(
        [FindPackageShare("librknn_yolov8_pose"), "model", "yolov8_pose_labels_list.txt"]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "fps",
            default_value="0",
            description="Frame rate for the YOLO node",
        ),
        DeclareLaunchArgument(
            "model_path",
            default_value=model_path,
            description="Path to .rknn model",
        ),
        DeclareLaunchArgument(
            "label_path",
            default_value=label_path,
            description="Path to label txt",
        ),
        Node(
            package="rknn_yolo",
            executable="yolo_node",
            parameters=[
                {"fps": LaunchConfiguration("fps")},
                {"model_path": LaunchConfiguration("model_path")},
                {"label_path": LaunchConfiguration("label_path")},
            ],
        ),
    ])
