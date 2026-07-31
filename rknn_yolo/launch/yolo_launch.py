from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


LAUNCH_ARGUMENTS = [
    ("fps", "0", "Frame rate for the YOLO node"),
    ("use_rga", "true", "Use RGA hardware for preprocessing; false = plain CPU (OpenCV)"),
    ("model", "yolov8_pose", "Model configuration name from librknn_yolo/config"),
]


def reject_removed_args(context):
    for name in ("model_path", "label_path"):
        if context.launch_configurations.get(name):
            raise RuntimeError(
                f"launch argument '{name}' was removed; select a model with "
                "model:=<config name from librknn_yolo/config> instead")
    return []


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in LAUNCH_ARGUMENTS
    ]
    launch_configs = {
        name: LaunchConfiguration(name)
        for name, _, _ in LAUNCH_ARGUMENTS
    }

    model_config = PathJoinSubstitution([
        FindPackageShare("librknn_yolo"),
        "config",
        [
            launch_configs["model"],
            TextSubstitution(text=".yaml"),
        ],
    ])

    return LaunchDescription(
        launch_args +
        [
            OpaqueFunction(function=reject_removed_args),
            Node(
                package="rknn_yolo",
                executable="yolo_node",
                parameters=[
                    model_config,
                    {
                        "fps": launch_configs["fps"],
                        "use_rga": launch_configs["use_rga"],
                    },
                ],
            ),
        ]
    )
