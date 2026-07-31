from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


LAUNCH_ARGUMENTS = [
    ("tracker", "bytetrack", "Tracker configuration name from yolo_tracking/config"),
    ("image_policy", "auto", "Image subscription policy: auto|on|off"),
]


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in LAUNCH_ARGUMENTS
    ]
    launch_configs = {
        name: LaunchConfiguration(name)
        for name, _, _ in LAUNCH_ARGUMENTS
    }
    tracker_config = PathJoinSubstitution([
        FindPackageShare("yolo_tracking"),
        "config",
        [
            launch_configs["tracker"],
            TextSubstitution(text=".yaml"),
        ],
    ])

    return LaunchDescription(
        launch_args
        + [
            Node(
                package="rknn_yolo",
                executable="tracking_node",
                parameters=[
                    tracker_config,
                    {
                        "image_policy": ParameterValue(
                            launch_configs["image_policy"],
                            value_type=str,
                        ),
                    },
                ],
            ),
        ]
    )
