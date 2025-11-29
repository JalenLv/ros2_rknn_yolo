from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rknn_yolo',
            executable='yolo_node',
            remappings=[
                ('/camera/color/image_raw', '/image_raw')
            ]
        )
    ])
