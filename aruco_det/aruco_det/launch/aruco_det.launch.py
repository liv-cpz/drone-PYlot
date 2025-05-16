from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    camera_config_path = PathJoinSubstitution([
        FindPackageShare("aruco_det"),
        "config",
        "camera.yaml"
    ])

    return LaunchDescription([
        Node(
            package='aruco_det',
            executable='aruco_det',
            name='aruco_det',
            output='screen',
            parameters=[{
                'dictionary_id': 10,    # DICT_6X6_250
                'aruco_id': 19,
                'aruco_length': 0.05,   # 5cm
                'camera_param_path': camera_config_path,
                'sub_image_topic': '/image_raw'
            }]
        )
    ])
