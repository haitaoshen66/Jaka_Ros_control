from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jaka_teleop',
            executable='oculus_reader_node',
            name='oculus_reader_node',
            output='screen'
        ),
        Node(
            package='jaka_teleop',
            executable='button_node',
            name='button_node',
            output='screen'
        ),
        Node(
            package='jaka_teleop',
            executable='servo_control_node',
            name='servo_control_node',
            output='screen'
        ),
        Node(
            package='jaka_teleop',
            executable='base_node',
            name='base_node',
            output='screen'
        ),
        # Node(
        #     package='jaka_teleop',
        #     executable='gripper_node',
        #     name='gripper_node',
        #     output='screen'
        # ),
        Node(
            package='jaka_teleop',
            executable='lift_node',
            name='lift_node',
            output='screen'
        ),
        Node(
            package='jaka_teleop',
            executable='workspace_zone_node',
            name='workspace_zone_node',
            output='screen'
        ),
        Node(
            package='jaka_teleop',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[
                {'config_file': '/home/sht/DIJA/jaka_ros2/src/jaka_teleop/config/camera_config.yaml'},
                {'publish_rate': 5.0},
                {'image_quality': 80}
            ]
        ),
        Node(
            package='jaka_teleop',
            executable='lerobot_control_node',
            name='lerobot_control_node',
            output='screen'
        ),
        Node(
            package='jaka_teleop',
            executable='lerobot_recorder_node',
            name='lerobot_recorder_node',
            output='screen',
            parameters=[
                {'output_dir': '/home/sht/DIJA/lerobot_datasets'},
                {'episode_name': 'jaka_teleop'},
                {'fps': 5.0}
            ]
        ),
        # Node(
        #     package='jaka_teleop',
        #     executable='enable_node', 
        #     name='enable_node',
        #     output='screen'
        # )
    ])
