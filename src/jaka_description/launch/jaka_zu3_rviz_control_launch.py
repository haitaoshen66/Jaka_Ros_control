import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, FindPackageShare

def generate_launch_description():
    # Declare launch arguments if needed (e.g., file paths)
    return LaunchDescription([
        # Declare the robot_description parameter using a URDF file
        DeclareLaunchArgument(
            'robot_description',
            default_value=FindPackageShare('jaka_description') + '/urdf/jaka_zu3.urdf',
            description='Path to the URDF file for the robot'
        ),

        # Start the robot_state_publisher node 
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            remappings=[('/joint_states', 'robot_jog_command')],
        ),
        
        # Start the jaka_jog_panel node 
        Node(
            package='jaka_jog_panel',
            executable='jakajogpanel',
            name='jaka_jog_panel',
        ),

        # Start rviz with the provided configuration file
        Node(
            package='rviz2',  # ROS 2 uses rviz2 instead of rviz
            executable='rviz2',
            name='rviz',
            arguments=['-d', FindPackageShare('jaka_description') + '/config/jaka_zu3_urdf.rviz'],
            required=True
        ),
    ])
