from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_gazebo_launch 


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("jaka_zu5", package_name="jaka_zu5_moveit_config").to_moveit_configs()
    return generate_demo_gazebo_launch(moveit_config)