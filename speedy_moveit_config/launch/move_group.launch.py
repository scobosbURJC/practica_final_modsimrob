from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from ament_index_python.packages import get_package_share_directory
from os.path import join

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("speedy", package_name="speedy_moveit_config").to_moveit_configs()

    return generate_move_group_launch(moveit_config)
