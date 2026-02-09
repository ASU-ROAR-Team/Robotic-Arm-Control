from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("New_URDF", package_name="ROAR_MoveIT").to_moveit_configs()
    return generate_spawn_controllers_launch(moveit_config)
