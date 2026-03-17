from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("omnirob_iisy_vgc10").to_dict()

    koi_controller_node = Node(
        package="koi_controller",
        executable="koi_controller",
        output="screen",
        parameters=[
            moveit_config,
            {"use_sim_time": True}
        ],
        arguments=["--log-level", "error"]
    )
    return LaunchDescription([koi_controller_node])