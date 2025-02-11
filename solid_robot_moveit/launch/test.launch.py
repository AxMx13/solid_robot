from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python import get_package_share_directory


def generate_launch_description():
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="true"
    )

    is_sim = LaunchConfiguration("is_sim")

    moveit_config = (
        MoveItConfigsBuilder("solid_robot", package_name="solid_robot_moveit")
        .robot_description(file_path=os.path.join(get_package_share_directory("solid_robot"), "urdf", "solid_robot.urdf.xacro"))
        .robot_description_semantic()
    )



    return LaunchDescription([])

