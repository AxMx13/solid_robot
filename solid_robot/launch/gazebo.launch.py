from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import AppendEnvironmentVariable
import os

def generate_launch_description():
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', # IGN_GAZEBO_RESOURCE_PATH ?
        os.path.join(get_package_prefix('solid_robot'),
                     'share'))

    robot_description = Command(["xacro ", os.path.join(
            get_package_share_directory("solid_robot"), "urdf", "solid_robot.urdf.xacro"
        )])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    default_world = os.path.join(
        get_package_share_directory("solid_robot"),
        'worlds',
        'empty.world'
        )    
    
    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
        )


    gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
            ),
            launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
        )

    spawn_robot = Node(
        package="ros_gz_sim",
         executable='create',
        arguments=[
            '-name', "solid_robot",
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01'
        ],
        output='screen',
    )

    bridge_params = os.path.join(get_package_share_directory("solid_robot"),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    return LaunchDescription([
        set_env_vars_resources,
        robot_state_publisher,
        world_arg,
        gazebo_launch,
        spawn_robot,
        ros_gz_bridge
    ])