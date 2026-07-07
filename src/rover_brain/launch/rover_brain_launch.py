import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# get path to package share directory
path_prefix = get_package_share_directory("rover_brain")


def generate_launch_description():
    rover_name = LaunchConfiguration("rover_name")
    use_sim_time = LaunchConfiguration("use_sim_time")

    rover_name_arg = DeclareLaunchArgument(
        "rover_name",
        default_value="rover",
        description="Namespace for the rover"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation time"
    )

    rover_core_node = Node(
        package="rover_brain",
        executable="rover_core",
        name="rover_core_node",
        namespace=rover_name,
        arguments=[rover_name], # used to set rover_name
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen"
    )

    picker_node = Node(
        package="rover_brain",
        executable="picker",
        name="picker_node",
        namespace=rover_name,
        arguments=[rover_name],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen"
    )

    # slam_node = Node(
    #     package="slam_toolbox",
    #     executable="async_slam_toolbox_node",
    #     name="slam_toolbox",
    #     parameters=[
    #         slam_toolbox_config_file_path,
    #         {"use_sim_time": True}
    #     ],
    #     output="screen"
    # )

    # slam_lifecycle_manager_node = Node(
    #     package='nav2_lifecycle_manager',
    #     executable='lifecycle_manager',
    #     name='slam_lifecycle_manager',
    #     parameters=[{
    #         'use_sim_time': True,
    #         'autostart': True,
    #         'node_names': ['slam_toolbox'] # Must match the 'name' in your slam_node
    #     }]
    # )

    # ekf_filter_node = Node(
    #     package="robot_localization",
    #     executable="ekf_node",
    #     name="ekf_filter_node",
    #     parameters=[
    #         ekf_config_file_path,
    #         {"use_sim_time": True}
    #     ],
    #     output="screen"
    # )

    return LaunchDescription([
        rover_name_arg,
        use_sim_time_arg,
        rover_core_node,
        picker_node
    ])