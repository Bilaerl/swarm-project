import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# get path to package share directory
path_prefix = get_package_share_directory("rover_brain")

# path to config files in the rover_brain package
slam_toolbox_config_file_path = os.path.join(path_prefix, "config", "mapper_params_online_async.yaml")
ekf_config_file_path = os.path.join(path_prefix, "config", "ekf.yaml")



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
        parameters=[{
            "use_sim_time": use_sim_time,
            "rover_name": rover_name    # needed to pass name to artifact manager in simulation, can be removed on physical rover
        }],
        output="screen"
    )

    ekf_filter_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        namespace=rover_name,
        parameters=[
            ekf_config_file_path,
            {
                "use_sim_time": use_sim_time,
                "odom_frame": [rover_name, "/odom"],
                "base_link_frame": [rover_name, "/base_link"],
                "world_frame": [rover_name, "/odom"],
                "map_frame": [rover_name, "/map"],
            }
        ],
        output="screen"
    )

    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace=rover_name,
        parameters=[
            slam_toolbox_config_file_path,
            {
                "use_sim_time": use_sim_time,
                "base_frame": [rover_name, "/base_link"],
                "odom_frame": [rover_name, "/odom"],
                "map_frame": [rover_name, "/map"],
            }
        ],
        remappings=[
            ("/map", "map"), # Remap the /map topic to the rover's namespace
            ("/map_metadata", "map_metadata") # Remap the /map_metadata topic to the rover's namespace
        ],
        output="screen"
    )

    slam_lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="slam_lifecycle_manager",
        namespace=rover_name,
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "bond_timeout": 0.0,  # Disables bond timer requirement for slam_toolbox
            "node_names": ["slam_toolbox"] # Must match the 'name' in your slam_node
        }],
        output="screen"
    )

    return LaunchDescription([
        rover_name_arg,
        use_sim_time_arg,
        rover_core_node,
        picker_node,
        ekf_filter_node,
        slam_node,
        slam_lifecycle_manager_node
    ])