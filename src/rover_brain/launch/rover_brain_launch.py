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

    image_processor_node = Node(
        package="rover_brain",
        executable="image_processor",
        name="image_processor_node",
        namespace=rover_name,
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen"
    )

    return LaunchDescription([
        rover_name_arg,
        use_sim_time_arg,
        rover_core_node,
        picker_node,
        ekf_filter_node,
        image_processor_node
    ])