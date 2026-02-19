import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


# gets paths to the gazebo worlds and models in the share directory
path_prefix = get_package_share_directory("swarm")
gz_sim_world_file = os.path.join(path_prefix, "worlds", "swarm-world.sdf")
gz_sim_models_folder = os.path.join(path_prefix, "models")

# gets the path to the gz_sim launch file in the ros_gz_sim package
# this is needed to launch gazebo with the swarm world and models
gz_sim_package_dir = get_package_share_directory("ros_gz_sim")
gz_sim_package_launch_file = os.path.join(gz_sim_package_dir, "launch", "gz_sim.launch.py")

urdf_file_path = os.path.join(path_prefix, "models", "robot.urdf")


def generate_launch_description():
    # needed so gazebo can find the swarm world and models when launched
    gz_sim_env_variables = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=gz_sim_models_folder
    )

    # using the ros_gz_sim package to launch gazebo
    gz_sim_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(gz_sim_package_launch_file),
        launch_arguments={
            "gz_args": gz_sim_world_file,
            "on_exit_shutdown": "True",
        }.items()
    )

    # creates a robot_description topic with the contents of robot.urdf
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{
            "robot_description": open(urdf_file_path).read()
        }],
        output="screen"
    )

    # spawn robot in gazebo via the robot_description topic
    robot_spawn_node = Node(
        package="ros_gz_sim",
        executable="create",
        name="robot_spawn_node",
        arguments=[
            "-topic", "robot_description",
            "-name", "my_robot",
            "-x", "0",
            "-y", "0",
            "-z", "1.56"
        ],
        output="screen"
    )

    return LaunchDescription([
        gz_sim_env_variables,
        gz_sim_launch,
        robot_state_publisher_node,
        robot_spawn_node
    ])