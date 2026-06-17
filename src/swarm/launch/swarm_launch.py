import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
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

# gets the path to the rover launch file
rover_launch_file = os.path.join(path_prefix, "launch", "rover_launch.py")

# gets the path to the artifact file
artifact_file_path = os.path.join(path_prefix, "models", "artifact.sdf")

# gets the path to config files in the swarm package
rviz_config_file_path = os.path.join(path_prefix, "rviz", "swarm-project.rviz")
slam_toolbox_config_file_path = os.path.join(path_prefix, "config", "mapper_params_online_async.yaml")
ekf_config_file_path = os.path.join(path_prefix, "config", "ekf.yaml")


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

    # saved the launch description in a variable instead of just returning it
    # so a loop can be used to add the rover spawning nodes to it before returning it
    swarm_launch_description = LaunchDescription([
        gz_sim_env_variables,
        gz_sim_launch,
    ])


    # configuration for each artifact to be added to the swarm
    # this artifacts are what the rovers will be foraging for in the swarm world
    artifact_configurations = [
        {"x":"1", "y":"0", "z":"1.56"},
        {"x":"1", "y":"1.5", "z":"1.56"},
    ]

    for num, artifact_config in enumerate(artifact_configurations, start=1):
        artifact_spawn_node = Node(
            package="ros_gz_sim",
            executable="create",
            name=f"artifact_spawn_node_{num}",
            arguments=[
                "-file", artifact_file_path,
                "-name", f"artifact_{num}",
                "-x", artifact_config["x"],
                "-y", artifact_config["y"],
                "-z", artifact_config["z"],
            ],
            output="screen"
        )

        swarm_launch_description.add_action(artifact_spawn_node) # add the artifact spawn node to this launch description

    
    # configuration for each rover to be added to the swarm
    swarm_agents_configurations = [
        {"x":"0", "y":"0", "z":"1.56"},
        {"x":"0", "y":"1.5", "z":"1.56"},
    ]

    for num, agent_config in enumerate(swarm_agents_configurations, start=1):
        rover_launch = IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(rover_launch_file),
            launch_arguments={
                "rover_name": f"rover_{num}",
                "x": agent_config["x"],
                "y": agent_config["y"],
                "z": agent_config["z"],
            }.items()
        )

        swarm_launch_description.add_action(rover_launch) # add the rover launch to this launch description

    return swarm_launch_description