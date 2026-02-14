import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


# gets paths to the gazebo sdf in the share directory
path_prefix = get_package_share_directory('my_robot')
gz_sim_world_file = os.path.join(path_prefix, 'worlds', 'swarm-world.sdf')
gz_sim_models_folder = os.path.join(path_prefix, 'models')


def generate_launch_description():

    gz_sim_node = ExecuteProcess(
        cmd=[f"export GZ_SIM_RESOURCE_PATH={gz_sim_models_folder}; $(which gz) sim", gz_sim_world_file],
        name="gz_sim",
        shell=True
    )

    return LaunchDescription([
        gz_sim_node,
    ])