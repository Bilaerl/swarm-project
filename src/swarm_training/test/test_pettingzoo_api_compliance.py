import os
import rclpy
from ament_index_python.packages import get_package_share_directory
from pettingzoo.test import parallel_api_test
from swarm_training import SwarmForagingEnvironment, parse_rover_spawn_config

def main():
    rclpy.init()
    
    # 1. Instantiate your custom environment
    rover_config_path_prefix = get_package_share_directory("swarm_bringup")
    rover_spawn_config_file_path = os.path.join(rover_config_path_prefix, "config", "agents_spawn_config.txt")

    rovers_dict = parse_rover_spawn_config(rover_spawn_config_file_path)
    env = SwarmForagingEnvironment(rovers_dict)
    
    print("Running PettingZoo Parallel API compliance tests...")
    
    # 2. Run the official test suite
    parallel_api_test(env, num_cycles=50)
    
    print("API compliance test passed successfully!")
    
    env.close()
    rclpy.shutdown()


if __name__ == "__main__":
    main()