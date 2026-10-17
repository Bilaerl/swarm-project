import os
import time
import rclpy
from ament_index_python import get_package_share_directory
from swarm_training import SwarmForagingEnvironment, parse_rover_spawn_config

def main():
    rclpy.init()
    
    # Initialize environment (make sure Gazebo simulation is already running)
    rover_config_path_prefix = get_package_share_directory("swarm_bringup")
    rover_spawn_config_file_path = os.path.join(rover_config_path_prefix, "config", "agents_spawn_config.txt")

    rovers_dict = parse_rover_spawn_config(rover_spawn_config_file_path)
    env = SwarmForagingEnvironment(rovers_dict)
    
    print("Resetting environment...")
    obs_dict, infos = env.reset()
    
    print(f"Active Agents: {env.agents}")
    for agent, obs in obs_dict.items():
        print(f"  {agent} initial observation shape: {obs.shape}")
        print(f"    - LiDAR bins: {obs[:8]}")
        print(f"    - Artifact (x,y): {obs[8:10]}")
        print(f"    - Nest Beacon (r, theta): {obs[10:12]}")
        print(f"    - Carrying Flag: {obs[12]}")

    print("\nStarting 100-step random action rollout...")
    try:
        for step in range(100):
            # Sample random actions for each rover within their valid action space bounds
            actions = {agent: env.action_space(agent).sample() for agent in env.agents}
            
            # Step the environment (publishes cmd_vel, spins rclpy, steps Gazebo physics)
            obs_dict, rewards, terminations, truncations, infos = env.step(actions)
            
            print(f"Step {step+1:03d} | Rewards: { {k: round(v, 2) for k, v in rewards.items()} }")
            
            # Check if any episode ending flags were triggered
            if any(terminations.values()) or any(truncations.values()):
                print("Episode finished/truncated. Resetting environment...")
                obs_dict, infos = env.reset()
                
            time.sleep(0.05) # Small pacing delay for visual observation in Gazebo
            
    except KeyboardInterrupt:
        print("Test interrupted by user.")
    finally:
        env.close()
        rclpy.shutdown()
        print("Test completed and cleaned up.")

if __name__ == "__main__":
    main()