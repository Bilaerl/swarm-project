import functools
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from ros_gz_interfaces.msg import Entity
from custom_interfaces.msg import PolarVector
from ros_gz_interfaces.srv import SetEntityPose
from custom_interfaces.srv import ResetArtifacts

from gymnasium import spaces
from pettingzoo import ParallelEnv



class SwarmForagingEnvironment(ParallelEnv, Node):
    metadata = {"name": "swarm_foraging_environment_v0",}

    def __init__(self, rovers_dict, render_mode=None):
        ParallelEnv.__init__(self)
        Node.__init__(self, "pettingzoo_node")
        
        self.possible_agents = list(rovers_dict.keys())
        self.agents = self.possible_agents[:]

        
        # internal state tracking
        self._world_name = "swarm_world"
        self._rover_initial_poses = rovers_dict
        self._latest_lidar_scans = {agent: np.ones(8, dtype=np.float32) * 5.0 for agent in self.possible_agents}

        self._latest_inventory_status = {agent: 0.0 for agent in self.possible_agents}
        self._previous_inventory_status = {agent: 0.0 for agent in self.possible_agents}

        self._latest_nest_vectors = {agent: {"r":np.inf, "theta":np.inf} for agent in self.possible_agents}
        self._previous_nest_vectors = {agent: {"r":np.inf, "theta":np.inf} for agent in self.possible_agents}

        self._latest_target_artifact_vectors = {agent: {"r":np.inf, "theta":np.inf} for agent in self.possible_agents}
        self._previous_target_artifact_vectors = {agent: {"r":np.inf, "theta":np.inf} for agent in self.possible_agents}

        # ROS2 Publishers & Subscribers
        self._cmd_vel_pubs = {}
        for agent in self.possible_agents:
            self._cmd_vel_pubs[agent] = self.create_publisher(Twist, f"/{agent}/cmd_vel", 10)
            self.create_subscription(Float32, f"/{agent}/rover_core/inventory_status", lambda msg, a=agent: self._inventory_status_callback(msg, a), 10)
            self.create_subscription(LaserScan, f"/{agent}/scan", lambda msg, a=agent: self._scan_callback(msg, a), 10)
            self.create_subscription(PolarVector, f"/{agent}/radio/nest_vector", lambda msg, a=agent: self._nest_vector_callback(msg, a), 10)
            self.create_subscription(PolarVector, f"/{agent}/rover_core/target_artifact_vector", lambda msg, a=agent: self._target_artifact_vector_callback(msg, a), 10)

        # Gazebo Set Entity Pose Service Client
        self.set_pose_client = self.create_client(SetEntityPose, f"/world/{self._world_name}/set_pose")
        while not self.set_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for Gazebo set_pose service...")

        self.reset_artifacts_client = self.create_client(ResetArtifacts, "/artifact_manager/reset_artifacts")
        while not self.reset_artifacts_client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info("Waiting for Artifact Manager reset service...")

    def reset(self, seed=None, options=None):
        self.agents = self.possible_agents[:]
        self._inventory_status = {agent: 0.0 for agent in self.possible_agents}
        
        # stop all rovers immediately
        for agent in self.agents:
            stop_twist = Twist()
            self._cmd_vel_pubs[agent].publish(stop_twist)

        # teleport rovers back to their starting spawn coordinates
        for agent in self.agents:
            pose_req = SetEntityPose.Request()
            pose_req.entity.name = agent
            pose_req.entity.type = Entity.MODEL
            
            # set initial position and orientation (quaternion w=1.0)
            target_pose = Pose()
            target_pose.position.x = self._rover_initial_poses[agent]["x"]
            target_pose.position.y = self._rover_initial_poses[agent]["y"]
            target_pose.position.z = self._rover_initial_poses[agent]["z"]
            target_pose.orientation.w = 1.0
            pose_req.pose = target_pose
            
            # call the service synchronously for the reset step
            self.set_pose_client.call_async(pose_req)

        # reset artifacts via your C++ artifact_manager service
        success = self._call_artifact_manager_reset()
        if not success:
            self.get_logger().error("Failed to reset artifacts via Artifact Manager service.")

        # 4. Spin ROS 2 to let physics and TF trees catch up
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds > 2e8: # 200ms settling time
            rclpy.spin_once(self, timeout_sec=0.01)

        # 5. Gather fresh observations
        observations = {agent: self._get_observation(agent)[0] for agent in self.agents}
        infos = {agent: {} for agent in self.agents}
        
        return observations, infos

    def step(self, actions):
        # publish velocity actions to ROS 2 topics
        for agent, action in actions.items():
            twist = Twist()
            twist.linear.x = float(action[0])
            twist.angular.z = float(action[1])
            self._cmd_vel_pubs[agent].publish(twist)

        # step ROS 2 event loop to advance Gazebo physics (100ms window)
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds < 1e8:
            rclpy.spin_once(self, timeout_sec=0.01)

        observations, rewards, terminations, truncations, infos = {}, {}, {}, {}, {}

        for agent in self.agents:
            observation, reward = self._get_observation(agent)
            observations[agent] = observation
            
            rewards[agent] = reward
            terminations[agent] = False
            truncations[agent] = False
            infos[agent] = {}

        return observations, rewards, terminations, truncations, infos

    def render(self):
        pass

    @functools.lru_cache(maxsize=None)
    def observation_space(self, agent):
        return spaces.Box(low=-np.inf, high=np.inf, shape=(13,), dtype=np.float32)

    @functools.lru_cache(maxsize=None)
    def action_space(self, agent):
        return spaces.Box(low=np.array([-0.5, -1.0]), high=np.array([0.5, 1.0]), dtype=np.float32)

    def _get_observation(self, agent_id):
        step_penalty = -0.05 # step penalty

        # get LiDAR bins
        lidar_obs = self._latest_lidar_scans[agent_id]
        collision_penalty = -5.0 if np.any(lidar_obs < 0.4) else 0.0

        # inventory_status
        latest_inventory_status = self._latest_inventory_status[agent_id]
        agent_inventory_status = np.array([float(self._inventory_status[agent_id])], dtype=np.float32)

        # inventory reward
        previous_inventory_status = self._previous_inventory_status[agent_id]
        inventory_reward = 0.0
        if latest_inventory_status > previous_inventory_status: # picked up more artifact
            inventory_reward = 10.0
        elif latest_inventory_status < previous_inventory_status: # dropped artifact at nest
            inventory_reward = 15.0
        
        # relative nest beacon vector
        latest_nest_vector = self._latest_nest_vectors[agent_id]
        nest_vector = np.array([latest_nest_vector["r"], latest_nest_vector["theta"]], dtype=np.float32)

        # relative nest beacon reward
        # the reward is scaled by agent inventory status,
        # therefore the fuller the inventory, the more the agent is incentivise to go to nest
        previous_nest_vector = self._previous_nest_vectors[agent_id]
        nest_r_reward = latest_nest_vector["r"] - previous_nest_vector["r"]
        nest_theta_reward = 0.1 * latest_nest_vector["theta"]
        nest_vector_reward = latest_inventory_status * (nest_r_reward + nest_theta_reward)

        # relative target artifact vector
        latest_target_artifact_vector = self._latest_target_artifact_vectors[agent_id]
        target_artifact_vector = np.array([latest_target_artifact_vector["r"], latest_target_artifact_vector["theta"]], dtype=np.float32)

        # relative target artifact vector reward
        # the reward is scaled by (1 - agent inventory status),
        # therefore the emptier the inventory, the more the agent is incentivise to forage for more artifacts
        previous_target_artifact_vector = self._previous_target_artifact_vectors[agent_id]
        target_artifact_r_reward = latest_target_artifact_vector["r"] - previous_target_artifact_vector["r"]
        target_artifact_theta_reward = 0.1 * latest_target_artifact_vector["theta"]
        target_artifact_vector_reward = (1 - latest_inventory_status) * (target_artifact_r_reward + target_artifact_theta_reward)

        # concatenate into 13-dimensional observation vector
        observation = np.concatenate([lidar_obs, target_artifact_vector, nest_vector, agent_inventory_status])
        reward = step_penalty + collision_penalty + inventory_reward + nest_vector_reward + target_artifact_vector_reward

        return observation, reward

    def _scan_callback(self, msg, agent_id):
        # bin 360-degree LiDAR scan into 8 directional sectors
        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges)] = msg.range_max
        binned = np.array_split(ranges, 8)
        self._latest_lidar_scans[agent_id] = np.array([b.min() for b in binned], dtype=np.float32)
    
    def _inventory_status_callback(self, msg, agent_id):
        self._previous_inventory_status[agent_id] = self._latest_inventory_status[agent_id]
        self._latest_inventory_status[agent_id] = msg.data

    def _nest_vector_callback(self, msg, agent_id):
        self._previous_nest_vectors[agent_id] = self._latest_nest_vectors[agent_id].copy() # shallow copy of dict
        self._latest_nest_vectors[agent_id] = {"r":msg.range, "theta":msg.theta}

    def _target_artifact_vector_callback(self, msg, agent_id):
        self._previous_nest_vectors[agent_id] = self._latest_nest_vectors[agent_id].copy() # shallow copy of dict
        self._latest_nest_vectors[agent_id] = {"r":msg.range, "theta":msg.theta}

    def _call_artifact_manager_reset(self):
        req = ResetArtifacts.Request()
        
        future = self.reset_artifacts_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        try:
            response = future.result()
            return response.success
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return False



def parse_rover_spawn_config(rover_spawn_config_file_path):
    """ Helper function to parse rover spawn config file """

    rover_dict = {}
    
    with open(rover_spawn_config_file_path, "r") as file:
        for line in file:
            # skip comments
            if line[0] == "#":
                continue

            rover_name, rover_x, rover_y, rover_z = line.split()

            rover_dict[rover_name] = {"x": float(rover_x), "y": float(rover_y), "z": float(rover_z)}
    
    return rover_dict