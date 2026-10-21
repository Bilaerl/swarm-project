import functools
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
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
        self._is_carrying = {agent: 0 for agent in self.possible_agents}
        self._latest_lidar_scans = {agent: np.ones(8, dtype=np.float32) * 5.0 for agent in self.possible_agents}
        self._latest_nest_vectors = {agent: {"r":np.inf, "theta":np.inf} for agent in self.possible_agents}
        self._latest_target_artifact_vectors = {agent: {"r":np.inf, "theta":np.inf} for agent in self.possible_agents}

        # ROS2 Publishers & Subscribers
        self._cmd_vel_pubs = {}
        for agent in self.possible_agents:
            self.cmd_pubs[agent] = self.create_publisher(Twist, f"/{agent}/cmd_vel", 10)
            self.create_subscription(Int32, f"/{agent}/rover_core/rover_inventory", lambda msg, a=agent: self._rover_inventory_callback(msg, a), 10)
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
        self._is_carrying = {agent: 0 for agent in self.possible_agents}
        
        # stop all rovers immediately
        for agent in self.agents:
            stop_twist = Twist()
            self.cmd_pubs[agent].publish(stop_twist)

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
        observations = {agent: self._get_observation(agent) for agent in self.agents}
        infos = {agent: {} for agent in self.agents}
        
        return observations, infos

    def step(self, actions):
        # publish velocity actions to ROS 2 topics
        for agent, action in actions.items():
            twist = Twist()
            twist.linear.x = float(action[0])
            twist.angular.z = float(action[1])
            self.cmd_pubs[agent].publish(twist)

        # step ROS 2 event loop to advance Gazebo physics (100ms window)
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds < 1e8:
            rclpy.spin_once(self, timeout_sec=0.01)

        observations, rewards, terminations, truncations, infos = {}, {}, {}, {}, {}

        for agent in self.agents:
            obs = self._get_observation(agent)
            observations[agent] = obs
            
            # simple reward calculation (Step penalty + collision penalty if LiDAR < 0.4m)
            reward = -0.1
            if np.any(self._latest_lidar_scans[agent] < 0.4):
                reward -= 5.0
            
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
        # get LiDAR bins
        lidar_obs = self._latest_lidar_scans[agent_id]
        
        # get relative nest beacon vector via TF2
        r_nest, theta_nest = 0.0, 0.0
        try:
            trans = self.tf_buffer.lookup_transform(f"{agent_id}/base_link", "nest_link", rclpy.time.Time())
            dx = trans.transform.translation.x
            dy = trans.transform.translation.y
            r_nest = np.sqrt(dx**2 + dy**2)
            theta_nest = np.arctan2(dy, dx)
        except Exception:
            pass

        # placeholder artifact relative pose (x, y) from image processor cache
        artifact_obs = np.array([0.0, 0.0], dtype=np.float32)
        
        carrying_flag = np.array([float(self._is_carrying[agent_id])], dtype=np.float32)

        # concatenate into 13-dimensional observation vector
        return np.concatenate([lidar_obs, artifact_obs, np.array([r_nest, theta_nest]), carrying_flag])

    def _scan_callback(self, msg, agent_id):
        # bin 360-degree LiDAR scan into 8 directional sectors
        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges)] = msg.range_max
        binned = np.array_split(ranges, 8)
        self._latest_lidar_scans[agent_id] = np.array([b.min() for b in binned], dtype=np.float32)
    
    def _rover_inventory_callback(self, msg, agent_id):
        pass

    def _nest_vector_callback(msg, agent_id):
        pass

    def _target_artifact_vector_callback(msg, agent_id):
        pass

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