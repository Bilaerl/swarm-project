import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


# gets paths to rover urdf in the package share directory
path_prefix = get_package_share_directory("rover")
urdf_xacro_file_path = os.path.join(path_prefix, "urdf", "my_rover.xacro")

# slam_toolbox_config_file_path = os.path.join(path_prefix, "config", "mapper_params_online_async.yaml")
# ekf_config_file_path = os.path.join(path_prefix, "config", "ekf.yaml")


def generate_launch_description():

    rover_name = LaunchConfiguration("rover_name")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")

    rover_name_arg = DeclareLaunchArgument(
        "rover_name",
        default_value="my_rover",
        description="Namespace for the rover"
    )

    rover_x_arg = DeclareLaunchArgument(
        "x",
        default_value="0",
        description="Initial x position of the rover in Gazebo"
    )

    rover_y_arg = DeclareLaunchArgument(
        "y",
        default_value="0",
        description="Initial y position of the rover in Gazebo"
    )

    rover_z_arg = DeclareLaunchArgument(
        "z",
        default_value="1.56",
        description="Initial z position of the rover in Gazebo"
    )

    # creates a robot_description topic with the contents of robot.urdf
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace= rover_name,
        parameters=[{
            "robot_description": ParameterValue(
                Command(["xacro ", urdf_xacro_file_path, " robot_name:=", rover_name]), value_type=str),
            "use_sim_time": True,
            "frame_prefix": [rover_name, "/"],  # so that each tf frame will be prefixed to differentiate between instances
        }],
        output="screen"
    )

    # spawn robot in gazebo via the robot_description topic
    robot_spawn_node = Node(
        package="ros_gz_sim",
        executable="create",
        name="robot_spawn_node",
        arguments=[
            "-topic", [rover_name, "/robot_description"],
            "-name", rover_name,
            "-x", x,
            "-y", y,
            "-z", z
        ],
        output="screen"
    )

    # ros_gz_bridge_node = Node(
    #     package="ros_gz_bridge",
    #     executable="parameter_bridge",
    #     name="ros_gz_bridge_node",
    #     arguments=[
    #         "clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
    #         "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
    #         "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
    #         "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
    #         "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
    #         "/model/my_robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
    #         "/model/my_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
    #         "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
    #         "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
    #     ],
    #     remappings=[
    #         ("/model/my_robot/tf", "/tf"),
    #         ("/model/my_robot/odometry", "/odom"),
    #     ],
    #     parameters=[{
    #         "use_sim_time": True
    #     }],
    #     output="screen"
    # )

    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz",
    #     output="screen"
    # )

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
        rover_x_arg,
        rover_y_arg,
        rover_z_arg,
        robot_state_publisher_node,
        robot_spawn_node,
        # ros_gz_bridge_node,
        # rviz_node,
        # ekf_filter_node,
        # slam_node,
        # slam_lifecycle_manager_node,
    ])