import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


# gets path to rover urdf in the package share directory
path_prefix = get_package_share_directory("swarm")
rover_xacro_file_path = os.path.join(path_prefix, "models", "rover", "rover.xacro")

# gets path to rover brain launch file in the rover_brain package share directory
rover_brain_path_prefix = get_package_share_directory("rover_brain")
rover_brain_launch_file_path = os.path.join(rover_brain_path_prefix, "launch", "rover_brain_launch.py")



def generate_launch_description():

    rover_name = LaunchConfiguration("rover_name")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")

    rover_name_arg = DeclareLaunchArgument(
        "rover_name",
        default_value="rover",
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
        namespace=rover_name,
        parameters=[{
            "robot_description": ParameterValue(
                Command(["xacro ", rover_xacro_file_path, " robot_name:=", rover_name]), value_type=str),
            "frame_prefix": [rover_name, "/"],  # so that each tf frame will be prefixed to differentiate between instances
            "use_sim_time": True,
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

    ros_gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge_node",
        namespace=rover_name,
        arguments=[
            ["/", rover_name ,"/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image"],
            ["/", rover_name ,"/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
            ["/", rover_name ,"/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"],
            ["/", rover_name ,"/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model"],
            ["/", rover_name ,"/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"],
            ["/", rover_name ,"/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry"],
            ["/", rover_name ,"/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"],
            ["/", rover_name ,"/imu@sensor_msgs/msg/Imu[gz.msgs.IMU"],
        ],
        parameters=[{"use_sim_time": True}],
        output="screen"
    )

    rover_brain_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(rover_brain_launch_file_path),
        launch_arguments={
            "rover_name": rover_name,
            "use_sim_time": "True",   # because this launch file is only used when working with a simulator
        }.items()
    )

    return LaunchDescription([
        rover_name_arg,
        rover_x_arg,
        rover_y_arg,
        rover_z_arg,
        robot_state_publisher_node,
        robot_spawn_node,
        ros_gz_bridge_node,
        rover_brain_launch,
    ])