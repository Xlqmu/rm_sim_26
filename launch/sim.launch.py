import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package name
    package_name = "rm_sim_26"

    # Get package share directory
    pkg_share = FindPackageShare(package_name)

    # World file path
    world_file = PathJoinSubstitution([pkg_share, "worlds", "omni.world"])

    robot_urdf_path = PathJoinSubstitution(
        [pkg_share, "moudles", "mecanum_car", "robot.urdf"]
    )

    # Robot model path
    robot_model_path = PathJoinSubstitution(
        [pkg_share, "moudles", "mecanum_car", "model.sdf"]
    )

    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation time"
    )

    # Launch Gazebo with the world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ]
        ),
        launch_arguments={
            "gz_args": ["-r -v 4 ", world_file],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"robot_description": Command(["cat ", robot_urdf_path])},
        ],
    )

    # Spawn the robot in Gazebo
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-file",
            robot_model_path,
            "-name",
            "mecanum_bot",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.3",
            "-R",
            "0.0",
            "-P",
            "0.0",
            "-Y",
            "0.0",
        ],
        output="screen",
    )

    # Bridge for cmd_vel topic
    cmd_vel_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist"],
        output="screen",
    )

    # Bridge for lidar topic
    lidar_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan"],
        output="screen",
    )

    # Bridge for IMU topic
    imu_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/imu@sensor_msgs/msg/Imu@gz.msgs.IMU"],
        output="screen",
    )

    # Bridge for gimbal yaw control
    gimbal_yaw_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/gimbal/yaw/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double"],
        output="screen",
    )

    # Bridge for gimbal pitch control
    gimbal_pitch_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/gimbal/pitch/cmd_pos@std_msgs/msg/Float64@gz.msgs.Double"],
        output="screen",
    )

    # Bridge for camera
    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/camera@sensor_msgs/msg/Image@gz.msgs.Image"],
        output="screen",
    )

    # Bridge for pose (TF)
    pose_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/model/mecanum_bot/pose@geometry_msgs/msg/PoseStamped@gz.msgs.Pose"
        ],
        output="screen",
    )

    # Bridge for joint states
    joint_state_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model"],
        output="screen",
    )

    # Delay the spawning of the robot to ensure Gazebo is ready
    delayed_spawn = TimerAction(period=3.0, actions=[spawn_robot])

    return LaunchDescription(
        [
            declare_use_sim_time,
            gazebo_launch,
            robot_state_publisher,
            delayed_spawn,
            cmd_vel_bridge,
            lidar_bridge,
            imu_bridge,
            gimbal_yaw_bridge,
            gimbal_pitch_bridge,
            camera_bridge,
            pose_bridge,
            joint_state_bridge,
        ]
    )
