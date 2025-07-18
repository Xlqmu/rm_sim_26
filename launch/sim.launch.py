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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package name
    package_name = "rm_sim_26"

    # Get package share directory
    pkg_share = FindPackageShare(package_name)

    # World file path
    world_file = PathJoinSubstitution([pkg_share, "worlds", "omni.world"])

    # Robot model path - 使用完整路径
    robot_model_path = PathJoinSubstitution(
        [pkg_share, "moudles", "mecanmu_car", "model.sdf"]
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

    # 使用 Node 方式创建机器人实体
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
            "0.15",
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

    # 延迟生成机器人，确保Gazebo完全启动
    delayed_spawn = TimerAction(period=3.0, actions=[spawn_robot])

    return LaunchDescription(
        [
            declare_use_sim_time,
            gazebo_launch,
            delayed_spawn,
            cmd_vel_bridge,
            lidar_bridge,
        ]
    )
