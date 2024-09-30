import os
from os import environ
import launch_ros
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    
    # Set the robot name for the simulation.
    robot_name = "b1"
    
    # Find the package directory for "champ_simulation" using ROS 2's package index.
    ign_pkg = launch_ros.substitutions.FindPackageShare(package="champ_simulation").find("champ_simulation")
                                
    # Define paths for the world file and launch directory.
    world_file = os.path.join(ign_pkg, 'worlds', 'default_ign.world')  # Path to the default Ignition world file.
    launch_dir = os.path.join(ign_pkg, "launch")  # Path to the launch directory of the "champ_simulation" package.

    # Define the path to the Ignition configuration file.
    # ign_config = os.path.join(
    #     launch_ros.substitutions.FindPackageShare(package="champ_simulation").find("champ_simulation"),
    #     "config/ignition/ign_config.yaml"
    # )

    # Declare launch arguments with default values and descriptions.
    declare_robot_name = DeclareLaunchArgument("robot_name", default_value="go1")  # Name of the robot to be spawned.
    declare_headless = DeclareLaunchArgument("headless", default_value="False")  # Whether to run Ignition in headless mode (without GUI).
    declare_world_init_x = DeclareLaunchArgument("world_init_x", default_value="0.0")  # Initial x-coordinate of the robot in the world.
    declare_world_init_y = DeclareLaunchArgument("world_init_y", default_value="0.0")  # Initial y-coordinate of the robot in the world.
    declare_world_init_z = DeclareLaunchArgument("world_init_z", default_value="0.30")  # Initial z-coordinate of the robot in the world.
    declare_world_init_heading = DeclareLaunchArgument("world_init_heading", default_value="0.0")  # Initial heading (yaw) of the robot.
    declare_ign_world = DeclareLaunchArgument("ign_world", default_value=world_file, description="Ignition world name")  # Path to the Ignition world file.
    # declare_ign_config = DeclareLaunchArgument("ign_config", default_value=ign_config, description="Ignition config file path")  # Path to the Ignition configuration file.
    
    # Launch configuration variables (these will be set by launch arguments).
    robot_name = LaunchConfiguration("robot_name")
    headless = LaunchConfiguration("headless")
    world_init_x = LaunchConfiguration("world_init_x")
    world_init_y = LaunchConfiguration("world_init_y")
    world_init_z = LaunchConfiguration("world_init_z")
    world_init_heading = LaunchConfiguration("world_init_heading")
    ign_world = LaunchConfiguration("ign_world")
    # ign_config = LaunchConfiguration("ign_config")

    # Ignition env variables
    env = {  # IGN GAZEBO FORTRESS env variables
        "IGN_GAZEBO_SYSTEM_PLUGIN_PATH": ":".join(
            [
                environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", default=""),
                environ.get("LD_LIBRARY_PATH", default=""),
            ]
        ),
    }
    
    # Start the Ignition client (GUI) if not running in headless mode.
    start_ign_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression([" not ", headless])),  # Only start if headless mode is not enabled.
        cmd=["ign", "gazebo", "-v", "4", "-g"],  # Command to start the Ignition client (GUI mode).
        cwd=[launch_dir],  # Set the working directory to the launch directory.
        output="screen",  # Output logs to the screen.
    )

    # Start the Ignition server (simulation backend).
    start_ign_server_cmd = ExecuteProcess(
        cmd=[
            "ign", "gazebo",  # Command to start the Ignition server.
            ign_world,  # The world file to load.
            '-r',  # Run the simulation in real-time.
            # "--ros-args",  # Pass ROS arguments.
            # '--params-file',  # Specify the parameters file.
            # ign_config  # Path to the Ignition configuration file.
        ],
        cwd=[launch_dir],
        
        additional_env=env,# Set the working directory to the launch directory.
        output="screen"  # Output logs to the screen.
    )

    pkg_ros_gz_sim = get_package_share_directory(
        'ros_gz_sim')

    gz_sim_launch = PathJoinSubstitution(
        [pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    
        
    # Ignition processes
    ign_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch]),
        launch_arguments=[
            ('gz_args', [ign_world,
                         ' -v 4'])
        ]
    )
    
    # Command to spawn the robot entity in the Ignition simulation.
    start_ign_spawner_cmd = Node(
        package="ros_gz_sim",  # Package that provides the spawn_entity service in Ignition.
        executable="create",  # Command to create an entity (spawn).
        output="screen",  # Output logs to the screen.
        arguments=[
            "-name", robot_name,  # Name of the entity to spawn.
            "-topic", "/robot_description",  # Topic from which to get the robot's URDF description.
            "-x", world_init_x,  # Initial x-coordinate.
            "-y", world_init_y,  # Initial y-coordinate.
            "-z", world_init_z,  # Initial z-coordinate.
            "-R", "0",  # Initial roll (fixed to 0).
            "-P", "0",  # Initial pitch (fixed to 0).
            "-Y", world_init_heading,  # Initial yaw (heading).
        ]
    )
    
    # ros_gz_bridge Node for ROS-Ignition bridge
    ros_gz_bridge = Node(
        package="ros_gz_bridge",  # Updated package name for ROS 2 and Ignition bridge
        executable="parameter_bridge",  # Executable that provides the bridge functionality.
        output="log",  # Output logs to the log.
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",  # Bridge between ROS 2 and Ignition Clock topics.
            '/world/default/model/robot/joint_state@sensor_msgs/msg/JointState@ignition.msgs.Model',  # Example joint state bridge.
            '/lidar_points/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',  # Correct remap here
            # "/lidar_points@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan",
            "--ros-args",
            "--log-level",
            "warn",
        ],
        parameters=[
            {"use_sim_time": True},
        ],  # Use simulated time from Ignition.
    )

    # Return the launch description, which includes all declared launch arguments and the commands to start Ignition and spawn the robot.
    return LaunchDescription(
        [
            declare_headless,
            declare_ign_world,
            declare_robot_name,
            declare_world_init_x,
            declare_world_init_y,
            declare_world_init_z,
            declare_world_init_heading,
            # declare_ign_config,
            start_ign_server_cmd,
            # ign_sim,
            # start_ign_client_cmd,
            start_ign_spawner_cmd,
            ros_gz_bridge
        ]
    )
