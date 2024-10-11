import os
import launch_ros
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

# Function to generate the launch description, which defines all the nodes and commands for launching.
def generate_launch_description():
    
    # Chose the robot name for the simulation
    robot_name = "b1"
    
    # Find the package directory for "champ_simulation" using ROS 2's package index.
    gazebo_pkg = launch_ros.substitutions.FindPackageShare(package="champ_simulation").find("champ_simulation")
    robot_description_pkg = launch_ros.substitutions.FindPackageShare(package= robot_name + "_description").find(robot_name + "_description")
                                
    # Define paths for the world file and launch directory.
    world_file = os.path.join(gazebo_pkg, 'worlds', 'default.world')  # Path to the default Gazebo world file.
    launch_dir = os.path.join(gazebo_pkg, "launch")  # Path to the launch directory of the "champ_simulation" package.
    links_config = os.path.join(robot_description_pkg, "config/links/links.yaml")

    # Define the path to the Gazebo configuration file.
    gazebo_config = os.path.join(
        launch_ros.substitutions.FindPackageShare(package="champ_simulation").find("champ_simulation"),
        "config/gazebo/gazebo.yaml"
    )

    # Declare launch arguments with default values and descriptions.
    declare_robot_name = DeclareLaunchArgument("robot_name", default_value="go1")  # Name of the robot to be spawned.
    declare_headless = DeclareLaunchArgument("headless", default_value="False")  # Whether to run Gazebo in headless mode (without GUI).
    declare_world_init_x = DeclareLaunchArgument("world_init_x", default_value="0.0")  # Initial x-coordinate of the robot in the world.
    declare_world_init_y = DeclareLaunchArgument("world_init_y", default_value="0.0")  # Initial y-coordinate of the robot in the world.
    declare_world_init_z = DeclareLaunchArgument("world_init_z", default_value="0.30")  # Initial z-coordinate of the robot in the world.
    declare_world_init_heading = DeclareLaunchArgument("world_init_heading", default_value="0.0")  # Initial heading (yaw) of the robot.
    declare_gazebo_world = DeclareLaunchArgument("gazebo_world", default_value=world_file, description="Gazebo world name")  # Path to the Gazebo world file.
    declare_gazebo_config = DeclareLaunchArgument("gazebo_config", default_value=gazebo_config, description="Gazebo config file path")  # Path to the Gazebo configuration file.
    
    # Launch configuration variables (these will be set by launch arguments).
    robot_name = LaunchConfiguration("robot_name")
    headless = LaunchConfiguration("headless")
    world_init_x = LaunchConfiguration("world_init_x")
    world_init_y = LaunchConfiguration("world_init_y")
    world_init_z = LaunchConfiguration("world_init_z")
    world_init_heading = LaunchConfiguration("world_init_heading")
    gazebo_world = LaunchConfiguration("gazebo_world")
    gazebo_config = LaunchConfiguration("gazebo_config")

    # Start the Gazebo client (GUI) if not running in headless mode.
    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression([" not ", headless])),  # Only start if headless mode is not enabled.
        cmd=["gzclient"],  # Command to start the Gazebo client.
        cwd=[launch_dir],  # Set the working directory to the launch directory.
        output="screen",  # Output logs to the screen.
    )

    # Start the Gazebo server (simulation backend).
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=[
            "gzserver",  # Command to start the Gazebo server.
            # '-u',  # Run the server in unpaused mode.
            "-s", "libgazebo_ros_init.so",  # Load the ROS 2 integration plugin.
            "-s", "libgazebo_ros_factory.so",  # Load the factory plugin for spawning entities.
            gazebo_world,  # The world file to load.
            '--ros-args',  # Pass ROS arguments.
            '--params-file',  # Specify the parameters file.
            gazebo_config  # Path to the Gazebo configuration file.
        ],
        cwd=[launch_dir],  # Set the working directory to the launch directory.
        output="screen"  # Output logs to the screen.
    )

    # Command to spawn the robot entity in the Gazebo simulation.
    start_gazebo_spawner_cmd = Node(
        package="gazebo_ros",  # Package that provides the spawn_entity.py script.
        executable="spawn_entity.py",  # Executable to run (spawns the robot in Gazebo).
        output="screen",  # Output logs to the screen.
        arguments=[
            "-entity", robot_name,  # Name of the entity to spawn.
            "-topic", "/robot_description",  # Topic from which to get the robot's URDF description.
            "-robot_namespace", "",  # Namespace for the robot (empty in this case).
            "-x", world_init_x,  # Initial x-coordinate.
            "-y", world_init_y,  # Initial y-coordinate.
            "-z", world_init_z,  # Initial z-coordinate.
            "-R", "0",  # Initial roll (fixed to 0).
            "-P", "0",  # Initial pitch (fixed to 0).
            "-Y", world_init_heading,  # Initial yaw (heading).
        ]
    )

    contact_sensor = Node(
        package="champ_gazebo",
        executable="contact_sensor",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")},links_config],
        # prefix=['xterm -e gdb -ex run --args'],
    )
        
    # Return the launch description, which includes all declared launch arguments and the commands to start Gazebo and spawn the robot.
    return LaunchDescription(
        [
            declare_headless,
            declare_gazebo_world,
            declare_robot_name,
            declare_world_init_x,
            declare_world_init_y,
            declare_world_init_z,
            declare_world_init_heading,
            declare_gazebo_config,
            start_gazebo_server_cmd,
            start_gazebo_client_cmd,
            start_gazebo_spawner_cmd,
            contact_sensor
        ]
    )
