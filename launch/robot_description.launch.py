import os
import launch_ros
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

# Function to generate the launch description, which defines all the nodes and parameters for launching.
def generate_launch_description():
    
    # Chose the robot name for the simulation
    robot_name = "go1"
    
    # Find the package directory for "b1_description" using ROS 2's package index.
    robot_description_pkg = launch_ros.substitutions.FindPackageShare(package= robot_name + "_description").find(robot_name + "_description")
    
    # Set the default path for the robot's URDF file (in Xacro format).
    default_model_path = os.path.join(robot_description_pkg, "xacro/robot.xacro")

    # Launch configuration variables (these will be set by launch arguments).
    use_sim_time = LaunchConfiguration("use_sim_time")
    description_path = LaunchConfiguration("description_path")
    
    # Declare launch arguments with default values and descriptions.
    declare_description_path = DeclareLaunchArgument(
        name="description_path",  # Name of the launch argument.
        default_value=default_model_path,  # Default value is the path to the URDF file.
        description="Absolute path to robot urdf file"  # Description for this argument.
    )
    
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",  # Name of the launch argument.
        default_value="false",  # Default value is "false", meaning real time is used by default.
        description="Use simulation (Gazebo) clock if true"  # Description for this argument.
    )

    declare_sensors_flag = DeclareLaunchArgument(
        "SENSORS", 
        default_value="false", 
        description="Flag use to load sensors"
    )   

    declare_sensors_flag = DeclareLaunchArgument(
        "ign", 
        default_value="false", 
        description="Flag use to load sensors"
    )   
        
    # Define the robot_state_publisher node, which publishes the robot's state (e.g., joint positions) to the ROS TF system.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",  # Package that provides the robot_state_publisher executable.
        executable="robot_state_publisher",  # The executable to run.
        
        parameters=[
            {"robot_description": Command(["xacro ", description_path, " SENSORS:=", LaunchConfiguration("SENSORS"), " ign:=", LaunchConfiguration("ign")])},  # The robot's URDF description, generated by processing the Xacro file.
            {"use_tf_static": False},  # Whether to use static transforms; False means the transforms are dynamic.
            {"publish_frequency": 200.0},  # Frequency at which the transforms are published (200 Hz).
            {"ignore_timestamp": True},  # Ignore timestamp in published transforms (useful when combining live data with simulation).
            {'use_sim_time': use_sim_time},  # Use simulated time if specified by the "use_sim_time" argument.
        ],
        # remappings=(("robot_description", "robot_description")),  # Commented out remapping; no need to remap in this setup.
    )
    
    joint_state_publisher_node = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            
            parameters=[
                {'use_sim_time': use_sim_time}  # Use simulated time if specified by the "use_sim_time" argument.
            ],
    )
    
    # Return the launch description, which includes all the declared launch arguments and the robot_state_publisher node.
    return LaunchDescription(
        [
            declare_description_path,  # Include the description path launch argument.
            declare_use_sim_time,  # Include the use_sim_time launch argument.
            declare_sensors_flag,
            robot_state_publisher_node,  # Include the robot_state_publisher node in the launch description.
            # joint_state_publisher_node
        ]
    )
