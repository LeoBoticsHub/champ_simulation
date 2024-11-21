import os
import launch_ros
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

def extract_base_link_from_yaml(yaml_file):
    with open(yaml_file, 'r') as file:
        for line in file:
            # Look for the 'base' key and extract its value
            if 'base' in line:
                base_link = line.split(':')[-1].strip().strip('"')
                return base_link
    return None
# Function to generate the launch description
def generate_launch_description():
    
    # Chose the robot name for the simulation
    robot_name = "go1"
                    
    # Package share directories
    gazebo_pkg = launch_ros.substitutions.FindPackageShare(package="champ_simulation").find("champ_simulation")    
    robot_description_pkg = launch_ros.substitutions.FindPackageShare(package= robot_name + "_description").find(robot_name + "_description")

    # Define paths to the robot's URDF, various configuration files, the Gazebo world file, and the launch directory.
    default_model_path = os.path.join(robot_description_pkg, "xacro/robot.xacro")
    world_file = os.path.join(gazebo_pkg, 'worlds', 'default.world')
    launch_dir = os.path.join(gazebo_pkg, "launch")
    
    # Paths to specific configuration files
    joints_config = os.path.join(robot_description_pkg, "config/joints/joints.yaml")
    links_config = os.path.join(robot_description_pkg, "config/links/links.yaml")
    gait_config = os.path.join(robot_description_pkg, "config/gait/gait.yaml")
    gazebo_config = os.path.join(gazebo_pkg, "config/gazebo/gazebo.yaml")

    base_link_frame = extract_base_link_from_yaml(links_config)

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
        default_value="true",  # Default value is "false", meaning real time is used by default.
        description="Use simulation (Gazebo) clock if true"  # Description for this argument.
    )

    declare_sensors_flag = DeclareLaunchArgument(
        "SENSORS", 
        default_value="false", 
        description="Flag use to load sensors"
    )   

    declare_ign_flag = DeclareLaunchArgument(
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
            declare_ign_flag,
            robot_state_publisher_node,  # Include the robot_state_publisher node in the launch description.
            # joint_state_publisher_node
        ]
    )