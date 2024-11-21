import os
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

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
    robot_name = "b1"
                    
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

    # Declare launch arguments for the robot description and simulation time
    declare_description_path = DeclareLaunchArgument(
        name="description_path", 
        default_value=default_model_path, 
        description="Absolute path to robot URDF file"
    )
    
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", 
        default_value="True", 
        description="Use simulation (Gazebo) clock if true"
    )    

    # Declare launch arguments for the bringup launch file
    declare_joints_map_path = DeclareLaunchArgument(
        name="joints_map_path", 
        default_value='', 
        description="Absolute path to joints map file"
    )
    
    declare_links_map_path = DeclareLaunchArgument(
        name="links_map_path", 
        default_value='', 
        description="Absolute path to links map file"
    )
    
    declare_gait_config_path = DeclareLaunchArgument(
        name="gait_config_path", 
        default_value='', 
        description="Absolute path to gait config file"
    )
    
    declare_orientation_from_imu = DeclareLaunchArgument(
        "orientation_from_imu", 
        default_value="false", 
        description="Take orientation from IMU data"
    )
    
    declare_robot_name = DeclareLaunchArgument(
        "robot_name", 
        default_value=robot_name, 
        description="Robot name"
    )
    
    declare_base_link_frame = DeclareLaunchArgument(
        "base_link_frame", 
        default_value=base_link_frame, 
        description="Base link frame"
    )
    
    declare_lite = DeclareLaunchArgument(
        "lite", 
        default_value="false", 
        description="Lite version"
    )
    
    declare_rviz = DeclareLaunchArgument(
        "rviz", 
        default_value="false", 
        description="Launch RViz"
    )
    
    declare_gazebo = DeclareLaunchArgument(
        "gazebo", 
        default_value="false", 
        description="If in Gazebo simulation"
    )
    
    declare_joint_controller_topic = DeclareLaunchArgument(
        "joint_controller_topic", 
        default_value="joint_group_effort_controller/joint_trajectory", 
        description="Joint controller topic"
    )
    
    declare_hardware_connected = DeclareLaunchArgument(
        "joint_hardware_connected", 
        default_value="false", 
        description="Whether hardware is connected"
    )
    
    declare_publish_joint_control = DeclareLaunchArgument(
        "publish_joint_control", 
        default_value="true", 
        description="Publish joint control"
    )
    
    declare_publish_joint_states = DeclareLaunchArgument(
        "publish_joint_states", 
        default_value="true", 
        description="Publish joint states"
    )
    
    declare_publish_foot_contacts = DeclareLaunchArgument(
        "publish_foot_contacts", 
        default_value="true", 
        description="Publish foot contacts"
    )
    
    declare_publish_odom_tf = DeclareLaunchArgument(
        "publish_odom_tf", 
        default_value="true", 
        description="Publish odom tf from cmd_vel estimation"
    )
    
    declare_close_loop_odom = DeclareLaunchArgument(
        "close_loop_odom", 
        default_value="false", 
        description="Use closed-loop odometry"
    )   

    # Declare launch arguments for the Gazebo launch file
    declare_headless = DeclareLaunchArgument(
        "headless", 
        default_value="False", 
        description="Run Gazebo in headless mode (no GUI)"
    )
    
    declare_gazebo_world = DeclareLaunchArgument(
        "gazebo_world", 
        default_value=world_file, 
        description="Gazebo world name"
    )
    
    declare_robot_name = DeclareLaunchArgument(
        "robot_name", 
        default_value=robot_name, 
        description="Robot name"
    )
    
    declare_world_init_x = DeclareLaunchArgument(
        "world_init_x", 
        default_value="0.0", 
        description="Initial X position in the Gazebo world"
    )
    
    declare_world_init_y = DeclareLaunchArgument(
        "world_init_y", 
        default_value="0.0", 
        description="Initial Y position in the Gazebo world"
    )
    
    declare_world_init_z = DeclareLaunchArgument(
        "world_init_z", 
        default_value="0.9", 
        description="Initial Z position in the Gazebo world"
    )
    
    declare_world_init_heading = DeclareLaunchArgument(
        "world_init_heading", 
        default_value="0.0", 
        description="Initial heading (yaw) in the Gazebo world"
    )
    
    declare_gazebo_config = DeclareLaunchArgument(
        "gazebo_config", 
        default_value=gazebo_config, 
        description="Gazebo config file path"
    )
    
    declare_launch_dir = DeclareLaunchArgument(
        "launch_dir", 
        default_value=launch_dir, 
        description="Launch directory"
    )    

    declare_sensors_flag = DeclareLaunchArgument(
        "SENSORS", 
        default_value="false", 
        description="Flag use to load sensors"
    )   
       
    # Include the robot description launch file
    description_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("champ_simulation"),
                "launch",
                "robot_description.launch.py",
            )
        ),
        launch_arguments={
            "description_path": LaunchConfiguration("description_path"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )
    
    # # Include the bringup launch file
    # bringup_ld = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory("champ_simulation"),
    #             "launch",
    #             "bringup.launch.py",
    #         )
    #     ),
    #     launch_arguments={
    #         "description_path": default_model_path,
    #         "joints_map_path": joints_config,
    #         "links_map_path": links_config,
    #         "gait_config_path": gait_config,
    #         "use_sim_time": LaunchConfiguration("use_sim_time"),
    #         "robot_name": LaunchConfiguration("robot_name"),
    #         "gazebo": "true",
    #         "lite": LaunchConfiguration("lite"),
    #         "rviz": LaunchConfiguration("rviz"),
    #         "joint_controller_topic": "joint_group_effort_controller/joint_trajectory",
    #         "hardware_connected": "false",
    #         "publish_foot_contacts": "false",
    #         "close_loop_odom": "true",
    #     }.items(),
    # )
    
    delay_duration = 5.0  # Delay in seconds
        
    # Include the Gazebo launch file
    gazebo_ld = TimerAction(
        period=delay_duration/2,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("champ_simulation"),
                        "launch",
                        "gazebo.launch.py",
                    )
                ),
                launch_arguments={
                    "headless": LaunchConfiguration("headless"),
                    "gazebo_world": LaunchConfiguration("gazebo_world"),
                    "robot_name": LaunchConfiguration("robot_name"),
                    "world_init_x": LaunchConfiguration("world_init_x"),
                    "world_init_y": LaunchConfiguration("world_init_y"),
                    "world_init_z": LaunchConfiguration("world_init_z"),
                    "world_init_heading": LaunchConfiguration("world_init_heading"),
                    "SENSORS": LaunchConfiguration("SENSORS")
                }.items(),
            )
        ]
    )    
    # # Include the ros2_control launch file
    # ros2_control_ld = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory("champ_simulation"),
    #             "launch",
    #             "ros2_control.launch.py",
    #         )
    #     )
    # )
        
    # Timer action to delay starting the ros2_control and bringup launch files

    # Include the ros2_control launch file after a delay
    ros2_control_ld = TimerAction(
        period=4*delay_duration,  # Delay for 'delay_duration' seconds
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("champ_simulation"),
                        "launch",
                        "ros2_control.launch.py",
                    )
                )
            )
        ]
    )

    # Include the bringup launch file after a delay
    bringup_ld = TimerAction(
        period=6*delay_duration,  # Delay for 'delay_duration' seconds
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("champ_simulation"),
                        "launch",
                        "bringup.launch.py",
                    )
                ),
                launch_arguments={
                    "description_path": default_model_path,
                    "joints_map_path": joints_config,
                    "links_map_path": links_config,
                    "gait_config_path": gait_config,
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "robot_name": LaunchConfiguration("robot_name"),
                    "gazebo": "true",
                    "lite": LaunchConfiguration("lite"),
                    "rviz": LaunchConfiguration("rviz"),
                    "joint_controller_topic": "joint_group_effort_controller/joint_trajectory",
                    "hardware_connected": "false",
                    "publish_foot_contacts": "false",
                    "close_loop_odom": "true",
                }.items(),
            )
        ]
    )
    # Return the LaunchDescription, including all declared launch arguments and included launch files
    return LaunchDescription(
        [
            declare_description_path,
            declare_use_sim_time,

            declare_description_path,
            declare_joints_map_path,
            declare_links_map_path,
            declare_gait_config_path,
            declare_orientation_from_imu,
            declare_robot_name,
            declare_base_link_frame,
            declare_lite,
            declare_rviz,
            declare_gazebo,
            declare_joint_controller_topic,
            declare_hardware_connected,
            declare_publish_joint_control,
            declare_publish_joint_states,
            declare_publish_foot_contacts,
            declare_publish_odom_tf,
            declare_close_loop_odom,
            
            declare_headless,
            declare_gazebo_world,
            declare_robot_name,
            declare_world_init_x,
            declare_world_init_y,
            declare_world_init_z,
            declare_world_init_heading,
            declare_gazebo_config,
            declare_launch_dir,
            
            declare_sensors_flag,
            
            description_ld,
            gazebo_ld,
            ros2_control_ld,
            bringup_ld,
        ]
    )
