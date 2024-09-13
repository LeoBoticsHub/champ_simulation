import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
# import launch_ros
from launch_ros.actions import Node

# Function to generate the launch description
def generate_launch_description():

    # robot_description_pkg = launch_ros.substitutions.FindPackageShare(package="go1_description").find("go1_description")
    # ros_control_config = os.path.join(robot_description_pkg, "config/ros_control/ros_control.yaml")

    # ExecuteProcess to load and activate the joint trajectory effort controller
    # This command uses the `ros2 control load_controller` command to load a specific controller
    # ('joint_group_effort_controller') and set its state to 'active'.
    # The output of the command is displayed on the screen.
    load_joint_trajectory_effort_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_group_effort_controller'],
        output='screen'
    )     

    # Return the LaunchDescription, which includes the process to load the controller
    return LaunchDescription(
        [
            # Node(
            #     package='controller_manager',
            #     executable='ros2_control_node',
            #     name='controller_manager',
            #     output='screen',
            #     parameters=[ros_control_config]
            # ),
            load_joint_trajectory_effort_controller
        ]
    )
