import os
import launch_ros
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    gazebo_pkg = launch_ros.substitutions.FindPackageShare(
        package="gazebo"
    ).find("gazebo")
    
    default_rviz_path = os.path.join(gazebo_pkg, "rviz/urdf_viewer.rviz")
    declare_rviz_path = DeclareLaunchArgument(name="rviz_path", default_value=default_rviz_path, description="Absolute path to rviz file")
    declare_rviz = DeclareLaunchArgument("rviz", default_value="false", description="Launch rviz")
    declare_rviz_ref_frame = DeclareLaunchArgument("rviz_ref_frame", default_value="odom", description="Rviz ref frame")

    rviz2 = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration("rviz_path")],
        condition=IfCondition(LaunchConfiguration("rviz"))
    )

    return LaunchDescription(
        [
            declare_rviz_path,
            declare_rviz,
            declare_rviz_ref_frame,
            rviz2
        ]
    )
