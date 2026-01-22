from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    urdf_file_path = os.path.join(get_package_share_directory('my_robot_description'), 'urdf', 'main.xacro')
    rviz_config_path = os.path.join(get_package_share_directory('my_robot_description'), 'rviz', 'urdf_config.rviz')


    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', urdf_file_path])}]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],  
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])