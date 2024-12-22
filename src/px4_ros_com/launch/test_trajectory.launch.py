import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the RViz configuration file
    rviz_config_file = os.path.join(
        get_package_share_directory('px4_ros_com'),
        'rviz',
        'test_trajectory.rviz'
    )

    print(f"Loading config file from: {rviz_config_file}")

    traj_test_node = Node(
        package='px4_ros_com',
        executable='test_trajectory',
        output='screen',
        shell=True,
    )

    # Node to start RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],  # Pass the RViz config file
        output='screen'
    )
    
    return LaunchDescription([
        traj_test_node,
        rviz_node
    ])