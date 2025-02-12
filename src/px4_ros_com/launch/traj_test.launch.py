import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the RViz configuration file
    rviz_config_file = os.path.join(
        get_package_share_directory('px4_ros_com'),
        'rviz',
        'test_trajectory.rviz'
    )

    print(f"Loading config file from: {rviz_config_file}")

    micro_ros_agent = ExecuteProcess(
        cmd=['xterm', '-e', 'MicroXRCEAgent', 'udp4', '-p', '8888', '-v'],
        shell=True,
        output='screen'
    )

    # Node to start RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],  # Pass the RViz config file
        output='screen'
    )

    delay_timer = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='px4_ros_com',
                executable='traj_test',
                output='screen',
                shell=True,
            ),
        ]
    )
                

    return LaunchDescription([
        micro_ros_agent,
        delay_timer,
        rviz_node
    ])