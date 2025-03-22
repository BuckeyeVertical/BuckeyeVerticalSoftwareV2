import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the RViz configuration file
    rviz_config_file = os.path.join(
        get_package_share_directory('px4_ros_com'),  # Replace with your package name if different
        'rviz',
        'test_trajectory.rviz'
    )

    print(f"Loading RViz config file from: {rviz_config_file}")

    # Start the Micro XRCE-DDS Agent in a new terminal
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

    # Delayed execution of the offboard_control node
    delay_timer = TimerAction(
        period=4.0,  # Delay in seconds
        actions=[
            Node(
                package='px4_ros_com',  # Replace with your package name if different
                executable='offboard_control',  # Name of the compiled executable
                name='offboard_control',  # Name of the node
                output='screen',
                shell=True,
            ),
        ]
    )

    # Return the LaunchDescription object
    return LaunchDescription([
        #micro_ros_agent,  # Start the Micro XRCE-DDS Agent
        rviz_node,        # Start RViz
        delay_timer       # Start the offboard_control node after a delay
    ])