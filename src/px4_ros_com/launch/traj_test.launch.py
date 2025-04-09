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

    traj_test_node = Node(
        package='px4_ros_com',
        executable='traj_test_copy',
        output='screen',
        shell=True,
    )

    # Node to start RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output='screen',
        arguments=['/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image'],
        shell=True
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
    
    # Add your modular C++ detection node
    detection_node = Node(
        package='px4_ros_com',       # Update if your detection node is in another package.
        executable='detection_node', # This should match the executable name from your C++ build.
        output='screen'
    )

    return LaunchDescription([
        # micro_ros_agent,
        traj_test_node,
        rviz_node,
        gz_bridge,
        detection_node,  # Added detection node here.
 
    ])
