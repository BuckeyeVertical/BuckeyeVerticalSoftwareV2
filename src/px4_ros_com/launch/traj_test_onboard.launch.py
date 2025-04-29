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

    # MicroXRCEAgent (Optional)
    micro_ros_agent = ExecuteProcess(
        cmd=['xterm', '-e', 'MicroXRCEAgent', 'udp4', '-p', '8888', '-v'],
        shell=True,
        output='screen'
    )

    # Trajectory test node (traj_test_copy)
    traj_test_node = Node(
        package='px4_ros_com',
        executable='traj_test',
        output='screen',
        shell=True,
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Gazebo bridge for camera images
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output='screen',
        arguments=['/camera@sensor_msgs/msg/Image@gz.msgs.Image'],
        shell=True
    )
    
    
    # Delayed start of traj_test
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
    
    # Image Stitching Node (PYTHON VERSION)
    image_stitching_node = Node(
        package='px4_ros_com',
        executable='image_stitching_node.py',  # <<-- Python script name
        output='screen'
    )
    
    detection = Node(
        package='px4_ros_com',
        executable='detection.py',  # <<-- Python script name
        output='screen'
    )

    return LaunchDescription([
        # micro_ros_agent,  # Uncomment if you want to launch the Micro XRCE Agent automatically
        traj_test_node,
        rviz_node,
        # gz_bridge,
        image_stitching_node,
        detection
        # <-- Launch the Python image stitcher
    ])
