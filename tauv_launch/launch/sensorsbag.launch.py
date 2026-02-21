import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess
from launch_ros.actions import Node
from datetime import datetime

def generate_launch_description():
    ld = LaunchDescription()

    # 1. Global Logging Settings for Jetson Orin Performance
    # Forces logs to stdout and buffers them to save CPU cycles
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'))
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))

    # Get files
    xsens_params_path = os.path.join(
        get_package_share_directory('xsens_mti_ros2_driver'),
        'param',
        'xsens_mti_node.yaml'
    )
    dronecan_db_path = os.path.join(
        get_package_share_directory('tauv_dronecan'),
        'dronecan_dna.db'
    )

    # 3. Define Nodes

    # Bar02 Depth Sensor
    depth_node = Node(
        package='tauv_depth',
        executable='depth',
        name='depth_sensor',
        output='screen',
        parameters=[{'i2c_bus': 7}]
    )

    # DroneCAN Driver
    dronecan_node = Node(
        package='tauv_dronecan',
        executable='can_driver',
        name='dronecan',
        output='screen',
        parameters=[{
            'interface': 'can1',
            'node_id': 12,
            'bitrate': 1000000,
            'esc_count': 8,
            'command_rate_hz': 50.0,
            'discovery_time_sec': 5.0,
            'dna_db_path': dronecan_db_path
        }]
    )

    # Xsens IMU Node
    xsens_node = Node(
        package='xsens_mti_ros2_driver',
        executable='xsens_mti_node',
        name='xsens_mti_node',
        output='screen',
        parameters=[xsens_params_path]
    )
    dvl_node = Node(
        package='dvl_a50',
        executable='dvl_a50_sensor', 
        name='dvl_a50',
        output='screen',
        parameters=[{'dvl_ip_address': '192.168.8.114'}]
    )
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0' 
        }]
    )
    timestamp = datetime.now().strftime('%Y.%m.%d_%H.%M.%S')
    bag_name = f"osprey_{timestamp}"
    recording = ExecuteProcess(            
        cmd=[
            'ros2',
            'bag',
            'record',
            '-a',
            '-s',
            'mcap',
            '-o',
            Path("/tauv-mono/ros_ws/bags") / f"rosbag_{bag_name}",
        ],
        output='screen',
    )

    # 4. Add Nodes to Launch Description
    ld.add_action(depth_node)
    ld.add_action(dronecan_node)
    ld.add_action(xsens_node)
    ld.add_action(dvl_node)
    ld.add_action(foxglove_bridge)
    ld.add_action(recording)

    return ld