import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess, TimerAction, LogInfo, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from datetime import datetime

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'tune', 
        default_value='False', 
        description='Enable autotuning for the controller'
    ))

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
    common_share_dir = Path(get_package_share_directory("tauv_core"))
    common_ekf_file = common_share_dir / "config" / "ekfFUNNY.yaml"

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
            'command_rate_hz': 100.0,
            'discovery_time_sec': 15.0,
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
    recording = ExecuteProcess(            
        cmd=[
            'ros2',
            'bag',
            'record',
            '-a',
            '-s',
            'mcap',
            '-o',
            Path("/tauv-mono/ros_ws/bags") / f"rosbag_osprey_{timestamp}",
        ],
        output='screen',
    )

    ekf = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg="Starting EKF filter node!!!!!!"),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                parameters=[str(common_ekf_file)],
                output="screen",
            ),
        ],
    )

    imu_converter = Node(
        package="tauv_core",
        executable="imu_converter",
        name="imu_converter",
        output="screen",
    )
    depth_converter = Node(
        package="tauv_core",
        executable="depth_converter",
        name="depth_converter",
        output="screen",
    )
    dvl_converter = Node(
        package="tauv_core",
        executable="dvl_converter",
        name="dvl_converter",
        output="screen",
    )
    watchdog_params = {
        'esc_topic': '/esc_telemetry',
        'imu_topic': 'os/sensors/imu_xsens',
        'system_state_topic': 'watchdog/system_state',
        'heartbeat_frequency_hz': 1.0,
        'esc_timeout_s': 1.0,
        'stale_startup_grace_s': 30.0,
        'warning_temperature_c': 70.0,
        'error_temperature_c': 90.0,
        'error_voltage_v': 12.0,
        'roll_threshold_deg': 35.0,
        'pitch_threshold_deg': 35.0,
        'angular_velocity_threshold_radps': 3,
        'expected_esc_ids': [100, 101, 102, 103, 104, 105, 106, 107],
    }

    # watchdog node
    watchdog = Node(
        package="tauv_core",
        executable="watchdog",
        name="watchdog",
        output="screen",
        parameters=[watchdog_params])
    imu_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu',
        arguments=['0', '0', '0', '3.14159', '0', '0', 'os/base_link', 'imu_link_xsens'], # Yaw Pitch Roll
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    depth_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_depth',
        arguments=['0', '0', '0', '0', '0', '0', 'os/base_link', 'depth_link'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    dvl_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_dvl',
        arguments=['0', '0', '0', '-1.5708', '0.0', '3.14159', 'os/base_link', 'dvl_link'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    controller = Node(
        package='tauv_autonomy',
        executable='controller',
        name='controller',
        parameters=[{
            'tune': LaunchConfiguration('tune')
        }],
        output='screen',
    )
    thruster_forces = Node(
        package='tauv_autonomy',
        executable='thruster_forces',
        name='thruster_forces',
        output='screen',
    )
    thruster_rpms = Node(
        package='tauv_autonomy',
        executable='thruster_rpms',
        name='thruster_rpms',
        output='screen',
    )

    # 4. Add Nodes to Launch Description
    ld.add_action(depth_node)
    ld.add_action(dronecan_node)
    ld.add_action(xsens_node)
    ld.add_action(dvl_node)
    ld.add_action(foxglove_bridge)
    ld.add_action(recording)
    ld.add_action(ekf)
    ld.add_action(imu_converter)
    ld.add_action(depth_converter)
    ld.add_action(dvl_converter)
    ld.add_action(watchdog)
    ld.add_action(imu_frame)
    ld.add_action(depth_frame)
    ld.add_action(dvl_frame)
    ld.add_action(controller)
    ld.add_action(thruster_forces)
    ld.add_action(thruster_rpms)

    return ld