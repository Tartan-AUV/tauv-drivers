from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        'port': ('/dev/kvh1775', 'KVH serial device'),
        'baud_rate': ('460800', 'Host baud; must match the separately configured IMU'),
        'frame_id_gyro': ('imu_link_fog_gyro', 'Gyro frame and gravity-projection IMU frame'),
        'frame_id_accel_x': ('imu_link_fog_accel_x', 'Accelerometer X frame'),
        'frame_id_accel_y': ('imu_link_fog_accel_y', 'Accelerometer Y frame'),
        'frame_id_accel_z': ('imu_link_fog_accel_z', 'Accelerometer Z frame'),
        'sample_queue_capacity': ('1000', 'Buffered samples before oldest-sample eviction'),
        'publisher_depth': ('20', 'ROS publisher history depth per topic'),
        # Preserve the previous launch covariance; tune against actual device filtering separately.
        'bandwidth_hz': ('500.0', 'Bandwidth used for measurement covariance'),
    }
    types = {'baud_rate': int, 'sample_queue_capacity': int, 'publisher_depth': int,
             'bandwidth_hz': float}
    arguments = [DeclareLaunchArgument(name, default_value=value, description=description)
                 for name, (value, description) in defaults.items()]
    parameters = {name: ParameterValue(LaunchConfiguration(name), value_type=types.get(name, str))
                  for name in defaults}
    parameters.update({'gyro_noise_density': 1.45e-5, 'accel_noise_density': 4.90e-4})
    return LaunchDescription(arguments + [Node(
        package='tauv_kvh', executable='kvh_node', name='kvh_imu',
        output='screen', parameters=[parameters],
    )])
