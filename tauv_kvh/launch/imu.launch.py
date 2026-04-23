from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/kvh1775',
            description='Serial device path for the KVH 1775 IMU',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='imu_link',
            description='ROS frame ID to stamp on published messages',
        ),
        Node(
            package='tauv_kvh',
            executable='kvh_node',
            name='kvh_imu',
            output='screen',
            parameters=[{
                'port':     LaunchConfiguration('port'),
                'frame_id': LaunchConfiguration('frame_id'),
                # Covariance from KVH 1775 datasheet noise specs:
                #   ARW  0.05 deg/sqrt(hr) -> 1.45e-5 rad/s/sqrt(Hz)
                #   Accel ~50 ug/sqrt(Hz)  -> 4.90e-4 m/s^2/sqrt(Hz)
                # Variance = noise_density^2 * bandwidth_hz (500 Hz default)
                'gyro_noise_density':  1.45e-5,
                'accel_noise_density': 4.90e-4,
                'bandwidth_hz':        500.0,
            }],
        ),
    ])
