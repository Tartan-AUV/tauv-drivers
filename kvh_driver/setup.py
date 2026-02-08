from setuptools import setup
import os
from glob import glob

package_name = 'kvh_gyroscope'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # If you have launch files later, include them here:
        # (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.com',
    description='Driver for KVH Fiber Optic Gyroscope ported to ROS 2',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # executable_name = package_name.file_name:main_function
            'gyro_node = kvh_gyroscope.gyro_node:main'
        ],
    },
)