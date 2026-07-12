from setuptools import find_packages, setup
import os
import glob
package_name = 'tauv_servo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aayan',
    maintainer_email='aamahesh@andrew.cmu.edu',
    description='Task-driven ROS 2 driver for Tartan-AUV Hitec MDB961WP-CAN servos.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 'executable_name = package_name.file_name:function_name'
            'servo_driver = tauv_servo.servo_driver:main',
            'find_setpoints = tauv_servo.find_setpoints:main',
        ],
    },
)
