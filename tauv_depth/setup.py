from setuptools import find_packages, setup

package_name = 'tauv_depth'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    # packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='ROS 2 driver for the Blue Robotics Bar02 (MS5837-02BA) depth/pressure sensor.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'depth = tauv_depth.depth:main', 

        ],
    },
)
