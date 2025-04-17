import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'picar_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
      ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
    glob('launch/picar_launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chen',
    maintainer_email='wujn.johnny020711@gmail.com',
    description='TODO: Package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher = picar_control.camera_publisher:main',
            'lidar_publisher = picar_control.lidar_publisher:main',
            'motor_subscriber = picar_control.motor_subscriber:main',
            'steer_servo_subscriber = picar_control.steer_servo_subscriber:main',
            'camera_servo_subscriber = picar_control.camera_servo_subscriber:main',
            'laser_tracker = picar_control.laser_tracker:main',
            'obstacle_avoidance = picar_control.obstacle_avoidance:main',
        ],
    },
)
