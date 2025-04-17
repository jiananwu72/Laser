from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='picar_control',
            executable='camera_publisher',
            name='camera_publisher',
            output='screen'
        ),
        Node(
            package='picar_control',
            executable='lidar_publisher',
            name='lidar_publisher',
            output='screen'
        ),
        Node(
            package='picar_control',
            executable='laser_tracker',
            name='laser_tracker',
            output='screen'
        ),
        Node(
            package='picar_control',
            executable='obstacle_avoidance',
            name='obstacle_avoidance',
            output='screen'
        ),
        Node(
            package='picar_control',
            executable='motor_subscriber',
            name='motor_subscriber',
            output='screen'
        ),
        Node(
            package='picar_control',
            executable='steer_servo_subscriber',
            name='steer_servo_subscriber',
            output='screen'
        ),
        Node(
            package='picar_control',
            executable='camera_servo_subscriber',
            name='camera_servo_subscriber',
            output='screen'
        ),
    ])
