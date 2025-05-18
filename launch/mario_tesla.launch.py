from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    
    os.environ["PYTHONPATH"] = "/home/pi/inputs-local:" + os.environ.get("PYTHONPATH", "")
    return LaunchDescription([
        Node(
            package='mario_kart',
            executable='motor_executor',
            name='motor_executor',
            output='screen'
        ),
        Node(
            package='mario_kart',
            executable='button_controller',
            name='button_controller'
        ),
        Node(
            package='mario_kart',
            executable='lights',
            name='lights'
        ),
        Node(
            package='mario_kart',
            executable='aruco_tracker',
            name='aruco_tracker'
        ),
        Node(
            package='mario_kart',
            executable='aruco_distance_processor',
            name='aruco_distance_processor'
        ),
        Node(
            package='mario_kart',
            executable='mario_tesla',
            name='mario_tesla'
        ),
    ])