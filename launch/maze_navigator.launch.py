from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='maze_navigator',
            executable='motor_executor',
            name='motor_executor',
            namespace='control',
        ),
        Node(
            package='maze_navigator',
            executable='sensors_node',
            name='sensors_node',
            namespace='sensing',
        ),
        Node(
            package='maze_navigator',
            executable='sensors_to_motor_commands',
            name='sensors_to_motor_commands',
            namespace='control',
        ),
        Node(
            package='maze_navigator',
            executable='sensors_processor',
            name='sensors_processor',
            namespace='sensing',
        ),
        Node(
            package='maze_navigator',
            executable='dead_reckoning',
            name='dead_reckoning',
            namespace='control',
            ),
    ])