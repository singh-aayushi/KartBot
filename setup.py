from setuptools import find_packages, setup
from glob import glob

package_name = 'maze_navigator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "motor_commander = maze_navigator.motor_commander:main",
            "motor_executor = maze_navigator.motor_executor:main",
            "sensors_node = maze_navigator.sensors_node:main",
            "sensors_processor = maze_navigator.sensors_processor:main",
            "sensors_to_motor_commands = maze_navigator.sensors_to_motor_commands:main",
            "dead_reckoning = maze_navigator.dead_reckoning:main",
        ],
    },
)
