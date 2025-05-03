from setuptools import find_packages, setup
from glob import glob

package_name = 'mario_kart'

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
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "motor_commander = mario_kart.motor_commander:main",
            "motor_executor = mario_kart.motor_executor:main",
            "servo_commander = mario_kart.servo_commander:main",
            "servo_executor = mario_kart.servo_executor:main",
            "controller = mario_kart.controller:main",
            "controller_processor = mario_kart.controller_processor:main",
            "driver = mario_kart.driver:main",
            "button_controller = mario_kart.button_controller:main",
        ],
    },
)
