from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dual_arm_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),  # your Python package folder
    data_files=[
        # Resource file for ROS2 indexing
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        
        # package.xml
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

        # Config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

        # URDF/XACRO files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf*')),

        # World files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ashwin',
    maintainer_email='ashwin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_cube = dual_arm_bringup.spawn_cube:main',
            'object_tracker = dual_arm_bringup.object_tracker:main',
            'robot_a_pick_place = dual_arm_bringup.robot_a_pick_place:main',
            'motion_pattern_generator = dual_arm_bringup.motion_pattern_generator:main',
            'franka_color_tracker = dual_arm_bringup.franka_color_tracker:main',
        ],
    },
)

