from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument, 
                            OpaqueFunction, ExecuteProcess, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch import LaunchContext

from launch.event_handlers import OnProcessStart
from launch.actions import RegisterEventHandler

from ament_index_python.packages import get_package_share_directory
import os
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro
import re

def get_franka_description(context, arm_id, load_gripper, franka_hand):
    """Generate Franka robot nodes for Gazebo with ros2_control"""

    arm_id_str = context.perform_substitution(arm_id)
    load_gripper_str = context.perform_substitution(load_gripper)
    franka_hand_str = context.perform_substitution(franka_hand)


    # Paths
    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', arm_id_str, arm_id_str + '.urdf.xacro'
    )
    controllers_file = os.path.join(
        get_package_share_directory('franka_bringup'),
        'config', 'controllers.yaml' 
    )

    # Process URDF xacro
    robot_description_config = xacro.process_file(
        franka_xacro_file,
        mappings={
            'arm_id': arm_id_str,
            'hand': load_gripper_str,
            'ros2_control': 'true',
            'gazebo': 'true',          
            'gazebo_effort': 'false',     
            'use_fake_hardware': 'false',  
            'fake_sensor_commands': 'false',
            'robot_ip': '',              
            'ee_id': franka_hand_str
        }
    )

    robot_urdf = robot_description_config.toxml()
    # Remove ALL XML comments 
    robot_urdf = re.sub(r'<!--.*?-->', '', robot_urdf, flags=re.DOTALL)

    # Also remove extra whitespace/newlines created by comment removal
    robot_urdf = re.sub(r'\n\s*\n', '\n', robot_urdf)

    robot_description = {'robot_description': robot_urdf}

    # ----------------------
    # Robot state publisher
    # ----------------------
    franka_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='franka',
        output='screen',
        parameters=[robot_description],
    )


    # Spawn Franka in Gazebo
    # ----------------------
    # Spawn Franka robot
    spawn_franka = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'franka_robot',
            '-topic', '/franka/robot_description',
            '-robot_namespace', '/franka',  # Key: sets namespace
            '-x', '2.1',
            '-y', '-0.5',
            '-z', '1.0',
        ],
        output='screen',
    )

    
    # ----------------------
    # Spawners for joint_state_broadcaster and position controller
    # ----------------------
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c", "/franka/controller_manager",  
            "--controller-manager-timeout", "30" 
        ],
    )

    joint_position_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
        "joint_group_position_controller",
        "--controller-manager",
        "/franka/controller_manager",
    ],
)


    return [
        franka_state_publisher,
        spawn_franka,
        joint_state_broadcaster_spawner,
        joint_position_spawner,
         ]


def generate_launch_description():
    # === Launch arguments for UR5e ===
    ur_type = LaunchConfiguration("ur_type", default="ur5e")
    prefix = LaunchConfiguration("prefix", default='""')
    spawn_x = LaunchConfiguration("spawn_x", default="0.6")
    spawn_y = LaunchConfiguration("spawn_y", default="0.0")
    spawn_z = LaunchConfiguration("spawn_z", default="0.8")
    spawn_R = LaunchConfiguration("spawn_R", default="0")
    spawn_P = LaunchConfiguration("spawn_P", default="0")
    spawn_Y = LaunchConfiguration("spawn_Y", default="0")

    # === Launch arguments for Franka ===
    load_gripper = LaunchConfiguration('load_gripper', default='false')
    franka_hand = LaunchConfiguration('franka_hand', default='franka_hand')
    arm_id = LaunchConfiguration('arm_id', default='fr3')

    # === Launch Gazebo with custom world ===
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch/gazebo.launch.py"]
        ),
        launch_arguments={
            "world": PathJoinSubstitution(
                [FindPackageShare("dual_arm_bringup"), "worlds", "cynlr.world"]
            ),
            "gui": "true",
            "verbose": "false"
        }.items()
    )

    # === Launch UR5e robot ===
    ur5e = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_simulation_gazebo"), "/launch/ur_sim_control.launch.py"]
        ),
        launch_arguments={
            "launch_gazebo": "false",
            "ur_type": ur_type,
            "prefix": prefix,
            "spawn_x": spawn_x,
            "spawn_y": spawn_y,
            "spawn_z": spawn_z,
            "spawn_R": spawn_R,
            "spawn_P": spawn_P,
            "spawn_Y": spawn_Y,
        }.items()
    )

    # CRITICAL: Delay Franka launch until UR completes
    # Wait 10 seconds after Gazebo starts for UR to fully load
    franka_delayed = TimerAction(
        period=10.0,  # 10 seconds delay
        actions=[
            OpaqueFunction(
                function=get_franka_description,
                args=[arm_id, load_gripper, franka_hand]
            )
        ]
    )

    return LaunchDescription([
        # Declare UR5e arguments
        DeclareLaunchArgument("ur_type", default_value="ur5e"),
        DeclareLaunchArgument("prefix", default_value='""'),
        DeclareLaunchArgument("spawn_x", default_value="0.6"),
        DeclareLaunchArgument("spawn_y", default_value="0.0"),
        DeclareLaunchArgument("spawn_z", default_value="0.8"),
        DeclareLaunchArgument("spawn_R", default_value="0"),
        DeclareLaunchArgument("spawn_P", default_value="0"),
        DeclareLaunchArgument("spawn_Y", default_value="0"),

        # Declare Franka arguments
        DeclareLaunchArgument('joint_state_rate', default_value='30', description='Joint state publish rate'),
        DeclareLaunchArgument('load_gripper', default_value='false'),
        DeclareLaunchArgument('franka_hand', default_value='franka_hand'),
        DeclareLaunchArgument('arm_id', default_value='fr3'),

        # Launch order: Gazebo -> UR5e -> Franka
        gazebo,
        ur5e,
        franka_delayed,
    ])
