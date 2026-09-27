from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory('arbotrix_sim')

    world_file = os.path.join(
        pkg_share,
        'worlds',
        'indoor_farmhouse.sdf'
    )

    xacro_file = os.path.join(
        pkg_share,
        'urdf',
        'arbotrix_bot.urdf.xacro'
    )

    robot_description = Command([
        'xacro ',
        xacro_file
    ])

    gazebo = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            world_file,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'robot_description': robot_description
            }
        ]
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'arbotrix_bot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '8.0',
            '-z', '0.05'
        ],
        output='screen'
    )

    delayed_spawn = TimerAction(
        period=3.0,
        actions=[spawn_robot]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[
            {
                'use_sim_time': True
            }
        ]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        delayed_spawn,
        rviz
    ])