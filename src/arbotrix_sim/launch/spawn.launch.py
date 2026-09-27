from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():

    gazebo = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            '/home/shahzaib/arbotrix_ws/src/arbotrix_sim/worlds/arbotrix_room.world',
            '-s',
            'libgazebo_ros_init.so',
            '-s',
            'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([
        gazebo,
        rviz,
    ])