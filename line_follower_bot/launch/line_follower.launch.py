import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'line_follower_bot'
    pkg_share = get_package_share_directory(package_name)
    
    world_path = os.path.join(pkg_share, 'world', 'line_follower.world')
    urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf')

    return LaunchDescription([
        # Launch Gazebo with specified world and ROS factory plugin
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Launch robot state publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'rsp.launch.py')
            ),
            launch_arguments={'use_sim_time': 'true'}.items()
        ),

        # Spawn entity with proper synchronization
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=ExecuteProcess(
                    cmd=['dummy'],  # Workaround for event handler
                    output='screen'
                ),
                on_start=[
                    TimerAction(
                        period=5.0,
                        actions=[
                            Node(
                                package='gazebo_ros',
                                executable='spawn_entity.py',
                                arguments=[
                                    '-file', urdf_path,
                                    '-entity', 'my_bot'
                                ],
                                output='screen'
                            )
                        ]
                    )
                ]
            )
        )
    ])