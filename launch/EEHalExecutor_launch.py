import launch
from launch.substitutions import Command, LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution, FindExecutable, TextSubstitution
from launch import LaunchDescription, LaunchContext, LaunchService
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    hand_name = LaunchConfiguration('hand_name')
    hand_name_launch_arg = DeclareLaunchArgument('hand_name')
    gazebo = LaunchConfiguration('gazebo')
    gazebo_launch_arg = DeclareLaunchArgument('gazebo', default_value='False')
    gdb = LaunchConfiguration('gdb')
    gdb_launch_arg = DeclareLaunchArgument('gdb', default_value='False')
    hal_lib = LaunchConfiguration('hal_lib')
    hal_lib_launch_arg = DeclareLaunchArgument('hal_lib', default_value='ROSEE::DummyHalPlugin')
    matlogger_path = LaunchConfiguration('matlogger_path')
    matlogger_path_launch_arg = DeclareLaunchArgument('matlogger_path', default_value=[os.getenv('HOME'), '/.ROSEE2/matlog/', hand_name, '/'])
    
    return LaunchDescription([
        
        gazebo_launch_arg,
        gdb_launch_arg,
        hal_lib_launch_arg,
        matlogger_path_launch_arg,
        hand_name_launch_arg,
        
        launch_ros.actions.Node(
            package='end_effector',
            executable='EEHalExecutor',
            name='EEHalExecutor',
            output='screen',
            parameters=[
                {'hal_library_name': hal_lib},
                {'matlogger_path': matlogger_path}
            ],
            condition=UnlessCondition(gdb)
        ),
        launch_ros.actions.Node(
            package='end_effector',
            executable='EEHalExecutor',
            name='EEHalExecutor',
            output='screen',
            parameters=[
                {'hal_library_name': hal_lib},
                {'matlogger_path': matlogger_path}
            ],
            prefix=['konsole -e gdb --args'],
            condition=IfCondition(gdb)
        ),
            
        launch_ros.actions.Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[
                {'source_list': ['/dummyHal/joint_command']},
            ],
            remappings=[
                ("joint_states", "/dummyHal/joint_states")
            ],
            condition=UnlessCondition(gazebo)
        ),
    ])

