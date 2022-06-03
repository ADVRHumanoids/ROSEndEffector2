import launch
from launch.substitutions import Command, LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution, FindExecutable, TextSubstitution
from launch import LaunchDescription, LaunchContext, LaunchService
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix



def launchconfiguration_to_string(context: LaunchContext, launchConfig):
    return context.perform_substitution(launchConfig)
    

def generate_launch_description():
    
    rosee_pkg = launch_ros.substitutions.FindPackageShare(package='end_effector')
    
    #hand_name = LaunchConfiguration('hand_name')
    #hand_name_launch_arg = DeclareLaunchArgument('hand_name')
    hand_name = os.getenv('HAND_NAME')
    
    if not hand_name :
        print(f'Please provide hand name as environment variable: "export HAND_NAME=hand_name"')
        return 
    
    #Necessary because lib are in subfolder which is not found
    ld_library_path = os.getenv('LD_LIBRARY_PATH')
    ld_library_path += ':'
    ld_library_path += get_package_prefix('end_effector')
    ld_library_path += '/lib/end_effector'
    os.environ['LD_LIBRARY_PATH'] = str(ld_library_path)
    
    hand_name_urdf = hand_name + ".urdf"
    hand_name_srdf = hand_name + ".srdf"
    
    urdf_file_name = 'test_ee.urdf'
    srdf_file_name = 'test_ee.srdf'
    
    gdb = LaunchConfiguration('gdb')
    gdb_launch_arg = DeclareLaunchArgument('gdb', default_value='False')
    gazebo = LaunchConfiguration('gazebo')
    gazebo_launch_arg = DeclareLaunchArgument('gazebo', default_value='False')
    hal_lib = LaunchConfiguration('hal_lib')
    hal_lib_launch_arg = DeclareLaunchArgument('hal_lib', default_value='ROSEE::DummyHalPlugin')
    actions_folder_path = LaunchConfiguration('actions_folder_path')
    actions_folder_path_launch_arg = DeclareLaunchArgument('actions_folder_path', default_value=[os.getenv('HOME'), '/.ROSEE2/actions/', hand_name, '/'])
    matlogger_path = LaunchConfiguration('matlogger_path')
    matlogger_path_launch_arg = DeclareLaunchArgument('matlogger_path', default_value=[os.getenv('HOME'), '/.ROSEE2/matlogs/', hand_name, '/'])
    
    urdf_path = os.path.join(
        get_package_share_directory('end_effector'),
        "configs",
        "urdf",
        hand_name_urdf)
    srdf_path = os.path.join(
        get_package_share_directory('end_effector'),
        "configs",
        "srdf",
        hand_name_srdf)
    
    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()
        
    with open(srdf_path, 'r') as infp:
        robot_description_semantic = infp.read()

            
    return LaunchDescription([
        
        #hand_name_launch_arg,
        gdb_launch_arg,
        gazebo_launch_arg,
        hal_lib_launch_arg,
        actions_folder_path_launch_arg,
        matlogger_path_launch_arg,
        
        launch_ros.actions.Node(
            package='end_effector',
            executable='UniversalRosEndEffector',
            name='UniversalRosEndEffector',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                {'robot_description_semantic': robot_description_semantic},
                {'urdf_path': urdf_path},
                {'srdf_path': srdf_path},
                {'actions_folder_path': actions_folder_path},
                {'primitive_aggregated_srv_name': "primitives_aggregated_available"},
                {'selectable_finger_pair_info': "selectable_finger_pair_info"},
                {'grasping_action_srv_name': "grasping_actions_available"},
                {'hand_info': "hand_info"},
                {'new_grasping_action_srv_name': "new_generic_grasping_action"},
                {'rosAction_grasping_command': "action_command"},
                {'rate': 100.0},
            ],
        ),
        
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    rosee_pkg,
                    'launch',
                    'EEHalExecutor_launch.py'
                ])
            ]),
            launch_arguments={
                'gazebo': gazebo,
                'hand_name': hand_name,
                'hal_lib': hal_lib,
                'gdb': gdb,
                'matlogger_path': matlogger_path,
            }.items()
        ),
        
        
        ############     <!-- NOT NEEDED FOR TESTS; BUT uncomment to debug the test itself on local machine -->
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                {'publish_frequency': 100.0}
            ],
            remappings=[
                ("joint_states", "/dummyHal/joint_states")
            ]
        ),
            
        #launch_ros.actions.Node(
            #package='rviz2',
            #executable='rviz2',
            #name='rviz2',
            #output='screen',
            #arguments=['-d', [rosee_pkg, '/configs/rviz/', hand_name, '.rviz']],
        #)

    ])
