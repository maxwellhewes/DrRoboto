import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_share = FindPackageShare('two_link_arm_sim')
    
    # Paths to important files
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'two_link_arm.urdf'])
    world_file = PathJoinSubstitution([pkg_share, 'worlds', 'arm_world.world'])
    controller_config = PathJoinSubstitution([pkg_share, 'config', 'arm_controllers.yaml'])
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(os.path.join(
                get_package_share_directory('two_link_arm_sim'),
                'urdf',
                'two_link_arm.urdf'
            )).read()
        }]
    )
    
    # Joint state publisher GUI (optional)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(PythonExpression(['not ', use_sim_time]))
    )
    
    # Gazebo server
    gazebo_server = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_file],
        output='screen'
    )
    
    # Gazebo client
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'two_link_arm',
                  '-topic', '/robot_description',
                  '-x', '0.0',
                  '-y', '0.0',
                  '-z', '0.0'],
        output='screen'
    )
    
    # Load joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    
    # Load arm controller
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        robot_state_publisher,
        joint_state_publisher_gui,
        gazebo_server,
        gazebo_client,
        spawn_robot,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
    ])
