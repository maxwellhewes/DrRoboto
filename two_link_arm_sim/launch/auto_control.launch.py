import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_share = FindPackageShare('two_link_arm_sim')
    
    # Path to URDF file
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'two_link_arm.urdf'])
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Read the URDF file
    with open(os.path.join(get_package_share_directory('two_link_arm_sim'), 'urdf', 'two_link_arm.urdf'), 'r') as urdf:
        robot_description = urdf.read()
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )
    
    # Simple joint controller (NO GUI - this will control the joints)
    simple_controller = Node(
        package='two_link_arm_sim',
        executable='simple_joint_controller.py',
        name='simple_joint_controller',
        output='screen'
    )
    
    # RViz
    rviz_config_file = PathJoinSubstitution([pkg_share, 'config', 'arm_display.rviz'])
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        
        robot_state_publisher,
        simple_controller,
        rviz,
    ])