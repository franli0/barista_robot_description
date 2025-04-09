import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('barista_robot_description')
    
    # Use simulation time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Configure Gazebo with an empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ])
    )
    
    # First, create a common reference frame - the world/map frame
    world_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
    )
    
    # Define robot names and positions
    rick_name = "rick"
    rick_x, rick_y = -1.0, 0.0
    
    morty_name = "morty"
    morty_x, morty_y = 1.0, 0.0
    
    # Position Rick's odom frame correctly in the map frame
    map_to_rick_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_rick_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', f'{rick_name}/odom']
    )
    
    # Position Morty's odom frame correctly in the map frame
    map_to_morty_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_morty_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', f'{morty_name}/odom']
    )
    
    # RICK (RED) ROBOT
    rick_xacro_file = os.path.join(pkg_dir, 'xacro', 'barista_robot_model.urdf.xacro')
    
    # Process Rick's XACRO file with explicit parameters
    rick_robot_description_content = Command(['xacro ', rick_xacro_file, 
                                             ' robot_name:=', rick_name, 
                                             ' include_laser:=true', 
                                             ' robot_color:=Red'])
    
    # Rick's robot state publisher
    rick_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=rick_name,
        parameters=[{
            'frame_prefix': rick_name + '/',
            'use_sim_time': use_sim_time,
            'robot_description': rick_robot_description_content
        }],
        output="screen"
    )
    
    # Rick's joint state publisher
    rick_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=rick_name,
        parameters=[{'use_sim_time': use_sim_time}],
        output="screen"
    )
    
    # Spawn Rick robot in Gazebo
    spawn_rick = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', rick_name,
            '-topic', f'/{rick_name}/robot_description',
            '-x', str(rick_x),
            '-y', str(rick_y),
            '-z', '0.1127',
            '-robot_namespace', rick_name
        ],
        output="screen"
    )
    
    # MORTY (BLUE) ROBOT
    morty_xacro_file = os.path.join(pkg_dir, 'xacro', 'barista_robot_model.urdf.xacro')
    
    # Process Morty's XACRO file with explicit parameters
    morty_robot_description_content = Command(['xacro ', morty_xacro_file, 
                                              ' robot_name:=', morty_name, 
                                              ' include_laser:=true', 
                                              ' robot_color:=Blue'])
    
    # Morty's robot state publisher
    morty_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=morty_name,
        parameters=[{
            'frame_prefix': morty_name + '/',
            'use_sim_time': use_sim_time,
            'robot_description': morty_robot_description_content
        }],
        output="screen"
    )
    
    # Morty's joint state publisher
    morty_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=morty_name,
        parameters=[{'use_sim_time': use_sim_time}],
        output="screen"
    )
    
    # Spawn Morty robot in Gazebo
    spawn_morty = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', morty_name,
            '-topic', f'/{morty_name}/robot_description',
            '-x', str(morty_x),
            '-y', str(morty_y),
            '-z', '0.1127',
            '-robot_namespace', morty_name
        ],
        output="screen"
    )
    
    # Create a custom RViz config file that will show both robots
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'two_robots_config.rviz')
    
    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output="screen"
    )
    
    return LaunchDescription([
        gazebo,
        world_to_map,
        map_to_rick_odom,
        map_to_morty_odom,
        rick_robot_state_publisher,
        rick_joint_state_publisher,
        spawn_rick,
        morty_robot_state_publisher,
        morty_joint_state_publisher,
        spawn_morty,
        rviz
    ])