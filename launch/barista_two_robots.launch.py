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
    
    # Configure Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ])
    )
    
    # First, create a common reference frame - the world/map frame
    map_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
    )
    
    # Define robot names and positions
    rick_name = "rick"
    rick_x, rick_y = 1.0, 0.0
    
    morty_name = "morty"
    morty_x, morty_y = -1.0, 0.0
    
    # RICK (RED) ROBOT
    rick_xacro_file = os.path.join(pkg_dir, 'xacro', 'barista_robot_model.urdf.xacro')
    
    # Rick's robot state publisher
    rick_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=rick_name,
        parameters=[{
            'frame_prefix': rick_name + '/', 
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', rick_xacro_file, ' include_laser:=true', ' robot_color:=Red'])
        }],
        output="screen"
    )
    
    # Rick's joint state publisher
    rick_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=rick_name,
        output="screen"
    )
    
    # Position Rick's odom frame correctly in the map frame
    map_to_rick_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_rick_odom',
        arguments=[str(rick_x), str(rick_y), '0', '0', '0', '0', 'map', f'{rick_name}/odom']
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
    
    # Morty's robot state publisher
    morty_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=morty_name,
        parameters=[{
            'frame_prefix': morty_name + '/',
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', morty_xacro_file, ' include_laser:=true', ' robot_color:=Blue'])
        }],
        output="screen"
    )
    
    # Morty's joint state publisher
    morty_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=morty_name,
        output="screen"
    )
    
    # Position Morty's odom frame correctly in the map frame
    map_to_morty_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_morty_odom',
        arguments=[str(morty_x), str(morty_y), '0', '0', '0', '0', 'map', f'{morty_name}/odom']
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
    
    # Return the launch description
    return LaunchDescription([
        gazebo,
        map_frame,
        rick_robot_state_publisher,
        rick_joint_state_publisher,
        map_to_rick_odom,
        spawn_rick,
        morty_robot_state_publisher,
        morty_joint_state_publisher,
        map_to_morty_odom,
        spawn_morty,
        rviz
    ])