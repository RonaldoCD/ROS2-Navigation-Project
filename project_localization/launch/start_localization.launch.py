import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    maps_dir = os.path.join(get_package_share_directory('project_mapping'), 'maps')
    map_file = os.path.join(maps_dir, 'turtlebot_area_1cm.yaml')
    config_dir = os.path.join(get_package_share_directory('project_localization'), 'config')
    amcl_yaml = os.path.join(config_dir, 'amcl_config.yaml')
    rviz_dir = os.path.join(get_package_share_directory('project_localization'), 'rviz')
    rviz_file = os.path.join(rviz_dir, 'localization_rviz_config.rviz')

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'yaml_filename':map_file}]),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_yaml]),
        
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server','amcl']}]),
        
        Node(
            package='project_localization',
            executable='spot_recorder',
            name='spot_recorder',
            output='screen'),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(rviz_dir, rviz_file)]) 
    ]) 