import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    config_dir = os.path.join(get_package_share_directory('project_path_planning'), 'config')
    
    planner_yaml = os.path.join(config_dir, 'path_planner.yaml')
    controller_yaml = os.path.join(config_dir, 'controller.yaml')
    bt_navigator_yaml = os.path.join(config_dir, 'bt_navigator.yaml')
    recovery_yaml = os.path.join(config_dir, 'recovery.yaml')

    return LaunchDescription([
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml]),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml]),
        
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            parameters=[recovery_yaml],
            output='screen'),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml]),
        
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_path_planning',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'bond_timeout':0.0},
                        {'node_names': ['planner_server',
                                        'controller_server',
                                        'recoveries_server',
                                        'bt_navigator']}])
        ]) 