import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Rutas
    pkg_share = get_package_share_directory('go2_planner')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'planner.rviz')
    map_file = os.path.join(pkg_share, 'maps', 'map.yaml')
    
    return LaunchDescription([
        # Nodo 1: Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, {'yaml_filename': map_file}]
        ),

        # Nodo 2: Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_planner',
            output='screen',
            parameters=[{'use_sim_time': True}, {'autostart': True}, {'node_names': ['map_server']}]
        ),

        # Nodo 3: Transformación Estática (Map -> Odom)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_bridge',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': True}]
        ),

        # Nodo 4: Planificador Global (Dijkstra)
        Node(
            package='go2_planner',
            executable='planner_node',
            name='planner_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # Nodo 5: Controlador Local (PID) - ¡EL NUEVO INTEGRANTE!
        Node(
            package='go2_planner',
            executable='pid_controller',
            name='pid_controller_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # Nodo 6: RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )
    ])
