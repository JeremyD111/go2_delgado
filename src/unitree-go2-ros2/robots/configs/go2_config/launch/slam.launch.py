import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # -----------------------------------------------------------------------
    # 1. DEFINICIÓN DE RUTAS (Todo ocurre aquí, sin intermediarios)
    # -----------------------------------------------------------------------
    
    # Tu paquete donde está el YAML
    go2_config_pkg = FindPackageShare('go2_config')
    
    # Paquetes del sistema necesarios
    slam_toolbox_pkg = FindPackageShare('slam_toolbox')
    nav2_bringup_pkg = FindPackageShare('nav2_bringup')
    champ_nav_pkg = FindPackageShare('champ_navigation') # Solo para robarle la config de RViz si quieres

    # Ruta directa a TU archivo de parámetros corregido
    slam_params_file = PathJoinSubstitution(
        [go2_config_pkg, 'config/autonomy', 'slam.yaml']
    )

    # Rutas a los launch files oficiales
    slam_launch_path = PathJoinSubstitution(
        [slam_toolbox_pkg, 'launch', 'online_async_launch.py']
    )
    
    nav2_launch_path = PathJoinSubstitution(
        [nav2_bringup_pkg, 'launch', 'navigation_launch.py']
    )

    # Ruta a la configuración visual de RViz
    rviz_config_path = PathJoinSubstitution(
        [champ_nav_pkg, 'rviz', 'slam.rviz']
    )

    # -----------------------------------------------------------------------
    # 2. DECLARACIÓN DE ARGUMENTOS
    # -----------------------------------------------------------------------
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_toggle = LaunchConfiguration('rviz')

    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Open RViz2 automatically'
    )

    # -----------------------------------------------------------------------
    # 3. INCLUSIÓN DE NODOS Y LAUNCHES
    # -----------------------------------------------------------------------

    # A) Lanzar Nav2 (Planner, Controller, Recoveries)
    # Nota: Nav2 es opcional si solo quieres mapear, pero útil si quieres mover el robot autónomamente mientras mapeas.
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_path),
        launch_arguments={
            'use_sim_time': use_sim_time,
            # 'params_file': ... (Podrías agregar tu propio nav2_params.yaml aquí en el futuro)
        }.items()
    )

    # B) Lanzar SLAM Toolbox (El núcleo del mapeo)
    # AQUÍ es donde la magia ocurre: Pasamos TU archivo directamente.
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file
        }.items()
    )

    # C) Lanzar RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(rviz_toggle),
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # -----------------------------------------------------------------------
    # 4. RETORNO DE LA DESCRIPCIÓN
    # -----------------------------------------------------------------------
    
    return LaunchDescription([
        declare_sim_time,
        declare_rviz,
        nav2_launch,
        slam_launch,
        rviz_node
    ])