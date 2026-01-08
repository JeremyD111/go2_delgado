import os
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    # 1. Recuperamos el nombre del mundo y la configuración de ros_control
    world_selection = LaunchConfiguration("world").perform(context)
    
    # 2. Definimos rutas base
    config_pkg_share = launch_ros.substitutions.FindPackageShare(package="go2_config").find("go2_config")
    descr_pkg_share = launch_ros.substitutions.FindPackageShare(package="go2_description").find("go2_description")
    
    # 3. Mapa de mundos con las coordenadas que definimos previamente
    world_map = {
        "bookstore": {
            "path": os.path.join(config_pkg_share, "worlds/bookstore/bookstore.world"),
            "x": "0.0", "y": "0.0", "z": "0.3"
        },
        "factory": {
            "path": os.path.join(config_pkg_share, "worlds/factory/factory.world"),
            "x": "0.0", "y": "0.0", "z": "0.4" # Elevado por seguridad en factory
        },
        "small_house": {
            "path": os.path.join(config_pkg_share, "worlds/small_house/small_house.world"),
            "x": "1.0", "y": "2.0", "z": "0.6"
            #"0.3" # Coordenadas personalizadas
        },
        "office": {
            "path": os.path.join(config_pkg_share, "worlds/office/office.world"),
            "x": "9.0", "y": "4.0", "z": "0.3" # Coordenadas personalizadas
        },
        "default": {
            "path": os.path.join(config_pkg_share, "worlds/default.world"),
            "x": "0.0", "y": "0.0", "z": "0.3"
        },
        "playground": {
            "path": os.path.join(config_pkg_share, "worlds/playground.world"),
            "x": "0.0", "y": "0.0", "z": "0.3"
        },
        "outdoor": {
            "path": os.path.join(config_pkg_share, "worlds/outdoor.world"),
            "x": "0.0", "y": "0.0", "z": "0.3"
        }
    }

    # Selección del mundo (usa default si no encuentra el nombre)
    selected = world_map.get(world_selection, world_map["default"])

    # 4. Rutas de configuración del robot
    joints_config = os.path.join(config_pkg_share, "config/joints/joints.yaml")
    gait_config = os.path.join(config_pkg_share, "config/gait/gait.yaml")
    links_config = os.path.join(config_pkg_share, "config/links/links.yaml")
    
    # NOTA: Mantenemos tu archivo xacro específico (robot_VLP.xacro)
    default_model_path = os.path.join(descr_pkg_share, "xacro/robot_VLP.xacro")

    # 5. Configuración de Bringup (CHAMP)
    bringup_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("champ_bringup"),
                "launch",
                "bringup.launch.py",
            )
        ),
        launch_arguments={
            "description_path": default_model_path,
            "joints_map_path": joints_config,
            "links_map_path": links_config,
            "gait_config_path": gait_config,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "robot_name": LaunchConfiguration("robot_name"),
            "gazebo": "true",
            "lite": LaunchConfiguration("lite"),
            "rviz": LaunchConfiguration("rviz"),
            "joint_controller_topic": "joint_group_effort_controller/joint_trajectory",
            "hardware_connected": "false",
            "publish_foot_contacts": "false",
            "close_loop_odom": "true",
        }.items(),
    )

    # 6. Configuración de Gazebo (CHAMP) con coordenadas inyectadas
    gazebo_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("champ_gazebo"),
                "launch",
                "gazebo.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "robot_name": LaunchConfiguration("robot_name"),
            "world": selected["path"], # Ruta automática
            "lite": LaunchConfiguration("lite"),
            # Coordenadas automáticas desde el diccionario:
            "world_init_x": selected["x"],
            "world_init_y": selected["y"],
            "world_init_z": selected["z"],
            "world_init_heading": LaunchConfiguration("world_init_heading"),
            "gui": LaunchConfiguration("gui"),
            "close_loop_odom": "true",
        }.items(),
    )

    return [bringup_ld, gazebo_ld]

def generate_launch_description():
    # Definir ruta por defecto para ros_control (necesario para el argumento)
    config_pkg_share = launch_ros.substitutions.FindPackageShare(package="go2_config").find("go2_config")
    ros_control_config = os.path.join(config_pkg_share, "/config/ros_control/ros_control.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation (Gazebo) clock if true"),
            DeclareLaunchArgument("rviz", default_value="false", description="Launch rviz"),
            DeclareLaunchArgument("robot_name", default_value="go2", description="Robot name"),
            DeclareLaunchArgument("lite", default_value="false", description="Lite"),
            DeclareLaunchArgument("ros_control_file", default_value=ros_control_config, description="Ros control config path"),
            DeclareLaunchArgument("gui", default_value="true", description="Use gui"),
            
            # Argumento world simplificado (acepta nombres como 'factory', 'office')
            DeclareLaunchArgument("world", default_value="default", description="World name: default, factory, office, small_house, bookstore, playground, outdoor"),
            
            # Mantenemos heading por si quieres rotar el robot manualmente al inicio
            DeclareLaunchArgument("world_init_heading", default_value="0.0"),

            # Ejecución de la lógica opaca
            OpaqueFunction(function=launch_setup)
        ]
    )
