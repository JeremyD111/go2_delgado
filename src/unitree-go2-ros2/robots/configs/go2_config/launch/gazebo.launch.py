import os
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    # Recuperamos el nombre del mundo enviado por terminal
    world_selection = LaunchConfiguration("world").perform(context)
    config_pkg_share = launch_ros.substitutions.FindPackageShare(package="go2_config").find("go2_config")
    
    # Mapa extendido con los nuevos mundos y sus coordenadas específicas
    world_map = {
        "bookstore": {
            "path": os.path.join(config_pkg_share, "worlds/bookstore/bookstore.world"),
            "x": "0.0", "y": "0.0", "z": "0.3"
        },
        "factory": {
            "path": os.path.join(config_pkg_share, "worlds/factory/factory.world"),
            "x": "0.0", "y": "0.0", "z": "0.4"
        },
        "small_house": {
            "path": os.path.join(config_pkg_share, "worlds/small_house/small_house.world"),
            "x": "1.0", "y": "2.0", "z": "0.3"
        },
        "office": {
            "path": os.path.join(config_pkg_share, "worlds/office/office.world"),
            "x": "9.0", "y": "4.0", "z": "0.3"
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

    # Si el mundo ingresado no está en el mapa, usamos 'default' por seguridad
    selected = world_map.get(world_selection, world_map["default"])
    
    descr_pkg_share = launch_ros.substitutions.FindPackageShare(package="go2_description").find("go2_description")
    joints_config = os.path.join(config_pkg_share, "config/joints/joints.yaml")
    gait_config = os.path.join(config_pkg_share, "config/gait/gait.yaml")
    links_config = os.path.join(config_pkg_share, "config/links/links.yaml")
    default_model_path = os.path.join(descr_pkg_share, "xacro/robot.xacro")

    bringup_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("champ_bringup"), "launch/bringup.launch.py")),
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

    gazebo_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("champ_gazebo"), "launch/gazebo.launch.py")),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "robot_name": LaunchConfiguration("robot_name"),
            "world": selected["path"],
            "lite": LaunchConfiguration("lite"),
            "world_init_x": selected["x"],
            "world_init_y": selected["y"],
            "world_init_z": selected["z"],
            "world_init_heading": "0.0",
            "gui": LaunchConfiguration("gui"),
            "close_loop_odom": "true",
        }.items(),
    )

    return [bringup_ld, gazebo_ld]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("robot_name", default_value="go2"),
        DeclareLaunchArgument("lite", default_value="false"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("world", default_value="default"),
        OpaqueFunction(function=launch_setup)
    ])
