# Guía de Instalación, Compilación y Ejecución
## 1. Dependencias del Proyecto

Para replicar este entorno, se requiere:

    Sistema Operativo: Ubuntu 22.04 LTS.

    Middleware: ROS 2 Humble Hawksbill.

    Simulador: Gazebo Classic.

    Paquetes ROS 2: * nav2_map_server y nav2_lifecycle_manager (Gestión de mapas).

        tf2_ros (Transformaciones de marcos de referencia).

        champ_bringup y champ_gazebo (Controladores del cuadrúpedo).

    Librerías Python: numpy (Cálculos de matriz de costo).

## 2. Instalación y Compilación

El repositorio debe clonarse dentro de la carpeta src de un workspace de ROS 2:
Bash

# Navegar al workspace
cd ~/go2_delgado/src

# Instalar dependencias del sistema
rosdep install -i --from-path . --rosdistro humble -y

# Compilación de los paquetes del proyecto
cd ~/go2_delgado
colcon build --packages-select go2_description go2_config go2_planner
source install/setup.bash

## 3. Estructura del Sistema (Node Graph)

La arquitectura del sistema sigue el flujo: Sensor de Odometría/Mapa -> Nodo Planificador (Dijkstra) -> Visualización en RViz.

    Nodos principales:

        map_server: Publica los datos del mapa estático.

        planner_node: Nodo propio que ejecuta Dijkstra al recibir un /goal_pose.

        static_transform_publisher: Vincula el marco map con odom.

## 4. Ejecución

Para visualizar la planificación, siga este orden en terminales independientes:

Terminal 1 (Simulación):
Bash

ros2 launch go2_config gazebo_velodyne.launch.py world:=small_house

Lanza Gazebo, carga los controladores del Go2 y spawnea el robot en el ambiente.

Terminal 2 (Planificación y RViz):
Bash

ros2 launch go2_planner planner.launch.py

Carga el mapa, activa los servicios de navegación y abre RViz configurado.

## 5. Descripción de Launch Files

    gazebo_velodyne.launch.py: Configura la física en Gazebo, los sensores LiDAR y la cinemática de CHAMP para el Unitree Go2. (2 líneas)

    planner.launch.py: Gestiona el ciclo de vida del mapa, la transformación estática map->odom y el nodo de planificación Dijkstra. (2 líneas)
