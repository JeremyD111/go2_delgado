# Guía de Instalación, Compilación y Ejecución

## 1. Dependencias del Proyecto

Para replicar este entorno, se requiere:

- Sistema Operativo: Ubuntu 22.04 LTS.
- Middleware: ROS 2 Humble.
- Simulador: Gazebo Classic.
- Paquetes ROS 2: `nav2_map_server` y `nav2_lifecycle_manager` (Gestión de mapas).
- tf2_ros (Transformaciones de marcos de referencia).
- champ_bringup y champ_gazebo (Controladores del cuadrúpedo).
- Librerías Python: numpy (Cálculos de matriz de costo).

## 2. Instalación y Compilación

### Instalar dependencias del sistema
```bash
rosdep install -i --from-path . --rosdistro humble -y
```

### Clonar el repositorio:
```bash
git clone https://github.com/JeremyD111/go2_delgado
```

### Compilación de los paquetes del proyecto
```bash
cd ~/go2_delgado
colcon build --packages-select go2_description go2_config go2_planner
source install/setup.bash
```

### Ejecución de la simulacion 

Siga este orden en terminales independientes:

- **Terminal 1 (Simulación):**
Lanza Gazebo, carga los controladores del Go2 y spawnea el robot en el ambiente.

```bash
cd ~/go2_delgado
source install/setup.bash
ros2 launch go2_config gazebo_velodyne.launch.py world:=small_house
```

- **Terminal 2 (Planificación y RViz):**
Carga el mapa, activa los servicios de navegación y abre RViz configurado.

```bash
cd ~/go2_delgado
source install/setup.bash
ros2 launch go2_planner planner.launch.py
```
## Generacion de trayectorias

EN el Rviz, pulse el botón "2D Goal Pose" ubicado en la parte superior y de clic en cualquier lugar del mapa para que se genere automaticamente una trayectoria.



## 3. Estructura del Sistema (Node Graph)
Abra una tercera terminal y ejecute:

```bash
cd ~/go2_delgado
source install/setup.bash
ros2 run rqt_graph rqt_graph
```
![Grafo de Nodos ROS 2](img/node_graph.png)

## 4. Descripción de Launch Files

- **gazebo_velodyne.launch.py:** Configura la física en Gazebo, los sensores LiDAR y la cinemática de CHAMP para el Unitree Go2. 

- **planner.launch.py:** Gestiona el ciclo de vida del mapa, la transformación estática map->odom y el nodo de planificación Dijkstra.
