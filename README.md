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

![Generacion de trayectoria](img/Trayectoria.png)

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





# Navegación y Planificación de Trayectorias: Unitree Go2 en ROS 2 Humble

## 1. Descripción del Proyecto
Este proyecto presenta la implementación de un sistema completo de **Mapeo (SLAM)** y **Planificación Global de Trayectorias** para el robot cuadrúpedo **Unitree Go2** dentro del entorno de simulación **Gazebo Classic**. 

El objetivo principal es permitir que el robot procese un entorno previamente mapeado, reciba una coordenada meta (Goal Pose) a través de la interfaz de **RViz2** y calcule de manera autónoma la ruta más eficiente evitando obstáculos conocidos. El sistema integra la descripción cinemática del robot, sensores LiDAR para la percepción y un nodo de planificación desarrollado desde cero en Python, garantizando una base robusta para la navegación autónoma en entornos interiores controlados.



## 2. Algoritmo de Planificación: Dijkstra
Para la planificación global se ha implementado el **Algoritmo de Dijkstra**. Este algoritmo de búsqueda en grafos garantiza encontrar el camino más corto entre el robot y su objetivo dentro de un mapa de ocupación discreto.

### Detalles de Implementación
* **Representación del Entorno:** El mapa (`nav_msgs/OccupancyGrid`) se transforma en una matriz de NumPy donde cada celda representa un nodo del grafo.
* **Conectividad:** Se utiliza **8-conectividad**, permitiendo movimientos horizontales, verticales y diagonales con costos diferenciados:
  * Movimiento Recto: $Costo = 1.0$
  * Movimiento Diagonal: $Costo = \sqrt{2} \approx 1.41$
* **Variables Clave del Nodo:**
    * `start_grid`: Posición actual del robot convertida a índices de matriz.
    * `goal_grid`: Posición objetivo recibida desde el tópico `/goal_pose`.
    * `cost_so_far`: Diccionario que registra el costo acumulado mínimo para alcanzar cada celda.
    * `came_from`: Estructura de datos para la reconstrucción de la trayectoria (backtracking).
* **Modificaciones y Mejoras:** * Se integró un perfil de **QoS (Quality of Service)** con durabilidad `Transient Local` para asegurar que el mapa sea recibido correctamente por el nodo de planificación, independientemente del orden de inicio.
    * Se desarrollaron funciones de transformación de coordenadas para mapear el espacio continuo de ROS (metros) al espacio discreto de la matriz (celdas).
