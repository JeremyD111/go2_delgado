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
