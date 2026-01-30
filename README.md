# PARTE #1: Unitree Go2 - SLAM y Planificador global de trayectorias (Dijkstra)

- **Planificador globar de trayectoria:** Dijkstra
- **Repositorio de referencia:** [https://github.com/widegonz/unitree-go2-ros2](https://github.com/widegonz/unitree-go2-ros2)
- **Mapa:** Small_house
![mapa small house](img/small_house.jpg)

## Estructura del paquete de ROS
```bash
├── go2_planner
│   ├── __init__.py
│   ├── pid_controller_node.py
│   └── planner_node.py
├── launch
│   └── planner.launch.py
├── maps
│   ├── map.pgm
│   └── map.yaml
├── package.xml
├── resource
│   └── go2_planner
├── rviz
│   └── planner.rviz
├── setup.cfg
├── setup.py
└── test
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py
```

# 1. Dependencias para ROS

```bash
sudo apt install ros-humble-gazebo-ros2-control
sudo apt install ros-humble-xacro
sudo apt install ros-humble-robot-localization
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-velodyne
sudo apt install ros-humble-velodyne-gazebo-plugins
sudo apt-get install ros-humble-velodyne-description
sudo apt install -y python3-rosdep
rosdep update
```

# 2. Guía de Instalación y Compilación
## 2.0 Clonar el repositorio:
En su carpeta HOME clonar el repositorio:
```bash
git clone https://github.com/JeremyD111/go2_delgado
```

## 2.1 Compilación de los paquetes del proyecto
```bash
cd ~/go2_delgado
colcon build 
source install/setup.bash
```
## Ejecutar simulacion final:
Una vez realizado los pasos anteriores puede saltarse a la parte 2 del proyecto para ejecutar la simulacion final que ya incluye DIJKSTRA Y PID [Ir a Parte#2](#compilacion-y-ejecucion-de-la-simulacion)

# 3. Mapeo del entorno (SLAM)
## 3.0 Abrir el mapa

```bash
cd ~/go2_delgado
ros2 launch go2_config gazebo_velodyne.launch.py world:=small_house
```

## 3.1 Ejecutar SLAM_Toolbox Package

```bash
cd ~/go2_delgado
ros2 launch go2_config slam.launch.py use_sim_time:=true
```

- Se abrirá Rviz tal como se muestra a continuación:
![slam1](img/slam1.png)

## 3.2 Nodo de teleoperacion

```bash
cd ~/go2_delgado
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 3.3 Mapeo 
Con ayuda del teclado empezaremos a mover el robot para mapear toda la zona del mapa small_house.

<div align="center">
  <video src="https://github.com/user-attachments/assets/d6383d34-b497-40fe-b45e-badf5860b23f" width="600px" controls></video>
  <p><i>Video 1: Demostración del proceso de mapeo (SLAM)</i></p>
</div>

## 3.4 Guardar el mapa
Guarda el mapa haciendo uso del siguiente comando:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```

## 3.5 Resultados
Se crearan dos archivos llamados:
- map.pgm
- map.yaml

![slam](img/map_slam.png)


# 4. Planificación global de trayectoria - Dijkstra
El algoritmo de Dijkstra es un método de búsqueda en grafos que garantiza encontrar la ruta de menor costo entre un punto de inicio y un objetivo. Funciona mediante una exploración expansiva que prioriza siempre los nodos con el menor costo acumulado. 

## 4.1 Entradas y salidas 
***Entradas (Subscribers):***
- `/map` `(nav_msgs/OccupancyGrid)`: Provee la información de la rejilla de ocupación y obstáculos del entorno.
- `/odom` `(nav_msgs/Odometry)`: Proporciona la posición y orientación actual del robot en tiempo real.
- `/goal_pose` `(geometry_msgs/PoseStamped)`: Recibe la coordenada de destino seleccionada manualmente por el usuario en RViz.

***Salidas (Publishers):***
- `/goal_pose` `(nav_msgs/Path)`: Envía la secuencia de waypoints óptima calculada por el algoritmo para ser visualizada en RViz.

## 4.2 Explicacion de algoritmo 

### 4.2.1 Función de Inicialización (`__init__`):
Esta función se encarga de configurar la infraestructura de comunicación y el estado inicial del nodo:

- ***Configuración de QoS:*** Define un perfil de Calidad de Servicio TRANSIENT_LOCAL para asegurar la recepción del mapa, incluso si fue publicado antes de iniciar el planificador.

- ***Suscripciones:*** Establece la comunicación con los tópicos del mapa, la odometría del robot y la pose objetivo.

- ***Publicadores:*** Inicializa el canal para enviar la trayectoria final calculada hacia RViz.

- ***Variables de estado:*** Declara las variables que almacenarán los metadatos del mapa (resolución, dimensiones, origen) y la ubicación en tiempo real del Unitree Go2.

```bash
def __init__(self):
        super().__init__('dijkstra_planner')
        
        # Configuración de QoS para el Mapa
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- Suscriptores ---
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_profile)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        # --- Publicadores ---
        self.path_pub = self.create_publisher(Path, '/global_path', 10)

        # --- Variables de estado ---
        self.map_data = None
        self.map_res = 0.0
        self.map_origin = [0.0, 0.0]
        self.current_pose = None # [x, y]
        self.width = 0
        self.height = 0
```

### 4.2.2 Gestión de Datos del Mapa (`map_callback`):
Esta función procesa el mensaje de tipo OccupancyGrid para construir el espacio de búsqueda digital:

- ***Extracción de Metadatos:*** Almacena la resolución (metros/píxel) y el punto de origen del mapa, valores críticos para realizar transformaciones de coordenadas exactas.

- ***Transformación de Datos:*** Convierte el arreglo unidimensional de datos crudos en una matriz bidimensional de NumPy (H×W).

- ***Discretización:*** Facilita la identificación de obstáculos, donde el valor 0 representa celdas libres y valores superiores indican la presencia de paredes u objetos, permitiendo al algoritmo validar trayectorias.

```bash
def map_callback(self, msg):
        """ Recibe el mapa y lo convierte en una matriz de ocupación """
        self.width = msg.info.width
        self.height = msg.info.height
        self.map_res = msg.info.resolution
        self.map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        # Convertir a matriz 2D: 0 es libre, >0 (u 100) es ocupado
        self.map_data = np.array(msg.data).reshape((self.height, self.width))
        self.get_logger().info("Mapa recibido correctamente")
```

### 4.2.3 Actualización de Posición (`odom_callback`):
- ***Extracción de Coordenadas:*** Filtra el mensaje de tipo Odometry para obtener exclusivamente la posición cartesiana (x,y) referenciada en metros.

- ***Definición del Punto de Inicio:*** Mantiene actualizada la variable current_pose, la cual es indispensable para determinar el nodo de origen (start node) al momento de iniciar la búsqueda del camino más corto.

```bash
 def odom_callback(self, msg):
        """ Actualiza la posición actual del robot """
        self.current_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]
```

### 4.2.4 Transformación de Coordenadas (`world_to_map` y `map_to_world`)
- ***world_to_map:*** Transforma coordenadas métricas del mundo real en índices discretos de la matriz del mapa (píxeles), restando el origen y dividiendo por la resolución

```bash
def world_to_map(self, x_world, y_world):
        """ Convierte coordenadas de mundo (metros) a índices de mapa (píxeles) """
        ix = int((x_world - self.map_origin[0]) / self.map_res)
        iy = int((y_world - self.map_origin[1]) / self.map_res)
        return (ix, iy)
```

- ***map_to_world:*** Realiza el proceso inverso para convertir los índices del camino encontrado en coordenadas espaciales que el robot pueda interpretar, multiplicando por la resolución y sumando el origen

```bash
def map_to_world(self, ix, iy):
        """ Convierte índices de mapa a coordenadas de mundo """
        wx = ix * self.map_res + self.map_origin[0]
        wy = iy * self.map_res + self.map_origin[1]
        return wx, wy
```

### 4.2.5 Generación de Vecinos (`get_neighbors`)
- ***8-Conectividad:*** Evalúa las celdas vecinas en todas las direcciones (arriba, abajo, lados y las cuatro diagonales), permitiendo una planificación de trayectoria más natural y fluida.

- ***Validación de Límites:*** Verifica que las coordenadas generadas se encuentren dentro de las dimensiones reales de la matriz de ocupación para evitar errores de desbordamiento de índices.

- ***Detección de Obstáculos:*** Filtra únicamente las celdas cuyo valor en map_data sea exactamente 0 (espacio libre), garantizando que la ruta no atraviese paredes.

- ***Ponderación de Costos:*** Asigna un costo diferenciado según el tipo de movimiento:
   - Costo 1.0: Para movimientos ortogonales.
   - Costo 1.41: Para movimientos diagonales, asegurando la precisión matemática de la distancia recorrida.

```bash
def get_neighbors(self, node):
        """ Retorna los vecinos válidos (8 conectividad) """
        neighbors = []
        for dx, dy in [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]:
            nx, ny = node[0] + dx, node[1] + dy
            if 0 <= nx < self.width and 0 <= ny < self.height:
                # El valor 0 en el OccupancyGrid es 'espacio libre'
                if self.map_data[ny, nx] == 0: 
                    # Costo 1 para rectos, 1.41 para diagonales
                    cost = np.sqrt(dx**2 + dy**2)
                    neighbors.append(((nx, ny), cost))
        return neighbors
```

### 4.2.6 Algoritmo Central (`run_dijkstra`)
- ***Priorización:*** Utiliza una cola de prioridad para explorar siempre la celda con el menor costo acumulado.

- ***Optimización:*** Registra en `cost_so_far` el costo mínimo por celda, actualizándolo si encuentra una ruta más corta.

- ***Trazabilidad:*** Almacena el nodo predecesor en `came_from` para permitir la reconstrucción del camino.

- ***Finalización:*** Detiene la búsqueda al alcanzar el objetivo y genera la trayectoria mediante backtracking inverso.

```bash
def run_dijkstra(self, start, goal):
        """ Implementación pura de Dijkstra """
        open_list = []
        heapq.heappush(open_list, (0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}

        while open_list:
            current_cost, current_node = heapq.heappop(open_list)

            if current_node == goal:
                break

            for neighbor, step_cost in self.get_neighbors(current_node):
                new_cost = cost_so_far[current_node] + step_cost
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    heapq.heappush(open_list, (new_cost, neighbor))
                    came_from[neighbor] = current_node
        
        # Reconstruir camino
        if goal not in came_from:
            return None
        
        path = []
        curr = goal
        while curr is not None:
            path.append(curr)
            curr = came_from[curr]
        return path[::-1] # Invertir para que vaya de start a goal
```

### 4.2.7 Activación del Planificador (`goal_callback`)
- ***Validación:*** Verifica la disponibilidad del mapa y la pose del robot antes de procesar cualquier solicitud.

- ***Preprocesamiento:*** Convierte las coordenadas métricas (inicio y meta) a índices de rejilla mediante `world_to_map`.

- ***Cálculo:*** Invoca la ejecución del algoritmo de Dijkstra para generar la secuencia de celdas óptima.

- ***Publicación:*** Envía el camino encontrado al tópico correspondiente para su visualización inmediata en RViz.

```bash
def goal_callback(self, msg):
        """ Se activa cuando el usuario pone un '2D Goal Pose' en RViz """
        if self.map_data is None or self.current_pose is None:
            self.get_logger().warn("Esperando mapa u odometría...")
            return

        start_grid = self.world_to_map(self.current_pose[0], self.current_pose[1])
        goal_grid = self.world_to_map(msg.pose.position.x, msg.pose.position.y)

        self.get_logger().info(f"Calculando ruta desde {start_grid} hasta {goal_grid}...")
        
        path_indices = self.run_dijkstra(start_grid, goal_grid)

        if path_indices:
            self.publish_path(path_indices)
            self.get_logger().info("¡Ruta encontrada y publicada!")
        else:
            self.get_logger().error("No se pudo encontrar una ruta válida.")
```

### 4.2.8 Publicación de Trayectoria (`publish_path`)
- ***Conversión:*** Traduce los índices de la rejilla a coordenadas métricas (x,y) mediante la función `map_to_world`.

- ***Construcción:*** Genera un mensaje `nav_msgs/Path` compuesto por una lista secuencial de poses (`PoseStamped`).

- ***Sincronización:*** Configura el encabezado del mensaje con el marco de referencia `map` y el tiempo actual del sistema.

- ***Salida:*** Publica la trayectoria procesada en el tópico `/global_path` para su renderizado en RViz.

```bash
def publish_path(self, path_indices):
        """ Convierte los índices en un mensaje nav_msgs/Path """
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for ix, iy in path_indices:
            pose = PoseStamped()
            wx, wy = self.map_to_world(ix, iy)
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
```

# 5. Guía de Ejecución de la simulacion 

Siga este orden en terminales independientes:

- **Terminal 1 (Simulación):**
Lanza Gazebo, carga los controladores del Go2 y spawnea el robot en el mapa small_house.

```bash
cd ~/go2_delgado
source install/setup.bash
ros2 launch go2_config gazebo_velodyne.launch.py world:=small_house
```
![go2_map](img/go2_map.png)


- **Terminal 2 (Planificación y RViz):**
Carga el mapa, activa los servicios de navegación y abre RViz configurado.

```bash
cd ~/go2_delgado
source install/setup.bash
ros2 launch go2_planner planner.launch.py
```
![go2_rviz](img/go2_rviz.png)

## 5.1 Generacion de trayectorias

EN el Rviz, pulse el botón "2D Goal Pose" ubicado en la parte superior y de clic en cualquier lugar del mapa para que se genere automaticamente una trayectoria.

![Generacion de trayectoria](img/Trayectoria.png)

<div align="center">
  <video src="https://github.com/user-attachments/assets/bfa73f40-68c1-40c6-a760-94608ed7b121
" width="500px" controls></video>
  <p><i>Video 2: Ejecución del algoritmo de Dijkstra </i></p>
</div>


## 5.2 Estructura de comunicacion de nodos (Node Graph)
Abra una tercera terminal y ejecute:

```bash
cd ~/go2_delgado
source install/setup.bash
ros2 run rqt_graph rqt_graph
```
![Grafo de Nodos ROS 2](img/node_graph.png)


# 6. Video de youtube
[https://www.youtube.com/watch?v=PY8p1WlFl1I ](https://www.youtube.com/watch?v=PY8p1WlFl1I )









# PARTE #2: Seguimiento de Trayectoria - Controlador PID 

Esta sección describe la implementación del controlador local encargado de seguir la ruta generada por el algoritmo de Dijkstra.

# Explicacion del algoritmo de control PID
## 1. Entradas y Salidas:
**Entradas (Subscribers)**
- `/global_path (nav_msgs/Path)`: La ruta óptima generada por el planificador Dijkstra.
- `/odom (nav_msgs/Odometry)`: Retroalimentación de la posición y velocidad actual del robot.

**Salidas (Publishers)**
- `/cmd_vel` (`geometry_msgs/Twist`): Comandos de velocidad enviados a la cinemática del robot.

```bash
class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')

        # --- Comunicaciones ---
        self.path_sub = self.create_subscription(Path, '/global_path', self.path_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

```

## 2. Configuración y Parámetros del Controlador
**Parámetros de Sintonización (Gains)**
- `kp_w (2.0)`: Ganancia Proporcional. Define la agresividad del giro.

- `ki_w (0.0021)`: Ganancia Integral. Elimina el error en estado estacionario.

- `kd_w (0.13)`: Ganancia Derivativa. Actúa como un amortiguador (evita zigzag).

**Límites Dinámicos y Seguridad**
- `v_nominal (0.55 m/s)`: Velocidad crucero definida para un entorno de interiores.

- `max_w (1.0 rad/s)`: Velocidad angular máxima. 

- `accel_limit (0.04)`: Es el parámetro de suavizado. Limita cuánto puede cambiar la velocidad en cada ciclo (50ms). 

**Variables de Estado del Lazo de Control**
- `target_idx`: El puntero que rastrea en qué waypoint de la ruta de Dijkstra nos encontramos.

- `prev_error_w`: Almacena el error del ciclo anterior para calcular la derivada (de/dt).

- `integral_w`: Acumula el error histórico para la componente integral.

- `current_v`: Realiza el seguimiento de la velocidad actual para aplicar la rampa de aceleración de forma incremental.
```bash
# --- Parámetros de Control (Tuning) ---
        self.kp_w, self.ki_w, self.kd_w = 2.0, 0.0021, 0.13
        self.v_nominal = 0.55   
        self.max_w = 1.0       
        self.accel_limit = 0.04 
        
        # --- Estado del Robot ---
        self.path = []
        self.current_pose = None   
        self.target_idx = 0
        self.prev_error_w = 0.0
        self.integral_w = 0.0
        self.current_v = 0.0       

        # --- Variables de Telemetría ---
        self.total_distance = 0.0
        self.last_odom_pos = None  
        self.start_time = None
        self.is_active = False
        self.telemetry_tick = 0 # Para limitar la frecuencia de impresión

        # Timer principal (20Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('*** SISTEMA GO2 LISTO - ESPERANDO RUTA GLOBAL ***')
```

## 3. Gestion de la Ruta (`path_callback`)

- `integral_w = 0.0`: Elimina cualquier acumulación de error de giros anteriores. 

- `prev_error_w = 0.0`: Limpia la memoria del término derivativo.

- Al configurar `is_active = True`, el nodo permite que el `control_loop` empiece a publicar comandos en el tópico `/cmd_vel`.

- Se inicializa el índice de seguimiento (`target_idx = 0`) para garantizar que el Go2 siempre se dirija al primer punto de la nueva trayectoria, sin importar su posición previa.

```bash
def path_callback(self, msg):
        if not msg.poses: return
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.target_idx = 0
        self.integral_w = 0.0
        self.prev_error_w = 0.0
        self.total_distance = 0.0
        self.start_time = self.get_clock().now()
        self.is_active = True
        self.get_logger().info(f'RUTA RECIBIDA: Iniciando seguimiento de {len(self.path)} puntos...')

```

## 4. Procesamiento de Odometría (`odom_callback`)
- **Conversión de Orientación:** Transforma la orientación del robot de Cuaterniones (formato original de ROS 2) a Ángulo Yaw (Euler) usando atan2.

- **Actualización de Pose:** Almacena continuamente la posición (x,y) y rotación actual en self.current_pose, sirviendo como la retroalimentación (feedback) en tiempo real para el lazo de control.

- **Cálculo de Distancia Acumulada:** Integra el desplazamiento del robot sumando la distancia euclidiana entre la posición actual y la anterior (last_odom_pos) únicamente cuando el seguimiento de ruta está activo.

- **Filtro de Estabilidad Física:** Implementa un umbral de seguridad (dist < 0.2). Si el robot sufre un salto brusco o "glitch" en la simulación mayor a 20 cm en un solo ciclo, la telemetría ignora ese dato para no falsear la distancia total recorrida.

```bash
def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.current_pose = [x, y, yaw]

        # Calcular distancia solo si el robot se está moviendo hacia una ruta
        if self.is_active and self.last_odom_pos is not None:
            dist = math.sqrt((x - self.last_odom_pos[0])**2 + (y - self.last_odom_pos[1])**2)
            if dist < 0.2: # Filtrar saltos bruscos por ruido
                self.total_distance += dist
        
        self.last_odom_pos = [x, y]
```

## 5. Seguimiento de trayectoria PID
- **Cálculo de Errores:** Determina la distancia euclidiana y el ángulo hacia el waypoint actual, normalizando el error angular mediante `atan2` para asegurar que el robot siempre gire por el camino más corto (rango de [−π,π]).

- **Gestión Dinámica de Waypoints:** Implementa una lógica de "mirada hacia adelante" (look-ahead) usando umbrales distintos: un margen amplio (0.35m) para fluir entre puntos intermedios y uno estricto (0.15m) para garantizar la precisión al llegar a la meta final.

- **PID Angular con Anti-Windup:** Calcula la velocidad de giro combinando la respuesta proporcional, integral y derivativa. Incluye un clipping (recorte) en el término integral para evitar que el error acumulado cause giros violentos tras una obstrucción.

- **Atenuación de Velocidad por Giro:** Ajusta el objetivo de velocidad lineal según el error de orientación mediante una función de coseno al cuadrado; esto obliga al robot a frenar automáticamente en curvas cerradas para mantener la estabilidad.

- **Rampa de Aceleración:** Compara la velocidad actual con la deseada y aplica un incremento gradual (`accel_limit`). Esto elimina los tirones bruscos de los motores, protegiendo el factor de tiempo real (RTF) de la simulación.

- **Condición de Meta:** Monitorea si se ha alcanzado el último punto de la lista de Dijkstra para desactivar el nodo, poner los motores a cero y ejecutar la impresión del resumen final de la misión.

```bash
def control_loop(self):
        if not self.path or self.current_pose is None or not self.is_active:
            return

        curr_x, curr_y, curr_yaw = self.current_pose
        target = self.path[self.target_idx]

        # 1. Cálculos de Error
        dist_to_target = math.sqrt((target[0] - curr_x)**2 + (target[1] - curr_y)**2)
        desired_yaw = math.atan2(target[1] - curr_y, target[0] - curr_x)
        error_w = math.atan2(math.sin(desired_yaw - curr_yaw), math.cos(desired_yaw - curr_yaw))

        # 2. Lógica de Waypoints
        threshold = 0.35 if self.target_idx < len(self.path) - 1 else 0.15
        if dist_to_target < threshold and self.target_idx < len(self.path) - 1:
            self.target_idx += 1
            return

        # 3. PID Angular
        self.integral_w = np.clip(self.integral_w + error_w * 0.05, -0.3, 0.3)
        derivative_w = (error_w - self.prev_error_w) / 0.05
        w_cmd = np.clip((self.kp_w * error_w) + (self.ki_w * self.integral_w) + (self.kd_w * derivative_w), -self.max_w, self.max_w)
        self.prev_error_w = error_w

        # 4. Velocidad con Rampa y Suavizado
        v_target = self.v_nominal * (math.cos(np.clip(error_w, -1.5, 1.5))**2)
        if self.current_v < v_target:
            self.current_v = min(self.current_v + self.accel_limit, v_target)
        else:
            self.current_v = max(self.current_v - self.accel_limit, v_target)

        # 5. Publicación y Meta
        cmd = Twist()
        if self.target_idx >= len(self.path) - 1 and dist_to_target < 0.15:
            cmd.linear.x, cmd.angular.z = 0.0, 0.0
            self.is_active = False
            self.imprimir_resumen_final()
        else:
            cmd.linear.x, cmd.angular.z = self.current_v, w_cmd
            self.gestionar_telemetria()
            
        self.cmd_pub.publish(cmd)
```

## 6. Monitorización y Reporte de Resultados
- **Control de Frecuencia de Salida:** Implementa un limitador mediante el operador módulo (% 10) para imprimir datos solo cada 0.5 segundos.

- **Feedback de Navegación en Tiempo Real:** Calcula el tiempo transcurrido desde el inicio de la ruta y muestra simultáneamente la distancia acumulada, la velocidad actual y el progreso de los waypoints.

- **Cálculo de Métricas de Desempeño:** Al detectar el fin de la trayectoria, computa el tiempo total de operación y la velocidad media real.

- **Reporte Final de Misión:** Genera un resumen visual estructurado en la terminal mediante get_logger().info, lo que garantiza que los resultados de distancia y tiempo aparezcan de forma inmediata incluso al ejecutar desde un archivo launch.


```bash
def gestionar_telemetria(self):
        """ Imprime progreso cada 0.5 segundos para evitar saturar la terminal """
        self.telemetry_tick += 1
        if self.telemetry_tick % 10 == 0: # Cada 10 ciclos (a 20Hz = 0.5s)
            tiempo_actual = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            self.get_logger().info(
                f"-> Dist: {self.total_distance:5.2f}m | "
                f"Meta: {self.target_idx+1}/{len(self.path)} | "
                f"V: {self.current_v:.2f}m/s | "
                f"T: {tiempo_actual:.1f}s"
            )

    def imprimir_resumen_final(self):
        """ Resumen detallado al llegar a la meta """
        tiempo_total = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        vel_media = self.total_distance / tiempo_total if tiempo_total > 0 else 0.0
        
        # Loggers para que aparezcan inmediatamente en la terminal del launch
        self.get_logger().info("=============================================")
        self.get_logger().info("     🏁 ¡META FINAL ALCANZADA! 🏁")
        self.get_logger().info("=============================================")
        self.get_logger().info(f" Distancia Total : {self.total_distance:.2f} metros")
        self.get_logger().info(f" Tiempo Total    : {tiempo_total:.2f} segundos")
        self.get_logger().info(f" Velocidad Media : {vel_media:.2f} m/s")
        self.get_logger().info("=============================================")
```



# Compilacion y ejecucion de la simulacion 

### 1. Lanza el entorno en Gazebo (Terminal1)
```bash
cd ~/go2_delgado
colcon build
source install/setup.bash
ros2 launch go2_config gazebo_velodyne.launch.py world:=small_house
```
### 2. Abrir el mapa en rviz + dijkstra + PID (Terminal 2):
En el archivo launch ya se incluye la ejecucion del Dijkstra y el PID.

```bash
ros2 launch go2_planner planner.launch.py
```
![go2_rviz](img/go2_rviz.png)

### 3. Generacion de trayectoria y control PID
EN el Rviz, pulse el botón "2D Goal Pose" ubicado en la parte superior y de clic en cualquier lugar del mapa para que se genere automaticamente una trayectoria y el robot comenzara a seguirla.

![Generacion de trayectoria](img/Trayectoria.png)

# Video de Youtube

- video youtube: [https://youtu.be/IdI1W1w757w?si=uEkZgezxuImxjMoD](https://youtu.be/IdI1W1w757w?si=uEkZgezxuImxjMoD)


# Descripción de Launch Files

- **planner.launch.py:** Gestiona el ciclo de vida del mapa, la transformación estática map->odom , el nodo de planificación Dijkstra y nodo de control PID

```bash
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

        # Nodo 5: Controlador Local (PID)
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
```

- **gazebo_velodyne.launch.py:** Configura la física en Gazebo, los sensores LiDAR y la cinemática de CHAMP para el Unitree Go2. 

```bash
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
```















