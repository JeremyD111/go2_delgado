import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
import heapq
import csv
import os
from scipy.ndimage import binary_dilation # Para inflar obstáculos

class DijkstraPlanner(Node):
    def __init__(self):
        super().__init__('dijkstra_planner')
        
        # --- Configuración de Seguridad ---
        # El Go2 mide aprox 0.25m de ancho. 
        # Ponemos 0.35m para tener margen de maniobra.
        self.robot_radius_meters = 0.55
        
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
        self.map_data = None       # Mapa original
        self.inflated_map = None   # Mapa con zonas de seguridad
        self.map_res = 0.0
        self.map_origin = [0.0, 0.0]
        self.current_pose = None 
        self.width = 0
        self.height = 0

    def map_callback(self, msg):
        """ Recibe el mapa e infla los obstáculos """
        self.width = msg.info.width
        self.height = msg.info.height
        self.map_res = msg.info.resolution
        self.map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        
        # 1. Convertir datos a matriz 2D
        # 0: libre, 100: ocupado, -1: desconocido
        raw_map = np.array(msg.data).reshape((self.height, self.width))
        
        # 2. Crear una máscara de obstáculos (consideramos ocupado > 50 y desconocido -1)
        obstacle_mask = (raw_map > 50) | (raw_map == -1)
        
        # 3. Calcular radio de inflación en píxeles
        pixel_radius = int(self.robot_radius_meters / self.map_res)
        
        # 4. Inflar obstáculos
        # Esto expande los obstáculos hacia afuera 'pixel_radius' celdas
        struct = np.ones((pixel_radius*2+1, pixel_radius*2+1))
        self.inflated_map = binary_dilation(obstacle_mask, structure=struct)
        
        # Guardamos el mapa procesado (0 libre, 1 ocupado/peligro)
        self.map_data = self.inflated_map.astype(int)
        
        self.get_logger().info(f"Mapa inflado con radio de {self.robot_radius_meters}m ({pixel_radius} px)")

    def world_to_map(self, x_world, y_world):
        ix = int((x_world - self.map_origin[0]) / self.map_res)
        iy = int((y_world - self.map_origin[1]) / self.map_res)
        return (ix, iy)

    def map_to_world(self, ix, iy):
        wx = ix * self.map_res + self.map_origin[0]
        wy = iy * self.map_res + self.map_origin[1]
        return wx, wy

    def get_neighbors(self, node):
        """ Retorna vecinos que no estén en la zona inflada """
        neighbors = []
        for dx, dy in [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]:
            nx, ny = node[0] + dx, node[1] + dy
            
            if 0 <= nx < self.width and 0 <= ny < self.height:
                # REVISAR MAPA INFLADO: 0 significa que es seguro para el robot
                if self.map_data[ny, nx] == 0:
                    cost = np.sqrt(dx**2 + dy**2)
                    neighbors.append(((nx, ny), cost))
        return neighbors

    def run_dijkstra(self, start, goal):
        # Verificar que el inicio no esté dentro de un obstáculo inflado
        if self.map_data[start[1], start[0]] == 1:
            self.get_logger().error("¡El robot está dentro de una zona de inflación! Muévelo manualmente.")
            return None

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
        
        if goal not in came_from:
            return None
        
        path = []
        curr = goal
        while curr is not None:
            path.append(curr)
            curr = came_from[curr]
        return path[::-1]

    def goal_callback(self, msg):
        if self.map_data is None or self.current_pose is None:
            self.get_logger().warn("Esperando mapa u odometría...")
            return

        start_grid = self.world_to_map(self.current_pose[0], self.current_pose[1])
        goal_grid = self.world_to_map(msg.pose.position.x, msg.pose.position.y)

        # Validación del objetivo: si el objetivo cae en zona inflada, buscar el punto libre más cercano
        if self.map_data[goal_grid[1], goal_grid[0]] == 1:
            self.get_logger().warn("Objetivo en zona no segura. Buscando alternativa...")
            # (Aquí podrías añadir lógica para mover el goal al pixel libre más cercano)

        self.get_logger().info(f"Calculando ruta segura...")
        path_indices = self.run_dijkstra(start_grid, goal_grid)

        if path_indices:
            # OPTIONAL: Suavizado simple (cada 3 puntos) para que el PID no sufra tanto
            path_indices = path_indices[::2] + [path_indices[-1]]
            
            self.publish_path(path_indices)
            self.save_waypoints_to_csv(path_indices)
        else:
            self.get_logger().error("No se pudo encontrar una ruta segura.")

    def odom_callback(self, msg):
        self.current_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]

    def publish_path(self, path_indices):
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

    def save_waypoints_to_csv(self, path_indices):
        file_path = os.path.expanduser('~/go2_delgado/waypoints_dijkstra.csv')
        with open(file_path, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y'])
            for ix, iy in path_indices:
                wx, wy = self.map_to_world(ix, iy)
                writer.writerow([wx, wy])

def main(args=None):
    rclpy.init(args=args)
    node = DijkstraPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
