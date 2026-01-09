import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
import heapq

class DijkstraPlanner(Node):
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

    def map_callback(self, msg):
        """ Recibe el mapa y lo convierte en una matriz de ocupación """
        self.width = msg.info.width
        self.height = msg.info.height
        self.map_res = msg.info.resolution
        self.map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        # Convertir a matriz 2D: 0 es libre, >0 (u 100) es ocupado
        self.map_data = np.array(msg.data).reshape((self.height, self.width))
        self.get_logger().info("Mapa recibido correctamente")

    def odom_callback(self, msg):
        """ Actualiza la posición actual del robot """
        self.current_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]

    def world_to_map(self, x_world, y_world):
        """ Convierte coordenadas de mundo (metros) a índices de mapa (píxeles) """
        ix = int((x_world - self.map_origin[0]) / self.map_res)
        iy = int((y_world - self.map_origin[1]) / self.map_res)
        return (ix, iy)

    def map_to_world(self, ix, iy):
        """ Convierte índices de mapa a coordenadas de mundo """
        wx = ix * self.map_res + self.map_origin[0]
        wy = iy * self.map_res + self.map_origin[1]
        return wx, wy

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

def main(args=None):
    rclpy.init(args=args)
    node = DijkstraPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
