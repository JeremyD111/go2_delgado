import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
import numpy as np
import math

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')

        # --- Comunicaciones ---
        self.path_sub = self.create_subscription(Path, '/global_path', self.path_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

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

    def path_callback(self, msg):
        if not msg.poses: return
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.target_idx = 0
        self.integral_w = 0.0
        self.prev_error_w = 0.0
        self.total_distance = 0.0
        self.start_time = self.get_clock().now()
        self.is_active = True
        self.get_logger().info(f'¡RUTA RECIBIDA! Iniciando seguimiento de {len(self.path)} puntos...')

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

def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
