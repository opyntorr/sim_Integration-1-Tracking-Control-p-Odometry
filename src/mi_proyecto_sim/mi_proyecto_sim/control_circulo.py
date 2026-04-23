#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import math
import tf_transformations
from rclpy.signals import SignalHandlerOptions

class ControlCirculoReal(Node):
    def __init__(self):
        super().__init__('control_circulo')
        
        # Suscriptores y Publicadores
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.error_pub = self.create_publisher(Point, '/tracking_error', 10)
        self.path_pub = self.create_publisher(Point, '/desired_trajectory', 10)
        self.actual_pub = self.create_publisher(Point, '/actual_trajectory', 10)
        
        # Parámetros Kelly & Diaz [cite: 8]
        self.h = 0.15      # Distancia del punto de control [cite: 52]
        self.k_p = 1.5     # Ganancia proporcional [cite: 169]
        
        # --- LÓGICA DE RESET INTERNO ---
        self.offset_x = None
        self.offset_y = None
        self.offset_theta = None
        
        self.x = 0.0; self.y = 0.0; self.theta = 0.0
        
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.control_loop)
    
    def odom_callback(self, msg):
        # Obtener pose cruda del sensor [cite: 18]
        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        raw_theta = euler[2]

        # Primer callback: establecer el origen local
        if self.offset_x is None:
            self.offset_x = raw_x
            self.offset_y = raw_y
            self.offset_theta = raw_theta
            self.get_logger().info(f"🔄 Círculo: Odometría reseteada a (0,0) local.")

        # Transformación de coordenadas relativas al inicio
        dx = raw_x - self.offset_x
        dy = raw_y - self.offset_y
        
        # Rotación para que el frente del robot al arrancar sea siempre el eje X+
        self.x = dx * math.cos(-self.offset_theta) - dy * math.sin(-self.offset_theta)
        self.y = dx * math.sin(-self.offset_theta) + dy * math.cos(-self.offset_theta)
        self.theta = raw_theta - self.offset_theta

    def control_loop(self):
        if self.offset_x is None: return

        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        # Trayectoria Deseada Circular [cite: 258]
        R = 0.25; w_d = 0.15
        x_d = R * math.cos(w_d * t)
        y_d = R * math.sin(w_d * t)
        xd_dot = -R * w_d * math.sin(w_d * t)
        yd_dot = R * w_d * math.cos(w_d * t)
        
        # Punto de control desplazado (Kelly & Diaz) [cite: 54]
        x_c = self.x + self.h * math.cos(self.theta)
        y_c = self.y + self.h * math.sin(self.theta)
        
        # Errores de posición [cite: 169]
        e_x = x_d - x_c
        e_y = y_d - y_c
        error_total = math.hypot(e_x, e_y)
        
        # Ley de control Proporcional + Feedforward [cite: 167]
        u_x = xd_dot + self.k_p * e_x
        u_y = yd_dot + self.k_p * e_y
        
        # Matriz inversa para robot diferencial [cite: 82]
        v = u_x * math.cos(self.theta) + u_y * math.sin(self.theta)
        w = (-u_x * math.sin(self.theta) + u_y * math.cos(self.theta)) / self.h
        
        cmd = Twist()
        cmd.linear.x = max(min(float(v), 0.5), -0.5)
        cmd.angular.z = max(min(float(w), 1.5), -1.5)
        self.cmd_pub.publish(cmd)
        
        # Publicar para rqt_plot en tiempo real
        self.error_pub.publish(Point(x=e_x, y=e_y, z=error_total))
        self.path_pub.publish(Point(x=x_d, y=y_d, z=0.0))
        self.actual_pub.publish(Point(x=x_c, y=y_c, z=0.0))

def main(args=None):
    # Usar SignalHandlerOptions.NO para evitar el crash de contexto al dar Ctrl+C
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ControlCirculoReal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Deteniendo prueba circular...")
        node.cmd_pub.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()