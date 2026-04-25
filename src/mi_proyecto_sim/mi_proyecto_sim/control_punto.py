#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import math
import tf_transformations
from rclpy.signals import SignalHandlerOptions

class ControlPuntoReal(Node):
    def __init__(self):
        super().__init__('control_punto')
        self.declare_parameter('target_x', 0.5)
        self.declare_parameter('target_y', 0.5)
        
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.error_pub = self.create_publisher(Point, '/tracking_error', 10)
        self.path_pub = self.create_publisher(Point, '/desired_trajectory', 10)
        self.actual_pub = self.create_publisher(Point, '/actual_trajectory', 10)
        
        # Parámetros Kelly & Diaz (Modificado a PID)
        self.h = 0.15      # Distancia de desplazamiento
        self.k_p = 2.5     # Ganancia proporcional
        self.k_i = 0.05    # Ganancia integral
        self.k_d = 0.2     # Ganancia derivativa
        
        # Variables PID
        self.int_e_x = 0.0
        self.int_e_y = 0.0
        self.prev_e_x = 0.0
        self.prev_e_y = 0.0
        
        # --- RESET INTERNO ---
        self.offset_x = None; self.offset_y = None; self.offset_theta = None
        self.x = 0.0; self.y = 0.0; self.theta = 0.0
        
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.control_loop)

    def odom_callback(self, msg):
        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        raw_theta = euler[2]

        if self.offset_x is None:
            self.offset_x = raw_x; self.offset_y = raw_y; self.offset_theta = raw_theta

        dx = raw_x - self.offset_x
        dy = raw_y - self.offset_y
        
        self.x = dx * math.cos(-self.offset_theta) - dy * math.sin(-self.offset_theta)
        self.y = dx * math.sin(-self.offset_theta) + dy * math.cos(-self.offset_theta)
        self.theta = raw_theta - self.offset_theta

    def control_loop(self):
        if self.offset_x is None: return

        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        x_d = self.get_parameter('target_x').get_parameter_value().double_value
        y_d = self.get_parameter('target_y').get_parameter_value().double_value
        
        # Punto de control desplazado (Kelly & Diaz)
        x_c = self.x + self.h * math.cos(self.theta)
        y_c = self.y + self.h * math.sin(self.theta)
        
        e_x = x_d - x_c
        e_y = y_d - y_c
        error_total = math.hypot(e_x, e_y)
        
        dt = 0.05
        # Integrales
        self.int_e_x += e_x * dt
        self.int_e_y += e_y * dt
        
        # Derivadas
        d_e_x = (e_x - self.prev_e_x) / dt
        d_e_y = (e_y - self.prev_e_y) / dt
        
        self.prev_e_x = e_x
        self.prev_e_y = e_y
        
        # Ley de control PID
        u_x = self.k_p * e_x + self.k_i * self.int_e_x + self.k_d * d_e_x
        u_y = self.k_p * e_y + self.k_i * self.int_e_y + self.k_d * d_e_y
        
        # Velocidades diferenciales
        v = u_x * math.cos(self.theta) + u_y * math.sin(self.theta)
        w = (-u_x * math.sin(self.theta) + u_y * math.cos(self.theta)) / self.h
        
        if error_total < 0.03: v = 0.0; w = 0.0
            
        cmd = Twist()
        cmd.linear.x = max(min(float(v), 0.4), -0.4)
        cmd.angular.z = max(min(float(w), 1.2), -1.2)
        self.cmd_pub.publish(cmd)
        
        self.error_pub.publish(Point(x=e_x, y=e_y, z=error_total))
        self.path_pub.publish(Point(x=x_d, y=y_d, z=0.0))
        self.actual_pub.publish(Point(x=x_c, y=y_c, z=0.0))

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ControlPuntoReal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Deteniendo controlador de punto...")
        node.cmd_pub.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()