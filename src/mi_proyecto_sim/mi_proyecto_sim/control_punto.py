#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import math
import tf_transformations
import csv
import matplotlib.pyplot as plt
import os
from datetime import datetime
from rclpy.signals import SignalHandlerOptions

class ControlPuntoReal(Node):
    def __init__(self):
        super().__init__('control_punto')
        self.declare_parameter('target_x', 2.0)
        self.declare_parameter('target_y', 2.0)
        
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.error_pub = self.create_publisher(Point, '/tracking_error', 10)
        self.path_pub = self.create_publisher(Point, '/desired_trajectory', 10)
        self.actual_pub = self.create_publisher(Point, '/actual_trajectory', 10)
        
        # Parámetros Kelly & Diaz
        self.h = 0.15      # Distancia de desplazamiento
        self.k_p = 1.5     # Ganancia proporcional
        
        # --- RESET INTERNO ---
        self.offset_x = None; self.offset_y = None; self.offset_theta = None
        self.x = 0.0; self.y = 0.0; self.theta = 0.0
        
        # --- HISTORIAL PARA REPORTES ---
        self.time_history = []
        self.error_history = []
        self.xc_history = [] # Trayectoria real del punto h
        self.yc_history = []
        
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
        
        # --- GUARDAR DATOS ---
        self.time_history.append(t)
        self.error_history.append(error_total)
        self.xc_history.append(x_c)
        self.yc_history.append(y_c)
        
        # Ley de control
        u_x = self.k_p * e_x
        u_y = self.k_p * e_y
        
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

    def exportar_datos(self):
        dir_path = "/ros2_ws/src/mi_proyecto_sim/"
        target_x = self.get_parameter('target_x').get_parameter_value().double_value
        target_y = self.get_parameter('target_y').get_parameter_value().double_value
        
        # 1. Guardar CSV fijo
        csv_file = os.path.join(dir_path, "ultimo_reporte_punto.csv")
        with open(csv_file, mode='w') as f:
            writer = csv.writer(f)
            writer.writerow(["Time", "Error"])
            for row in zip(self.time_history, self.error_history): 
                writer.writerow(row)

        # 2. Guardar PNG de Error
        plt.figure(figsize=(10, 4))
        plt.plot(self.time_history, self.error_history, color='green', label='Error')
        plt.title(f'Regulación hacia ({target_x}, {target_y})')
        plt.xlabel('Tiempo [s]'); plt.ylabel('Error [m]'); plt.grid(True)
        plt.savefig(os.path.join(dir_path, "ultima_grafica_error_punto.png"))
        plt.close()

        # 3. Guardar PNG de Trayectoria XY
        plt.figure(figsize=(8, 8))
        plt.plot(0, 0, 'go', label='Inicio')
        plt.plot(target_x, target_y, 'rx', label='Meta')
        plt.plot(self.xc_history, self.yc_history, 'b-', label='Trayecto')
        plt.title('Plano XY - Punto Fijo')
        plt.axis('equal'); plt.grid(True); plt.legend()
        plt.savefig(os.path.join(dir_path, "ultima_trayectoria_punto.png"))
        plt.close()
        
        self.get_logger().info(f"✅ Reportes de PUNTO actualizados en: {dir_path}")

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ControlPuntoReal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Generando archivos finales...")
        node.cmd_pub.publish(Twist())
        node.exportar_datos()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()