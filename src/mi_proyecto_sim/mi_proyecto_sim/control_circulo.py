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
        self.time_history = []; self.error_history = []
        
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.control_loop)

        # --- NUEVAS LISTAS PARA EL PLANO XY ---
        self.xd_history = []
        self.yd_history = []
        self.xc_history = []
        self.yc_history = []
        # Mantener las de error
        self.time_history = []
        self.error_history = []

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
        R = 2.0; w_d = 0.15
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
        
        # --- GUARDAR EN HISTORIAL ---
        self.xd_history.append(x_d)
        self.yd_history.append(y_d)
        self.xc_history.append(x_c)
        self.yc_history.append(y_c)
        
        # Guardar error y tiempo
        self.time_history.append(t)
        self.error_history.append(error_total)
        
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

    def exportar_datos(self):
        # Eliminamos el timestamp para que el nombre sea fijo
        dir_path = "/ros2_ws/src/mi_proyecto_sim/"
        
        # 1. Guardar CSV (Sobreescribe siempre el mismo archivo)
        csv_file = os.path.join(dir_path, "ultimo_reporte_circulo.csv")
        with open(csv_file, mode='w') as f:
            writer = csv.writer(f)
            writer.writerow(["Time", "Error"])
            for row in zip(self.time_history, self.error_history): 
                writer.writerow(row)

        # 2. Guardar PNG de Error
        plt.figure(figsize=(10, 5))
        plt.plot(self.time_history, self.error_history, color='blue', label='Error de Trayectoria')
        plt.title('Control de Seguimiento Circular (Kelly & Diaz)')
        plt.xlabel('Tiempo [s]'); plt.ylabel('Error [m]'); plt.grid(True)
        plt.savefig(os.path.join(dir_path, "ultima_grafica_error_circulo.png"))
        plt.close()

        # 3. Guardar PNG de Trayectoria XY
        plt.figure(figsize=(8, 8))
        plt.plot(self.xd_history, self.yd_history, 'r--', label='Deseada')
        plt.plot(self.xc_history, self.yc_history, 'b-', label='Real')
        plt.title('Plano XY - Círculo')
        plt.axis('equal'); plt.grid(True); plt.legend()
        plt.savefig(os.path.join(dir_path, "ultima_trayectoria_circulo.png"))
        plt.close()
        
        self.get_logger().info(f"✅ Reportes de CÍRCULO actualizados en: {dir_path}")

def main(args=None):
    # Usar SignalHandlerOptions.NO para evitar el crash de contexto al dar Ctrl+C
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ControlCirculoReal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Deteniendo prueba circular...")
        node.cmd_pub.publish(Twist())
        node.exportar_datos()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()