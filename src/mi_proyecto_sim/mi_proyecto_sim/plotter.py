#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import csv
import os
from rclpy.signals import SignalHandlerOptions

class RosmasterPlotter(Node):
    def __init__(self):
        super().__init__('rosmaster_plotter')
        
        # Suscriptores
        self.error_sub = self.create_subscription(Point, '/tracking_error', self.error_callback, 10)
        self.desired_sub = self.create_subscription(Point, '/desired_trajectory', self.desired_callback, 10)
        self.actual_sub = self.create_subscription(Point, '/actual_trajectory', self.actual_callback, 10)
        
        # Historial de datos
        self.time_history = []
        self.error_history = []
        self.xd_history = []
        self.yd_history = []
        self.xc_history = []
        self.yc_history = []
        
        self.start_time = None
        
        # Últimas lecturas recibidas
        self.current_xd = 0.0; self.current_yd = 0.0
        self.current_xc = 0.0; self.current_yc = 0.0
        
        # Configuración de gráfica en tiempo real
        plt.ion()
        self.fig = plt.figure(figsize=(12, 6))
        gs = gridspec.GridSpec(1, 2, figure=self.fig)
        
        # Subplot Trayectoria XY
        self.ax_xy = self.fig.add_subplot(gs[0, 0])
        self.ax_xy.set_title('Trayectoria XY (Tiempo Real)')
        self.ax_xy.set_xlabel('X [m]')
        self.ax_xy.set_ylabel('Y [m]')
        self.ax_xy.plot([0], [0], 'go', label='Inicio')
        self.line_d, = self.ax_xy.plot([], [], 'r--', label='Deseada')
        self.line_c, = self.ax_xy.plot([], [], 'b-', label='Real')
        self.ax_xy.legend()
        self.ax_xy.grid(True)
        self.ax_xy.axis('equal')
        
        # Subplot Error vs Tiempo
        self.ax_err = self.fig.add_subplot(gs[0, 1])
        self.ax_err.set_title('Error de Seguimiento')
        self.ax_err.set_xlabel('Tiempo [s]')
        self.ax_err.set_ylabel('Error [m]')
        self.line_err, = self.ax_err.plot([], [], 'g-', label='Error Total')
        self.ax_err.grid(True)
        self.ax_err.legend()
        
        self.fig.tight_layout()
        plt.show()

        # Timer para actualizar la gráfica (10 Hz)
        self.timer = self.create_timer(0.1, self.update_plot)
        self.messages_received = 0

    def desired_callback(self, msg):
        self.current_xd = msg.x
        self.current_yd = msg.y

    def actual_callback(self, msg):
        self.current_xc = msg.x
        self.current_yc = msg.y

    def error_callback(self, msg):
        current_time = self.get_clock().now()
        
        if self.start_time is None:
            self.start_time = current_time
            
        t = (current_time - self.start_time).nanoseconds / 1e9
        
        # Guardar en el historial usando las lecturas actuales (suponemos que están sincronizadas)
        self.time_history.append(t)
        self.error_history.append(msg.z)
        self.xd_history.append(self.current_xd)
        self.yd_history.append(self.current_yd)
        self.xc_history.append(self.current_xc)
        self.yc_history.append(self.current_yc)
        
        self.messages_received += 1

    def update_plot(self):
        if not self.time_history:
            return
            
        # Actualizar gráfica en tiempo real (cada 2 mensajes recibidos)
        if self.messages_received % 2 == 0:
            try:
                # Actualizar XY
                self.line_d.set_data(self.xd_history, self.yd_history)
                self.line_c.set_data(self.xc_history, self.yc_history)
                
                # Ajustar límites de XY
                all_x = self.xd_history + self.xc_history + [0]
                all_y = self.yd_history + self.yc_history + [0]
                if all_x and all_y:
                    min_x, max_x = min(all_x), max(all_x)
                    min_y, max_y = min(all_y), max(all_y)
                    margen_x = max(0.5, (max_x - min_x) * 0.1)
                    margen_y = max(0.5, (max_y - min_y) * 0.1)
                    self.ax_xy.set_xlim([min_x - margen_x, max_x + margen_x])
                    self.ax_xy.set_ylim([min_y - margen_y, max_y + margen_y])
                
                # Actualizar Error
                self.line_err.set_data(self.time_history, self.error_history)
                
                max_t = max(5.0, self.time_history[-1])
                self.ax_err.set_xlim([0, max_t])
                
                if self.error_history:
                    min_e, max_e = min(self.error_history), max(self.error_history)
                    self.ax_err.set_ylim([max(0, min_e - 0.1), max_e + 0.1])
                
                self.fig.canvas.draw()
                self.fig.canvas.flush_events()
            except Exception as e:
                pass

    def exportar_datos(self):
        dir_path = "/ros2_ws/src/mi_proyecto_sim/"
        
        # 1. Guardar CSV fijo
        csv_file = os.path.join(dir_path, "ultimo_reporte_rosmaster.csv")
        with open(csv_file, mode='w') as f:
            writer = csv.writer(f)
            writer.writerow(["Time", "Error_Total", "X_Desired", "Y_Desired", "X_Actual", "Y_Actual"])
            for row in zip(self.time_history, self.error_history, self.xd_history, self.yd_history, self.xc_history, self.yc_history): 
                writer.writerow(row)

        # 2. Guardar PNG de Error
        plt.figure(figsize=(10, 4))
        plt.plot(self.time_history, self.error_history, color='green', label='Error')
        plt.title('Error de Seguimiento')
        plt.xlabel('Tiempo [s]')
        plt.ylabel('Error [m]')
        plt.grid(True)
        plt.legend()
        plt.savefig(os.path.join(dir_path, "ultima_grafica_error_rosmaster.png"))
        plt.close()

        # 3. Guardar PNG de Trayectoria XY
        plt.figure(figsize=(8, 8))
        plt.plot(0, 0, 'go', label='Inicio')
        plt.plot(self.xd_history, self.yd_history, 'r--', label='Deseada')
        plt.plot(self.xc_history, self.yc_history, 'b-', label='Real')
        plt.title('Plano XY - Trayectoria')
        plt.axis('equal')
        plt.grid(True)
        plt.legend()
        plt.savefig(os.path.join(dir_path, "ultima_trayectoria_rosmaster.png"))
        plt.close()
        
        self.get_logger().info(f"✅ Reportes guardados en: {dir_path}")

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = RosmasterPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Deteniendo graficador y guardando archivos...")
        node.exportar_datos()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
