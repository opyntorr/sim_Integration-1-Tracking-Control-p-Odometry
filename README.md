# Demo ROSMaster Simulation

Este repositorio contiene el entorno de simulación para el robot Yahboom ROSMaster X3 utilizando ROS 2 Humble y Docker.

## Requisitos

- Docker
- Docker Compose v2
- NVIDIA Docker Toolkit (para aceleración por hardware)

## Inicio rápido

1. Construir e iniciar el contenedor:
   ```bash
   docker compose up -d --build
   ```

2. Entrar al contenedor:
   ```bash
   docker exec -it rosmaster_sim bash
   ```

3. (Dentro del contenedor) Compilar el workspace:
   ```bash
   colcon build --symlink-install
   ```

## Estructura del Proyecto

- `src/`: Código fuente de los paquetes ROS 2.
- `docker-compose.yml`: Configuración del entorno Docker.
- `Dockerfile`: Definición de la imagen base de ROS 2.

## Instrucciones de Control

Una vez dentro del contenedor y con el workspace compilado, puedes ejecutar los siguientes nodos de control:

### 1. Control hacia un Punto Fijo
Este nodo mueve el robot hacia una posición `(x, y)` específica utilizando la ley de control de Kelly & Diaz.

- **Comando básico:**
  ```bash
  ros2 run mi_proyecto_sim control_punto.py
  ```

- **Mandar una posición específica desde la terminal:**
  Para cambiar el punto de destino (por ejemplo a X=3.0, Y=1.5), usa parámetros de ROS 2:
  ```bash
  ros2 run mi_proyecto_sim control_punto.py --ros-args -p target_x:=3.0 -p target_y:=1.5
  ```

### 2. Seguimiento de Trayectoria Circular
Este nodo hace que el robot siga una trayectoria circular de radio 2.0m.

- **Comando:**
  ```bash
  ros2 run mi_proyecto_sim control_circulo.py
  ```

---
**Nota:** Al terminar la ejecución con `Ctrl+C`, se generarán automáticamente reportes (CSV y gráficas PNG) en la carpeta `src/mi_proyecto_sim/` con los resultados de la prueba.
