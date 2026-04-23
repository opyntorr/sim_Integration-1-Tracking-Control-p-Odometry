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
Este nodo hace que el robot siga una trayectoria circular de radio 0.25m.

- **Comando:**
  ```bash
  ros2 run mi_proyecto_sim control_circulo.py
  ```

---
**Nota:** Al terminar la ejecución con `Ctrl+C`, se generarán automáticamente reportes (CSV y gráficas PNG) en la carpeta `src/mi_proyecto_sim/` con los resultados de la prueba.

## ¿Cómo funciona la simulación?

La simulación se basa en tres capas principales que trabajan en conjunto:

1.  **Motor de Física (Ignition Gazebo):** Es el entorno virtual que calcula la gravedad, colisiones y el comportamiento físico de las ruedas. Utiliza el mundo `empty.sdf`.
2.  **Modelado del Robot (Yahboom Description):** El paquete `yahboom_rosmaster_description` proporciona los archivos **URDF/Xacro**. Estos archivos definen la "anatomía" del robot (masas, inercias, sensores y dimensiones exactas). Sin esto, Gazebo no sabría cómo luce ni cómo se mueve el robot.
3.  **El Puente (`ros_gz_bridge`):** Actúa como traductor. Convierte los mensajes nativos de Ignition Gazebo en tópicos de ROS 2. Esto permite que tu código de control envíe comandos de velocidad a `/cmd_vel` y reciba la odometría de `/odom` sin preocuparse de que está en un entorno virtual.

## Traslado al Robot Real

Una de las mayores ventajas de usar ROS 2 es la **abstracción de hardware**. Para llevar este proyecto a un ROSMaster X3 físico, el proceso es el siguiente:

### 1. El Concepto de "Engaño" Positivo
Tu código de control (`control_punto.py` o `control_circulo.py`) no sabe si está hablando con un simulador o con motores reales. Él solo busca un tópico llamado `/cmd_vel` y espera datos en `/odom`. Por lo tanto, **no necesitas modificar la lógica de control**.

### 2. Sustitución de Capas
Para pasar al hardware, simplemente "apagas" la capa de simulación y "enciendes" la capa de hardware:
*   **En simulación:** Corres `simulacion.launch.py` (lanza Gazebo + Puente).
*   **En hardware:** Corres el paquete de "Bringup" oficial del robot (`yahboom_rosmaster_bringup`). Este nodo se encarga de hablar con la placa electrónica mediante los drivers seriales.

### 3. Ajustes Necesarios
Aunque la lógica es la misma, los entornos reales tienen variables que el simulador simplifica:
*   **Tiempo del Sistema:** En el robot real, el parámetro `use_sim_time` debe ser `false` (valor por defecto).
*   **Fricción y Deslizamiento:** Las ruedas mecánicas reales pueden patinar sobre ciertas superficies o tener pequeñas variaciones de motor. Es probable que necesites realizar un "Tuning" (ajuste fino) de la ganancia proporcional `k_p` y la distancia de control `h`.
*   **Frecuencia:** Asegúrate de que la frecuencia del bucle de control sea estable y compatible con la latencia del hardware real.
