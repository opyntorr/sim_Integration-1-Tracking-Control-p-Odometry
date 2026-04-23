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
