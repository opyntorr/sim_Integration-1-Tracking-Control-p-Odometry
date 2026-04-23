cmake_minimum_required(VERSION 3.8)
project(mi_proyecto_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Encontrar dependencias de ROS 2
# Eliminamos OpenCV, rclcpp y std_msgs porque ya no compilamos C++
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 2. Reglas de Instalación

# Instalar el script de Python del Control de Kelly para que ros2 run/launch lo encuentre
install(PROGRAMS
  mi_proyecto_sim/control_circulo.py
  mi_proyecto_sim/control_punto.py
  mi_proyecto_sim/plotter.py
  DESTINATION lib/${PROJECT_NAME}
)

# Instalar directorios de recursos visuales y de simulación
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()