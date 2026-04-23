import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, AppendEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Rutas de los paquetes
    pkg_description = get_package_share_directory('yahboom_rosmaster_description')
    
    # Archivo Xacro del robot oficial [cite: 1]
    xacro_file = os.path.join(pkg_description, 'urdf', 'robots', 'rosmaster_x3.urdf.xacro')

    # 1. Variables de entorno para Gazebo
    plugin_env = AppendEnvironmentVariable(
        name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        value='/opt/ros/humble/lib'
    )

    # 2. Lanzar Gazebo (Mundo vacío)
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', 'empty.sdf'],
        output='screen'
    )

    # 3. Puente de ROS 2 a Gazebo
    puente = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/model/rosmaster_x3/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/model/rosmaster_x3/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'
        ],
        remappings=[
            ('/model/rosmaster_x3/odometry', '/odom'),
            ('/model/rosmaster_x3/tf', '/tf')
        ],
        output='screen'
    )

    # 4. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': ParameterValue(Command(['xacro ', xacro_file, ' use_gazebo:=true']), value_type=str)
        }]
    )

    # 5. Spawner de Gazebo
    spawner = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'rosmaster_x3',
            '-topic', 'robot_description',
            '-z', '0.1'
        ],
        output='screen'
    )

    # 6. RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '/ros2_ws/src/mi_proyecto_sim/configuracion.rviz'],
        parameters=[{'use_sim_time': True}]
    )

    # 7. GRÁFICAS DE RQT (Configuradas para esperar datos)
    # 8. GRÁFICA 1: Error de seguimiento (Accediendo al campo .z del mensaje Point)
    plot_error = Node(
        package='rqt_plot',
        executable='rqt_plot',
        name='plot_error',
        # Agregamos /z al final para que rqt_plot sepa qué valor graficar
        arguments=['/tracking_error/z'],
        output='screen'
    )

    # 9. GRÁFICA 2: Comparativa de Coordenadas (Accediendo a campos .x y .y)
    plot_trayectoria = Node(
        package='rqt_plot',
        executable='rqt_plot',
        name='plot_trayectoria',
        arguments=[
            '/desired_trajectory/x', 
            '/actual_trajectory/x',
            '/desired_trajectory/y', 
            '/actual_trajectory/y'
        ],
        output='screen'
    )

    return LaunchDescription([
        plugin_env,
        gazebo,
        puente,
        robot_state_publisher,
        spawner,
        rviz_node,
        plot_error,
        plot_trayectoria
    ])