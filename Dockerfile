FROM osrf/ros:humble-desktop

# Variables de entorno para evitar prompts interactivos
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Instalar dependencias esenciales, Ignition Gazebo (Fortress), Bridge y utilidades
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ignition-fortress \
    ros-humble-ros-gz \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim \
    ros-humble-teleop-twist-joy \
    ros-humble-joy \
    ros-humble-nav2-map-server \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-slam-toolbox \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-tf-transformations \
    ros-humble-rqt-plot \
    ros-humble-rqt-image-view \
    ros-humble-urdf-tutorial \
    ros-humble-joint-state-publisher-gui \
    ros-humble-gazebo-ros2-control \
    ros-humble-ign-ros2-control \
    ros-humble-controller-manager \
    ros-humble-robot-localization \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-twist-mux \
    ros-humble-apriltag-ros \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-image-proc \
    ros-humble-depthimage-to-laserscan \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    python3-transforms3d \
    python3-matplotlib \
    python3-opencv \
    libopencv-dev \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Configuración de usuario para evitar problemas de permisos (EACCES)
ARG USERNAME=opyntorr
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Crear el workspace y dar permisos
WORKDIR /ros2_ws
RUN chown -R $USERNAME:$USERNAME /ros2_ws

# Cambiar al usuario opyntorr
USER $USERNAME

# Personalizar el Prompt de la terminal: (docker) en Azul Brillante
RUN echo "export PS1='\[\033[01;34m\](docker)\[\033[00m\] \[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '" >> ~/.bashrc

# Agregar fuentes de ROS al bashrc del usuario
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]