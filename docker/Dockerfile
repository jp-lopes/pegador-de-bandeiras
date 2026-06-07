# ==========================================
# Dockerfile ROS2 Humble + Ignition + CycloneDDS
# ==========================================
FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV TERM=xterm-256color

# -------------------------
# Instala utilitários e dependências de sistema
# -------------------------
RUN apt update && apt install -y \
    evince dbus-x11 python3-colcon-common-extensions python3-rosdep python3-vcstool \
    git wget curl build-essential sudo gedit lsb-release gnupg2 bash-completion vim \
    libcanberra-gtk-module libcanberra-gtk3-module \
    && rm -rf /var/lib/apt/lists/*

# Inicializa rosdep
RUN rosdep init || true
RUN rosdep update

# -------------------------
# Dependências ROS 2
# -------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-dev ros-humble-rmw-cyclonedds-cpp ros-humble-topic-tools \
    ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers \
    ros-humble-ign-ros2-control ros-humble-gz-ros2-control \
    ros-humble-joint-state-publisher-gui ros-humble-tf-transformations \
    ros-humble-tf2-tools ros-humble-tf2-geometry-msgs python3-transforms3d \
    ros-humble-joint-state-publisher ros-humble-robot-state-publisher \
    ros-humble-xacro ros-humble-ros-ign-bridge ros-humble-ros-ign-gazebo libserial-dev \
    && rm -rf /var/lib/apt/lists/*

# -------------------------
# Instala pacotes Python
# -------------------------
RUN pip3 install --no-cache-dir transforms3d flask pyserial flask-ask-sdk ask-sdk

# -------------------------
# Clona o pymoveit2 como pacote ROS 2
# -------------------------
RUN mkdir -p /opt/ros2_deps/src \
    && git clone https://github.com/AndrejOrsula/pymoveit2.git /opt/ros2_deps/src/pymoveit2 \
    && . /opt/ros/humble/setup.sh \
    && cd /opt/ros2_deps \
    && rosdep install --from-paths src --ignore-src -r -y \
    && colcon build --symlink-install \
    && echo "source /opt/ros2_deps/install/setup.bash" >> /etc/bash.bashrc

# -------------------------
# Ignition Fortress
# -------------------------
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/ignition-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ignition-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ignition-stable.list > /dev/null \
    && apt update && apt install -y ignition-fortress \
    && rm -rf /var/lib/apt/lists/*

# -------------------------
# Configuração de Usuário Genérico (user)
# -------------------------
ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME \
    && usermod -aG sudo $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

RUN mkdir -p /home/$USERNAME/ros2_ws/src \
    && chown -R $USERNAME:$USERNAME /home/$USERNAME/ros2_ws

# -------------------------
# Troca para usuário
# -------------------------
USER $USERNAME
WORKDIR /home/$USERNAME/ros2_ws

# -------------------------
# Configurações do ambiente (DDS, ROS, Aliases)
# -------------------------
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV RCUTILS_COLORIZED_OUTPUT=1
ENV COLCON_FORCE_COLOR=1
ENV RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"

RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "source /opt/ros2_deps/install/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/$USERNAME/.bashrc && \
    echo "alias cb='colcon build --symlink-install'" >> /home/$USERNAME/.bashrc && \
    echo "if [ -f /home/$USERNAME/ros2_ws/install/setup.bash ]; then source /home/$USERNAME/ros2_ws/install/setup.bash; fi" >> /home/$USERNAME/.bashrc && \
    echo 'export PS1="\[\e[1;32m\]\u@\h:\[\e[1;34m\]\w\[\e[0m\]\$ "' >> /home/$USERNAME/.bashrc

CMD ["bash"]