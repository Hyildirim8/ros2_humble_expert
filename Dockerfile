# ROS 2 Humble geliştirme ortamı
FROM osrf/ros:humble-desktop-full

# Gerekli araçları yükle
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    vim \
    nano \
    git \
    wget \
    curl \
    build-essential \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Gazebo ve simülasyon araçları
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    gazebo \
    && rm -rf /var/lib/apt/lists/*

# Navigation2 stack
RUN apt-get update && apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    && rm -rf /var/lib/apt/lists/*

# Görselleştirme ve robot araçları
RUN apt-get update && apt-get install -y \
    ros-humble-rviz2 \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-tf2-tools \
    ros-humble-urdf-tutorial \
    && rm -rf /var/lib/apt/lists/*

# Kamera ve görüntü işleme
RUN apt-get update && apt-get install -y \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-camera-info-manager \
    ros-humble-vision-opencv \
    ros-humble-cv-bridge \
    ros-humble-image-pipeline \
    && rm -rf /var/lib/apt/lists/*

# Teleop ve kontrol
RUN apt-get update && apt-get install -y \
    ros-humble-teleop-twist-keyboard \
    ros-humble-teleop-twist-joy \
    ros-humble-joy \
    ros-humble-controller-manager \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

# Diagnostic ve log araçları
RUN apt-get update && apt-get install -y \
    ros-humble-diagnostic-updater \
    ros-humble-rosbag2 \
    ros-humble-rosbag2-storage-default-plugins \
    && rm -rf /var/lib/apt/lists/*

# Python geliştirme araçları
RUN apt-get update && apt-get install -y \
    python3-pytest \
    python3-pytest-cov \
    python3-numpy \
    python3-matplotlib \
    python3-scipy \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# Host kullanıcısıyla eşleşen kullanıcı oluştur
ARG USER_ID=1000
ARG GROUP_ID=1000
ARG USERNAME=master

# Grup ve kullanıcı oluştur
RUN groupadd -g ${GROUP_ID} ${USERNAME} || true && \
    useradd -l -u ${USER_ID} -g ${USERNAME} -m -s /bin/bash ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Kullanıcıya geç
USER ${USERNAME}

# Çalışma dizinini oluştur
WORKDIR /workspace

# ROS 2 ortamını bashrc'ye ekle
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Workspace'i otomatik source et (varsa)
RUN echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> ~/.bashrc

# Varsayılan komut
CMD ["/bin/bash"]
