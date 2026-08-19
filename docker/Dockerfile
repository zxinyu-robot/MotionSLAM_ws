# MotionSLAM 自研导航栈运行环境
# 宿主机: Jetson Orin NX / Ubuntu 20.04 (JetPack 5, kernel 5.10) — 不满足 Super-LIO ros2 分支要求
# 容器:   Ubuntu 22.04 + ROS2 Humble + C++20 工具链, 纯 CPU 负载, 无需 GPU 运行时
FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# 端侧 apt 清华源（arm64 用 ubuntu-ports）
RUN sed -i 's|http://.*archive.ubuntu.com|https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports|g; \
            s|http://.*security.ubuntu.com|https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports|g' \
        /etc/apt/sources.list.d/ubuntu.sources 2>/dev/null || \
    sed -i 's|http://.*archive.ubuntu.com|https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports|g; \
            s|http://.*security.ubuntu.com|https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports|g' \
        /etc/apt/sources.list || true
RUN if [ -f /etc/apt/sources.list.d/ros2.list ]; then \
      sed -i 's|https://.*ros2/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu|g' \
        /etc/apt/sources.list.d/ros2.list; \
    fi

# ---- 系统依赖: Super-LIO(glog/TBB/PCL/Eigen) + CycloneDDS RMW + Foxglove ----
# dev 不装 Nav2 全栈；保留 nav2-msgs 供 verify_h5_arrival.py (Hybrid 对照脚本)
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git wget \
    libgoogle-glog-dev libtbb-dev \
    libpcl-dev libeigen3-dev \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-nav2-msgs \
    ros-humble-pcl-ros ros-humble-pcl-conversions \
    ros-humble-foxglove-bridge \
    ros-humble-rosx-introspection \
    libasio-dev libwebsocketpp-dev \
    ros-humble-tf2-tools ros-humble-tf-transformations \
    ros-humble-rosbag2-storage-mcap \
    && rm -rf /var/lib/apt/lists/*

# ---- Legacy foxglove_bridge 0.8.2 (foxglove.websocket.v1, 端口 8766) ----
RUN git clone --depth 1 --branch 0.8.2 https://github.com/foxglove/ros-foxglove-bridge.git /tmp/fb_legacy \
    && . /opt/ros/humble/setup.bash \
    && cd /tmp/fb_legacy \
    && colcon build --merge-install --install-base /opt/foxglove_legacy --cmake-args -DBUILD_TESTING=OFF \
    && rm -rf /tmp/fb_legacy

# unitree_ros2 消息包编译依赖 (单独一层, 避免改动上方大包列表导致缓存失效)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rosidl-generator-dds-idl \
    libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

# ---- Livox-SDK2 (livox_ros_driver2 的前置依赖, mid360) ----
RUN git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git /tmp/Livox-SDK2 \
    && cmake -S /tmp/Livox-SDK2 -B /tmp/Livox-SDK2/build \
    && cmake --build /tmp/Livox-SDK2/build -j"$(nproc)" \
    && cmake --install /tmp/Livox-SDK2/build \
    && rm -rf /tmp/Livox-SDK2

# ---- DDS 与宇树域 0 原生互通 ----
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=/ws/docker/cyclonedds.xml
ENV ROS_DOMAIN_ID=0

# colcon 工作区挂载点 (run_container.sh 中 -v 挂入)
WORKDIR /ws

# 交互进入时自动 source 环境
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
    && echo '[ -f /opt/foxglove_legacy/setup.bash ] && source /opt/foxglove_legacy/setup.bash' >> /root/.bashrc \
    && echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /root/.bashrc

CMD ["bash"]
