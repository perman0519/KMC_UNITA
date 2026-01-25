# 명시적으로 linux/arm64 플랫폼을 사용하도록 설정
FROM ros:foxy-ros-base-focal

SHELL ["/bin/bash", "-c"]


# =========================
# 1) Base (ROS 2 Foxy) + simulator dependencies
# =========================
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=foxy
ENV ROS_WS=/ws
WORKDIR ${ROS_WS}

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-rosdep \
    python3-colcon-common-extensions \
    build-essential \
    cmake \
    git \
    pkg-config \
    libeigen3-dev \
    libglm-dev \
    nlohmann-json3-dev \
    libyaml-cpp-dev \
    libx11-dev \
    libxext-dev \
    libxrandr-dev \
    libxcursor-dev \
    libxfixes-dev \
    libxi-dev \
    libudev-dev \
    libgl-dev \
  && rm -rf /var/lib/apt/lists/* \
  && rosdep init || true \
  && rosdep update

# =========================
# 1-1) Example dependencies (pkg_example_2)
#    TODO (TEAM): 필요 없으면 제거
# =========================
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-aiohttp \
  && rm -rf /var/lib/apt/lists/*


# =========================
# 1-1) Team dependencies (pure_pursuit_controller)
# =========================
RUN apt-get update && apt-get install -y --no-install-recommends \
    nlohmann-json3-dev \
    libyaml-cpp-dev \
    ros-foxy-geometry-msgs \
    ros-foxy-nav-msgs \
    ros-foxy-tf2 \
    ros-foxy-tf2-ros \
    ros-foxy-visualization-msgs \
    ros-foxy-ament-index-cpp \
    ros-foxy-std-msgs \
  && rm -rf /var/lib/apt/lists/*


# =========================
# 2) simulator: install SDL3
# =========================
RUN git clone --branch release-3.2.2 --depth 1 https://github.com/libsdl-org/SDL.git /tmp/SDL \
  && cmake -S /tmp/SDL -B /tmp/SDL/build \
      -DCMAKE_BUILD_TYPE=Release \
      -DSDL_STATIC=OFF \
      -DSDL_SHARED=ON \
      -DSDL_TESTS=OFF \
  && cmake --build /tmp/SDL/build -j"$(nproc)" \
  && cmake --install /tmp/SDL/build \
  && ldconfig \
  && rm -rf /tmp/SDL

RUN mkdir -p ${ROS_WS}/src

# =========================
# 3) Simulator packages
#    - Copy only package.xml
# =========================
COPY Mobility_Challenge_Simulator/src/communication_manager/package.xml ${ROS_WS}/src/communication_manager/package.xml
COPY Mobility_Challenge_Simulator/src/domain_bridge/package.xml ${ROS_WS}/src/domain_bridge/package.xml
COPY Mobility_Challenge_Simulator/src/hv_handler/package.xml ${ROS_WS}/src/hv_handler/package.xml
COPY Mobility_Challenge_Simulator/src/scene_srv/package.xml ${ROS_WS}/src/scene_srv/package.xml
COPY Mobility_Challenge_Simulator/src/simulator/package.xml ${ROS_WS}/src/simulator/package.xml
COPY Mobility_Challenge_Simulator/src/simulator_launch/package.xml ${ROS_WS}/src/simulator_launch/package.xml
# 기존의 4줄짜리 COPY를 지우고 아래 한 줄로 대체
COPY Mobility_Challenge_Simulator/profile* ${ROS_WS}/
# =========================
# 4) Example packages (pkg_example_*)
#    TODO (TEAM): pkg_example_*를 참가팀 패키지로 교체
# =========================
# COPY pkg_example_1/package.xml ${ROS_WS}/src/pkg_example_1/package.xml
# COPY pkg_example_2/package.xml ${ROS_WS}/src/pkg_example_2/package.xml
COPY pure_pursuit_controller/package.xml ${ROS_WS}/src/pure_pursuit_controller/package.xml
COPY map_visualizer/package.xml ${ROS_WS}/src/map_visualizer/package.xml

# =========================
# 5) Install dependencies with rosdep
# =========================
RUN apt-get update \
  && rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO} --skip-keys python3-aiohttp \
  && rm -rf /var/lib/apt/lists/*

# =========================
# 6) Copy sources
# =========================
# simulator packages
COPY Mobility_Challenge_Simulator/src/ ${ROS_WS}/src/

# # Example packages (pkg_example_*)
# # TODO (TEAM): pkg_example_*를 참가팀 패키지로 교체
# COPY pkg_example_1/ ${ROS_WS}/src/pkg_example_1/
# COPY pkg_example_2/ ${ROS_WS}/src/pkg_example_2/

# TODO (TEAM): Copy your team packages source code
COPY pure_pursuit_controller/ ${ROS_WS}/src/pure_pursuit_controller/
COPY map_visualizer/ ${ROS_WS}/src/map_visualizer/
# COPY src/ ${ROS_WS}/src/   # If you keep your packages under ./src


COPY entrypoint.sh /entrypoint.sh
RUN sed -i 's/\r$//' /entrypoint.sh && chmod +x /entrypoint.sh

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
  && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

# # VNC 및 데스크탑 환경 설치
# RUN apt-get update && apt-get install -y \
#     xfce4 xfce4-goodies \
#     tightvncserver \
#     novnc python3-websockify \
#     && rm -rf /var/lib/apt/lists/*

# # VNC 서버 실행 스크립트 설정 (생략 가능, 수동 실행 가능)
# # =========================
# # 5-1) GUI/VNC environment
# # =========================
# RUN apt-get update && apt-get install -y --no-install-recommends \
#     xvfb \
#     x11vnc \
#     novnc \
#     python3-websockify \
#     fluxbox \
#     mesa-utils \
#     libgl1-mesa-dri \
#   && rm -rf /var/lib/apt/lists/*

# # noVNC 포트 설정
# EXPOSE 6080

ENV ROS_LOCALHOST_ONLY=0
ENTRYPOINT ["/entrypoint.sh"]
