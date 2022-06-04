# Build arguments
# ---------------
# BUILD_COMMAND: the command that should be used to build the Catkin workspace.
#   By default, catkin_make is used, but one may also use "catkin build"
#   instead as a means of avoiding certain build issues.
# DISTRO: specifies the ROS distribution that should be used by the base image
# DIRECTORY: specifies the directory that provides files for the robot
#
# References
# ----------
# * http://blog.fx.lv/2017/08/running-gui-apps-in-docker-containers-using-vnc
# * https://qxf2.com/blog/view-docker-container-display-using-vnc-viewer
# * https://github.com/ConSol/docker-headless-vnc-container/blob/master/src/ubuntu/install/libnss_wrapper.sh
# * https://www.digitalocean.com/community/tutorials/how-to-install-and-configure-vnc-on-ubuntu-18-04
# * https://hackernoon.com/installation-of-vnc-server-on-ubuntu-1cf035370bd3
# * https://www.tecmint.com/install-and-configure-vnc-server-on-ubuntu
# * https://stackoverflow.com/questions/48601146/docker-how-to-set-tightvncserver-password

ARG DISTRO
FROM osrf/ros:${DISTRO}-desktop-full

ARG DISTRO
ENV DEBIAN_FRONTEND noninteractive
ENV ROS_DISTRO ${DISTRO}

# Dependencies for glvnd and X11.
RUN apt-get update \
  && apt-get install -y -qq --no-install-recommends \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
  && rm -rf /var/lib/apt/lists/*

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all 
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

# NOTE: We need to install the cmake_modules to avoid some build failures
# that are due to unspecified dependencies.
# (https://github.com/ros-industrial/industrial_calibration/issues/50)
# supervisor
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
COPY rootfs /
WORKDIR /ros_ws

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      apt-utils \
      bzip2 \
      cmake \
      clang \
      build-essential \
      ca-certificates \
      curl \
      gcc \
      g++ \
      "ros-${ROS_DISTRO}-cmake-modules" \
      software-properties-common \
      vim \
      wget \
      git \
 && echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros-latest.list \
 && wget http://packages.ros.org/ros.key -O - | apt-key add - \
 && apt-get update \
 && apt-get upgrade -y \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

# build package
## Install dependent packages
ARG DIRECTORY
COPY "${DIRECTORY}"/pkgs.rosinstall "${DIRECTORY}"/apt.list* /.dockerinstall/
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
 && apt-get update \
 && (test -f /.dockerinstall/apt.list \
     && xargs -a /.dockerinstall/apt.list apt-get install -y --no-install-recommends \
     || exit 0) \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

## Download Top-level repos
RUN mv /.dockerinstall/pkgs.rosinstall /ros_ws/pkgs.rosinstall \
 && apt-get update \
 && wstool init -j8 src pkgs.rosinstall \
 && rosdep fix-permissions \
 && rosdep update \
 && rosdep install -i -y -r --from-paths src \
      --ignore-src \
      --skip-keys="python3-rosdep python3-catkin-pkg python3-rospkg" \
      --rosdistro="${ROS_DISTRO}" \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

## Build PLEXIL and GSAP
COPY "${DIRECTORY}" /.dockerinstall/
RUN ls -al /.dockerinstall
RUN (test -f /.dockerinstall/prebuild.sh \
     && ((echo "running prebuild step..." && apt-get update && /bin/bash /.dockerinstall/prebuild.sh && apt-get clean && rm -rf /var/lib/apt/lists/* )|| exit 1) \
     || (echo "skipping prebuild step [no prebuild.sh]" && exit 0))

## Build  ow_simulator, ow_autonomy and other related projects
ARG BUILD_COMMAND=". ~/.profile; catkin build"
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
 && eval "${BUILD_COMMAND} -j8 -DCMAKE_EXPORT_COMPILE_COMMANDS=1"


ENTRYPOINT ["/ros_ws/entrypoint.sh"]
CMD ["/bin/bash"]


