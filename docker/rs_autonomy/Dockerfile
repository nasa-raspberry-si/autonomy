# Build arguments
# ---------------
# BUILD_COMMAND: the command that should be used to build the Catkin workspace.
#   By default, catkin_make is used, but one may also use "catkin build"
#   instead as a means of avoiding certain build issues.
# DISTRO: specifies the ROS distribution that should be used by the base image

ARG OWIMG
FROM ${OWIMG}

ARG DISTRO
ENV DEBIAN_FRONTEND noninteractive
ENV ROS_DISTRO ${DISTRO}

WORKDIR /ros_ws

## Download Raspberry-SI autonomy code
ARG CODE_BRANCH
RUN git clone --branch ${CODE_BRANCH} https://github.com/nasa-raspberry-si/autonomy.git ./src/rs_autonomy

## Build Raspberry-SI autonomy, rs_autonomy
ARG BUILD_COMMAND=". ~/.profile; catkin build"
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
 && eval "${BUILD_COMMAND} -j8 -DCMAKE_EXPORT_COMPILE_COMMANDS=1 rs_autonomy --no-deps"

ENTRYPOINT ["/ros_ws/entrypoint.sh"]
CMD ["/bin/bash"]
