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

# Install PRISM
WORKDIR /root
ENV PRISM_VER=4.6
COPY prism-${PRISM_VER}-linux64.tar.gz /root
RUN tar zxf prism-${PRISM_VER}-linux64.tar.gz && \
    cd prism-${PRISM_VER}-linux64 && \
    ./install.sh && \
    echo "export LD_LIBRARY_PATH=/root/prism-${PRISM_VER}-linux64/lib:\$LD_LIBRARY_PATH" >> ~/.bashrc && \
    cd .. && \
    rm -f prism-${PRISM_VER}-linux64.tar.gz && \
    ln -s /root/prism-${PRISM_VER}-linux64/bin/prism /usr/bin/prism


## Download Raspberry-SI autonomy code
WORKDIR /ros_ws
ARG CODE_BRANCH
RUN git clone --branch ${CODE_BRANCH} https://github.com/nasa-raspberry-si/autonomy.git ./src/rs_autonomy

## Build Raspberry-SI autonomy, rs_autonomy
ARG BUILD_COMMAND=". ~/.profile; catkin build"
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
 && eval "${BUILD_COMMAND} -j8 -DCMAKE_EXPORT_COMPILE_COMMANDS=1 rs_autonomy --no-deps"

ENTRYPOINT ["/ros_ws/entrypoint.sh"]
CMD ["/bin/bash"]
