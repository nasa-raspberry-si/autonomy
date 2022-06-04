# Build a docker image for OceanWATERS with the support of OpenGL and X11 for supporting the rendering in Gazebo
   * Reference: [OpenGL and CUDA Applications in Docker](https://medium.com/@benjamin.botto/opengl-and-cuda-applications-in-docker-af0eece000f1)

   * Addtional part added to the beginning of the Dockerfile
   ```
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
   ```

# Build And Test
## Build the docker image
`./build.sh <ROS_DISTRO> <IMAGE_VERSION> <DOCKERHUB_ORG>`
<br>Inside the directory, *versions*, there should be a subdirectory with the name, <IMAGE_VERSION>
<br>Currently, there are two verions available in the repo, which correspond to the release 8 and release 9 of ow_simulator
## Test
CONTAINER_NAME=oceanworld

### terminal 1
`$> xhost +local:root`
<br>`$> docker run --name <CONTAINER_NAME> --rm -it --gpus all -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 raspberrysi/oceanworld:r9 bash`

Inside the container, run:
<br>`roslaunch ow europa_terminator.launch`

### terminal 2
`$> xhost +local:root`
<br>`$> docker exec -it <Container_NAME> bash`

Inside the container, run:
<br>`source devel/setup.bash`
<br>`roslaunch ow_plexil ow_exec.launch plan:=ReferenceMission1.plx`

# Play With Two Example Images
## Release 9 of ow_simulator: ROS Noetic
`docker pull oceank/oceanworld:r9`
## Release 8 of ow_simulator: ROS Melodic
`docker pull oceank/oceanworld:r8`

# Useful References
   1. [HW accelerated GUI apps on Docker](https://medium.com/@pigiuz/hw-accelerated-gui-apps-on-docker-7fd424fe813e)
   2. [nvidia-docker 1 can run OpenGL applications; nvidia-docker 2 can't](https://github.com/NVIDIA/nvidia-docker/issues/534)
   3. [Running OpenGL & CUDA Applications with nvidia-docker2 on a Remote (headless) GPU System](https://trn84.medium.com/running-opengl-cuda-applications-with-nvidia-docker2-on-a-remote-headless-gpu-system-6b19c665286d)
   4. [Supporting OpenGL/WebGL and using HW acceleration (GPU)](https://github.com/accetto/ubuntu-vnc-xfce-g3/discussions/10)
   5. [NASA OceanWORLD simulator, ow_simulator](https://github.com/nasa/ow_simulator)
   6. [TheRobotCooperative](https://github.com/TheRobotCooperative/TheRobotCooperative)
   7. [OceanWATERS](https://github.com/nasa/ow_simulator)
