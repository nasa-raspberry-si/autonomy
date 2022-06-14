# MAPE-K loop based autonomy
Please refer to docments in the directory [doc](https://github.com/nasa-raspberry-si/autonomy/tree/ow9/doc) for the detailed design idea.

# Build 
This branch is based on the release 9 of OceanWATERS testbed (actually, a version of noetic-devel branch).
  - It will first need to follow the ["Get Started" instruction in nasa/ow_simulator](https://github.com/nasa/ow_simulator#getting-started) to install software prerequisites.
  - Create a catkin workspace
  `mkdirs -p ~/oceanwater_ws/src`
  - pull down the ow9 branch of the repos (ow_simulator, ow_autonomy, ow_europa and irg_ope) from nasa-raspberry-si git organization into ~/oceanwater_ws/src. After checking out the ow9 branch of nasa-raspberry-si/ow_simulator, create a soft link .rosinstall inside ~/oceanwater_ws/src using the command below and then use wstool to update code bases.
    * `ln -s ~/oceanwater_ws/src/ow_simulator/oceanwaters/workspaces/oceanwaters_ow9_rs.rosinstall  ~/oceanwater_ws/src/.rosinstall`
    * `wstool update`
  - Pull down ow9 branch of nasa-raspberry-si/autonomy into ~/oceanwater_ws/src
  - Source ROS noetic envirionment
    * `source /opt/ros/noetic/setup.bash`
  - Build using catkin
    * `catkin build`

# Run
  - Run ow_simulator in one terminal
     * cd ~/oceanwater_ws && source devel/setup.bash
     * roslaunch ow europa_terminator.launch 
  - Run ow_autonomy (actually an PLEXIL executive) in another terminal
     * cd ~/oceanwater_ws && source devel/setup.bash
     * roslaunch ow_plexil ow_exec.launch 
  - Run the Raspberry-SI autonomy in a new terminal
     * setup environment
        - cd ~/oceanwater_ws && source devel/setup.bash
     * Export three environment variables
        - `export EVALUATION\_ROOT\_DIR=< evaluation directory >`
        - `export OW\_PLEXIL\_LIB\_SOURCE\_DIR=< PLEXIL library source directory >`
        - `export PLEXIL\_LIB\_COMPILED\_PLAN\_DIR=< the direcotry of compiled PLEXIL plan in devel dir >`
     * Run with an example mission specification file, [mission1.txt](https://github.com/nasa-raspberry-si/autonomy/blob/ow9/evaluation/mission1.txt)
        - roslaunch rs\_autonomy rs\_autonomy.launch mission\_spec\_filename:=mission1.txt
     * If the evaluation directory is decided to be inside this code base, [run.bash](https://github.com/nasa-raspberry-si/autonomy/blob/ow9/run.bash) script can be used to avoid the above two steps.

# Code Organization
   - Mission Control node and MAPE-K nodes (except knowledge node)
      * autonomy/src/mape-k
   - Knowledge Node and other service nodesow8-rosnodes
      * autonomy/script
   - Python packages for support services in autonomy/src
      * acknowledgement management (for knowledge node): maintain machine learning models and task-dependent runtime information.
      * prism planning: given some runtime info, automatically generate a PRISM model for a task and use it to get policy and actions
      * plan translation: based on the actions (i.e., high-level plan), generate a PLEXIL plan using a task-related template and then compile it
      * fault management: a ROS server to inject and clear arm fault in ow_simulator
   - The organization of evaluation directory
      * An example of directory structure: autonomy/evaluation
      * For detailed description, please refer to autonomy/doc/structure_of_evaluation_directory.txt

# Current Adaptation Cases
   - List of cases
      * case 1: digging fails due to an incorrectly estimated excavation probability
      * case 2: the beliefs of supporting machine-learning models drop below predefined
      * case 3: surface vibration caused by earthquake
      * case 4: an arm fault is detected during executing a plan for excavation task
      * case 5: a manual plan sent from the Earth center
      * case 6: has a new task and need to terminate the current task
      * case 7: the current task finishes and has a new task needs
   - The categorization of above cases (excluding case 7)
   ![image](https://user-images.githubusercontent.com/5262552/169354723-1b032497-6ebd-4576-8073-7cd352b3b8dc.png)
   - Demo of adaptation cases:
      * [Adaptation case 1 and case 2](https://github.com/nasa-raspberry-si/autonomy/tree/ow9/demo#readme)

# Docker Image
  - Build oceanworld and rs_autonomy images
    * Build oceanworld image, aisys/raspberry-oceanworld:v9</br>
    `cd docker/oceanworld && ./build.sh noetic v9 aisys`
    * Build rs_autonomy image, aisys/raspberry-autonomy:v2</br>
    `cd docker/rs_autonomy && ./build.sh aisys/raspberrysi-oceanworld:v9 noetic v2 aisys ow9`
    * Two available docker images are available for play at:
       - aisys/raspberrysi-oceanworld:v9
       - aisys/raspberrysi-autonomy:v2
  - Run a docker container using the rs_autonomy image
    * Create the docker container, oceanworld, with the GPU support.</br>
    `xhost +local:root`</br>
     `docker run --name oceanworld --rm -it --gpus all -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 aisys/raspberrysi-autonomy:v2 bash`
    * Open two terminals. In each terminal, access to the docker container, oceanworld, by using the following command. One terminal is for running the PLEXIL Executive (ow_plexil package) while the other one is for running rs_autonomy.</br>
    `docker exec -it oceanworld bash`
