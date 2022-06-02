#!/bin/bash

# Assume that the root of this code base locates in the 'src' directory of a catkin
# workspace and that ow_autonomy code base also locates in the 'src' directory.
#
# Run the script at the root of this code base

CURRENT_DIR=$(pwd)
export OW_PLEXIL_LIB_SOURCE_DIR=${CURRENT_DIR}/../ow_autonomy/ow_plexil/src/plans
export PLEXIL_LIB_COMPILED_PLAN_DIR=${CURRENT_DIR}/../../devel/etc/plexil
export EVALUATION_ROOT_DIR=${CURRENT_DIR}/evaluation

roslaunch rs_autonomy rs_autonomy.launch mission_spec_filename:=mission1.txt
