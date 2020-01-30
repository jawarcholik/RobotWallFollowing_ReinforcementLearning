#!/usr/bin/env bash
# generated from catkin/cmake/templates/setup.bash.in

CATKIN_SHELL=bash

# source setup.sh from same directory as this file
_CATKIN_SETUP_DIR=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)
. "$_CATKIN_SETUP_DIR/setup.sh"

export GAZEBO_RESOURCE_PATH=~/Stingray-Simulation/catkin_ws/src/stingray_sim
export GAZEBO_MODEL_PATH=~/Stingray-Simulation/catkin_ws/src/stingray_sim/models
export GAZEBO_PLUGIN_PATH=~/Stingray-Simulation/catkin_ws/src/stingray_sim/plugins/build

