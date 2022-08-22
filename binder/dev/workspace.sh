#!/bin/bash

export PROJECT=phoenix1
export WORKSPACE=${HOME}
export ROBOT_WS=${WORKSPACE}/robot_ws
export SIMULATION_WS=${WORKSPACE}/simulation_ws
export LOG_DIR=${HOME}'/.ais/logs/'
export MAP_DIR=${HOME}'/.ais/maps/'
export MISSION_DIR=${HOME}'/.ais/missions/'
export GAZEBO_DIR=${HOME}'/.gazebo'
export GAZEBO_MODEL_PATH=${GAZEBO_DIR}'/models'
export GAZEBO_MODEL_PATH=~/simulation_ws/src/phoenix1_simulation/models:$GAZEBO_MODEL_PATH
export GAZEBO_MODEL_PATH=/opt/simulation_ws/share/phoenix1_simulation/models:$GAZEBO_MODEL_PATH
export GAZEBO_RESOURCE_PATH=${GAZEBO_DIR}'/worlds'
export ROS_DISTRO=noetic
export TERM=xterm-256color
export PATH="/root/.local/bin:$PATH"
export PATH="${WORKSPACE}/field-scripts:$PATH"

cp /opt/phoenix1/share/ais_state_machine/test/*.csv $MISSION_DIR
