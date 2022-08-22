#!/bin/env/bash

source /opt/ros/noetic/setup.bash
source /home/ais/robot_ws/install/setup.bash

rosservice call /create_rail_mission "rail_mission:
  name: 'example_mission'
  goals:
  - position: -1.02
    intensity: -1.02
    speed: -1.0"