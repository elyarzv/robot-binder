#!/bin/bash

source /opt/phoenix1/setup.bash
rosparam set /MISSION_DIR /opt/phoenix1/share/ais_state_machine/test/
rosservice call /start_named_mission "mission: lab_mission"
. /tmp/wait_for_complete.bash