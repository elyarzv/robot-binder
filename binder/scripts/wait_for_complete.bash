#!/usr/bin/env bash

source /opt/phoenix1/setup.bash
export status=$(/bin/bash -c ". /opt/ros/${ROS_DISTRO}/setup.bash && rostopic echo -n 1 /autonomy/robot_status" | grep "AUTO" | wc -l)
echo $status
echo "Waiting for AUTO"
while [[ $status -eq 0 ]]
do
  export status=$(/bin/bash -c ". /opt/ros/${ROS_DISTRO}/setup.bash && rostopic echo -n 1 /autonomy/robot_status" | grep "AUTO" | wc -l)
  rostopic echo -n 1 /autonomy/robot_status
done
export status=$(/bin/bash -c ". /opt/ros/${ROS_DISTRO}/setup.bash && rostopic echo -n 1 /autonomy/robot_status" | grep "IDLE" | wc -l)
echo $status
echo "Waiting for IDLE"
while [[ $status -eq 0 ]]
do
  export status=$(/bin/bash -c ". /opt/ros/${ROS_DISTRO}/setup.bash && rostopic echo -n 1 /autonomy/robot_status" | grep "IDLE" | wc -l)
  rostopic echo -n 1 /autonomy/robot_status
  rostopic echo -n 1 /encoder_localization_node/pose_1_d
done
exit 0