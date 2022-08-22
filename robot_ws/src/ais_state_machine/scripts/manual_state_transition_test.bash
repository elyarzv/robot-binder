#!/bin/bash

source /opt/ros/noetic/setup.bash
source /home/ais/robot_ws/setup.bash

rosservice call /start_mission "{}"
rostopic pub -1 /encoder_localization_node/pose_1_d std_msgs/Float32 "data: 0.0"
rostopic pub -1 /encoder_localization_node/pose_1_d std_msgs/Float32 "data: 5.0"
rostopic pub -1 /encoder_localization_node/pose_1_d std_msgs/Float32 "data: 21.0"
rostopic pub -1 /encoder_localization_node/pose_1_d std_msgs/Float32 "data: 25.0"
rostopic pub -1 /encoder_localization_node/pose_1_d std_msgs/Float32 "data: 41.0"
rostopic pub -1 /encoder_localization_node/pose_1_d std_msgs/Float32 "data: 50.0"
rostopic pub -1 /encoder_localization_node/pose_1_d std_msgs/Float32 "data: 100.0"
rostopic pub -1 /encoder_localization_node/pose_1_d std_msgs/Float32 "data: -1.0"