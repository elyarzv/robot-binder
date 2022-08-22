#!/usr/bin/env python3
from os import stat_result
import rosgraph
import os
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import String
import math
import numpy as np
from std_msgs.msg import Int32
from std_msgs.msg import UInt8
from vision_msgs.msg import Detection2DArray
import time

class RosbagRecorder(object):
    def __init__(self):
        self.robot_status_topic_name = rospy.get_param("ais_rosbags_recorder/robot_status_topic_name")
        self.current_state_topic_name = rospy.get_param("ais_rosbags_recorder/current_state_topic_name")
        self.rail_detections_topic_name = rospy.get_param("ais_rosbags_recorder/rail_detections_topic_name")
        self.recording_starter_topic_name = rospy.get_param("ais_rosbags_recorder/recording_starter_topic_name")
        self.recording_stopper_topic_name = rospy.get_param("ais_rosbags_recorder/recording_stopper_topic_name")
        self.timer_duration = rospy.get_param("ais_rosbags_recorder/timer_duration")
        self.adjusting_flag_initial_value = rospy.get_param("ais_rosbags_recorder/adjusting_flag_initial_value")
        self.rail_detection_flag_initial_value = rospy.get_param("ais_rosbags_recorder/rail_detection_flag_initial_value")
        self.recovery_mode_flag_initial_value = rospy.get_param("ais_rosbags_recorder/recovery_mode_flag_initial_value")
        self.camera_down_flag_initial_vlaue = rospy.get_param("ais_rosbags_recorder/camera_down_flag_initial_vlaue")
        self.robot_current_state = rospy.get_param("ais_rosbags_recorder/robot_current_state")
        self.last_recovery_initial_value = rospy.get_param("ais_rosbags_recorder/last_recovery_initial_value")
        self.recording_flag_initial_value = rospy.get_param("ais_rosbags_recorder/recording_flag_initial_value")
        self.camera_down_error_code = rospy.get_param("ais_rosbags_recorder/camera_down_error_code")
        self.recovery_state_number = rospy.get_param("ais_rosbags_recorder/states/recovery")
        self.adjusting_state_number = rospy.get_param("ais_rosbags_recorder/states/adjusting")
        self.min_failed_detection = rospy.get_param("ais_rosbags_recorder/min_failed_detection")

        rospy.Subscriber(self.robot_status_topic_name, String, self.robot_status_cb, queue_size=1)
        rospy.Subscriber(self.current_state_topic_name, Int32, self.current_state_cb, queue_size=1)
        rospy.Subscriber(self.rail_detections_topic_name, Detection2DArray, self.rail_head_detection_cb, queue_size=1)

        self.start_perception_recorder_pub = rospy.Publisher(self.recording_starter_topic_name, UInt8, queue_size=10)
        self.stop_perception_recorder_pub = rospy.Publisher(self.recording_stopper_topic_name, UInt8, queue_size=10)


        self.is_robot_adjusting_to_rail = self.adjusting_flag_initial_value
        self.is_rail_detected = self.rail_detection_flag_initial_value
        self.is_recovery_mode_activated = self.recovery_mode_flag_initial_value
        self.is_the_camera_down = self.camera_down_flag_initial_vlaue
        self.robot_current_state = self.robot_current_state
        self.last_recovery_value = self.last_recovery_initial_value
        self.recording_flag = False
        self.current_time = 0
        self.start_time = -999
        self.detection_counter = 0
   
    def robot_status_cb(self, msg):
        # print(msg.data)
        camera_error_code_index = msg.data.find(self.camera_down_error_code)
        if camera_error_code_index != -1:
            self.is_the_camera_down = True
        else:
            self.is_the_camera_down = False
    
    def current_state_cb(self, msg):
        self.robot_current_state = msg.data
    
    def rail_head_detection_cb(self, msg):
        self.current_time = msg.header.stamp.secs
        num_detected_rails = len(msg.detections)
        if num_detected_rails > 0 and self.robot_current_state==self.adjusting_state_number:
            self.is_rail_detected = True
            self.stop_recording()
            self.detection_counter = 0
            self.recording_flag = False

        if num_detected_rails == 0 and self.robot_current_state==self.adjusting_state_number:
            self.detection_counter += 1

        if num_detected_rails == 0 and self.robot_current_state == self.adjusting_state_number and self.detection_counter > self.min_failed_detection and self.recording_flag==False:
            self.is_rail_detected = False
            self.recording_flag = True
            self.start_recording()
            self.detection_counter = 0 
            
        if self.robot_current_state != self.adjusting_state_number:
            self.stop_recording()
            self.detection_counter = 0
            self.recording_flag = False

    def start_recording(self):
        st_string = UInt8()
        st_string.data = 1
        self.start_perception_recorder_pub.publish(st_string)
        # time.sleep(10)
    
    def stop_recording(self):
        st_string = UInt8()
        st_string.data = 1 
        self.stop_perception_recorder_pub.publish(st_string)
        time.sleep(0.5)



if __name__ == '__main__':
    
    rospy.init_node('monitor_rail_head_detection')
    r = RosbagRecorder()
    rospy.spin()
