#!/usr/bin/env python3
from os import stat_result
import rosgraph
import os
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Bool
import math
import numpy as np

class DiagnosticsNode(object):
    def __init__(self):
        self.roscore_down_diag_code = "00000" #rospy.get_param('diagnostics/roscore=1)
        camera_topic_name = rospy.get_param('camera_topic_name')
        diagnostics_topic_name = rospy.get_param('diagnostics_topic_name')
        self.diagnostics_frequency_rate = rospy.get_param('diagnostics_frequency_rate')
        self.diagnostics_frequency_time_window = rospy.get_param('diagnostics_frequency_time_window')
        self.system_running_diag_code = rospy.get_param('system_running_diag_code')
        self.roscore_down_diag_code = rospy.get_param('roscore_down_diag_code')
        self.camera_down_diag_code = rospy.get_param('camera_down_diag_code')
        self.rail_detection_down_diag_code = rospy.get_param('rail_detection_down_diag_code')
        self.camera_acceptable_frequency = rospy.get_param('camera_acceptable_frequency')
        rospy.Subscriber(camera_topic_name, Image, self.camera_cb)
        self.diagnostics_publisher = rospy.Publisher(diagnostics_topic_name, String, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(self.diagnostics_frequency_rate), self.RUN)
        self.camera_counter = 0.0
        self.camera_freq_window = np.zeros((1, self.diagnostics_frequency_time_window))
        self.roscore_up = True
        self.time_window_counter = 0

    def camera_cb(self, data):
        try:
            self.camera_counter += 1
        except:
            rospy.loginfo("CALLBACK ERROR: Could not update camera frequency!")

    def RUN(self, timer):
        try:
            if self.time_window_counter > self.diagnostics_frequency_time_window - 1:
                self.time_window_counter = 0
            self.camera_freq_window[0, self.time_window_counter] = self.camera_counter
            self.time_window_counter += 1

            self.camera_avg_freq = sum(list(self.camera_freq_window[0]))/self.diagnostics_frequency_time_window
            diag_msg = String()
            if rosgraph.is_master_online():
                rospy.loginfo("INFO: ROSCORE is up and running!")

                if self.camera_avg_freq >= self.camera_acceptable_frequency:
                    diag_msg = self.system_running_diag_code
                    rospy.loginfo("INFO: The camera frequency is acceptable! Current Value = %s" % self.camera_avg_freq)
                else:
                    diag_msg.data = self.camera_down_diag_code
                    self.diagnostics_publisher.publish(diag_msg)
                    rospy.loginfo("INFO: The vision sensor is down or its frequency is LOWER than the acceptable criteria! Current Value = : %s" % self.camera_counter)
                self.camera_counter = 0.0
                self.camera_avg_freq = 0.0
            else:
                diag_msg.data = self.roscore_down_diag_code
                self.diagnostics_publisher.publish(diag_msg)
                rospy.loginfo("INFO: ROSCORE is down!")
                self.camera_counter = 0.0
                self.camera_avg_freq = 0.0
        except (RuntimeError, TypeError, NameError):
            pass
            # rospy.loginfo("ERROR: Could not communicate with ROSCORE!")
        
if __name__ == '__main__':
    
    rospy.init_node('diagnostics')
    diag = DiagnosticsNode()
    rospy.spin()
