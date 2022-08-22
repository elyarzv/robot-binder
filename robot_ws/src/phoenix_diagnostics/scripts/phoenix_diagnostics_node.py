#!/usr/bin/env python3
from numpy import uint8
import roslib
roslib.load_manifest('diagnostic_updater')
import rospy
import diagnostic_updater
from diagnostic_updater._update_functions import FrequencyStatus
import diagnostic_msgs
import std_msgs
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64
from std_msgs.msg import UInt16
from std_msgs.msg import Float32
from std_msgs.msg import UInt8
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty

class diagTestClass(object):
    def __init__(self):
        self.timer = rospy.Timer(rospy.Duration(0.5), self.RUN)
        # zed_camera_rgb_image_topic--------------------------------------
        self.zed_camera_rgb_image_updater = diagnostic_updater.Updater()
        self.zed_camera_rgb_image_updater.setHardwareID("TODO_ID" )
        self.zed_camera_rgb_image_topic_info = {'topic_name': rospy.get_param("diagnostics_topics/zed_camera_rgb_image_topic/topic_name"),
            'error_code': rospy.get_param("diagnostics_topics/zed_camera_rgb_image_topic/error_code"),
            'error_msg': rospy.get_param("diagnostics_topics/zed_camera_rgb_image_topic/error_msg"),
            'freq_bounds': rospy.get_param("diagnostics_topics/zed_camera_rgb_image_topic/freq_bounds"),
            'tolerance': rospy.get_param("diagnostics_topics/zed_camera_rgb_image_topic/tolerance"),
            'window_size': rospy.get_param("diagnostics_topics/zed_camera_rgb_image_topic/window_size")}

        self.zed_camera_rgb_image_pub_freq = diagnostic_updater.HeaderlessTopicDiagnostic(self.zed_camera_rgb_image_topic_info['topic_name'], self.zed_camera_rgb_image_updater,
            diagnostic_updater.FrequencyStatusParam(self.zed_camera_rgb_image_topic_info['freq_bounds'], self.zed_camera_rgb_image_topic_info['tolerance'], self.zed_camera_rgb_image_topic_info['window_size']), self.zed_camera_rgb_image_topic_info['error_code'])
        self.zed_camera_rgb_image_updater.force_update()
        self.zed_camera_rgb_image_pub_freq.tick()
        self.zed_camera_rgb_image_updater.update()
        rospy.Subscriber(self.zed_camera_rgb_image_topic_info['topic_name'], Image, self.zed_camera_rgb_image_topic_callback)
        rospy.loginfo("Diagnostics initialized <:/%s:>" % self.zed_camera_rgb_image_topic_info['topic_name'])


        # front_bumper_topic---------------------------------------------
        self.front_bumper_topic_updater = diagnostic_updater.Updater()
        self.front_bumper_topic_updater.setHardwareID("TODO_ID" )
        self.front_bumper_topic_info = {'topic_name': rospy.get_param("diagnostics_topics/front_bumper_topic/topic_name"),
            'error_code': rospy.get_param("diagnostics_topics/front_bumper_topic/error_code"),
            'error_msg': rospy.get_param("diagnostics_topics/front_bumper_topic/error_msg"),
            'freq_bounds': rospy.get_param("diagnostics_topics/front_bumper_topic/freq_bounds"),
            'tolerance': rospy.get_param("diagnostics_topics/front_bumper_topic/tolerance"),
            'window_size': rospy.get_param("diagnostics_topics/front_bumper_topic/window_size")}

        self.front_bumper_topic_pub_freq = diagnostic_updater.HeaderlessTopicDiagnostic(self.front_bumper_topic_info['topic_name'], self.front_bumper_topic_updater,
            diagnostic_updater.FrequencyStatusParam(self.front_bumper_topic_info['freq_bounds'], self.front_bumper_topic_info['tolerance'], self.front_bumper_topic_info['window_size']), self.front_bumper_topic_info['error_code'])
        self.front_bumper_topic_updater.force_update()
        self.front_bumper_topic_updater.get_error_code("000000")
        self.front_bumper_topic_pub_freq.tick()
        self.front_bumper_topic_updater.update()
        rospy.Subscriber(self.front_bumper_topic_info['topic_name'], Bool, self.front_bumper_topic_callback)
        rospy.loginfo("Diagnostics initialized <:/%s:>" % self.front_bumper_topic_info['topic_name'])


        # back_bumper_topic-----------------------------------------------
        self.back_bumper_topic_updater = diagnostic_updater.Updater()
        self.back_bumper_topic_updater.setHardwareID("TODO_ID" )
        self.back_bumper_topic_topic_info = {'topic_name': rospy.get_param("diagnostics_topics/back_bumper_topic/topic_name"),
            'error_code': rospy.get_param("diagnostics_topics/back_bumper_topic/error_code"),
            'error_msg': rospy.get_param("diagnostics_topics/back_bumper_topic/error_msg"),
            'freq_bounds': rospy.get_param("diagnostics_topics/back_bumper_topic/freq_bounds"),
            'tolerance': rospy.get_param("diagnostics_topics/back_bumper_topic/tolerance"),
            'window_size': rospy.get_param("diagnostics_topics/back_bumper_topic/window_size")}

        self.back_bumper_topic_pub_freq = diagnostic_updater.HeaderlessTopicDiagnostic(self.back_bumper_topic_topic_info['topic_name'], self.back_bumper_topic_updater,
            diagnostic_updater.FrequencyStatusParam(self.back_bumper_topic_topic_info['freq_bounds'], self.back_bumper_topic_topic_info['tolerance'], self.back_bumper_topic_topic_info['window_size']), self.back_bumper_topic_topic_info['error_code'])
        self.back_bumper_topic_updater.force_update()
        self.back_bumper_topic_pub_freq.tick()
        self.back_bumper_topic_updater.update()
        rospy.Subscriber(self.back_bumper_topic_topic_info['topic_name'], Bool, self.back_bumper_topic_callback)
        rospy.loginfo("Diagnostics initialized <:/%s:>" % self.back_bumper_topic_topic_info['topic_name'])


        # battery_level_topic-----------------------------------------------
        self.battery_level_updater = diagnostic_updater.Updater()
        self.battery_level_updater.setHardwareID("TODO_ID" )
        self.battery_level_topic_info = {'topic_name': rospy.get_param("diagnostics_topics/battery_level_topic/topic_name"),
            'error_code': rospy.get_param("diagnostics_topics/battery_level_topic/error_code"),
            'error_msg': rospy.get_param("diagnostics_topics/battery_level_topic/error_msg"),
            'freq_bounds': rospy.get_param("diagnostics_topics/battery_level_topic/freq_bounds"),
            'tolerance': rospy.get_param("diagnostics_topics/battery_level_topic/tolerance"),
            'window_size': rospy.get_param("diagnostics_topics/battery_level_topic/window_size")}

        self.battery_level_topic_pub_freq = diagnostic_updater.HeaderlessTopicDiagnostic(self.battery_level_topic_info['topic_name'], self.battery_level_updater,
            diagnostic_updater.FrequencyStatusParam(self.battery_level_topic_info['freq_bounds'], self.battery_level_topic_info['tolerance'], self.battery_level_topic_info['window_size']), self.battery_level_topic_info['error_code'])
        self.battery_level_updater.force_update()
        self.battery_level_topic_pub_freq.tick()
        self.battery_level_updater.update()
        rospy.Subscriber(self.battery_level_topic_info['topic_name'], UInt8, self.battery_level_topic_callback)
        rospy.loginfo("Diagnostics initialized <:/%s:>" % self.battery_level_topic_info['topic_name'])


        # battery_current_draw_topic-----------------------------------------------
        self.battery_current_draw_updater = diagnostic_updater.Updater()
        self.battery_current_draw_updater.setHardwareID("TODO_ID" )
        self.battery_current_draw_topic_info = {'topic_name': rospy.get_param("diagnostics_topics/battery_current_draw_topic/topic_name"),
            'error_code': rospy.get_param("diagnostics_topics/battery_current_draw_topic/error_code"),
            'error_msg': rospy.get_param("diagnostics_topics/battery_current_draw_topic/error_msg"),
            'freq_bounds': rospy.get_param("diagnostics_topics/battery_current_draw_topic/freq_bounds"),
            'tolerance': rospy.get_param("diagnostics_topics/battery_current_draw_topic/tolerance"),
            'window_size': rospy.get_param("diagnostics_topics/battery_current_draw_topic/window_size")}

        self.battery_current_draw_topic_pub_freq = diagnostic_updater.HeaderlessTopicDiagnostic(self.battery_current_draw_topic_info['topic_name'], self.battery_current_draw_updater,
            diagnostic_updater.FrequencyStatusParam(self.battery_current_draw_topic_info['freq_bounds'], self.battery_current_draw_topic_info['tolerance'], self.battery_current_draw_topic_info['window_size']), self.battery_current_draw_topic_info['error_code'])
        self.battery_current_draw_updater.force_update()
        self.battery_current_draw_topic_pub_freq.tick()
        self.battery_current_draw_updater.update()
        rospy.Subscriber(self.battery_current_draw_topic_info['topic_name'], Float32, self.battery_current_draw_topic_callback)
        rospy.loginfo("Diagnostics initialized <:/%s:>" % self.battery_current_draw_topic_info['topic_name'])


        # external_encoder_topic-----------------------------------------------
        self.external_encoder_updater = diagnostic_updater.Updater()
        self.external_encoder_updater.setHardwareID("TODO_ID" )
        self.external_encoder_topic_info = {'topic_name': rospy.get_param("diagnostics_topics/external_encoder_topic/topic_name"),
            'error_code': rospy.get_param("diagnostics_topics/external_encoder_topic/error_code"),
            'error_msg': rospy.get_param("diagnostics_topics/external_encoder_topic/error_msg"),
            'freq_bounds': rospy.get_param("diagnostics_topics/external_encoder_topic/freq_bounds"),
            'tolerance': rospy.get_param("diagnostics_topics/external_encoder_topic/tolerance"),
            'window_size': rospy.get_param("diagnostics_topics/external_encoder_topic/window_size")}

        self.external_encoder_topic_pub_freq = diagnostic_updater.HeaderlessTopicDiagnostic(self.external_encoder_topic_info['topic_name'], self.external_encoder_updater,
            diagnostic_updater.FrequencyStatusParam(self.external_encoder_topic_info['freq_bounds'], self.external_encoder_topic_info['tolerance'], self.external_encoder_topic_info['window_size']), self.external_encoder_topic_info['error_code'])
        self.external_encoder_updater.force_update()
        self.external_encoder_topic_pub_freq.tick()
        self.external_encoder_updater.update()
        rospy.Subscriber(self.external_encoder_topic_info['topic_name'], Int64, self.external_encoder_topic_callback)
        rospy.loginfo("Diagnostics initialized <:/%s:>" % self.external_encoder_topic_info['topic_name'])


        # uv_lamp_status_topic-----------------------------------------------
        self.uv_lamp_status_updater = diagnostic_updater.Updater()
        self.uv_lamp_status_updater.setHardwareID("TODO_ID" )
        self.uv_lamp_status_topic_info = {'topic_name': rospy.get_param("diagnostics_topics/uv_lamp_status_topic/topic_name"),
            'error_code': rospy.get_param("diagnostics_topics/uv_lamp_status_topic/error_code"),
            'error_msg': rospy.get_param("diagnostics_topics/uv_lamp_status_topic/error_msg"),
            'freq_bounds': rospy.get_param("diagnostics_topics/uv_lamp_status_topic/freq_bounds"),
            'tolerance': rospy.get_param("diagnostics_topics/uv_lamp_status_topic/tolerance"),
            'window_size': rospy.get_param("diagnostics_topics/uv_lamp_status_topic/window_size")}

        self.uv_lamp_status_topic_pub_freq = diagnostic_updater.HeaderlessTopicDiagnostic(self.uv_lamp_status_topic_info['topic_name'], self.uv_lamp_status_updater,
            diagnostic_updater.FrequencyStatusParam(self.uv_lamp_status_topic_info['freq_bounds'], self.uv_lamp_status_topic_info['tolerance'], self.uv_lamp_status_topic_info['window_size']), self.uv_lamp_status_topic_info['error_code'])
        self.uv_lamp_status_updater.force_update()
        self.uv_lamp_status_topic_pub_freq.tick()
        self.uv_lamp_status_updater.update()
        rospy.Subscriber(self.uv_lamp_status_topic_info['topic_name'], UInt16, self.uv_lamp_status_topic_callback)
        rospy.loginfo("Diagnostics initialized <:/%s:>" % self.uv_lamp_status_topic_info['topic_name'])


        # embedded_power_inverter_status-----------------------------------------------
        self.embedded_power_inverter_updater = diagnostic_updater.Updater()
        self.embedded_power_inverter_updater.setHardwareID("TODO_ID" )
        self.embedded_power_inverter_status_info = {'topic_name': rospy.get_param("diagnostics_topics/embedded_power_inverter_status/topic_name"),
            'error_code': rospy.get_param("diagnostics_topics/embedded_power_inverter_status/error_code"),
            'error_msg': rospy.get_param("diagnostics_topics/embedded_power_inverter_status/error_msg"),
            'freq_bounds': rospy.get_param("diagnostics_topics/embedded_power_inverter_status/freq_bounds"),
            'tolerance': rospy.get_param("diagnostics_topics/embedded_power_inverter_status/tolerance"),
            'window_size': rospy.get_param("diagnostics_topics/embedded_power_inverter_status/window_size")}

        self.embedded_power_inverter_status_pub_freq = diagnostic_updater.HeaderlessTopicDiagnostic(self.embedded_power_inverter_status_info['topic_name'], self.embedded_power_inverter_updater,
            diagnostic_updater.FrequencyStatusParam(self.embedded_power_inverter_status_info['freq_bounds'], self.embedded_power_inverter_status_info['tolerance'], self.embedded_power_inverter_status_info['window_size']), self.embedded_power_inverter_status_info['error_code'])
        self.embedded_power_inverter_updater.force_update()
        self.embedded_power_inverter_status_pub_freq.tick()
        self.embedded_power_inverter_updater.update()
        rospy.Subscriber(self.embedded_power_inverter_status_info['topic_name'], Bool, self.embedded_power_inverter_status_topic_callback)
        rospy.loginfo("Diagnostics initialized <:/%s:>" % self.embedded_power_inverter_status_info['topic_name'])


        # embedded_right_motor_driver_read_ff-----------------------------------------------
        self.embedded_right_motor_driver_read_ff_updater = diagnostic_updater.Updater()
        self.embedded_right_motor_driver_read_ff_updater.setHardwareID("TODO_ID" )
        self.embedded_right_motor_driver_read_ff_info = {'topic_name': rospy.get_param("diagnostics_topics/embedded_right_motor_driver_read_ff/topic_name"),
            'error_code': rospy.get_param("diagnostics_topics/embedded_right_motor_driver_read_ff/error_code"),
            'error_msg': rospy.get_param("diagnostics_topics/embedded_right_motor_driver_read_ff/error_msg"),
            'freq_bounds': rospy.get_param("diagnostics_topics/embedded_right_motor_driver_read_ff/freq_bounds"),
            'tolerance': rospy.get_param("diagnostics_topics/embedded_right_motor_driver_read_ff/tolerance"),
            'window_size': rospy.get_param("diagnostics_topics/embedded_right_motor_driver_read_ff/window_size")}

        self.embedded_right_motor_driver_read_ff_pub_freq = diagnostic_updater.HeaderlessTopicDiagnostic(self.embedded_right_motor_driver_read_ff_info['topic_name'], self.embedded_right_motor_driver_read_ff_updater,
            diagnostic_updater.FrequencyStatusParam(self.embedded_right_motor_driver_read_ff_info['freq_bounds'], self.embedded_right_motor_driver_read_ff_info['tolerance'], self.embedded_right_motor_driver_read_ff_info['window_size']), self.embedded_right_motor_driver_read_ff_info['error_code'])
        self.embedded_right_motor_driver_read_ff_updater.force_update()
        self.embedded_right_motor_driver_read_ff_pub_freq.tick()
        self.embedded_right_motor_driver_read_ff_updater.update()
        rospy.Subscriber(self.embedded_right_motor_driver_read_ff_info['topic_name'], Int64, self.embedded_right_motor_driver_read_ff_topic_callback)
        rospy.loginfo("Diagnostics initialized <:/%s:>" % self.embedded_right_motor_driver_read_ff_info['topic_name'])


        # embedded_left_motor_driver_read_ff-----------------------------------------------
        self.embedded_left_motor_driver_read_ff_updater = diagnostic_updater.Updater()
        self.embedded_left_motor_driver_read_ff_updater.setHardwareID("TODO_ID" )
        self.embedded_left_motor_driver_read_ff_info = {'topic_name': rospy.get_param("diagnostics_topics/embedded_left_motor_driver_read_ff/topic_name"),
            'error_code': rospy.get_param("diagnostics_topics/embedded_left_motor_driver_read_ff/error_code"),
            'error_msg': rospy.get_param("diagnostics_topics/embedded_left_motor_driver_read_ff/error_msg"),
            'freq_bounds': rospy.get_param("diagnostics_topics/embedded_left_motor_driver_read_ff/freq_bounds"),
            'tolerance': rospy.get_param("diagnostics_topics/embedded_left_motor_driver_read_ff/tolerance"),
            'window_size': rospy.get_param("diagnostics_topics/embedded_left_motor_driver_read_ff/window_size")}

        self.embedded_left_motor_driver_read_ff_pub_freq = diagnostic_updater.HeaderlessTopicDiagnostic(self.embedded_left_motor_driver_read_ff_info['topic_name'], self.embedded_left_motor_driver_read_ff_updater,
            diagnostic_updater.FrequencyStatusParam(self.embedded_left_motor_driver_read_ff_info['freq_bounds'], self.embedded_left_motor_driver_read_ff_info['tolerance'], self.embedded_left_motor_driver_read_ff_info['window_size']), self.embedded_left_motor_driver_read_ff_info['error_code'])
        self.embedded_left_motor_driver_read_ff_updater.force_update()
        self.embedded_left_motor_driver_read_ff_pub_freq.tick()
        self.embedded_left_motor_driver_read_ff_updater.update()
        rospy.Subscriber(self.embedded_left_motor_driver_read_ff_info['topic_name'], Int64, self.embedded_left_motor_driver_read_ff_topic_callback)
        rospy.loginfo("Diagnostics initialized <:/%s:>" % self.embedded_left_motor_driver_read_ff_info['topic_name'])


        # scan_multi_filtered-----------------------------------------------
        self.scan_multi_filtered_updater = diagnostic_updater.Updater()
        self.scan_multi_filtered_updater.setHardwareID("TODO_ID" )
        self.scan_multi_filtered_info = {'topic_name': rospy.get_param("diagnostics_topics/scan_multi_filtered/topic_name"),
            'error_code': rospy.get_param("diagnostics_topics/scan_multi_filtered/error_code"),
            'error_msg': rospy.get_param("diagnostics_topics/scan_multi_filtered/error_msg"),
            'freq_bounds': rospy.get_param("diagnostics_topics/scan_multi_filtered/freq_bounds"),
            'tolerance': rospy.get_param("diagnostics_topics/scan_multi_filtered/tolerance"),
            'window_size': rospy.get_param("diagnostics_topics/scan_multi_filtered/window_size")}

        self.scan_multi_filtered_pub_freq = diagnostic_updater.HeaderlessTopicDiagnostic(self.scan_multi_filtered_info['topic_name'], self.scan_multi_filtered_updater,
            diagnostic_updater.FrequencyStatusParam(self.scan_multi_filtered_info['freq_bounds'], self.scan_multi_filtered_info['tolerance'], self.scan_multi_filtered_info['window_size']), self.scan_multi_filtered_info['error_code'])
        self.scan_multi_filtered_updater.force_update()
        self.scan_multi_filtered_pub_freq.tick()
        self.scan_multi_filtered_updater.update()
        rospy.Subscriber(self.scan_multi_filtered_info['topic_name'], LaserScan, self.scan_multi_filtered_topic_callback)
        rospy.loginfo("Diagnostics initialized <:/%s:>" % self.scan_multi_filtered_info['topic_name'])


        # embedded_proximiterstatus-----------------------------------------------
        self.embedded_proximiterstatus_updater = diagnostic_updater.Updater()
        self.embedded_proximiterstatus_updater.setHardwareID("TODO_ID" )
        self.embedded_proximiterstatus_info = {'topic_name': rospy.get_param("diagnostics_topics/embedded_proximiterstatus/topic_name"),
            'error_code': rospy.get_param("diagnostics_topics/embedded_proximiterstatus/error_code"),
            'error_msg': rospy.get_param("diagnostics_topics/embedded_proximiterstatus/error_msg"),
            'freq_bounds': rospy.get_param("diagnostics_topics/embedded_proximiterstatus/freq_bounds"),
            'tolerance': rospy.get_param("diagnostics_topics/embedded_proximiterstatus/tolerance"),
            'window_size': rospy.get_param("diagnostics_topics/embedded_proximiterstatus/window_size")}

        self.embedded_proximiterstatus_pub_freq = diagnostic_updater.HeaderlessTopicDiagnostic(self.embedded_proximiterstatus_info['topic_name'], self.embedded_proximiterstatus_updater,
            diagnostic_updater.FrequencyStatusParam(self.embedded_proximiterstatus_info['freq_bounds'], self.embedded_proximiterstatus_info['tolerance'], self.embedded_proximiterstatus_info['window_size']), self.embedded_proximiterstatus_info['error_code'])
        self.embedded_proximiterstatus_updater.force_update()
        self.embedded_proximiterstatus_pub_freq.tick()
        self.embedded_proximiterstatus_updater.update()
        rospy.Subscriber(self.embedded_proximiterstatus_info['topic_name'], UInt8, self.embedded_proximiterstatus_topic_callback)  
        rospy.loginfo("Diagnostics initialized <:/%s:>" % self.embedded_proximiterstatus_info['topic_name'])

        # is_online Test Topic (TO-be-deletted)
        self.is_online_updater = diagnostic_updater.Updater()
        self.is_online_updater.setHardwareID("TODO_ID" )
        self.is_online_info = {'topic_name': rospy.get_param("diagnostics_topics/is_online/topic_name"),
            'error_code': rospy.get_param("diagnostics_topics/is_online/error_code"),
            'error_msg': rospy.get_param("diagnostics_topics/is_online/error_msg"),
            'freq_bounds': rospy.get_param("diagnostics_topics/is_online/freq_bounds"),
            'tolerance': rospy.get_param("diagnostics_topics/is_online/tolerance"),
            'window_size': rospy.get_param("diagnostics_topics/is_online/window_size")}

        self.is_online_pub_freq = diagnostic_updater.HeaderlessTopicDiagnostic(self.is_online_info['topic_name'], self.is_online_updater,
            diagnostic_updater.FrequencyStatusParam(self.is_online_info['freq_bounds'], self.is_online_info['tolerance'], self.is_online_info['window_size']), self.is_online_info['error_code'])
        self.is_online_updater.force_update()
        self.is_online_pub_freq.tick()
        self.is_online_updater.update()
        rospy.Subscriber(self.is_online_info['topic_name'], Empty, self.is_online_topic_callback)  
        rospy.loginfo("Diagnostics initialized <:/%s:>" % self.is_online_info['topic_name'])

    # Timer Callback
    def RUN(self, timer):
        self.zed_camera_rgb_image_updater.update()
        self.front_bumper_topic_updater.update()
        self.back_bumper_topic_updater.update()
        self.battery_level_updater.update()
        self.battery_current_draw_updater.update()
        self.external_encoder_updater.update()
        self.uv_lamp_status_updater.update()
        self.embedded_power_inverter_updater.update()
        self.embedded_right_motor_driver_read_ff_updater.update()
        self.embedded_left_motor_driver_read_ff_updater.update()
        self.scan_multi_filtered_updater.update()
        self.embedded_proximiterstatus_updater.update()
        self.is_online_updater.update()

    # ZED2 Camera Callback Fuction
    def zed_camera_rgb_image_topic_callback(self, data):
        self.zed_camera_rgb_image_pub_freq.tick()
        self.zed_camera_rgb_image_updater.update()
    
    # Front bumper Topic Callback
    def front_bumper_topic_callback(self, data):
        self.front_bumper_topic_pub_freq.tick()
        self.front_bumper_topic_updater.update()
    
    # Back bumper Topic Callback
    def back_bumper_topic_callback(self, data):
        self.back_bumper_topic_pub_freq.tick()
        self.back_bumper_topic_updater.update()
    
    # Battery Level Topic Callback
    def battery_level_topic_callback(self, data):
        self.battery_level_topic_pub_freq.tick()
        self.battery_level_updater.update()
    
    # battery_current_draw_topic_callback
    def battery_current_draw_topic_callback(self, data):
        self.battery_current_draw_topic_pub_freq.tick()
        self.battery_current_draw_updater.update()
    
    # External Encoder Topic Callback
    def external_encoder_topic_callback(self, data):
        self.external_encoder_topic_pub_freq.tick()
        self.external_encoder_updater.update()
    
    # UV Lamp Status Topic Callback
    def uv_lamp_status_topic_callback(self, data):
        self.uv_lamp_status_topic_pub_freq.tick()
        self.uv_lamp_status_updater.update()

    def embedded_power_inverter_status_topic_callback(self, data):
        self.embedded_power_inverter_status_pub_freq.tick()
        self.embedded_power_inverter_updater.update()

    def embedded_right_motor_driver_read_ff_topic_callback(self, data):
        self.embedded_right_motor_driver_read_ff_pub_freq.tick()
        self.embedded_right_motor_driver_read_ff_updater.update()
    
    def embedded_left_motor_driver_read_ff_topic_callback(self, data):
        self.embedded_left_motor_driver_read_ff_pub_freq.tick()
        self.embedded_left_motor_driver_read_ff_updater.update()

    def scan_multi_filtered_topic_callback(self, data):
        self.scan_multi_filtered_pub_freq.tick()
        self.scan_multi_filtered_updater.update()
    
    def embedded_proximiterstatus_topic_callback(self, data):
        self.embedded_proximiterstatus_pub_freq.tick()
        self.embedded_proximiterstatus_updater.update()
    
    def is_online_topic_callback(self, data):
        self.is_online_pub_freq.tick()
        self.is_online_updater.update()

if __name__ == '__main__': 
    rospy.init_node('diagnostic_updater_example')
    p = diagTestClass()
    rospy.spin()
