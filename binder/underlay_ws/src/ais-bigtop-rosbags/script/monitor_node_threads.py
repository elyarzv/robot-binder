#!/usr/bin/env python

import time, threading
import datetime
import subprocess
import os
import rospy
import commands

from std_msgs.msg import Float32

class Monitor:
    def __init__(self):
        self.node_name = "pickup_test"
        self.node_pid = None
        self.get_pid(self.node_name)
        self.publisher_temp = rospy.Publisher('temp_cpu_core', Float32, queue_size=1)

        if self.node_pid == None:
            return
        if rospy.has_param('~data_directory'):
            file_name = os.path.join(rospy.get_param('~data_directory')+"/log_cpu_{}".format(datetime.datetime.now()))
        else:
            file_name = "log_cpu_{}".format(datetime.datetime.now())
            rospy.logerr("data_directory param not been set so use {} as the file to record cpu usage".format(file_name))
        self.file = open(file_name, "wa")
        self.file.write("USER       PID  SPID %CPU %MEM    VSZ   RSS TTY      STAT START   TIME COMMAND\n")

    def cpu_temp_pub(self):
        temp = commands.getoutput("sensors| grep Core\ 0| awk '{print $3}'")
        if len(temp)>6:
            temp = temp[1:-3]
            temp = float(temp)
            self.publisher_temp.publish(Float32(temp))

    def write_log_to_file(self):
        try:
            self.file.write("Time:  "+str(datetime.datetime.now())+"\n")
            self.file.write(self.get_all_threads(self.node_pid)[1]+"\n\n")
        except Exception as e:
            rospy.logerr("cannot write to file or get all threads, try to reinit and get new pid for pickup node, Error: {}".format(e))
            self.get_pid(self.node_name)


    def __del__(self):
        self.file.close()

    def get_all_threads(self, pid):
        return (commands.getstatusoutput("ps aux  -T | grep {}".format(pid)))

    def get_pid(self, node_name):
        got_pid = False
        while not got_pid and not rospy.is_shutdown():
            try:
                self.node_pid = int(commands.getstatusoutput("rosnode info /{} 2>/dev/null | grep Pid| cut -d' ' -f2".format(node_name))[1])
                got_pid = True
            except Exception as e:
                rospy.logerr (" error happend in getting pid of /{}, check if it exist, trying again: {}".format(node_name, e))
            rospy.sleep(0.1)



if __name__ == '__main__':
    rospy.init_node('monitor_pickup', anonymous=True)
    monitor = Monitor()

    rospy.loginfo("start writing to file and cpu core temp publishing")
    while not rospy.is_shutdown():
        rospy.sleep(0.4)
        monitor.write_log_to_file()
        monitor.cpu_temp_pub()
