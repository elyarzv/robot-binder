#!/usr/bin/env python

import rosnode
import rospy


class shutdown_class:

  def __init__(self):
    self.death_publisher = rospy.Publisher('/record/stop', UInt8, queue_size=10)

  def hook(self):
    pass
      #self.death_publisher.publish(UInt8())

def shutdown_hook():
  shutdown_obj.hook()

if __name__ == "__main__":
  rospy.init_node("rosbag_stopper")
  shutdown_obj = shutdown_class()
  rospy.on_shutdown(shutdown_hook)
  while not rospy.is_shutdown():
    rospy.sleep(100)
