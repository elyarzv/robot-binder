# AIS Safety

The ais_safety package contains the safety node to avoid collision

## Functionality overview

Safety node uses the isPointInPolygon function to determine if lidar data is inside the robot footprint. It uses different footprints for forward, reverse, right rotation, and left rotation. The footprints are constructed in the makeFootprints function and are formed using a vector of points. A larger footprint is used to slow down the robot, clamping the speed to the values set in `launch/safety.launch` using the following topic tools node (max vel = 0.25 max rotation = 0.1):


`  <node pkg="topic_tools" type="transform" respawn="true" name="reduced_vel_publisher"  args="/brain_cmd_vel /reduced_cmd_vel geometry_msgs/Twist 'geometry_msgs.msg.Twist(linear = geometry_msgs.msg.Vector3(x=max(-0.25, min(m.linear.x, 0.25)), y=0, z=0), angular = geometry_msgs.msg.Vector3(z=max(-0.1, min(m.angular.z, 0.1)), y=0, x=0))' --import geometry_msgs" /> `