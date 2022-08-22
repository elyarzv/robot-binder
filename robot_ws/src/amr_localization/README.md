# README #

This repo contains localization algorithms developed at AIS

## Contents ##

### 1D Localization ###

#### Usage ####

```rosrun amr_localization amr_localization_1d_node``` 


See [example_one_d_localization.launch](https://bitbucket.org/ais_admin/amr_localization/src/master/launch/example_one_d_localization.launch) for example usage.

#### Parameters ####


| Topic (default)     | Type    | Parameter | Description | 
| --------|---------|-------|-------| 
| ```node_name/pose_1_d```  | ```std_msgs::Float32```   | ```pose_topic```  | The topic to publish the 1 dimensional pose estimate in meters| 
| ```node_name/reset```  | ```std_srvs::Trigger```   | | Use this service to reset the pose on the ```pose_1_d``` topic| 
| ```node_name/wheelodom```  | ```nav_msgs::Odometry``` or ```std_msgs::Int64MultiArray```   | ```odometry_topic```  | The source of odometry the node will subscribe to | 
|  | ```std::string("Pose")```   | ```topic_type```  | The "Pose" topic type will compute the transform from the odom frame to the robot's current pose (published on ```odometry_topic```) when the ```node_name/reset``` service is called. It will then publish the current position *in the X axis* of the robot with respect to the pose it was in when the service was called. You must publish a ```nav_msgs::Odometry``` message on ```odometry_topic``` when this type is used| 
|  | ```std::string("Twist")```   | ```topic_type```  |  The "Twist" topic type will integrate the X portion of the velocity component of the ```nav_msgs::Odometry``` message on ```odometry_topic``` at a rate of 100Hz. The twist component must be in local coordinates| 
|  | ```std::string("MultiEncoder")```   | ```topic_type```  | The "MultiEncoder" topic type will calculate the distance travelled based on encoder counts using the ```encoder_counts_per_meter``` param. You must publish a ```std_msgs::Int64MultiArray``` message on ```odometry_topic``` when this type is used. Multiple values are averaged and must use the same ```encoder_counts_per_meter```| 
|  | ```std::string("Encoder")```   | ```topic_type```  | The "Encoder" topic type will calculate the distance travelled based on encoder counts using the ```encoder_counts_per_meter``` param. You must publish a ```std_msgs::Int64``` message on ```odometry_topic``` when this type is used. | 
|  | ```float```   | ```wheel_scaling```  | A scaling factor to appy to the wheels, in case the contact wheel diameter does not match  ```nav_msgs::Odometry``` wheel size. For use with the "Pose" topic type | 
|  | ```int```   | ```encoder_counts_per_meter```  | A scaling factor to appy to the encoder for use with the "Encoder" topic type  | 
