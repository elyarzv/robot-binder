/*
    AIS CONFIDENTIAL
*/

#ifndef ASM_WHEEL_ODOM_MANAGER_H_
#define ASM_WHEEL_ODOM_MANAGER_H_

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <string>

using std::string;

namespace ais_state_machine
{

class WheelOdomManager
{
public:
    WheelOdomManager(ros::NodeHandle&, string);
    ~WheelOdomManager();
    nav_msgs::Odometry getWheelOdom();

private:
    void odomCb(const nav_msgs::OdometryConstPtr&);
    
    string topic_name_;
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    nav_msgs::Odometry curr_odom_;
};
}  // end namespace ais_state_machine

#endif  // End ASM_WHEEL_ODOM_MANAGER_H_