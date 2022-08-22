#include "ais_state_machine/Helpers/WheelOdomManager.h"

using namespace ais_state_machine;

WheelOdomManager::WheelOdomManager(ros::NodeHandle& nh, string topic_name) :
    nh_(nh), topic_name_(topic_name)
{
    odom_sub_ = nh.subscribe(topic_name.c_str(), 1, &WheelOdomManager::odomCb, this);

    curr_odom_.header.stamp = ros::Time::now();
    curr_odom_.pose.pose.position.x = 0;
    curr_odom_.pose.pose.position.y = 0;
    curr_odom_.pose.pose.position.z = 0;
    curr_odom_.pose.pose.orientation.w = 1;

    curr_odom_.twist.twist.linear.x = 0;
    curr_odom_.twist.twist.linear.y = 0;
    curr_odom_.twist.twist.linear.z = 0;
    curr_odom_.twist.twist.angular.z = 0;

}

WheelOdomManager::~WheelOdomManager()
{

}

nav_msgs::Odometry WheelOdomManager::getWheelOdom()
{
    return curr_odom_;
}

void WheelOdomManager::odomCb(const nav_msgs::OdometryConstPtr& msg)
{
    curr_odom_ = *msg;
}