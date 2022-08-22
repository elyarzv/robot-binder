#include "ais_state_machine/Helpers/RailHeadPoseManager.h"

using namespace ais_state_machine;

RailHeadPoseManager::RailHeadPoseManager(ros::NodeHandle& nh, string topic_name, double tolerance) 
    : nh_(nh), topic_name_(topic_name), time_tol_(tolerance)
{
    rail_head_pose_sub_ = nh.subscribe(topic_name_.c_str(), 1, &RailHeadPoseManager::railheadCb, this);
    zero_time_ = ros::Time(0.0);
    zero_pose_.position.x = 0.0;
    zero_pose_.position.y = 0.0;
    zero_pose_.position.z = 0.0;
    zero_pose_.orientation.x = 0.0;
    zero_pose_.orientation.y = 0.0;
    zero_pose_.orientation.z = 0.0;
    zero_pose_.orientation.w = 1.0;
}

RailHeadPoseManager::~RailHeadPoseManager()
{
    ROS_DEBUG("RailHeadPoseManager with %s Destructed!", topic_name_.c_str());
}

geometry_msgs::Pose RailHeadPoseManager::getRailHead()
{
    if (isPoseValid()) {
        return curr_pose_stamped_.pose;
    } else 
    {
        ROS_WARN("return zero pose");
        return zero_pose_;
    }

}

void RailHeadPoseManager::resetData()
{
    curr_pose_stamped_.header.stamp = zero_time_;
    curr_pose_stamped_.pose = zero_pose_;
}

void RailHeadPoseManager::setTopicToMonitor(string topic_name)
{
    topic_name_ = topic_name;
    // TODO: Check if topic exists first
    rail_head_pose_sub_ = nh_.subscribe(topic_name_.c_str(), 1, &RailHeadPoseManager::railheadCb, this);
    ROS_INFO("RailHeadPoseManager is tracking topic: %s", topic_name.c_str());
}

bool RailHeadPoseManager::isPoseValid()
{
    if (ros::Time::now().toSec() - curr_pose_stamped_.header.stamp.toSec() > time_tol_ 
        || curr_pose_stamped_.header.stamp.toSec() <= 0.0) 
    {
        return false;
    }
    return true;
}

void RailHeadPoseManager::railheadCb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    curr_pose_stamped_ = *msg;
}

void RailHeadPoseManager::setTimeTolerance(double tol)
{
    time_tol_ = tol;
}

double RailHeadPoseManager::getTimeTolerance()
{
    return time_tol_;
}