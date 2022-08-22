#include "ais_state_machine/Helpers/RobotBumperHandler.h"

using namespace ais_state_machine;

RobotBumperHandler::RobotBumperHandler(ros::NodeHandle& nh) : nh_(nh)
{
    last_time_front_bumper_hit_ = 0;
    last_time_back_bumper_hit_ = 0;

    front_bumper_sub_ = nh.subscribe("embedded/bumper/front", 5, &RobotBumperHandler::FrontBumperCb, this);
    back_bumper_sub_ = nh.subscribe("embedded/bumper/back", 5, &RobotBumperHandler::BackBumperCb, this);

    // ToDo (Michael Chiou) Remove magic numbers
    front_bumper_time_tol_ = 0.3;
    back_bumper_time_tol_ = 0.3;
}

RobotBumperHandler::~RobotBumperHandler() 
{

}

bool RobotBumperHandler::isFrontBumperHit()
{
    double time_diff = ros::Time::now().toSec() - last_time_front_bumper_hit_; 
    if (time_diff < front_bumper_time_tol_) ROS_WARN("Front Bumper was triggered!");

    return (time_diff < front_bumper_time_tol_);
}

bool RobotBumperHandler::isBackBumperHit()
{   
    double time_diff = ros::Time::now().toSec() - last_time_back_bumper_hit_;
    if (time_diff < back_bumper_time_tol_) ROS_WARN("Back Bumper was triggered!");
    return (time_diff < back_bumper_time_tol_);
}

void RobotBumperHandler::FrontBumperCb(const std_msgs::Bool::ConstPtr& msg)
{   
    if (msg->data) last_time_front_bumper_hit_ = ros::Time::now().toSec();
}

void RobotBumperHandler::BackBumperCb(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data) last_time_back_bumper_hit_ = ros::Time::now().toSec();
}