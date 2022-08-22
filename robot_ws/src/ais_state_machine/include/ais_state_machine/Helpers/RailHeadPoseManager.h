/*
    AIS CONFIDENTIAL
    Author: Michael Chiou (m.chiou@ai-systems.ca)
*/

#ifndef ASM_RAIL_HEAD_POSE_MANAGER_H_
#define ASM_RAIL_HEAD_POSE_MANAGER_H_

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <string>

using std::string;

namespace ais_state_machine
{

class RailHeadPoseManager
{
public:
    //RailHeadPoseManager(ros::NodeHandle&);
    RailHeadPoseManager(ros::NodeHandle&, string, double);
    ~RailHeadPoseManager();
    geometry_msgs::Pose getRailHead();
    void resetData();
    bool isPoseValid();
    void setTopicToMonitor(string);
    void setTimeTolerance(double);
    double getTimeTolerance();
protected:

private:
    ros::NodeHandle nh_;
    ros::Subscriber rail_head_pose_sub_;
    ros::Time zero_time_;
    geometry_msgs::PoseStamped curr_pose_stamped_;
    geometry_msgs::Pose zero_pose_;

    string topic_name_;
    double time_tol_;

    void railheadCb(const geometry_msgs::PoseStamped::ConstPtr&);
};
}
#endif // End ASM_FAIL_HEAD_POSE_MANAGER_H_