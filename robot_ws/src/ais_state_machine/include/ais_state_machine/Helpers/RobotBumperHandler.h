/* 
    AIS CONFIDENTIAL
    Author: Michael Chiou (m.chiou@ai-systems.ca)
*/

#ifndef ASM_ROBOT_BUMPER_HANDLER_H_
#define ASM_ROBOT_BUMPER_HANDLER_H_

#include <algorithm>

#include "ros/ros.h"
#include "std_msgs/Bool.h"

namespace ais_state_machine
{

class RobotBumperHandler
{
    public:
        RobotBumperHandler(ros::NodeHandle&);
        ~RobotBumperHandler();
        bool isFrontBumperHit();
        bool isBackBumperHit();

    private:
        void FrontBumperCb(const std_msgs::Bool::ConstPtr&);
        void BackBumperCb(const std_msgs::Bool::ConstPtr&);

        ros::NodeHandle nh_;
        ros::Subscriber front_bumper_sub_;
        ros::Subscriber back_bumper_sub_;

        double last_time_front_bumper_hit_;
        double last_time_back_bumper_hit_;

        double front_bumper_time_tol_;
        double back_bumper_time_tol_;

};
} // end of namespace ais_state_machine

#endif // END ASM_ROBOT_BUMPER_HANDLER_H_