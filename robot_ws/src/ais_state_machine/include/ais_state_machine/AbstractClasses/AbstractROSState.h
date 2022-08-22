/*
    AIS CONFIDENTIAL
    Author: Michael Chiou (m.chiou@ai-systems.ca)
*/

#ifndef ASM_ABSTRACT_ROS_STATE_H_
#define ASM_ABSTRACT_ROS_STATE_H_

#include "ros/ros.h"

/*
    This Abstract class is meant to include ROS dependent objects and dependencies for 
    state machine states.
*/
namespace ais_state_machine 
{
class AbstractROSState {
    public:
    /* 
        Constructor that takes in memory address of global node handle
    */
        AbstractROSState(ros::NodeHandle& nh);
    /* 
        Virtual destructor
    */
        virtual ~AbstractROSState() = default;
    protected:
        ros::NodeHandle nh_;
};

}; // End namespace ais_state_machine


#endif // ASM_ABSTRACT_ROS_STATE_H_