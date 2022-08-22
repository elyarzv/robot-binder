/*
    AIS CONFIDENTIAL
    Author: Michael Chiou (m.chiou@ai-systems.ca)
*/

#ifndef ASM_STATE_MOUNT_ROBOT_TO_RAIL_H_
#define ASM_STATE_MOUNT_ROBOT_TO_RAIL_H_

#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "ais_state_machine/AbstractClasses/AbstractState.h"
#include "ais_state_machine/AbstractClasses/AbstractROSState.h"
#include <string>
#include "nav_msgs/Odometry.h"
#include "ais_state_machine/Helpers/PoseUtilities.h"

namespace ais_state_machine 
{

class StateMountRobotToRail final: public AbstractState, public AbstractROSState
{
public: 
    StateMountRobotToRail(shared_ptr<AbstractStateMachineManager> state_manager, ros::NodeHandle& nh);
    ~StateMountRobotToRail() override;
    void runState() override;
    bool isStateFinished() override;
    StateTypes getNextStateName() override;
    void reset() override;
    void finish() override;
    void init() override;
    void interruptState() override;
private: 
    void evaluateStateStatus() override;
private:
    ros::NodeHandle nh_;
    nav_msgs::Odometry curr_pose_;
    nav_msgs::Odometry initial_pose_;
    PoseUtilities p_util_;
    tf2::Transform tf_init2orig_;
    ros::Time start_time;
    float mount_distance_;
    
};

} // end of namespace ais_state_machine
#endif // END ASM_STATE_MOUNT_ROBOT_TO_RAIL_H_