/*
    AIS CONFIDENTIAL
    Author: Michael Chiou (m.chiou@ai-systems.ca)
*/

#ifndef ASM_STATE_MOVE_ROBOT_OFF_H_
#define ASM_STATE_MOVE_ROBOT_OFF_H_

#define safety_timeout 1

#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "ais_state_machine/AbstractClasses/AbstractState.h"
#include "ais_state_machine/AbstractClasses/AbstractROSState.h"
#include "ais_state_machine/AutonomyStates/StateFixOrientation.h"
#include <string>
#include "nav_msgs/Odometry.h"
#include "ais_state_machine/Helpers/PoseUtilities.h"

namespace ais_state_machine
{

class StateMoveRobotOffRail final: public AbstractState, public AbstractROSState
{
public:
    StateMoveRobotOffRail(
        shared_ptr<AbstractStateMachineManager> state_manager, ros::NodeHandle& nh,
        shared_ptr<StateFixOrientation> state_fix_orientation
    );
    ~StateMoveRobotOffRail() override;
    void runState() override;
    bool isStateFinished() override;
    StateTypes getNextStateName() override;
    void reset() override;
    void finish() override;
    void init() override;
    void interruptState() override;
    void proximityCb(const std_msgs::UInt8ConstPtr &msg);
private:
    void evaluateStateStatus() override;
private:
    ros::NodeHandle nh_;
    nav_msgs::Odometry curr_pose_;
    nav_msgs::Odometry initial_pose_;
    PoseUtilities p_util_;
    tf2::Transform tf_init2orig_;
    ros::Time start_time;
    bool collision_flag_;
    ros::Time safety_stop_time_;
    float dismount_distance_;
    bool fixing_orientation_;
    shared_ptr<StateFixOrientation> state_fix_orientation_;

    bool immediate_recovery_;
};

} // end of namespace ais_state_machine
#endif // END ASM_STATE_MOVE_ROBOT_OFF_H_
