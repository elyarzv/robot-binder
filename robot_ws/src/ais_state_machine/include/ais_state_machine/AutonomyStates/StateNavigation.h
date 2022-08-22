/*
    AIS CONFIDENTIAL
    Author: Rakesh Shrestha (r.shrestha@ai-systems.ca)
*/

#ifndef ASM_STATE_NAVIGATION_H_
#define ASM_STATE_NAVIGATION_H_

#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "actionlib/client/simple_action_client.h"

#include "ais_state_machine/AbstractClasses/AbstractState.h"
#include "ais_state_machine/AbstractClasses/AbstractROSState.h"
#include "phoenix_msgs/SendRailGoalAction.h"
#include "phoenix_msgs/StartLocalization.h"

namespace ais_state_machine
{

class StateNavigation final: public AbstractState, public AbstractROSState
{
public:
    StateNavigation(shared_ptr<AbstractStateMachineManager> state_manager, ros::NodeHandle& nh);
    ~StateNavigation() override;
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
    int count;
    bool reached_goal_;
    ros::Publisher cmd_vel_publisher_;

    ros::ServiceClient activate_localization_client_;
    ros::ServiceClient deactivate_localization_client_;
    ros::ServiceClient deactivate_slam_client_;

    actionlib::SimpleActionClient<phoenix_msgs::SendRailGoalAction> send_goal_client_;
};

} // end of namespace ais_state_machine
#endif // END ASM_STATE_NAVIGATION_H_

