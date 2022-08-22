/*
    AIS CONFIDENTIAL
    Author: Michael Chiou (m.chiou@ai-systems.ca)
*/

#ifndef ASM_STATE_ADJUST_TO_RAIL_H_
#define ASM_STATE_ADJUST_TO_RAIL_H_

#define safety_timeout 1
#define WINDOW_SIZE 50

#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "ais_state_machine/AbstractClasses/AbstractState.h"
#include "ais_state_machine/AbstractClasses/AbstractROSState.h"
#include "ais_state_machine/Helpers/RailHeadPoseManager.h"

namespace ais_state_machine 
{

class StateAdjustToRail final: public AbstractState, public AbstractROSState
{
public: 
    StateAdjustToRail(shared_ptr<AbstractStateMachineManager> state_manager, ros::NodeHandle& nh);
    ~StateAdjustToRail() override;
    void runState() override;
    bool isStateFinished() override;
    StateTypes getNextStateName() override;
    void reset() override;
    void finish() override;
    void init() override;
    void interruptState() override;
private: 
    void evaluateStateStatus() override;
    void useLinearRailAdjustment();
    void useAngularRailAdjustment();
private:
    geometry_msgs::Pose target_pose_;
    geometry_msgs::Pose current_rail_pose_;
    double local_error_x_;
    double local_error_y_;
    double local_error_angle_;
    double tol_x_;
    double tol_y_;
    double tol_angle_;
    float rolling_window;

    float cmd_speed_x_;
    float cmd_speed_y_;
    float cmd_angular_;

    double state_timeout_;

    shared_ptr<RailHeadPoseManager> rail_head_ptr_;
    double time_since_last_error_;
    bool collision_flag_;
    ros::Time safety_stop_time_;
    

};

} // end of namespace ais_state_machine
#endif // END ASM_STATE_ADJUST_TO_RAIL_H_