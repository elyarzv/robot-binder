/*
    AIS CONFIDENTIAL
    Author: Michael Chiou (m.chiou@ai-systems.ca)
*/

#ifndef ASM_STATE_RAIL_DISINFECTION_H_
#define ASM_STATE_RAIL_DISINFECTION_H_
#define RECOVERY_TIME 2
#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "ais_state_machine/AbstractClasses/AbstractState.h"
#include "ais_state_machine/AbstractClasses/AbstractROSState.h"
namespace ais_state_machine 
{
/**
 * @brief This State is intended to execute control logic for disinfection on the rail all the way
 *          until the specified end point from MissionGoalManager is reached. This may include the
 *          end of the rail or some intermediate points in the rail. Disinfection will exit when 
 *          the corresponding bumper in the direction of travel is triggered or when all points are
 *          reached
 */
class StateRailDisinfection final: public AbstractState, public AbstractROSState
{
public: 
    StateRailDisinfection(shared_ptr<AbstractStateMachineManager> state_manager, ros::NodeHandle& nh);
    ~StateRailDisinfection() override;
    void runState() override;
    bool isStateFinished() override;
    StateTypes getNextStateName() override;
    void reset() override;
    void finish() override;
    void init() override;
    void interruptState() override;
private: 
    void evaluateStateStatus() override;
    bool isRobotPosAfterGoalPos();
    bool isReversedRobotPosAfterGoalPos();
    bool doesRobotNeedToTransition();
    std_msgs::Float32 getCurrentPose();

private:
    int count;
    geometry_msgs::Twist cmd_vel;
    
    ros::Publisher cmd_vel_publisher_;
    ros::Publisher led_publisher_;
    
    std_msgs::Float32 curr_robot_rail_pose_;
    bool is_waiting_at_goal_;
    double start_pause_time_;
    double end_pause_time_;
    ros::Time last_time_in_motion_;
    
};

} // end of namespace ais_state_machine
#endif // END ASM_STATE_RAIL_DISINFECTION_H_