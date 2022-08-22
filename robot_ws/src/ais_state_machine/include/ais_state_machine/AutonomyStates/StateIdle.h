/*
    AIS CONFIDENTIAL
    Author: Michael Chiou (m.chiou@ai-systems.ca)
*/

#ifndef ASM_STATE_IDLE_H
#define ASM_STATE_IDLE_H

#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "ais_state_machine/AbstractClasses/AbstractState.h"
#include "ais_state_machine/AbstractClasses/AbstractROSState.h"
#include "phoenix_msgs/startMission.h"
namespace ais_state_machine 
{

class StateIdle final: public AbstractState, public AbstractROSState
{
public: 
    StateIdle(shared_ptr<AbstractStateMachineManager> state_manager, ros::NodeHandle& nh);
    ~StateIdle() override;
    void runState() override;
    bool isStateFinished() override;
    StateTypes getNextStateName() override;
    void reset() override;
    void finish() override;
    void init() override;
    void interruptState() override;
private: 
    void evaluateStateStatus() override;
    bool startMissionCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res); //Sends state machine to INITIALIZE state, which uses ros param MISSION_NAME
    bool startNamedMissionCb(phoenix_msgs::startMission::Request &req, phoenix_msgs::startMission::Response &res); //Sets ros param MISSION_NAME by adding ".csv" to req.mission and sends state machine to INITIALIZE state
private:
    int count;
    ros::Publisher cmd_vel_publisher_;
    ros::ServiceServer start_mission_service_;
    ros::ServiceServer start_named_mission_service_;
};

} // end of namespace ais_state_machine
#endif // END ASM_STATE_IDLE_H