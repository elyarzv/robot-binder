/*
    AIS CONFIDENTIAL
    Author: Michael Chiou (m.chiou@ai-systems.ca)
*/

#ifndef ASM_STATE_MANUAL_CONTROL_H_
#define ASM_STATE_MANUAL_CONTROL_H_

#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "ais_state_machine/AbstractClasses/AbstractState.h"
#include "ais_state_machine/AbstractClasses/AbstractROSState.h"
namespace ais_state_machine 
{

class StateManualControl final: public AbstractState, public AbstractROSState
{
public: 
    StateManualControl(shared_ptr<AbstractStateMachineManager> state_manager, ros::NodeHandle& nh);
    ~StateManualControl() override;
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
    ros::Publisher cmd_vel_publisher_;
};

} // end of namespace ais_state_machine
#endif // END ASM_STATE_MANUAL_CONTROL_H_