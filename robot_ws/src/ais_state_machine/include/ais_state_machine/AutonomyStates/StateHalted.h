/*
    AIS CONFIDENTIAL
    Author: Michael Wrock (m.wrock@ai-systems.ca)
*/

#ifndef ASM_STATE_HALTED_H_
#define ASM_STATE_HALTED_H_

#include "ros/ros.h"
#include "ais_state_machine/AbstractClasses/AbstractState.h"
#include "ais_state_machine/AbstractClasses/AbstractROSState.h"
namespace ais_state_machine 
{

class StateHalted final: public AbstractState, public AbstractROSState
{
public: 
    StateHalted(shared_ptr<AbstractStateMachineManager> state_manager, ros::NodeHandle& nh);
    ~StateHalted() override;
    void runState() override;
    bool isStateFinished() override;
    StateTypes getNextStateName() override;
    void reset() override;
    void finish() override;
    void init() override;
    void interruptState() override;
private: 

    ros::Publisher cmd_vel_publisher_;
    void evaluateStateStatus() override;
};

} // end of namespace ais_state_machine
#endif // END ASM_STATE_HALTED_H_