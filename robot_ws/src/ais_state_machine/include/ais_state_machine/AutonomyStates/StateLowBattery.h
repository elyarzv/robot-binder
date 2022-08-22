/*
    AIS CONFIDENTIAL
    Author: Michael Wrock (m.wrock@ai-systems.ca)
*/

#ifndef ASM_STATE_LOW_BATTERY_H_
#define ASM_STATE_LOW_BATTERY_H_

#include "ros/ros.h"
#include "ais_state_machine/AbstractClasses/AbstractState.h"
#include "ais_state_machine/AbstractClasses/AbstractROSState.h"
namespace ais_state_machine 
{

class StateLowBattery final: public AbstractState, public AbstractROSState
{
public: 
    StateLowBattery(shared_ptr<AbstractStateMachineManager> state_manager, ros::NodeHandle& nh);
    ~StateLowBattery() override;
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
};

} // end of namespace ais_state_machine
#endif // END ASM_STATE_LOW_BATTERY_H_