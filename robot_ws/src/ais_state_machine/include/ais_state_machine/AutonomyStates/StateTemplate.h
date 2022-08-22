/*
    AIS CONFIDENTIAL
    Author: Michael Chiou (m.chiou@ai-systems.ca)
*/

#ifndef ASM_STATE_TEMPLATE_H_
#define ASM_STATE_TEMPLATE_H_

#include "ros/ros.h"
#include "ais_state_machine/AbstractClasses/AbstractState.h"
#include "ais_state_machine/AbstractClasses/AbstractROSState.h"
namespace ais_state_machine 
{

class StateTemplate final: public AbstractState, public AbstractROSState
{
public: 
    StateTemplate(shared_ptr<AbstractStateMachineManager> state_manager, ros::NodeHandle& nh);
    ~StateTemplate() override;
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
#endif // END ASM_STATE_TEMPLATE_H_