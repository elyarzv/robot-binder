/*
    AIS CONFIDENTIAL
    Author: Michael Chiou (m.chiou@ai-systems.ca)
*/

#ifndef ASM_STATE_COMPLETE_H_
#define ASM_STATE_COMPLETE_H_

#include "ais_state_machine/AbstractClasses/AbstractState.h"
#include "ais_state_machine/AbstractClasses/AbstractROSState.h"
#include "std_msgs/UInt8.h"
namespace ais_state_machine 
{

class StateComplete final: public AbstractState, public AbstractROSState
{
public: 
    StateComplete(shared_ptr<AbstractStateMachineManager> state_manager, ros::NodeHandle& nh);
    ~StateComplete() override;
    void runState() override;
    bool isStateFinished() override;
    StateTypes getNextStateName() override;
    void interruptState() override;
    void reset() override;
    void finish() override;
    void init() override;
private: 
    void evaluateStateStatus() override;
private:
    int count;
    ros::Publisher cmd_vel_publisher_;
    ros::Publisher rosbag_publisher_;
};

} // end of namespace ais_state_machine
#endif // END ASM_STATE_COMPLETE_H_