/*
    AIS CONFIDENTIAL
    Author: Michael Chiou (m.chiou@ai-systems.ca)
*/

#ifndef ASM_STATE_TEMPLATE_H
#define ASM_STATE_TEMPLATE_H

#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "std_msgs/UInt8.h"
#include "ais_state_machine/AbstractClasses/AbstractState.h"
#include "ais_state_machine/AbstractClasses/AbstractROSState.h"
#include <vector>
using std::vector;

namespace ais_state_machine 
{

/**
 * @brief This State is intended to initialize and load mission parameters and configurations.
 *          Mision Goals and other setup procedures should also be executed here before proceeding 
 */

class StateInitializeRobot final: public AbstractState, public AbstractROSState
{
public: 
    StateInitializeRobot(shared_ptr<AbstractStateMachineManager> state_manager, ros::NodeHandle& nh);
    ~StateInitializeRobot() override;
    void runState() override;
    bool isStateFinished() override;
    StateTypes getNextStateName() override;
    void reset() override;
    void finish() override;
    void init() override;
    void interruptState() override;
private: 
    void evaluateStateStatus() override;
    void initializeAndLoadMissionParameters();
    void incrementMissionCounter();
    bool doesMissionHaveGoals();
    void initializeMissionParams();
    void initializeMissionWaitTime();
private:
    bool all_params_initialized;
    // the first target rail to be disinfected may not be rail 1
    // in which case we first need to go to the target rail and then disinfect it
    bool is_disinfecting_current_target_;
    int count;
    vector<string> param_list_;
    ros::Publisher rosbag_publisher_;
    ros::Publisher flashlight_publisher_;

    /** time when mission was initiated */
    ros::Time mission_initialization_time_;
    /** wait time before starting mission */
    ros::Duration mission_wait_time_;
};

} // end of namespace ais_state_machine
#endif // END ASM_STATE_TEMPLATE_H
