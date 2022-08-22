/*
    AIS CONFIDENTIAL
    Author: Michael Chiou (m.chiou@ai-systems.ca)
*/

#ifndef ASM_STATE_MACHINE_MANAGER_H_
#define ASM_STATE_MACHINE_MANAGER_H_

#include <memory>
#include <iostream>
#include <fstream>
#include <sys/types.h>
#include <dirent.h>

#include "ros/ros.h"
#include "ais_state_machine/AbstractClasses/AbstractStateMachineManager.h"
#include "ais_state_machine/Core/StateFactory.h"
#include "ais_state_machine/Helpers/RobotStatusManager.h"
#include "ais_state_machine/Enums/phoenix_state_enums.h"
#include "ais_state_machine/Helpers/VersionController.h"
#include "phoenix_msgs/setMode.h"
#include "std_msgs/Int32.h"

namespace ais_state_machine {

class AbstractStateFactory; // forward declaration
class RobotStatusManager; // forward declaration

class StateMachineManager : public AbstractStateMachineManager, 
                            public std::enable_shared_from_this<StateMachineManager>
{
public:
    StateMachineManager(ros::NodeHandle& nh, 
                        StateTypes init_state,
                        shared_ptr<AbstractStateFactory> state_factory_ptr);
    ~StateMachineManager() override;
    StateMachineManager(StateMachineManager&&) = default;
    void runCurrentStateOnce() override;
    void transitionToNextState(StateTypes input) override;
    bool isCurrentStateFinished() override;
    /**
     * @brief Main run loop for autonomous missions
     * 
     */
    void run();
    /**
     * @brief Initialize helper modules for running the state machine for disinfections
     * 
     */
    void initialize();

private:
    void interruptCurrentState();
    void publishCurrentState();
    void resetPreviousStatePtr();
    bool savePreviousState();
    shared_ptr<VersionController> autonomy_version_ptr_;
    bool changeModeCB(phoenix_msgs::setMode::Request &req, phoenix_msgs::setMode::Response &res);
    bool offRailDetectionEnabled();
    bool set_interrupted_flag_;
    ros::ServiceServer change_mode_service_;
    ros::Publisher state_name_pub_;
    StateTypes requested_state_;
    bool state_change_request_;
    bool diagnostic_safety_;
};

} // end namespace ais_state_machine

#endif // End of ASM_STATE_MACHINE_MANAGER_H_
