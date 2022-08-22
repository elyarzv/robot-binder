// AIS CONFIDENTIAL

// Author: Michael Chiou (m.chiou@ai-systems.ca)

#ifndef ASM_ABSTRACT_STATE_MACHINE_MANAGER_H_
#define ASM_ABSTRACT_STATE_MACHINE_MANAGER_H_

#include <memory>
#include <string>
#include <vector>
#include "ros/ros.h"
#include "ais_state_machine/AbstractClasses/AbstractState.h"
#include "ais_state_machine/AbstractClasses/AbstractStateFactory.h"
#include "ais_state_machine/AbstractClasses/AbstractGoalHandler.h"
#include "ais_state_machine/Enums/phoenix_state_enums.h"
#include "ais_state_machine/Enums/robot_status.h"
#include "ais_state_machine/Helpers/MissionConfig.h"
#include "ais_state_machine/Helpers/UVLampController.h"
#include "ais_state_machine/Helpers/MissionGoalManager.h"
#include "ais_state_machine/Helpers/RobotBumperHandler.h"
#include "ais_state_machine/Helpers/RobotStatusManager.h"
#include "ais_state_machine/Helpers/RailHeadPoseManager.h"
#include "ais_state_machine/Helpers/RobotVelocityController.h"
#include "ais_state_machine/Helpers/WheelOdomManager.h"
#include "ais_state_machine/Helpers/RailStatusManager.h"

using std::shared_ptr;
using std::string;

namespace ais_state_machine {
// Forward class declaration
class AbstractState;
class AbstractStateFactory;
class RobotStatusManager;
class AbstractStateMachineManager {
public: 
    /*
    * Constructor for StateMachineManager, takes in inputs which define initial state of 
    * StateMachine and factory which will generate all necessary states 
    */
    AbstractStateMachineManager(ros::NodeHandle& nh, 
                                StateTypes init_state,
                                shared_ptr<AbstractStateFactory> factory_ptr);
    /*
    * Virtual Destructor
    */
    virtual ~AbstractStateMachineManager() = default;
    /*
    * Calls AbstractState runState function (executed in ros while loop)
    */   
    virtual void runCurrentStateOnce() = 0;
    /*
    * Given an enumerated type, use the provided factory object to transition to next state
    */   
    virtual void transitionToNextState(StateTypes input) = 0;
    /*
    * Determine is current state is finished by calling isStateFinished() from pointer to state
    */   
    virtual bool isCurrentStateFinished() = 0;
    /*
    * Reset prev_state_name_ to UNKNOWN
    */   
    virtual void resetPreviousStatePtr() = 0;
    /*
    * Return internal ROS Node Handler if internal members require access to it
    */
    ros::NodeHandle& getInternalNodeHandler();
    shared_ptr<MissionGoalManager> mission_goal_manager_;

    // Carries persistent mission configuration booleans and other assorted values
    shared_ptr<MissionConfig> mission_config_ptr_;
    // TODO (Michael Chiou) Make AbstractClass for testing later
    shared_ptr<UVLampController> uv_lamp_controller_ptr_; 

    shared_ptr<RobotBumperHandler> bumper_handler_ptr_;
    shared_ptr<RobotStatusManager> status_manager_ptr_;
    shared_ptr<RailHeadPoseManager> front_rail_manager_ptr_;
    shared_ptr<RailHeadPoseManager> back_rail_manager_ptr_;
    shared_ptr<RobotVelocityController> vel_controller_ptr_;
    shared_ptr<WheelOdomManager> wheel_odom_ptr_;
    shared_ptr<RailStatusManager> rail_status_ptr_;
    
    
protected:
    ros::NodeHandle nh_;
    StateTypes curr_state_name_;
    StateTypes prev_state_name_;
    shared_ptr<AbstractState> state_ptr_;
    shared_ptr<AbstractStateFactory> factory_ptr_;
    
};

} // end namespace ais_state_machine

#endif // END ASM_ABSTRACT_STATE_MACHINE_MANAGER_H_

