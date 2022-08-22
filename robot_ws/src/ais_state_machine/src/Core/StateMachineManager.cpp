#include "ais_state_machine/Core/StateMachineManager.h"
#include "std_srvs/Trigger.h"

using namespace ais_state_machine;

StateMachineManager::StateMachineManager(ros::NodeHandle& nh, 
                                        StateTypes init_state,
                                        shared_ptr<AbstractStateFactory> state_factory_ptr) 
    : AbstractStateMachineManager(nh, init_state, state_factory_ptr)
{
	ros::NodeHandle private_nh_("~");
    ROS_INFO("StateMachine Receive %d", (int) init_state);
    
    // ToDo(Michael Chiou) Consider moving some of these objects to initialize() function
    // This is to allow for 'resetting' the state machine for testing purposes
    set_interrupted_flag_ = false;
    mission_config_ptr_ = make_shared<MissionConfig>();
    uv_lamp_controller_ptr_ = make_shared<UVLampController>(nh);
    mission_goal_manager_ = make_shared<MissionGoalManager>();
    bumper_handler_ptr_ = make_shared<RobotBumperHandler>(nh);
    rail_status_ptr_ = make_shared<RailStatusManager>(nh, "/embedded/rail_encoder", mission_config_ptr_);
    status_manager_ptr_ = make_shared<RobotStatusManager>(nh, private_nh_,mission_goal_manager_,bumper_handler_ptr_,rail_status_ptr_,uv_lamp_controller_ptr_);
    autonomy_version_ptr_ = make_shared<VersionController>(nh);
    front_rail_manager_ptr_ = make_shared<RailHeadPoseManager>(nh, "/perception/rail_head_point_front", 1.0);
    back_rail_manager_ptr_ = make_shared<RailHeadPoseManager>(nh, "/perception/rail_head_point_back", 1.0);
    vel_controller_ptr_ = make_shared<RobotVelocityController>(nh, "/embedded/cmd_vel");
    wheel_odom_ptr_ = make_shared<WheelOdomManager>(nh, "/embedded/wheelodom");
    change_mode_service_ = nh.advertiseService("set_robot_mode", &StateMachineManager::changeModeCB, this);
    state_name_pub_ = nh.advertise<std_msgs::Int32>("current_state", 1);
    state_change_request_ = false;
    prev_state_name_ = StateTypes::UNKNOWN;
    nh.param<bool>("DIAGNOSTIC_SAFETY", diagnostic_safety_, true);
}

StateMachineManager::~StateMachineManager() 
{
    ROS_WARN("StateMachineManager destructed");
}

bool StateMachineManager::changeModeCB(phoenix_msgs::setMode::Request &req, phoenix_msgs::setMode::Response &res)
{
    ROS_INFO("Change mode request: %s", req.mode.c_str());
    if(curr_state_name_ == StateTypes::CRITICAL_BATTERY && !status_manager_ptr_->batteryRecharged()){
        ROS_ERROR("Remote attempt to override critical battery");
        res.response = false;
        return true;
    }
    else if(req.mode.compare("TELEOP") == 0){
        if(curr_state_name_ != StateTypes::MANUAL_CONTROL){
            requested_state_ = StateTypes::TELEOPERATION;
        }else{
            ROS_ERROR("Remote attempt to override manual");
            res.response = false;
            return true;
        }
    }else if(req.mode.compare("MANUAL_OPERATION") == 0){
        requested_state_ = StateTypes::MANUAL_CONTROL;
    }else if(req.mode.compare("MANUAL_TREATMENT") == 0){
        requested_state_ = StateTypes::MANUAL_TREATMENT;
    }else if(req.mode.compare("NAVIGATION") == 0){
        requested_state_ = StateTypes::NAVIGATION;
    }else if(req.mode.compare("FIX_ORIENTATION") == 0){
        requested_state_ = StateTypes::FIX_ORIENTATION;
    }else if(req.mode.compare("MOUNT") == 0){
        requested_state_ = StateTypes::MOUNT_ROBOT_TO_RAIL;
    }else if(req.mode.compare("INITIALIZE") == 0){
        if(curr_state_name_ == StateTypes::TELEOPERATION){
            requested_state_ = StateTypes::INITIALIZE_ROBOT;
        }else{
            ROS_ERROR("Attempt to initialize from %s", req.mode.c_str());
            res.response = false;
            return true;
        }
    }else if(req.mode.compare("AUTO") == 0){
        if(prev_state_name_ == StateTypes::UNKNOWN){
            ROS_ERROR("AUTO state change request without previous state to return to");
            requested_state_ = StateTypes::IDLE;
            state_change_request_ = true;
            res.response = false;
            return true;
        }else{
            ROS_INFO("Returning to state %d", prev_state_name_);
            requested_state_ = prev_state_name_;
        }        //If there is some error flags to be cleared, do it here
    }else if(req.mode.compare("RESET") == 0){
        requested_state_ = StateTypes::COMPLETE;
    }else{
        ROS_ERROR("Unknown mode request: %s", req.mode.c_str());
        res.response = false;
        return false;
    }
    res.response = true;
    state_change_request_ = true;
    return true;
}

void StateMachineManager::runCurrentStateOnce() 
{
    //ROS_INFO("SM: Executing one loop in State (%d)", (int) curr_state_name_);
    state_ptr_->runState();
}

bool StateMachineManager::isCurrentStateFinished()
{
    return state_ptr_->isStateFinished();
}

void StateMachineManager::transitionToNextState(StateTypes state_enum) 
{
    //ROS_INFO("SM: Attempting transition from %d to %d prev %d", (int) curr_state_name_,(int) state_enum, (int) prev_state_name_);
    if (factory_ptr_ == nullptr) {
        ROS_ERROR("Null factory pointer!!");
    }
    auto next_state_ptr = factory_ptr_->generateState(state_enum);
    if (next_state_ptr == nullptr) {
        ROS_ERROR("SM : (%d) state was not a valid state name", (int) state_enum);
    } else {
        if (state_ptr_ != next_state_ptr) { 
            state_ptr_->finish(); // When transitioning states, call the finish() function as we leave and the init() function when we enter
            next_state_ptr->init();
            if(savePreviousState()){
                if(curr_state_name_ == StateTypes::INITIALIZE_ROBOT){ //Since we only execute initialize once we should not return to it. For this special case we set the prev state pointer to rail disinfection
                    prev_state_name_ = StateTypes::RAIL_DISINFECTION;
                }else{
                    prev_state_name_ = curr_state_name_;
                }
            }
        }
        state_ptr_ = next_state_ptr;
        curr_state_name_ = state_enum;
        resetPreviousStatePtr();
        //ROS_INFO("SM: State is now %d", (int) curr_state_name_);
    }
}

bool StateMachineManager::offRailDetectionEnabled(){

    //Check if all off-rail detection is enabled
    if(!status_manager_ptr_->off_rail_detection){
        return false;
    }
    //Check if enabled for manual treatemant and in manual mode
    else if(status_manager_ptr_->getRobotStatus() == RobotStatus::MANUAL_TREATMENT && status_manager_ptr_->off_rail_detection_semi_autonomous){
        return true;
    }
    //Check if enabled for autonomous mode and in AUTO
    else if(status_manager_ptr_->getRobotStatus() == RobotStatus::AUTO && status_manager_ptr_->off_rail_detection_autonomous){
        return true;
    }
    else{
        return false;
    }

}

void StateMachineManager::interruptCurrentState(){

    //Check to see if /stop_mission service was called
    if(status_manager_ptr_->abortMission()){
        ROS_WARN_THROTTLE(1,"Aborting mission");
        //Clear the flag set by /stop_mission
        status_manager_ptr_->setMissionAbort(false);
        transitionToNextState(StateTypes::COMPLETE);
    //Check to see if battery is low
    }else if(status_manager_ptr_->lowRobotBattery()){
        if(status_manager_ptr_->isBatteryCritical()){
            ROS_ERROR("Battery level is critical. Stopping");
            transitionToNextState(StateTypes::CRITICAL_BATTERY);
        }else{
            if(state_change_request_ && requested_state_ == StateTypes::MANUAL_CONTROL){
                ROS_WARN_THROTTLE(1,"Overriding low battery for manual control");
                transitionToNextState(StateTypes::MANUAL_CONTROL);
            }
            else if(status_manager_ptr_->isRobotOutOfTheWay(curr_state_name_) || (state_change_request_ && curr_state_name_ == StateTypes::MANUAL_CONTROL)){
                ROS_WARN_THROTTLE(1,"Battery low detected");
                transitionToNextState(StateTypes::LOW_BATTERY);
            }else{
                ROS_WARN_THROTTLE(1,"Battery low detected but robot in an incovenient location");
                transitionToNextState(state_ptr_->getNextStateName());
            }
        }
    //Check to see if robot unintentially drove off the rails
    }else if(!status_manager_ptr_->isRobotOnRails()){
        ROS_INFO_THROTTLE(1,"Robot is not on rails");
        if(offRailDetectionEnabled()){
            transitionToNextState(StateTypes::HALTED);
        }else{
            //If off rail detection is diabled, the normal thing to do is call transitionToNextState(state_ptr_->getNextStateName());
            ROS_WARN_THROTTLE(1,"Off rail flag set to: %d (auto) %d (semi auto) %d (all)",status_manager_ptr_->off_rail_detection_autonomous,status_manager_ptr_->off_rail_detection_semi_autonomous,status_manager_ptr_->off_rail_detection);
            transitionToNextState(state_ptr_->getNextStateName());
        }
    //Check if E-Stop is pressed (by detecting AUX power loss)
    }else if(status_manager_ptr_->isEStopPressed()){
        ROS_ERROR_THROTTLE(1,"AUX power loss detected (Previous state %lu)",curr_state_name_);
        transitionToNextState(StateTypes::HALTED);
    //Check if robot diagnostics are ok
    }else if(status_manager_ptr_->robot_diagnostic_->isRobotError()){

        Json::FastWriter fastWriter;
        std::string error_string_;
        error_string_ = fastWriter.write(status_manager_ptr_->robot_diagnostic_->getRobotErrorCodes()).c_str();
        ROS_ERROR_STREAM_THROTTLE(1,"Diagnostic failure" << error_string_);
        if(diagnostic_safety_){
            if(status_manager_ptr_->robot_diagnostic_->getRobotDiagnosticLevel().compare("CRITICAL") == 0){
                ROS_ERROR_THROTTLE(1,"Transitioning to HALTED immediately");
                transitionToNextState(StateTypes::HALTED);
            }else if(status_manager_ptr_->robot_diagnostic_->getRobotDiagnosticLevel().compare("MODERATE") == 0){
                ROS_WARN_THROTTLE(1,"Moderate failure, waiting to return to rail home");
                if(status_manager_ptr_->isRobotOutOfTheWay(curr_state_name_)){
                    ROS_WARN_THROTTLE(1,"Robot is out of the way entering halted from %d", curr_state_name_);
                    transitionToNextState(StateTypes::HALTED);
                }else{
                    ROS_WARN_THROTTLE(1,"Robot is not out of the way yet");
                    transitionToNextState(state_ptr_->getNextStateName());
                }
            }
        } 
        else{
            ROS_WARN_THROTTLE(1,"Ignoring diagnostic error");
            if(state_change_request_ && requested_state_ != curr_state_name_){
                ROS_WARN_THROTTLE(1,"Changing mode from %lu to %lu",curr_state_name_,requested_state_);
                transitionToNextState(requested_state_);
            }else{
                transitionToNextState(state_ptr_->getNextStateName());
            }
        }
    //Check if /set_robot_mode was used to change states
    }else if(state_change_request_){
        state_change_request_ = false;
        if(requested_state_ != curr_state_name_){
        //if(status_manager_ptr_->getRobotStatus() == RobotStatus::IDLE){ //Add exclusions here to prevent state switching if needed
            ROS_WARN_THROTTLE(1,"Leaving AUTO mode from %lu to %lu",curr_state_name_,requested_state_);
            transitionToNextState(requested_state_);
        }else{
            ROS_WARN_THROTTLE(1,"State change request for the current state received");
        }
    //Check if battery has been recharged and automatically resume
    // }else if((curr_state_name_ == StateTypes::LOW_BATTERY || curr_state_name_ == StateTypes::CRITICAL_BATTERY)
    //              && status_manager_ptr_->batteryRecharged()){
    //     if(prev_state_name_ == StateTypes::UNKNOWN){
    //         ROS_INFO_THROTTLE(1,"Battery recharged, going to IDLE");
    //         curr_state_name_ = StateTypes::IDLE;
    //     }else if(prev_state_name_ == StateTypes::IDLE ||
    //                add more states to automatically resume here){
    //         ROS_INFO_THROTTLE(1,"Battery recharged, going to previous state %d", prev_state_name_);
    //         curr_state_name_ = prev_state_name_;
    //     }
    }else{ //Just in case interrupts clear before this gets executed
        ROS_WARN("Errors cleared");
        transitionToNextState(state_ptr_->getNextStateName());
    }

}

void StateMachineManager::run() 
{   
    initialize();
    ros::Rate loop_rate(25);
    
    while (ros::ok()) {

        ROS_DEBUG("SM: Starting Loop with %d",(int) curr_state_name_);

        // General Autonomy States
        if (!status_manager_ptr_->interruptStateMachine() && !state_change_request_) {
            transitionToNextState(state_ptr_->getNextStateName());
        } else { // External Influence starts here
            interruptCurrentState();
        }
        runCurrentStateOnce();
        status_manager_ptr_->update();
        publishCurrentState();
        ros::spinOnce();
        loop_rate.sleep();        
    }
}

void StateMachineManager::publishCurrentState()
{
    std_msgs::Int32 msg;
    msg.data = (int) curr_state_name_;
    state_name_pub_.publish(msg);
}

void StateMachineManager::initialize()
{
    factory_ptr_->setStateMachineManager(shared_from_this());
    factory_ptr_->initializeStates();
    state_ptr_ = factory_ptr_->generateState(curr_state_name_);
}
void StateMachineManager::resetPreviousStatePtr()
{
    //Invalidate the prev_state_pointer if we enter any state that resumes the mission
    if(savePreviousState()){
        prev_state_name_ = StateTypes::UNKNOWN;
    }else{
        //Add additional states that should invalidate the prev_state_pointer
        switch (curr_state_name_)
        {
            case StateTypes::COMPLETE:
            case StateTypes::IDLE:
                prev_state_name_ = StateTypes::UNKNOWN;
                break;
            default:
                break;
        }
    }

}

bool StateMachineManager::savePreviousState(){
    switch (curr_state_name_)
    {
        //List all states we can return to here
        case StateTypes::RAIL_DISINFECTION:
        case StateTypes::RETURN_TO_RAIL_HOME:
        case StateTypes::ADJUST_TO_RAIL:
        case StateTypes::MOVE_ROBOT_OFF_RAIL:
        case StateTypes::MOVE_TO_NEXT_RAIL:
        case StateTypes::MOUNT_ROBOT_TO_RAIL:
        case StateTypes::MANUAL_TREATMENT:
        case StateTypes::INITIALIZE_ROBOT:
            return true;
            break;
        default:
            return false;
            break;
    }
}