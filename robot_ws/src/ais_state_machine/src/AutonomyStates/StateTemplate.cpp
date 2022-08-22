#include "ais_state_machine/AutonomyStates/StateTemplate.h"

using namespace ais_state_machine;

StateTemplate::StateTemplate(shared_ptr<AbstractStateMachineManager> state_manager, ros::NodeHandle& nh) 
    : AbstractState(state_manager), AbstractROSState(nh)
{
    next_state_name_ = StateTypes::TEMPLATE;

}

StateTemplate::~StateTemplate() 
{

}

void StateTemplate::runState()
{
    ROS_INFO("State Template: Next State is %d", (int) next_state_name_);

    evaluateStateStatus();
}

bool StateTemplate::isStateFinished()
{
    ROS_INFO("State Template Status: %s", has_finished_state_ ? "true" : "false");
    return has_finished_state_;
}

StateTypes StateTemplate::getNextStateName()
{
    return next_state_name_;
}

void StateTemplate::evaluateStateStatus() 
{
}

void StateTemplate::reset() 
{
    next_state_name_ = StateTypes::TEMPLATE;
}

void StateTemplate::finish()
{
    ROS_INFO("StateTemplate is now exiting");
    reset();
}

void StateTemplate::init()
{
    ROS_INFO("StateTemplate is initializing");
    reset();
    //state_manager_->status_manager_ptr_->setRobotStatus(RobotStatus::ROBOT_STATUS);
}

void StateTemplate::interruptState()
{
    ROS_INFO("StateTemplate is interrupted");
}