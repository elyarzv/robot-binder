#include "ais_state_machine/AutonomyStates/StateRecovery.h"

using namespace ais_state_machine;

StateRecovery::StateRecovery(shared_ptr<AbstractStateMachineManager> state_manager, ros::NodeHandle& nh) 
    : AbstractState(state_manager), AbstractROSState(nh)
{
    next_state_name_ = StateTypes::RECOVERY;

}

StateRecovery::~StateRecovery() 
{

}

void StateRecovery::runState()
{
    ROS_INFO_THROTTLE(1,"State Recovery");
    state_manager_->vel_controller_ptr_->sendVelocityCmd(0, 0, 0);
    state_manager_->uv_lamp_controller_ptr_->setUVLampOn(false);
    ros::spinOnce();
    evaluateStateStatus();
}

bool StateRecovery::isStateFinished()
{
    ROS_INFO_THROTTLE(1,"State Recovery Status: %s", has_finished_state_ ? "true" : "false");
    return has_finished_state_;
}

StateTypes StateRecovery::getNextStateName()
{
    return next_state_name_;
}

void StateRecovery::evaluateStateStatus() 
{
    state_manager_->status_manager_ptr_->setRobotStatus(RobotStatus::RECOVERY);
}

void StateRecovery::reset() 
{
    next_state_name_ = StateTypes::RECOVERY;
}

void StateRecovery::finish()
{
    ROS_INFO("StateRecovery is now exiting");
    reset();
}

void StateRecovery::init()
{
    ROS_INFO("StateRecovery is initializing");
    reset();
}

void StateRecovery::interruptState()
{
    ROS_INFO("StateRecovery is interrupted");
}