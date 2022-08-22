#include "ais_state_machine/AutonomyStates/StateLowBattery.h"

using namespace ais_state_machine;

StateLowBattery::StateLowBattery(shared_ptr<AbstractStateMachineManager> state_manager, ros::NodeHandle& nh) 
    : AbstractState(state_manager), AbstractROSState(nh)
{
    next_state_name_ = StateTypes::LOW_BATTERY;

}

StateLowBattery::~StateLowBattery() 
{

}

void StateLowBattery::runState()
{
    ROS_INFO_THROTTLE(1,"State Low Battery %f",state_manager_->status_manager_ptr_->getSoC());
    state_manager_->uv_lamp_controller_ptr_->setUVLampOn(false);
    state_manager_->vel_controller_ptr_->sendVelocityCmd(0, 0, 0);
    state_manager_->status_manager_ptr_->setRobotStatus(RobotStatus::LOW_BATTERY);
    evaluateStateStatus();
}

bool StateLowBattery::isStateFinished()
{
    ROS_INFO("State Low Battery Status: %s", has_finished_state_ ? "true" : "false");
    return has_finished_state_;
}

StateTypes StateLowBattery::getNextStateName()
{
    return next_state_name_;
}

void StateLowBattery::evaluateStateStatus() 
{
}

void StateLowBattery::reset() 
{
    next_state_name_ = StateTypes::LOW_BATTERY;
}

void StateLowBattery::finish()
{
    ROS_INFO("StateLowBattery is now exiting");
    reset();
}

void StateLowBattery::init()
{
    ROS_INFO("StateLowBattery is initializing");
    reset();
}

void StateLowBattery::interruptState()
{
    ROS_INFO("StateLowBattery is interrupted");
}
