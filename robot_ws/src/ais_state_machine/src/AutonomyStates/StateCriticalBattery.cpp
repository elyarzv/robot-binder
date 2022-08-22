#include "ais_state_machine/AutonomyStates/StateCriticalBattery.h"

using namespace ais_state_machine;

StateCriticalBattery::StateCriticalBattery(shared_ptr<AbstractStateMachineManager> state_manager, ros::NodeHandle& nh) 
    : AbstractState(state_manager), AbstractROSState(nh)
{
    next_state_name_ = StateTypes::CRITICAL_BATTERY;

}

StateCriticalBattery::~StateCriticalBattery() 
{

}

void StateCriticalBattery::runState()
{
    ROS_INFO_THROTTLE(1,"State Critical Battery");
    state_manager_->uv_lamp_controller_ptr_->setUVLampOn(false);
    state_manager_->vel_controller_ptr_->sendVelocityCmd(0, 0, 0); 
    ros::spinOnce();
    state_manager_->status_manager_ptr_->setRobotStatus(RobotStatus::CRITICAL_BATTERY);
    evaluateStateStatus();
}

bool StateCriticalBattery::isStateFinished()
{
    ROS_INFO("State Critical Battery Status: %s", has_finished_state_ ? "true" : "false");
    return has_finished_state_;
}

StateTypes StateCriticalBattery::getNextStateName()
{
    return next_state_name_;
}

void StateCriticalBattery::evaluateStateStatus() 
{
    //Check battery levels here
}

void StateCriticalBattery::reset() 
{
    next_state_name_ = StateTypes::CRITICAL_BATTERY;
}

void StateCriticalBattery::finish()
{
    ROS_INFO("StateCriticalBattery is now exiting");
    reset();
}

void StateCriticalBattery::init()
{
    ROS_INFO("StateCriticalBattery is initializing");
    reset();
}

void StateCriticalBattery::interruptState()
{
    ROS_INFO("StateCriticalBattery is interrupted");
}
