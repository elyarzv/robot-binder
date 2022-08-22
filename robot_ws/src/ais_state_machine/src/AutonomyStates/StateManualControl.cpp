#include "ais_state_machine/AutonomyStates/StateManualControl.h"

using namespace ais_state_machine;

StateManualControl::StateManualControl(shared_ptr<AbstractStateMachineManager> state_manager, ros::NodeHandle& nh) 
    : AbstractState(state_manager), AbstractROSState(nh)
{
    next_state_name_ = StateTypes::MANUAL_CONTROL;
    cmd_vel_publisher_ = nh.advertise<geometry_msgs::Twist>("embedded/cmd_vel", 1000);

}

StateManualControl::~StateManualControl() 
{

}

void StateManualControl::runState()
{
    ROS_INFO("State ManualControl: count %d, Next State is %d", count, (int) next_state_name_);
    cmd_vel_publisher_.publish(state_manager_->status_manager_ptr_->getManualCmdVel());
    ros::spinOnce();
    state_manager_->status_manager_ptr_->setRobotStatus(RobotStatus::MANUAL_OPERATION);
    evaluateStateStatus();
}

bool StateManualControl::isStateFinished()
{
    ROS_INFO("State ManualControl Status: %s", has_finished_state_ ? "true" : "false");
    return has_finished_state_;
}

StateTypes StateManualControl::getNextStateName()
{
    return next_state_name_;
}

void StateManualControl::evaluateStateStatus() 
{
}

void StateManualControl::reset() 
{
    next_state_name_ = StateTypes::MANUAL_CONTROL;
}

void StateManualControl::finish()
{
    ROS_INFO("StateManualControl is now exiting");
    reset();
}

void StateManualControl::init()
{
    ROS_INFO("StateManualControl is initializing");
    reset();
}

void StateManualControl::interruptState()
{
    ROS_INFO("StateManualControl is interrupted");
}