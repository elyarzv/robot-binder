#include "ais_state_machine/AutonomyStates/StateManualTreatment.h"

using namespace ais_state_machine;

StateManualTreatment::StateManualTreatment(shared_ptr<AbstractStateMachineManager> state_manager, ros::NodeHandle& nh) 
    : AbstractState(state_manager), AbstractROSState(nh)
{
    next_state_name_ = StateTypes::MANUAL_TREATMENT;
    cmd_vel_publisher_ = nh.advertise<geometry_msgs::Twist>("embedded/cmd_vel", 1000);

}

StateManualTreatment::~StateManualTreatment() 
{

}

void StateManualTreatment::runState()
{
    ROS_INFO("State ManualTreatment: count %d, Next State is %d", count, (int) next_state_name_);
    state_manager_->status_manager_ptr_->setRobotStatus(RobotStatus::MANUAL_TREATMENT);

    cmd_vel_publisher_.publish(state_manager_->status_manager_ptr_->getRollerCmdVel());
    ros::spinOnce();
    evaluateStateStatus();
}

bool StateManualTreatment::isStateFinished()
{
    ROS_INFO("State ManualTreatment Status: %s", has_finished_state_ ? "true" : "false");
    return has_finished_state_;
}

StateTypes StateManualTreatment::getNextStateName()
{
    return next_state_name_;
}

void StateManualTreatment::evaluateStateStatus() 
{
}

void StateManualTreatment::reset() 
{
    next_state_name_ = StateTypes::MANUAL_TREATMENT;
}

void StateManualTreatment::finish()
{
    ROS_INFO("StateManualTreatment is now exiting");
    reset();
    state_manager_->rail_status_ptr_->publishOnRailErrors(false);
}

void StateManualTreatment::init()
{
    ROS_INFO("StateManualTreatment is initializing");
    state_manager_->status_manager_ptr_->getUpdatedParams();
    reset();
    state_manager_->rail_status_ptr_->publishOnRailErrors(true);
}

void StateManualTreatment::interruptState()
{
    ROS_INFO("StateManualTreatment is interrupted");
}