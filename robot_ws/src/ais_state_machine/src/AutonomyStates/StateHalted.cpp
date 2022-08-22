#include "ais_state_machine/AutonomyStates/StateHalted.h"

using namespace ais_state_machine;

StateHalted::StateHalted(shared_ptr<AbstractStateMachineManager> state_manager, ros::NodeHandle& nh) 
    : AbstractState(state_manager), AbstractROSState(nh)
{
    next_state_name_ = StateTypes::HALTED;
    cmd_vel_publisher_ = nh.advertise<geometry_msgs::Twist>("embedded/cmd_vel", 1000);

}

StateHalted::~StateHalted() 
{

}

void StateHalted::runState()
{
    ROS_INFO_THROTTLE(1,"State Halted");

    geometry_msgs::Twist cmd_vel;
    cmd_vel_publisher_.publish(cmd_vel);

    state_manager_->uv_lamp_controller_ptr_->setUVLampOn(false);
    ros::spinOnce();
    //Switch mux here

    state_manager_->status_manager_ptr_->setRobotStatus(RobotStatus::HALTED);
    
    evaluateStateStatus();
}

bool StateHalted::isStateFinished()
{
    ROS_INFO("State Halted Status: %s", has_finished_state_ ? "true" : "false");
    return has_finished_state_;
}

StateTypes StateHalted::getNextStateName()
{
    return next_state_name_;
}

void StateHalted::evaluateStateStatus() 
{
}

void StateHalted::reset() 
{
    next_state_name_ = StateTypes::HALTED;
}

void StateHalted::finish()
{
    ROS_INFO("StateHalted is now exiting");
    reset();
}

void StateHalted::init()
{
    ROS_INFO("StateHalted is initializing");
    reset();
    geometry_msgs::Twist cmd_vel;
    cmd_vel_publisher_.publish(cmd_vel);
    state_manager_->uv_lamp_controller_ptr_->setUVLampOn(false);
    ros::spinOnce();
    state_manager_->status_manager_ptr_->setRobotStatus(RobotStatus::HALTED);
}

void StateHalted::interruptState()
{
    ROS_INFO("StateHalted is interrupted");
}