#include "ais_state_machine/AutonomyStates/StateTeleoperation.h"

using namespace ais_state_machine;

StateTeleoperation::StateTeleoperation(shared_ptr<AbstractStateMachineManager> state_manager, ros::NodeHandle& nh) 
    : AbstractState(state_manager), AbstractROSState(nh)
{
    next_state_name_ = StateTypes::TELEOPERATION;
    cmd_vel_publisher_ = nh.advertise<geometry_msgs::Twist>("embedded/cmd_vel", 1000);

}

StateTeleoperation::~StateTeleoperation() 
{

}

void StateTeleoperation::runState()
{
    ROS_INFO("State Teleoperation: count %d, Next State is %d", count, (int) next_state_name_);
    state_manager_->status_manager_ptr_->setRobotStatus(RobotStatus::TELEOP);
    cmd_vel_publisher_.publish(state_manager_->status_manager_ptr_->getTeleopCmdVel());
    state_manager_->uv_lamp_controller_ptr_->setUVLampOn(false);
    ros::spinOnce();

    evaluateStateStatus();
}

bool StateTeleoperation::isStateFinished()
{
    ROS_INFO("State Teleoperation Status: %s", has_finished_state_ ? "true" : "false");
    return has_finished_state_;
}

StateTypes StateTeleoperation::getNextStateName()
{
    return next_state_name_;
}

void StateTeleoperation::evaluateStateStatus() 
{
}

void StateTeleoperation::reset() 
{
    next_state_name_ = StateTypes::TELEOPERATION;
}

void StateTeleoperation::finish()
{
    ROS_INFO("StateTeleoperation is now exiting");
    reset();
    // assume user has moved the robot to the next rail
    const auto old_rail_number = state_manager_->rail_status_ptr_->getCurrentRailNumber();
    const auto target_rail_number = state_manager_->rail_status_ptr_->getTargetRailNumber();
    state_manager_->rail_status_ptr_->setCurrentRailNumber(target_rail_number);
    ROS_INFO(
        "State Teleoperation: updated current rail number to %d from %d",
        state_manager_->rail_status_ptr_->getCurrentRailNumber(), old_rail_number
    );
}

void StateTeleoperation::init()
{
    ROS_INFO("StateTeleoperation is initializing");
    reset();
}

void StateTeleoperation::interruptState()
{
    ROS_INFO("StateTeleoperation is interrupted");
}
