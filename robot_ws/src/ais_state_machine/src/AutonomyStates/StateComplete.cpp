#include "ais_state_machine/AutonomyStates/StateComplete.h"

using namespace ais_state_machine;

StateComplete::StateComplete(shared_ptr<AbstractStateMachineManager> state_manager, 
                            ros::NodeHandle& nh) 
    : AbstractState(state_manager), AbstractROSState(nh)
{
    state_manager_->mission_goal_manager_->reset();
    next_state_name_ = StateTypes::COMPLETE;
    count = 0;
    rosbag_publisher_ = nh.advertise<std_msgs::UInt8>("/record/stop", 1000);
    cmd_vel_publisher_ = nh.advertise<geometry_msgs::Twist>("embedded/cmd_vel", 1000);
}

StateComplete::~StateComplete() 
{

}

void StateComplete::runState()
{
    count++;
    ROS_INFO("State Complete: count %d, Next State is %d", count, (int) next_state_name_);

    if (state_manager_->mission_config_ptr_->use_uv_lamp) {
        state_manager_->uv_lamp_controller_ptr_->setUVLampOn(false);
        ros::spinOnce();
    }

    geometry_msgs::Twist cmd_vel;
    cmd_vel_publisher_.publish(cmd_vel);
    ros::spinOnce();
    state_manager_->status_manager_ptr_->setRobotStatus(RobotStatus::COMPLETE);
    evaluateStateStatus();
}

bool StateComplete::isStateFinished()
{
    ROS_INFO("State Complete Status: %s", has_finished_state_ ? "true" : "false");
    return has_finished_state_;
}

StateTypes StateComplete::getNextStateName()
{
    return next_state_name_;
}

void StateComplete::evaluateStateStatus() 
{   

    if (count >= 10) {
        next_state_name_ = StateTypes::IDLE;
    } else {
        next_state_name_ = StateTypes::COMPLETE;
    }
}

void StateComplete::reset()
{    
    count = 0;
    next_state_name_ = StateTypes::COMPLETE;
}
void StateComplete::finish()
{
    ROS_INFO("StateComplete is now exiting");
    nh_.setParam("RAIL_MISSION_COUNTER", 0);
    state_manager_->rail_status_ptr_->resetMissionStatus();
    reset();
}
void StateComplete::init()
{
    ROS_INFO("StateComplete is initializing");
    std_msgs::UInt8 trigger;
    rosbag_publisher_.publish(trigger);
    ros::spinOnce();
    reset();
    state_manager_->status_manager_ptr_->setRobotStatus(RobotStatus::COMPLETE);
}

void StateComplete::interruptState()
{
    ROS_INFO("StateComplete is interrupted");
}