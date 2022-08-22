#include "ais_state_machine/AutonomyStates/StateIdle.h"

using namespace ais_state_machine;

StateIdle::StateIdle(shared_ptr<AbstractStateMachineManager> state_manager, ros::NodeHandle& nh) 
    : AbstractState(state_manager), AbstractROSState(nh)
{
    next_state_name_ = StateTypes::IDLE;
    cmd_vel_publisher_ = nh.advertise<geometry_msgs::Twist>("embedded/cmd_vel", 1000);
    start_mission_service_ = nh.advertiseService("start_mission", &StateIdle::startMissionCb, this);
    start_named_mission_service_ = nh.advertiseService("start_named_mission", &StateIdle::startNamedMissionCb, this);

}

StateIdle::~StateIdle() 
{

}

bool StateIdle::startMissionCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
    next_state_name_ = StateTypes::INITIALIZE_ROBOT;
    state_manager_->mission_goal_manager_->reset();
    res.success = true;
    return true;
}

bool StateIdle::startNamedMissionCb(phoenix_msgs::startMission::Request &req, phoenix_msgs::startMission::Response &res){
    
    next_state_name_ = StateTypes::INITIALIZE_ROBOT;
    state_manager_->mission_goal_manager_->reset();
    std::stringstream string_stream;
    string_stream.str("");
    string_stream << req.mission << ".csv";
    nh_.setParam("MISSION_NAME", string_stream.str());
    res.success = true;
    return true;
}

void StateIdle::runState()
{
    ROS_INFO_THROTTLE(1,"State Idle");
    geometry_msgs::Twist cmd_vel;
    cmd_vel_publisher_.publish(cmd_vel);
    state_manager_->uv_lamp_controller_ptr_->setUVLampOn(false);
    ros::spinOnce();
    
    state_manager_->status_manager_ptr_->setRobotStatus(RobotStatus::IDLE);
    evaluateStateStatus();
}

bool StateIdle::isStateFinished()
{
    ROS_INFO("State Idle Status: %s", has_finished_state_ ? "true" : "false");
    return has_finished_state_;
}

StateTypes StateIdle::getNextStateName()
{
    return next_state_name_;
}

void StateIdle::evaluateStateStatus() 
{
}

void StateIdle::reset() 
{
    count = 0;
    next_state_name_ = StateTypes::IDLE;
}

void StateIdle::finish()
{
    ROS_INFO("StateIdle is now exiting");
    reset();
}

void StateIdle::init()
{
    ROS_INFO("StateIdle is initializing");
    reset();
    state_manager_->status_manager_ptr_->setRobotStatus(RobotStatus::IDLE);
}

void StateIdle::interruptState()
{
    ROS_INFO("StateIdle is interrupted");
}
