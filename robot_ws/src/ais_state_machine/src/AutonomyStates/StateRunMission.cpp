#include "ais_state_machine/AutonomyStates/StateRunMission.h"

using namespace ais_state_machine;

StateRunMission::StateRunMission(shared_ptr<AbstractStateMachineManager> state_manager, ros::NodeHandle& nh) 
    : AbstractState(state_manager), AbstractROSState(nh)
{
	ros::NodeHandle private_nh_("~");
    
    std::string localization_source, pose_topic, reset_topic;
    //This should be the namespace of amr_localization amr_localization_1d_node
    private_nh_.param<std::string>("localization_source", localization_source, std::string("wheel_localization_node"));
    pose_topic.assign(localization_source);
    pose_topic.append("/pose_1_d");
    reset_topic.assign(localization_source);
    reset_topic.append("/reset");

    pose_subscriber_ = nh.subscribe(pose_topic, 1, &StateRunMission::poseCb, this);

    reset_pose_service_ = nh.serviceClient<std_srvs::Trigger>(reset_topic);
    send_intensity_client_ = nh.serviceClient<phoenix_msgs::SetLampStatus>("set_dimmer_values");
    lamp_power_ = nh.advertise<std_msgs::UInt8>("embedded/power_inverter/switch", 1000);
    cmd_vel_publisher_ = nh.advertise<geometry_msgs::Twist>("embedded/cmd_vel", 1000);
    led_publisher_ = nh.advertise<std_msgs::UInt8>("embedded/notification_lamp/colour", 1000);
    next_state_name_ = StateTypes::RUN_MISSION;
    //TODO: add wiat for dimmer service
    //set the lamp_vector_ size
    int lamp_number;
    nh.param<int>("max_lamp_number", lamp_number, 6);
    for(int ctr = 0; ctr < lamp_number; ++ctr){
        lamp_vector_.data.push_back(0);
    }
}

StateRunMission::~StateRunMission() 
{

}

void StateRunMission::runState()
{
    //run state
    ROS_DEBUG("Running StateRunMission");
    state_manager_->status_manager_ptr_->setRobotStatus(RobotStatus::AUTO);
    evaluateStateStatus();
}
void StateRunMission::interruptState()
{
    //If this state is interupted, go to the complete state
    ROS_INFO("Aborting StateRunMission");
    //Set 0 velocity
    geometry_msgs::Twist cmd_vel;
    cmd_vel_publisher_.publish(cmd_vel);
    ros::spinOnce();
    //Turn off lights
}

bool StateRunMission::isStateFinished()
{
    ROS_DEBUG("State StateRunMission Status: %s", has_finished_state_ ? "true" : "false");
    return has_finished_state_;
}

StateTypes StateRunMission::getNextStateName()
{
    return next_state_name_;
}

void StateRunMission::evaluateStateStatus(){
    ROS_INFO("Pose: %f Goal: %f", current_pose_.data, state_manager_->mission_goal_manager_->getCurrentGoal().position);
    //Send the current goal's speed to the motors
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = state_manager_->mission_goal_manager_->getGoalSpeed();
    cmd_vel_publisher_.publish(cmd_vel);
    ros::spinOnce();
    //Set intensity (or led colour in this case)
    //std_msgs::UInt8 led;
    //led.data = (int)state_manager_->mission_goal_manager_->getIntensity();
    //led_publisher_.publish(led);
    //Check if we've reached the goal
    if ((current_pose_.data > state_manager_->mission_goal_manager_->getCurrentGoal().position && cmd_vel.linear.x > 0) 
        || (current_pose_.data < state_manager_->mission_goal_manager_->getCurrentGoal().position && cmd_vel.linear.x < 0)) {
        //Check if there are more goals
        if(state_manager_->mission_goal_manager_->incrementGoalCounter()){
            int goal_intensity = (int)state_manager_->mission_goal_manager_->getAvgGoalIntensity();
            if(goal_intensity >= 0 && goal_intensity <= 100){
                ROS_INFO("Setting intensity to %d", goal_intensity);
                sendIntensity((int)state_manager_->mission_goal_manager_->getAvgGoalIntensity());
                uvOn();
                ros::spinOnce();
            }else{
                uvOff();
            }
            next_state_name_ = StateTypes::RUN_MISSION;
        }else{
            //Mission is complete
            has_finished_state_ = true;
            cmd_vel.linear.x = 0;
            cmd_vel_publisher_.publish(cmd_vel);
            ros::spinOnce();
            uvOff();
            state_manager_->mission_goal_manager_->reset();
            next_state_name_ = StateTypes::COMPLETE;
        }
    } else {
        next_state_name_ = StateTypes::RUN_MISSION;
    }
}
void StateRunMission::reset()
{
    std_srvs::Trigger trigger;
    reset_pose_service_.call(trigger);
    uvOff();
    next_state_name_ = StateTypes::RUN_MISSION;
}
void StateRunMission::finish()
{
    ROS_INFO("StateRunMission is now exiting");
    //Set 0 velocity
    geometry_msgs::Twist cmd_vel;
    cmd_vel_publisher_.publish(cmd_vel);
    uvOff();
    ros::spinOnce();
}
void StateRunMission::init()
{
    //Reset the localization node
    std_srvs::Trigger trigger;
    reset_pose_service_.call(trigger);
    ROS_INFO("StateRunMission is initializing");

}


void StateRunMission::poseCb(const std_msgs::Float32ConstPtr &pose){
    current_pose_ = *pose;
}

void StateRunMission::sendIntensity(int val){
    for(int ctr = 0; ctr < lamp_vector_.data.size(); ++ctr){
        lamp_vector_.data[ctr] = val;
    }
    phoenix_msgs::SetLampStatus req;
    req.request.lamp_values = lamp_vector_.data;
    send_intensity_client_.call(req);

}
void StateRunMission::uvOn(){
    std_msgs::UInt8 uv_bulb;
    uv_bulb.data = 1;
    lamp_power_.publish(uv_bulb);
    ros::spinOnce();
}
void StateRunMission::uvOff(){
    std_msgs::UInt8 uv_bulb;
    uv_bulb.data = 0;
    lamp_power_.publish(uv_bulb);
    ros::spinOnce();
}
