#include "ais_state_machine/AutonomyStates/StateReturnToRailHome.h"

using namespace ais_state_machine;

StateReturnToRailHome::StateReturnToRailHome(shared_ptr<AbstractStateMachineManager> state_manager, 
                                            ros::NodeHandle& nh) 
    : AbstractState(state_manager), AbstractROSState(nh)
{
    next_state_name_ = StateTypes::RETURN_TO_RAIL_HOME;

    ros::NodeHandle private_nh_("~");
    std::string localization_source, pose_topic;
    //This should be the namespace of amr_localization amr_localization_1d_node
    private_nh_.param<std::string>("localization_source", localization_source, std::string("wheel_localization_node"));
    pose_topic = localization_source + "/pose_1_d";
    pose_subscriber_ = nh.subscribe(pose_topic, 1, &StateReturnToRailHome::poseCb, this);
    cmd_vel_publisher_ = nh.advertise<geometry_msgs::Twist>("embedded/cmd_vel", 1000);

}

StateReturnToRailHome::~StateReturnToRailHome() 
{

}


void StateReturnToRailHome::runState()
{
    float speed = state_manager_->mission_config_ptr_->return_to_rail_home_speed;
    if (state_manager_->mission_config_ptr_->is_forward_robot_rail_direction){
        speed *= -1.0;
    }
    if(state_manager_->mission_config_ptr_->irradiance_speed_control){
        if(state_manager_->mission_config_ptr_->return_to_rail_home_with_uv_on){
            if(state_manager_->bumper_handler_ptr_->isFrontBumperHit() && state_manager_->mission_config_ptr_->is_forward_robot_rail_direction){
                ROS_WARN("Attempting to unbump front bumper");
            }
            else if(state_manager_->bumper_handler_ptr_->isBackBumperHit() && !state_manager_->mission_config_ptr_->is_forward_robot_rail_direction){
                ROS_WARN("Attempting to unbump back bumper");
            }else{
                auto avg_disinfection_intensity = state_manager_->mission_goal_manager_->getAvgGoalIntensity();
                auto avg_return_home_intensity = std::reduce(
                    state_manager_->mission_config_ptr_->return_home_intensity.begin(),
                    state_manager_->mission_config_ptr_->return_home_intensity.end()
                ) / state_manager_->mission_config_ptr_->return_home_intensity.size();

                ROS_INFO_THROTTLE(
                    0.5, "Disinfection intensity: %f, Return home intensity: %f, Current: %f",
                    avg_disinfection_intensity, avg_return_home_intensity,
                    state_manager_->uv_lamp_controller_ptr_->getCurrentIrradiancePercent()
                );

                double multiplier
                    = (avg_return_home_intensity >= avg_disinfection_intensity)
                        ? state_manager_->uv_lamp_controller_ptr_->getCurrentIrradiancePercent()
                            / state_manager_->mission_goal_manager_->getAvgGoalIntensity()
                        : 1.0;

                speed *= multiplier;
                ROS_INFO_THROTTLE(0.5, "Speed-UV Multiplier value is %lf, %lf", multiplier, speed);
            }
        }
    }
    ROS_INFO_THROTTLE(0.5,"StateReturnToRailHome: Position %f Speed %f", curr_robot_rail_pose_.data, speed);
    //TODO: confirm UV lights are still off if use_uv_lamp is true and return_to_rail_home_with_uv_on is false
    state_manager_->vel_controller_ptr_->sendVelocityCmd(speed, 0, 0);
    state_manager_->status_manager_ptr_->setRobotStatus(RobotStatus::RETURN_TO_RAIL_HOME);
    if(state_manager_->mission_config_ptr_->return_to_rail_home_with_uv_on){
        if(state_manager_->mission_config_ptr_->return_home_intensity.size() == 8){ //TODO: Use ros param instead
            ROS_INFO("Setting lamps to:");
            for(int i = 0; i < state_manager_->mission_config_ptr_->return_home_intensity.size(); ++i ){
                ROS_INFO("%f", state_manager_->mission_config_ptr_->return_home_intensity[i]);
            }
            state_manager_->uv_lamp_controller_ptr_->setUVIntensity(state_manager_->mission_config_ptr_->return_home_intensity);
            state_manager_->uv_lamp_controller_ptr_->setUVLampOn(true);
            ros::spinOnce();
        }else{
            ROS_ERROR("Lamp intensity size wrong");
        }
    }
    evaluateStateStatus();
}

bool StateReturnToRailHome::isStateFinished()
{
    ROS_INFO("StateReturnToRailHome Status: %s", has_finished_state_ ? "true" : "false");
    return has_finished_state_;
}

StateTypes StateReturnToRailHome::getNextStateName()
{
    return next_state_name_;
}

void StateReturnToRailHome::evaluateStateStatus() 
{

    if (state_manager_->mission_config_ptr_->is_forward_robot_rail_direction && 
            state_manager_->bumper_handler_ptr_->isBackBumperHit() || 
        !state_manager_->mission_config_ptr_->is_forward_robot_rail_direction && 
            state_manager_->bumper_handler_ptr_->isFrontBumperHit())
    {
        ROS_WARN("Robot hit something when returning home, Stopping and completing task early!");
        next_state_name_ = StateTypes::COMPLETE;
    } 
    // When robot has reached back to initial position according to odometry or has gone off the rail
    else if (
        (state_manager_->mission_config_ptr_->is_forward_robot_rail_direction && !isRobotPosAfterHomePos()) ||
        (!state_manager_->mission_config_ptr_->is_forward_robot_rail_direction && isRobotPosAfterHomePos())
    )
    {
        if(state_manager_->status_manager_ptr_->lowRobotBattery()){
            ROS_WARN("Battery low, transitioning to low state");
            next_state_name_ = StateTypes::LOW_BATTERY;
        }
        else{

            ROS_INFO(
                "State ReturnToRailHome: %d/%d rails completed in current sub job, "
                "%d/%ld jobs completed",
                state_manager_->rail_status_ptr_->getCurrentSubJobIdx(),
                state_manager_->rail_status_ptr_->getTotalSubJobs(),
                state_manager_->rail_status_ptr_->getCurrentJobIdx(),
                state_manager_->mission_config_ptr_->rail_jobs.size()
            );

            if (state_manager_->rail_status_ptr_->isMissionComplete()) {
                ROS_INFO("Returned to home, not showing multiple rails");
                next_state_name_ = StateTypes::COMPLETE;
            }else{
                ROS_INFO("Returned to home, dismounting rails");
                next_state_name_ = StateTypes::MOVE_ROBOT_OFF_RAIL;
            }
        }
    } 
    // If robot is still moving to initial rail position
    else if (state_manager_->mission_config_ptr_->is_forward_robot_rail_direction && isRobotPosAfterHomePos() ||
        !state_manager_->mission_config_ptr_->is_forward_robot_rail_direction && !isRobotPosAfterHomePos())
    {
        ROS_DEBUG_THROTTLE(1, "Still returning to home");
        next_state_name_ = StateTypes::RETURN_TO_RAIL_HOME;
    }
    else
    {
        ROS_WARN("Don't know where to go :(");
    }
}

void StateReturnToRailHome::reset() 
{
    next_state_name_ = StateTypes::RETURN_TO_RAIL_HOME;
}

void StateReturnToRailHome::finish()
{
    ROS_INFO("StateReturnToRailHome is now exiting");
    state_manager_->vel_controller_ptr_->sendVelocityCmd(0, 0, 0);
    reset();
    if (state_manager_->mission_config_ptr_->return_to_rail_home_with_uv_on && 
            !(state_manager_->rail_status_ptr_->getTotalSubJobs() > 1 || state_manager_->mission_config_ptr_->rail_jobs.size() > 1)) {
        ROS_INFO("Turning return home lamps off");
        state_manager_->uv_lamp_controller_ptr_->setUVIntensity(0);
        state_manager_->uv_lamp_controller_ptr_->setUVLampOn(false);
        ros::spinOnce();
    }
    state_manager_->rail_status_ptr_->publishOnRailErrors(false);
}

void StateReturnToRailHome::init()
{
    ROS_INFO("StateReturnToRailHome is initializing");
    reset();
    if(state_manager_->mission_config_ptr_->return_to_rail_home_with_uv_on){
        if(state_manager_->mission_config_ptr_->return_home_intensity.size() == 8){ //TODO: Use ros param instead
            ROS_INFO("Setting lamps to:");
            for(int i = 0; i < state_manager_->mission_config_ptr_->return_home_intensity.size(); ++i ){
                ROS_INFO("%f", state_manager_->mission_config_ptr_->return_home_intensity[i]);
            }
            state_manager_->uv_lamp_controller_ptr_->setUVIntensity(state_manager_->mission_config_ptr_->return_home_intensity);
            state_manager_->uv_lamp_controller_ptr_->setUVLampOn(true);
            ros::spinOnce();
        }else{
            ROS_ERROR("Lamp intensity size wrong");
        }
    }
    else if (state_manager_->mission_config_ptr_->use_uv_lamp) {
        ROS_INFO("Turning lamps off");
        state_manager_->uv_lamp_controller_ptr_->setUVIntensity(0);
        state_manager_->uv_lamp_controller_ptr_->setUVLampOn(false);
        ros::spinOnce();
    }
    state_manager_->status_manager_ptr_->setRobotOnRails(false);
    state_manager_->status_manager_ptr_->setRobotStatus(RobotStatus::RETURN_TO_RAIL_HOME);
    state_manager_->rail_status_ptr_->publishOnRailErrors(true);
}

void StateReturnToRailHome::interruptState()
{
    ROS_INFO("StateReturnToRailHome is interrupted");
    //Set 0 velocity
    state_manager_->vel_controller_ptr_->sendVelocityCmd(0, 0, 0);
}

inline bool StateReturnToRailHome::isRobotPosAfterHomePos()
{   
    return (curr_robot_rail_pose_.data >= 0.0);
}

void StateReturnToRailHome::poseCb(const std_msgs::Float32ConstPtr &pose)
{
    curr_robot_rail_pose_ = *pose;
}
