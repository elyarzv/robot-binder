#include "ais_state_machine/AutonomyStates/StateAdjustToRail.h"

using namespace ais_state_machine;

StateAdjustToRail::StateAdjustToRail(shared_ptr<AbstractStateMachineManager> state_manager, ros::NodeHandle& nh) 
    : AbstractState(state_manager), AbstractROSState(nh)
{
    next_state_name_ = StateTypes::ADJUST_TO_RAIL;
    rail_head_ptr_ = nullptr;
    time_since_last_error_ = 0.0;

    // Based off of pixel of camera with coordinate frame origin at CENTER of image (1080 x 1920)
    target_pose_.position.x = 0.0;
    target_pose_.position.y = 215.0;
    target_pose_.orientation.w = 1.0;
    tol_x_ = 15.0;
    tol_y_ = 15.0;
    tol_angle_ = 10.0;
    local_error_x_ = 9999.9;
    local_error_y_ = 9999.9;
    rolling_window = 0;
    state_timeout_ = 20.0;

    cmd_speed_x_ = 0.0f;
    cmd_speed_y_ = 0.0f;
    cmd_angular_ = 0.0f;

}

StateAdjustToRail::~StateAdjustToRail() 
{

}

void StateAdjustToRail::runState()
{
    if (!rail_head_ptr_) {
        ROS_WARN("No Rail head pointer available!");
    }
    if (!rail_head_ptr_->isPoseValid())
    {
        if (time_since_last_error_ == 0.0f) 
        {
            ROS_WARN_THROTTLE(1, "AdjustToRail: No valid rail head pose received");
            time_since_last_error_ = ros::Time::now().toSec();
        } else 
        {
            ROS_WARN_THROTTLE(1, "AdjustToRail: No valid rail head pose received since %lf", time_since_last_error_);
        }
        state_manager_->vel_controller_ptr_->sendVelocityCmd(0,0,0);

    } else 
    {
        useLinearRailAdjustment();
        useAngularRailAdjustment();
        //ROS_INFO_THROTTLE(0.5, "State AdjustToRail: Output speed %f - %f - %f", cmd_speed_x_,cmd_speed_y_,cmd_angular_);
        state_manager_->vel_controller_ptr_->sendVelocityCmd(0,cmd_speed_y_,cmd_angular_);
    }
    if (state_manager_->mission_config_ptr_->use_uv_lamp){
        state_manager_->uv_lamp_controller_ptr_->setUVIntensity(state_manager_->mission_config_ptr_->rail_transition_intensity);
        state_manager_->uv_lamp_controller_ptr_->setUVLampOn(true);
        ros::spinOnce();
    }
    state_manager_->status_manager_ptr_->setRobotStatus(RobotStatus::AUTO);
    evaluateStateStatus();
}

bool StateAdjustToRail::isStateFinished()
{
    ROS_DEBUG("State StateAdjustToRail Status: %s", has_finished_state_ ? "true" : "false");
    return has_finished_state_;
}

StateTypes StateAdjustToRail::getNextStateName()
{
    return next_state_name_;
}

void StateAdjustToRail::evaluateStateStatus() 
{   
    // Timeout since robot is unable to adjust to a detectable rail
    if (ros::Time::now().toSec() - time_since_last_error_ > state_timeout_ && time_since_last_error_ != 0.0)
    {
        ROS_ERROR_THROTTLE(1, "AdjustToRail: Has been unable to adjust! No valid rail pose");
        next_state_name_ = StateTypes::RECOVERY;
    }else {
    // Evaluate next state if robot has detectable rail
        // Adjustments are completed
        if (fabs(local_error_y_) <= tol_y_) 
        {   
            ROS_INFO("Robot is within tolerance! X_error: %f Y_error: %f count: %f", local_error_x_, local_error_y_, rolling_window);
            if(rolling_window > WINDOW_SIZE){
                next_state_name_ = StateTypes::MOUNT_ROBOT_TO_RAIL;
                // robot has reached target rail. Update the rail number
                const auto old_rail_number = state_manager_->rail_status_ptr_->getCurrentRailNumber();
                const auto target_rail_number = state_manager_->rail_status_ptr_->getTargetRailNumber();
                state_manager_->rail_status_ptr_->setCurrentRailNumber(target_rail_number);
                ROS_INFO(
                    "State AdjustToRail: updated current rail number to %d from %d",
                    state_manager_->rail_status_ptr_->getCurrentRailNumber(), old_rail_number
                );

            }else{
                ++rolling_window;
                next_state_name_ = StateTypes::ADJUST_TO_RAIL; 
            }
        }
        // Adjustments not completed
        else if (state_manager_->status_manager_ptr_->willRobotCollide()){
            if(collision_flag_){
                ROS_ERROR("State AdjustToRail: Stopped by safety for %f seconds", ros::Time::now().toSec() - safety_stop_time_.toSec());
                if(ros::Time::now().toSec() - safety_stop_time_.toSec() > safety_timeout){
                    next_state_name_ = StateTypes::RECOVERY;
                }else{
                    next_state_name_ = StateTypes::ADJUST_TO_RAIL;
                }
            }else{
                collision_flag_ = true;
                safety_stop_time_ = ros::Time::now();
                next_state_name_ = StateTypes::ADJUST_TO_RAIL;
            }
            rolling_window = 0;
        }else if(state_manager_->bumper_handler_ptr_->isFrontBumperHit() || state_manager_->bumper_handler_ptr_->isBackBumperHit())
        {
            ROS_ERROR("StateAdjustToRail: Bumper hit while moving");
            state_manager_->status_manager_ptr_->setErrorCode("210020");
            next_state_name_ = StateTypes::HALTED;
        }else{
        rolling_window = 0;
        collision_flag_ = false;
        next_state_name_ = StateTypes::ADJUST_TO_RAIL;
        }
    }
}

void StateAdjustToRail::reset() 
{
    state_manager_->back_rail_manager_ptr_->resetData();
    state_manager_->front_rail_manager_ptr_->resetData();
    local_error_x_ = 9999.9;
    local_error_y_ = 9999.9;
    cmd_speed_x_ = 0;
    cmd_speed_y_ = 0;
    cmd_angular_ = 0;
    time_since_last_error_ = 0.0;
    geometry_msgs::Pose tmp;
    current_rail_pose_ = tmp;
    
    next_state_name_ = StateTypes::ADJUST_TO_RAIL;
}

void StateAdjustToRail::finish()
{
    ROS_INFO("StateAdjustToRail is now exiting");
    reset();
    state_manager_->rail_status_ptr_->publishOffRailErrors(false);
}

void StateAdjustToRail::init()
{
    ROS_INFO("StateAdjustToRail is initializing");

    reset();
    rolling_window = 0;
    if (state_manager_->mission_config_ptr_->is_forward_robot_rail_direction)
    {
        rail_head_ptr_ = state_manager_->front_rail_manager_ptr_;
    } else{
        rail_head_ptr_ = state_manager_->back_rail_manager_ptr_;
    }
    state_manager_->rail_status_ptr_->publishOffRailErrors(true);
}

void StateAdjustToRail::interruptState()
{
    ROS_INFO("StateAdjustToRail is interrupted");
    state_manager_->vel_controller_ptr_->sendVelocityCmd(0,0,0);
}

void StateAdjustToRail::useLinearRailAdjustment()
{   
    if (!rail_head_ptr_->isPoseValid())
    {
        ROS_WARN("Pose not Valid!");
        cmd_speed_x_ = 0;
        cmd_speed_y_ = 0;
        return;
    }
    current_rail_pose_ = rail_head_ptr_->getRailHead();
    // rail pose coordinate frame centered around origin (+x axis right, +y axis down)
    // error_x_ has negative sign to account for transformation to robot local frame
    local_error_x_ = current_rail_pose_.position.y - target_pose_.position.y;
    local_error_y_ = current_rail_pose_.position.x - target_pose_.position.x;

    local_error_x_ *= -1.0;
    local_error_y_ *= -1.0;
    ROS_INFO_THROTTLE(0.5,"Rail head at x:%f y:%f, local_error_x:%f, local_error_y:%f", current_rail_pose_.position.x, current_rail_pose_.position.y,local_error_x_,local_error_y_);

    time_since_last_error_ = 0.0f;     
    // (1080 x 1920) (540 x 960) (y,x) in local frame

    if (fabs(local_error_y_) > 100) 
    {
        cmd_speed_y_ = 0.15;
    }
    else 
    {
        if(fabs(local_error_y_) < tol_y_){
            cmd_speed_y_ = 0.0;
        }else{
            cmd_speed_y_ = 0.05;
        }
    }

    if (local_error_y_ < 0)
    {
        cmd_speed_y_ *= -1.0f;
    }
}

void StateAdjustToRail::useAngularRailAdjustment()
{
    //TODO (Implement IMU follower later)
    cmd_angular_ = 0.0f;
    
}
