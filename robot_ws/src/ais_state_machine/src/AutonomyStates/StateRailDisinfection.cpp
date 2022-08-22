#include "ais_state_machine/AutonomyStates/StateRailDisinfection.h"

using namespace ais_state_machine;

StateRailDisinfection::StateRailDisinfection(shared_ptr<AbstractStateMachineManager> state_manager, ros::NodeHandle &nh) : AbstractState(state_manager), AbstractROSState(nh) {
    next_state_name_ = StateTypes::RAIL_DISINFECTION;
    cmd_vel_publisher_ = nh.advertise<geometry_msgs::Twist>("embedded/cmd_vel", 1000);

    is_waiting_at_goal_ = false;
}

StateRailDisinfection::~StateRailDisinfection() {}

void StateRailDisinfection::runState() {
    ROS_INFO_THROTTLE(0.5, "RailDisinfection: Position %f Goal %f Speed %f", curr_robot_rail_pose_.data, state_manager_->mission_goal_manager_->getCurrentGoal().position,
                                        state_manager_->mission_goal_manager_->getGoalSpeed());
    if (!is_waiting_at_goal_) {
        float goal_velocity = state_manager_->mission_goal_manager_->getGoalSpeed();
        if (!state_manager_->mission_config_ptr_->is_forward_robot_rail_direction) {
            goal_velocity *= -1.0;
        }

        if (state_manager_->mission_config_ptr_->irradiance_speed_control) {
            if (state_manager_->mission_config_ptr_->use_uv_lamp) {
                double multiplier = (state_manager_->uv_lamp_controller_ptr_->getCurrentIrradiancePercent()) / state_manager_->mission_goal_manager_->getAvgGoalIntensity();
                goal_velocity *= multiplier;
                ROS_INFO_THROTTLE(0.5, "Speed-UV Multiplier value is %lf, %lf", multiplier, goal_velocity);
                ROS_INFO_THROTTLE(
                    0.5, "current and goal intensity %lf, %lf:",
                    state_manager_->uv_lamp_controller_ptr_->getCurrentIrradiancePercent(),
                    state_manager_->mission_goal_manager_->getAvgGoalIntensity()
                );
            }
        }
        if (!(state_manager_->bumper_handler_ptr_->isFrontBumperHit() && state_manager_->mission_config_ptr_->is_forward_robot_rail_direction) &&
                !(state_manager_->bumper_handler_ptr_->isBackBumperHit() && !state_manager_->mission_config_ptr_->is_forward_robot_rail_direction)) {
            // Only send cmd_vel if the bumpers in the direction of travel are not
            // pressed
            state_manager_->vel_controller_ptr_->sendVelocityCmd(goal_velocity, 0, 0);
            last_time_in_motion_ = ros::Time::now();

            // Set speed and intenisty for current goal
            if (state_manager_->mission_config_ptr_->use_uv_lamp) {
                std::vector<float> goal_intensity = state_manager_->mission_goal_manager_->getGoalIntensity();
                state_manager_->uv_lamp_controller_ptr_->setUVIntensity(goal_intensity);
                state_manager_->uv_lamp_controller_ptr_->setUVLampOn(true);
                ros::spinOnce();
            }
        } else {
            state_manager_->vel_controller_ptr_->sendVelocityCmd(0, 0, 0);
        }

        if (isRobotPosAfterGoalPos() && state_manager_->mission_config_ptr_->is_forward_robot_rail_direction ||
                isReversedRobotPosAfterGoalPos() && !state_manager_->mission_config_ptr_->is_forward_robot_rail_direction) {
            if (state_manager_->mission_config_ptr_->use_stop_as_uv) {
                is_waiting_at_goal_ = true;
                state_manager_->status_manager_ptr_->setRobotStatus(RobotStatus::AUTO);
                state_manager_->vel_controller_ptr_->sendVelocityCmd(0, 0, 0);
                start_pause_time_ = ros::Time::now().toSec();
                end_pause_time_ = ros::Time::now().toSec() + state_manager_->mission_config_ptr_->robot_goal_stop_time;
            } else {
                state_manager_->mission_goal_manager_->incrementGoalCounter();
                ros::spinOnce();
            }
        }
    } else {
        state_manager_->vel_controller_ptr_->sendVelocityCmd(0, 0, 0);
        if (ros::Time::now().toSec() > end_pause_time_) {
            is_waiting_at_goal_ = false;
            state_manager_->status_manager_ptr_->setRobotStatus(RobotStatus::AUTO);
            state_manager_->mission_goal_manager_->incrementGoalCounter();
        }
    }
    state_manager_->status_manager_ptr_->setRobotStatus(RobotStatus::AUTO);

    evaluateStateStatus();
}

void StateRailDisinfection::evaluateStateStatus() {
    if (doesRobotNeedToTransition()) {
        // if bumper was triggered

        // if visual odometry says we are there

        // if wheel odom and visual odometry says we are there

        // if wheel odom says we are there
        if (ros::Time::now().toSec() - last_time_in_motion_.toSec() > RECOVERY_TIME) {
            state_manager_->status_manager_ptr_->setErrorCode("210019");
            next_state_name_ = StateTypes::HALTED;
        } else if (state_manager_->mission_config_ptr_->need_return_init_rail_pos) {
            next_state_name_ = StateTypes::RETURN_TO_RAIL_HOME;
        } else {
            next_state_name_ = StateTypes::COMPLETE;
        }
    }
}

bool StateRailDisinfection::isStateFinished() {
    ROS_INFO("State RailDisinfection Status: %s", has_finished_state_ ? "true" : "false");
    return has_finished_state_;
}

StateTypes StateRailDisinfection::getNextStateName() { return next_state_name_; }

void StateRailDisinfection::reset() {
    count = 0;
    next_state_name_ = StateTypes::RAIL_DISINFECTION;
    is_waiting_at_goal_ = false;
    start_pause_time_ = ros::Time::now().toSec();
    end_pause_time_ = ros::Time::now().toSec();
    last_time_in_motion_ = ros::Time::now();
}

void StateRailDisinfection::finish() {
    ROS_INFO("StateRailDisinfection is now exiting");
    state_manager_->vel_controller_ptr_->sendVelocityCmd(0, 0, 0);
    reset();
    state_manager_->rail_status_ptr_->publishOnRailErrors(false);
    }

void StateRailDisinfection::init() {
    ROS_INFO("StateRailDisinfection is initializing");
    reset();
    state_manager_->rail_status_ptr_->publishOnRailErrors(true);
}



void StateRailDisinfection::interruptState() {
    ROS_INFO("StateRailDisinfection is interrupted");
    // Set 0 velocity
    state_manager_->vel_controller_ptr_->sendVelocityCmd(0, 0, 0);
}

bool StateRailDisinfection::doesRobotNeedToTransition() {
    curr_robot_rail_pose_ = getCurrentPose();
    // Did robot hit something going forward
    if (state_manager_->bumper_handler_ptr_->isFrontBumperHit() && state_manager_->mission_config_ptr_->is_forward_robot_rail_direction) {
        ROS_WARN("Robot has hit something going forwards!");
        if (((curr_robot_rail_pose_.data / state_manager_->mission_goal_manager_->getTotalMissionLength()) > 0.9) || (ros::Time::now().toSec() - last_time_in_motion_.toSec() > RECOVERY_TIME)) {
            return true;
        }else if(!state_manager_->rail_status_ptr_->isEncoderUpdating()){
            ROS_ERROR("Rail encoder is not updating, assuming end of rail");
            return true;
        }
    }
    // Did robot hit something going backward
    if (state_manager_->bumper_handler_ptr_->isBackBumperHit() && !state_manager_->mission_config_ptr_->is_forward_robot_rail_direction) {
        ROS_WARN("Robot has hit something going backwards!");
        if (((curr_robot_rail_pose_.data / state_manager_->mission_goal_manager_->getTotalMissionLength()) > 0.9) || (ros::Time::now().toSec() - last_time_in_motion_.toSec() > RECOVERY_TIME)) {
            return true;
        }else if(!state_manager_->rail_status_ptr_->isEncoderUpdating()){
            ROS_ERROR("Rail encoder is not updating, assuming end of rail");
            return true;
        }
    }
    // Did robot finish all goals according to odometry
    else if (state_manager_->mission_goal_manager_->hasCompletedAllGoals() && !is_waiting_at_goal_) {
        ROS_INFO("Robot has completed all goals!");
        return true;
    }

    else if (curr_robot_rail_pose_.data > state_manager_->mission_config_ptr_->max_tolerable_rail_length) {
        ROS_WARN("Travelled more than tolerable rail length!");
        return true;
    }

    return false;
}

std_msgs::Float32 StateRailDisinfection::getCurrentPose() { return state_manager_->status_manager_ptr_->getCurrentPose(); }

inline bool StateRailDisinfection::isRobotPosAfterGoalPos() {
    curr_robot_rail_pose_ = getCurrentPose();
    return curr_robot_rail_pose_.data >= state_manager_->mission_goal_manager_->getCurrentGoal().position;
}

inline bool StateRailDisinfection::isReversedRobotPosAfterGoalPos() {
    curr_robot_rail_pose_ = getCurrentPose();
    // Note the negative sign
    return curr_robot_rail_pose_.data <= -state_manager_->mission_goal_manager_->getCurrentGoal().position;
}