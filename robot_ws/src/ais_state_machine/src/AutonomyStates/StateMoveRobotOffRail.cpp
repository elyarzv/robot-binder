#include "ais_state_machine/AutonomyStates/StateMoveRobotOffRail.h"

using namespace ais_state_machine;

StateMoveRobotOffRail::StateMoveRobotOffRail(
    shared_ptr<AbstractStateMachineManager> state_manager, ros::NodeHandle &nh,
    shared_ptr<StateFixOrientation> state_fix_orientation)
    : AbstractState(state_manager), AbstractROSState(nh), fixing_orientation_(false),
      state_fix_orientation_(state_fix_orientation) {
    next_state_name_ = StateTypes::MOVE_ROBOT_OFF_RAIL;
    immediate_recovery_ = false;
    nh_.param<float>("DISMOUNT_DISTANCE", dismount_distance_, 0.95);
}

StateMoveRobotOffRail::~StateMoveRobotOffRail() {}

void StateMoveRobotOffRail::runState() {
    ROS_INFO_THROTTLE(1, "Move Robot Off Rail");
    if (fixing_orientation_)
    {
        ROS_INFO_THROTTLE(1, "Move Robot Off Rail: fixing orientation");
        state_fix_orientation_->runState();
        evaluateStateStatus();
        return;
    }

    float speed = 0.25; // Increase this if we have a higher proximeter status update rate
    if (state_manager_->mission_config_ptr_->is_forward_robot_rail_direction) {
        speed *= -1.0;
    }

    state_manager_->vel_controller_ptr_->sendVelocityCmd(speed, 0, 0);
    ROS_INFO_THROTTLE(1, "Sent Velocity %f", speed);
    curr_pose_ = state_manager_->wheel_odom_ptr_->getWheelOdom();

    if (state_manager_->mission_config_ptr_->use_uv_lamp) {
        state_manager_->uv_lamp_controller_ptr_->setUVIntensity(state_manager_->mission_config_ptr_->rail_transition_intensity);
        state_manager_->uv_lamp_controller_ptr_->setUVLampOn(true);
        ros::spinOnce();
    }
    state_manager_->status_manager_ptr_->setRobotStatus(RobotStatus::AUTO);
    evaluateStateStatus();
}

bool StateMoveRobotOffRail::isStateFinished() {
    ROS_DEBUG("State MoveRobotOffRail Status: %s", has_finished_state_ ? "true" : "false");
    return has_finished_state_;
}

StateTypes StateMoveRobotOffRail::getNextStateName() { return next_state_name_; }

void StateMoveRobotOffRail::evaluateStateStatus() {

    fixing_orientation_ = false;
    if(immediate_recovery_){
        next_state_name_ = StateTypes::RECOVERY;
    }
    else if (state_manager_->status_manager_ptr_->willRobotCollide()) {
        if (collision_flag_) {
            ROS_ERROR("State MoveRobotOffRail: Stopped by safety for %f seconds", ros::Time::now().toSec() - safety_stop_time_.toSec());
            if (ros::Time::now().toSec() - safety_stop_time_.toSec() > safety_timeout) {
                next_state_name_ = StateTypes::RECOVERY;
            } else {
                next_state_name_ = StateTypes::MOVE_ROBOT_OFF_RAIL;
            }
        } else {
            collision_flag_ = true;
            safety_stop_time_ = ros::Time::now();
            next_state_name_ = StateTypes::MOVE_ROBOT_OFF_RAIL;
        }
    }else if(state_manager_->bumper_handler_ptr_->isFrontBumperHit() || state_manager_->bumper_handler_ptr_->isBackBumperHit())
    {
        ROS_ERROR("State MoveRobotOffRail: Bumper hit while moving");
        state_manager_->status_manager_ptr_->setErrorCode("210020");
        next_state_name_ = StateTypes::HALTED;
    }else {
        collision_flag_ = false;
        next_state_name_ = StateTypes::MOVE_ROBOT_OFF_RAIL;
        if (state_manager_->rail_status_ptr_->isProxyOnRails()) {

            initial_pose_ = state_manager_->wheel_odom_ptr_->getWheelOdom();
            curr_pose_ = initial_pose_;
            tf_init2orig_ = p_util_.getTransformFromPoseToOrigin(initial_pose_.pose.pose);
            next_state_name_ = StateTypes::MOVE_ROBOT_OFF_RAIL;

        } else {
            tf2::Vector3 vec_local_curr(curr_pose_.pose.pose.position.x,
                                        curr_pose_.pose.pose.position.y,
                                        curr_pose_.pose.pose.position.z);

            tf2::Vector3 vec_local_init(initial_pose_.pose.pose.position.x,
                                        initial_pose_.pose.pose.position.y,
                                        initial_pose_.pose.pose.position.z);

            vec_local_curr = tf_init2orig_ * vec_local_curr;
            vec_local_init = tf_init2orig_ * vec_local_init;

            float pose_diff = vec_local_curr.getX() - vec_local_init.getX();

            nh_.getParamCached("DISMOUNT_DISTANCE", dismount_distance_);
            ROS_INFO_THROTTLE(0.5, "State MoveRobotOffRail: pose_diff %f", pose_diff);
            if ((state_manager_->mission_config_ptr_->is_forward_robot_rail_direction && pose_diff < -dismount_distance_) ||
                    (!state_manager_->mission_config_ptr_->is_forward_robot_rail_direction && pose_diff > dismount_distance_)) {
                if (!state_fix_orientation_->isOrientationFixed()) {
                    fixing_orientation_ = true;
                    next_state_name_ = StateTypes::MOVE_ROBOT_OFF_RAIL;
                } else {
                    const auto current_rail_number = state_manager_->rail_status_ptr_->getCurrentRailNumber();
                    const auto target_rail_number = state_manager_->rail_status_ptr_->getTargetRailNumber();
                    ROS_INFO(
                        "State MoveRobotOffRail: end of state. current rail: %d, target rail: %d",
                        current_rail_number, target_rail_number
                    );

                    if (std::abs(target_rail_number - current_rail_number) > 1){
                        // go to target rail number
                        next_state_name_ = StateTypes::NAVIGATION;
                    } else if (std::abs(target_rail_number - current_rail_number) == 1) {
                        double midpoint = (double)(state_manager_->mission_config_ptr_->num_rails + 1) / 2.0;

                        if (
                            (midpoint - target_rail_number < 0 && midpoint - current_rail_number < 0) ||
                            (midpoint - target_rail_number > 0 && midpoint - current_rail_number > 0)
                        ) {
                            // go to the adjacent rail
                            next_state_name_ = StateTypes::MOVE_TO_NEXT_RAIL;
                        } else {
                            // difference = 1 might not mean adjacent rails if multiple rows of rails
                            next_state_name_ = StateTypes::NAVIGATION;
                        }
                    } else {
                        // redundant rail in adjacent jobs
                        ROS_WARN(
                            "State MoveRobotOffRail: Rail %d redundant in adjacent jobs",
                            target_rail_number
                        );
                        next_state_name_ = StateTypes::MOUNT_ROBOT_TO_RAIL;
                    }
                }
            } else {
                next_state_name_ = StateTypes::MOVE_ROBOT_OFF_RAIL;
            }
        }
    }
    // if(ros::Time::now() > start_time + ros::Duration(30)){
    //         ROS_ERROR("State timeout");
    //         next_state_name_ = StateTypes::RECOVERY;
    // }
}

void StateMoveRobotOffRail::reset() {
    next_state_name_ = StateTypes::MOVE_ROBOT_OFF_RAIL;
    nav_msgs::Odometry tmp;
    curr_pose_ = tmp;
    state_manager_->front_rail_manager_ptr_->resetData();
    state_manager_->back_rail_manager_ptr_->resetData();
    state_fix_orientation_->reset();
    collision_flag_ = false;
}

void StateMoveRobotOffRail::finish() {
    ROS_INFO("StateMoveRobotOffRail is now exiting");
    reset();
    // state_fix_orientation_->deactivateSLAM();
}

void StateMoveRobotOffRail::init() {
    ROS_INFO("StateMoveRobotOffRail is initializing");
    reset();
    start_time = ros::Time::now();
    initial_pose_ = state_manager_->wheel_odom_ptr_->getWheelOdom();
    curr_pose_ = initial_pose_;
    tf_init2orig_ = p_util_.getTransformFromPoseToOrigin(initial_pose_.pose.pose);

    // deactivate slam just in case (to reinitialize the pose and clear the map)
    state_fix_orientation_->deactivateLocalization();
    state_fix_orientation_->deactivateSLAM();
    state_fix_orientation_->activateSLAM();
    if(state_manager_->status_manager_ptr_->robot_diagnostic_->isRobotError()){
        immediate_recovery_ = true;
    }else immediate_recovery_ = false;
}

void StateMoveRobotOffRail::interruptState() {
    ROS_INFO("StateMoveRobotOffRail is interrupted");
    // state_fix_orientation_->deactivateSLAM();
}
