#include "ais_state_machine/AutonomyStates/StateMoveToNextRail.h"

using namespace ais_state_machine;

StateMoveToNextRail::StateMoveToNextRail(
    shared_ptr<AbstractStateMachineManager> state_manager,
    ros::NodeHandle& nh,
    shared_ptr<StateFixOrientation> state_fix_orientation)
    : AbstractState(state_manager), AbstractROSState(nh),
      state_fix_orientation_(state_fix_orientation), fixing_orientation_(false)
{
    next_state_name_ = StateTypes::MOVE_TO_NEXT_RAIL;
}

StateMoveToNextRail::~StateMoveToNextRail()
{
}

void StateMoveToNextRail::runState()
{
    if (fixing_orientation_)
    {
        ROS_INFO_THROTTLE(1, "Move To Next Rail: fixing orientation");
        state_fix_orientation_->runState();
        evaluateStateStatus();
        return;
    }

    const auto current_rail_number = state_manager_->rail_status_ptr_->getCurrentRailNumber();
    const auto target_rail_number = state_manager_->rail_status_ptr_->getTargetRailNumber();

    ROS_INFO(
        "State MoveToNextRail: end of state. current rail: %d, target rail: %d",
        current_rail_number, target_rail_number
    );
    if (std::abs(target_rail_number - current_rail_number) != 1) {
        ROS_WARN(
            "State StateMoveToNextRail: target (%d) and current (%d) rail diff > 1",
            target_rail_number,- current_rail_number
        );
    }

    float speed = 0.25;
    /**
     * use rail_status_mananger instead of rail_is_left_adjacent
     * even when old mission params used, these values will work
     */
    if (target_rail_number < current_rail_number)
    // if (!state_manager_->mission_config_ptr_->rail_is_left_adjacent)
    {
        speed *= -1.0;
        state_manager_->mission_config_ptr_->rail_is_left_adjacent = false;
    }
    else
    {
        state_manager_->mission_config_ptr_->rail_is_left_adjacent = true;
    }

    state_manager_->vel_controller_ptr_->sendVelocityCmd(0, speed, 0);
    curr_pose_ = state_manager_->wheel_odom_ptr_->getWheelOdom();

    if (state_manager_->mission_config_ptr_->use_uv_lamp){
        state_manager_->uv_lamp_controller_ptr_->setUVIntensity(state_manager_->mission_config_ptr_->rail_transition_intensity);
        state_manager_->uv_lamp_controller_ptr_->setUVLampOn(true);
        ros::spinOnce();
    }
    state_manager_->status_manager_ptr_->setRobotStatus(RobotStatus::AUTO);
    evaluateStateStatus();
}

bool StateMoveToNextRail::isStateFinished()
{
    ROS_DEBUG("State StateMoveToNextRail Status: %s", has_finished_state_ ? "true" : "false");
    return has_finished_state_;
}

StateTypes StateMoveToNextRail::getNextStateName()
{
    return next_state_name_;
}

void StateMoveToNextRail::evaluateStateStatus()
{
    tf2::Vector3 vec_local_curr(curr_pose_.pose.pose.position.x,
                                curr_pose_.pose.pose.position.y,
                                curr_pose_.pose.pose.position.z);

    tf2::Vector3 vec_local_init(initial_pose_.pose.pose.position.x,
                                initial_pose_.pose.pose.position.y,
                                initial_pose_.pose.pose.position.z);

    vec_local_curr = tf_init2orig_ * vec_local_curr;
    vec_local_init = tf_init2orig_ * vec_local_init;

    float pose_diff = vec_local_curr.getY() - vec_local_init.getY();
    float odom_diff = 1.25;
    fixing_orientation_ = false;
    ROS_INFO_THROTTLE(0.5, "State StateMoveToNextRail: pose_diff %f", pose_diff);

    if ((state_manager_->mission_config_ptr_->rail_is_left_adjacent && pose_diff > odom_diff) ||
            (!state_manager_->mission_config_ptr_->rail_is_left_adjacent && pose_diff < -odom_diff))
    {
        if (!state_fix_orientation_->isOrientationFixed()) {
            fixing_orientation_ = true;
            next_state_name_ = StateTypes::MOVE_TO_NEXT_RAIL;
        } else {
            next_state_name_ = StateTypes::ADJUST_TO_RAIL;  // Change to StateType::ADJUST_TO_RAIL
        }
    }
    else if (state_manager_->status_manager_ptr_->willRobotCollide()){
        if(collision_flag_){
            ROS_ERROR("StateMoveToNextRail: Stopped by safety for %f seconds", ros::Time::now().toSec() - safety_stop_time_.toSec());
            if(ros::Time::now().toSec() - safety_stop_time_.toSec() > safety_timeout){
                next_state_name_ = StateTypes::RECOVERY;
            }else{
                next_state_name_ = StateTypes::MOVE_TO_NEXT_RAIL;
            }
        }else{
            collision_flag_ = true;
            safety_stop_time_ = ros::Time::now();
            next_state_name_ = StateTypes::MOVE_TO_NEXT_RAIL;
        }
    }else if(state_manager_->bumper_handler_ptr_->isFrontBumperHit() || state_manager_->bumper_handler_ptr_->isBackBumperHit())
    {
        ROS_ERROR("StateMoveToNextRail: Bumper hit while moving");
        state_manager_->status_manager_ptr_->setErrorCode("210020");
        next_state_name_ = StateTypes::HALTED;
    }else{
        collision_flag_ = false;
        next_state_name_ = StateTypes::MOVE_TO_NEXT_RAIL;
    }
    if(ros::Time::now() > start_time + ros::Duration(STATE_TIMEOUT)){
        ROS_ERROR("State MoveToNextRail timeout");
        state_manager_->status_manager_ptr_->setErrorCode("210021");
        next_state_name_ = StateTypes::RECOVERY;
    }
}

void StateMoveToNextRail::reset()
{
    next_state_name_ = StateTypes::MOVE_ROBOT_OFF_RAIL;
    nav_msgs::Odometry tmp;
    curr_pose_ = tmp;
    state_manager_->front_rail_manager_ptr_->resetData();
    state_manager_->back_rail_manager_ptr_->resetData();
    state_fix_orientation_->reset();
}

void StateMoveToNextRail::finish()
{
    ROS_INFO("StateMoveToNextRail is now exiting");
    reset();
    state_fix_orientation_->deactivateSLAM();
    state_manager_->rail_status_ptr_->publishOffRailErrors(false);
}

void StateMoveToNextRail::init()
{
    ROS_INFO("StateMoveToNextRail is initializing");
    reset();
    start_time = ros::Time::now();
    initial_pose_ = state_manager_->wheel_odom_ptr_->getWheelOdom();
    curr_pose_ = initial_pose_;
    tf_init2orig_ = p_util_.getTransformFromPoseToOrigin(initial_pose_.pose.pose);
    // SLAM should already be activated in MoveRobotOffRail
    // state_fix_orientation_->activateSLAM();
    state_manager_->rail_status_ptr_->publishOffRailErrors(true);
}

void StateMoveToNextRail::interruptState()
{
    ROS_INFO("StateMoveToNextRail is interrupted");
    state_fix_orientation_->deactivateSLAM();
}
