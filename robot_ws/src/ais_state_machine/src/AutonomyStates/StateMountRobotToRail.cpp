#include "ais_state_machine/AutonomyStates/StateMountRobotToRail.h"

using namespace ais_state_machine;

StateMountRobotToRail::StateMountRobotToRail(shared_ptr<AbstractStateMachineManager> state_manager,
                                             ros::NodeHandle& nh)
    : AbstractState(state_manager), AbstractROSState(nh), nh_(nh)
{
    next_state_name_ = StateTypes::MOUNT_ROBOT_TO_RAIL;
    nh_.param<float>("MOUNT_DISTANCE", mount_distance_, 0.75);
}

StateMountRobotToRail::~StateMountRobotToRail()
{
}

void StateMountRobotToRail::runState()
{
    ROS_INFO_THROTTLE(1, "Mount Robot onto Rail");
    float speed = 0.25; //Increase this if we have a higher proximeter status update rate
    /*
    if (state_manager_->mission_config_ptr_->is_forward_robot_rail_direction) {
        speed *= -1.0;
    }
    */
    state_manager_->vel_controller_ptr_->sendVelocityCmd(speed, 0, 0);
    ROS_INFO_THROTTLE(1, "Sent Velocity %f", speed);
    curr_pose_ = state_manager_->wheel_odom_ptr_->getWheelOdom();

    if (state_manager_->mission_config_ptr_->use_uv_lamp){
        state_manager_->uv_lamp_controller_ptr_->setUVIntensity(state_manager_->mission_config_ptr_->rail_transition_intensity);
        state_manager_->uv_lamp_controller_ptr_->setUVLampOn(true);
        ros::spinOnce();
    }

    state_manager_->status_manager_ptr_->setRobotStatus(RobotStatus::AUTO);
    evaluateStateStatus();
}

bool StateMountRobotToRail::isStateFinished()
{
    ROS_DEBUG("State MountRobotToRail Status: %s", has_finished_state_ ? "true" : "false");
    return has_finished_state_;
}

StateTypes StateMountRobotToRail::getNextStateName()
{
    return next_state_name_;
}

void StateMountRobotToRail::evaluateStateStatus()
{
    if(state_manager_->bumper_handler_ptr_->isFrontBumperHit() || state_manager_->bumper_handler_ptr_->isBackBumperHit())
    {
        ROS_ERROR("State MountRobotToRail: Bumper hit while moving");
        state_manager_->status_manager_ptr_->setErrorCode("210020");
        next_state_name_ = StateTypes::HALTED;
    }
    else if(!state_manager_->rail_status_ptr_->isProxyOnRails()){

        initial_pose_ = state_manager_->wheel_odom_ptr_->getWheelOdom();
        curr_pose_ = initial_pose_;
        tf_init2orig_ = p_util_.getTransformFromPoseToOrigin(initial_pose_.pose.pose);
        next_state_name_ = StateTypes::MOUNT_ROBOT_TO_RAIL;
        
    }else{
        tf2::Vector3 vec_local_curr(curr_pose_.pose.pose.position.x, 
                                    curr_pose_.pose.pose.position.y,
                                    curr_pose_.pose.pose.position.z);

        tf2::Vector3 vec_local_init(initial_pose_.pose.pose.position.x,
                                    initial_pose_.pose.pose.position.y,
                                    initial_pose_.pose.pose.position.z);

        vec_local_curr = tf_init2orig_ * vec_local_curr;
        vec_local_init = tf_init2orig_ * vec_local_init;

        float pose_diff = vec_local_curr.getX() - vec_local_init.getX();
        nh_.getParamCached("MOUNT_DISTANCE", mount_distance_);
        ROS_INFO_THROTTLE(0.5, "State MountRobotToRail: pose_diff %f", pose_diff);
        if (
            (state_manager_->mission_config_ptr_->is_forward_robot_rail_direction &&
                 pose_diff > mount_distance_) ||
            (!state_manager_->mission_config_ptr_->is_forward_robot_rail_direction &&
                 pose_diff < -mount_distance_)
       )
       {
           next_state_name_ = StateTypes::INITIALIZE_ROBOT;
       }
       else
       {
           next_state_name_ = StateTypes::MOUNT_ROBOT_TO_RAIL;
       }
    }
    // if(ros::Time::now() > start_time + ros::Duration(30)){
    //     ROS_ERROR("State timeout");
    //     next_state_name_ = StateTypes::RECOVERY;
    // }
}

void StateMountRobotToRail::reset()
{
    next_state_name_ = StateTypes::MOUNT_ROBOT_TO_RAIL;
    nav_msgs::Odometry tmp;
    curr_pose_ = tmp;
    state_manager_->front_rail_manager_ptr_->resetData();
    state_manager_->back_rail_manager_ptr_->resetData();
}

void StateMountRobotToRail::finish()
{
    ROS_INFO("StateMountRobotToRail is now exiting");
    reset();
}

void StateMountRobotToRail::init()
{
    ROS_INFO("StateMountRobotToRail is initializing");
    reset();
    start_time = ros::Time::now();
    initial_pose_ = state_manager_->wheel_odom_ptr_->getWheelOdom();
    curr_pose_ = initial_pose_;
    tf_init2orig_ = p_util_.getTransformFromPoseToOrigin(initial_pose_.pose.pose);
}

void StateMountRobotToRail::interruptState()
{
    ROS_INFO("StateMountRobotToRail is interrupted");
}
