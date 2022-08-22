/*
    AIS CONFIDENTIAL
    Author: Rakesh Shrestha (r.shrestha@ai-systems.ca)
*/

#include "ais_state_machine/AutonomyStates/StateNavigation.h"

using namespace ais_state_machine;

StateNavigation::StateNavigation(shared_ptr<AbstractStateMachineManager> state_manager, ros::NodeHandle& nh)
    : AbstractState(state_manager), AbstractROSState(nh),
      send_goal_client_("/rail_navigation/send_rail_goal", true)
{
    next_state_name_ = StateTypes::NAVIGATION;
    cmd_vel_publisher_ = nh.advertise<geometry_msgs::Twist>("embedded/cmd_vel", 1000);
    activate_localization_client_ = nh.serviceClient<phoenix_msgs::StartLocalization>(
        "activate_slam_toolbox_localization"
    );
    deactivate_localization_client_ = nh.serviceClient<std_srvs::Trigger>("deactivate_slam_toolbox_localization");
    deactivate_slam_client_ = nh.serviceClient<std_srvs::Trigger>("deactivate_slam_toolbox");

    ROS_INFO("Waiting for send goal action server to start...");
    if(send_goal_client_.waitForServer(ros::Duration(10))){
        ROS_INFO("send goal action server started.");
    }else{
        state_manager_->status_manager_ptr_->setErrorCode("110099");
    }
}

StateNavigation::~StateNavigation()
{

}

void StateNavigation::runState()
{
    ROS_INFO("State Navigation: count %d, Next State is %d", count, (int) next_state_name_);
    cmd_vel_publisher_.publish(state_manager_->status_manager_ptr_->getNavigationCmdVel());
    ros::spinOnce();
    state_manager_->status_manager_ptr_->setRobotStatus(RobotStatus::NAVIGATION);

    reached_goal_ = false;

    // https://sir.upc.edu/projects/rostutorials/7-actions_tutorial/index.html
    // wait for the action to return
    bool finished_before_timeout = send_goal_client_.waitForResult(ros::Duration(0.1));

    // Preempting task
    // send_goal_client_.cancelGoal();

    if (finished_before_timeout) {
        reached_goal_ = true;
        ROS_INFO("Sent to target rail");

        actionlib::SimpleClientGoalState state = send_goal_client_.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
        // Preempting the process
        send_goal_client_.cancelGoal();
    }

    evaluateStateStatus();
}

bool StateNavigation::isStateFinished()
{
    ROS_INFO("State Navigation Status: %s", has_finished_state_ ? "true" : "false");
    return has_finished_state_;
}

StateTypes StateNavigation::getNextStateName()
{
    return next_state_name_;
}

void StateNavigation::evaluateStateStatus()
{
    if (reached_goal_) {
        next_state_name_ = StateTypes::ADJUST_TO_RAIL;  // Change to StateType::ADJUST_TO_RAIL
    } else {
        next_state_name_ = StateTypes::NAVIGATION;
    }
}

void StateNavigation::reset()
{
    next_state_name_ = StateTypes::NAVIGATION;
}

void StateNavigation::finish()
{
    ROS_INFO("StateNavigation is now exiting");

    /*
     * Either localization or SLAM needs to be activate
     * for fixing orientation at the end of AdjustToRail
     * Hence, do not deactivate localization at the end of navigation state
     */
    // std_srvs::Trigger trigger_srv;
    // if (deactivate_localization_client_.call(trigger_srv)) {
    //     ROS_INFO("Localization deactivated");
    // } else {
    //     ROS_ERROR("Unable to deactivate localization");
    // }

    reset();
    state_manager_->rail_status_ptr_->publishOffRailErrors(false);
}

void StateNavigation::init()
{
    ROS_INFO("StateNavigation is initializing");

    // SLAM might be activated, deactivate it before starting localization
    std_srvs::Trigger trigger_srv;
    auto success = deactivate_slam_client_.call(trigger_srv);
    if (success && trigger_srv.response.success) {
        ROS_INFO("SLAM deactivated");
    } else {
        ROS_ERROR("Unable to deactivate SLAM: %s", trigger_srv.response.message.c_str());
    }

    phoenix_msgs::StartLocalization start_localization_srv;
    start_localization_srv.request.rail_number
        = state_manager_->rail_status_ptr_->getCurrentRailNumber();
    success = activate_localization_client_.call(start_localization_srv);

    if (success && start_localization_srv.response.success) {
        ROS_INFO("Localization activated");
        // wait for localization to stabilize
        ros::Duration(2.5).sleep();
    } else {
        ROS_ERROR(
            "Unable to activate localization: %s", start_localization_srv.response.message
        );
    }

    reset();
    state_manager_->rail_status_ptr_->publishOffRailErrors(true);

    const auto target_rail_number = state_manager_->rail_status_ptr_->getTargetRailNumber();

    if (target_rail_number > 0) {
        ROS_INFO("Sending to rail number %d...", target_rail_number);

        phoenix_msgs::SendRailGoalGoal goal;
        goal.rail_number = target_rail_number;

        send_goal_client_.sendGoal(goal);
    } else {
        ROS_ERROR("Invalid target rail number %d", target_rail_number);
    }
}

void StateNavigation::interruptState()
{
    ROS_INFO("StateNavigation is interrupted");

    std_srvs::Trigger trigger_srv;
    if (deactivate_localization_client_.call(trigger_srv)) {
        ROS_INFO("Localization deactivated");
    } else {
        ROS_ERROR("Unable to deactivate localization");
    }
}

