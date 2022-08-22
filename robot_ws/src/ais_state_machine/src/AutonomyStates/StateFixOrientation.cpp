/*
    AIS CONFIDENTIAL
    Author: Rakesh Shrestha (r.shrestha@ai-systems.ca)
*/

#include "ais_state_machine/AutonomyStates/StateFixOrientation.h"

#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/TransformStamped.h"

#include <cmath>

#define WINDOW_SIZE 5

using namespace ais_state_machine;
double constrainAngle(double x);
double rad2deg(double rad);

StateFixOrientation::StateFixOrientation(
    shared_ptr<AbstractStateMachineManager> state_manager, ros::NodeHandle& nh
) : AbstractState(state_manager), AbstractROSState(nh),
    is_orientation_fixed_(false), rolling_window_(0) {
    next_state_name_ = StateTypes::FIX_ORIENTATION;
    cmd_vel_publisher_ = nh.advertise<geometry_msgs::Twist>("embedded/cmd_vel", 1000);

    activate_slam_client_ = nh.serviceClient<std_srvs::Trigger>("activate_slam_toolbox");
    deactivate_slam_client_ = nh.serviceClient<std_srvs::Trigger>("deactivate_slam_toolbox");
    deactivate_localization_client_ = nh.serviceClient<std_srvs::Trigger>("deactivate_slam_toolbox_localization");

    tf_buffer_ = make_shared<tf2_ros::Buffer>();
    tf_listener_ = make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

StateFixOrientation::~StateFixOrientation() {
}

void StateFixOrientation::runState() {
    ROS_INFO_THROTTLE(0.5, "State FixOrientation");

    auto orientation = getRobotOrientation();
    if(state_manager_->bumper_handler_ptr_->isFrontBumperHit() || state_manager_->bumper_handler_ptr_->isBackBumperHit())
    {
        ROS_ERROR("State FixOrientation: Bumper hit while moving");
        state_manager_->status_manager_ptr_->setErrorCode("210020");
    }
    else if (orientation)
    {
        auto error = (*orientation);
        ROS_INFO("Orientation error: %f", error);
        if (is_orientation_fixed_ || std::abs(error) < error_tolerance_)
        {
            ++rolling_window_;
            if (rolling_window_ >= WINDOW_SIZE)
            {
                state_manager_->vel_controller_ptr_->sendVelocityCmd(0, 0, 0);
                is_orientation_fixed_ = true;
                // avoid overflow
                rolling_window_ = WINDOW_SIZE;
            }
        }

        if (!is_orientation_fixed_)
        {
            if (std::abs(error) >= error_tolerance_)
            {
                rolling_window_ = 0;
            }
            auto angular_velocity = -(p_gain_ * error);
            if (std::abs(angular_velocity) < min_angular_velocity_)
            {
                angular_velocity = (angular_velocity > 0)
                                    ? min_angular_velocity_
                                    : -min_angular_velocity_;
            }
            if (std::abs(angular_velocity) > max_angular_velocity_)
            {
                angular_velocity = (angular_velocity > 0)
                                    ? max_angular_velocity_
                                    : -max_angular_velocity_;
            }
            state_manager_->vel_controller_ptr_->sendVelocityCmd(0, 0, angular_velocity);
            is_orientation_fixed_ = false;
        }
    }
    else
    {
        ROS_ERROR("Could not get robot orientation, moving to next state");
        state_manager_->vel_controller_ptr_->sendVelocityCmd(0, 0, 0);
        is_orientation_fixed_ = true;
    }

    // cmd_vel_publisher_.publish(state_manager_->status_manager_ptr_->getNavigationCmdVel());
    ros::spinOnce();
    // state_manager_->status_manager_ptr_->setRobotStatus(RobotStatus::NAVIGATION);
    // evaluateStateStatus();
}

std::optional<double> StateFixOrientation::getRobotOrientation() {
    geometry_msgs::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_->lookupTransform(
          "map", "base_link", ros::Time(0)
      );
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      return std::nullopt;
    }

    return getYaw(transform_stamped.transform.rotation);
}

double constrainAngle(double x) {
    x = fmod(x + 180, 360);
    if (x < 0)
        x += 360;
    return x - 180;
}

double rad2deg(double rad) {
    return rad * 180 / M_PI;
}

double StateFixOrientation::getYaw(const geometry_msgs::Quaternion quaternion) {
    tf2::Matrix3x3 matrix3x3(tf2::Quaternion(
        quaternion.x, quaternion.y, quaternion.z, quaternion.w
    ));
    double r, p, y;
    matrix3x3.getRPY(r, p, y);
    return constrainAngle(rad2deg(y));
}

void StateFixOrientation::evaluateStateStatus() {
}

bool StateFixOrientation::isStateFinished() {
    ROS_INFO("State FixOrientation Status: %s", has_finished_state_ ? "true" : "false");
    return has_finished_state_;
}

StateTypes StateFixOrientation::getNextStateName() {
    return next_state_name_;
}

void StateFixOrientation::reset() {
    // next_state_name_ = StateTypes::MOVE_TO_NEXT_RAIL;
    is_orientation_fixed_ = false;
    rolling_window_ = 0;

    nh_.param<double>("ORIENTATION_P_GAIN", p_gain_, 0.05);
    nh_.param<double>("ORIENTATION_ERROR_TOLERANCE", error_tolerance_, 1.0);
    nh_.param<double>("ORIENTATION_MIN_VELOCITY", min_angular_velocity_, 0.175);
    nh_.param<double>("ORIENTATION_MAX_VELOCITY", max_angular_velocity_, 0.5);

    min_angular_velocity_ = std::abs(min_angular_velocity_);
}

void StateFixOrientation::finish() {
    ROS_INFO("StateFixOrientation is now exiting");
    reset();
}

void StateFixOrientation::init() {
    ROS_INFO("StateFixOrientation is initializing");
    reset();
}

void StateFixOrientation::interruptState() {
    ROS_INFO("StateFixOrientation is now exiting");
}

void StateFixOrientation::activateSLAM() {
    // deactivate slam just in case (to reinitialize the pose and clear the map)
    // deactivateSLAM();

    std_srvs::Trigger trigger_srv;
    auto success = activate_slam_client_.call(trigger_srv);
    if (success && trigger_srv.response.success) {
        ROS_INFO("SLAM activated");
    } else {
        ROS_ERROR("Unable to activate SLAM: %s", trigger_srv.response.message.c_str());
    }
}

void StateFixOrientation::deactivateSLAM() {
    std_srvs::Trigger trigger_srv;
    auto success = deactivate_slam_client_.call(trigger_srv);
    if (success && trigger_srv.response.success) {
        ROS_INFO("SLAM deactivated");
    } else {
        ROS_ERROR("Unable to deactivate SLAM: %s", trigger_srv.response.message.c_str());
    }
}

void StateFixOrientation::deactivateLocalization() {
    std_srvs::Trigger trigger_srv;
    auto success = deactivate_localization_client_.call(trigger_srv);
    if (success && trigger_srv.response.success) {
        ROS_INFO("Localization deactivated");
    } else {
        ROS_ERROR("Unable to deactivate Localization: %s", trigger_srv.response.message.c_str());
    }
}
