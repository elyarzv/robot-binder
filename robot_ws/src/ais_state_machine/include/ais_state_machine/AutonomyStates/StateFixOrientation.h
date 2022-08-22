/*
    AIS CONFIDENTIAL
    Author: Rakesh Shrestha (r.shrestha@ai-systems.ca)
*/

#ifndef ASM_STATE_FIX_ORIENTATION_H_
#define ASM_STATE_FIX_ORIENTATION_H_

#include "ros/ros.h"
#include "std_srvs/Trigger.h"

#include "ais_state_machine/AbstractClasses/AbstractState.h"
#include "ais_state_machine/AbstractClasses/AbstractROSState.h"

#include "tf2_ros/transform_listener.h"

#include <optional>

namespace ais_state_machine
{
class StateFixOrientation final: public AbstractState, public AbstractROSState
{
public:
    StateFixOrientation(shared_ptr<AbstractStateMachineManager> state_manager, ros::NodeHandle& nh);
    ~StateFixOrientation() override;
    void runState() override;
    bool isStateFinished() override;
    StateTypes getNextStateName() override;
    void reset() override;
    void finish() override;
    void init() override;
    void interruptState() override;

    void activateSLAM();
    void deactivateSLAM();
    void deactivateLocalization();
    bool isOrientationFixed() { return is_orientation_fixed_; }
private:
    void evaluateStateStatus() override;

    double error_tolerance_;
    double p_gain_;
    double min_angular_velocity_;
    double max_angular_velocity_;

    int rolling_window_;

    std::optional<double> getRobotOrientation();
    double getYaw(const geometry_msgs::Quaternion quaternion);
    bool is_orientation_fixed_;

    ros::Publisher cmd_vel_publisher_;

    ros::ServiceClient activate_slam_client_;
    ros::ServiceClient deactivate_slam_client_;
    ros::ServiceClient deactivate_localization_client_;

    shared_ptr<tf2_ros::Buffer> tf_buffer_;
    shared_ptr<tf2_ros::TransformListener> tf_listener_;

}; // endclass StateFixOrientation
}
#endif // ASM_STATE_FIX_ORIENTATION_H_
