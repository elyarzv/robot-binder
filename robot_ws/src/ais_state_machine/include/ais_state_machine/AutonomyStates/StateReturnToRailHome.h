/*
    AIS CONFIDENTIAL
    Author: Michael Chiou (m.chiou@ai-systems.ca)
*/

#ifndef ASM_STATE_RETURN_TO_RAIL_HOME_H_
#define ASM_STATE_RETURN_TO_RAIL_HOME_H_

#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt8.h"
#include "geometry_msgs/Twist.h"
#include "ais_state_machine/AbstractClasses/AbstractState.h"
#include "ais_state_machine/AbstractClasses/AbstractROSState.h"
#include <atomic>
namespace ais_state_machine 
{
/**
 * @brief This State is return the robot to the initial rail position based on odometry. Generally
 *        this is assumed to be the point where the robot initially positioned itself on the rail.
 *        Exit conditions include reaching home (reached 0 wheel odomoetry) or when the bumper has
 *        been triggered when travelling in the corresponding direction 
 */
class StateReturnToRailHome final: public AbstractState, public AbstractROSState
{
public: 
    StateReturnToRailHome(shared_ptr<AbstractStateMachineManager> state_manager, ros::NodeHandle& nh);
    ~StateReturnToRailHome() override;
    void runState() override;
    bool isStateFinished() override;
    StateTypes getNextStateName() override;
    void reset() override;
    void finish() override;
    void init() override;
    void interruptState() override;
private: 
    void evaluateStateStatus() override;

    void poseCb(const std_msgs::Float32ConstPtr&);
    /**
     * @brief Check if robot has returned home relative to robot orientation on rail
     *        Use inverse if robot started on rail in reverse orientation
     * 
     * @return true 
     * @return false 
     */
    bool isRobotPosAfterHomePos();
    bool doesRobotNeedToTransition();
private:
    
    geometry_msgs::Twist cmd_vel;
    std_msgs::Float32 curr_robot_rail_pose_;

    ros::Subscriber pose_subscriber_;
    
    ros::Publisher cmd_vel_publisher_;
    ros::Publisher led_publisher_;
    
    ros::ServiceClient reset_pose_service_;
};

} // end of namespace ais_state_machine
#endif // END ASM_STATE_RETURN_TO_RAIL_HOME_H_