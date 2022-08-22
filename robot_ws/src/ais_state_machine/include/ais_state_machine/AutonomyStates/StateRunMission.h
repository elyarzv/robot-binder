/*
    AIS CONFIDENTIAL
    Author: Michael Wrock (m.wrock@ai-systems.ca)
*/

#ifndef ASM_STATE_RUN_MISSION_H_
#define ASM_STATE_RUN_MISSION_H_

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt8.h"
#include "std_srvs/Trigger.h"
#include "geometry_msgs/Twist.h"
#include "ais_state_machine/AbstractClasses/AbstractState.h"
#include "ais_state_machine/AbstractClasses/AbstractROSState.h"
namespace ais_state_machine 
{

class StateRunMission final: public AbstractState, public AbstractROSState
{
public: 
    StateRunMission(shared_ptr<AbstractStateMachineManager> state_manager, ros::NodeHandle& nh);
    ~StateRunMission() override;
    void runState() override;
    void finish() override;
    void init() override;
    void interruptState() override;
    bool isStateFinished() override;
    StateTypes getNextStateName() override;
    void reset() override;        
    /**
    * @brief send the lamp intensity to the micros
    * 
    * @param intensity 
    * @return true 
    * @return false 
    */
    void sendIntensity(int val);
    /**
     * @brief turn off uv relay
     * 
     */
    void uvOff();
    /**
     * @brief turn on uv relay
     * 
     */
    void uvOn();
private: 
    void evaluateStateStatus() override;
    void poseCb(const std_msgs::Float32ConstPtr &pose);
private:
    int count;
    std_msgs::Float32 current_pose_;
    ros::Subscriber pose_subscriber_;
    ros::Publisher cmd_vel_publisher_;
    ros::Publisher led_publisher_;
    ros::ServiceClient reset_pose_service_;
    ros::ServiceClient send_intensity_client_;
    ros::Publisher lamp_power_;
    std_msgs::ByteMultiArray lamp_vector_;
};

} // end of namespace ais_state_machine
#endif // END ASM_STATE_RUN_MISSION_H_