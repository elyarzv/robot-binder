#include "gtest/gtest.h"
#include "ros/ros.h"
#include "ais_utilities/ais_test_node.h"
#include "ais_state_machine/Enums/phoenix_state_enums.h"
#include "phoenix_msgs/setMode.h"
#include "std_msgs/Int32.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_srvs/Trigger.h"
#include <vector>
#include "phoenix_msgs/SetLampStatus.h"

using namespace std;
using namespace ais_utilities;
using namespace ais_state_machine;

/**
 * @brief This TestSuite provides functionality to how the overall state machine runs in terms of
 *          expected outputs and behaviors. Targetted to analyze the entire node as a blackbox
 *  
 */
class StateMachineTestSuite : public AISTestNode
{
public:
    StateMachineTestSuite();
    ~StateMachineTestSuite();
    /**
     * @brief service call to start a mission for needed to test transitiong from IDLE to INITIALIZE
     * 
     */
    void startMission();
    /**
     * @brief publish fake odometry data (useful for odometry on a rail mission)
     * 
     */
    void publishFakeOdom(float);
    /**
     * @brief publish fake state of charge
     * 
     */
    void publishFakeFrontBumper(bool value);
    void publishFakeSoC(uint);
    void rechargeBattery(uint);
    void publishFakeProxy(uint);
        /**
     * @brief Check current state being used in StateMachineManager (state exposed via ROS publisher)
     * 
     * @return StateTypes 
     */
    StateTypes getCurrentState();
    /**
     * @brief Wait for StateMachineManager to finish transitioning out of inputted state
     * 
     */
    void waitForStateTransition(StateTypes);
    /**
     * @brief Sets the MISSION_DIR param to the install space test folder of ais_state_machine
     * 
     */
    void setMissionDir();
    /**
     * @brief call /set_robot_mode service
     * 
     */
    void callSetRobotMode(string);
    /**
     * @brief Wait for StateMachineManager to transition in to inputted state
     * 
     */
    void waitForState(StateTypes);
    /**
     * @brief Get the Current Intensity object set by the setDimmerValuesService service call
     * 
     * @param intensity 
     */
    void getCurrentIntensity(std::vector<float>& intensity);

protected:
    void SetUp() override;
    ros::Subscriber current_state_sub_;
    ros::ServiceServer set_dimming_server_;

private:
    void currentStateCb(const std_msgs::Int32::ConstPtr&);

    StateTypes curr_state_;

    ros::Publisher fake_odom_pub_;
    ros::Publisher fake_soc_pub_;
    ros::Publisher fake_proxy_pub_;
    ros::Publisher fake_front_bumper_pub_;
    ros::ServiceClient start_mission_client_;
    ros::ServiceClient robot_mode_client_;
    phoenix_msgs::SetLampStatus::Request uv_request;

    bool setDimmerValuesService(phoenix_msgs::SetLampStatus::Request&, 
                                phoenix_msgs::SetLampStatus::Response&);
};