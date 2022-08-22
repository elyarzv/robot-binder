#include "gtest/gtest.h"
#include "ros/ros.h"
#include "ais_utilities/ais_test_node.h"
#include "ais_state_machine/Enums/phoenix_state_enums.h"
#include "ais_state_machine/Core/StateFactory.h"
#include "ais_state_machine/Core/StateMachineManager.h"
#include "ais_state_machine/AbstractClasses/AbstractState.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_srvs/Trigger.h"
#include "geometry_msgs/Twist.h"
#include <vector>
#include <phoenix_msgs/SetLampStatus.h>
#include "phoenix_msgs/DimmingIrradianceData.h"

using namespace std;
using namespace ais_utilities;
using namespace ais_state_machine;

/**
 * @brief This TestSuite provides functionality to test individual states run in the State Machine
 *        Test transitions should not be tested here but rather how individual states can react 
 *        to parameters or inputs. Targetted to examine the StateMachineManager and States classes
 */
class IndividualStateTestSuite : public AISTestNode
{
public:
    IndividualStateTestSuite();
    ~IndividualStateTestSuite();

    /**
     * @brief Check current state being used in StateMachineManager (state exposed via ROS publisher)
     * 
     * @return StateTypes 
     */
    StateTypes getCurrentState();
    /**
     * @brief Get cmd_velocity published from StateMachineManager (exposed via ROS Publisher)
     * 
     * @return geometry_msgs::Twist 
     */
    geometry_msgs::Twist getPublishedCmdVel();
    /**
     * @brief publish fake odometry data (useful for odometry on a rail mission)
     * 
     */
    void publishFakeOdom(float);
    /**
     * @brief service call to start a mission for needed to test transitiong from IDLE to INITIALIZE
     * 
     */
    void startMission();
    /**
     * @brief Wait for StateMachineManager to finish transitioning out of inputted state
     * 
     */
    void waitForStateTransition(StateTypes);
    /**
     * @brief Force StateMachineManager to transition to this state, no checks to see if 
     *        input state is a proper state
     */
    void transitionStateTo(StateTypes);
    /**
     * @brief Execute current state used by StateMachineManager once (State virtual run() function)
     * 
     */
    void runStateOnce();

    /*
    *   @brief Publishes fake irradiance values for tests, uses corresponding phoenix_msg
    */

    void publishFakeUVIrradiance(double irrad_val, double irrad_percent);
    
protected:
    void SetUp() override;
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher fake_odom_pub_;
    ros::Publisher fake_irradiance_pub_;
    ros::ServiceServer set_dimming_server_;
    shared_ptr<StateFactory> factory_ptr_;
    shared_ptr<StateMachineManager> sm_ptr_;
    shared_ptr<AbstractState> state_ptr_;
    StateTypes curr_state_;
    geometry_msgs::Twist received_cmd_vel_;

private:
    void cmdVelocityCb(const geometry_msgs::Twist::ConstPtr&);
    void currentStateCb(const std_msgs::Int32::ConstPtr&);

    /**
     * @brief Dummy service since StateMachineManager checks for this service or else it blocks
     * 
     * @return true 
     * @return false 
     */
    bool setDimmerValuesService(phoenix_msgs::SetLampStatus::Request&, 
                                phoenix_msgs::SetLampStatus::Response&);
    
};