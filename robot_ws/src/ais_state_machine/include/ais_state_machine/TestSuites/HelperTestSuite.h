#include "gtest/gtest.h"
#include "ros/ros.h"
#include "ais_utilities/ais_test_node.h"
#include "ais_state_machine/Helpers/RailStatusManager.h"
#include "std_msgs/UInt8.h"

using namespace std;
using namespace ais_utilities;
using namespace ais_state_machine;

/**
 * @brief This TestSuite provides functionality to test individual helpers in the State Machine
 */
class HelperTestSuite : public AISTestNode
{
public:
    HelperTestSuite();
    ~HelperTestSuite();
    bool getProxyStatus();
    void publishFakeProxy(uint val);
    
protected:
    void SetUp() override;
    ros::Publisher fake_proxy_pub_;
    shared_ptr<RailStatusManager> rail_status_ptr_;

private:
    
};