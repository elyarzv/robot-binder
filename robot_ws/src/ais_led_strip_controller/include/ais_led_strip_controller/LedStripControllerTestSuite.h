#include "gtest/gtest.h"
#include "ros/ros.h"
#include "phoenix_msgs/LedPattern.h"
#include "std_msgs/UInt32MultiArray.h"
#include <ais_utilities/ais_test_node.h>

using namespace ais_utilities;

class LedStripControllerTestSuite : public AISTestNode
{
public:
    LedStripControllerTestSuite();
    ~LedStripControllerTestSuite();
    bool setLedPattern(uint8_t type, uint32_t rate, uint32_t r, uint32_t g, uint32_t b);
    std_msgs::UInt32MultiArray getCurrentPattern();

protected:
    void SetUp() override;
    ros::Subscriber led_pattern_sub_;

private:
    void LedStripPatternCb(const std_msgs::UInt32MultiArray::ConstPtr&);
private:
    ros::ServiceClient send_pattern_client_;
    std_msgs::UInt32MultiArray current_pattern_;
    phoenix_msgs::LedPattern requested_pattern_;
};