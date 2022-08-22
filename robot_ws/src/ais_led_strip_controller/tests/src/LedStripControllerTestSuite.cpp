#include "ais_led_strip_controller/LedStripControllerTestSuite.h"

using namespace ais_utilities;

LedStripControllerTestSuite::LedStripControllerTestSuite()
{
}

LedStripControllerTestSuite::~LedStripControllerTestSuite()
{
}

bool LedStripControllerTestSuite::setLedPattern(uint8_t type, uint32_t rate, uint32_t r, uint32_t g,
                                                uint32_t b)
{
    // call client
    requested_pattern_.request.pattern = type;
    requested_pattern_.request.rate = rate;
    requested_pattern_.request.red = r;
    requested_pattern_.request.blue = g;
    requested_pattern_.request.green = b;

    requested_pattern_.request.size = 49;
    requested_pattern_.request.use_RGB = true;
    requested_pattern_.request.fill_percent = 100;

    send_pattern_client_.call(requested_pattern_);

    return true;
}

std_msgs::UInt32MultiArray LedStripControllerTestSuite::getCurrentPattern()
{
    ros::Duration(0.25).sleep();
    return current_pattern_;
}

void LedStripControllerTestSuite::LedStripPatternCb(const std_msgs::UInt32MultiArray::ConstPtr& msg)
{
    current_pattern_.layout = msg->layout;
    current_pattern_.data = msg->data;
}

void LedStripControllerTestSuite::SetUp()
{
    initNode();
    led_pattern_sub_ = nh_.subscribe("/embedded/LED_strip", 1,
                                     &LedStripControllerTestSuite::LedStripPatternCb, this);
    send_pattern_client_ = nh_.serviceClient<phoenix_msgs::LedPattern>("set_led_pattern");

    ros::service::waitForService("set_led_pattern");
}
/*
rosservice
/set_led_pattern


rostopic
/embedded/LED_strip
*/