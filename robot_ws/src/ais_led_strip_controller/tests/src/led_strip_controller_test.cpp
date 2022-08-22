#include "ais_led_strip_controller/LedStripControllerTestSuite.h"

TEST_F(LedStripControllerTestSuite, ChangeLEDStripPattern)
{

    waitForTopic(led_pattern_sub_);
    std_msgs::UInt32MultiArray data;
    data = getCurrentPattern();
    ASSERT_TRUE(data.data.empty());
    setLedPattern(0, 10, 255, 0, 0);
    data = getCurrentPattern();
    ASSERT_EQ(data.data.size(), 49);

    // Test Solid color pattern
    int color = data.data[0];
    for (const auto led : data.data) 
    {
        EXPECT_EQ(color, led);
    }

}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "LedStripControllerTestSuite");
    int ret = RUN_ALL_TESTS();
    return ret;
}