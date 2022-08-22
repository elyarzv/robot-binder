#include "ais_cru_manager/GetHostnameTestSuite.h"

TEST_F(GetHostnameTest, GetHostname)
{
    string name;
    EXPECT_TRUE(name.empty());
    ASSERT_TRUE(getHostname(name));
    ASSERT_FALSE(name.empty());
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "GetHostnameTest");
    ROSCONSOLE_AUTOINIT;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    return ret;
}