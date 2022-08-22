#include "ais_state_machine/TestSuites/HelperTestSuite.h"

TEST_F(HelperTestSuite, checkProxyFilter)
{   
    ASSERT_EQ(getProxyStatus(), false);
    publishFakeProxy(1);
    ASSERT_EQ(getProxyStatus(), false);
    publishFakeProxy(1);
    ASSERT_EQ(getProxyStatus(), true);
    publishFakeProxy(1);
    ASSERT_EQ(getProxyStatus(), true);
    publishFakeProxy(0);
    ASSERT_EQ(getProxyStatus(), true);
    publishFakeProxy(1);
    ASSERT_EQ(getProxyStatus(), true);
    publishFakeProxy(1);
    ASSERT_EQ(getProxyStatus(), true);
    publishFakeProxy(0);
    ASSERT_EQ(getProxyStatus(), true);
    publishFakeProxy(0);
    ASSERT_EQ(getProxyStatus(), false);
    publishFakeProxy(0);
    ASSERT_EQ(getProxyStatus(), false);
    publishFakeProxy(0);
    ASSERT_EQ(getProxyStatus(), false);
    publishFakeProxy(1);
    ASSERT_EQ(getProxyStatus(), false);
    publishFakeProxy(0);
    ASSERT_EQ(getProxyStatus(), false);
    publishFakeProxy(0);
    ASSERT_EQ(getProxyStatus(), false);
    publishFakeProxy(1);
    ASSERT_EQ(getProxyStatus(), false);
    publishFakeProxy(1);
    ASSERT_EQ(getProxyStatus(), true);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "HelperTestSuite");
    int ret = RUN_ALL_TESTS();

    return ret;
}
