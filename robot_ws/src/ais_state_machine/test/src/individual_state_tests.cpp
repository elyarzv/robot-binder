
#include "ais_state_machine/TestSuites/IndividualStateTestSuite.h"

TEST_F(IndividualStateTestSuite, RailDisinfectionCmdVelDirectionPositive) {
    // This test checks that robot moves forward during RailDisinfection when 
    // FORWARD_DIRECTION_ON_RAIL parameter is true
    // Usually for when robot is mounted on rail in forward orientation
    waitForTopic(cmd_vel_sub_);
    publishFakeUVIrradiance(613, 1.0);
    ros::param::set("/FORWARD_DIRECTION_ON_RAIL", true);
    runStateOnce();
    publishFakeOdom(0);
    transitionStateTo(StateTypes::RAIL_DISINFECTION);
    publishFakeOdom(15);
    runStateOnce();
    publishFakeOdom(30);    

    EXPECT_FALSE(getPublishedCmdVel().linear.x == 0);
    ASSERT_TRUE(getPublishedCmdVel().linear.x > 0);
}

TEST_F(IndividualStateTestSuite, RailDisinfectionCmdVelDirectionNegative) {
    // This test checks that robot moves backward during RailDisinfection when 
    // FORWARD_DIRECTION_ON_RAIL parameter is false
    // Usually for when robot is mounted on rail in reverse orientation
    waitForTopic(cmd_vel_sub_);
    publishFakeUVIrradiance(613, 1.0);
    ros::param::set("/FORWARD_DIRECTION_ON_RAIL", false);
    runStateOnce();
    publishFakeOdom(0);
    transitionStateTo(StateTypes::RAIL_DISINFECTION);
    publishFakeOdom(15);
    runStateOnce();
    publishFakeOdom(30);
    EXPECT_FALSE(getPublishedCmdVel().linear.x == 0);
    ASSERT_TRUE(getPublishedCmdVel().linear.x < 0);
}

TEST_F(IndividualStateTestSuite, ReturnToRailHomeCmdVelDirectionPositive) {
    // This test checks that robot moves backward during ReturnToRailHome when 
    // FORWARD_DIRECTION_ON_RAIL parameter is true
    // Usually for when robot is going home after reaching end of rail
    waitForTopic(cmd_vel_sub_);
    publishFakeUVIrradiance(613, 1.0);
    ros::param::set("/FORWARD_DIRECTION_ON_RAIL", true);
    runStateOnce();
    publishFakeOdom(0);
    transitionStateTo(StateTypes::RETURN_TO_RAIL_HOME);
    publishFakeOdom(15);
    runStateOnce();
    publishFakeOdom(30);
    EXPECT_FALSE(getPublishedCmdVel().linear.x == 0);
    ASSERT_TRUE(getPublishedCmdVel().linear.x < 0);
}

TEST_F(IndividualStateTestSuite, ReturnToRailHomeCmdVelDirectionNegative) {
    // This test checks that robot moves forward during ReturnToRailHome when 
    // FORWARD_DIRECTION_ON_RAIL parameter is false
    // Usually for when robot is going home after reaching end of rail
    waitForTopic(cmd_vel_sub_);
    publishFakeUVIrradiance(613, 1.0);
    ros::param::set("/FORWARD_DIRECTION_ON_RAIL", false);
    runStateOnce();
    publishFakeOdom(0);
    transitionStateTo(StateTypes::RETURN_TO_RAIL_HOME);
    publishFakeOdom(15);
    runStateOnce();
    publishFakeOdom(30);
    EXPECT_FALSE(getPublishedCmdVel().linear.x == 0);
    ASSERT_TRUE(getPublishedCmdVel().linear.x > 0);
}
//
TEST_F(IndividualStateTestSuite, RailDisinfectionCmdVelDirectionPositiveZeroUVIrradiance) {
    // This test checks that robot moves forward during RailDisinfection when stops when UV lamps 
    // are warming up 
    // FORWARD_DIRECTION_ON_RAIL parameter is true
    // Usually for when robot is mounted on rail in forward orientation
    waitForTopic(cmd_vel_sub_);
    publishFakeUVIrradiance(0.0, 0.0);
    ros::param::set("/FORWARD_DIRECTION_ON_RAIL", true);
    ros::param::set("/IRRADIANCE_SPEED_CONTROL", true);
    runStateOnce();
    publishFakeOdom(0);
    transitionStateTo(StateTypes::RAIL_DISINFECTION);
    publishFakeOdom(15);
    runStateOnce();
    publishFakeOdom(30);    
    EXPECT_NEAR(getPublishedCmdVel().linear.x, 0, 0.005);
    ASSERT_TRUE(getPublishedCmdVel().linear.x == 0);
}

TEST_F(IndividualStateTestSuite, RailDisinfectionCmdVelDirectionNegativeZeroUVIrradiance) {
    // This test checks that robot moves backward during RailDisinfection when 
    // FORWARD_DIRECTION_ON_RAIL parameter is false
    // Usually for when robot is mounted on rail in reverse orientation
    waitForTopic(cmd_vel_sub_);
    publishFakeUVIrradiance(0.0, 0.0);
    ros::param::set("/FORWARD_DIRECTION_ON_RAIL", false);
    ros::param::set("/IRRADIANCE_SPEED_CONTROL", true);
    runStateOnce();
    publishFakeOdom(0);
    transitionStateTo(StateTypes::RAIL_DISINFECTION);
    publishFakeOdom(15);
    runStateOnce();
    publishFakeOdom(30);
    EXPECT_FALSE(getPublishedCmdVel().linear.x < 0);
    ASSERT_TRUE(getPublishedCmdVel().linear.x == 0);
}

TEST_F(IndividualStateTestSuite, ReturnToRailHomeCmdVelDirectionPositiveZeroUVIrradiance) {
    // This test checks that robot moves backward during ReturnToRailHome when 
    // FORWARD_DIRECTION_ON_RAIL parameter is true
    // Usually for when robot is going home after reaching end of rail
    waitForTopic(cmd_vel_sub_);
    publishFakeUVIrradiance(0.0, 0.0);
    ros::param::set("/FORWARD_DIRECTION_ON_RAIL", true);
    ros::param::set("/RETURN_TO_RAIL_HOME_WITH_UV_ON", true);
    ros::param::set("/IRRADIANCE_SPEED_CONTROL", true);
    runStateOnce();
    publishFakeOdom(0);
    transitionStateTo(StateTypes::RETURN_TO_RAIL_HOME);
    publishFakeOdom(15);
    runStateOnce();
    publishFakeOdom(30);
    EXPECT_FALSE(getPublishedCmdVel().linear.x < 0);
    ASSERT_TRUE(getPublishedCmdVel().linear.x == 0);
}

TEST_F(IndividualStateTestSuite, ReturnToRailHomeCmdVelDirectionNegativeZeroUVIrradiance) {
    // This test checks that robot moves forward during ReturnToRailHome when 
    // FORWARD_DIRECTION_ON_RAIL parameter is false
    // Usually for when robot is going home after reaching end of rail
    waitForTopic(cmd_vel_sub_);
    publishFakeUVIrradiance(0.0, 0.0);
    ros::param::set("/FORWARD_DIRECTION_ON_RAIL", false);
    ros::param::set("/RETURN_TO_RAIL_HOME_WITH_UV_ON", true);
    ros::param::set("/IRRADIANCE_SPEED_CONTROL", true);
    runStateOnce();
    publishFakeOdom(0);
    transitionStateTo(StateTypes::RETURN_TO_RAIL_HOME);
    publishFakeOdom(15);
    runStateOnce();
    publishFakeOdom(30);
    EXPECT_FALSE(getPublishedCmdVel().linear.x < 0);
    ASSERT_TRUE(getPublishedCmdVel().linear.x == 0);
}


int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "IndividualStateTestSuite");
    int ret = RUN_ALL_TESTS();
    return ret;
}