#include "ais_state_machine/TestSuites/StateMachineTestSuite.h"


TEST_F(StateMachineTestSuite, checkLowBattery)
{   
    //Set mission directory to whatever workspace this binary was built in
    setMissionDir();
    //Wait for state machine to start publishing it's current state
    waitForTopic(current_state_sub_);
    rechargeBattery(80); //Reset robot
    // start in IDLE State
    waitForState(StateTypes::IDLE);
    ASSERT_EQ(getCurrentState(), StateTypes::IDLE);
    publishFakeSoC(26); //batteries are dead
    //Wait for robot to leave IDLE
    waitForState(StateTypes::LOW_BATTERY);
    //System shold now be in LOW_BATTERY
    ASSERT_EQ(getCurrentState(), StateTypes::LOW_BATTERY);
    rechargeBattery(81); //Reset robot
    waitForState(StateTypes::IDLE);
    //Make sure robot enters IDLE state
    ASSERT_EQ(getCurrentState(), StateTypes::IDLE);
    //Robot is ready to start a new mission
}
TEST_F(StateMachineTestSuite, checkCriticalBattery)
{   
    //Set mission directory to whatever workspace this binary was built in
    setMissionDir();
    //Wait for state machine to start publishing it's current state
    waitForTopic(current_state_sub_);
    rechargeBattery(82); //Reset robot
    // start in IDLE State
    ASSERT_EQ(getCurrentState(), StateTypes::IDLE);
    publishFakeSoC(19); //batteries are critical
    //Wait for robot to leave IDLE
    waitForStateTransition(StateTypes::IDLE);
    //System shold now be in LOW_BATTERY
    ASSERT_EQ(getCurrentState(), StateTypes::CRITICAL_BATTERY);
    rechargeBattery(83); //Reset robot
    // transition out of LOW_BATTERY State
    waitForStateTransition(StateTypes::LOW_BATTERY);
    waitForState(StateTypes::IDLE);
    //Make sure robot enters IDLE state
    ASSERT_EQ(getCurrentState(), StateTypes::IDLE);
    //Robot is ready to start a new mission
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "StateMachineTestSuite");
    int ret = RUN_ALL_TESTS();

    return ret;
}
