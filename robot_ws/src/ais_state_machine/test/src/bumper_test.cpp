#include "ais_state_machine/TestSuites/StateMachineTestSuite.h"


TEST_F(StateMachineTestSuite, checkFrontBumper)
{  
    //Set mission directory to whatever workspace this binary was built in
    setMissionDir();
    //Wait for state machine to start publishing it's current state
    waitForTopic(current_state_sub_);
    publishFakeFrontBumper(0); //Bumper un-pressed
    startMission();
    //Wait for robot to start treatment
    waitForState(StateTypes::RAIL_DISINFECTION);
    //Make sure robot is still in disinfection state
    ASSERT_EQ(getCurrentState(), StateTypes::RAIL_DISINFECTION);
    //example_mission.csv ends at 100m, so tell the robot we've gone this far
    publishFakeOdom(95); //Must be more than 90% as of PH-1238
    //bumper pressing
    publishFakeFrontBumper(1); //Bumper pressed during the missiong
    //wait for the robot to go in to return home state
    waitForState(StateTypes::RETURN_TO_RAIL_HOME);

    publishFakeFrontBumper(0); //Robot started to return home and the front bumper un-pressed
    //Tell the robot it's home
    publishFakeOdom(-1);
    // transition out of RETURN_TO_RAIL_HOME State
    waitForStateTransition(StateTypes::RETURN_TO_RAIL_HOME);
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
