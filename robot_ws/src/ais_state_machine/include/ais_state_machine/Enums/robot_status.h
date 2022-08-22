/*
    AIS CONFIDENTIAL
    Author: Michael Wrock (m.wrock@ai-systems.ca)
*/

#ifndef ASM_ROBOT_STATUS_ENUMS_H_
#define ASM_ROBOT_STATUS_ENUMS_H_

/* 
 * Enum list for all states that will be used in State Machine
 */

namespace ais_state_machine
{

enum class RobotStatus
{
    COMPLETE = 1, //Complete state
    IDLE = 2, //Idle state
    MANUAL_OPERATION = 3, //Pendant Control
    MANUAL_TREATMENT = 4, //Semi autonomous
    RETURN_TO_RAIL_HOME = 5, //Returning to home state
    OTHER = 6, //Generic handler for unknown states
    TELEOP = 7, //Robot is being teleoperated (Current the idle state is considered a teleop state)
    AUTO = 8, //For all autonomous states
    HALTED = 9, //For when the robot must halt for safety
    WAITING = 10, //For when the robot uses USE_STOP_AS_UV, robot is in WAITING state for ROBOT_GOAL_STOP_TIME
    MOUNTING = 11, //For when the robot is mounting or dismounting the rail
    NAVIGATING = 12, //For when the robot is navigating to the next rail
    RECOVERY = 13, //For when the robot needs a teleoperator
    ADJUSTING = 14, //Robot is adjusting position to match rail head
    LOW_BATTERY = 15, //Battery is too low to start a mission
    CRITICAL_BATTERY = 16, //Battery is too low to operate
    NAVIGATION = 17, //Navigation is for navigating using nav stack
    FIX_ORIENTATION = 18, // For fixing orientation
};

} // end ais_state_machine namespace
#endif //end ASM_ROBOT_STATUS_ENUMS_H_