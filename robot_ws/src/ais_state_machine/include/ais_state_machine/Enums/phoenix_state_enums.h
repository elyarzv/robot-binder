/*
    AIS CONFIDENTIAL
    Author: Michael Chiou (m.chiou@ai-systems.ca)
*/

#ifndef ASM_STATE_MACHINE_ENUMS_H_
#define ASM_STATE_MACHINE_ENUMS_H_

/* 
 * Enum list for all states that will be used in State Machine
 */

namespace ais_state_machine
{

enum class StateTypes
{
    UNKNOWN = -1,
    IDLE = 0,
    COMPLETE = 1,
    RUN_MISSION = 2,
    INITIALIZE_ROBOT = 3,
    RAIL_DISINFECTION = 4,
    RETURN_TO_RAIL_HOME = 5,
    HALTED = 6,
    MANUAL_CONTROL = 7,
    MANUAL_TREATMENT = 8,
    TELEOPERATION = 9,
    ADJUST_TO_RAIL = 10,
    MOVE_ROBOT_OFF_RAIL = 11,
    MOVE_TO_NEXT_RAIL = 12,
    MOUNT_ROBOT_TO_RAIL = 13,
    RECOVERY = 14,
    LOW_BATTERY = 15,
    CRITICAL_BATTERY = 16,
    NAVIGATION = 17,
    FIX_ORIENTATION = 18,
};

} // end ais_state_machine namespace
#endif //end ASM_STATE_MACHINE_ENUMS_H_