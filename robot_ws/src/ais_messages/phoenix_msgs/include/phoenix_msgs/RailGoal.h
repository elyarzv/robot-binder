/*
    AIS CONFIDENTIAL
    Author: Michael Chiou (m.chiou@ai-systems.ca)
*/

#ifndef PH_RAIL_GOAL_POINT_H_
#define PH_RAIL_GOAL_POINT_H_

#include <string>

using std::string;

/* @brief This struct contains disinfection parameters for any individual point in a disinfection 
 *        mission
 */

struct RailGoal {

    float position;
    float intensity;
    float speed;
};

#endif // PH_RAIL_GOAL_POINT_H_