/*
    AIS CONFIDENTIAL
    Author: Michael Chiou (m.chiou@ai-systems.ca)
*/

#ifndef ASM_MISSION_GOAL_MANAGER_H_
#define ASM_MISSION_GOAL_MANAGER_H_

#include "ros/ros.h"
#include "ais_state_machine/AbstractClasses/AbstractGoalHandler.h"
#include "ais_state_machine/csv.h"
#include "std_msgs/ByteMultiArray.h"
#include "phoenix_msgs/SetLampStatus.h"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <iterator>
#include <regex>
#include <string>
#include <vector>
#include <numeric>

#define MAX_LAMP_SIZE 8
namespace ais_state_machine
{

using std::string;
using std::map;
using std::shared_ptr;
using std::make_shared;

/**
 * @brief MissionGoalManager is responsible for managing the array of goal points defined in each 
 *        mission. Functionality such as loading, clearing, and tracking current disinfection points
 *        should be implemented here as well.  
 */

class MissionGoalManager : public AbstractGoalHandler{
    public:
        MissionGoalManager();
        ~MissionGoalManager();
        void loadMission(std::string mission_name);
        void clearMission();
        void reset() override;
        /**
         * @brief Get the Speed of the current goal
         * 
         * @return float m/s cmd_vel speed
         */
        float getGoalSpeed();
        /**
         * @brief Get the averageIntensity for the current goal
         * 
         * @return float Intensity percentage for all bulbs
         */
        float getAvgGoalIntensity();
        /**
         * @brief Get the Intensity for the current goal
         * 
         * @return vector of float Intensity percentage for each bulb
         */
        std::vector<float> getGoalIntensity();
        void addGoal(phoenix_msgs::RailGoalPoint new_goal) override;
        phoenix_msgs::RailGoalPoint getCurrentGoal() override;
        bool incrementGoalCounter() override;

        /**
         * @brief Get the current index of goal in array
         * 
         * @return int index of goal array
         */
        int getCurrentGoalCount();
        /**
         * @brief Return total number of goals in goal array 
         *          
         * @return int size of goal array
         */ 
        int getTotalGoalsInMission();

        /**
         * @brief boolean check if index is at final goal index in array
         *          
         * @return boolean true if index = goal_array.size - 1
         */
        bool isCompletingFinalGoal();
        
        /**
         * @brief boolean check if index is at end of goal array size in array
         *          
         * @return boolean true if index = goal_array.size()
         */
        bool hasCompletedAllGoals();
        /**
         * @brief Get the Total Mission Length
         * 
         * @return float 
         */
        float getTotalMissionLength();
};
} // end ais_state_machine namespace
#endif // ASM_MISSION_GOAL_MANAGER_H_