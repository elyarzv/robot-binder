/*
    AIS CONFIDENTIAL
    Author: Michael Wrock (m.wrock@ai-systems.ca)
*/

#ifndef ASM_GOAL_HANDLER_H_
#define ASM_GOAL_HANDLER_H_

#include "ros/ros.h"
#include "phoenix_msgs/RailGoal.h"
#include "phoenix_msgs/RailGoalPoint.h"
#include "phoenix_msgs/RailMission.h"

namespace ais_state_machine
{

using std::string;
using std::map;
using std::shared_ptr;
using std::make_shared;

//Should be a forward declaration:


class AbstractGoalHandler{
    public:
        AbstractGoalHandler();
        ~AbstractGoalHandler();
        /**
         * @brief virtual function to reset the mission
         * 
         */
        virtual void reset()=0;
        /**
         * @brief Adds a goal to the mission
         * 
         * @param new_goal goal to add at end of goal array
         */
        virtual void addGoal(phoenix_msgs::RailGoalPoint new_goal) = 0;
        /**
         * @brief Get the Current Goal in the mission
         * 
         * @return Goal 
         */
        virtual phoenix_msgs::RailGoalPoint getCurrentGoal() = 0;
        /**
         * @brief increments the current goal id
         * 
         * @return true if successful
         * @return false if current goal id is end of goal array
         */
        virtual bool incrementGoalCounter() = 0;
        int number_of_lamps_;

    protected:
        /**
         * @brief The goal in progress
         * 
         */
        int current_goal_id_;
        /**
         * @brief a mission consists of a vector of goals
         * 
         */
        std::vector<phoenix_msgs::RailGoalPoint> goal_array;
    private:
};
} // end ais_state_machine namespace
#endif // ASM_GOAL_HANDLER_H_