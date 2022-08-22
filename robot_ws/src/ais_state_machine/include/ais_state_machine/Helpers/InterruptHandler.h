/*
    AIS CONFIDENTIAL
    Author: Michael Wrock (m.wrock@ai-systems.ca)
*/

#ifndef ASM_INTERRUPT_H_
#define ASM_INTERRUPT_H_

#include "ros/ros.h"
#include "std_srvs/Trigger.h"

namespace ais_state_machine
{

class InterruptHandler{
    public:
        InterruptHandler(ros::NodeHandle& nh);
        ~InterruptHandler();
        /**
         * @brief check if any interrupts have been triggered
         * 
         * @return true if interrupts have been trigggered
         * @return false 
         */
        bool checkInterrupts();
        /**
         * @brief Set the Mission Aborted value
         * 
         * @param val 
         */
        void setMissionAborted(bool val);
    protected:
    private:
        ros::NodeHandle nh_;
        ros::ServiceServer abort_mission_service_;
        //This flag gets set to true when the abortCB is called, 
        // and gets set back to false (using setMissionAborted()) once runMission has aboted the mission
        bool abort_mission_;
        bool abortCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
};
} // end ais_state_machine namespace
#endif // ASM_INTERRUPT_H_