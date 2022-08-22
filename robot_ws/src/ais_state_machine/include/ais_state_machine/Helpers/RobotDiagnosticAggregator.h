/* 
    AIS CONFIDENTIAL
    Author: Michael Wrock (m.wrock@ai-systems.ca)
*/

#ifndef ASM_ROBOT_DIAGNOSTIC_AGGREGATOR_H_
#define ASM_ROBOT_DIAGNOSTIC_AGGREGATOR_H_

#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "ais_state_machine/json.h"
#include <vector>
#include <mutex>
#include <atomic>

namespace ais_state_machine
{
class RobotDiagnosticAggregator
{
    public:
        RobotDiagnosticAggregator(ros::NodeHandle&);
        ~RobotDiagnosticAggregator();
        std::string getRobotDiagnosticLevel();
        Json::Value getRobotErrorCodes();
        bool isRobotError();

    private:
        void DiagnosticCb(const diagnostic_msgs::DiagnosticArray::ConstPtr&);
        bool resetCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        ros::Subscriber phoenix_diagnostics_sub_;
        ros::ServiceServer clear_errors_service_;
        std::atomic_bool moderate_error;
        std::atomic_bool critical_error;
        std::mutex error_mutex_;
        std::mutex bool_mutex_;
        std::vector<float> errors_;

};
} // end of namespace ais_state_machine

#endif // END ASM_ROBOT_DIAGNOSTIC_AGGREGATOR_H_
