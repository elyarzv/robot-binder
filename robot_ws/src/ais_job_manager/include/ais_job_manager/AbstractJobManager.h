/*
    AIS CONFIDENTIAL
    Author: Michael Chiou (m.chiou@ai-systems.ca)
*/

#ifndef AIS_ABSTRACT_JOB_MANAGER_H_
#define AIS_ABSTRACT_JOB_MANAGER_H_

#include "ros/ros.h"
#include <string>
#include <vector>
#include "phoenix_msgs/RailGoal.h"
#include "phoenix_msgs/RailGoalPoint.h"
#include "phoenix_msgs/RailMission.h"
#include <fstream>
#include <iostream>


using std::ofstream;
using std::vector;
using std::string;

namespace ais_job_manager
{

class AbstractJobManager
{
    public:
    AbstractJobManager(ros::NodeHandle&);
    virtual ~AbstractJobManager() = default;
    /**
     * @brief generic save mission if given RailMission type
    */ 
    virtual bool saveRailMissionToFile(const phoenix_msgs::RailMission&) = 0;

    protected:
        ros::NodeHandle nh_;

};

};

#endif // End AIS_ABSTRACT_JOB_MANAGER_H_