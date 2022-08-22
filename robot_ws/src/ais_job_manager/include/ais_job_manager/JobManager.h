/* 
    AIS CONFIDENTIAL
    Author: Michael Chiou (m.chiou@ai-systems.ca)
*/

#ifndef AIS_JOB_MANAGER_H_
#define AIS_JOB_MANAGER_H_

#include "ais_job_manager/AbstractJobManager.h"
#include "std_srvs/Trigger.h"

#include "phoenix_msgs/CreateRailMission.h"
#include "phoenix_msgs/ListOfRailMissions.h"
#include "phoenix_msgs/SendRailMission.h"
#include "phoenix_msgs/RailMissionName.h"

#include "ais_job_manager/csv.h"
#include <dirent.h>
#include <algorithm>

using std::min;
using std::max;
namespace ais_job_manager
{

class JobManager : public AbstractJobManager
{
    public:
    /**
     * @brief Constructor
    */
    JobManager(ros::NodeHandle&);
    /**
     * @brief Destructor
    */
    ~JobManager() override;
    /**
     * @brief initialize parameters that and do error checks
    */
    void initialize();

    /**
     * @brief spin and handle incoming service requests as needed
    */
    void runProcess();
    /**
     * @brief takes in RailMission type and converts to array form to output to CSV file using
     *        specified array
    */
    bool saveRailMissionToFile(const phoenix_msgs::RailMission&) override; 

    /**
     * @brief Rosservice callback to show list of available CSV files in MISSION_DIR
    */ 
    bool listAvailableRailMissionsCb(phoenix_msgs::ListOfRailMissions::Request& req, 
                                phoenix_msgs::ListOfRailMissions::Response& res);
    /**
     * @brief Rosservice callback to create mission if given a RailMission message
    */ 
    bool createRailMissionToFileCb(phoenix_msgs::CreateRailMission::Request& req, 
                                phoenix_msgs::CreateRailMission::Response& res);

    // This services are not implemented for now but are intended for Generic JobQueue and Mission
    // Handling   
    bool saveCurrentRailMissionToFileCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    bool showCurrentRailMissionNameCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res); // Trigger

    bool loadRailMissionToQueueCb(phoenix_msgs::RailMissionName::Request& req, 
                                phoenix_msgs::RailMissionName::Response& res);

    bool listQueuedRailMissionsCb(phoenix_msgs::ListOfRailMissions::Request& req, 
                                        phoenix_msgs::ListOfRailMissions::Response& res);
    
    protected:
        // vector<phoenix_msgs::RailMission> job_queue_;
        // int queue_index_;
        string map_directory_;
    
    private:
        ros::ServiceServer list_available_mission_service_;
        ros::ServiceServer create_mission_to_file_service_;
        ros::ServiceServer save_mission_to_file_service_;
        ros::ServiceServer list_current_mission_service_;
        ros::ServiceServer load_mission_to_queue_service_;
        ros::ServiceServer list_queued_missions_service_;
};


}; // End namespace ais_job_manager

#endif // AIS_JOB_MANAGER_H_