#include "ais_job_manager/JobManager.h"

using namespace ais_job_manager;

JobManager::JobManager(ros::NodeHandle& nh) : AbstractJobManager(nh)
{
    map_directory_ = "";
    list_available_mission_service_ = nh.advertiseService("list_missions", &JobManager::listAvailableRailMissionsCb, this);
    create_mission_to_file_service_ = nh.advertiseService("create_rail_mission", &JobManager::createRailMissionToFileCb, this);
    //save_mission_to_file_service_ = nh.advertiseService("save_current_rail_mission", &JobManager::saveCurrentRailMissionToFileCb, this);
    //list_current_mission_service_ = nh.advertiseService("show_current_rail_mission", &JobManager::showCurrentRailMissionNameCb, this);
    //load_mission_to_queue_service_ = nh.advertiseService("add_rail_mission_to_queue", &JobManager::loadRailMissionToQueueCb, this);
    //list_queued_missions_service_ = nh.advertiseService("list_queued_rail_missions", &JobManager::listQueuedRailMissionsCb, this);

}

JobManager::~JobManager() 
{

}

void JobManager::initialize()
{ 
    ros::Rate loop_rate(1);
    while (!nh_.hasParam("/MISSION_DIR"))
    {
        ROS_WARN("Missing MISSION_DIR ROS parameter!");
        loop_rate.sleep();
        ros::spinOnce();
    }

    nh_.getParam("/MISSION_DIR", map_directory_);
    ROS_ERROR("DIRECTORY: %s", map_directory_.c_str());

}

void JobManager::runProcess()
{
    ros::Rate loop_rate(10);
    initialize();
    while (ros::ok())
    {   
        ROS_INFO_ONCE("Run Processing Job Manager");
        ros::spinOnce();
        loop_rate.sleep();
    }
}

bool JobManager::saveRailMissionToFile(const phoenix_msgs::RailMission& mission) 
{   
    string mission_name = mission.name;
    if (map_directory_.empty()) {
        ROS_ERROR("Unable to save mission to file! Map Directory is empty");
        return false;
    }
    // Check if empty mission name
    if (mission_name.empty()) 
    {
        ROS_ERROR("Unable to save mission name to file! Empty mission name");
        return false;
    } // Check if mission name ends with .csv, if it does not, try and append .csv
    // ToDo(Michael Chiou) Does not handle .csv.test... case
    else if (mission_name.size() <= 4 || 
        mission_name.find(".csv") == string::npos) 
    {   
        size_t dot_index = mission_name.find_last_of(".");
        if (dot_index == string::npos) {
            mission_name += ".csv";
        } else {
            ROS_ERROR("Unable to save mission name to file! Invalid mission name %s", mission_name.c_str());
            return false;
        }
    }
    // if mission goals array is empty 
    if (mission.goals.empty())
    {
        ROS_ERROR("Unable to save mission! Empty goal array");
        return false;
    }

    // ToDo: Fix security issue here
    string file_path = map_directory_ + mission_name;
    ofstream fout(file_path);

    ROS_DEBUG("mission size: %zu", mission.goals.size());
    ROS_DEBUG("map_directory: %s", map_directory_.c_str());
    ROS_DEBUG("mission.name %s", mission_name.c_str());
    ROS_DEBUG("file_path: %s", file_path.c_str());
    
    if (fout.is_open())
    {   
        
        fout << "Position,Lamp1,Lamp2,Lamp3,Lamp4,Lamp5,Lamp6,Lamp7,Lamp8,Speed\n";
        for (const auto& goal : mission.goals)
        {
            float goal_position = max(0.0f, goal.position);
            float goal_speed = fabs(goal.speed);
            fout << goal_position << ",";
            for(auto val : goal.intensity){
                val = max((float)0, val);
                val = min((float)100,val);
                fout << val << ",";
            }
            fout << goal_speed << "\n";
            fout.flush();
        }
        fout.close();
    } else 
    {
        ROS_WARN("fout is not open! Possible permission error");
        return false;
    }
    return true;
}

bool JobManager::listAvailableRailMissionsCb(phoenix_msgs::ListOfRailMissions::Request& req, 
                            phoenix_msgs::ListOfRailMissions::Response& res)
{
    nh_.getParam("/MISSION_DIR", map_directory_);
    if (map_directory_.empty()) {
        res.success = false;
        ROS_ERROR("Mission Directory Empty");
        return false;
    }
    auto dirp = opendir(map_directory_.c_str());
    if(dirp != NULL){
        auto dp = readdir(dirp);
        if(dp != NULL && std::string(dp->d_name).compare(0,1,".") != 0 && std::string(dp->d_name).compare(std::string(dp->d_name).size()-3,3,"csv") == 0) res.missions.push_back(std::string(dp->d_name).substr(0, std::string(dp->d_name).size()-4));
        while((dp = readdir(dirp)) != NULL){
            if(std::string(dp->d_name).compare(0,1,".") != 0 && std::string(dp->d_name).compare(std::string(dp->d_name).size()-3,3,"csv") == 0) res.missions.push_back(std::string(dp->d_name).substr(0, std::string(dp->d_name).size()-4));
        }
        (void)closedir(dirp);
        res.success = true;
        return true;
    }
    res.success = false;
    
    ROS_ERROR("Mission Directory Incorrect");
    return false;
}

bool JobManager::createRailMissionToFileCb(phoenix_msgs::CreateRailMission::Request& req, 
                            phoenix_msgs::CreateRailMission::Response& res)
{
    if (!saveRailMissionToFile(req.rail_mission))
    {
        res.success = false;
        res.message = "Error encountered writing to file";
        return false;
    }

    res.success = true;
    res.message = "File successfully written";
    return true;
}

bool JobManager::showCurrentRailMissionNameCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    return true;
}

bool JobManager::saveCurrentRailMissionToFileCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{   


    return true;
}

bool JobManager::loadRailMissionToQueueCb(phoenix_msgs::RailMissionName::Request& req, 
                                phoenix_msgs::RailMissionName::Response& res)
{
    return true;
}



bool JobManager::listQueuedRailMissionsCb(phoenix_msgs::ListOfRailMissions::Request& req, 
                            phoenix_msgs::ListOfRailMissions::Response& res)
{
    return true;
}
