#include "ais_state_machine/Helpers/MissionGoalManager.h"

using namespace ais_state_machine;

MissionGoalManager::MissionGoalManager() 
{
    current_goal_id_ = 0;
    number_of_lamps_ = 0;
}

MissionGoalManager::~MissionGoalManager() 
{
}

void MissionGoalManager::loadMission(std::string mission_name){
    ROS_ERROR_STREAM("Loading mission: " << mission_name);
    clearMission();
    try{    
        io::CSVReader<10> in(mission_name);
        in.read_header(io::ignore_missing_column, "Position", "Lamp1", "Lamp2", "Lamp3", "Lamp4", "Lamp5", "Lamp6", "Lamp7", "Lamp8", "Speed");
        double position; double speed; float lamp[MAX_LAMP_SIZE];
        phoenix_msgs::RailGoalPoint new_goal;
        if(in.has_column("Lamp8")){
            number_of_lamps_ = 8;
        }else if(in.has_column("Lamp7")){
            number_of_lamps_ = 7;
        }else if(in.has_column("Lamp6")){
            number_of_lamps_ = 6;
        }else if(in.has_column("Lamp5")){
            number_of_lamps_ = 5;
        }else if(in.has_column("Lamp4")){
            number_of_lamps_ = 4;
        }else if(in.has_column("Lamp3")){
            number_of_lamps_ = 3;
        }else if(in.has_column("Lamp2")){
            number_of_lamps_ = 2;
        }else if(in.has_column("Lamp1")){
            number_of_lamps_ = 1;
        }else{
            ROS_ERROR("No lamp columns defined");
            number_of_lamps_ = 0;
            throw std::invalid_argument("Missing Lamp Columns");

        }
        while(in.read_row(position, lamp[0], lamp[1], lamp[2], lamp[3], lamp[4], lamp[5], lamp[6], lamp[7], speed)){
            new_goal.position = position;
            new_goal.intensity.clear();
            for(int i = 0; i < MAX_LAMP_SIZE; ++i){
                new_goal.intensity.push_back(lamp[i]);
            }
            new_goal.avg_intensity = (lamp[0] + lamp[1] + lamp[2] + lamp[3] + lamp[4] + lamp[5] + lamp[6] + lamp[7])/number_of_lamps_;
            new_goal.speed = speed;
            addGoal(new_goal);
        }
    }catch (std::exception e) {
        ROS_ERROR("Mission format incorrect");
    }
}
void MissionGoalManager::clearMission(){
    goal_array.clear();
    reset();
}
void MissionGoalManager::reset(){
    current_goal_id_ = 0;
}
float MissionGoalManager::getGoalSpeed(){
    ROS_INFO_THROTTLE(1,"goal array speed is :%f", goal_array[current_goal_id_].speed);
    if(current_goal_id_ >= goal_array.size()){
        ROS_ERROR_THROTTLE(1,"Goal array out of bounds");
        return 0.0;
    }else{
        return goal_array[current_goal_id_].speed;
    }
}
std::vector<float> MissionGoalManager::getGoalIntensity(){
    std::vector<float> intensities;
    if(current_goal_id_ >= goal_array.size()){
        ROS_DEBUG("Goal array not initialized");
        intensities.push_back(110.0);
        return intensities;
    }else{
        ROS_DEBUG_STREAM("goal array intensities are :" << goal_array[current_goal_id_].intensity.size() << " - " << goal_array[current_goal_id_].intensity[0] << " | " 
                                                                    << goal_array[current_goal_id_].intensity[1] << " | "
                                                                    << goal_array[current_goal_id_].intensity[2] << " | "
                                                                    << goal_array[current_goal_id_].intensity[3] << " | "
                                                                    << goal_array[current_goal_id_].intensity[4] << " | "
                                                                    << goal_array[current_goal_id_].intensity[5] << " | "
                                                                    << goal_array[current_goal_id_].intensity[6] << " | "
                                                                    << goal_array[current_goal_id_].intensity[7] );
        return goal_array[current_goal_id_].intensity;
    }
}
float MissionGoalManager::getAvgGoalIntensity(){
    if(current_goal_id_ >= goal_array.size()){
        ROS_DEBUG("Goal array not initialized");
        float avg = std::reduce(goal_array[goal_array.size()-1].intensity.begin(), goal_array[goal_array.size()-1].intensity.begin() + (number_of_lamps_)) / number_of_lamps_;
        return avg;
    }else{
        float avg = std::reduce(goal_array[current_goal_id_].intensity.begin(), goal_array[current_goal_id_].intensity.begin() + (number_of_lamps_)) / number_of_lamps_;
        ROS_INFO_THROTTLE(1,"Average goal array intensity is : %f", avg);
        return avg;
    }
}
void MissionGoalManager::addGoal(phoenix_msgs::RailGoalPoint new_goal){
    ROS_INFO("ADDING GOAL %ld", new_goal.intensity.size());
    goal_array.push_back(new_goal);
}
phoenix_msgs::RailGoalPoint MissionGoalManager::getCurrentGoal(){
    if(current_goal_id_ >= goal_array.size()){
        ROS_ERROR("Current goal out of bounds");
        phoenix_msgs::RailGoalPoint ret;
        return ret;
    }else{
        return goal_array[current_goal_id_];
    }
}
bool MissionGoalManager::incrementGoalCounter(){

    if (current_goal_id_ + 1 > goal_array.size()) {
        ROS_ERROR("Cannot increment Goal counter past array size!");
        return false;
    }
    ++current_goal_id_;
    ROS_INFO("goal counter is: %d <-- %d", current_goal_id_, current_goal_id_ - 1);
    return true; 
}
bool MissionGoalManager::hasCompletedAllGoals()
{   
    ROS_DEBUG("hasCompletedAllGoals :%s", (current_goal_id_ >= goal_array.size()) ? "true" : "false");
    return current_goal_id_ >= goal_array.size();
}

bool MissionGoalManager::isCompletingFinalGoal()
{   
    ROS_DEBUG("isCompletingFinalGoal :%s", (current_goal_id_ == goal_array.size() - 1) ? "true" : "false");
    return current_goal_id_ == goal_array.size() - 1;
}

int MissionGoalManager::getTotalGoalsInMission()
{
    ROS_DEBUG("getTotalGoalsInMission: %d", (int) goal_array.size());
    return (int) goal_array.size();
}

float MissionGoalManager::getTotalMissionLength()
{
    ROS_DEBUG("getTotalMissionLength: %f", goal_array[goal_array.size()-1].position);
    return goal_array[goal_array.size()-1].position;
}

int MissionGoalManager::getCurrentGoalCount()
{
    ROS_DEBUG("Current Goal Count is :%d", current_goal_id_);
    return current_goal_id_;
}
