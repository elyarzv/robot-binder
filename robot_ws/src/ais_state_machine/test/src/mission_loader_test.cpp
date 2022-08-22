#include "gtest/gtest.h"
#include "ros/ros.h"
#include "ais_state_machine/Helpers/MissionGoalManager.h"
#include <memory>
#include <vector>
using namespace ais_state_machine;
using namespace std;
TEST(MissionLoaderTestSuite, checkUVBulbTopicInfoIsCorrect)
{   
    ros::NodeHandle nh;
    std::shared_ptr<ais_state_machine::MissionGoalManager> mission_goal_manager_;
    mission_goal_manager_ = std::make_shared<ais_state_machine::MissionGoalManager>();
    stringstream ss;
    ss << getenv("PWD") << "/../../install/share/ais_state_machine/test/lab_mission.csv";
    mission_goal_manager_->ais_state_machine::MissionGoalManager::loadMission(ss.str());
    std::vector<float> intensities = mission_goal_manager_->getGoalIntensity();
    std::vector<float> expected_intensities = {25,25,25,40,50,60,80,100};
    for(int i =0; i < intensities.size(); ++i){
        ASSERT_FLOAT_EQ(intensities[i],expected_intensities[i]) << "Unexpected intensity at " << i;
    }
    float avg_intensity = mission_goal_manager_->ais_state_machine::MissionGoalManager::getAvgGoalIntensity();
    ASSERT_FLOAT_EQ(avg_intensity,50.625) << "Average intensity is wrong";
    mission_goal_manager_->ais_state_machine::MissionGoalManager::incrementGoalCounter();
    intensities.clear();
    intensities = mission_goal_manager_->getGoalIntensity();
    expected_intensities.clear();
    expected_intensities = {100,80,75,50,25,25,25,100};
    for(int i =0; i < intensities.size(); ++i){
        ASSERT_FLOAT_EQ(intensities[i],expected_intensities[i]) << "Unexpected intensity at " << i;
    }
    avg_intensity = mission_goal_manager_->ais_state_machine::MissionGoalManager::getAvgGoalIntensity();
    ASSERT_FLOAT_EQ(avg_intensity,60) << "Average intensity is wrong";
}

TEST(MissionLoaderTestSuite, checkFileErrorHandling)
{   
    ros::NodeHandle nh;
    std::shared_ptr<ais_state_machine::MissionGoalManager> mission_goal_manager_;
    mission_goal_manager_ = std::make_shared<ais_state_machine::MissionGoalManager>();
    stringstream ss;
    ss << getenv("PWD") << "/../../install/share/ais_state_machine/test/malformed_mission.csv"; //Old mission format
    mission_goal_manager_->ais_state_machine::MissionGoalManager::loadMission(ss.str());
    ASSERT_EQ(0,mission_goal_manager_->ais_state_machine::MissionGoalManager::getTotalGoalsInMission()) << "Malformed mission is properly formed";
    ss.clear();
    ss << "/nonexistant_mission.csv"; //File should not exist
    mission_goal_manager_->ais_state_machine::MissionGoalManager::loadMission(ss.str());
    ASSERT_EQ(0,mission_goal_manager_->ais_state_machine::MissionGoalManager::getTotalGoalsInMission()) << "Nonexisant mission exists";
}

TEST(MissionLoaderTestSuite, checkAltLampConfiguration)
{   
    ros::NodeHandle nh;
    std::shared_ptr<ais_state_machine::MissionGoalManager> mission_goal_manager_;
    mission_goal_manager_ = std::make_shared<ais_state_machine::MissionGoalManager>();
    stringstream ss;
    ss << getenv("PWD") << "/../../install/share/ais_state_machine/test/four_lamp_mission.csv"; //Mission with only 4 lamps
    mission_goal_manager_->ais_state_machine::MissionGoalManager::loadMission(ss.str());
    std::vector<float> intensities = mission_goal_manager_->getGoalIntensity();
    std::vector<float> expected_intensities = {20,20,20,40};
    for(int i =0; i < mission_goal_manager_->number_of_lamps_; ++i){
        ASSERT_FLOAT_EQ(intensities[i],expected_intensities[i]) << "Unexpected intensity at " << i;
    }
    float avg_intensity = mission_goal_manager_->ais_state_machine::MissionGoalManager::getAvgGoalIntensity();
    ASSERT_FLOAT_EQ(avg_intensity,25) << "Average intensity is wrong";
    mission_goal_manager_->ais_state_machine::MissionGoalManager::incrementGoalCounter();
    intensities.clear();
    intensities = mission_goal_manager_->getGoalIntensity();
    expected_intensities.clear();
    expected_intensities = {100,80,70,50};
    for(int i =0; i < mission_goal_manager_->number_of_lamps_; ++i){
        ASSERT_FLOAT_EQ(intensities[i],expected_intensities[i]) << "Unexpected intensity at " << i;
    }
    avg_intensity = mission_goal_manager_->ais_state_machine::MissionGoalManager::getAvgGoalIntensity();
    ASSERT_FLOAT_EQ(avg_intensity,75) << "Average intensity is wrong";
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "MissionLoaderTestSuite");
  return RUN_ALL_TESTS();
}