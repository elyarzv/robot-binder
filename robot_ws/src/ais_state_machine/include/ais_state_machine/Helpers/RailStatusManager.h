/*
    AIS CONFIDENTIAL
*/

#ifndef ASM_RAIL_ENCODER_MANAGER_H_
#define ASM_RAIL_ENCODER_MANAGER_H_

#define TOPIC_NOISE 10
#define UPDATE_RATE 2
#define FILTER_WINDOW 3

#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include "std_msgs/UInt8.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "geometry_msgs/Twist.h"
#include "diagnostic_msgs/KeyValue.h"
#include "ais_state_machine/Helpers/MissionConfig.h"
#include <string>
#include <vector>
#include <numeric>
#include <memory>

using std::string;

namespace ais_state_machine
{

class RailStatusManager
{
public:
    RailStatusManager(ros::NodeHandle&, string, std::shared_ptr<MissionConfig>);
    ~RailStatusManager();
    std_msgs::Int64 getEncoderCount();
    bool isEncoderUpdating();
    bool isProxyOnRails();
    void publishOnRailErrors(bool);
    void publishOffRailErrors(bool);

    void initializeMissionStatus();
    void resetMissionStatus();

    int getTargetRailNumber();
    bool isCurrentJobValid();
    void incrementRailTarget();
    bool isMissionComplete();
    int getTotalSubJobs();

    int getCurrentRailNumber() { return current_rail_number_; }
    int getCurrentJobIdx() { return current_job_idx_; }
    int getCurrentSubJobIdx() { return current_sub_job_idx_; }

    void setCurrentRailNumber(int i) { current_rail_number_ = i; }
    void setCurrentJobIdx(int i) { current_job_idx_ = i; }
    void setCurrentSubJobIdx(int i) { current_sub_job_idx_ = i; }

private:
    void encoderCb(const std_msgs::Int64ConstPtr&);
    void proxyCb(const std_msgs::UInt8ConstPtr&);
    void cmd_velCb(const geometry_msgs::TwistConstPtr&);
    void timerCb(const ros::TimerEvent& event);
    void reset();
    void publishErrorOnce(int);
    
    string topic_name_;
    ros::NodeHandle nh_;
    ros::Subscriber encoder_sub_;
    ros::Subscriber proximeter_sub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher error_code_pub_;
    ros::Timer timer_;
    ros::Time most_recent_change_;
    std_msgs::Int64 current_count_;
    std_msgs::Int64 most_recent_count_;
    geometry_msgs::Twist cmd_vel_;
    bool publish_on_rail_errors_;
    bool publish_off_rail_errors_;
    bool proxy_status_;
    diagnostic_msgs::DiagnosticArray error_code_;
    std::vector<int> proxy_data_;

    // keep track of the rail number robot is in, current rail job idx
    std::shared_ptr<MissionConfig> mission_config_ptr_;
    int current_rail_number_;
    // job = consecutive sequence of rails, sub-job = rail within a job
    int current_job_idx_;
    int current_sub_job_idx_;
};
}  // end namespace ais_state_machine

#endif  // End ASM_RAIL_ENCODER_MANAGER_H_