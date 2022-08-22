#include "ais_state_machine/Helpers/RailStatusManager.h"

using namespace ais_state_machine;

RailStatusManager::RailStatusManager(
    ros::NodeHandle& nh, string rail_encoder_topic_name,
    std::shared_ptr<MissionConfig> mission_config_ptr
) : nh_(nh), topic_name_(rail_encoder_topic_name),
    mission_config_ptr_(mission_config_ptr),
    current_rail_number_(-1)
{
    assert(mission_config_ptr_);
    encoder_sub_ = nh.subscribe(topic_name_.c_str(), 1, &RailStatusManager::encoderCb, this);
    proximeter_sub_ = nh.subscribe("embedded/proximiterstatus", 1, &RailStatusManager::proxyCb, this);
    cmd_vel_sub_ = nh.subscribe("embedded/cmd_vel", 5, &RailStatusManager::cmd_velCb, this);
    error_code_pub_ = nh.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 1);
    timer_ = nh.createTimer(ros::Duration(1), &RailStatusManager::timerCb, this);
    most_recent_change_ = ros::Time::now();
    diagnostic_msgs::KeyValue keyvalue;
    keyvalue.key = "error_code";
    keyvalue.value = "-1";
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "rail_status_diagnostic";
    status.values.push_back(keyvalue);
    error_code_.status.push_back(status);
    publish_on_rail_errors_ = false;
    publish_off_rail_errors_ = false;
    for(int ctr =0; ctr < FILTER_WINDOW; ++ctr ) proxy_data_.push_back(0);
    proxy_status_ = false;
}

RailStatusManager::~RailStatusManager()
{

}

void RailStatusManager::reset(){
    most_recent_change_ = ros::Time::now();
    most_recent_count_ = current_count_;
}

void RailStatusManager::initializeMissionStatus() {
    current_rail_number_ = mission_config_ptr_->start_rail_number;
    current_job_idx_ = 0;
    current_sub_job_idx_ = 0;
}

void RailStatusManager::resetMissionStatus() {
    current_rail_number_ = -1;
    current_job_idx_ = 0;
    current_sub_job_idx_ = 0;
}

bool RailStatusManager::isCurrentJobValid()
{
    const auto jobs = mission_config_ptr_->rail_jobs;

    if (current_job_idx_ >= 0 && current_job_idx_ < jobs.size()) {
        const auto start_rail = jobs[current_job_idx_][0];
        const auto end_rail = jobs[current_job_idx_][1];

        if (current_sub_job_idx_ >= 0
                && current_sub_job_idx_ <= std::abs(end_rail - start_rail)) {
        } else {
            ROS_ERROR("Invalid sub-job idx %d", current_sub_job_idx_);
            return false;
        }
    } else {
        ROS_ERROR("Invalid job idx %d", current_job_idx_);
        return false;
    }
    return true;
}

int RailStatusManager::getTargetRailNumber() {
    if (!isCurrentJobValid()) {
        return -1;
    }

    const auto jobs = mission_config_ptr_->rail_jobs;
    const auto start_rail = jobs[current_job_idx_][0];
    const auto end_rail = jobs[current_job_idx_][1];

    if (end_rail > start_rail) {
        // moving left
        return start_rail + current_sub_job_idx_;
    } else {
        // moving right
        return start_rail - current_sub_job_idx_;
    }
}

void RailStatusManager::incrementRailTarget() {
    if (!isCurrentJobValid()) {
        return;
    }

    ROS_INFO("Old job_idx: %d, sub_job_idx: %d", current_job_idx_, current_sub_job_idx_);

    const auto jobs = mission_config_ptr_->rail_jobs;
    const auto start_rail = jobs[current_job_idx_][0];
    const auto end_rail = jobs[current_job_idx_][1];

    const auto num_sub_jobs = std::abs(end_rail - start_rail) + 1;
    if (current_sub_job_idx_ < (num_sub_jobs - 1)) {
        // sub-job not finished, move to the next rail in the sub-job
        ++current_sub_job_idx_;
    } else {
        // sub-job finished, move to the next job (row)
        current_sub_job_idx_ = 0;
        ++current_job_idx_;
    }
    ROS_INFO("New job_idx: %d, sub_job_idx: %d", current_job_idx_, current_sub_job_idx_);
}

bool RailStatusManager::isMissionComplete() {
    const auto jobs = mission_config_ptr_->rail_jobs;
    return current_job_idx_ >= jobs.size();
}

int RailStatusManager::getTotalSubJobs() {
    if (!isCurrentJobValid()) {
        return -1;
    }

    const auto jobs = mission_config_ptr_->rail_jobs;
    const auto start_rail = jobs[current_job_idx_][0];
    const auto end_rail = jobs[current_job_idx_][1];
    return std::abs(end_rail - start_rail) + 1;
}

void RailStatusManager::publishErrorOnce(int code){
    error_code_.status[0].values[0].value = std::to_string(code);
    error_code_pub_.publish(error_code_);
    ros::spinOnce();
}

void RailStatusManager::publishOnRailErrors(bool val){
    publish_on_rail_errors_ = val;
    if(val){
        reset();
    }
}

void RailStatusManager::publishOffRailErrors(bool val){
    publish_off_rail_errors_ = val;
    if(val){
        reset();
    }
}

std_msgs::Int64 RailStatusManager::getEncoderCount(){
    return current_count_;
}

bool RailStatusManager::isEncoderUpdating(){
    if(ros::Time::now().toSec() - most_recent_change_.toSec() > UPDATE_RATE){
        return false;
    }else{
        return true;
    }
}

bool RailStatusManager::isProxyOnRails(){
    return proxy_status_;
}


void RailStatusManager::encoderCb(const std_msgs::Int64ConstPtr& msg)
{
    if(fabs(msg->data - most_recent_count_.data) >  TOPIC_NOISE){
        most_recent_change_ = ros::Time::now();
        most_recent_count_ = *msg;
    }
    current_count_ = *msg;
}
void RailStatusManager::proxyCb(const std_msgs::UInt8ConstPtr& msg)
{
    //Apply low pass filtering to ignore single outliers over a FILTER_WINDOW (3) message window
    assert(!proxy_data_.empty());
    proxy_data_.erase(proxy_data_.begin());
    proxy_data_.push_back(msg->data);
    int sum = std::accumulate(proxy_data_.begin(), proxy_data_.end(), 0);
    if(sum >= (FILTER_WINDOW - 1)){
        proxy_status_ = true;
    }else if(sum <= 1){
        proxy_status_ = false;
    }
}
void RailStatusManager::cmd_velCb(const geometry_msgs::TwistConstPtr& msg){
    cmd_vel_ = *msg;
}
void RailStatusManager::timerCb(const ros::TimerEvent& event)
{
    //ROS_INFO("Checking status %d, %d", isEncoderUpdating(), isProxyOnRails());
    if(publish_on_rail_errors_){ //Only publish these error codes when robot is on rails
        //Check if encoder stopped updating and proxy is on rail
        if(!isEncoderUpdating() && isProxyOnRails()){
            if(fabs(cmd_vel_.linear.x) > 0.05){
                ROS_ERROR("Encoder not updating while robot on rails");
                publishErrorOnce(210015);
            }else{
                most_recent_change_ = ros::Time::now();
                ROS_WARN("Encoder not updating but robot is not moving");
            }
        }//Check if proxy says off rail while encoder is updating
        else if(isEncoderUpdating() && !isProxyOnRails()){
            ROS_ERROR("Proxy not registering while robot on rails");
            publishErrorOnce(110016);
        }//Check if proxy and encoder says the robot is off rails
        else if(!isEncoderUpdating() && !isProxyOnRails()){
            ROS_ERROR("Proxy and encoder not registering while robot on rails");
            publishErrorOnce(210017);
        }
    }

    if(publish_off_rail_errors_){ //Only publish these error codes when robot is off rails
        //Check if encoder stopped updating and proxy is on rail
        if(!isEncoderUpdating() && isProxyOnRails()){
            ROS_ERROR("Proxy sensor triggered while off rails");
            publishErrorOnce(210018);
        }
    }
}