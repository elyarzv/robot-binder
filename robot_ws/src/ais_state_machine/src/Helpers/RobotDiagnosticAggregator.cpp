#include "ais_state_machine/Helpers/RobotDiagnosticAggregator.h"

using namespace ais_state_machine;

RobotDiagnosticAggregator::RobotDiagnosticAggregator(ros::NodeHandle& nh)
{
    phoenix_diagnostics_sub_ = nh.subscribe("diagnostics", 5, &RobotDiagnosticAggregator::DiagnosticCb, this);
    clear_errors_service_ = nh.advertiseService("phoenix_diagnostics/clear", &RobotDiagnosticAggregator::resetCB, this);
    bool_mutex_.lock();
    moderate_error = false;
    critical_error = false;
    bool_mutex_.unlock();
}

RobotDiagnosticAggregator::~RobotDiagnosticAggregator() 
{
}

bool RobotDiagnosticAggregator::resetCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){

    try{
        error_mutex_.lock();
        if(errors_.size()) errors_.clear();
        error_mutex_.unlock();
    }catch (...){
        ROS_ERROR("Unknown problem clearing errors");
    }
    bool_mutex_.lock();
    moderate_error = false;
    critical_error = false;
    bool_mutex_.unlock();

    return true;

}
bool RobotDiagnosticAggregator::isRobotError(){

    bool_mutex_.lock();
    bool retval = critical_error || moderate_error;
    bool_mutex_.unlock();
    return retval;

}

void RobotDiagnosticAggregator::DiagnosticCb(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg){
    float error_code;
    error_mutex_.lock();
    for(auto statuses : msg->status)
        for(auto msgs : statuses.values){
            if(msgs.key.compare("error_code") == 0){
                error_code = std::stof(msgs.value);
                if(error_code > 100000){
                    errors_.push_back(error_code);
                    ROS_ERROR("Topic %s not publishing, error code %f",statuses.name.c_str(),error_code);
                    bool_mutex_.lock();
                    if(error_code > 200000){
                        critical_error = true;
                    }else{
                        moderate_error = true;
                    }
                    bool_mutex_.unlock();
                }
            }
        }
    // sort followed by unique, to remove all duplicates
    if(errors_.size()){
        std::sort(errors_.begin(), errors_.end());
        auto last = std::unique(errors_.begin(), errors_.end());
        errors_.erase(last, errors_.end());
    }
    error_mutex_.unlock();

}
std::string RobotDiagnosticAggregator::getRobotDiagnosticLevel(){
    std::string retval;
    bool_mutex_.lock();
    if(critical_error){
        retval = "CRITICAL";
    }else if(moderate_error){
        retval = "MODERATE";
    }else{
        retval = "OK";
    }
    bool_mutex_.unlock();
    return retval;
}
Json::Value RobotDiagnosticAggregator::getRobotErrorCodes(){
    Json::Value error_codes;
    error_mutex_.lock();
    for(auto iter : errors_){
        error_codes.append(iter);
    }
    error_mutex_.unlock();
    return error_codes;
}