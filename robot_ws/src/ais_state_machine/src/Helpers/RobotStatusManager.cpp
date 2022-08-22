#include "ais_state_machine/Helpers/RobotStatusManager.h"

using namespace ais_state_machine;

RobotStatusManager::RobotStatusManager(ros::NodeHandle& nh,ros::NodeHandle& private_nh, shared_ptr<MissionGoalManager> mission_goal_manager_,shared_ptr<RobotBumperHandler> bumper_handler_ptr_,shared_ptr<RailStatusManager> rail_status_ptr_, shared_ptr<UVLampController> uv_lamp_controller_ptr_) : nh_(nh), mission_goal_manager_(mission_goal_manager_), bumper_handler_ptr_(bumper_handler_ptr_), rail_status_ptr_(rail_status_ptr_), private_nh_(private_nh), uv_lamp_controller_ptr_(uv_lamp_controller_ptr_)
{
    last_time_dimmer_pub_ = 0;
    std::string reset_topic;
    //This should get the namespace of amr_localization amr_localization_1d_node
    private_nh_.param<std::string>("localization_source", localization_source, std::string("encoder_localization_node"));
    reset_topic = localization_source + "/reset";

    reset_pose_service_ = nh.serviceClient<std_srvs::Trigger>(reset_topic);
    std::string pose_topic = localization_source + "/pose_1_d";
    uv_dimmer_sub_ = nh.subscribe("embedded/uv_lamp_status", 5, &RobotStatusManager::DimmerCb, this);
    dimmer_cmd_sub_ = nh.subscribe("embedded/uv_lamp_dimmer_values", 5, &RobotStatusManager::DimmerCmdCb, this);
    aux_power_sub_ = nh.subscribe("embedded/aux_power/status", 5, &RobotStatusManager::AuxCb, this);
    safety_status_sub_ =nh.subscribe("safety_status", 1, &RobotStatusManager::SafetyStatusCb, this);
    battery_soc_sub_ = nh.subscribe("embedded/battery/state_of_charge", 5, &RobotStatusManager::SoCCb, this);
    robot_status_pub_ = nh.advertise<std_msgs::String>("/autonomy/robot_status", 1000);
    diagnostic_error_pub_ = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1000);
    wheel_speed_sub_ = nh.subscribe("embedded/wheelodom", 5, &RobotStatusManager::WheelodomCb, this);
    roller_cmd_vel_sub_ = nh.subscribe("embedded/roller_cmd_vel", 1, &RobotStatusManager::roller_cmd_velCb, this);
    manual_cmd_vel_sub_ = nh.subscribe("embedded/manual_cmd_vel", 1, &RobotStatusManager::manual_cmd_velCb, this);
    teleop_cmd_vel_sub_ = nh.subscribe("embedded/teleop_cmd_vel", 1, &RobotStatusManager::teleop_cmd_velCb, this);
    navigation_cmd_vel_sub_ = nh.subscribe("embedded/navigation_cmd_vel", 1, &RobotStatusManager::navigation_cmd_velCb, this);  
    pose_subscriber_ = nh.subscribe(pose_topic, 1, &RobotStatusManager::poseCb, this);
    led_strip_ = nh.serviceClient<phoenix_msgs::LedPattern>("set_led_pattern");
    abort_mission_service_ = nh.advertiseService("stop_mission", &RobotStatusManager::abortCB, this);
    robot_diagnostic_ = make_shared<RobotDiagnosticAggregator>(nh);
    abort_mission_ = false;
    dimmer_time_tol_ = 0.3; //The topic piblishes at 10Hz, this allows for a third of the messages to drop before assuming the dimmer module has died
    prev_robot_rail_pose_.data = 0;
    aux_status_ = false;
    errors_cleared_ = false;
    estop_pressed_ = false;
    wheel_speed_ = {0};
    cmd_vel_ = {0};
    encoder_speed_ = {0};
    rail_status_known = false;
    off_rail_detection_autonomous = false;
    off_rail_detection = false;
    off_rail_detection_semi_autonomous = false;
    battery_soc_ = 100;
    diagnostic_msgs::KeyValue keyvalue;
    keyvalue.key = "error_code";
    keyvalue.value = "-1";
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "on_rail_bumper_diagnostic";
    status.values.push_back(keyvalue);
    error_code_.status.push_back(status);
}

RobotStatusManager::~RobotStatusManager() 
{
}
void RobotStatusManager::getUpdatedParams(){
    nh_.param<bool>("OFF_RAIL_DETECTION_AUTONOMOUS", off_rail_detection_autonomous, false);
    nh_.param<bool>("OFF_RAIL_DETECTION_SEMI_AUTONOMOUS", off_rail_detection_semi_autonomous, false);
    nh_.param<bool>("OFF_RAIL_DETECTION", off_rail_detection, false);
}
float RobotStatusManager::getSoC(){
    return battery_soc_;
}
bool RobotStatusManager::interruptStateMachine(){
    return abort_mission_ || !isRobotOnRails() || isEStopPressed() || lowRobotBattery() || isBatteryCritical() || !(robot_diagnostic_->getRobotDiagnosticLevel().compare("OK") == 0 );
}

bool RobotStatusManager::abortCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){

    abort_mission_ = true;
    if(robot_status_ == RobotStatus::AUTO){
        res.success = true; //Only respond mission successfully aborted if an autonomous mission is running
    }else{
        res.success = false;
    }
    return true;

}
bool RobotStatusManager::abortMission(){

    return abort_mission_;

}
void RobotStatusManager::setMissionAbort(bool val){

    abort_mission_ = val;

}
void RobotStatusManager::setRobotOnRails(bool val){

    rail_status_known = val;

}
void RobotStatusManager::setMissionStartTime(ros::Time start_time){
    mission_start_time_ = start_time;
}

float RobotStatusManager::getTimeRemainingForStart(){
    return round(mission_start_time_.toSec() - ros::Time::now().toSec());
}
bool RobotStatusManager::isEStopPressed(){

    if(robot_status_ == RobotStatus::AUTO || robot_status_ == RobotStatus::RETURN_TO_RAIL_HOME){
        return estop_pressed_;
    }else{
        return false;
    }

}
bool RobotStatusManager::isRobotOutOfTheWay(ais_state_machine::StateTypes current_state){

    switch (current_state)
    {
        case ais_state_machine::StateTypes::RAIL_DISINFECTION:
        case ais_state_machine::StateTypes::RETURN_TO_RAIL_HOME:
        case ais_state_machine::StateTypes::ADJUST_TO_RAIL:
        case ais_state_machine::StateTypes::MOUNT_ROBOT_TO_RAIL:
        case ais_state_machine::StateTypes::MOVE_TO_NEXT_RAIL:
        case ais_state_machine::StateTypes::MANUAL_CONTROL:
            return false;
            break;
        case ais_state_machine::StateTypes::MOVE_ROBOT_OFF_RAIL:
            return rail_status_ptr_->isProxyOnRails();
            break;
        default:
            return true;
            break;
    }

}

bool RobotStatusManager::lowRobotBattery(){

    float low_battery, estimated_battery;
    nh_.getParamCached("LOW_BATTERY", low_battery);
    nh_.getParamCached("ESTIMATED_BATTERY_USAGE", estimated_battery);
    float battery_required = low_battery + estimated_battery;
    if(battery_soc_ < battery_required){
        ROS_WARN_THROTTLE(1,"Robot battery is low, do not continue");
        return true;
    }else{
        return false;
    }

}
bool RobotStatusManager::batteryRecharged(){

    float min_battery;
    nh_.getParamCached("MINIMUM_BATTERY", min_battery);
    if(battery_soc_ > min_battery){
        ROS_INFO_THROTTLE(1,"Robot battery is recharged %f/%f", battery_soc_, min_battery);
        return true;
    }else{
        ROS_INFO_THROTTLE(1,"Robot battery is not recharged %f/%f", battery_soc_, min_battery);
        return false;
    }

}

bool RobotStatusManager::isBatteryCritical(){

    float crit_battery;
    nh_.getParamCached("CRITICAL_BATTERY", crit_battery);
    if(battery_soc_ < crit_battery){
        ROS_ERROR_THROTTLE(1,"Robot battery is critical, will not continue");
        return true;
    }else{
        return false;
    }

}

bool RobotStatusManager::isDimmerAlive(){

    double time_diff = ros::Time::now().toSec() - last_time_dimmer_pub_; 
    if (time_diff >= dimmer_time_tol_) ROS_WARN_THROTTLE(2,"UV dimmer not publishing!");

    return (time_diff < dimmer_time_tol_);
}

float average(std::vector<float> const& v){
    if(v.empty()){
        return 0;
    }

    auto const count = static_cast<float>(v.size());
    return std::reduce(v.begin(), v.end()) / count;
}

bool RobotStatusManager::isRobotOnRails(){
    float avg_wheel_speed = average(wheel_speed_);
    float avg_encoder_speed = average(encoder_speed_);
    float avg_cmd_vel = average(cmd_vel_);
    ROS_DEBUG("Wheel speed %f Encoder speed %f",avg_wheel_speed,avg_encoder_speed);
    if((avg_encoder_speed/fabs(avg_encoder_speed)) != (avg_wheel_speed/fabs(avg_wheel_speed))){
        ROS_DEBUG("Robot is switching directions");
        return true;
    }else{
        if(fabs(avg_encoder_speed) < MIN_ENCODER_SPEED && fabs(avg_wheel_speed) > MIN_WHEEL_SPEED && rail_status_known){
            ROS_WARN("Robot is off the rails! %f - %f ",avg_encoder_speed,avg_wheel_speed);
            return false; //Feature flag: Change to true to disable feature
        }else if(fabs(avg_encoder_speed) > MIN_ENCODER_SPEED && fabs(avg_wheel_speed) > MIN_WHEEL_SPEED){
            ROS_DEBUG("Robot is ON rails!");
            rail_status_known = true;
            return true;
        }else{
            rail_status_known  = false;
            return true;
        }
    }
}

void RobotStatusManager::setErrorCode(std::string code)
{   
    error_code_.status[0].values[0].value = code;
    diagnostic_error_pub_.publish(error_code_);
}
void RobotStatusManager::DimmerCb(const std_msgs::UInt16::ConstPtr& msg)
{   
    last_time_dimmer_pub_ = ros::Time::now().toSec();
    if(last_time_dimmer_pub_ - last_time_dimmer_cmd_pub_ > MIN_DIMMER_ADJUST_TIME){
        if(curr_uv_cmd_byte_.data != msg->data && uv_lamp_controller_ptr_->isUVOn()){
            ROS_ERROR("Dimmer bulb malfunction detected: command %d != dimmer value %d",curr_uv_cmd_byte_, msg->data);
            setErrorCode("230022");
        }
    }
}
void RobotStatusManager::DimmerCmdCb(const std_msgs::ByteMultiArray::ConstPtr& msg)
{   
    if(curr_uv_cmd_.data != msg->data){
        last_time_dimmer_cmd_pub_ = ros::Time::now().toSec();
        curr_uv_cmd_ = *msg;
        curr_uv_cmd_byte_.data = 128*(bool)msg->data[7] + 64*(bool)msg->data[6] + 32*(bool)msg->data[5] + 16*(bool)msg->data[4] + 8*(bool)msg->data[3] + 4*(bool)msg->data[2] + 2*(bool)msg->data[1] + (bool)msg->data[0];
    }
}
void RobotStatusManager::SafetyStatusCb(const std_msgs::String::ConstPtr& msg)
{   
    safety_status = msg->data;
}
void RobotStatusManager::AuxCb(const std_msgs::Bool::ConstPtr& msg)
{   
    if(aux_status_ == true){
        estop_pressed_ = false;
        if(msg->data == false){
            ROS_WARN("AUX power shut off, possible E-Stop condition");
            estop_pressed_ = true;
        }
    }
    aux_status_ = msg->data;
}
void RobotStatusManager::SoCCb(const std_msgs::UInt8::ConstPtr& msg)
{   
    battery_soc_ = (float) msg->data;
}
void RobotStatusManager::WheelodomCb(const nav_msgs::OdometryConstPtr& msg){
    wheel_speed_.push_back(msg->twist.twist.linear.x);
    wheel_speed_.erase(wheel_speed_.begin());
}
void RobotStatusManager::cmd_velCb(const geometry_msgs::TwistConstPtr& msg){
    cmd_vel_.push_back(msg->linear.x);
    cmd_vel_.erase(cmd_vel_.begin());
}
void RobotStatusManager::manual_cmd_velCb(const geometry_msgs::TwistConstPtr& msg){
    manual_cmd_vel_ = *msg;
    last_manual_message_ = ros::Time::now();
}
void RobotStatusManager::teleop_cmd_velCb(const geometry_msgs::TwistConstPtr& msg){
    teleop_cmd_vel_ = *msg;
    last_teleop_message_ = ros::Time::now();
}
void RobotStatusManager::navigation_cmd_velCb(const geometry_msgs::TwistConstPtr& msg){
    navigation_cmd_vel_ = *msg;
    last_navigation_cmd_vel_message_ = ros::Time::now();
}
void RobotStatusManager::roller_cmd_velCb(const geometry_msgs::TwistConstPtr& msg){
    roller_cmd_vel_.linear.x = msg->linear.x*1.71621621622;  //Ratio between roller wheel diameter and meccanum wheel diameter
    last_roller_message_ = ros::Time::now();
}

geometry_msgs::Twist RobotStatusManager::getManualCmdVel(){
    if(ros::Time::now().toSec() - last_manual_message_.toSec() > 1){
        geometry_msgs::Twist ret;
        return ret;
    }else{
        return manual_cmd_vel_;
    }
}
geometry_msgs::Twist RobotStatusManager::getTeleopCmdVel(){
    if(ros::Time::now().toSec() - last_teleop_message_.toSec() > 1){
        geometry_msgs::Twist ret;
        return ret;
    }else{
        return teleop_cmd_vel_;
    }
}
geometry_msgs::Twist RobotStatusManager::getRollerCmdVel(){
    if(ros::Time::now().toSec() - last_roller_message_.toSec() > 1){
        geometry_msgs::Twist ret;
        return ret;
    }else{
        return roller_cmd_vel_;
    }
}

geometry_msgs::Twist RobotStatusManager::getNavigationCmdVel(){
    if(ros::Time::now().toSec() - last_navigation_cmd_vel_message_.toSec() > 1){
        geometry_msgs::Twist ret;
        return ret;
    }else{
        return navigation_cmd_vel_;
    }
}

void RobotStatusManager::poseCb(const std_msgs::Float32ConstPtr &pose)
{
    ros::Time now = ros::Time::now();
    ros::Duration time_step = now - last_pose_message_;
    if(time_step.toSec() > SPEED_CALCULATION_FREQUENCY){
        encoder_speed_.push_back((pose->data- prev_robot_rail_pose_.data)/(time_step.toSec()));
        encoder_speed_.erase(encoder_speed_.begin());
        last_pose_message_ = now;
        prev_robot_rail_pose_ = *pose;
    }
    curr_robot_rail_pose_ = *pose;
}
void RobotStatusManager::setRobotStatus(RobotStatus status){

    robot_status_ = status;
}
bool RobotStatusManager::willRobotCollide(){

    if (safety_status.compare("COLLISION") == 0 ){
        return true;
    }else{
        return false;
    }
}
std::string RobotStatusManager::getRobotStatusString(){

    switch (robot_status_)
    {
    case RobotStatus::IDLE:
        return "IDLE";
        break;
    case RobotStatus::COMPLETE:
        return "COMPLETE";
        break;
    case RobotStatus::MANUAL_TREATMENT:
        return "SEMI-AUTO";
    case RobotStatus::RETURN_TO_RAIL_HOME:
    case RobotStatus::AUTO:
    case RobotStatus::ADJUSTING:
    case RobotStatus::MOUNTING:
    case RobotStatus::NAVIGATING:
        return "AUTO";
        break;
    case RobotStatus::WAITING:
        return "WAITING";
        break;
    case RobotStatus::TELEOP:
        return "TELEOPERATION";
        break;
    case RobotStatus::MANUAL_OPERATION:
        return "MANUAL";
        break;
    case RobotStatus::HALTED:
        return "HALTED";
        break;
    case RobotStatus::LOW_BATTERY:
        return "LOW_BATTERY";
        break;
    case RobotStatus::CRITICAL_BATTERY:
        return "CRITICAL_BATTERY";
        break;
    case RobotStatus::RECOVERY:
        return "RECOVERY";
        break;
    default:
        return "UNKNOWN";
        break;
    }
}
std::string RobotStatusManager::getMissionStatusString(){

    switch (robot_status_)
    {
    case RobotStatus::IDLE:
    case RobotStatus::COMPLETE:
    case RobotStatus::HALTED:
    case RobotStatus::MANUAL_OPERATION:
        return "STOPPED";
        break;
    case RobotStatus::TELEOP:
    case RobotStatus::WAITING:
    case RobotStatus::RECOVERY:
    case RobotStatus::LOW_BATTERY:
    case RobotStatus::CRITICAL_BATTERY:
        return "PAUSED";
        break;
    case RobotStatus::MANUAL_TREATMENT:
        return "";
        break;
    case RobotStatus::RETURN_TO_RAIL_HOME:
    case RobotStatus::AUTO:
    case RobotStatus::ADJUSTING:
    case RobotStatus::MOUNTING:
    case RobotStatus::NAVIGATING:
        return "RUNNING";
        break;
    default:
        return "UNKNOWN";
        break;
    }
}

RobotStatus RobotStatusManager::getRobotStatus(){

    return robot_status_;
}
void RobotStatusManager::update()
{   

    Json::FastWriter fastWriter;
    Json::Value newValue;
    newValue["robot_status"] = getRobotStatusString();
    newValue["mission_status"] = getMissionStatusString();
    newValue["diagnostics"] = robot_diagnostic_->getRobotDiagnosticLevel(); //OK or NOT_OK
    newValue["error_codes"] = robot_diagnostic_->getRobotErrorCodes(); //[0000, 0001, 0009];
    int row_total_,row_ctr_;
    std::string mission_name_;
    nh_.getParamCached("/MISSION_NAME", mission_name_);
    nh_.getParamCached("/NUMBER_OF_RAIL_MISSIONS", row_total_);
    nh_.getParamCached("/RAIL_MISSION_COUNTER", row_ctr_);
    newValue["row_total"] = std::to_string(row_total_);
    newValue["row_counter"] = std::to_string(row_ctr_);
    newValue["mission_name"] = mission_name_;
    newValue["time_remaining"] =  getTimeRemainingForStart();
    std_msgs::String status_string_;
    status_string_.data = fastWriter.write(newValue).c_str();
    robot_status_pub_.publish(status_string_);
    phoenix_msgs::LedPattern pattern;
    pattern.request.size = 49;
    pattern.request.use_RGB = true;
    pattern.request.fill_percent = 100;
    if(!aux_status_){ //AUX power off
        errors_cleared_ = false;
        pattern.request.pattern = pattern.request.SOLID;
        pattern.request.rate = 1;
        pattern.request.red = 255;
        pattern.request.blue = 255;
        pattern.request.green = 0;
        led_strip_.call(pattern);
    }else if(!isDimmerAlive()){ //Dimmer teensy stopped publishing
        errors_cleared_ = false;
        pattern.request.pattern = pattern.request.BLINKING;
        pattern.request.rate = 2;
        pattern.request.red = 255;
        pattern.request.blue = 0;
        pattern.request.green = 100;
        led_strip_.call(pattern);
    }else if(bumper_handler_ptr_->isBackBumperHit() || bumper_handler_ptr_->isFrontBumperHit()){ //Front or back bumper hit
        errors_cleared_ = false;
        pattern.request.pattern = pattern.request.BLINKING;
        pattern.request.rate = 2;
        pattern.request.red = 255;
        pattern.request.blue = 0;
        pattern.request.green = 0;
        led_strip_.call(pattern);
    }else if(prev_robot_status_ != robot_status_ || errors_cleared_ == false || robot_status_ == RobotStatus::AUTO ){ //Dont keep updating led pattern is status hasn't changed or you will interrupt the animation
        errors_cleared_ = true; //Set a flag so that we update our pattern if the robot status hasn't changed but the errors have been cleared
        //Status updates happen here
        if(robot_status_ == RobotStatus::MANUAL_OPERATION){ //Manual operation mode 
            pattern.request.pattern = pattern.request.SOLID;
            pattern.request.rate = 1;
            pattern.request.red = 255;
            pattern.request.blue = 0;
            pattern.request.green = 150;
            led_strip_.call(pattern);
        }else if(robot_status_ == RobotStatus::COMPLETE || robot_status_ == RobotStatus::IDLE){ //Complete and IDLE mode 
            pattern.request.pattern = pattern.request.SOLID;
            pattern.request.rate = 1;
            pattern.request.red = 0;
            pattern.request.blue = 0;
            pattern.request.green = 255;
            led_strip_.call(pattern);
        }else if(robot_status_ == RobotStatus::HALTED){ //Halted mode 
            pattern.request.pattern = pattern.request.SOLID;
            pattern.request.rate = 1;
            pattern.request.red = 255;
            pattern.request.blue = 0;
            pattern.request.green = 0;
            led_strip_.call(pattern);
        }else if(robot_status_ == RobotStatus::AUTO || robot_status_ == RobotStatus::RETURN_TO_RAIL_HOME){ //Autonomous mode
            pattern.request.pattern = pattern.request.SOLID;
            pattern.request.rate = 1;
            pattern.request.red = 0;
            pattern.request.blue = 255;
            pattern.request.green = 255;
            pattern.request.fill_percent = uv_lamp_controller_ptr_->getCurrentIrradiancePercent();
            led_strip_.call(pattern);
        }else if((robot_status_ == RobotStatus::TELEOP || robot_status_ == RobotStatus::RECOVERY) && (prev_robot_status_ == RobotStatus::AUTO || prev_robot_status_ == RobotStatus::RECOVERY)){ //Teleop when a mission is alredy selected
            pattern.request.pattern = pattern.request.SOLID;
            pattern.request.rate = 1;
            pattern.request.red = 0;
            pattern.request.blue = 255;
            pattern.request.green = 255;
            pattern.request.fill_percent = uv_lamp_controller_ptr_->getCurrentIrradiancePercent();
            led_strip_.call(pattern);
        }else if((robot_status_ == RobotStatus::TELEOP || robot_status_ == RobotStatus::RECOVERY) && prev_robot_status_ != RobotStatus::AUTO ){ //Teleop when there is no mission
            pattern.request.pattern = pattern.request.SOLID;
            pattern.request.rate = 1;
            pattern.request.red = 0;
            pattern.request.blue = 255;
            pattern.request.green = 255;
            led_strip_.call(pattern);
        }else if(robot_status_ == RobotStatus::MANUAL_TREATMENT){ //Manual treatment mode
            pattern.request.pattern = pattern.request.SOLID;
            pattern.request.rate = 1;
            pattern.request.red = 0;
            pattern.request.blue = 255;
            pattern.request.green = 255;
            led_strip_.call(pattern);
        }else if(robot_status_ == RobotStatus::WAITING){ //Autonomous mode
            pattern.request.pattern = pattern.request.BLINKING;
            pattern.request.rate = 2;
            pattern.request.red = 0;
            pattern.request.blue = 255;
            pattern.request.green = 255;
            pattern.request.fill_percent = 100;
            led_strip_.call(pattern);
        }else if(robot_status_ == RobotStatus::CRITICAL_BATTERY){ //Battery critically low 
            pattern.request.pattern = pattern.request.FILLING;
            pattern.request.rate = 1;
            pattern.request.red = 255;
            pattern.request.blue = 0;
            pattern.request.green = 0;
            led_strip_.call(pattern);
        }else if(robot_status_ == RobotStatus::LOW_BATTERY){ //Battery low
            pattern.request.pattern = pattern.request.FILLING;
            pattern.request.rate = 1;
            pattern.request.red = 255;
            pattern.request.blue = 0;
            pattern.request.green = 100;
            led_strip_.call(pattern);
        }else{
            ROS_ERROR("Unknown status");
            pattern.request.pattern = pattern.request.SOLID;
            pattern.request.rate = 1;
            pattern.request.red = 255;
            pattern.request.blue = 255;
            pattern.request.green = 255;
            led_strip_.call(pattern);
        }
    }
    prev_robot_status_ = robot_status_;
}


std_msgs::Float32 RobotStatusManager::getCurrentPose(){
    return curr_robot_rail_pose_;
}


void RobotStatusManager::resetPose()
{
    //Reset the localization node
    std_srvs::Trigger trigger;
    reset_pose_service_.call(trigger);
}
