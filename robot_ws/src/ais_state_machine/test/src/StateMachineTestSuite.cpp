#include "ais_state_machine/TestSuites/StateMachineTestSuite.h"

using namespace ais_utilities;
using namespace ais_state_machine;

StateMachineTestSuite::StateMachineTestSuite()
{
    
}

StateMachineTestSuite::~StateMachineTestSuite()
{

}

void StateMachineTestSuite::SetUp()
{
    initNode();
    fake_odom_pub_ = nh_.advertise<std_msgs::Float32>("encoder_localization_node/pose_1_d", 1);
    fake_soc_pub_ = nh_.advertise<std_msgs::UInt8>("embedded/battery/state_of_charge", 1);
    fake_proxy_pub_ = nh_.advertise<std_msgs::UInt8>("embedded/proximiterstatus", 1);
    fake_front_bumper_pub_ = nh_.advertise<std_msgs::Bool>("/embedded/bumper/front", 1);
    set_dimming_server_ = nh_.advertiseService("set_dimmer_values", 
                                            &StateMachineTestSuite::setDimmerValuesService, this);
    current_state_sub_ = nh_.subscribe("current_state", 1, 
                            &StateMachineTestSuite::currentStateCb, this);
    start_mission_client_ = nh_.serviceClient<std_srvs::Trigger>("start_mission");
    robot_mode_client_ = nh_.serviceClient<phoenix_msgs::setMode>("set_robot_mode");
    ros::service::waitForService("start_mission");
    curr_state_ = StateTypes::IDLE;
}
void StateMachineTestSuite::setMissionDir(){
    stringstream ss;
    ss << getenv("PWD") << "/../../install/share/ais_state_machine/test/";
    nh_.setParam("MISSION_DIR", ss.str());
}
void StateMachineTestSuite::startMission()
{
    std_srvs::Trigger srv;
    
    start_mission_client_.call(srv);
}


void StateMachineTestSuite::currentStateCb(const std_msgs::Int32::ConstPtr& msg)
{
    curr_state_= (StateTypes) msg->data;
}

void StateMachineTestSuite::publishFakeOdom(float value)
{   
    std_msgs::Float32 msg;
    msg.data = value;
    fake_odom_pub_.publish(msg);
    ros::spinOnce();
    ros::Duration(0.2).sleep();
}

void StateMachineTestSuite::publishFakeProxy(uint value)
{   
    std_msgs::UInt8 msg;
    msg.data = value;
    fake_proxy_pub_.publish(msg);
    ros::spinOnce();
}

void StateMachineTestSuite::rechargeBattery(uint value)
{   
    while (curr_state_ != StateTypes::IDLE) {
        publishFakeSoC(value);
        ros::spinOnce();
        callSetRobotMode("AUTO");
    }
}

void StateMachineTestSuite::publishFakeSoC(uint value)
{   
    std_msgs::UInt8 msg;
    msg.data = value;
    fake_soc_pub_.publish(msg);
    ros::spinOnce();
    ros::Duration(0.2).sleep();
}

void StateMachineTestSuite::publishFakeFrontBumper(bool value)
{   
    std_msgs::Bool msg;
    msg.data = value;
    fake_front_bumper_pub_.publish(msg);
    ros::spinOnce();
    ros::Duration(0.2).sleep();
}

StateTypes StateMachineTestSuite::getCurrentState()
{
    return curr_state_;
}

void StateMachineTestSuite::waitForStateTransition(StateTypes state)
{
    while (curr_state_ == state) {
        ros::spinOnce();
    }

}
void StateMachineTestSuite::waitForState(StateTypes state)
{
    while (curr_state_ != state) {
        ros::spinOnce();
    }

}

void StateMachineTestSuite::callSetRobotMode(string mode_)
{
    phoenix_msgs::setMode newMode;
    newMode.request.mode = mode_;
    robot_mode_client_.call(newMode);
}

void StateMachineTestSuite::getCurrentIntensity(std::vector<float>& intensity){
    intensity.resize(0);
    for(auto val : uv_request.lamp_values){
        intensity.push_back(val);
    }
}
bool StateMachineTestSuite::setDimmerValuesService(phoenix_msgs::SetLampStatus::Request& req, 
                                                phoenix_msgs::SetLampStatus::Response& res)
{
    uv_request = req;
    res.success = true;
    res.message = "Testing values recieved";
    return true;
}