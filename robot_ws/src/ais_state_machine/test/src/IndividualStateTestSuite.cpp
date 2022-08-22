#include "ais_state_machine/TestSuites/IndividualStateTestSuite.h"

using namespace ais_utilities;
using namespace ais_state_machine;

IndividualStateTestSuite::IndividualStateTestSuite()
{
    
}

IndividualStateTestSuite::~IndividualStateTestSuite()
{

}

void IndividualStateTestSuite::SetUp()
{
    initNode();
    ROS_ERROR("Setting up");
    fake_odom_pub_ = nh_.advertise<std_msgs::Float32>("encoder_localization_node/pose_1_d", 1);
    fake_irradiance_pub_ = nh_.advertise<phoenix_msgs::DimmingIrradianceData>("uv_estimated_irradiance", 1);
    set_dimming_server_ = nh_.advertiseService("set_dimmer_values", 
                                            &IndividualStateTestSuite::setDimmerValuesService, this);
                                            
    cmd_vel_sub_ = nh_.subscribe("embedded/cmd_vel", 1, &IndividualStateTestSuite::cmdVelocityCb, this);
    curr_state_ = StateTypes::INITIALIZE_ROBOT;
    factory_ptr_ = make_shared<StateFactory>(nullptr);
    sm_ptr_ = make_shared<StateMachineManager>(nh_, StateTypes::INITIALIZE_ROBOT, factory_ptr_);
    sm_ptr_->initialize();
    received_cmd_vel_.linear.x = 0;
    received_cmd_vel_.angular.z = 0;
    stringstream ss;
    ss << getenv("PWD") << "/../../install/share/ais_state_machine/test/";
    nh_.setParam("MISSION_DIR", ss.str());
    ss.clear();
    ss.str(string());
    ss << "example_mission.csv";
    nh_.setParam("MISSION_NAME", ss.str()); 
}

void IndividualStateTestSuite::transitionStateTo(StateTypes state_enum)
{
    sm_ptr_->transitionToNextState(state_enum);
}


void IndividualStateTestSuite::currentStateCb(const std_msgs::Int32::ConstPtr& msg)
{
    curr_state_= (StateTypes) msg->data;
}

void IndividualStateTestSuite::cmdVelocityCb(const geometry_msgs::Twist::ConstPtr& msg)
{
    received_cmd_vel_= *msg;
}

geometry_msgs::Twist IndividualStateTestSuite::getPublishedCmdVel()
{
    return received_cmd_vel_;
}

StateTypes IndividualStateTestSuite::getCurrentState()
{
    return curr_state_;
}

void IndividualStateTestSuite::waitForStateTransition(StateTypes state)
{
    while (curr_state_ == state) {
        ros::spinOnce();
    }
}

void IndividualStateTestSuite::runStateOnce()
{
    sm_ptr_->runCurrentStateOnce();
    ros::Duration(0.25).sleep();
}

void IndividualStateTestSuite::publishFakeOdom(float value)
{   
    std_msgs::Float32 msg;
    msg.data = value;
    fake_odom_pub_.publish(msg);
    ros::spinOnce();
    ros::Duration(0.2).sleep();
}

void IndividualStateTestSuite::publishFakeUVIrradiance(double irrad_val, double irrad_percent)
{   
    phoenix_msgs::DimmingIrradianceData msg;
    msg.peak_irradiance = 658;
    msg.max_stable_irradiance = 613;
    msg.mode_duration = ros::Duration(500.0);

    msg.irradiance = irrad_val;
    msg.percentage = irrad_percent;

    fake_irradiance_pub_.publish(msg);
    ros::spinOnce();
    ros::Duration(0.2).sleep();
}

bool IndividualStateTestSuite::setDimmerValuesService(phoenix_msgs::SetLampStatus::Request& req, 
                                                phoenix_msgs::SetLampStatus::Response& res)
{
    res.success = true;
    res.message = "Testing values recieved";
    return true;
}