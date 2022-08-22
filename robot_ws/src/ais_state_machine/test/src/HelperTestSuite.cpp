#include "ais_state_machine/TestSuites/HelperTestSuite.h"

using namespace ais_utilities;
using namespace ais_state_machine;

HelperTestSuite::HelperTestSuite()
{
    
}

HelperTestSuite::~HelperTestSuite()
{

}

void HelperTestSuite::SetUp()
{
    initNode();
    ROS_ERROR("Setting up");
    auto mission_config_ptr_ = make_shared<MissionConfig>();
    rail_status_ptr_ = make_shared<RailStatusManager>(
        nh_, "/embedded/rail_encoder", mission_config_ptr_
    );
    fake_proxy_pub_ = nh_.advertise<std_msgs::UInt8>("embedded/proximiterstatus", 1);
}

void HelperTestSuite::publishFakeProxy(uint val)
{
    std_msgs::UInt8 msg;
    msg.data = val;
    fake_proxy_pub_.publish(msg);
    ros::spinOnce();
    ros::Duration(0.2).sleep();
}


bool HelperTestSuite::getProxyStatus()
{
    return rail_status_ptr_->isProxyOnRails();
}
