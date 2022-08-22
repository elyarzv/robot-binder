#include "ais_cru_manager/GetHostnameTestSuite.h"

GetHostnameTest::GetHostnameTest()
{
    get_hostname_client_ = nh.serviceClient<std_srvs::Trigger>("get_hostname");
    ros::service::waitForService("get_hostname");
}

GetHostnameTest::~GetHostnameTest()
{
}

bool GetHostnameTest::getHostname(string& name)
{
    if (get_hostname_client_.exists() && get_hostname_client_.call(trigger_))
    {
        if (trigger_.response.success)
        {
            name = trigger_.response.message;
            ROS_ERROR("Hostname is: %s", name.c_str());
            std::cout << "Hostname is: " << name << std::endl;
            return true;
        }
    }
    return false;
}