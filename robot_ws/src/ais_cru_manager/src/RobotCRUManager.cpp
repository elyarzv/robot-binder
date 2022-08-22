#include "ais_cru_manager/RobotCRUManager.h"

using namespace ais_cru_manager;

RobotCRUManager::RobotCRUManager(ros::NodeHandle& nh) : nh_(nh)
{
    get_hostname_service_ =
        nh.advertiseService("get_hostname", &RobotCRUManager::getHostnameCb, this);
}

bool RobotCRUManager::getHostnameCb(std_srvs::Trigger::Request& req,
                                    std_srvs::Trigger::Response& res)
{
    // HOST_NAME_MAX is defined as 64 in Linux for POSIX systems
    // Using gethostname from unistd.h (UNIX standard headers)
    char hostname[HOST_NAME_MAX + 1];
    int flag = gethostname(hostname, HOST_NAME_MAX + 1);
    if (flag == 0)
    {
        res.message = hostname;
        res.success = true;
    }
    else
    {
        res.message = "Error has occured in retrieving hostname";
        res.success = false;
        return false;
    }
    return true;
}