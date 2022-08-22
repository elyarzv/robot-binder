#include "ais_state_machine/Helpers/VersionController.h"

using namespace ais_state_machine;


VersionController::VersionController(ros::NodeHandle& nh) : nh_(nh)
{
    // Set Version here
    version_ = "v0.18.0";
    get_version_service_ = nh.advertiseService("/autonomy/get_version", &VersionController::getVersionCb, this);
}

bool VersionController::getVersionCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    // Basic error check
    if (version_.empty()){
        res.message = "Empty version identifier!";
        res.success = false;
        return false;
    }
    res.message = version_;
    res.success = true;

    return true;
}
