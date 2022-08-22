#ifndef AIS_VERSION_CONTROLLER_H_
#define AIS_VERSION_CONTROLLER_H_

#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include <string>

using std::string;

namespace ais_state_machine
{
class VersionController
{
    public:
        VersionController(ros::NodeHandle&);
        ~VersionController() = default;

    protected:
    /*
     * Service callback to return string verison_
    */
        bool getVersionCb(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&);

    private:
        ros::NodeHandle nh_;
        string version_;
        ros::ServiceServer get_version_service_;
};
}; // End ais_state_machine namespace
#endif // End AIS_VERSION_CONTROLLER_H_