#include "ais_cru_manager/RobotCRUManager.h"
#include "ros/ros.h"

using namespace ais_cru_manager;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cru_device_manager");
    ros::NodeHandle nh;

    RobotCRUManager cru_manager(nh);

    ros::spin();
    return 0;
}