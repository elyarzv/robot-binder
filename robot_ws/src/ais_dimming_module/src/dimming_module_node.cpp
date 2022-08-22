
#include "ros/ros.h"
#include "ais_dimming_module/DimModuleController.h"

using namespace ais_dimming_module;

int main(int argc, char **argv) 
{

    ros::init(argc, argv, "dimming_module_controller");
    ros::NodeHandle nh;
    
    int update_freq = 10;
    ros::Rate loop_rate(update_freq);

    ROS_INFO("Initiating Dimming Module Node");
    DimModuleController dmc(nh, update_freq);
    while(ros::ok())
    {
        dmc.run();
        loop_rate.sleep();
    }
    
    return 0;
}