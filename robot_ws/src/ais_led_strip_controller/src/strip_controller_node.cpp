#include <memory>
#include "ros/ros.h"
#include "ais_led_strip_controller/strip_controller.h"
using namespace ais_led_strip_controller;
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "led_strip_controller");
    ros::NodeHandle nh;

    ros::Rate loop_rate(1);
    LEDController controller(nh);
    //SolidPattern solid(15);
    //solid.setColour(9000);
    

    //ROS_INFO_STREAM("Initiating LED Controller Node" << solid.getFrame());
    while(ros::ok()){
        controller.run();
    }


    return 0;
}