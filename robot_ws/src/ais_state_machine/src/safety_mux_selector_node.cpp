/*
    AIS CONFIDENTIAL
    Author: Rakesh Shrestha (r.shrestha@ai-systems.ca)
*/

#include "ros/ros.h"
#include "ais_state_machine/SafetyMuxSelector/safety_mux_selector.h"

using namespace ais_state_machine;

int main(int argc, char **argv) {
    ros::init(argc, argv, "safety_mux_selector");
    ros::Time::init();
    ros::Rate loop_rate(10);
    SafetyMuxSelector safety_mux_selector;
    while(ros::ok()){
        ros::spinOnce();
        if(!safety_mux_selector.isSourceAlive()){
            safety_mux_selector.callMuxSelector(SAFE_CMD_VEL_TOPIC);
        }
        loop_rate.sleep();
    }
}
