#include <memory>
#include "ros/ros.h"
#include "ais_state_machine/Core/StateMachineManager.h"
#include "ais_state_machine/Core/StateFactory.h"
#include "ais_state_machine/Enums/phoenix_state_enums.h"
using namespace ais_state_machine;
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "phoenix_state_machine");
    ros::NodeHandle nh;

    ros::Rate loop_rate(1);

    ROS_INFO("Initiating State Machine Node");
    
    shared_ptr<StateFactory> factory_ptr = make_shared<StateFactory>(nullptr);
    shared_ptr<StateMachineManager> sm_ptr = make_shared<StateMachineManager>(nh, StateTypes::IDLE, factory_ptr);

    ROS_INFO("Entering while loop");
    sm_ptr->run();
    ROS_INFO("Exiting State Machine Node");

    return 0;
}