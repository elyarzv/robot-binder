#include "ais_state_machine/Helpers/InterruptHandler.h"

using namespace ais_state_machine;

InterruptHandler::InterruptHandler(ros::NodeHandle& nh): nh_(nh)
{
    abort_mission_service_ = nh_.advertiseService("stop_mission", &InterruptHandler::abortCB, this);
    abort_mission_ = false;
}
InterruptHandler::~InterruptHandler()
{
}

bool InterruptHandler::checkInterrupts(){
    return abort_mission_;
}

void InterruptHandler::setMissionAborted(bool val){
    abort_mission_ = val;
}

bool InterruptHandler::abortCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){

    abort_mission_ = true;
    res.success = true;
    return true;

}