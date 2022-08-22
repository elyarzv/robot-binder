
#include "ais_state_machine/AbstractClasses/AbstractStateMachineManager.h"

using namespace ais_state_machine;

AbstractStateMachineManager::AbstractStateMachineManager(ros::NodeHandle& nh,
                                                        StateTypes state,
                                                        shared_ptr<AbstractStateFactory> factory_ptr)
    : nh_(nh), curr_state_name_(state), state_ptr_(nullptr), factory_ptr_(factory_ptr)
{
    
}

ros::NodeHandle& AbstractStateMachineManager::getInternalNodeHandler()
{
    return nh_;
}