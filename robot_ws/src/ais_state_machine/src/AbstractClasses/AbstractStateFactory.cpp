#include "ais_state_machine/AbstractClasses/AbstractStateFactory.h"

using namespace ais_state_machine;

AbstractStateFactory::AbstractStateFactory(shared_ptr<AbstractStateMachineManager> state_manager)
    : state_manager_ptr_(state_manager)
{

}