#include "ais_state_machine/AbstractClasses/AbstractState.h"

using namespace ais_state_machine;

AbstractState::AbstractState(shared_ptr<AbstractStateMachineManager>state_manager)
          : state_manager_(state_manager),
            has_finished_state_(false),
            next_state_name_(StateTypes::IDLE)
{
  
}
