/*
    AIS CONFIDENTIAL
    Author: Michael Chiou (m.chiou@ai-systems.ca)
*/

#ifndef ASM_ABSTRACT_STATE_FACTORY_H_
#define ASM_ABSTRACT_STATE_FACTORY_H_

#include <memory>
#include <string>
#include <map>
#include <ctype.h>

#include "ais_state_machine/AbstractClasses/AbstractStateMachineManager.h"
#include "ais_state_machine/AbstractClasses/AbstractState.h"

#include "ais_state_machine/Enums/phoenix_state_enums.h"

using std::string;
using std::shared_ptr;

namespace ais_state_machine
{
// Forward class declarations
class AbstractStateMachineManager;
class AbstractState;
class AbstractStateFactory
{
    public:
    /*
    *  Constructor class which takes in shared pointer to StateMachineManager. This pointer is given
    *  to provide to States
    */     
        AbstractStateFactory(shared_ptr<AbstractStateMachineManager>);
    /*
    *  Virtual destructor
    */ 
        virtual ~AbstractStateFactory() = default;
    /*
    *  Returns pointer to State if given a StateTypes
    */ 
        virtual shared_ptr<AbstractState> generateState(StateTypes) = 0;
    /*
    *  Change pointer to StateMachineManager that is provided to Concrete States
    */ 
        virtual void setStateMachineManager(shared_ptr<AbstractStateMachineManager>) = 0;
    /*
    *  Generates shared pointers to all states which must be used in State Machine
    */ 
        virtual bool initializeStates() = 0;
    protected:   
        shared_ptr<AbstractStateMachineManager> state_manager_ptr_;
};

}; // End namespace ais_state_machine


#endif // ASM_ABSTRACT_STATE_FACTORY_H_