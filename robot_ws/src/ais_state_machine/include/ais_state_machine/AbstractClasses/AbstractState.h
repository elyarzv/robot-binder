/*
    AIS CONFIDENTIAL
    Author: Michael Chiou (m.chiou@ai-systems.ca)
*/
#ifndef ASM_ABSTRACT_STATE_H_
#define ASM_ABSTRACT_STATE_H_

#include <memory>
#include <string>
#include <iostream>
#include "ais_state_machine/AbstractClasses/AbstractStateMachineManager.h"
#include "ais_state_machine/Enums/phoenix_state_enums.h"


using std::shared_ptr;
using std::string;
using std::cout;
using std::endl;


/* Implicit Assumption
    Suggestion to use latest topic only in subscriber
*/
namespace ais_state_machine{
// Forward declaration for AbstractStateMachineManager
class AbstractStateMachineManager;
/*
 *   Abstract class that defines foundation for state machine state implementation
 */
class AbstractState {
public:
/*
 *   Constructor that takes in shared_ptr of AbstractStateMachineManager. StateMachineManager will
 *   carry persistent data which may be used by a child State      
 */
    AbstractState(shared_ptr<AbstractStateMachineManager> state_manager);
/*
 *   Required virtual destructor     
 */
    virtual ~AbstractState() = default;
/*
 *  Execution loop for state. Must not include long blocking calls as the StateMachineManager
 *  will run this function in a loop. Function must  
 */
    virtual void runState() = 0;
/*
 *  Function that signals if state has completed its action from evaluateStateStatus() and needs to
 *  transition to another state
 */    
    virtual bool isStateFinished() = 0;
/*
 *  Function that returns the next StateTypes enum. (Which state to transition to)
 */ 
    virtual StateTypes getNextStateName() = 0;
/*
 *  This function is called when the state machine manager transitions IN to this state
 */ 
    virtual void init() = 0;
/*
 *  This function is called when the state machine manager transitions OUT of this state
 */ 
    virtual void finish() = 0;
/*
 *  This function is called when an interrupt is triggered in the StateMachineManager. Function 
 *  should cleanly resolve any currently running process in the state. Example: move_base action  
 */ 
    virtual void interruptState() = 0;
/*
 *  Is called when the need to reinitialize State member variables back to clean state for when 
 *  state is transitioned to.
 */ 
    virtual void reset() = 0;

protected:
/*
 *  Function that contains code that determines which state should be run next. If the state is not
 *  finished, then the state should point to itself, if the state is finished, state should point
 *  to the next state
 */ 
    virtual void evaluateStateStatus() = 0;

    shared_ptr<AbstractStateMachineManager> state_manager_;
    bool has_finished_state_;
    StateTypes next_state_name_;
};

} // End namespace ais_state_machine
#endif //ASM_ABSTRACT_STATE_H_