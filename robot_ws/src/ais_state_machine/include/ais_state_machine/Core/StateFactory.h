/*
    AIS CONFIDENTIAL
    Author: Michael Chiou (m.chiou@ai-systems.ca)
*/

#ifndef ASM_STATE_FACTORY_H_
#define ASM_STATE_FACTORY_H_

#include "ros/ros.h"
#include "ais_state_machine/AbstractClasses/AbstractStateFactory.h"
#include "ais_state_machine/AutonomyStates/StateIdle.h"
#include "ais_state_machine/AutonomyStates/StateHalted.h"
#include "ais_state_machine/AutonomyStates/StateComplete.h"
#include "ais_state_machine/AutonomyStates/StateRunMission.h"
#include "ais_state_machine/AutonomyStates/StateInitializeRobot.h"
#include "ais_state_machine/AutonomyStates/StateRailDisinfection.h"
#include "ais_state_machine/AutonomyStates/StateReturnToRailHome.h"
#include "ais_state_machine/AutonomyStates/StateTeleoperation.h"
#include "ais_state_machine/AutonomyStates/StateManualControl.h"
#include "ais_state_machine/AutonomyStates/StateManualTreatment.h"
#include "ais_state_machine/AutonomyStates/StateAdjustToRail.h"
#include "ais_state_machine/AutonomyStates/StateMoveRobotOffRail.h"
#include "ais_state_machine/AutonomyStates/StateMoveToNextRail.h"
#include "ais_state_machine/AutonomyStates/StateMountRobotToRail.h"
#include "ais_state_machine/AutonomyStates/StateRecovery.h"
#include "ais_state_machine/AutonomyStates/StateLowBattery.h"
#include "ais_state_machine/AutonomyStates/StateCriticalBattery.h"
#include "ais_state_machine/AutonomyStates/StateNavigation.h"
#include "ais_state_machine/Enums/phoenix_state_enums.h"

namespace ais_state_machine
{

using std::string;
using std::map;
using std::shared_ptr;
using std::make_shared;

class StateFactory : public AbstractStateFactory{
    public:
        StateFactory(shared_ptr<AbstractStateMachineManager> state_manager_ptr);
        shared_ptr<AbstractState> generateState(StateTypes state_enum) override;
        void setStateMachineManager(shared_ptr<AbstractStateMachineManager> ptr) override;
        bool initializeStates() override;

    protected:
    private:
        map<StateTypes, shared_ptr<AbstractState>> state_map_;
};
} // end ais_state_machine namespace
#endif // ASM_STATE_FACTORY_H_