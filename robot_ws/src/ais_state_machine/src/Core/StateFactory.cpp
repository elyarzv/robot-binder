#include "ais_state_machine/Core/StateFactory.h"

using namespace ais_state_machine;

StateFactory::StateFactory(shared_ptr<AbstractStateMachineManager> state_manager_ptr) 
    : AbstractStateFactory(state_manager_ptr)
{   
    ROS_INFO("SF: StateFactory construction");
    if (state_manager_ptr == nullptr) {
        ROS_WARN("SF: Null state_manager!");
    }
}

bool StateFactory::initializeStates()
{
    if (state_manager_ptr_ == nullptr) {
        ROS_ERROR("SF: State Factory does not have a valid state_manager_ptr!");
        return false;
    }

    state_map_[StateTypes::IDLE] = make_shared<StateIdle>(state_manager_ptr_, state_manager_ptr_->getInternalNodeHandler());
    state_map_[StateTypes::COMPLETE] = make_shared<StateComplete>(state_manager_ptr_, state_manager_ptr_->getInternalNodeHandler());
    state_map_[StateTypes::RUN_MISSION] = make_shared<StateRunMission>(state_manager_ptr_, state_manager_ptr_->getInternalNodeHandler());
    state_map_[StateTypes::INITIALIZE_ROBOT] = make_shared<StateInitializeRobot>(state_manager_ptr_, state_manager_ptr_->getInternalNodeHandler());
    state_map_[StateTypes::RAIL_DISINFECTION] = make_shared<StateRailDisinfection>(state_manager_ptr_, state_manager_ptr_->getInternalNodeHandler());
    state_map_[StateTypes::RETURN_TO_RAIL_HOME] = make_shared<StateReturnToRailHome>(state_manager_ptr_, state_manager_ptr_->getInternalNodeHandler());
    state_map_[StateTypes::HALTED] = make_shared<StateHalted>(state_manager_ptr_, state_manager_ptr_->getInternalNodeHandler());
    state_map_[StateTypes::MANUAL_CONTROL] = make_shared<StateManualControl>(state_manager_ptr_, state_manager_ptr_->getInternalNodeHandler());
    state_map_[StateTypes::MANUAL_TREATMENT] = make_shared<StateManualTreatment>(state_manager_ptr_, state_manager_ptr_->getInternalNodeHandler());
    state_map_[StateTypes::TELEOPERATION] = make_shared<StateTeleoperation>(state_manager_ptr_, state_manager_ptr_->getInternalNodeHandler());
    auto state_fix_orientation = make_shared<StateFixOrientation>(state_manager_ptr_, state_manager_ptr_->getInternalNodeHandler());
    state_map_[StateTypes::FIX_ORIENTATION] = state_fix_orientation;
    state_map_[StateTypes::MOVE_ROBOT_OFF_RAIL] = make_shared<StateMoveRobotOffRail>(state_manager_ptr_, state_manager_ptr_->getInternalNodeHandler(), state_fix_orientation);
    state_map_[StateTypes::MOVE_TO_NEXT_RAIL] = make_shared<StateMoveToNextRail>(state_manager_ptr_, state_manager_ptr_->getInternalNodeHandler(), state_fix_orientation);
    state_map_[StateTypes::ADJUST_TO_RAIL] = make_shared<StateAdjustToRail>(state_manager_ptr_, state_manager_ptr_->getInternalNodeHandler());
    state_map_[StateTypes::MOUNT_ROBOT_TO_RAIL] = make_shared<StateMountRobotToRail>(state_manager_ptr_, state_manager_ptr_->getInternalNodeHandler());
    state_map_[StateTypes::RECOVERY] = make_shared<StateRecovery>(state_manager_ptr_, state_manager_ptr_->getInternalNodeHandler());
    state_map_[StateTypes::LOW_BATTERY] = make_shared<StateLowBattery>(state_manager_ptr_, state_manager_ptr_->getInternalNodeHandler());
    state_map_[StateTypes::CRITICAL_BATTERY] = make_shared<StateCriticalBattery>(state_manager_ptr_, state_manager_ptr_->getInternalNodeHandler());
    state_map_[StateTypes::NAVIGATION] = make_shared<StateNavigation>(state_manager_ptr_, state_manager_ptr_->getInternalNodeHandler());
    return true;
}

shared_ptr<AbstractState> StateFactory::generateState(StateTypes state_enum) 
{

    if (state_map_.find(state_enum) != state_map_.end())  {
        ROS_DEBUG("SF: State enum! %d", (int)state_enum);
        return state_map_[state_enum];
    } else {
        ROS_WARN("SF: State name %d not found in map!", (int) state_enum);
    }

    return nullptr;
}

void StateFactory::setStateMachineManager(shared_ptr<AbstractStateMachineManager> ptr)
{
    state_manager_ptr_ = ptr;
}