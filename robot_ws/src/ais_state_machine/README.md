# README #


### What is this repository for? ###

* Running State machine for Phoenix project
* Version: 0.00
* [Learn Markdown](https://bitbucket.org/tutorials/markdowndemo)

### How do I get set up? ###

```
roslaunch ais_state_machine phoenix_state_machine.launch
```

# Adding states to the state machine #
## Add state to phoenix_state_enums.h 

## Copy StateTemplate 

Copy the .cpp and .h files, update the header guards (IFNDEF), and replace all instances of "StateTemplate" in the namespace    

## Set the robot status appropriately 
```
state_manager_->status_manager_ptr_->setRobotStatus(RobotStatus::ROBOT_STATUS);    
```
If introducing a new RobotStatus, be sure to set the following:     

* RobotStatusManager::getRobotStatusString() //For the frontend to display robot status and set appropriate display     

* RobotStatusManager::getMissionStatusString() //For the frontend to display the current mission status     

* RobotStatusManager::update() //Add new LED colours/animations here     

## Set state permissions 

If the state can be interrupted and resumed:     

* Add it to the list of states to save when being interrupted (StateMachineManager::savePreviousState())     

If your state should invalidate the previous state pointer (Once in your state the robot should not ba able to "resume" a previous state)     

* If it isn't in the list in StateMachineManager::savePreviousState(), add it to StateMachineManager::resetPreviousStatePtr()     

## Add state to CMakeLists.txt 
## Add state headers to StateFactory.h 
## Generate state in StateFactory.cpp 
Add state to initializeStates()