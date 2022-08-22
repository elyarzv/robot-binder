# Changelog
All notable changes to the project will be documented in this file.  
This project adheres to [Semantic Versioning 2.0.0](https://semver.org/spec/v2.0.0.html).  
The baseline for this **CHANGELOG.md** is [keep a changelog](https://keepachangelog.com/en/1.0.0/).  

## Documentation

Additional documentation can be found in confluence in [Interfaces and Operations Instructions](https://ais-ugv2.atlassian.net/wiki/spaces/PHOENIX/pages/2962554881/Interfaces+and+Operations+Instructions)
## **Unreleased Version** 
## [0.0.0] - YYYY-MM-DD
### Added
### Changed
### Deprecated
### Removed
### Fixed
### Security 

## **Latest Released Version** 
## [0.18.0] - 2022-05-25
### Added
 - Added "time_remaining" key to /autonomy/robot_status message to show seconds remaining until mission starts (PH-1652)
 - Status manager monitors embedded/uv_lamp_status and embedded/uv_lamp_dimmer_values, and sets the error code 230022 if they don't match (PH-1621)
 - Perception recorder launch file from the rosbags.launch (PH-1629)
 - Turning off robot lights while it is waiting to start the treatment and turn them back on as soon as the mission starts (PH-1682)
### Changed
 - Reduced safety mux selector timeout to 3 seconds (PH-1658)
### Deprecated
### Removed
### Fixed
 - Logic for recording perception topics when the rail head detection fails (PH-1629)
 - Frequency bounds for lower rate topics eg. is_online rate is 8 minutes (PH-1441)
 - Strange Behaviour when the Teleoperator Starts Row Treatment (PH-1639)
### Security 

## **Previous Released Versions**
## [0.17.0] - 2022-05-19
### Added
### Changed
 - Removed "WAITING" status when USE_STOP_AS_UV is true and added "WAITING" robot status and "PAUSED" mission status for when robot is waiting to start a mission (PH-1637)
 - Encoder malfunction on rails error code changed form moderate to critical (110015->210015) (PH-1607)
### Deprecated
### Removed
### Fixed
### Security 

## [0.16.1] - 2022-05-13
### Added
 - Added support for missions of less than 8 lamps (PH-607)
### Changed
 - Refinement and documentation (PH-1487)
### Deprecated
### Removed
### Fixed
 - Speed control issue while irradiance speed controller activated (PH-1514)
 - Unit tests fixes for ais_state_machine
### Security 

## [0.16.0] - 2022-05-09
### Added
### Changed
### Deprecated
### Removed
### Fixed
 - Robot will return to LOW_BATTERY when battery is low and user leaves MANUAL_OPERATION (PH-1515)
 - Added timeout, fixed error (PH-1373)
 - - Removed DEMO_MULTIPLE_RAILS parameter. The logic is now determined based on the number of rail missions set in configuration (PH-1455)
 - When return home light intensity is less than treatment light intensity, the robot returns home with given speed (PH-1575)
### Security 

## [0.15.1] - 2022-04-29
### Added
 - Robot enters HALTED state if bumper is pressed during FixOrientation, MoveRobotOffRail, MoveToNextRail, AdjustToRail, or MountRobotToRail states (PH-1501)
 - Robot enters RECOVERY state if MoveToNextRail state takes longer than 45 seconds (PH-1491)
 - Capability of adding desired delay before starting the treatment (PH-1571)
### Changed
### Deprecated
### Removed
### Fixed
 - Simulated power inverter now publishes UInt8 instead of Bool to match the robot's hardware (PH-1456)
 - Added error check for listing mission in a directory that does not exist (PH-1372)
 - Added mutex locks to make clearing error messages threadsafe (PH-1381)
 - Fix for "SLAM cannot get orientation when dismounting off the rail / move to next rail" (PH-1497)
    - more robust checks when activating/deactivating slam_toolbox
    - continue without fixing orientation if still error in getting orientation
 - Fixed "Cancelling the current robot's job will not clear the counter for completed rails" (PH-1561)
 - Placing objects near rail head in simulation further from it to prevent false detection (PH-1564)
 - Removed list_available_rails_missions service (PH-664)
### Security 

## [0.15.0] - 2022-04-21
### Added
 - Specify row numbers for treatment (PH-1445)
    - uses ROS navigation stack with saved maps to navigate to non-adjacent rails
    - can specify sequence of rows for treatment in the defaultRobotMissionConfig.yaml file
    - added obstacles to simulation world file for map features
 - Stress test can handle variable number of rails (PH-1342)
### Changed
### Deprecated
### Removed
### Fixed
 - Added mutex locks to make clearing error messages threadsafe (PH-1381)
 - Calling AUTO on /set_robot_mode while in TELEOP now resumes the mission (PH-1453)
### Security 

## [0.14.0] - 2022-04-07
### Added
 - Exposed params for mount and dismount distance (PH-1368)
 - Added gazebo_ros_pkgs and modified package so it does not publish odom, slam_toolbox will publish odom
   <publishOdometryTF>false</publishOdometryTF> on libgazebo_ros_planar_move.so (PH-1443)
 - Added parameter /use_sim_time depending if runs with the real hardware or in the simulation (PH-1445)
### Changed
 - Changed on ais_embedded_simulator to publish proximity sensor before rail_encoder so there is no conflict with
   ais_state_machine (PH-1443)
### Deprecated
### Removed
### Fixed
 - Using irradiance based speed control no longer causes a diagnostic failure in rail disinfection state (PH-1446)
 - Bringing the rails closer to the ground so the robot dismounts and does not collide with the body (PH-1346)
### Security 

## [0.13.1] - 2022-04-01
### Added
 - Publish error code when robot enters halted state due to bumper press (PH-1432)
### Changed
### Deprecated
### Removed
### Fixed
 - Return to home state displays estimated irradiance instead of filling green bar (PH-1383)
 - Robot allows manual control in low battery state (PH-1406)
### Security 

## [0.13.0] - 2022-03-25
### Added
 - Fixing robot's orientation when dismounting off the rail using SLAM (PH-1049)
 - Topic /embedded/proximitystatus to the simulation to simulate the proximity sensor when the robot touches the ground (PH-1346)
 - Added error codes for when rail encoder or proxy sensor publishes incorrect information (PH-1348)
### Changed
 - Applied low pass filter to porximiterstatus topic so that 2 of the last 3 messages must agree to change the status (PH-1305)
### Deprecated
### Removed
 - Speed test and moved it to the performant pipeline container (PH-1336)
### Fixed
### Security 

## [0.12.1] - 2022-03-21
### Added
 - Added row_total, row_counter, and mission_name keypairs to autonomy/robot_status topic (PH-1253)
 - docker-compose-wait script so that there is no conflict between containers problem started with local-websocket and ports (PH-1229)
 - Added log folder to the parameter server (PH-1229)
### Changed
 - Robot will attempt to complete the rail disinfection and stop out of the way when a moderate sensor failure occurs (PH-1304)
 - Renumbered error codes to match severity level (https://ais-ugv2.atlassian.net/wiki/spaces/PHOENIX/pages/3151003732/Phoenix+Error+Codes)
### Deprecated
### Removed
 - Removed rosbridge from the connect pane so that there is no conflicts with the ephemeral ports (PH-1229)
### Fixed
 - Changed robot_status string for COMPLETE state from IDLE to COMPLETE (PH-860)
 - Fixed a bug that would prevent the robot from resuming when /set_robot_mode was called with AUTO after interrupting the INITIALIZE state (PH-1362)
### Security 

## [0.12.0] - 2022-03-15
### Added
- Moving from Base to the first treatment row (PH-1225)
   - new state/mode called NAVIGATION
   - navigation to arbitrary rail number
   - localization on a pre-mapped environment
   - robot parameters as environment variables (similar to Orion)
 - Robot will not return home if bumper is pressed and it has completed less than 90% of the final goal. If the bumper remains pressed for more than 2 seconds, the robot will enter HALTED (PH-1238)
 - Robot uses values RAIL_TRANSITION_INTENSITY from defaultRobotMissionConfig.yaml to set UV intensity when transitioning between rails (PH-1299)
### Changed
 - Reduced blink rate to 2Hz (PH-1017)
### Deprecated
### Removed
### Fixed
 - Robot will not toggle UV relay when dismounting if DEMO_MULTIPLE_RAILS is set to true (PH-1298)
 - Fixed bug in which robot does not enter recovery when dismounting the rails (PH-1287)
 - When robot goes to battery low state now cmd_vel publishes 0 so the robot does not move
 - Robot publishes COMPLETE status in COMPLETE state (PH-860)
### Security 

## [0.11.1] - 2022-03-09
### Added
 - [ais-pipeline-library](https://bitbucket.org/ais_admin/ais-pipeline-library/src/main/) for Jenkins (PH-1106) 
 - Added environment presseter to run rosmaster and set robot name, set folder for logs (PH-1252) 
### Changed
 - Behaviour on LED when goes to RECOVERY or TELEOP, the LED display blue as in running a mission as long as the intensity is preseti, it will fill led with the intensity otherwise it will fill it completely.
 - HALTED state LED becomes red
 - Simulation change the camera topics and placement matches the real robot for phoenix_vehicle_model, phoenix_visualization
### Deprecated
### Removed
 - Removed embedded tmux panes from navigation container it is now in [embedded-interface](https://bitbucket.org/ais_admin/platformio-binder/src/master/embedded-controller/)
### Fixed
### Security 

## [0.11.0] - 2022-02-25
### Added
 - Exporting lvcov results with HTML jenkins plugin (PH-1197)
### Changed
 - Stress testing feature for rail head detection implemented along with rail adjustment tuning. Enable stress test using rosparam STRESS_TEST = true (PH-1184)
 - Robot uses proximity sensor to dismount rails
### Deprecated
### Removed
### Fixed
### Security 

## [0.10.1] - 2022-02-17
### Added
### Changed
 - The robot will not enter HALTED state due to a diagnostic failure if the param DIAGNOSTIC_SAFETY is set to false in defaultRobotMissionConfig.yaml (PH-1216)
 - The robot will enter RECOVERY state if safety node stops AdjustToRail, MoveToNextRail, or MoveRobotOffRail states for more than 1 second (PH-1207)
 - Temporarly launch file for amr dimming module launcher
### Deprecated
### Removed
### Fixed
 - IMU TF orientation description parameters
 - IMU publishing out of the TF tree
### Security 

## [0.10.0] - 2022-02-14
### Added
 - Robot enters recovery state if sensor topics do not publish (PH-1003)
 - Enter recovery state if obstacles prevent mission execution (PH-1076)
 - Robot enters halted state if diagnostics report failure (PH-1099)
 - Dismounting safety: when the robot is in StateMoveRobotOffRail, we need to only check the back of the robot for safety (PH-1002)
 - Using the proxy sensor data to detect when the robot has passed the rail home position (PH-1131)
### Changed
 - Move static TF's of camera from phoenix1_bringup description.launch to urdf (PH-732)
 - Translated the rail in the multiple_rail.world to 0 0 0 (PH-1066)
### Deprecated
### Removed
### Fixed
 - Phoenix simulation moving in y direction tips over (PH-1066)
 - Fix "Inbound TCP/IP connection failed: connection from sender terminated before handshake header received" (PH-1157)
 - Fix the issue of amr.yaml not loading (PH-1153)
### Security 

## [0.9.1] - 2022-02-07
### Added
 - Include Manual Mode in Safety Node with Enable/Disable flag (PH-1039)
 - Added diagnostic aggregator (PH-1098)
 - Briging the robot to halted state if any of the sensors fail (PH-1052)
### Changed
### Deprecated
### Removed
### Fixed
 - Phoenix simulation moving in y direction tips over (PH-1066)
### Security 

## [0.9.0] - 2022-01-26
### Added
 - Robot will not enter teleop state if it is in manual control (PH-441)
 - Multiple rail missions can be selected by setting the ROS param NUMBER_OF_RAIL_MISSIONS and DEMO_MULTIPLE_RAILS (true) (PH-865)
 - Robot will enter CRITICAL_BATTERY state any time the state of charge drops below CRITICAL_BATTERY (Currently 25%) (PH-789)
 - Robot will enter LOW_BATTERY state any time the state of charge drops below LOW_BATTERY+ESTIMATED_BATTERY_USAGE (Currently 30% and 0% respectively) AND the robot is not currently in a mission (PH-789)
 - Robot will enter LOW_BATTERY state after completing a mission if the state of charge drops below LOW_BATTERY+ESTIMATED_BATTERY_USAGE (Currently 30% and 0% respectively) while the robot is currently in a mission (PH-789)
 - Robot will enter LOW_BATTERY state after mounting the next rail if the state of charge drops below LOW_BATTERY+ESTIMATED_BATTERY_USAGE (Currently 30% and 0% respectively) while the robot is transitioning between rails (PH-789)
 - Robot will resume from CRITICAL_BATTERY state only if state of charge is above MINIMUM_BATTERY (Currently 80%) (PH-789)
 - Enable safe command velocities during MOVE_TO_NEXT_RAIL and ADJUST_TO_RAIL states using topic multiplexing (PH-904)
 - Robot will enter RECOVERY state if it does not receive a rail head position for longer than 20 seconds (PH-717, PH-941)
 - Robot footprint updated to avoid getting too close to obstacles (PH-904)
 - Not hard coding parameter file location to load the footprint parameters
### Changed
### Deprecated
### Removed
### Fixed
 - Fixed return home state, turn the uv lights on after the bumper was hit, added check to not run on uv lights if they are already on (PH-937)
### Security 

## [0.8.0] - 2022-01-17
### Added
 - Added colcon tests and simulations to the pipelines for feature/\*, develop, hotfix/\*, bugfix/\*, and PR-\* branches (PH-118)
 - Added ROS_INFO for debugging, distance for next rail, off rail among other (PH-576)
   (PH-576) The assumption is for the robot to be turned on in front of the rail facing front just ready to mount to the rail
 - Pose transformations for correct behavior when initializing robot in different orientations (PH-762)
 - Dimming Module performs UV irradiance estimation and disinfection missions will adjust speed based on irradiance (PH-721)
 - Added ros param to disable irradiance based speed control (IRRADIANCE_SPEED_CONTROL)
 - Robot enters HALTED state if AUX power switches from on to off while RobotStatus is AUTO or RETURN_TO_RAIL_HOME (PH-755)
 - Calling /set_robot_mode with "AUTO" now returns the robot to the previous state instead of IDLE (PH-818)
 - ais_makefile contains commands to start docker containers running the robot in simulation (PH-118)
 - Separated runners such that:
    - Bitbucket runs develop, feature/, and bugfix/ with the option to manually trigger colcon tests or simulation tests
    - Jenkins runs main, release/, test/, and tags with colcon and simulation tests running by default
 - Added internet speed metrics from speedtest-cli, outpus a csv file under the log folder for the mission (PH-548)
 - Added initialize command and updated robot/mission status strings (PH-916)
 - Only use the velocity multiplier if the UV lamp is on (PH-898)

### Changed
 - Calling /set_robot_mode with the string "INITIALIZE" will start a new rail disinfection (PH-916)
 - Robot now uses RETURN_TO_RAIL_HOME_SPEED instead of MAX_RAIL_SPEED to determine the return to rail home speed (PH-605)
### Fixed
 - Phoenix bringup script now brings up cru_manager and provides /get_hostname service (PH-773)
### Removed

---

## [0.7.0] - 2021-12-21
### Added
 - Robot can turn on lights in return to home using RETURN_TO_RAIL_HOME_WITH_UV_ON and RETURN_HOME_INTENSITY (PH-588)
 - Added hardcoded TF's for the cameras to base link (PH-601) to be changed on (PH-732)
   (PH-576) The assumption is for the robot to be turned on in front of the rail facing front just ready to mount to the rail
 - Added State MoveRobotOffRail to move robot off rail (PH-576)
 - Added State AdjustToRail to locally adjust robot for mounting onto rail (PH-576)
 - Added State MountRobotToRail which moves robot forward and onto rail (PH-576)
 - Robot can semi-autonomously perform a rail disinfection mission and move to adjacent rail under IDEAL circumstances (PH-576)
 - ROS parameter to demo only rail portion (without perception modules) for disinfection (PH-708)
### Changed
### Fixed
### Removed

---

## [0.6.0] - 2021-12-02
### Added
 - Added feature flags to disable off-rail detection using ros params OFF_RAIL_DETECTION (all) OFF_RAIL_DETECTION_AUTONOMOUS (auto) OFF_RAIL_DETECTION_SEMI_AUTONOMOUS (manual treatment) PH-475
### Changed
 - Removed lidar and zed topics to reduce bag file size https://bitbucket.org/ais_admin/phoenix1_bringup/pull-requests/21
 - Increased state machine loop rate to 25Hz and reduced cmd_vel message callback que to 1 (PH-466)
 - Missions must now be defined with 8 individual lamp intensities (PH-463)
 - Modified calculation of rail speed (PH-474)
### Fixed
 - Robot can start disinfection mission backwards if enabled via mission config (PH-487)
 - Robot turns off lights in halted mode (PH-482)
### Removed

---

## [0.5.0] - 2021-11-18

### Removed
 - Off rail detection, because of false positives, the feature is in the source code the flag is set to off (PH-358)
 - If the robot starts off rails it will enter the halted state within 1.2m (PH-426)

### Added
 - Added ais_utilities for testing infrastructure (PH-430)
 - Automated tests based on GoogleTest for some ROS repos (PH-430)
 - Added Makefile commands for executing Automated Googletests (PH-430)
### Changed
 - Created patterns folder in ais_led_strip_controller package (PH-430)
 - Moved extra srv and msgs to phoenix_msgs (PH-430)
 - Jenkins support (PH-357)
 - Updated ais_led_strip_controller to refresh quicker (PH-359)
### Fixed
 - Robot reset pose when starting a new mission (PH-468)
## [0.4.0] - 2021-11-09
### Added
- Robot publishes status to frontend, see https://ais-ugv2.atlassian.net/wiki/spaces/PHOENIX/pages/3064397960/AIS+state+machine (PH-346)
 - Added manual treeatment, manual operation, and teleoperation states. See https://ais-ugv2.atlassian.net/wiki/spaces/PHOENIX/pages/2962554881/Interfaces+and+Operations+Instructions and related pages for documentation (PH-339)
 - When robot is running a mission or in manual treatement, it will stop within 1.2m after dismounting the rails (PH-358)
### Changed
 - /set_robot_mode service is now required to change modes (manual treatment/manual operation/idle) (PH-339)

---

## [0.3.0] - 2021-10-28
### Added
 - Launch ais_job_manager node which handles creating/saving missions and in the future jobQueues (PH-327)
 - Expose CRU Hostname retrieval via "get_hostname" ROS service (Trigger) (PH-331)
 - added "/autonomy/get_version" Trigger service call in ais_state_machine (PH-345)
 - Robot publishes on-rail status always (PH-349)
 
### Changed
 - Updated default robot parameters, UV lamp to true and example_mission.csv updated (PH-388)

### Fixed
 - UV lamps now turn on in Initialization for UV Disinfection (PH-352)
 - LED indicator updates with UV intensity when UV lights are on (PH-387)
 - Robot does not perform stepping motion (PH-374)
 - Fix rosbridge websocket overwrite (PH-390)
 - Start mission hotfix in the state machine after merging (PH-358)(PH-339)(PH-346)
## [0.2.1] - 2021-10-06
### Changed
 - Mission default changed https://bitbucket.org/ais_admin/ais_state_machine/pull-requests/29
 - test/example_mission.csv
## [0.2.0] - 2021-10-04
### Added
 - /start_named_mission accepts a string and will start a mission (located in MISSION_DIR) with that name. Note: the ".csv" portion of the file name should be omitted (PH-254)
 - /list_missions service will return a vector of string with all the .csv filenames located in MISSION_DIR (PH-254)
 - UV Robot will turn off UV lamps (if UV lamps requested) in return home and Complete state (PH-322)
 - Added LED strip controller to display LED status information. See https://ais-ugv2.atlassian.net/wiki/spaces/PHOENIX/pages/3065380975/LED+Status+Indicator (PH-284) 
### Changed
 - Robot now uses /encoder_localization_node/pose_1_d instead of /wheel_localization_node/pose_1_d for localization (PH-309)
 - updated max_lamp_number in ais_dimming_module to 8 from 6 (PH-336)
### Fixed
 - /start_named_mission accepts a string and will start a mission (located in MISSION_DIR) with that name. Note: the ".csv" portion of the file name should be omitted (PH-254)
 - /list_missions service will return a vector of string with all the .csv filenames located in MISSION_DIR (PH-254)
 - Set mission abort flag to false to fix infinite loop when calling stop_mission service (PH-311)
 - Now generates missing srv message for listMission (PH-341)

---

## [0.1.1] - 2021-09-21
### Added
 - amr_localization supports a single encoder topic with a std_msgs::Int64 message using the "Encoder" type (PH-209)
 - bumper detection when returning home in RETURN_TO_RAIL_HOME state (PH-282)
 - documentation for classes (PH-282) 
### Changed
 - "Encoder" type has been repaced by "MultiEncoder" type to allow for using a std_msgs::Int64 message on the encoder topic with the "Encoder" type (amr_localization PH-209)
 - Changed ais_state_machine to use encoder_localization_node for localization (PH-209)
### Fixed

---
## **Latest Released Version** 
## [0.1.0] - 2021-09-16
### Added
 - Refactored State Machine and added bumper detection functionality (PH-216)
 - Robot uses /opt/phoenix1/share/ais_state_machine/test/example_mission.csv to load a mission from file when the node is launched (PH-189)
### Changed
 - Safety node is not launched for feasibility test at Houwelings (PH-239)
 - Safety node now publishes on /embedded/cmd_vel (PH-239)
 - Removed relay topics, embedded is publishing with the predefined namespace /embedded (PH-146)
 - Dimming module (ais_dimming_module) fix for running in pipeline (PH-239)
 - Merged kpu visit for the ais_state_machine (PH-151)
### Fixed
 - /stop_mission service now stops the mission correctly (PH-236)
## [0.0.1] - 2021-09-07
### Added
Initial working image containing submodules on:

  - robot_ws
    -  ais_safety --> branch feature/PH-31-houwelings-nursery (PH-31)
    -  ais_state_machine (PH-135)
    -  amr_localization (PH-120)
    -  phoenix1_bringup
    -  phoenix1_parameters
    -  phoenix_vehicle_model  
    -  phoenix_visualization  

  -  simulation_ws
     -  phoenix1_simulation

  -  binder/underlay_ws
     -  ira_laser_tools
     -  ais-bigtop-rosbags
     -  xsens_ros_mti_driver (ROB-2462)  
  
  - Branch under ais_safety module removing dependencies on ais_brain and uv_brain (PH-31)
  - Release the first simulation environment for phoenix (PH-118)
  - Setting up IMU tests for speed resonance frequency, updating IMU firmware and software to MT Software Suite 2021.0 (ROB-2462)
  - Added ais_state_machine repo that provides foundations for robot planning, Design state machine, current states are Idle, Complete, Run_Mission (PH-135)
  - Added start mission service /start_mission (PH-166)
  - Default localization set to /wheelodom
  - Localize the robot on the rails with wheel odometry (PH-120)
  - Adding encoder rail simulator (PH-145)
  - Deploy Phoenix inside binder folder
---
## **Previous Versions**
## [Unreleased] - 2021-07-06

### Added
 - Preparation for KPU

### Changed
 - Forked from UV-binder

### Fixed
## **Unreleased** 
