/* 
    AIS CONFIDENTIAL
    Author: Michael Wrock (m.wrock@ai-systems.ca)
*/

#ifndef ASM_ROBOT_STATUS_MANAGER_H_
#define ASM_ROBOT_STATUS_MANAGER_H_

#define MIN_ENCODER_SPEED 0.01
#define MIN_WHEEL_SPEED 0.01
#define SPEED_CALCULATION_FREQUENCY 0.1
#define MIN_DIMMER_ADJUST_TIME 2
#include "ros/ros.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt8.h"
#include "std_srvs/Trigger.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "phoenix_msgs/LedPattern.h"
#include "ais_state_machine/Enums/robot_status.h"
#include "ais_state_machine/Enums/phoenix_state_enums.h"
#include "ais_state_machine/Helpers/MissionGoalManager.h"
#include "ais_state_machine/Helpers/RobotBumperHandler.h"
#include "ais_state_machine/Helpers/RailStatusManager.h"
#include "ais_state_machine/Helpers/RobotDiagnosticAggregator.h"
#include "ais_state_machine/Helpers/UVLampController.h"
#include "ais_state_machine/json.h"
#include <memory>
#include <vector>
#include <numeric>
#include <string>
// #include "ais_state_machine/StateMachineManager.h"

namespace ais_state_machine
{

class StateMachineManager; // forward declaration
class RobotDiagnosticAggregator;
class RobotStatusManager
{
    public:
        RobotStatusManager(ros::NodeHandle&, ros::NodeHandle&, std::shared_ptr<MissionGoalManager> mission_goal_manager_,std::shared_ptr<RobotBumperHandler> bumper_handler_ptr_,std::shared_ptr<RailStatusManager> rail_status_ptr_,std::shared_ptr<UVLampController> uv_lamp_controller_ptr_);
        ~RobotStatusManager();
        void update(); //Call this function to send the latest LED strip animation
        void setRobotStatus(RobotStatus status); //Public function to change the robot status
        bool isDimmerAlive(); //Check function for dimmer module, made public for other classes to use
        void resetPose(); //Resets pose estimate
        bool isEStopPressed(); //Check if estop was pressed during autonomous mode
        bool isRobotOnRails(); 
        std_msgs::Float32 getCurrentPose();
        float getSoC();
        bool interruptStateMachine();
        bool abortMission();
        bool lowRobotBattery(); //Check is battery SoC is below LOW_BATTERY+ESTIMATED_BATTERY_USAGE
        bool batteryRecharged(); //Check if battery is above MINIMUM_BATTERY
        bool isBatteryCritical(); //Check is battery SoC is below CRITICAL_BATTERY
        void getUpdatedParams();
        void setMissionAbort(bool);
        void setRobotOnRails(bool);
        void setErrorCode(std::string);
        bool isRobotOutOfTheWay(ais_state_machine::StateTypes);
        bool willRobotCollide();
        geometry_msgs::Twist getManualCmdVel();
        geometry_msgs::Twist getRollerCmdVel();
        geometry_msgs::Twist getTeleopCmdVel();
        geometry_msgs::Twist getNavigationCmdVel();
        RobotStatus getRobotStatus();
        bool off_rail_detection_autonomous;
        bool off_rail_detection_semi_autonomous;
        bool off_rail_detection;
        string safety_status; //So other classes can check the safety status
        shared_ptr<RobotDiagnosticAggregator> robot_diagnostic_;
        void setMissionStartTime(ros::Time start_time);

    private:
        void DimmerCb(const std_msgs::UInt16::ConstPtr&); //Callback to ensure the dimmer teensy communication is online
        void DimmerCmdCb(const std_msgs::ByteMultiArray::ConstPtr&); //Callback to ensure the dimmer is publishing what was asked
        void AuxCb(const std_msgs::Bool::ConstPtr&); //Callback to monitor AUX power
        void SafetyStatusCb(const std_msgs::String::ConstPtr&); //Callback to monitor status of safety node
        void SoCCb(const std_msgs::UInt8::ConstPtr&); //Callback to monitor battery state of charge
        std::string getRobotStatusString(); //See https://ais-ugv2.atlassian.net/wiki/spaces/PHOENIX/pages/3064397960/AIS+state+machine
        std::string getMissionStatusString(); //See https://ais-ugv2.atlassian.net/wiki/spaces/PHOENIX/pages/3064397960/AIS+state+machine
        void poseCb(const std_msgs::Float32ConstPtr&);
        void WheelodomCb(const nav_msgs::OdometryConstPtr&);
        void cmd_velCb(const geometry_msgs::TwistConstPtr&);
        void roller_cmd_velCb(const geometry_msgs::TwistConstPtr&);
        void manual_cmd_velCb(const geometry_msgs::TwistConstPtr&);
        void teleop_cmd_velCb(const geometry_msgs::TwistConstPtr&);
        void navigation_cmd_velCb(const geometry_msgs::TwistConstPtr&);
        bool abortCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        float getTimeRemainingForStart();

        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        ros::Subscriber uv_dimmer_sub_;
        ros::Subscriber dimmer_cmd_sub_;
        ros::Subscriber aux_power_sub_;
        ros::Subscriber battery_soc_sub_;
        ros::Subscriber safety_status_sub_;
        ros::Publisher robot_status_pub_;
        ros::Publisher diagnostic_error_pub_;
        ros::Subscriber wheel_speed_sub_;
        ros::Subscriber cmd_vel_sub_;
        ros::Subscriber manual_cmd_vel_sub_;
        ros::Subscriber roller_cmd_vel_sub_;
        ros::Subscriber teleop_cmd_vel_sub_;
        ros::Subscriber navigation_cmd_vel_sub_;
        ros::Subscriber pose_subscriber_;
        ros::ServiceServer abort_mission_service_;
        ros::ServiceClient led_strip_; //Sets the LED strip pattern 
        ros::ServiceClient reset_pose_service_;
        std::string localization_source;
        double last_time_dimmer_pub_; //Time container to keep track of how long ago the last dimmer module message arrived 
        double last_time_dimmer_cmd_pub_; //Time container to keep track of how long ago the last dimmer module message was sent 
        double dimmer_time_tol_; //Tolerance for the rate at whcih the dimmer module publishes
        float battery_soc_; //Battery state of charge
        bool aux_status_; //Persistant storage of aux status
        bool estop_pressed_; //Storage for E-Stop flag
        bool rail_status_known;
        bool errors_cleared_; //Flag so that the animation will only reset if an error has occurred and needs to be cleared
        //This flag gets set to true when the abortCB is called, 
        // and gets set back to false (using setMissionAborted()) once runMission has aboted the mission
        bool abort_mission_;
        std_msgs::Float32 curr_robot_rail_pose_;
        std_msgs::Float32 prev_robot_rail_pose_;
        std_msgs::UInt16 curr_uv_cmd_byte_;
        std_msgs::ByteMultiArray curr_uv_cmd_;
        std::vector<float> wheel_speed_;
        std::vector<float> encoder_speed_;
        std::vector<float> cmd_vel_;
        ros::Time last_pose_message_;
        ros::Time last_manual_message_;
        ros::Time last_roller_message_;
        ros::Time last_teleop_message_;
        ros::Time last_navigation_cmd_vel_message_;
        ros::Time mission_start_time_;
        geometry_msgs::Twist manual_cmd_vel_;
        geometry_msgs::Twist roller_cmd_vel_;
        geometry_msgs::Twist teleop_cmd_vel_;
        geometry_msgs::Twist navigation_cmd_vel_;
        diagnostic_msgs::DiagnosticArray error_code_;
        RobotStatus robot_status_; //Persistant stroage of the robot's current status
        RobotStatus prev_robot_status_; //Keep track of what the least status was so we only update the animation if the status changes
        shared_ptr<MissionGoalManager> mission_goal_manager_; //Pointer so we can access goal intensity
        shared_ptr<RobotBumperHandler> bumper_handler_ptr_; //Pointer so we can check for bumper hits
        shared_ptr<RailStatusManager> rail_status_ptr_; //Pointer to check proximeter status
        shared_ptr<UVLampController> uv_lamp_controller_ptr_; //Pointer to check average uv intensity

};
} // end of namespace ais_state_machine

#endif // END ASM_ROBOT_STATUS_MANAGER_H_
