/*
 *   Author: Abdullah Mohiuddin
 *   Email: a.mohiuddin@ai-systems.ca
 *
*/

#include <math.h>
#include <mutex>
#include <iostream>
#include <random>
#include <chrono>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int32.h>    //For the alarm set off by the dynamixel
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int64.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <gazebo_msgs/ContactsState.h>
#include <std_msgs/ByteMultiArray.h>
#include <cstdlib>
#define LOOPRATE 30

/**
 *
 * @brief class for embedded simulator
 */
class EmbededSimulator
{
public:
  /**
   *
   * @brief initializes all the publisher/subscriber and loads rosparams
   * @param nh ROS Node handler
   */
  EmbededSimulator(ros::NodeHandle nh);

   /**
   * @brief subscriber callbacks
   * @param none
   * @return none
   */
  void callpublishers();
  void velCallback(const geometry_msgs::Twist &msg);
  void initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &msg);
  void modeCb(const std_msgs::Int8 &msg);
  void auxPowerOnCallback(const std_msgs::Bool &msg);
  void powerUWBCallback(const std_msgs::Bool &msg);
  void setFanSpeedCallback(const std_msgs::UInt8 &msg);
  void setRelayCallback(const std_msgs::UInt8 &msg);
  void frontbumperCallback(const gazebo_msgs::ContactsState& msg);
  void backbumperCallback(const gazebo_msgs::ContactsState& msg);
  void left_front_wheel_contactCallback(const gazebo_msgs::ContactsState& msg);
  void left_rear_wheel_contactCallback(const gazebo_msgs::ContactsState& msg);
  void right_front_wheel_contactCallback(const gazebo_msgs::ContactsState& msg);
  void right_rear_wheel_conactCallback(const gazebo_msgs::ContactsState& msg);
  void setUVLampDimmerValues(const std_msgs::ByteMultiArray& msg);
  void odomCallback(const nav_msgs::Odometry& odom);
  bool reset_battery(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
  bool service_firmware_versionCallback(std_srvs::Trigger::Request& req,  std_srvs::Trigger::Response& res);
  bool service_dimming_firmware_versionCallback(std_srvs::Trigger::Request& req,  std_srvs::Trigger::Response& res);


private:
  ros::NodeHandle nh_;
  ros::ServiceServer service_battery_reset;
  ros::ServiceServer service_firmware_version;
  ros::ServiceServer service_dimming_firmware_version;
  ros::Subscriber cmdVel_subscriber_;
  ros::Subscriber initialpose_subscriber_;
  ros::Subscriber robot_status_subscriber_;
  ros::Subscriber au_auxPowerOn_subscriber_;
  ros::Subscriber au_powerUWB_subscriber_;
  ros::Subscriber au_setFanSpeed_subscriber_;
  ros::Subscriber au_relay_subscriber_;
  ros::Subscriber frontcontactstate_subscriber_;
  ros::Subscriber backcontactstate_subscriber_;
  ros::Subscriber left_front_wheel_contact_subscriber;
  ros::Subscriber left_rear_wheel_contact_subscriber;
  ros::Subscriber right_front_wheel_contact_subscriber;
  ros::Subscriber right_rear_wheel_conact_subscriber;
  ros::Subscriber set_uv_lamp_dimmer_values_subscriber_;
  ros::Subscriber odom_subscriber_;


  ros::Publisher au_batterySoC_publisher_;
  ros::Publisher cmd_vel_publisher_;
  ros::Publisher au_batteryVoltage_publisher_;
  ros::Publisher au_AuxPowerStatus_publisher_;
  ros::Publisher au_systemLoadAmps_publisher_;
  ros::Publisher au_tempCRU_publisher_;
  ros::Publisher au_RelayStatus_publisher_;
  ros::Publisher wifi_signal_status_publisher_;
  ros::Publisher backBumperStatus_publisher_;
  ros::Publisher frontBumperStatus_publisher_;  
  ros::Publisher lamp_status_publisher_;
  ros::Publisher rail_encoder_;
  ros::Publisher proximity_sensor_;

  std_msgs::Bool uv_light_status;
  std_msgs::Bool power_main;
  std_msgs::Float64 mileage;
  std_msgs::Float32 battery_voltage;
  std_msgs::UInt8  battery_SoC;
  std_msgs::UInt8 wifi_status;
  std_msgs::UInt16 lamp_status;  
  std_msgs::Int64 rail_encoder_values;
  std_msgs::UInt8 proximity_sensor_values;
  geometry_msgs::Twist cmd_vel_transition;
  std_msgs::Bool front_bumper_status;
  std_msgs::Bool back_bumper_status;
  std_msgs::Bool on_rail_status;
  double counts_per_meter_;
  double new_x;
  double new_y;
  double diff_x;
  double diff_y;
  double prev_x;
  double prev_y;
  double dif_count;
  double commulative_count;

  const std::string FIRMWARE_VERSION="SIMULATION v1.1.3";
  const std::string DIMMING_FIRMWARE_VERSION="SIMULATION v0.1.2";
  const double MILEAGE_FACTOR=0.033;
  const double VOLTAGE_FACTOR=0.01;
  const double MAX_VOLTAGE=29.4;
  const double LOW_VOLTAGE=23.5;

};
