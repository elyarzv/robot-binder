/*
 *   Author: Abdullah Mohiuddin
 *   Email: a.mohiuddin@ai-systems.ca
 *
*/

#include <amr_embedded_simulator/embeded_simulator.h>

EmbededSimulator::EmbededSimulator(ros::NodeHandle nh) :
		nh_(nh)
{
  std::string aux_power_status;
  std::string aux_power_switch;          
  std::string au_relay_status; 
  std::string au_relay_switch;  
  std::string au_battery_current; 
  std::string battery_state_of_charge;   
  std::string au_battery_voltage;       
  std::string au_lamp_colour;     
  std::string au_tempCRU;     
  std::string au_encoder_count;  
  std::string au_get_firmware_version; 
  std::string au_get_firmware_version_dimming_module; 
  std::string uv_lamp_dimmer_values;  
  std::string lamp_status;  
  std::string rail_encoder; 
  std::string rail_encoder_source;
  std::string bumper_front_collision_status; 
  std::string bumper_back_collision_status; 
  int counts_per_meter_param;
  std::string left_front_wheel_contact;
  std::string left_rear_wheel_contact;
  std::string right_front_wheel_contact;
  std::string right_rear_wheel_conact;
  std::string proximity_sensor;

  nh_.getParam("left_front_wheel_contact" , left_front_wheel_contact);
  nh_.getParam("left_rear_wheel_contact" , left_rear_wheel_contact);
  nh_.getParam("right_front_wheel_contact" , right_front_wheel_contact);
  nh_.getParam("right_rear_wheel_conact" , right_rear_wheel_conact);
  nh_.getParam("aux_power_status" , aux_power_status);
  nh_.getParam("aux_power_switch" , aux_power_switch);
  nh_.getParam("au_relay_status" , au_relay_status);
  nh_.getParam("au_relay_switch" , au_relay_switch);
  nh_.getParam("au_battery_current" , au_battery_current);
  nh_.getParam("battery_state_of_charge" ,battery_state_of_charge);
  nh_.getParam("au_battery_voltage" , au_battery_voltage);
  nh_.getParam("au_lamp_colour" , au_lamp_colour);
  nh_.getParam("au_tempCRU" , au_tempCRU);
  nh_.getParam("au_encoder_count" , au_encoder_count);
  nh_.getParam("au_get_firmware_version" ,au_get_firmware_version);
  nh_.getParam("au_get_firmware_version_dimming_module" , au_get_firmware_version_dimming_module);
  nh_.getParam("uv_lamp_dimmer_values" , uv_lamp_dimmer_values);
  nh_.getParam("lamp_status" , lamp_status);
  nh_.getParam("rail_encoder" , rail_encoder);
  nh_.getParam("rail_encoder_source_topic_name" , rail_encoder_source);
  nh_.getParam("bumper_front_collision_status" , bumper_front_collision_status);
  nh_.getParam("bumper_back_collision_status" , bumper_back_collision_status);
  nh_.getParam("encoder_counts_per_meter", counts_per_meter_param);
  nh_.getParam("proximity_sensor", proximity_sensor);

  ROS_INFO("Embedded simulator recieved the params");
  au_batteryVoltage_publisher_ = nh_.advertise<std_msgs::Float32>(au_battery_voltage, 10);
  au_batterySoC_publisher_ = nh_.advertise<std_msgs::UInt8>(battery_state_of_charge, 10);
  au_AuxPowerStatus_publisher_ = nh_.advertise<std_msgs::Bool>(aux_power_status, 10);
  backBumperStatus_publisher_ = nh_.advertise<std_msgs::Bool>( bumper_back_collision_status, 10);
  frontBumperStatus_publisher_ = nh_.advertise<std_msgs::Bool>(bumper_front_collision_status, 10);  
  au_systemLoadAmps_publisher_ = nh_.advertise<std_msgs::Float32 >("/au_systemLoadAmps", 10);
  au_tempCRU_publisher_ = nh_.advertise<std_msgs::Float32 >(au_tempCRU, 10);
  wifi_signal_status_publisher_ = nh_.advertise<std_msgs::UInt8 >("/wifi_signal_status", 10);  
  au_RelayStatus_publisher_ = nh_.advertise<std_msgs::Bool >(au_relay_status, 10);
  lamp_status_publisher_=nh_.advertise<std_msgs::UInt16>(lamp_status, 10);
  cmd_vel_publisher_=nh_.advertise<geometry_msgs::Twist >("/cmd_vel", 10);
  rail_encoder_=nh_.advertise<std_msgs::Int64>(rail_encoder, 10);
  proximity_sensor_ = nh_.advertise<std_msgs::UInt8>(proximity_sensor, 10);

  frontcontactstate_subscriber_ = nh_.subscribe("/embedded/bumper/readfront", 10, &EmbededSimulator::frontbumperCallback, this);
  backcontactstate_subscriber_ = nh_.subscribe("/embedded/bumper/readback", 10, &EmbededSimulator::backbumperCallback, this);
  left_front_wheel_contact_subscriber= nh_.subscribe(left_front_wheel_contact, 10, &EmbededSimulator::left_front_wheel_contactCallback, this);
  left_rear_wheel_contact_subscriber= nh_.subscribe(left_rear_wheel_contact, 10, &EmbededSimulator::left_rear_wheel_contactCallback, this);
  right_front_wheel_contact_subscriber= nh_.subscribe(right_front_wheel_contact, 10, &EmbededSimulator::right_front_wheel_contactCallback, this);
  right_rear_wheel_conact_subscriber= nh_.subscribe(right_rear_wheel_conact, 10, &EmbededSimulator::right_rear_wheel_conactCallback, this);
  set_uv_lamp_dimmer_values_subscriber_ = nh_.subscribe(uv_lamp_dimmer_values, 10, &EmbededSimulator::setUVLampDimmerValues, this);
  cmdVel_subscriber_ = nh_.subscribe("/cmd_vel_embedded", 10, &EmbededSimulator::velCallback, this);
  initialpose_subscriber_ = nh_.subscribe("/initialpose", 10, &EmbededSimulator::initPoseCallback, this);
  robot_status_subscriber_ = nh_.subscribe("/ais_brain/robot_status", 10, &EmbededSimulator::modeCb, this);
  au_auxPowerOn_subscriber_ = nh_.subscribe(aux_power_switch, 10, &EmbededSimulator::auxPowerOnCallback, this);
  au_powerUWB_subscriber_ = nh_.subscribe("/au_powerUWB", 10, &EmbededSimulator::powerUWBCallback, this);
  au_setFanSpeed_subscriber_ = nh_.subscribe("/au_setFanSpeed", 10, &EmbededSimulator::setFanSpeedCallback, this);
  au_relay_subscriber_ = nh_.subscribe(au_relay_switch, 10, &EmbededSimulator::setRelayCallback, this);
  odom_subscriber_ = nh_.subscribe( rail_encoder_source, 1, &EmbededSimulator::odomCallback, this);

  service_battery_reset = nh_.advertiseService("reset_battery", &EmbededSimulator::reset_battery, this);
  service_firmware_version = nh_.advertiseService("/embedded/firmware_version/base", &EmbededSimulator::service_firmware_versionCallback, this);
  service_dimming_firmware_version = nh_.advertiseService("/embedded/firmware_version/dimming_module", &EmbededSimulator::service_dimming_firmware_versionCallback, this);

  battery_voltage.data=MAX_VOLTAGE-mileage.data*VOLTAGE_FACTOR;
  battery_SoC.data = 100*((battery_voltage.data-LOW_VOLTAGE)/(MAX_VOLTAGE-LOW_VOLTAGE));
  prev_x=0;
  prev_y=0;
  counts_per_meter_= counts_per_meter_param;
  power_main.data=true;
  uv_light_status.data=false;
  ROS_INFO("Starting the embedded simulator");
}

void EmbededSimulator::odomCallback(const nav_msgs::Odometry& odom)
{
  if (on_rail_status.data==true)
  { 
    // This code is to take the pose data from the odometry and convert back into one encoder value
    new_x=odom.pose.pose.position.x;
    new_y=odom.pose.pose.position.y;
    diff_x=new_x-prev_x;
    diff_y=new_y-prev_y;
    dif_count=abs(sqrt((diff_x*diff_x+diff_y*diff_y)))*counts_per_meter_; 
    dif_count=(abs(sqrt(new_x*new_x+new_y*new_y))-abs(sqrt(prev_x*prev_x+prev_y*prev_y)))*counts_per_meter_;
    commulative_count=commulative_count+dif_count;
    rail_encoder_values.data= (commulative_count);
    prev_x=new_x;
    prev_y=new_y;
  }  
  ROS_INFO_THROTTLE(120, "Rail Encoder values %f",odom.pose.pose.position.x*counts_per_meter_);
}

void EmbededSimulator::frontbumperCallback(const gazebo_msgs::ContactsState& msg)
{
  if (msg.states.size()>0)
  {
    if (msg.states[0].collision1_name=="/::base_footprint::base_footprint_fixed_joint_lump__front_bumper_collision_collision_2" || msg.states[0].collision2_name=="/::base_footprint::base_footprint_fixed_joint_lump__front_bumper_collision_collision_2")
    {
        ROS_INFO("Front bumper collision");
        front_bumper_status.data=true;
    }
  }
   else
  {
      front_bumper_status.data=false;      
  }
}

void EmbededSimulator::backbumperCallback(const gazebo_msgs::ContactsState& msg)
{
  if (msg.states.size()>0)
  {
    if (msg.states[0].collision1_name=="/::base_footprint::base_footprint_fixed_joint_lump__back_bumper_collision_collision_1" || msg.states[0].collision2_name=="/::base_footprint::base_footprint_fixed_joint_lump__back_bumper_collision_collision_1")
    {
        ROS_INFO("Back bumper collision");
        back_bumper_status.data=true;
    }
  }
  else
  {
      back_bumper_status.data=false;      
  }  
}

void EmbededSimulator::left_front_wheel_contactCallback(const gazebo_msgs::ContactsState& msg)
{
  if (msg.states.size()>0)
  {
    if (msg.states[0].collision1_name=="ground_plane::link::collision" || msg.states[0].collision2_name=="ground_plane::link::collision")
    {
      ROS_INFO_THROTTLE(50, "Back on ground");
      on_rail_status.data = false;
    }
    else
    {
      on_rail_status.data = true;
    }  
  }
}
void EmbededSimulator::left_rear_wheel_contactCallback(const gazebo_msgs::ContactsState& msg)
{
  if (msg.states.size()>0)
  {
    if (msg.states[0].collision1_name=="ground_plane::link::collision" || msg.states[0].collision2_name=="ground_plane::link::collision")
    {
      ROS_INFO_THROTTLE(50, "Back on ground");
      on_rail_status.data = false;
      proximity_sensor_values.data = 0;
    }
  else
    {
      on_rail_status.data = true;
      proximity_sensor_values.data = 1;
    }  
  }
}
void EmbededSimulator::right_front_wheel_contactCallback(const gazebo_msgs::ContactsState& msg)
{
  if (msg.states.size()>0)
  {
    if (msg.states[0].collision1_name=="ground_plane::link::collision" || msg.states[0].collision2_name=="ground_plane::link::collision")
    {
        ROS_INFO_THROTTLE(50,"Back on ground");
        on_rail_status.data=false;
    }
    else
      {
          on_rail_status.data=true;      
      }  
  }
}
void EmbededSimulator::right_rear_wheel_conactCallback(const gazebo_msgs::ContactsState& msg)
{
  if (msg.states.size()>0)
  {
    if (msg.states[0].collision1_name=="ground_plane::link::collision" || msg.states[0].collision2_name=="ground_plane::link::collision")
    {
        ROS_INFO_THROTTLE(50,"Back on ground");
        on_rail_status.data=false;
    }
    else
      {
          on_rail_status.data=true;      
      }  
  }
}

void EmbededSimulator::setUVLampDimmerValues(const std_msgs::ByteMultiArray& msg)
{
  ROS_INFO_THROTTLE(20, "Message recieved to set the UV lamp dimmer values, the values are:");
  for (int i = 0; i <= msg.data.size(); i = i + 1) 
  {
  ROS_INFO_THROTTLE(20, " %i ", msg.data[i]);
  }

}

bool EmbededSimulator::reset_battery(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
  mileage.data=0;
  ROS_INFO("Resetting the battery");
  res.success = true;
  return true;
}

bool EmbededSimulator::service_firmware_versionCallback(std_srvs::Trigger::Request& req,  std_srvs::Trigger::Response& res)
{
  res.success = true;
	res.message = FIRMWARE_VERSION;
  return true;
}
bool EmbededSimulator::service_dimming_firmware_versionCallback(std_srvs::Trigger::Request& req,  std_srvs::Trigger::Response& res)
{
  res.success = true;
	res.message = DIMMING_FIRMWARE_VERSION;
  return true;
}

void EmbededSimulator::callpublishers()
{
  au_RelayStatus_publisher_.publish(uv_light_status);
  au_AuxPowerStatus_publisher_.publish(power_main);
  wifi_status.data=(rand()%100)+1;
  wifi_signal_status_publisher_.publish(wifi_status);
  au_batteryVoltage_publisher_.publish(battery_voltage);
  cmd_vel_publisher_.publish(cmd_vel_transition);
  au_batterySoC_publisher_.publish(battery_SoC);
  backBumperStatus_publisher_.publish(back_bumper_status);
  frontBumperStatus_publisher_.publish(front_bumper_status);
  lamp_status_publisher_.publish(lamp_status);
  proximity_sensor_.publish(proximity_sensor_values);
  rail_encoder_.publish(rail_encoder_values);
}

// VelCalback recieves the cmd_vel for the embedded
void EmbededSimulator::velCallback(const geometry_msgs::Twist &msg)
{
  mileage.data =mileage.data + abs(msg.linear.x) * MILEAGE_FACTOR;
  battery_voltage.data=MAX_VOLTAGE-mileage.data*VOLTAGE_FACTOR;
  battery_SoC.data = 100*((battery_voltage.data-LOW_VOLTAGE)/(MAX_VOLTAGE-LOW_VOLTAGE));
  if (battery_voltage.data<=LOW_VOLTAGE)
  {
    power_main.data=false;
    uv_light_status.data=false;    
    cmd_vel_transition.linear.x=0;
    cmd_vel_transition.linear.y=0;
    cmd_vel_transition.linear.z=0;
    cmd_vel_transition.angular.x=0;
    cmd_vel_transition.angular.y=0;
    cmd_vel_transition.angular.z=0;
    ROS_FATAL("WARNING BATTERY LOW!!! ALL FUNCTIONS STOPPED");
    battery_voltage.data=LOW_VOLTAGE; 
    battery_SoC.data = 0;   
  }
  else
  {
    cmd_vel_transition.linear.x=msg.linear.x;
    cmd_vel_transition.linear.y=msg.linear.y;
    cmd_vel_transition.linear.z=msg.linear.z;
    cmd_vel_transition.angular.x=msg.angular.x;
    cmd_vel_transition.angular.y=msg.angular.y;
    cmd_vel_transition.angular.z=msg.angular.z; 
  }    
}
// 
void EmbededSimulator::initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{}
void EmbededSimulator::modeCb(const std_msgs::Int8& msg)
{}
// auxPowerOnCallback turns the system on when called with "true"
void EmbededSimulator::auxPowerOnCallback(const std_msgs::Bool& msg)
{
  if (msg.data==1)
    power_main.data=true;
  else
  {
    power_main.data=false;
    uv_light_status.data=false;
    for (int i = 0 ; i < 12; i++)
    {
      lamp_status.data |= (0 << i);
    }
  }
}

void EmbededSimulator::powerUWBCallback(const std_msgs::Bool &msg)
{}
void EmbededSimulator::setFanSpeedCallback(const std_msgs::UInt8 &msg)
{}

// setRelayCallback is used to turn the UV lights on or off. Its either "1" to turn it on or it is "0" to turn it off
void EmbededSimulator::setRelayCallback(const std_msgs::UInt8 &msg)
{
  if (msg.data==1)
   { 
    uv_light_status.data=true;
    for (int i = 0 ; i < 12; i++)
    {
      lamp_status.data |= (1 << i);
    }
   }
  else
    uv_light_status.data=false;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "EmbededSimulator");
  ros::NodeHandle nh("~");
  EmbededSimulator EmbededSimulator(nh);

 ros::Rate loop_rate(30);    
    while (ros::ok())
    {
      EmbededSimulator.callpublishers();
      ros::spinOnce();
      loop_rate.sleep();
    }
  ros::spin();
  return 0;
}
