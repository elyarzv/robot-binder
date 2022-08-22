#ifndef AIS_SAFETY_H
#define AIS_SAFETY_H
 
#include <topic_tools/MuxSelect.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>

#include <boost/thread.hpp>
//#include <std_msgs/UInt8.h>
#include <atomic>
#include <memory>

#include <ais_safety/DynamicParamsConfig.h>

class safetyNode
{
protected:

  ros::NodeHandle* nh_;
  ros::Subscriber safety_mode_sub;
  ros::Subscriber teleop_sub;
  ros::Subscriber lidar_sub;
  ros::Subscriber cmd_vel_sub;
  ros::Publisher safety_status_pub;
  ros::Publisher cmd_vel_pub;
  ros::Publisher footprint_pub;
  ros::Publisher throttle_pub;
  geometry_msgs::Twist brain_cmd_vel;
  geometry_msgs::Twist zero_cmd_vel;
  ros::Time last_cmd_message_;

public:

  safetyNode(ros::NodeHandle* nodeHandle);

  ~safetyNode()
  {
  }
  enum  footprint {fwd, rev, left, right};
  std::vector<geometry_msgs::Point32> fwdFootprint, revFootprint, leftFootprint, rightFootprint, leftRotationFootprint, rightRotationFootprint;
  std::vector<geometry_msgs::Point32> fwdFootprintManual, revFootprintManual, leftFootprintManual, rightFootprintManual, leftRotationFootprintManual, rightRotationFootprintManual;
  std::vector<geometry_msgs::Point32> fwdFootprintTeleop, revFootprintTeleop, leftFootprintTeleop, rightFootprintTeleop, leftRotationFootprintTeleop, rightRotationFootprintTeleop;
  std::vector<geometry_msgs::Point32> fwdFootprintThrottle, revFootprintThrottle, leftFootprintThrottle, rightFootprintThrottle, leftRotationFootprintThrottle, rightRotationFootprintThrottle;
  enum  cmd_vel_mux {brain_cmd_vel_mux, zero_cmd_vel_mux, reduced_cmd_vel_mux};
  geometry_msgs::Twist current_cmd_vel;
  int current_mux;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  int current_state_;
  //void safetyCB(const std_msgs::UInt8::ConstPtr& status);
  void cmd_velCB(const geometry_msgs::Twist::ConstPtr& cmd_vel);
  geometry_msgs::Twist clamp_vel(geometry_msgs::Twist &cmd_vel, float clamp_x, float clamp_y, float clamp_angle);
  void lidarCB(const sensor_msgs::LaserScan::ConstPtr& scan);
  void makeFootprints();
  void checkFootprints();
  void getLeftRotationFootprint(std::vector<geometry_msgs::Point32> &verts);
  void getRightRotationFootprint(std::vector<geometry_msgs::Point32> &verts);
  void publish_cmd_vel();
  void getFwdFootprint(std::vector<geometry_msgs::Point32> &verts);
  void getRightFootprint(std::vector<geometry_msgs::Point32> &verts);
  void getLeftFootprint(std::vector<geometry_msgs::Point32> &verts);
  void getRevFootprint(std::vector<geometry_msgs::Point32> &verts);
 	bool isPointInPolygon(geometry_msgs::Point32 _point, std::vector<geometry_msgs::Point32> &verts);
 	bool checkForPointsInFootprint(sensor_msgs::PointCloud scan, std::vector<geometry_msgs::Point32> &footprint, std::vector<geometry_msgs::Point32> &throttle_footprint);

  typedef ais_safety::DynamicParamsConfig dyn_cfg_t;
  void dynamicReconfigureCallback(dyn_cfg_t &config, uint32_t level);

private:

  bool collision_;
  float FRONT;
  float LEFT;
  float RIGHT;
  float REAR;
  float MANUAL_PADDING_X;
  float MANUAL_PADDING_Y;
  float ROTATION_X;
  float ROTATION_Y;
  float TELEOP_PADDING_X;
  float TELEOP_PADDING_Y;
  float THROTTLE_PADDING_X;
  float THROTTLE_PADDING_Y;
  float AUTONOMOUS_PADDING_X;
  float AUTONOMOUS_PADDING_Y;
  std_msgs::String safety_string;

  std::shared_ptr< dynamic_reconfigure::Server<dyn_cfg_t> > dynamic_reconfigure_server_;
  std::atomic<bool> forward_safety_;
  std::atomic<bool> backward_safety_;

};

#endif
