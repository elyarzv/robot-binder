/* 
    AIS CONFIDENTIAL
*/

#ifndef ASM_ROBOT_VELOCITY_CONTROLLER_H_
#define ASM_ROBOT_VELOCITY_CONTROLLER_H_

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <string>


using std::string;

namespace ais_state_machine
{



class RobotVelocityController{
public:
    RobotVelocityController(ros::NodeHandle&, string);
    ~RobotVelocityController();
    void sendVelocityCmd(float lin_x, float lin_y, float ang_speed);
    void setMaxSpeed(float, float);
    inline void setMaxLinearSpeed(float);
    inline void setMaxAngularSpeed(float);

private:

    void initialize();
    geometry_msgs::Twist limitCmdSpeed(geometry_msgs::Twist);

    ros::NodeHandle nh_;
    ros::Publisher cmd_pub_;

    geometry_msgs::Twist vel_cmd_;

    float max_linear_speed_;
    float max_angular_speed_;

    string topic_name_;


}; // End class declaration

} // end ais_state_machine namespace

#endif // END ASM_ROBOT_VELOCITY_CONTROLLER_H_
