#include "ais_state_machine/Helpers/RobotVelocityController.h"

using namespace ais_state_machine;

RobotVelocityController::RobotVelocityController(ros::NodeHandle& nh, string topic_name) 
                                                : nh_(nh), topic_name_(topic_name)
{
    cmd_pub_ = nh.advertise<geometry_msgs::Twist>(topic_name.c_str(), 1000);

    initialize();

}


RobotVelocityController::~RobotVelocityController()
{
}

void RobotVelocityController::initialize()
{
    setMaxSpeed(0.78, 1.0);
}

void RobotVelocityController::sendVelocityCmd(float lin_speed_x, float lin_speed_y, float ang_speed)
{
    geometry_msgs::Twist cmd;
    cmd.linear.x = lin_speed_x;
    cmd.linear.y = lin_speed_y;
    cmd.angular.z = ang_speed;
    
    cmd = limitCmdSpeed(cmd);
    cmd_pub_.publish(cmd);
}

void RobotVelocityController::setMaxSpeed(float max_lin_speed, float max_ang_speed)
{
    setMaxLinearSpeed(max_lin_speed);
    setMaxAngularSpeed(max_ang_speed);
}

void RobotVelocityController::setMaxLinearSpeed(float max_lin_speed)
{
    max_linear_speed_ = max_lin_speed;
}

void RobotVelocityController::setMaxAngularSpeed(float max_ang_speed)
{
    max_angular_speed_ = max_ang_speed;
}

geometry_msgs::Twist RobotVelocityController::limitCmdSpeed(geometry_msgs::Twist vel)
{   
    geometry_msgs::Twist cmd = vel;
    // Redo this as function?
    if (fabs(cmd.linear.x) > max_linear_speed_) 
    {
        cmd.linear.x = max_linear_speed_;
        if (vel.linear.x < 0) 
        {
            cmd.linear.x *= -1.0;
        }
    } else if (fabs(cmd.linear.x != 0.0) && fabs(cmd.linear.x) < 0.05)
    {
        cmd.linear.x = 0.0;
    }

    // Kinda incorrect, should patch this later
    if (fabs(cmd.linear.y) > max_linear_speed_) 
    {
        cmd.linear.y = max_linear_speed_;
        if (vel.linear.y < 0) 
        {
            cmd.linear.y *= -1.0;
        }
    } else if (fabs(cmd.linear.y != 0.0) && (fabs(cmd.linear.y) < 0.05))
    {
        cmd.linear.y = 0.0;
    }

    if (fabs(cmd.angular.z) > max_angular_speed_)
    {
        cmd.angular.z = max_angular_speed_;
        if (vel.angular.z < 0) 
        {
            cmd.angular.z *= -1.0;
        }
    } else if (fabs(cmd.angular.z != 0.0) && fabs(cmd.angular.z) < 0.05)
    {
        cmd.angular.z = 0.0;
    }
    return cmd;
}
