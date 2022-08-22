#ifndef AMR_LOCALIZATION_H
#define AMR_LOCALIZATION_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Trigger.h>
#include <tf/tf.h>
#include <math.h>


class oneDNode
{
protected:

    ros::NodeHandle* nh_;
    ros::NodeHandle* private_nh_;

public:

    oneDNode(ros::NodeHandle* nodeHandle, ros::NodeHandle* private_nodeHandle);
    ~oneDNode()
    {
    }
    static float test_function(float output);
    void run();
    void reset();
    float computeDistance(std::string _topic_type);

private:

    ros::Subscriber odom_subscriber_;
    ros::ServiceServer reset_pose_service_;
    ros::Publisher pose_1d_pub_;
	std::string odometry_topic_;
    std::string pose_topic_;
    std::string topic_type_;
    int counts_per_meter_;
    float wheel_scaling_;
    std_msgs::Float32 pose_1d;
    std_msgs::Int64MultiArray prev_encoder_counts_;
    std_msgs::Int64MultiArray current_encoder_counts_;
    nav_msgs::Odometry  prev_odom_;
    nav_msgs::Odometry  current_odom_;
    tf::Transform odomToOrigin_;
    tf::Transform computeOdomToOrigin();
    void multiEncoderCb(const std_msgs::Int64MultiArrayConstPtr &odom);
    void encoderCb(const std_msgs::Int64ConstPtr &odom);
    void odomCb(const nav_msgs::OdometryConstPtr &odom);
    bool resetCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

};


#endif