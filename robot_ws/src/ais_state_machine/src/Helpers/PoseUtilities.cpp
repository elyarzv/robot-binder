#include "ais_state_machine/Helpers/PoseUtilities.h"

using namespace ais_state_machine;

PoseUtilities::PoseUtilities()
{
    // Constructor
}

PoseUtilities::~PoseUtilities()
{
}

float PoseUtilities::getDistanceBetweenPoses(geometry_msgs::Pose p1, geometry_msgs::Pose p2)
{
    float x_diff_squared = pow(p1.position.x - p2.position.x, 2);
    float y_diff_squared = pow(p1.position.y - p2.position.y, 2);
    float z_diff_squared = pow(p1.position.z - p2.position.z, 2);
    return pow(x_diff_squared + y_diff_squared + z_diff_squared, 0.5);
}
/*
tf::Transform oneDNode::computeOdomToOrigin(){
    tf::Transform transform;
    tf::Quaternion q(
        current_odom_.pose.pose.orientation.x,
        current_odom_.pose.pose.orientation.y,
        current_odom_.pose.pose.orientation.z,
        current_odom_.pose.pose.orientation.w);
    tf::Vector3
v(current_odom_.pose.pose.position.x,current_odom_.pose.pose.position.y,current_odom_.pose.pose.position.z);
    transform.setOrigin(v);
    transform.setRotation(q);
    return transform.inverse();
}
*/
tf2::Transform PoseUtilities::getTransformFromOriginToPose(geometry_msgs::Pose p)
{
    tf2::Transform t;
    tf2::Vector3 origin_vec3(p.position.x, p.position.y, p.position.z);
    tf2::Quaternion quat(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
    t.setOrigin(origin_vec3);
    t.setRotation(quat);
    return t;
}

tf2::Transform PoseUtilities::getTransformFromPoseToOrigin(geometry_msgs::Pose p)
{
    tf2::Transform t;
    tf2::Vector3 origin_vec3(p.position.x, p.position.y, p.position.z);
    tf2::Quaternion quat(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
    t.setOrigin(origin_vec3);
    t.setRotation(quat);
    return t.inverse();
}

