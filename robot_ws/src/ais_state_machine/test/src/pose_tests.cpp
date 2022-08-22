#include "ais_state_machine/Helpers/PoseUtilities.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <gtest/gtest.h>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

using namespace ais_state_machine;

void printVec3(tf2::Vector3 tf)
{
    ROS_ERROR("Vector: %lf, %lf, %lf", tf.getX(), tf.getY(), tf.getZ());
}

TEST(PoseUtilities, OriginToPoseTransformationWithDifferentInitialRobotOrientation)
{
    // Values derived from real data with robot rotated ~-90 degrees to rail
    // Robot travelled ~0.815m backwards off rail

    geometry_msgs::Pose pose_final;
    pose_final.position.x = -2.337;
    pose_final.position.y = -3.718;
    pose_final.position.z = 0.0;
    pose_final.orientation.z = -0.6715699434280396;
    pose_final.orientation.w = -0.7409411072731018;

    geometry_msgs::Pose pose_start;
    pose_start.position.x = -2.2582900524139404;
    pose_start.position.y = -2.9038498401641846;
    pose_start.position.z = 0.0;
    pose_start.orientation.z = -0.6715699434280396;
    pose_start.orientation.w = -0.7409411072731018;

    geometry_msgs::Pose pose_diff;
    pose_diff.position.x = -0.815;
    pose_diff.orientation.w = 1.0;

    PoseUtilities p_util;
    tf2::Transform tf = p_util.getTransformFromOriginToPose(pose_start);
    tf2::Vector3 vec_start(pose_start.position.x, pose_start.position.y, pose_start.position.z);
    tf2::Vector3 vec_final(pose_final.position.x, pose_final.position.y, pose_final.position.z);
    tf2::Vector3 vec_diff(-0.815,0,0);
    tf2::Vector3 vec_zero;
    vec_start = vec_start;
    vec_final = vec_final;
    vec_diff = tf*vec_diff;
    vec_zero = tf*vec_zero;


    ASSERT_NEAR(vec_final.getX() - vec_diff.getX(), 0, 0.05);
    ASSERT_NEAR(vec_final.getY() - vec_diff.getY(), 0, 0.05);

    ASSERT_NEAR(vec_start.getX() - vec_zero.getX(), 0, 0.05);
    ASSERT_NEAR(vec_start.getY() - vec_zero.getY(), 0, 0.05);
    printVec3(vec_start);
    printVec3(vec_diff);
    printVec3(vec_final);
    printVec3(vec_zero);
}

TEST(PoseUtilities, PoseToOriginTransformationWithDifferentInitialRobotOrientation)
{
    // Values derived from real data with robot rotated ~-90 degrees to rail
    // Robot travelled ~0.815m backwards off rail

    geometry_msgs::Pose pose_final;
    pose_final.position.x = -2.337;
    pose_final.position.y = -3.718;
    pose_final.position.z = 0.0;
    pose_final.orientation.z = -0.6715699434280396;
    pose_final.orientation.w = -0.7409411072731018;

    geometry_msgs::Pose pose_start;
    pose_start.position.x = -2.2582900524139404;
    pose_start.position.y = -2.9038498401641846;
    pose_start.position.z = 0.0;
    pose_start.orientation.z = -0.6715699434280396;
    pose_start.orientation.w = -0.7409411072731018;

    geometry_msgs::Pose pose_diff;
    pose_diff.position.x = -0.815;
    pose_diff.orientation.w = 1.0;

    PoseUtilities p_util;
    tf2::Transform tf = p_util.getTransformFromPoseToOrigin(pose_start);
    tf2::Vector3 vec_start(pose_start.position.x, pose_start.position.y, pose_start.position.z);
    tf2::Vector3 vec_final(pose_final.position.x, pose_final.position.y, pose_final.position.z);
    vec_start = tf * vec_start;
    vec_final = tf * vec_final;


    tf2::Vector3 vec_zero;
    tf2::Vector3 vec_diff(-0.815,0,0);
    ASSERT_NEAR(vec_final.getX() - vec_start.getX(), vec_diff.getX(), 0.05);
    ASSERT_NEAR(vec_final.getY() - vec_start.getY(), vec_diff.getY(), 0.05);

    ASSERT_NEAR(vec_start.getX() - vec_zero.getX(), 0, 0.05);
    ASSERT_NEAR(vec_start.getY() - vec_zero.getY(), 0, 0.05);
    printVec3(vec_start);
    printVec3(vec_zero);

    printVec3(vec_diff);
    printVec3(vec_final);
}

int main (int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    
    return RUN_ALL_TESTS();
}