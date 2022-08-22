#include <gtest/gtest.h>

#include <ais_safety/thread_safe_var.h>
#include <memory>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/LaserScan.h>

// fixtures for tests. Test defined by TEST_F(TestNode, ...) uses this class
class TestNode : public ::testing::Test
{
// everything must be protected in a test fixture
protected:
    // Code here will be called immediately after the constructor
    // (right before each test).
    void SetUp() override;

    void safeCmdVelCB(const geometry_msgs::Twist::ConstPtr &cmd_vel);

    ros::NodeHandle nh_;
    std::shared_ptr<ros::AsyncSpinner> spinner_;

    ThreadSafeVar<geometry_msgs::TwistStamped> safe_cmd_vel_;

    ros::Publisher  brain_cmd_vel_pub_;
    ros::Publisher  laser_pub_;

    ros::Subscriber safe_cmd_vel_sub_;
}; // endclass TestNode

void TestNode::SetUp()
{
    nh_ = ros::NodeHandle();
    spinner_ = std::make_shared<ros::AsyncSpinner>(4);
    spinner_->start();

    safe_cmd_vel_sub_ = nh_.subscribe("/cmd_vel_embedded", 1, &TestNode::safeCmdVelCB, this);

    // TODO: add test publishing brain_cmd_vel and scan
    brain_cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(
        "/brain_cmd_vel", 1
    );

    laser_pub_ = nh_.advertise<sensor_msgs::LaserScan>(
        "/scan_multi_filtered", 1
    );

    ROS_ERROR("TestNode Setup");
}

void TestNode::safeCmdVelCB(const geometry_msgs::Twist::ConstPtr &cmd_vel)
{
    geometry_msgs::TwistStamped cmd_vel_stamped;
    cmd_vel_stamped.twist = *cmd_vel;
    cmd_vel_stamped.header.stamp = ros::Time::now();
    safe_cmd_vel_.set(cmd_vel_stamped);
}

// without publication of /brain_cmd_vel, safe cmd vel should be zero
TEST_F(TestNode, cmdVelCheckZero)
{
    ASSERT_EQ("/cmd_vel_embedded", safe_cmd_vel_sub_.getTopic());

    double timeout;
    double check_interval;
    nh_.param<double>("/cmd_vel_timeout", timeout, 10.0);
    nh_.param<double>("/cmd_vel_check_interval", check_interval, 0.1);

    // wait for topic
    auto start_time = ros::Time::now();
    ros::Duration time_elapsed;
    do
    {
        ros::Duration(check_interval).sleep();
        time_elapsed = ros::Time::now() - start_time;
    } while (safe_cmd_vel_sub_.getNumPublishers() < 1 \
                && time_elapsed.toSec() < timeout);

    ASSERT_EQ(1, safe_cmd_vel_sub_.getNumPublishers());

    bool cmd_vel_received = false;
    start_time = ros::Time::now();
    do
    {
        auto safe_cmd_vel = safe_cmd_vel_.get();
        if (safe_cmd_vel.header.stamp.toSec() > 0)
        {
            cmd_vel_received = true;
            ROS_ERROR_STREAM("safe twist: " << safe_cmd_vel);
            EXPECT_EQ(0.0, safe_cmd_vel.twist.linear.x);
            EXPECT_EQ(0.0, safe_cmd_vel.twist.linear.y);
            EXPECT_EQ(0.0, safe_cmd_vel.twist.angular.z);
        }

        ros::Duration(check_interval).sleep();
        time_elapsed = ros::Time::now() - start_time;
    } while (time_elapsed.toSec() < timeout);

    EXPECT_TRUE(cmd_vel_received);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "ais_safetey_node_tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
