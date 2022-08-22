#include <gtest/gtest.h>
#include <atomic>
#include <cmath>
#include <memory>

#include <ais_safety/thread_safe_var.h>
#include <ais_brain/loadMission.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/SetBool.h>

// fixtures for tests. Test defined by TEST_F(TestNode, ...) uses this class
class TestNode : public ::testing::Test
{
// everything must be protected in a test fixture
protected:
    // Code here will be called immediately after the constructor
    // (right before each test).
    void SetUp() override;

    void laserScanCB(const sensor_msgs::LaserScan::ConstPtr &scan);

    bool startMission();

    ThreadSafeVar<sensor_msgs::LaserScan> laser_scan_;
    std::atomic_bool new_laser_scan_;

    ros::NodeHandle nh_;
    std::shared_ptr<ros::AsyncSpinner> spinner_;

    ros::Subscriber laser_sub_;
    ros::ServiceClient start_mission_client_;
}; // endclass TestNode

void TestNode::SetUp()
{
    nh_ = ros::NodeHandle();
    spinner_ = std::make_shared<ros::AsyncSpinner>(4);
    spinner_->start();

    new_laser_scan_ = false;

    laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>(
        "/scan_multi_filtered", 1,
        &TestNode::laserScanCB, this
    );

    start_mission_client_ = nh_.serviceClient<ais_brain::loadMission>(
        "/uv_interface/start_mission"
    );
}

void TestNode::laserScanCB(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    laser_scan_.set(*scan);
    new_laser_scan_ = true;
}

bool TestNode::startMission()
{
    ais_brain::loadMission srv_load_mission;
    srv_load_mission.request.mission = "ais_board_room1sec";
    srv_load_mission.request.return_home = true;

    start_mission_client_.waitForExistence(ros::Duration(20.0));

    if (!start_mission_client_.exists())
    {
        ROS_ERROR("start_mission service does not exist");
        return false;
    }

    if (start_mission_client_.call(srv_load_mission))
    {
        return srv_load_mission.response.success;
    }
    else
    {
        ROS_ERROR("start_mission service call failed");
        return false;
    }
}

// check for collision
TEST_F(TestNode, checkCollision)
{
    double test_timeout;
    double laser_timeout;
    double check_interval;
    nh_.param<double>("/test_timeout", test_timeout, 60.0);
    nh_.param<double>("/laser_timeout", laser_timeout, 10.0);
    nh_.param<double>("/laser_check_interval", check_interval, 0.1);

    ASSERT_TRUE(startMission());

    // wait for topic
    auto start_time = ros::WallTime::now();
    ros::WallDuration time_elapsed;
    do
    {
        ros::WallDuration(check_interval).sleep();
        time_elapsed = ros::WallTime::now() - start_time;
    } while (laser_sub_.getNumPublishers() < 1
                && time_elapsed.toSec() < test_timeout);

    ASSERT_EQ(1, laser_sub_.getNumPublishers());

    bool laser_received = false;
    start_time = ros::WallTime::now();
    auto last_laser_time = ros::WallTime::now();
    ros::WallDuration laser_elapsed;
    do
    {
        if (new_laser_scan_)
        {
            new_laser_scan_ = false;

            auto laser_scan = laser_scan_.get();

            if (laser_scan.header.stamp.toSec() > 0)
            {
                laser_received = true;
                last_laser_time = ros::WallTime::now();

                for (const auto range: laser_scan.ranges)
                {
                    if (std::isnan(range) || range < 1e-3)
                    {
                        continue;
                    }

                    ASSERT_LE(0.01, range);
                }
            }
         }

        ros::WallDuration(check_interval).sleep();
        time_elapsed = ros::WallTime::now() - start_time;
        laser_elapsed = ros::WallTime::now() - last_laser_time;
    } while (time_elapsed.toSec() < test_timeout
                && laser_elapsed.toSec() < laser_timeout);

    EXPECT_TRUE(laser_received);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "ais_safetey_node_tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}

