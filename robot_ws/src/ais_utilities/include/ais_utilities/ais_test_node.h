#ifndef AIS_SAFETY_AIS_TEST_NODE
#define AIS_SAFETY_AIS_TEST_NODE

#include <gtest/gtest.h>
#include <ros/ros.h>

namespace ais_utilities
{
// fixtures for tests. Test defined by TEST_F(TestNode, ...) uses this class
class AISTestNode : public ::testing::Test
{

// everything must be protected in a test fixture
protected:
    ros::NodeHandle nh_;
    std::shared_ptr<ros::AsyncSpinner> spinner_;
    ros::ServiceClient rosbag_pause_client_;

    double topic_check_interval_;
    double topic_timeout_;

    void initNode();
    void waitForTopic(ros::Subscriber &subscriber);
    bool startROSBag();
}; // endclass TestNode
} // endnamespace ais_utilities

#endif // AIS_SAFETY_AIS_TEST_NODE

