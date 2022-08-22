#include <ais_utilities/ais_test_node.h>
#include <std_srvs/SetBool.h>

using namespace ais_utilities;

void AISTestNode::initNode()
{
    nh_ = ros::NodeHandle();
    spinner_ = std::make_shared<ros::AsyncSpinner>(4);
    spinner_->start();

    rosbag_pause_client_ = nh_.serviceClient<std_srvs::SetBool>("/rosbag_play/pause_playback");

    nh_.param<double>("/topic_timeout", topic_timeout_, 10.0);
    nh_.param<double>("/topic_check_interval", topic_check_interval_, 0.1);
}

void AISTestNode::waitForTopic(ros::Subscriber &subscriber)
{
    // wait for topic
    auto start_time = ros::Time::now();
    ros::Duration time_elapsed;
    do
    {
        ros::Duration(topic_check_interval_).sleep();
        time_elapsed = ros::Time::now() - start_time;
    } while (subscriber.getNumPublishers() < 1 \
                && time_elapsed.toSec() < topic_timeout_);
}

bool AISTestNode::startROSBag()
{
    std_srvs::SetBool srv;
    srv.request.data = false;

    rosbag_pause_client_.waitForExistence(ros::Duration(10));
    if (rosbag_pause_client_.call(srv))
    {
        if (!srv.response.success)
        {
            ROS_ERROR_STREAM("rosbag play error: " << srv.response.message);
        }
        return srv.response.success;
    }
    else
    {
        ROS_ERROR("rosbag play failed ");
        return false;
    }

}
