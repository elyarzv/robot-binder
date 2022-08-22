/*
    * All ROS callbacks are defined here
    * Author : Abdullah MOhiuddin
    * Email : a.mohiuddin@ai-systems.ca
*/
#include <encoder_simulator_node.h>

using namespace encodersimulator;

void EncoderSimulatorNode::init(void)
{
    createSubscribers();
    createPublishers();
}

void EncoderSimulatorNode::createSubscribers(void)
{
    // all of the subscriber are created here
    wheelodom_cmd_subscriber = nh_.subscribe("/wheelodom", 10, &EncoderSimulatorNode::wheelodom_cmd_cb, this);
}
void EncoderSimulatorNode::createPublishers(void)
{
    //all publishers are defined here
    encoder_publisher = nh_.advertise<std_msgs::Float64MultiArray>("au_encoderCount", 1);
}

void EncoderSimulatorNode::wheelodom_cmd_cb(const nav_msgs::Odometry& msg)
{
    v1 = (rand() % 10 + 90);
    wheel1_ecoder_reading=v1*msg.pose.pose.position.x/0.0635/3.14/100;
    v1 = (rand() % 10 + 90);
    wheel2_ecoder_reading=v1*msg.pose.pose.position.x/0.0635/3.14/100;
    v1 = (rand() % 10 + 90);
    wheel3_ecoder_reading=v1*msg.pose.pose.position.x/0.0635/3.14/100;
    v1 = (rand() % 10 + 90);
    wheel4_ecoder_reading=v1*msg.pose.pose.position.x/0.0635/3.14/100;

    encoder_output.data={wheel1_ecoder_reading,wheel2_ecoder_reading,wheel3_ecoder_reading,wheel4_ecoder_reading};
    encoder_publisher.publish(encoder_output);
}

void EncoderSimulatorNode::run(void)
{
    ros::Rate loop_rate(LOOPRATE);
    
    while (ros::ok())
    {
		ROS_INFO("encoder simulator is running \n");
		ros::spinOnce();
		loop_rate.sleep();
	}
}
