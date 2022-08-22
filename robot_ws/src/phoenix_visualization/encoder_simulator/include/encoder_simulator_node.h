/*
    Goal processing Node/Cartpuller
    Author: Abdullah Mohiuddin
    Email: a.mohiuddin@ai-systems.ca
*/
#include "ros/ros.h"
#include <ros/spinner.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>

#define LOOPRATE 30

namespace encodersimulator
{
    class EncoderSimulatorNode
	{
        protected:
            ros::NodeHandle nh_;
            ros::NodeHandle private_nh_;
            ros::Subscriber wheelodom_cmd_subscriber;
            ros::Publisher encoder_publisher;

        private: //Define all private variables here
            void init(void);
            void createSubscribers(void);
            void createPublishers(void);
            void wheelodom_cmd_cb(const nav_msgs::Odometry& msg);
            std_msgs::Float64MultiArray encoder_output;
            float wheel1_ecoder_reading;
            float wheel2_ecoder_reading;
            float wheel3_ecoder_reading;                        
            float wheel4_ecoder_reading;
            int v1;  // random number
        public:
            void run(void);
            // Defining the class constructor
            EncoderSimulatorNode(int argc, char** argv)
            :private_nh_("~")
            {
                init();
            }

};

}


