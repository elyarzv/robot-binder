#include "ais_job_manager/JobManager.h"

using namespace ais_job_manager;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "phoenix_job_manager");
    ros::NodeHandle nh;

    ros::Rate loop_rate(10);

    ROS_INFO("Starting Job Manager");
    JobManager job_manager_(nh);

    job_manager_.runProcess();

    
    return 0;
}