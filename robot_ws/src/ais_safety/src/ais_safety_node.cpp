#include <ais_safety/ais_safety.h>

int main (int argc, char **argv) 
{
	ros::init(argc, argv, "interface_node");
	ros::NodeHandle nh_;
	safetyNode safety_node_(&nh_);
	ros::Rate loop_rate(100);
	while(ros::ok()){
		safety_node_.publish_cmd_vel();
		ros::spinOnce();
		loop_rate.sleep();
	}
  //exit
	return 0;
}