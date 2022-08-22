#include <amr_localization/1d_localization.h>


int main(int argc, char** argv)
{	
	ros::init(argc, argv, "one_d_localization_node");
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_("~");
	oneDNode one_d_node_(&nh_,&private_nh_);
	ros::Rate loop_rate(100);
	while(ros::ok()){
		one_d_node_.run();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
