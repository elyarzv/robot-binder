#include <amr_localization/1d_localization.h>


oneDNode::oneDNode(ros::NodeHandle* nodeHandle, ros::NodeHandle* private_nodeHandle) :
    nh_(nodeHandle), private_nh_(private_nodeHandle)
{
	private_nh_->param<std::string>("odometry_topic", odometry_topic_, std::string("embedded/wheelodom"));
    private_nh_->param<std::string>("pose_topic", pose_topic_, std::string("pose_1_d"));
    private_nh_->param<std::string>("topic_type", topic_type_, std::string("Invalid"));
    private_nh_->param<int>("encoder_counts_per_meter", counts_per_meter_, 1000);
    private_nh_->param<float>("wheel_scaling", wheel_scaling_, 1);
    pose_1d_pub_ = private_nh_->advertise<std_msgs::Float32>(pose_topic_, 1000);
    reset_pose_service_ = private_nh_->advertiseService("reset", &oneDNode::resetCb, this);
    ROS_INFO_STREAM("Topic type [" << topic_type_ << "] Topic name [" << odometry_topic_ << "]");
    if(topic_type_.compare("Twist") == 0 || topic_type_.compare("Pose") == 0 ){
        odom_subscriber_ = nh_->subscribe(odometry_topic_, 1, &oneDNode::odomCb, this);
    }else if(topic_type_.compare("MultiEncoder") == 0){
        odom_subscriber_ = nh_->subscribe(odometry_topic_, 1, &oneDNode::multiEncoderCb, this);
        current_encoder_counts_.data = {0, 0, 0, 0};
    }else if(topic_type_.compare("Encoder") == 0){
        odom_subscriber_ = nh_->subscribe(odometry_topic_, 1, &oneDNode::encoderCb, this);
        current_encoder_counts_.data = {0};
    }else{
        ROS_ERROR_THROTTLE(1,"Invalid topic type");
    }
    reset();
}

void oneDNode::multiEncoderCb(const std_msgs::Int64MultiArrayConstPtr &odom){
    current_encoder_counts_ = *odom;
}
void oneDNode::encoderCb(const std_msgs::Int64ConstPtr &odom){
    current_encoder_counts_.data[0] = odom->data;
}
void oneDNode::odomCb(const nav_msgs::OdometryConstPtr &odom){
    current_odom_ = *odom;
}

//For use in TDD
float oneDNode::test_function(float input){
	return input;
}
float oneDNode::computeDistance(std::string _topic_type){
    if(_topic_type.compare("Twist") == 0){
        float time_step = current_odom_.header.stamp.toSec()-prev_odom_.header.stamp.toSec();
        float distance;
        ROS_DEBUG("Time step %f avg speed %f",time_step, ((current_odom_.twist.twist.linear.x + prev_odom_.twist.twist.linear.x)/2));
        if(time_step > 1){
            ROS_ERROR("Odometry messages are old %f",time_step);
        }else{
            //Average the two twist speeds and multiply by the time step to integrate
            distance = ((current_odom_.twist.twist.linear.x + prev_odom_.twist.twist.linear.x)/2) * (time_step);
        }
	    prev_odom_ = current_odom_;
        return distance;

    }else if(_topic_type.compare("Pose") == 0){
        float time_step = current_odom_.header.stamp.toSec()-prev_odom_.header.stamp.toSec();
        if(time_step <= 0){ return 0;}
        else{
            //Convert the current and previous poses from the odom frame to the origin frame using the odomToOrigin_ transform, then
            //compute the change in X position and multiply by wheel scaling factor
            tf::Vector3 current_pose_odom(current_odom_.pose.pose.position.x,current_odom_.pose.pose.position.y,current_odom_.pose.pose.position.z);
            tf::Vector3 prev_pose_odom(prev_odom_.pose.pose.position.x,prev_odom_.pose.pose.position.y,prev_odom_.pose.pose.position.z);
            tf::Vector3 current_pose_origin = odomToOrigin_ * current_pose_odom;
            tf::Vector3 prev_pose_origin = odomToOrigin_ * prev_pose_odom; 
            prev_odom_ = current_odom_;
            return (current_pose_origin[0]-prev_pose_origin[0])*wheel_scaling_;
        }
    }else if(_topic_type.compare("MultiEncoder") == 0){
        int ctr = 0;
        float avg = 0;
        //Reset the relative estimation if the number of encoders suddenly changes
        if(current_encoder_counts_.data.size() != prev_encoder_counts_.data.size()){
            ROS_ERROR("Encoder size has changed");
            prev_encoder_counts_ = current_encoder_counts_;
            return 0;
        }else{
            //Average the encoder counts together and conver to meters
            for(auto n : current_encoder_counts_.data){
                avg = avg + (n - prev_encoder_counts_.data[ctr++]);
            }
            prev_encoder_counts_ = current_encoder_counts_;
            return ((avg / ctr) / counts_per_meter_);
        }
    }else if(_topic_type.compare("Encoder") == 0){
        float prev = prev_encoder_counts_.data[0];
        float current = current_encoder_counts_.data[0];
        float dist =  (prev - current) / counts_per_meter_;
        prev_encoder_counts_ = current_encoder_counts_;
        return dist;
    }else{
        ROS_ERROR("Invalid topic type");
    }
    return 0;
}
void oneDNode::run(){
    pose_1d.data = pose_1d.data + computeDistance(topic_type_);
	pose_1d_pub_.publish(pose_1d);
}
void oneDNode::reset(){
    //Reset position estimate
	pose_1d.data = 0;
    if(topic_type_.compare("Pose") == 0){
        //Compute transform from the odometry topic's frame to the robot's current frame, which will ot become the origin frame
        odomToOrigin_ = computeOdomToOrigin();
    }else if(topic_type_.compare("Encoder") == 0 || topic_type_.compare("MultiEncoder") == 0){
        //Reset encoder counter
        prev_encoder_counts_ = current_encoder_counts_;
    }
}
tf::Transform oneDNode::computeOdomToOrigin(){
    tf::Transform transform;
    tf::Quaternion q(
        current_odom_.pose.pose.orientation.x,
        current_odom_.pose.pose.orientation.y,
        current_odom_.pose.pose.orientation.z,
        current_odom_.pose.pose.orientation.w);
    tf::Vector3 v(current_odom_.pose.pose.position.x,current_odom_.pose.pose.position.y,current_odom_.pose.pose.position.z);
    transform.setOrigin(v);
    transform.setRotation(q);
    return transform.inverse();
}

bool oneDNode::resetCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
    reset();
    res.success = true;
    return true;
}
