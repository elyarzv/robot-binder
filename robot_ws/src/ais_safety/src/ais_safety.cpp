#include <ais_safety/ais_safety.h>

#define THROTTLE_LINEAR 0.1
#define THROTTLE_ANGLE 0.1
#define MAX_LINEAR 0.5
#define MAX_ANGLE 1.0

  safetyNode::safetyNode(ros::NodeHandle* nodeHandle) :
    nh_(nodeHandle)
  {
        //safety_mode_sub = nh_->subscribe("/ais_brain/robot_status", 1, &safetyNode::safetyCB, this);
        lidar_sub = nh_->subscribe("scan_multi_filtered", 1, &safetyNode::lidarCB, this);
        cmd_vel_sub = nh_->subscribe("base_cmd_vel", 1, &safetyNode::cmd_velCB, this);
        cmd_vel_pub = nh_->advertise<geometry_msgs::Twist>("embedded/cmd_vel", 1000);
        safety_status_pub = nh_->advertise<std_msgs::String>("/safety_status", 1);
        footprint_pub = nh_->advertise<geometry_msgs::PolygonStamped>("safety_footprint", 1000);
        throttle_pub = nh_->advertise<geometry_msgs::PolygonStamped>("throttle_footprint", 1000);
        collision_ = false;
        ros::Rate loop_rate(1);
        while(!ros::param::has("robot_config") && ros::ok()){
            ROS_INFO_NAMED("safety","Waiting for robot config params");
            loop_rate.sleep();
        }
        if (ros::param::get("/robot_front", FRONT) &&
                ros::param::get("/robot_left", LEFT) &&
                ros::param::get("/robot_right", RIGHT) &&
                ros::param::get("/robot_rear", REAR) &&
                ros::param::get("/robot_manual_x", MANUAL_PADDING_X) &&
                ros::param::get("/robot_manual_y", MANUAL_PADDING_Y) &&
                ros::param::get("/robot_rotation_x", ROTATION_X) &&
                ros::param::get("/robot_rotation_y", ROTATION_Y) &&
                ros::param::get("/robot_teleop_x", TELEOP_PADDING_X) &&
                ros::param::get("/robot_teleop_y", TELEOP_PADDING_Y) &&
                ros::param::get("/robot_autonomous_x", AUTONOMOUS_PADDING_X) &&
                ros::param::get("/robot_autonomous_y", AUTONOMOUS_PADDING_Y) &&
                ros::param::get("/robot_throttle_x", THROTTLE_PADDING_X) &&
                ros::param::get("/robot_throttle_y", THROTTLE_PADDING_Y)){
            ROS_INFO_NAMED("safety","Robot parameters loaded");
        }else{
            while(ros::ok() && !(ros::param::get("/robot_front", FRONT) &&
                ros::param::get("/robot_left", LEFT) &&
                ros::param::get("/robot_right", RIGHT) &&
                ros::param::get("/robot_rear", REAR) &&
                ros::param::get("/robot_manual_x", MANUAL_PADDING_X) &&
                ros::param::get("/robot_manual_y", MANUAL_PADDING_Y) &&
                ros::param::get("/robot_rotation_x", ROTATION_X) &&
                ros::param::get("/robot_rotation_y", ROTATION_Y) &&
                ros::param::get("/robot_teleop_x", TELEOP_PADDING_X) &&
                ros::param::get("/robot_teleop_y", TELEOP_PADDING_Y) &&
                ros::param::get("/robot_autonomous_x", AUTONOMOUS_PADDING_X) &&
                ros::param::get("/robot_autonomous_y", AUTONOMOUS_PADDING_Y) &&
                ros::param::get("/robot_throttle_x", THROTTLE_PADDING_X) &&
                ros::param::get("/robot_throttle_y", THROTTLE_PADDING_Y))){
                ROS_ERROR_NAMED("safety","Error loading robot footprint, config file is malformed!!!");
                loop_rate.sleep();
            }
        }
        makeFootprints();
        checkFootprints();
        collision_ = false;
        current_mux = zero_cmd_vel_mux;

       dynamic_reconfigure_server_
           = std::make_shared< dynamic_reconfigure::Server<dyn_cfg_t> >();

       dynamic_reconfigure::Server<dyn_cfg_t>::CallbackType dyn_config_cb
           = boost::bind(&safetyNode::dynamicReconfigureCallback, this, _1, _2);

       dynamic_reconfigure_server_->setCallback(dyn_config_cb);
 }

   void safetyNode::dynamicReconfigureCallback(
       safetyNode::dyn_cfg_t &config, uint32_t level
   )
   {
       forward_safety_ = config.FORWARD_SAFETY;
       backward_safety_ = config.BACKWARD_SAFETY;

       ROS_INFO_THROTTLE(
           1.0, "forward safety: %d, backward safety: %d",
           (int)forward_safety_, (int)backward_safety_
       );
  }


    bool safetyNode::isPointInPolygon(geometry_msgs::Point32 _point, std::vector<geometry_msgs::Point32> &verts){
        int i, j = 0;
        int nvert = verts.size();
        bool c = false;
        for(i = 0, j = nvert-1; i < nvert; j = i++) {
            if ( ((verts[i].y>=_point.y) != (verts[j].y>=_point.y)) &&
            (_point.x <= (verts[j].x - verts[i].x) * (_point.y-verts[i].y) / (verts[j].y-verts[i].y) + verts[i].x) )
                c = !c;
        }
        return c;
    }

  void safetyNode::cmd_velCB(const geometry_msgs::Twist::ConstPtr& cmd_vel){
        brain_cmd_vel = *cmd_vel;
        last_cmd_message_ = ros::Time::now();
    }

  geometry_msgs::Twist safetyNode::clamp_vel(geometry_msgs::Twist &cmd_vel, float clamp_x, float clamp_y, float clamp_angle){
		geometry_msgs::Twist ret_vel = cmd_vel;
		if(cmd_vel.linear.x < -clamp_x) ret_vel.linear.x = -clamp_x;
		if(cmd_vel.linear.x > clamp_x) ret_vel.linear.x = clamp_x;
		if(cmd_vel.linear.y < -clamp_y) ret_vel.linear.y = -clamp_y;
		if(cmd_vel.linear.y > clamp_y) ret_vel.linear.y = clamp_y;
		if(cmd_vel.angular.z > clamp_angle) ret_vel.angular.z = clamp_angle;
		if(cmd_vel.angular.z < -clamp_angle) ret_vel.angular.z = -clamp_angle;
		return ret_vel;
	}

  void safetyNode::lidarCB(const sensor_msgs::LaserScan::ConstPtr& scan){

        geometry_msgs::PolygonStamped poly;
        geometry_msgs::PolygonStamped poly_throttle;
        geometry_msgs::PolygonStamped poly_safety;
        bool reduce_speed = false;
        bool collision = false;
        poly.header.frame_id = "base_link";
        poly.header.stamp = ros::Time::now();
        poly_throttle.header.frame_id = "base_link";
        poly_throttle.header.stamp = ros::Time::now();
        poly_safety.header.frame_id = "base_link";
        poly_safety.header.stamp = ros::Time::now();

        //The scan_multi topic is in base link so we dont need the following transform

        // if(!listener_.waitForTransform(
    //   scan->header.frame_id,
    //   "/base_link",
    //   scan->header.stamp + ros::Duration().fromSec(scan->ranges.size()*scan->time_increment),
    //     ros::Duration(1.0))){
        //   ROS_ERROR_THROTTLE_NAMED(0.2,"safety","Could not look up transform!");
    //   return;
      // }

      sensor_msgs::PointCloud cloud;
      //projector_.transformLaserScanToPointCloud("/base_link",*scan,cloud,listener_,2);
        projector_.projectLaser(*scan, cloud);
    ////////////////////FWD-REW///////////////////////
        if (forward_safety_ && (brain_cmd_vel.linear.x > 0)) {
            getFwdFootprint(poly.polygon.points);
            if(checkForPointsInFootprint(cloud,poly.polygon.points,fwdFootprintThrottle)){
                if(collision_){
                    ROS_ERROR_THROTTLE_NAMED(0.2,"safety","Warning forward collision imminent");
                    collision = true;
                    poly_safety.polygon.points.insert(poly_safety.polygon.points.end(), poly.polygon.points.begin(), poly.polygon.points.end());
                }else{
                    ROS_WARN_THROTTLE_NAMED(0.2,"safety","Warning forward collision possible");
                    reduce_speed = true;
                    poly_throttle.polygon.points.insert(poly_throttle.polygon.points.end(), fwdFootprintThrottle.begin(), fwdFootprintThrottle.end());
                }
            }
        } else if (backward_safety_ && (brain_cmd_vel.linear.x < 0)) {
          getRevFootprint(poly.polygon.points);
            if(checkForPointsInFootprint(cloud,poly.polygon.points,revFootprintThrottle)){
                if(collision_){
                    ROS_ERROR_THROTTLE_NAMED(0.2,"safety","Warning reverse collision imminent");
                    collision = true;
                    poly_safety.polygon.points.insert(poly_safety.polygon.points.end(), poly.polygon.points.begin(), poly.polygon.points.end());
                }else{
                    ROS_WARN_THROTTLE_NAMED(0.2,"safety","Warning reverse collision possible");
                    reduce_speed = true;
                    poly_throttle.polygon.points.insert(poly_throttle.polygon.points.end(), revFootprintThrottle.begin(), revFootprintThrottle.end());
                }
            }
        }
        /////////////////////////LEFT-RIGHT//////////////////////////
        if(brain_cmd_vel.linear.y > 0){
            getLeftFootprint(poly.polygon.points);
            if(checkForPointsInFootprint(cloud,poly.polygon.points,leftFootprintThrottle)){
                if(collision_){
                    ROS_ERROR_THROTTLE_NAMED(0.2,"safety","Warning left collision imminent");
                    collision = true;
                    poly_safety.polygon.points.insert(poly_safety.polygon.points.end(), poly.polygon.points.begin(), poly.polygon.points.end());
                }else{
                    ROS_WARN_THROTTLE_NAMED(0.2,"safety","Warning left collision possible");
                    reduce_speed = true;
                    poly_throttle.polygon.points.insert(poly_throttle.polygon.points.end(), leftFootprintThrottle.begin(), leftFootprintThrottle.end());
                }
            }
        }else if(brain_cmd_vel.linear.y < 0){
          getRightFootprint(poly.polygon.points);
            if(checkForPointsInFootprint(cloud,poly.polygon.points,rightFootprintThrottle)){
                if(collision_){
                    ROS_ERROR_THROTTLE_NAMED(0.2,"safety","Warning right collision imminent");
                    collision = true;
                    poly_safety.polygon.points.insert(poly_safety.polygon.points.end(), poly.polygon.points.begin(), poly.polygon.points.end());
                }else{
                    ROS_WARN_THROTTLE_NAMED(0.2,"safety","Warning right collision possible");
                    reduce_speed = true;
                    poly_throttle.polygon.points.insert(poly_throttle.polygon.points.end(), rightFootprintThrottle.begin(), rightFootprintThrottle.end());
                }
            }
        }
        ////////////////////////ROTATION//////////////////////////////
        if(brain_cmd_vel.angular.z > 0){
          getLeftRotationFootprint(poly.polygon.points);
            if(checkForPointsInFootprint(cloud,poly.polygon.points,leftRotationFootprintThrottle)){
                if(collision_){
                    ROS_ERROR_THROTTLE_NAMED(0.2,"safety","Warning left rotation collision imminent");
                    collision = true;
                    poly_safety.polygon.points.insert(poly_safety.polygon.points.end(), poly.polygon.points.begin(), poly.polygon.points.end());
                }else{
                    ROS_WARN_THROTTLE_NAMED(0.2,"safety","Warning left rotation collision possible");
                    reduce_speed = true;
                    poly_throttle.polygon.points.insert(poly_throttle.polygon.points.end(), leftRotationFootprintThrottle.begin(), leftRotationFootprintThrottle.end());
                }
            }
        }else if(brain_cmd_vel.angular.z < 0){
          getRightRotationFootprint(poly.polygon.points);
            if(checkForPointsInFootprint(cloud,poly.polygon.points,rightRotationFootprintThrottle)){
                if(collision_){
                    ROS_ERROR_THROTTLE_NAMED(0.2,"safety","Warning right rotation collision imminent");
                    collision = true;
                    poly_safety.polygon.points.insert(poly_safety.polygon.points.end(), poly.polygon.points.begin(), poly.polygon.points.end());
                }else{
                    ROS_WARN_THROTTLE_NAMED(0.2,"safety","Warning right rotation collision possible");
                    reduce_speed = true;
                    poly_throttle.polygon.points.insert(poly_throttle.polygon.points.end(), rightRotationFootprintThrottle.begin(), rightRotationFootprintThrottle.end());
                }
            }
        }
        footprint_pub.publish(poly_safety);
        throttle_pub.publish(poly_throttle);
        if(collision && !reduce_speed){
            safety_string.data = "COLLISION";
            safety_status_pub.publish(safety_string);
        }else{
            safety_string.data = "SAFE";
            safety_status_pub.publish(safety_string);
        }
        ros::spinOnce();
        if(collision || (brain_cmd_vel.linear.x == 0 && brain_cmd_vel.linear.y == 0 && brain_cmd_vel.angular.z == 0)){
            current_mux = zero_cmd_vel_mux;
        }else if(reduce_speed){
            current_mux = reduced_cmd_vel_mux;
        }else{
            current_mux = brain_cmd_vel_mux;
        }
    }

  void safetyNode::publish_cmd_vel(){
    ros::Duration message_time = ros::Time::now()-last_cmd_message_;
    if( message_time.toSec() > 0.5){
        ROS_ERROR_THROTTLE_NAMED(1,"safety","brain_cmd_vel messages are %f seconds old", message_time.toSec());
        cmd_vel_pub.publish(zero_cmd_vel);
    }else{
        switch(current_mux){
            case brain_cmd_vel_mux:
                {
                cmd_vel_pub.publish(clamp_vel(brain_cmd_vel,MAX_LINEAR,MAX_LINEAR,MAX_ANGLE));
                break;
                }
            case zero_cmd_vel_mux:
                {
                cmd_vel_pub.publish(zero_cmd_vel);
                break;
                }
            case reduced_cmd_vel_mux:
                {
                cmd_vel_pub.publish(clamp_vel(brain_cmd_vel,THROTTLE_LINEAR,THROTTLE_LINEAR,THROTTLE_ANGLE));
                break;
                }
            default:
                cmd_vel_pub.publish(zero_cmd_vel);
        }
            ros::spinOnce();
    }

  }

     bool safetyNode::checkForPointsInFootprint(sensor_msgs::PointCloud cloud, std::vector<geometry_msgs::Point32> &footprint, std::vector<geometry_msgs::Point32> &throttle_footprint){
        int num_pts_in_polygon = 0;
        int num_pts_in_throttle_region = 0;
        //Here we do two calculations, first we check if a lidar point is inside the larger footprint, and if so then we check if its inside the smaller footprint
        for(int ctr = 0; ctr < cloud.points.size(); ++ctr){
            if(isPointInPolygon(cloud.points[ctr],throttle_footprint)){
                ++num_pts_in_throttle_region;
                if(isPointInPolygon(cloud.points[ctr],footprint)){
                    //point_in_polygon = true;
                    ++num_pts_in_polygon;
                }
            }
        }
        collision_ = num_pts_in_polygon > 5;
        return num_pts_in_throttle_region > 5;
     }


  void safetyNode::getFwdFootprint(std::vector<geometry_msgs::Point32> &verts){
		verts.clear();
                    verts = fwdFootprint;

    }
  void safetyNode::getRightFootprint(std::vector<geometry_msgs::Point32> &verts){
		verts.clear();
		    verts = rightFootprintManual;
	}
  void safetyNode::getLeftFootprint(std::vector<geometry_msgs::Point32> &verts){
		verts.clear();
		    verts = leftFootprintManual;
	}
  void safetyNode::getRightRotationFootprint(std::vector<geometry_msgs::Point32> &verts){
		verts.clear();
		    verts = rightRotationFootprintManual;
	}
  void safetyNode::getLeftRotationFootprint(std::vector<geometry_msgs::Point32> &verts){
		verts.clear();
		    verts = leftRotationFootprintManual;
	}
  void safetyNode::getRevFootprint(std::vector<geometry_msgs::Point32> &verts){
		verts.clear();
		    verts = revFootprintManual;
	}


    void safetyNode::checkFootprints(){
        //Since the code assumes the throttle region is greater than or equal to the safety region this function ensures that if throttle footprints are too small it uses the safety footprint
        if(ROTATION_X < AUTONOMOUS_PADDING_X) ROTATION_X = AUTONOMOUS_PADDING_X;
        if(ROTATION_Y < AUTONOMOUS_PADDING_Y) ROTATION_Y = AUTONOMOUS_PADDING_Y;
        if(ROTATION_X < TELEOP_PADDING_X) ROTATION_X = TELEOP_PADDING_X;
        if(ROTATION_Y < TELEOP_PADDING_Y) ROTATION_Y = TELEOP_PADDING_Y;
        if(ROTATION_X < MANUAL_PADDING_X) ROTATION_X = MANUAL_PADDING_X;
        if(ROTATION_Y < MANUAL_PADDING_Y) ROTATION_Y = MANUAL_PADDING_Y;
        if(THROTTLE_PADDING_X < AUTONOMOUS_PADDING_X) THROTTLE_PADDING_X = AUTONOMOUS_PADDING_X;
        if(THROTTLE_PADDING_Y < AUTONOMOUS_PADDING_Y) THROTTLE_PADDING_Y = AUTONOMOUS_PADDING_Y;
        if(THROTTLE_PADDING_X < TELEOP_PADDING_X) THROTTLE_PADDING_X = TELEOP_PADDING_X;
        if(THROTTLE_PADDING_Y < TELEOP_PADDING_Y) THROTTLE_PADDING_Y = TELEOP_PADDING_Y;
        if(THROTTLE_PADDING_X < MANUAL_PADDING_X) THROTTLE_PADDING_X = MANUAL_PADDING_X;
        if(THROTTLE_PADDING_Y < MANUAL_PADDING_Y) THROTTLE_PADDING_Y = MANUAL_PADDING_Y;
    }
    void safetyNode::makeFootprints(){
        /************
        * The following 16 footprints are used by the robot:
        * fwdFootprint, revFootprint, leftFootprint, rightFootprint;
      * fwdFootprintManual, revFootprintManual, leftFootprintManual, rightFootprintManual;
      * fwdFootprintTeleop, revFootprintTeleop, leftFootprintTeleop, rightFootprintTeleop;
      * fwdFootprintThrottle, revFootprintThrottle, leftFootprintThrottle, rightFootprintThrottle;
        * They define the footprint when moving in 4 of it's possible directions as well as in 3 modes and finally the throttle region (the larger footprint in which the robot will slow down)
        * Each footprint is a std::vector<geometry_msgs::Point32>, with the last line automatically connected (to make a box 4 points would be needed).
        * A polygon of any number of sides can be used as long as care is taken to order the points such that lines dont cross each other.
        *********************/
        geometry_msgs::Point32 temp_pt;
        //////////////////AUTONOMOUS//////////////////////
        //Forward footprint
        temp_pt.y = LEFT;
        temp_pt.x = FRONT+AUTONOMOUS_PADDING_X;
        fwdFootprint.push_back(temp_pt);
        temp_pt.y = RIGHT;
        temp_pt.x = FRONT+AUTONOMOUS_PADDING_X;
        fwdFootprint.push_back(temp_pt);
        temp_pt.y = RIGHT;
        temp_pt.x = 0;
        fwdFootprint.push_back(temp_pt);
        temp_pt.y = LEFT;
        temp_pt.x = 0;
        fwdFootprint.push_back(temp_pt);
        //Reverse footprint
        temp_pt.y = LEFT;
        temp_pt.x = 0;
        revFootprint.push_back(temp_pt);
        temp_pt.y = LEFT;
        temp_pt.x = REAR-AUTONOMOUS_PADDING_X;
        revFootprint.push_back(temp_pt);
        temp_pt.y = RIGHT;
        temp_pt.x = REAR-AUTONOMOUS_PADDING_X;
        revFootprint.push_back(temp_pt);
        temp_pt.y = RIGHT;
        temp_pt.x = 0;
        revFootprint.push_back(temp_pt);
        //Left rotation footprint
        temp_pt.y = LEFT+AUTONOMOUS_PADDING_Y;
        temp_pt.x = FRONT+AUTONOMOUS_PADDING_X;
        leftRotationFootprint.push_back(temp_pt);
        temp_pt.y = LEFT+AUTONOMOUS_PADDING_Y;
        temp_pt.x = 0;
        leftRotationFootprint.push_back(temp_pt);
        temp_pt.y = RIGHT-AUTONOMOUS_PADDING_Y;
        temp_pt.x = REAR-AUTONOMOUS_PADDING_X;
        leftRotationFootprint.push_back(temp_pt);
        temp_pt.y = RIGHT-AUTONOMOUS_PADDING_Y;
        temp_pt.x = 0;
        leftRotationFootprint.push_back(temp_pt);
        //Right rotation footprint
        temp_pt.y = RIGHT-AUTONOMOUS_PADDING_Y;
        temp_pt.x = FRONT+AUTONOMOUS_PADDING_X;
        rightRotationFootprint.push_back(temp_pt);
        temp_pt.y = RIGHT-AUTONOMOUS_PADDING_Y;
        temp_pt.x = 0;
        rightRotationFootprint.push_back(temp_pt);
        temp_pt.y = LEFT+AUTONOMOUS_PADDING_Y;
        temp_pt.x = REAR-AUTONOMOUS_PADDING_X;
        rightRotationFootprint.push_back(temp_pt);
        temp_pt.y = LEFT+AUTONOMOUS_PADDING_Y;
        temp_pt.x = 0;
        rightRotationFootprint.push_back(temp_pt);
        //Left footprint
        temp_pt.y = LEFT+AUTONOMOUS_PADDING_Y;
        temp_pt.x = FRONT;
        leftFootprint.push_back(temp_pt);
        temp_pt.y = LEFT+AUTONOMOUS_PADDING_Y;
        temp_pt.x = REAR;
        leftFootprint.push_back(temp_pt);
        temp_pt.y = 0;
        temp_pt.x = REAR;
        leftFootprint.push_back(temp_pt);
        temp_pt.y = 0;
        temp_pt.x = FRONT;
        leftFootprint.push_back(temp_pt);
        //Right footprint
        temp_pt.y = RIGHT-AUTONOMOUS_PADDING_Y;
        temp_pt.x = FRONT;
        rightFootprint.push_back(temp_pt);
        temp_pt.y = RIGHT-AUTONOMOUS_PADDING_Y;
        temp_pt.x = REAR;
        rightFootprint.push_back(temp_pt);
        temp_pt.y = 0;
        temp_pt.x = REAR;
        rightFootprint.push_back(temp_pt);
        temp_pt.y = 0;
        temp_pt.x = FRONT;
        rightFootprint.push_back(temp_pt);
        //////////////////MANUAL///////////////////////
        //Forward footprint
        temp_pt.y = LEFT+MANUAL_PADDING_Y;
        temp_pt.x = FRONT+MANUAL_PADDING_X;
        fwdFootprintManual.push_back(temp_pt);
        temp_pt.y = RIGHT-MANUAL_PADDING_Y;
        temp_pt.x = FRONT+MANUAL_PADDING_X;
        fwdFootprintManual.push_back(temp_pt);
        temp_pt.y = RIGHT-MANUAL_PADDING_Y;
        temp_pt.x = 0;
        fwdFootprintManual.push_back(temp_pt);
        temp_pt.y = LEFT+MANUAL_PADDING_Y;
        temp_pt.x = 0;
        fwdFootprintManual.push_back(temp_pt);
        //Reverse footprint
        temp_pt.y = LEFT+MANUAL_PADDING_Y;
        temp_pt.x = 0;
        revFootprintManual.push_back(temp_pt);
        temp_pt.y = LEFT+MANUAL_PADDING_Y;
        temp_pt.x = REAR-MANUAL_PADDING_X;
        revFootprintManual.push_back(temp_pt);
        temp_pt.y = RIGHT-MANUAL_PADDING_Y;
        temp_pt.x = REAR-MANUAL_PADDING_X;
        revFootprintManual.push_back(temp_pt);
        temp_pt.y = RIGHT-MANUAL_PADDING_Y;
        temp_pt.x = 0;
        revFootprintManual.push_back(temp_pt);
        //Left footprint
        temp_pt.y = LEFT+MANUAL_PADDING_Y;
        temp_pt.x = FRONT;
        leftFootprintManual.push_back(temp_pt);
        temp_pt.y = LEFT+MANUAL_PADDING_Y;
        temp_pt.x = REAR;
        leftFootprintManual.push_back(temp_pt);
        temp_pt.y = 0;
        temp_pt.x = REAR;
        leftFootprintManual.push_back(temp_pt);
        temp_pt.y = 0;
        temp_pt.x = FRONT;
        leftFootprintManual.push_back(temp_pt);
        //Right footprint
        temp_pt.y = RIGHT-MANUAL_PADDING_Y;
        temp_pt.x = FRONT;
        rightFootprintManual.push_back(temp_pt);
        temp_pt.y = RIGHT-MANUAL_PADDING_Y;
        temp_pt.x = REAR;
        rightFootprintManual.push_back(temp_pt);
        temp_pt.y = 0;
        temp_pt.x = REAR;
        rightFootprintManual.push_back(temp_pt);
        temp_pt.y = 0;
        temp_pt.x = FRONT;
        //Left rotation footprint
        temp_pt.y = LEFT+MANUAL_PADDING_Y;
        temp_pt.x = FRONT+MANUAL_PADDING_X;
        leftRotationFootprintManual.push_back(temp_pt);
        temp_pt.y = LEFT+MANUAL_PADDING_Y;
        temp_pt.x = 0;
        leftRotationFootprintManual.push_back(temp_pt);
        temp_pt.y = RIGHT-MANUAL_PADDING_Y;
        temp_pt.x = REAR-MANUAL_PADDING_X;
        leftRotationFootprintManual.push_back(temp_pt);
        temp_pt.y = RIGHT-MANUAL_PADDING_Y;
        temp_pt.x = 0;
        leftRotationFootprintManual.push_back(temp_pt);
        //Right rotation footprint
        temp_pt.y = RIGHT-MANUAL_PADDING_Y;
        temp_pt.x = FRONT+MANUAL_PADDING_X;
        rightRotationFootprintManual.push_back(temp_pt);
        temp_pt.y = RIGHT-MANUAL_PADDING_Y;
        temp_pt.x = 0;
        rightRotationFootprintManual.push_back(temp_pt);
        temp_pt.y = LEFT+MANUAL_PADDING_Y;
        temp_pt.x = REAR-MANUAL_PADDING_X;
        rightRotationFootprintManual.push_back(temp_pt);
        temp_pt.y = LEFT+MANUAL_PADDING_Y;
        temp_pt.x = 0;
        rightRotationFootprintManual.push_back(temp_pt);
        //////////////////////////TELEOP////////////////////////
        //Forward footprint
        temp_pt.y = LEFT+TELEOP_PADDING_Y;
        temp_pt.x = FRONT+TELEOP_PADDING_X;
        fwdFootprintTeleop.push_back(temp_pt);
        temp_pt.y = RIGHT-TELEOP_PADDING_Y;
        temp_pt.x = FRONT+TELEOP_PADDING_X;
        fwdFootprintTeleop.push_back(temp_pt);
        temp_pt.y = RIGHT-TELEOP_PADDING_Y;
        temp_pt.x = 0;
        fwdFootprintTeleop.push_back(temp_pt);
        temp_pt.y = LEFT+TELEOP_PADDING_Y;
        temp_pt.x = 0;
        fwdFootprintTeleop.push_back(temp_pt);
        //Reverse footprint
        temp_pt.y = LEFT+TELEOP_PADDING_Y;
        temp_pt.x = 0;
        revFootprintTeleop.push_back(temp_pt);
        temp_pt.y = LEFT+TELEOP_PADDING_Y;
        temp_pt.x = REAR-TELEOP_PADDING_X;
        revFootprintTeleop.push_back(temp_pt);
        temp_pt.y = RIGHT-TELEOP_PADDING_Y;
        temp_pt.x = REAR-TELEOP_PADDING_X;
        revFootprintTeleop.push_back(temp_pt);
        temp_pt.y = RIGHT-TELEOP_PADDING_Y;
        temp_pt.x = 0;
        revFootprintTeleop.push_back(temp_pt);
        //Left footprint
        temp_pt.y = LEFT+TELEOP_PADDING_Y;
        temp_pt.x = FRONT;
        leftFootprintTeleop.push_back(temp_pt);
        temp_pt.y = LEFT+TELEOP_PADDING_Y;
        temp_pt.x = REAR;
        leftFootprintTeleop.push_back(temp_pt);
        temp_pt.y = 0;
        temp_pt.x = REAR;
        leftFootprintTeleop.push_back(temp_pt);
        temp_pt.y = 0;
        temp_pt.x = FRONT;
        leftFootprintTeleop.push_back(temp_pt);
        //Right footprint
        temp_pt.y = RIGHT-TELEOP_PADDING_Y;
        temp_pt.x = FRONT;
        rightFootprintTeleop.push_back(temp_pt);
        temp_pt.y = RIGHT-TELEOP_PADDING_Y;
        temp_pt.x = REAR;
        rightFootprintTeleop.push_back(temp_pt);
        temp_pt.y = 0;
        temp_pt.x = REAR;
        rightFootprintTeleop.push_back(temp_pt);
        temp_pt.y = 0;
        temp_pt.x = FRONT;
        rightFootprintTeleop.push_back(temp_pt);
        //Left rotation footprint
        temp_pt.y = LEFT+TELEOP_PADDING_Y;
        temp_pt.x = FRONT+TELEOP_PADDING_X;
        leftRotationFootprintTeleop.push_back(temp_pt);
        temp_pt.y = LEFT+TELEOP_PADDING_Y;
        temp_pt.x = 0;
        leftRotationFootprintTeleop.push_back(temp_pt);
        temp_pt.y = RIGHT-TELEOP_PADDING_Y;
        temp_pt.x = REAR-TELEOP_PADDING_X;
        leftRotationFootprintTeleop.push_back(temp_pt);
        temp_pt.y = RIGHT-TELEOP_PADDING_Y;
        temp_pt.x = 0;
        leftRotationFootprintTeleop.push_back(temp_pt);
        //Right rotation footprint
        temp_pt.y = RIGHT-TELEOP_PADDING_Y;
        temp_pt.x = FRONT+TELEOP_PADDING_X;
        rightRotationFootprintTeleop.push_back(temp_pt);
        temp_pt.y = RIGHT-TELEOP_PADDING_Y;
        temp_pt.x = 0;
        rightRotationFootprintTeleop.push_back(temp_pt);
        temp_pt.y = LEFT+TELEOP_PADDING_Y;
        temp_pt.x = REAR-TELEOP_PADDING_X;
        rightRotationFootprintTeleop.push_back(temp_pt);
        temp_pt.y = LEFT+TELEOP_PADDING_Y;
        temp_pt.x = 0;
        rightRotationFootprintTeleop.push_back(temp_pt);
        //////////////////////////THROTTLE////////////////////////
        //Forward footprint
        temp_pt.y = LEFT;
        temp_pt.x = FRONT+THROTTLE_PADDING_X;
        fwdFootprintThrottle.push_back(temp_pt);
        temp_pt.y = RIGHT;
        temp_pt.x = FRONT+THROTTLE_PADDING_X;
        fwdFootprintThrottle.push_back(temp_pt);
        temp_pt.y = RIGHT;
        temp_pt.x = 0;
        fwdFootprintThrottle.push_back(temp_pt);
        temp_pt.y = LEFT;
        temp_pt.x = 0;
        fwdFootprintThrottle.push_back(temp_pt);
        //Reverse footprint
        temp_pt.y = LEFT;
        temp_pt.x = 0;
        revFootprintThrottle.push_back(temp_pt);
        temp_pt.y = LEFT;
        temp_pt.x = REAR-THROTTLE_PADDING_X;
        revFootprintThrottle.push_back(temp_pt);
        temp_pt.y = RIGHT;
        temp_pt.x = REAR-THROTTLE_PADDING_X;
        revFootprintThrottle.push_back(temp_pt);
        temp_pt.y = RIGHT;
        temp_pt.x = 0;
        revFootprintThrottle.push_back(temp_pt);
        //Left footprint
        temp_pt.y = LEFT+THROTTLE_PADDING_Y;
        temp_pt.x = FRONT;
        leftFootprintThrottle.push_back(temp_pt);
        temp_pt.y = LEFT+THROTTLE_PADDING_Y;
        temp_pt.x = REAR;
        leftFootprintThrottle.push_back(temp_pt);
        temp_pt.y = 0;
        temp_pt.x = REAR;
        leftFootprintThrottle.push_back(temp_pt);
        temp_pt.y = 0;
        temp_pt.x = FRONT;
        leftFootprintThrottle.push_back(temp_pt);
        //Right footprint
        temp_pt.y = RIGHT-THROTTLE_PADDING_Y;
        temp_pt.x = FRONT;
        rightFootprintThrottle.push_back(temp_pt);
        temp_pt.y = RIGHT-THROTTLE_PADDING_Y;
        temp_pt.x = REAR;
        rightFootprintThrottle.push_back(temp_pt);
        temp_pt.y = 0;
        temp_pt.x = REAR;
        rightFootprintThrottle.push_back(temp_pt);
        temp_pt.y = 0;
        temp_pt.x = FRONT;
        rightFootprintThrottle.push_back(temp_pt);
        //Left rotation footprint
        temp_pt.y = LEFT+ROTATION_Y;
        temp_pt.x = FRONT+ROTATION_X;
        leftRotationFootprintThrottle.push_back(temp_pt);
        temp_pt.y = LEFT+ROTATION_Y;
        temp_pt.x = 0;
        leftRotationFootprintThrottle.push_back(temp_pt);
        temp_pt.y = RIGHT-ROTATION_Y;
        temp_pt.x = REAR-ROTATION_X;
        leftRotationFootprintThrottle.push_back(temp_pt);
        temp_pt.y = RIGHT-ROTATION_Y;
        temp_pt.x = 0;
        leftRotationFootprintThrottle.push_back(temp_pt);
        //Right rotation footprint
        temp_pt.y = RIGHT-ROTATION_Y;
        temp_pt.x = FRONT+ROTATION_X;
        rightRotationFootprintThrottle.push_back(temp_pt);
        temp_pt.y = RIGHT-ROTATION_Y;
        temp_pt.x = 0;
        rightRotationFootprintThrottle.push_back(temp_pt);
        temp_pt.y = LEFT+ROTATION_Y;
        temp_pt.x = REAR-ROTATION_X;
        rightRotationFootprintThrottle.push_back(temp_pt);
        temp_pt.y = LEFT+ROTATION_Y;
        temp_pt.x = 0;
        rightRotationFootprintThrottle.push_back(temp_pt);
    }
