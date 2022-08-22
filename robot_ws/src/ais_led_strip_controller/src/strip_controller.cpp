#include "ais_led_strip_controller/strip_controller.h"

using namespace ais_led_strip_controller;

LEDController::LEDController(ros::NodeHandle& nh) : nh_(nh) {
  led_strip_publisher =
      nh.advertise<std_msgs::UInt32MultiArray>("embedded/LED_strip", 1000);
  led_pattern_server = nh_.advertiseService("set_led_pattern",
                                            &LEDController::setPatternCB, this);
  pattern_vector_.resize(4);
  pattern_vector_[phoenix_msgs::LedPattern::Request::SOLID] =
      make_shared<SolidPattern>();
  pattern_vector_[phoenix_msgs::LedPattern::Request::FILLING] =
      make_shared<FillingPattern>();
  pattern_vector_[phoenix_msgs::LedPattern::Request::SCROLLING] =
      make_shared<ScrollingPattern>();
  pattern_vector_[phoenix_msgs::LedPattern::Request::BLINKING] =
      make_shared<BlinkingPattern>();
  current_pattern_ = phoenix_msgs::LedPattern::Request::SOLID;
  loop_rate_ = 1;
  publish_timer_ = nh_.createTimer(ros::Duration(1 / loop_rate_),
                                   &LEDController::publishFrame, this);
}

LEDController::~LEDController() {}
void LEDController::publishFrame(const ros::TimerEvent& event) {
  led_strip_publisher.publish(pattern_vector_[current_pattern_]->getFrame());
}
void LEDController::setRate(double rate) {
  loop_rate_ = rate;
  publish_timer_ = nh_.createTimer(ros::Duration(1 / loop_rate_),
                                   &LEDController::publishFrame, this);
}

bool LEDController::setPatternCB(phoenix_msgs::LedPattern::Request &req, phoenix_msgs::LedPattern::Response &res){
    
    if(req.pattern >= pattern_vector_.size()){
        ROS_ERROR("Invalid Pattern");
        res.response = false;
        return false;
    }else{
        ROS_INFO_STREAM("Request receieved: " << req);
        current_pattern_ = req.pattern;
        pattern_vector_[current_pattern_]->setSize(req.size);
        if(req.use_RGB) {pattern_vector_[current_pattern_]->setRGB(req.red, req.green, req.blue);}
        else {pattern_vector_[current_pattern_]->setColour(req.colour);}
        pattern_vector_[current_pattern_]->setFillPercent(req.fill_percent);
        if(loop_rate_ != req.rate){
            loop_rate_ = req.rate;
            publish_timer_ = nh_.createTimer(ros::Duration(1/loop_rate_), &LEDController::publishFrame, this);
        }
        led_strip_publisher.publish(pattern_vector_[current_pattern_]->getFrame());
        res.response = true;
        return true;
    }
    pattern_vector_[current_pattern_]->setFillPercent(req.fill_percent);
    if (loop_rate_ != req.rate) {
      loop_rate_ = req.rate;
      publish_timer_ = nh_.createTimer(ros::Duration(1 / loop_rate_),
                                       &LEDController::publishFrame, this);
    }
    res.response = true;
    return true;
}

void LEDController::run() { ros::spin(); }
