/* 
    AIS CONFIDENTIAL
    Author: Michael Wrock (m.wrock@ai-systems.ca)
*/

#ifndef AIS_LED_STRIP_CONTROLLER_H_
#define AIS_LED_STRIP_CONTROLLER_H_

#include "ros/ros.h"
#include "std_msgs/UInt32MultiArray.h"
#include "ais_led_strip_controller/Patterns/solid_pattern.h"
#include "ais_led_strip_controller/Patterns/filling_pattern.h"
#include "ais_led_strip_controller/Patterns/scrolling_pattern.h"
#include "ais_led_strip_controller/Patterns/blinking_pattern.h"
#include "phoenix_msgs/LedPattern.h"
#include "ais_led_strip_controller/pattern_enums.h"
#include <map>
#include <memory>

namespace ais_led_strip_controller
{

using std::string;
using std::map;
using std::shared_ptr;
using std::make_shared;

/**
 * @brief 
 */
class LEDController
{
    public:
        /*
        *   @brief Constructor
        */
        LEDController(ros::NodeHandle&);
        /*
        *   @brief Destructor
        */
        ~LEDController();
        void run();
        void setRate(double rate);
        /**
         * @brief Publishes and advances the frame of the animation
         * 
         * @param event 
         */
        void publishFrame(const ros::TimerEvent& event);
        bool setPatternCB(phoenix_msgs::LedPattern::Request &req, phoenix_msgs::LedPattern::Response &res);


    private:
        ros::NodeHandle nh_;
        ros::Publisher led_strip_publisher;
        ros::ServiceServer led_pattern_server;
        ros::Timer publish_timer_;
        double loop_rate_;
        int current_pattern_;
        std::vector<shared_ptr<AbstractPattern>> pattern_vector_;

};
} // end of namespace ais_led_strip_controller

#endif // END AIS_LED_STRIP_CONTROLLER_H_