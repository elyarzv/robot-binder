/* 
    AIS CONFIDENTIAL
    Author: Michael Wrock (m.wrock@ai-systems.ca)
*/

#ifndef AIS_LED_ABSTRACT_PATTERN_H_
#define AIS_LED_ABSTRACT_PATTERN_H_

#include "ros/ros.h"
#include "std_msgs/UInt32MultiArray.h"
#include <vector>

using std::shared_ptr;

namespace ais_led_strip_controller
{
/**
 * @brief 
 */
class AbstractPattern
{
    public:
        /*
        *   @brief Constructor
        */
        AbstractPattern();
        /*
        *   @brief Destructor
        */
        virtual ~AbstractPattern() = default;
        virtual std_msgs::UInt32MultiArray getFrame() = 0;
        virtual void setColour(float colour) = 0;
        void setRGB(int red, int green, int blue);
        virtual void setSize(int size) = 0;
        virtual void setFillPercent(float fill) = 0;


    protected:
        float colour_;
        int size_;
        float fill_percent_;
        std_msgs::UInt32MultiArray frame_;

};
} // end of namespace ais_led_strip_controller

#endif // END AIS_LED_ABSTRACT_PATTERN_H_