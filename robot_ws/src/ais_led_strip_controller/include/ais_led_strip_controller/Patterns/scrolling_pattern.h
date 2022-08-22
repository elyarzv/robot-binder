/* 
    AIS CONFIDENTIAL
    Author: Michael Wrock (m.wrock@ai-systems.ca)
*/

#ifndef AIS_LED_SCROLLING_PATTERN_H_
#define AIS_LED_SCROLLING_PATTERN_H_

#include "ais_led_strip_controller/Patterns/abstract_pattern.h"

using std::shared_ptr;

#define SCROLL_RATE 1 //Number of leds to scroll per frame

namespace ais_led_strip_controller
{
/**
 * @brief 
 */
class ScrollingPattern final: public AbstractPattern
{
    public:
        ScrollingPattern();
        ScrollingPattern(int size);
        virtual ~ScrollingPattern() override;
        std_msgs::UInt32MultiArray getFrame() override;
        void setColour(float colour) override;
        void setSize(int size) override;
        void setFillPercent(float fill) override;


    private:
        float current_pos_;

};
} // end of namespace ais_led_strip_controller

#endif // END AIS_LED_SCROLLING_PATTERN_H_