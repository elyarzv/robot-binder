/* 
    AIS CONFIDENTIAL
    Author: Michael Wrock (m.wrock@ai-systems.ca)
*/

#ifndef AIS_LED_FILLING_PATTERN_H_
#define AIS_LED_FILLING_PATTERN_H_

#include "ais_led_strip_controller/Patterns/abstract_pattern.h"

using std::shared_ptr;

#define FILL_RATE 0.1 //Percent fill increase for each frame

namespace ais_led_strip_controller
{
/**
 * @brief 
 */
class FillingPattern final: public AbstractPattern
{
    public:
        FillingPattern();
        FillingPattern(int size);
        virtual ~FillingPattern() override;
        std_msgs::UInt32MultiArray getFrame() override;
        void setColour(float colour) override;
        void setSize(int size) override;
        void setFillPercent(float fill) override;


    private:
        float current_fill_;

};
} // end of namespace ais_led_strip_controller

#endif // END AIS_LED_FILLING_PATTERN_H_