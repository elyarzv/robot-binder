/* 
    AIS CONFIDENTIAL
    Author: Michael Wrock (m.wrock@ai-systems.ca)
*/

#ifndef AIS_LED_SOLID_PATTERN_H_
#define AIS_LED_SOLID_PATTERN_H_

#include "ais_led_strip_controller/Patterns/abstract_pattern.h"

using std::shared_ptr;

namespace ais_led_strip_controller
{
/**
 * @brief 
 */
class SolidPattern final: public AbstractPattern
{
    public:
        SolidPattern();
        SolidPattern(int size);
        virtual ~SolidPattern() override;
        std_msgs::UInt32MultiArray getFrame() override;
        void setColour(float colour) override;
        void setSize(int size) override;
        void setFillPercent(float fill) override;


    private:

};
} // end of namespace ais_led_strip_controller

#endif // END AIS_LED_SOLID_PATTERN_H_