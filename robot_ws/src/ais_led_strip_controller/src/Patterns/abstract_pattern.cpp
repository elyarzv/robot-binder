#include "ais_led_strip_controller/Patterns/abstract_pattern.h"


namespace ais_led_strip_controller
{
    AbstractPattern::AbstractPattern()
    {
    
    }
        
void AbstractPattern::setRGB(int red, int green, int blue){
    float colour = blue + green*256 + red*65536;
    setColour(colour);
}

} // end of namespace ais_led_strip_controller