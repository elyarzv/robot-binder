#include "ais_led_strip_controller/Patterns/blinking_pattern.h"

using namespace ais_led_strip_controller;

BlinkingPattern::BlinkingPattern(){
    colour_ = 0;
    size_ = 0;
    fill_percent_ = 100;
    blink = 1;
}
BlinkingPattern::BlinkingPattern(int size){
    colour_ = 0;
    size_ = size;
    fill_percent_ = 100;
    frame_.data.resize(size);
    blink = 1;
}
BlinkingPattern::~BlinkingPattern(){
    
}

void BlinkingPattern::setColour(float colour){

    colour_ = colour;
}

void BlinkingPattern::setSize(int size){
    if(size >= 0) size_ = size;
    else size_ = 0;
    frame_.data.resize(size_);
}
void BlinkingPattern::setFillPercent(float fill){
    if(fill > 100) fill_percent_ = 100;
    else if(fill < 0) fill_percent_ = 0;
    else    fill_percent_ = fill;
}
std_msgs::UInt32MultiArray BlinkingPattern::getFrame(){
    if(blink < 0) {std::fill (frame_.data.begin(), frame_.data.end()+1,0);}
    else{
        int fill_size = ceil(fill_percent_ * size_ / 100);
        if(fill_size > size_) {std::fill (frame_.data.begin(), frame_.data.end()+1,colour_);}
        else if(fill_size > 0) {std::fill (frame_.data.begin(), frame_.data.begin()+fill_size,colour_);}
    }
    blink = blink * -1;
    return frame_;
}