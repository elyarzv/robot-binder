#include "ais_led_strip_controller/Patterns/scrolling_pattern.h"

using namespace ais_led_strip_controller;

ScrollingPattern::ScrollingPattern(){
    colour_ = 0;
    size_ = 0;
    fill_percent_ = 100;
    current_pos_ = 0;
}
ScrollingPattern::ScrollingPattern(int size){
    colour_ = 0;
    size_ = size;
    fill_percent_ = 100;
    frame_.data.resize(size);
    current_pos_ = 0;
}
ScrollingPattern::~ScrollingPattern(){
    
}

void ScrollingPattern::setColour(float colour){

    colour_ = colour;
    current_pos_ = 0;
}

void ScrollingPattern::setSize(int size){
    if(size >= 0) size_ = size;
    else size_ = 0;
    frame_.data.resize(size_);
    current_pos_ = 0;
}
void ScrollingPattern::setFillPercent(float fill){
    if(fill > 100) fill_percent_ = 100;
    else if(fill < 0) fill_percent_ = 0;
    else    fill_percent_ = fill;
    current_pos_ = 0;
}
std_msgs::UInt32MultiArray ScrollingPattern::getFrame(){
    std::fill (frame_.data.begin(), frame_.data.end()+1,0);
    frame_.data[current_pos_++] = colour_;
    if(current_pos_ >= ceil(fill_percent_ * size_ / 100)) {current_pos_ = 0;}
    return frame_;
}