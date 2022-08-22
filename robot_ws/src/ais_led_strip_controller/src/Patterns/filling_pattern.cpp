#include "ais_led_strip_controller/Patterns/filling_pattern.h"

using namespace ais_led_strip_controller;

FillingPattern::FillingPattern(){
    colour_ = 0;
    size_ = 0;
    fill_percent_ = 100;
    current_fill_ = -FILL_RATE;
}
FillingPattern::FillingPattern(int size){
    colour_ = 0;
    size_ = size;
    fill_percent_ = 100;
    frame_.data.resize(size);
    current_fill_ = -FILL_RATE;
}
FillingPattern::~FillingPattern(){
    
}

void FillingPattern::setColour(float colour){

    colour_ = colour;
    current_fill_ = -FILL_RATE;
}

void FillingPattern::setSize(int size){
    if(size >= 0) size_ = size;
    else size_ = 0;
    frame_.data.resize(size_);
    current_fill_ = -FILL_RATE;
}
void FillingPattern::setFillPercent(float fill){
    if(fill > 100) fill_percent_ = 100;
    else if(fill < 0) fill_percent_ = 0;
    else    fill_percent_ = floor(fill*size_/100);
    current_fill_ = -FILL_RATE;
}
std_msgs::UInt32MultiArray FillingPattern::getFrame(){
    std::fill (frame_.data.begin(), frame_.data.end()+1,0);
    current_fill_ += FILL_RATE;
    int fill_size = ceil(fill_percent_ * current_fill_);
    if(fill_size > size_) {std::fill (frame_.data.begin(), frame_.data.end()+1,colour_);}
    else if(fill_size > 0) {std::fill (frame_.data.begin(), frame_.data.begin()+fill_size,colour_);}
    if(current_fill_ >= 1) {current_fill_ = -FILL_RATE;}
    return frame_;
}