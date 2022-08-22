#include "ais_led_strip_controller/Patterns/solid_pattern.h"

using namespace ais_led_strip_controller;

SolidPattern::SolidPattern() {
  colour_ = 0;
  size_ = 0;
  fill_percent_ = 100;
}
SolidPattern::SolidPattern(int size) {
  colour_ = 0;
  size_ = size;
  fill_percent_ = 100;
  frame_.data.resize(size);
}
SolidPattern::~SolidPattern() {}

void SolidPattern::setColour(float colour) {
  colour_ = colour;
  int fill_size = floor((float)size_ * (fill_percent_ / 100));
  if (fill_size > 0) {
    std::fill(frame_.data.begin(), frame_.data.begin() + fill_size, colour);
  }
}

void SolidPattern::setSize(int size) {
  if (size >= 0) {
    size_ = size;
  } else {
    size_ = 0;
  }
  frame_.data.resize(size);
  int fill_size = floor((float)size_ * (fill_percent_ / 100));
  if (fill_size != 0) {
    std::fill(frame_.data.begin(), frame_.data.begin() + fill_size, colour_);
  }
}
void SolidPattern::setFillPercent(float fill) {
  if (fill > 100)
    fill_percent_ = 100;
  else if (fill < 0)
    fill_percent_ = 0;
  else
    fill_percent_ = fill;
  std::fill(frame_.data.begin(), frame_.data.end() + 1, 0);
  int fill_size = floor((float)size_ * (fill_percent_ / 100));
  std::fill(frame_.data.begin(), frame_.data.begin() + fill_size, colour_);
}
std_msgs::UInt32MultiArray SolidPattern::getFrame() { return frame_; }