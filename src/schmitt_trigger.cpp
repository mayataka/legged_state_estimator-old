#include "legged_state_estimator/schmitt_trigger.hpp"


namespace legged_state_estimator {

SchmittTrigger::SchmittTrigger(const SchmittTriggerSettings& settings)
  : lower_threshold_(settings.lower_threshold),
    higher_threshold_(settings.higher_threshold),
    timer_(0), 
    previous_time_(0),
    lower_time_delay_(settings.lower_time_delay), 
    higher_time_delay_(settings.higher_time_delay) {
}


SchmittTrigger::SchmittTrigger()
  : lower_threshold_(0),
    higher_threshold_(0),
    timer_(0), 
    previous_time_(0),
    lower_time_delay_(0), 
    higher_time_delay_(0) {
}


SchmittTrigger::~SchmittTrigger() {}


void SchmittTrigger::reset() {
  state_ = false;
  previous_time_ = 0; 
  timer_ = 0;
  first_call_ = true;
}


void SchmittTrigger::update(const uint64_t current_time, const double value) {
  if (first_call_) {
    first_call_ = false;
    previous_time_ = current_time;
  }
  if (state_) {
    if (value <= lower_threshold_) {
      if (timer_ > lower_time_delay_) 
        state_ = false;
      else 
        timer_ += (current_time - previous_time_);
    } 
    else {
      timer_ = 0;
    }
  } 
  else {
    if (value >= higher_threshold_) {
      if (timer_ > higher_time_delay_) 
        state_ = true;
      else 
        timer_ += (current_time - previous_time_);
    } 
    else {
      timer_ = 0;
    }
  }
}


bool SchmittTrigger::getState() const {
  return state_;
}


void SchmittTrigger::setParameters(const SchmittTriggerSettings& settings) {
  lower_threshold_ = settings.lower_threshold;
  higher_threshold_ = settings.higher_threshold;
  lower_time_delay_ = settings.lower_time_delay; 
  higher_time_delay_ = settings.higher_time_delay;
}

}