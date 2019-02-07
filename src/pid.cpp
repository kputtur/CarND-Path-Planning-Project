/*
 * PID controller from term2
 *
 * file:pid.cpp
 *
 *
 */
#include "PID.h"

#include <cmath>
#include <numeric>
#include <iostream>

// constructor PID, initialize to zero
PID::PID() {
  errors_ = std::vector<double>();
  current_cross_track_error_ = 0;
  previous_cross_track_error_ = 0;
  total_cross_track_error_ = 0;
  Kd_ = 0;
  Ki_ = 0;
  Kp_ = 0;
}

// destructor
PID::~PID() {}


// initialize PID.
// @param (Kp_, Ki_, Kd_) The initial PID coefficients 
void PID::init(const double parameters[], const bool reset_errors) {
  Kp_ = parameters[0];
  Ki_ = parameters[1];
  Kd_ = parameters[2];
  if (reset_errors) {
    errors_ = std::vector<double>();
    current_cross_track_error_ = 0;
    previous_cross_track_error_ = 0;
    total_cross_track_error_ = 0;
  }
}

//get parameters 
void PID::get_parameters(double new_parameters[]) {
  new_parameters[0] = Kp_;
  new_parameters[1] = Ki_;
  new_parameters[2] = Kd_;
}

//compute control value
double PID::compute_control_value(const double cross_track_error) {
  previous_cross_track_error_ = current_cross_track_error_;
  current_cross_track_error_ = cross_track_error;
  errors_.push_back(std::abs(cross_track_error));

  return
     - current_cross_track_error_ * Kp_
     - (current_cross_track_error_ - previous_cross_track_error_) * Kd_;
}

//reset the value in case of errors.
double PID::get_and_reset_total_error() {
  current_cross_track_error_ = 0;
  previous_cross_track_error_ = 0;
  total_cross_track_error_ = 0;
  double total_error = std::accumulate(errors_.begin(), errors_.end(), 0.0);
  errors_.clear();
  return total_error;
}
