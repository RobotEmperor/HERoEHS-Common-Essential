/*
 * heroehs_pid_control.cpp
 *
 *  Created on: 2018. 2. 27.
 *      Author: Crowban
 */

#include "heroehs_math/heroehs_pid_control.h"


using namespace heroehs;

PDController::PDController(double control_time_sec)
{
  control_time_sec_ = control_time_sec;
  desired_ = 0;
  p_gain_ = 0;
  d_gain_ = 0;
  curr_err_ = 0;
  prev_err_ = 0;
}

PDController::~PDController()
{  }


double PDController::getFeedBack(double present_sensor_output)
{
  prev_err_ = curr_err_;
  curr_err_ = desired_ - present_sensor_output;

  return (p_gain_*curr_err_ + d_gain_*(curr_err_ - prev_err_)/control_time_sec_);
}

PIDController::PIDController(double control_time_sec)
{
  control_time_sec_ = control_time_sec;
  desired_ = 0;
  p_gain_ = 0;
  i_gain_ = 0;
  d_gain_ = 0;
  curr_err_ = 0;
  prev_err_ = 0;
  sum_err_ = 0;
}

PIDController::~PIDController()
{  }


double PIDController::getFeedBack(double present_sensor_output)
{
  prev_err_ = curr_err_;
  curr_err_ = desired_ - present_sensor_output;
  sum_err_ += curr_err_;

  return (p_gain_*curr_err_ + i_gain_*sum_err_*control_time_sec_ + d_gain_*(curr_err_ - prev_err_)/control_time_sec_);
}
