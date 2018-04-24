/*
 * heroehs_signal_processing.cpp
 *
 *  Created on: 2018. 2. 27.
 *      Author: Crowban
 */

#include "heroehs_math/heroehs_signal_processing.h"

using namespace heroehs;

LowPassFilter::LowPassFilter()
{
  cut_off_freq_ = 1.0;
  control_cycle_sec_ = 0.008;
  prev_output_ = 0;

  alpha_ = (2.0*M_PI*cut_off_freq_*control_cycle_sec_)/(1.0+2.0*M_PI*cut_off_freq_*control_cycle_sec_);
}

LowPassFilter::LowPassFilter(double control_cycle_sec, double cut_off_frequency)
{
  cut_off_freq_ = cut_off_frequency;
  control_cycle_sec_ = control_cycle_sec;
  prev_output_ = 0;

  if(cut_off_frequency > 0)
    alpha_ = (2.0*M_PI*cut_off_freq_*control_cycle_sec_)/(1.0+2.0*M_PI*cut_off_freq_*control_cycle_sec_);
  else
    alpha_ = 1;
}

LowPassFilter::~LowPassFilter()
{ }

void LowPassFilter::initialize(double control_cycle_sec, double cut_off_frequency)
{
  cut_off_freq_ = cut_off_frequency;
  control_cycle_sec_ = control_cycle_sec;
  prev_output_ = 0;

  if(cut_off_frequency > 0)
    alpha_ = (2.0*M_PI*cut_off_freq_*control_cycle_sec_)/(1.0+2.0*M_PI*cut_off_freq_*control_cycle_sec_);
  else
    alpha_ = 1;
}

void LowPassFilter::setCutOffFrequency(double cut_off_frequency)
{
  cut_off_freq_ = cut_off_frequency;

  if(cut_off_frequency > 0)
    alpha_ = (2.0*M_PI*cut_off_freq_*control_cycle_sec_)/(1.0+2.0*M_PI*cut_off_freq_*control_cycle_sec_);
  else
    alpha_ = 1;
}

double LowPassFilter::getCutOffFrequency(void)
{
  return cut_off_freq_;
}

double LowPassFilter::getFilteredOutput(double present_raw_value)
{
  prev_output_ = alpha_*present_raw_value + (1.0 - alpha_)*prev_output_;
  return prev_output_;
}


