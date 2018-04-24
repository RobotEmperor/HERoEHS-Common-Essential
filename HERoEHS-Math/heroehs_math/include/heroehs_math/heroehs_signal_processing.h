/*
 * heroehs_signal_processing.h
 *
 *  Created on: 2018. 2. 27.
 *      Author: Crowban
 */

#ifndef HEROEHS_MATH_HEROEHS_SIGNAL_PROCESSING_H_
#define HEROEHS_MATH_HEROEHS_SIGNAL_PROCESSING_H_

#include <cmath>

namespace heroehs
{

class LowPassFilter
{
public:
  LowPassFilter();
  LowPassFilter(double control_cycle_sec, double cut_off_frequency);
  ~LowPassFilter();

  void initialize(double control_cycle_sec_, double cut_off_frequency);
  void setCutOffFrequency(double cut_off_frequency);
  double getCutOffFrequency(void);
  double getFilteredOutput(double present_raw_value);

private:
  double cut_off_freq_;
  double control_cycle_sec_;
  double alpha_;

  double prev_output_;
};

}


#endif /* HEROEHS_MATH_HEROEHS_SIGNAL_PROCESSING_H_ */
