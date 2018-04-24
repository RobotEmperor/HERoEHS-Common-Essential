/*
 * heroehs_pid_control.h
 *
 *  Created on: 2018. 2. 27.
 *      Author: Crowban
 */

#ifndef HEROEHS_MATH_HEROEHS_PID_CONTROL_H_
#define HEROEHS_MATH_HEROEHS_PID_CONTROL_H_

namespace heroehs
{

class PDController
{
public:
  PDController(double control_time_sec = 0.008);
  ~PDController();

  double desired_;

  double p_gain_, d_gain_;

  double getFeedBack(double present_sensor_output);

private:
  double control_time_sec_;
  double curr_err_;
  double prev_err_;
};

class PIDController
{
public:
  PIDController(double control_time_sec = 0.008);
  ~PIDController();

  double desired_;

  double p_gain_, i_gain_, d_gain_;

  double getFeedBack(double present_sensor_output);

private:
  double control_time_sec_;
  double curr_err_;
  double prev_err_;
  double sum_err_;
};

}



#endif /* HEROEHS_MATH_HEROEHS_PID_CONTROL_H_ */
