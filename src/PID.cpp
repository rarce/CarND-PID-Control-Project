#include "PID.h"
#include <iostream>


using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, bool twiddle) {
  PID::Kp = Kp;
  PID::Ki = Ki;
  PID::Kd = Kd;

  if (twiddle) {
    PID::twiddle = twiddle;
    min_error = numeric_limits<double>::max();
    param_index = 2;
    steps = 0;
    total_error = 0;

    dp = {Kp*0.25, Ki*0.25, Kd*0.25};
    showSearchStatus();
  }
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;

  if (twiddle) {
    if (steps > SETTLE_STEPS)
      total_error += cte*cte;
    steps +=1;
    
    // eval steps
    if (steps % (EVAL_STEPS+SETTLE_STEPS) == 0 || total_error > min_error) {
      if (total_error <= min_error) {
        showSearchStatus();
        std::cout << "Improve param:" << param_index << std::endl;
        min_error = total_error;
        dp[param_index] *= 1.1;
        // set next param
        search = 1;
        NextParam();
        UpdateParam(dp[param_index]);
      } else if (total_error > min_error && search==-1) {
        showSearchStatus();
        std::cout << "Worst decrease step param:" << param_index << std::endl;
        double update_dp = dp[param_index];
        UpdateParam(update_dp);
        dp[param_index] *= 0.9;
        search = 1;
        NextParam();
        UpdateParam(dp[param_index]);
      } else {
        showSearchStatus();
        std::cout << "Worst change direction param:" << param_index << std::endl;
        UpdateParam(-2*dp[param_index]);
        search=-1;
      }
      // reset error
      steps = 0;
      total_error = 0;
    }
  }
}

double PID::TotalError() {
  return (Kp * p_error + Kd * d_error + Ki * i_error);
}

void PID::UpdateParam(double dp) {
  if (param_index==0) {
    Kp += dp;
  } else if (param_index==1) {
    Ki += dp;
  } else {
    Kd += dp;
  }
}

void PID::NextParam(){
  param_index = (param_index + 1) % 3;
}

void PID::showSearchStatus(){
  std::cout << "------------------------------------------------------" << std::endl;
  std::cout << "Iteration: " << steps << " Error: " << total_error << std::endl;
  std::cout << "Kp: " << Kp << " Ki: " << Ki << " Kd: " << Kd << std::endl;
  std::cout << "dp: " << dp[0] << ", " << dp[1] << ", " << dp[2] << std::endl;
  std::cout << "param_index: " << param_index << " dp " << dp[param_index] << " dir " << search << std::endl;
  std::cout << "------------------------------------------------------" << std::endl;
}