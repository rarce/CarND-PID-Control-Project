#ifndef PID_H
#define PID_H

#include <vector>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * twiddle search
  */
  const int EVAL_STEPS = 1000;
  const int SETTLE_STEPS = 100;

  bool twiddle;
  double total_error;
  double min_error;
  int steps;
  int param_index;
  int search = 1; // options add, sub
  
  std::vector<double> dp;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd, bool twiddle);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  void UpdateParam(double dp);

  void NextParam(void);

  void showSearchStatus(void);
};

#endif /* PID_H */
