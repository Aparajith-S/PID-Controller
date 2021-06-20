#ifndef PID_H
#define PID_H
#include "integrator.h"
#include <deque>
#include<cstdlib>
class MSE
{
  public:
  double meanSquareError(double val)
  {
    m_store=m_storage.integrate(val);
    m_store/=m_storage.size();
    return m_store;
  }
  double getMeanSquareError()
  {
    return m_store;
  }
  MSE(double init=0.0,double Ns=kMaxIntegSize):
    m_store(0.0),
    m_N(Ns){}
  virtual ~MSE(){}
  private:
    limitedIntegrator m_storage;
    double m_store;
    double m_N;
};
static inline double saturate(double val,double min_val, double max_val)
{
  return std::max(std::min(val,max_val),min_val);
}

class PID {
 public:
  PID();
  virtual ~PID();

  ///@brief Initialize PID.
  ///@param (Kp_, Ki_, Kd_) The initial PID coefficients  
  void Init(double Kp_, double Ki_, double Kd_);

  ///@brief Update the PID error variables given cross track error.
  ///@param cte The current cross track error
  void UpdateError(double cte);

double Control();
  
  /// @brief Calculate the total PID error.
  /// @return The total PID error
  double TotalError();

  /// @brief get the MSE error for twiddle algorithm.
  /// @return The total MSE
  double getMSEerror();
 private:
  static constexpr double kdT =1.0; 
  limitedIntegrator m_integ;
  MSE m_sqError;
  //PID Errors   
  double m_p_error;
  double m_i_error;
  double m_d_error;
  //PID Coefficients
  double m_kp;
  double m_ki;
  double m_kd;
  int m_count;
};

#endif  // PID_H
