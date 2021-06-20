/// @brief PID controller
/// @details definition for functions of the class PID. 
/// @author s.aparajith@live.com
/// @date 19.06.2021
/// @license MIT

#include "PID.h"
#include "integrator.h"
#include <algorithm>
#include <numeric>
#include <iostream>

PID::PID():
m_integ(),
m_sqError(),
m_p_error(0.0),
m_i_error(0.0),
m_d_error(0.0),
m_kp(0.0),
m_ki(0.0),
m_kd(0.0),
m_count(0)
 {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  m_kp=Kp_;
  m_ki=Ki_;
  m_kd=Kd_;
  m_p_error=0.0;
  m_i_error=(0.0);
  m_d_error=(0.0);
  std::cout<<"Kp: "<<Kp_<<"Ki: "<<Ki_<<"Kd: "<<Kd_<<std::endl;
}

void PID::UpdateError(double cte) 
{
  // derivative
  m_d_error =cte-m_p_error;
  // proportional
  m_p_error = cte;
  //integral
  m_i_error = m_integ.integrate(cte,-1.0,1.0);
  if(m_count>=100)
  {//compute average error for twiddle
  m_sqError.meanSquareError(cte*cte);
  }
  else
  {
    m_count++;
  }
}
double PID::getMSEerror()
{
 return -1;// m_sqError.getMeanSquareError();
}

double PID::TotalError() {
  return (m_kp*m_p_error+m_kd*m_d_error+m_ki*m_i_error);  
}

double PID::Control()
{
  double ctrl=-1.0*(this->TotalError());
  return saturate(ctrl,-1.0,1.0);

}
