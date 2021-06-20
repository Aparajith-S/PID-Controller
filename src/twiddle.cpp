#include"twiddle.h"
#include<iostream>
#include<cmath>
#include<cstdlib>
namespace twiddle{
void Twiddle::cyclic(PID & io_pid)
{
    double pid_err = io_pid.getMSEerror();
    switch(m_state)
    {
        case state::kInit:
            m_best_error = pid_err;
            m_p[m_idx] += m_dp[m_idx];
            m_state = state::kIncrement;
        break;
        case state::kIncrement:
            if (pid_err<m_best_error) 
            {
                m_best_error = pid_err;
                m_dp[m_idx] *= 1.1;
                m_idx = (m_idx + 1) % 3; //rotate over the 3 vector indices
                m_p[m_idx] += m_dp[m_idx];
            } 
            else 
            {
                m_p[m_idx] -= 2.0 * m_dp[m_idx];
                if (m_p[m_idx] < 0) 
                {
                    m_p[m_idx] =  m_dp[m_idx];
                    m_idx = (m_idx + 1) % 3;
                }
                m_state = state::kDecrement;
            }
        break;
        case state::kDecrement: 
            if (pid_err<m_best_error ) 
            {
                m_best_error = pid_err;
                m_dp[m_idx] *= 1.1;
            }  
            else 
            {
                m_p[m_idx] += m_dp[m_idx];
                m_dp[m_idx] *= 0.95;
            }
            m_state = state::kIncrement;
            m_idx = (m_idx + 1) % 3;
            m_p[m_idx] += m_dp[m_idx];
        break;
        io_pid.Init(m_p[0], m_p[1], m_p[2]);
    }
}
}
