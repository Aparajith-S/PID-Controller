#ifndef TWIDDLE_H
#define TWIDDLE_H
#include"PID.h"
#include<limits>
#include<vector>
#include<cstdint>
namespace twiddle{
enum class state: std::uint8_t
{
    kInit=0U,
    kIncrement= 1U,
    kDecrement=2U
};
class Twiddle
{
public:
Twiddle():
m_twiddle_on(true),
m_best_error(std::numeric_limits<double>::max()),
m_state(state::kInit),
m_idx(0),
m_iterations(0),
m_p{0.3,0.0004,4.0},
m_dp{0.03,0.00004,0.4}
{}
void init(std::vector<double> values)
{
    m_p=values;
    m_dp[0] = values[0]/10.0;
    m_dp[1] = values[0]/10.0;
    m_dp[2] = values[0]/10.0;
}
virtual ~Twiddle(){}
void cyclic(PID & io_pid);
private:
    bool m_twiddle_on;
    double m_best_error;
    state m_state;
    int m_idx = 0;
    int m_iterations = 0;
    std::vector<double> m_p;
    std::vector<double> m_dp;
};
}
#endif