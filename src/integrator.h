/// @brief limited integral with limited storage. 
/// @details on an embedded platform resources are scarce hence a limited integrator will yield really 
/// good results as the integration is done smoothly by a moving window.
/// @author s.aparajith@live.com
/// @date 19.06.2021
/// @license MIT

#ifndef INTEGRATOR_H
#define INTEGRATOR_H
#include <deque>
#include <numeric>
#include <limits>
constexpr size_t kMaxIntegSize = 100U;
constexpr double kMaxDBL = std::numeric_limits<double>::max();
constexpr double kMinDBL = std::numeric_limits<double>::min();
/// \class moving window integrator. 
class limitedIntegrator
{
    public:
      limitedIntegrator(double dT=1.0,
      double init=0.0,
      size_t UserSize=kMaxIntegSize):
            m_storage(),
            m_offset(init),
            m_dT(dT),
            m_N(std::min(UserSize,kMaxIntegSize))
            {}
      virtual ~limitedIntegrator(){}
      
      /// @brief Reset the integrator with an initial value.
      void reset(double init=0.0)
      {
         m_offset=init;
      }

      /// @brief return the actual current fill size.
      size_t size()
      {
          return m_storage.size();
      }

      ///@brief return the capacity of the integrator
      size_t capacity()
      {
          return m_N;
      }
      ///@brief integrate the value
      ///@param[in] value : to-be-integrated value
      ///@param[in] min : limit the integration to this lower limit
      ///@param[in] min : limit the integration to this upper limit
      double integrate(double value,double min=kMinDBL,double max=kMinDBL)
      {
        //moving window integrator
        if(m_storage.size()<m_N)
        {
            m_storage.push_back(value);
        }
        else
        {
            m_storage.pop_front();
            m_storage.push_back(value);
        }
        return std::accumulate(m_storage.begin(),m_storage.end(),m_offset,
        [this](double acc,double newval)
        {return (acc+(newval*m_dT));});
    }
    private:
      std::deque<double> m_storage; 
      double m_offset;
      double m_dT;
      size_t m_N;
};
#endif