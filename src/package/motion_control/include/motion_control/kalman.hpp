
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef KALMAN_HPP_
#define KALMAN_HPP_

#include <iostream>
#include <string>
#include <pthread.h>
#include <thread>
#include <chrono>
#include <boost/thread/pthread/shared_mutex.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <ros/ros.h>

namespace kalman
{
  class OneKalmanClass
  {
  public:
    OneKalmanClass(float vR = 0.01, float vQ = 5) : R(vR), Q(vQ) {}
    ~OneKalmanClass() {}
    float Update(const float Data)
    {
      mid = last;
      p_mid = p_last + Q;
      kg = p_mid / (p_mid + R);
      now = mid + kg * (Data - mid);
      p_now = (1 - kg) * p_mid;
      p_last = p_now;
      last = now;

      return now;
    }

    void Reset(void)
    {
      last = 0;
      mid = 0;
      now = 0;
      p_last = 0;
      p_mid = 0;
      p_now = 0;
      kg = 0;
    }

  private:
    float R;
    float Q;
    float last = 0;
    float mid = 0;
    float now = 0;
    float p_last = 0;
    float p_mid = 0;
    float p_now = 0;
    float kg = 0;
  };
}

namespace pid
{

  class PidClass
  {
  public:
    PidClass() {}
    ~PidClass() {}
    float PidCal(const float err)
    {
      float output;

      error = err;
      integ += error * dt;
      if (integ > iLimitMax)
        integ = iLimitMax;
      else if (integ < iLimitMix)
        integ = iLimitMix;

      deriv = error - prevError;
      outP = kp * error;
      outI = ki * integ;
      outD = kd * deriv;

      output = outP + outI + outD;
      if (output > outLimitMax)
        output = outLimitMax;
      else if (output < outLimitMix)
        output = outLimitMix;

      prevError = error;
      return output;
    }

    void PidInit(float p, float i, float d, float oMin, float oMax)
    {
      kp = p;
      ki = i;
      kd = d;
      outLimitMix = oMin;
      outLimitMax = oMax;
    }

    void PidReset(void)
    {
      error = 0;
      prevError = 0;
      integ = 0;
      deriv = 0;
      outP = 0;
      outI = 0;
      outD = 0;
    }

  private:
    float kp = 0;
    float ki = 0;
    float kd = 0;

    float error = 0;
    float prevError = 0;
    float integ = 0;
    float deriv = 0;
    float outP = 0;
    float outI = 0;
    float outD = 0;
    float iLimitMax = 0;
    float iLimitMix = 0;
    float outLimitMax = 0;
    float outLimitMix = 0;
    float dt = 0;
  };

}
#endif
