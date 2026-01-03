/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef KALMAN_HPP_
#define KALMAN_HPP_

#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <cmath>
#include <limits>


namespace kalman
{
  /**
   * @brief 一维卡尔曼滤波器
   * @note 纯算法实现，无ROS依赖
   */
  class OneKalmanClass
  {
  public:
    /**
     * @brief 构造函数
     * @param vR 观测噪声协方差
     * @param vQ 过程噪声协方差
     */
    OneKalmanClass(float vR = 0.01f, float vQ = 5.0f) 
      : R(vR), Q(vQ) 
    {
      Reset(); // 初始化状态
    }
    
    ~OneKalmanClass() = default;

    /**
     * @brief 卡尔曼滤波更新
     * @param Data 新的观测数据
     * @return 滤波后的数据
     */
    float Update(const float Data)
    {
      // 预测阶段
      mid = last;
      p_mid = p_last + Q;

      // 更新阶段
      kg = p_mid / (p_mid + R);
      now = mid + kg * (Data - mid);
      p_now = (1 - kg) * p_mid;

      // 状态更新
      p_last = p_now;
      last = now;

      return now;
    }

    /**
     * @brief 重置滤波器状态
     */
    void Reset(void)
    {
      last = 0.0f;
      mid = 0.0f;
      now = 0.0f;
      p_last = 0.0f;
      p_mid = 0.0f;
      p_now = 0.0f;
      kg = 0.0f;
    }

    /**
     * @brief 设置滤波器参数
     * @param vR 观测噪声协方差
     * @param vQ 过程噪声协方差
     */
    void SetParams(float vR, float vQ)
    {
      R = vR;
      Q = vQ;
      Reset();
    }

    /**
     * @brief 获取当前滤波结果
     */
    float GetCurrentValue() const { return now; }

  private:
    float R{0.01f};    // 观测噪声协方差
    float Q{5.0f};     // 过程噪声协方差
    float last{0.0f};  // 上一时刻估计值
    float mid{0.0f};   // 中间估计值
    float now{0.0f};   // 当前估计值
    float p_last{0.0f};// 上一时刻协方差
    float p_mid{0.0f}; // 中间协方差
    float p_now{0.0f}; // 当前协方差
    float kg{0.0f};    // 卡尔曼增益
  };
}

namespace pid
{
  /**
   * @brief 位置式PID控制器
   * @note 纯算法实现，无ROS依赖
   */
  class PidClass
  {
  public:
    PidClass() = default;
    ~PidClass() = default;

    /**
     * @brief PID计算
     * @param err 当前误差
     * @return PID输出
     */
    float PidCal(const float err)
    {
      if (dt <= 0.0f) {
        // RCLCPP_WARN(rclcpp::get_logger("PidClass"), "PID dt未设置，使用默认值0.01s");
        dt = 0.01f; // 默认采样时间
      }

      // 误差更新
      error = err;
      
      // 积分项（带积分限幅）
      integ += error * dt;
      integ = std::clamp(integ, iLimitMix, iLimitMax);

      // 微分项
      deriv = (error - prevError) / dt;

      // PID输出计算
      outP = kp * error;
      outI = ki * integ;
      outD = kd * deriv;

      float output = outP + outI + outD;
      
      // 输出限幅
      output = std::clamp(output, outLimitMix, outLimitMax);

      // 保存当前误差作为下一时刻前项误差
      prevError = error;

      return output;
    }

    /**
     * @brief PID初始化
     * @param p 比例系数
     * @param i 积分系数
     * @param d 微分系数
     * @param oMin 输出最小值
     * @param oMax 输出最大值
     * @param dt_ 采样时间(s)
     * @param iMin 积分最小值（默认与输出限幅一致）
     * @param iMax 积分最大值（默认与输出限幅一致）
     */
    void PidInit(float p, float i, float d, 
                 float oMin, float oMax,
                 float dt_ = 0.01f,
                 float iMin = std::numeric_limits<float>::lowest(),
                 float iMax = std::numeric_limits<float>::max())
    {
      kp = p;
      ki = i;
      kd = d;
      outLimitMix = oMin;
      outLimitMax = oMax;
      dt = dt_;

      // 积分限幅（未指定时使用输出限幅）
      iLimitMix = (iMin == std::numeric_limits<float>::lowest()) ? oMin : iMin;
      iLimitMax = (iMax == std::numeric_limits<float>::max()) ? oMax : iMax;

      PidReset();
    }

    /**
     * @brief 重置PID状态
     */
    void PidReset(void)
    {
      error = 0.0f;
      prevError = 0.0f;
      integ = 0.0f;
      deriv = 0.0f;
      outP = 0.0f;
      outI = 0.0f;
      outD = 0.0f;
    }

    /**
     * @brief 设置采样时间
     * @param dt_ 采样时间(s)
     */
    void SetDt(float dt_) { dt = dt_; }

    /**
     * @brief 获取PID参数
     */
    void GetParams(float &p, float &i, float &d) const 
    { 
      p = kp; i = ki; d = kd; 
    }

    /**
     * @brief 获取各分项输出
     */
    void GetComponents(float &p, float &i, float &d) const
    {
      p = outP; i = outI; d = outD;
    }

  private:
    // PID核心参数
    float kp{0.0f};    // 比例系数
    float ki{0.0f};    // 积分系数
    float kd{0.0f};    // 微分系数

    // 计算中间变量
    float error{0.0f};     // 当前误差
    float prevError{0.0f}; // 上一时刻误差
    float integ{0.0f};     // 积分项
    float deriv{0.0f};     // 微分项
    float outP{0.0f};      // 比例输出
    float outI{0.0f};      // 积分输出
    float outD{0.0f};      // 微分输出

    // 限幅参数
    float iLimitMax{0.0f}; // 积分上限
    float iLimitMix{0.0f}; // 积分下限
    float outLimitMax{0.0f};// 输出上限
    float outLimitMix{0.0f};// 输出下限
    float dt{0.0f};        // 采样时间(s)
  };

}

#endif // KALMAN_HPP_