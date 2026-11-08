#pragma once

#include <algorithm>
#include <cmath>

#include "app_framework.hpp"
#include "cycle_value.hpp"
#include "libxr_def.hpp"

/**
 * @brief Pitch 轴积分滑模控制器（指数趋近律）
 *
 * 控制律：
 *   s = c1 * e + e_dot + c2 * integral(e)
 *   u = J * (target_ddot - c1 * e_dot - c2 * e - K * s
 *            - eta * sat(s / phi))
 *   e = pos - target，e_dot = vol - target_vol
 *
 * 调用方传入目标角度、目标角速度、目标角加速度，
 * 以及当前角度、当前角速度和控制周期 dt。
 * 在 Gimbal 中，角度反馈取 AHRS 欧拉角，角速度反馈取 IMU 陀螺。
 * 本控制器包含误差积分状态，失能或重新对齐目标时需要调用 Reset()。
 * 控制律已包含 J * target_ddot 前馈，重力补偿由 Gimbal 叠加。
 */

class EISMC : public LibXR::Application {
 public:
  typedef struct {
    float J; /*转动惯量 kg·m²*/

    float K; /*指数趋近律增益 (s⁻¹)*/

    float c1; /*滑模面位置误差系数 (s⁻¹)*/

    float c2; /*滑模面积分误差系数 (s⁻²)*/

    float eta; /*切换增益 (rad/s²)*/

    float phi; /*Sat 边界层宽度*/

    float pos_eps; /*死区阈值，同时检查位置和角速度误差*/

    float integral_limit; /*误差积分限幅 (rad·s)，<= 0 时不限幅*/

    float output_limit; /*输出力矩限幅 (N·m)，<= 0 时不限幅*/

  } Param;

  EISMC(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,

        const Param& param)

      : param_(param) {
    UNUSED(hw);

    UNUSED(app);
  }

  /**
   * @brief 一步积分滑模计算（非循环角度，用于限位后的 Pitch）
   * @param target 目标角度 (rad)
   * @param pos 当前角度 (rad)
   * @param vol 当前角速度 (rad/s)
   * @param target_vol 目标角速度 (rad/s)
   * @param target_ddot 目标角加速度 (rad/s²)
   * @param dt 控制周期 (s)，无效时使用 0.002 s
   * @return 限幅后的控制力矩，不包含重力补偿
   */

  float Calculate(float target, float pos, float vol, float target_vol,

                  float target_ddot, float dt) {
    return CalculateError(pos - target, vol, target_vol, target_ddot, dt);
  }

  /**
   * @brief 循环角度计算，位置误差自动环绕到 [-π, π)
   */

  float Calculate(const LibXR::CycleValue<float>& target,

                  const LibXR::CycleValue<float>& pos, float vol,

                  float target_vol, float target_ddot, float dt) {
    return CalculateError(static_cast<float>(pos - target), vol, target_vol,

                          target_ddot, dt);
  }

  /**
   * @brief 清零积分及调试观测状态
   */

  void Reset() {
    integral_error_ = 0.0f;

    sliding_surface_ = 0.0f;

    last_error_ = 0.0f;

    last_output_ = 0.0f;
  }

  float SlidingSurface() const { return sliding_surface_; }

  float IntegralError() const { return integral_error_; }

  float LastError() const { return last_error_; }

  float LastOutput() const { return last_output_; }

  void OnMonitor() override {}

 private:
  float CalculateError(float e, float vol, float target_vol, float target_ddot,

                       float dt) {
    if (!std::isfinite(e) || !std::isfinite(vol) ||

        !std::isfinite(target_vol) || !std::isfinite(target_ddot)) {
      Reset();

      return 0.0f;
    }

    const float SAMPLE_TIME =

        (std::isfinite(dt) && dt > 1.0e-6f && dt < 0.1f) ? dt : 0.002f;

    const float E_DOT = vol - target_vol;

    integral_error_ += e * SAMPLE_TIME;

    if (param_.integral_limit > 0.0f) {
      integral_error_ = std::clamp(integral_error_, -param_.integral_limit,

                                   param_.integral_limit);
    }

    sliding_surface_ = param_.c1 * e + E_DOT + param_.c2 * integral_error_;

    float output = param_.J * (target_ddot - param_.c1 * E_DOT - param_.c2 * e -

                               param_.K * sliding_surface_ -

                               param_.eta * Sat(sliding_surface_));

    // 死区内保留积分以抑制扰动，不再叠加 -c1 * e_dot 项。

    if (std::fabs(e) < param_.pos_eps && std::fabs(E_DOT) < param_.pos_eps) {
      output = param_.J *

               (target_ddot - param_.c2 * e - param_.K * sliding_surface_ -

                param_.eta * Sat(sliding_surface_));
    }

    if (param_.output_limit > 0.0f) {
      output = std::clamp(output, -param_.output_limit, param_.output_limit);
    }

    last_error_ = e;

    last_output_ = output;

    return output;
  }

  /**
   * @brief 饱和函数 sat(s/φ)：边界层内线性、外饱和到 ±1
   */

  float Sat(float s) const {
    if (param_.phi <= 1.0e-6f) {
      return (s > 0.0f) ? 1.0f : ((s < 0.0f) ? -1.0f : 0.0f);
    }

    return std::clamp(s / param_.phi, -1.0f, 1.0f);
  }

  Param param_;

  float integral_error_ = 0.0f;

  float sliding_surface_ = 0.0f;

  float last_error_ = 0.0f;

  float last_output_ = 0.0f;
};
