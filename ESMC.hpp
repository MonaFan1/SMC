#pragma once

#include <algorithm>
#include <cmath>

#include "app_framework.hpp"
#include "cycle_value.hpp"
#include "libxr_def.hpp"

/**
 * @brief Yaw 轴滑模控制器（指数趋近律）
 */

class ESMC : public LibXR::Application {
 public:
  typedef struct {
    float J; /*转动惯量 kg·m²，复用 Gimbal 的 j_yaw_*/

    float K; /*指数趋近律增益 (s⁻¹)*/

    float c; /*滑模面斜率 (s⁻¹)*/

    float eta; /*切换增益 (rad/s²)*/

    float phi; /*Sat 边界层宽度（滑模面 s 量纲）*/

    float pos_eps; /*位置死区 (rad)*/

  } Param;

  ESMC(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,

       const Param& param)

      : param_(param) {
    UNUSED(hw);

    UNUSED(app);
  }

  /**
   * @brief 一步滑模计算（无状态，纯函数）
   * @param target 目标角度（CycleValue，自动处理 ±π 环绕）
   * @param pos 当前角度（AHRS 欧拉角，float 隐式构造 CycleValue）
   * @param vol 当前角速度（AHRS 陀螺）
   * @param target_vol 目标角速度（必传：遥控 = yaw×MAX_SPEED×因子，AI =
   * yaw_dot）
   * @param target_ddot 目标角加速度前馈 θ̈_t（默认 0：遥控阶跃不加；AI 传
   * yaw_ddot）
   * @return 输出力矩（不限幅，达妙电机侧 clamp 到 ±T_MAX）
   */

  float Calculate(const LibXR::CycleValue<float>& target,

                  const LibXR::CycleValue<float>& pos, float vol,

                  float target_vol, float target_ddot = 0.0f) {
    const float e = pos - target;  // [-π, π)，CycleValue 减法自动环绕

    // 死区：位置误差小于阈值直接输出 0

    // TODO 边界处力矩不连续（死区内 u 突跳为 0），若实测边界振荡再改平滑

    if (std::fabs(e) < param_.pos_eps) {
      return 0.0f;
    }

    const float e_dot = vol - target_vol;

    const float s = param_.c * e + e_dot;

    const float u = param_.J * (target_ddot - param_.c * e_dot - param_.K * s -

                                param_.eta * Sat(s));

    return u;
  }

  void OnMonitor() override {}

 private:
  /**
   * @brief 饱和函数 sat(s/φ)：边界层内线性、外饱和到 ±1
   */

  float Sat(float s) const {
    if (param_.phi <= 1e-9f) {
      return (s > 0.0f) ? 1.0f : ((s < 0.0f) ? -1.0f : 0.0f);
    }

    return std::clamp(s / param_.phi, -1.0f, 1.0f);
  }

  Param param_;
};
