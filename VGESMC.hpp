#pragma once

#include <algorithm>
#include <cmath>

#include "app_framework.hpp"
#include "cycle_value.hpp"
#include "libxr_def.hpp"

/**
 * @brief Yaw 轴滑模控制器（指数趋近律）
 *
 * 控制律（由 2025 版 EXPONENT 拆解而来，恢复 θ̈_t 前馈）：
 *   s  = c·e + ė            // 滑模面
 *   u  = J·(θ̈_t − c·ė − K·s − η·(|e|+|ė|)·sat(s/φ))
 *   e  = pos − target       // CycleValue 减法，自动绕到 [-π, π)
 *   ė  = vol − target_vol   // 角速度误差，vol 取 AHRS 陀螺，不差分
 *
 * 新型趋近律（变增益：切换增益随状态范数 (|e|+|ė|) 自适应，误差→0
 * 时抖振自动减小）： ṡ  = −K·s − η·(|e|+|ė|)·sat(s/φ)
 *
 * 参数对应关系（原 EXPONENT 一物两用的 ε 已拆开）：
 *   K    = 原 K（指数趋近律增益，s⁻¹）
 *   eta  = 切换增益系数（有效切换增益 = η·(|e|+|ė|)，随状态收敛自动减小）
 *   phi  = 原 ε 的 “边界层宽度”职责（滑模面 s 量纲）
 *   θ̈_t  = target_ddot 前馈（原代码里被注释掉的 tar_differential_second）
 *
 * 设计：这个类就是一个纯函数
 *   - 目标角速度 target_vol 由调用方算好传入（遥控 = yaw×MAX_SPEED×因子，
 *     AI = yaw_dot），因此必传、无默认值，防止误用成 0。
 *   - 目标角加速度 target_ddot 默认 0（遥控阶跃本就不该前馈，AI 才传
 *     yaw_ddot）。预留口：若将来要在普通模式加回 θ̈_t，由调用方在
 *     Gimbal 侧用 dt_ 差分 target_vol 后传入此参数，本类无需改动。
 *
 *     一.目前只有EXPONENT趋近率
 *     二.关于target_vol
 *      1.在遥控模式下，这个值一般会有阶跃的情况，需要根据实际情况决定是否使用;
 *      2.在AI模式下，这个值一般是连续的，可以使用，如果目标角度是三角函数，应该可以直接求导
 */

class VGESMC : public LibXR::Application {
 public:
  typedef struct {
    float J; /*转动惯量 kg·m²，复用 Gimbal 的 j_yaw_*/

    float K; /*指数趋近律增益 (s⁻¹)*/

    float c; /*滑模面斜率 (s⁻¹)*/

    float eta; /*切换增益系数：有效切换增益 = η·(|e|+|ė|)（状态范数）*/

    float phi; /*Sat 边界层宽度（滑模面 s 量纲）*/

    float u_max; /*输出力矩限幅*/

    float pos_eps; /*位置死区 (rad)*/

  } Param;

  VGESMC(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,

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
   * @return 输出力矩（已限幅 ±u_max）
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

    const float u = param_.J *

                    (target_ddot - param_.c * e_dot - param_.K * s -

                     param_.eta * (std::fabs(e) + std::fabs(e_dot)) * Sat(s));

    return std::clamp(u, -param_.u_max, param_.u_max);
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
