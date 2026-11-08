#pragma once

#include <algorithm>
#include <cmath>

#include "app_framework.hpp"
#include "cycle_value.hpp"
#include "libxr_def.hpp"

/**
 * @brief Yaw 轴超螺旋滑模控制器（Super-Twisting Algorithm, STA）
 *
 * 控制律（二阶滑模 = 等价控制 + 超螺旋趋近）：
 *   s  = c·e + ė                          // 滑模面
 *   v  = ∫ α·sat(s/φ) dt                  // 超螺旋积分项（抗扰 + 稳态消差）
 *   u  = J·(θ̈_t − c·ė − λ·√|s|·sign(s) − v)
 *   e  = pos − target                     // CycleValue 减法，自动绕到 [-π, π)
 *   ė  = vol − target_vol                 // 角速度误差，vol 取 AHRS
 * 陀螺，不差分
 *
 * 与 2025 EXPONENT（u=J·(θ̈_t−c·ė−K·s−η·sat(s/φ))）的区别：
 *   - K·s（线性趋近）  →  λ·√|s|·sign(s)：|s| 大时更缓、|s| 小时更强（有限时间
 *     收敛），且 √|s|·sign(s) 在 s=0 连续，天然抑制抖振；
 *   - η·sat(s)（开关项，直接进 u）→  α·∫sat(s)dt（积分项）：sign 被积分平滑，
 *     输出连续，抗常值/缓变扰动时稳态误差趋 0。
 *   速度反馈系数由 J·(c+K) 降到 J·c，陀螺噪声放大更小。
 *
 * 参数：
 *   c      = 滑模面斜率 (s⁻¹)，决定上滑面后的收敛带宽
 *   lambda = √|s| 趋近增益（经验值，量纲
 * (rad/s)^{3/2}）；越大收敛越快、过大抖振 alpha  =
 * 积分增益（rad/s³）；抗扰/消稳态误差，太小稳态误差大、太大超调 phi    = Sat
 * 边界层宽度（滑模面 s 量纲），边界层内线性平滑；输出饱和时冻结积分 θ̈_t    =
 * target_ddot 前馈（遥控阶跃传 0，AI 传 yaw_ddot）
 *
 * 设计：本类已从"无状态纯函数"变为含积分状态的二阶控制器，需调用方在
 *   模式切换/失能时调 Reset() 清积分。target_vol 由调用方算好传入（遥控 =
 *   yaw×MAX_SPEED×因子，AI = yaw_dot），必传无默认值。
 */
class STSMC : public LibXR::Application {
 public:
  typedef struct {
    float J;       /*转动惯量 kg·m²，复用 Gimbal 的 j_yaw_*/
    float c;       /*滑模面斜率 (s⁻¹)*/
    float lambda;  /*超螺旋 √|s| 趋近增益 ((rad/s)^{3/2})*/
    float alpha;   /*超螺旋积分增益 (rad/s³)*/
    float phi;     /*Sat 边界层宽度（滑模面 s 量纲）*/
    float u_max;   /*输出力矩限幅*/
    float pos_eps; /*位置死区 (rad)*/
  } Param;

  STSMC(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
        const Param& param)
      : param_(param) {
    UNUSED(hw);
    UNUSED(app);
  }

  /** @brief 清超螺旋积分状态（模式切换/失能时调用） */
  void Reset() { v_ = 0.0f; }

  /**
   * @brief 一步超螺旋滑模计算
   * @param target 目标角度（CycleValue，自动处理 ±π 环绕）
   * @param pos 当前角度（AHRS 欧拉角，float 隐式构造 CycleValue）
   * @param vol 当前角速度（AHRS 陀螺）
   * @param target_vol 目标角速度（必传：遥控 = yaw×MAX_SPEED×因子，AI =
   * yaw_dot）
   * @param target_ddot 目标角加速度前馈 θ̈_t（遥控阶跃传 0，AI 传 yaw_ddot）
   * @param dt 控制周期 (s)，用于积分项
   * @return 输出力矩（已限幅 ±u_max）
   */
  float Calculate(const LibXR::CycleValue<float>& target,
                  const LibXR::CycleValue<float>& pos, float vol,
                  float target_vol, float target_ddot, float dt) {
    const float e = pos - target;  // [-π, π)，CycleValue 减法自动环绕

    // 死区：位置误差小于阈值直接输出 0，并清积分防止出死区时残量跳变
    if (std::fabs(e) < param_.pos_eps) {
      v_ = 0.0f;
      return 0.0f;
    }

    const float e_dot = vol - target_vol;
    const float s = param_.c * e + e_dot;

    // 超螺旋：u = J·(θ̈_t − c·ė − λ·√|s|·sign(s) − v)
    const float root_s = std::sqrt(std::fabs(s));
    const float sign_s = (s > 0.0f) ? 1.0f : ((s < 0.0f) ? -1.0f : 0.0f);
    const float u = param_.J * (target_ddot - param_.c * e_dot -
                                param_.lambda * root_s * sign_s - v_);

    // 抗饱和：仅当输出未限幅时推进积分（v̇ = α·sat(s/φ)），饱和则冻结
    if (std::fabs(u) < param_.u_max) {
      v_ += param_.alpha * Sat(s) * dt;
    }

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
  float v_ = 0.0f;  // 超螺旋积分项 (rad/s²)
};
