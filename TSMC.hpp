#pragma once

#include <algorithm>
#include <cmath>

#include "app_framework.hpp"
#include "cycle_value.hpp"
#include "libxr_def.hpp"

/**
 * @brief Yaw 轴滑模控制器（指数趋近律 + 终端滑模面，仿 2026 复旦星云 EGA）
 *
 * 控制律（|e| 大用终端面，|e| 小用线性面规避 1/e 奇点）：
 *
 *   终端滑模面（|e| ≥ e_lin）：
 *     s  = ė + c·sign(e)·|e|^α        α = q/p ∈ (0,1)
 *     u  = J·(θ̈_t − c·α·|e|^(α−1)·ė − K·s − η·sat(s/φ))
 *
 *   线性滑模面（|e| < e_lin，即原 EXPONENT）：
 *     s  = c·e + ė
 *     u  = J·(θ̈_t − c·ė − K·s − η·sat(s/φ))
 *
 *   e   = pos − target       // CycleValue 减法，自动绕到 [-π, π)
 *   ė   = vol − target_vol   // 角速度误差，vol 取 AHRS 陀螺，不差分
 *
 * 参数对应关系：
 *   K      = 指数趋近律增益 (s⁻¹)
 *   eta    = 切换增益 (rad/s²)（原 ε 的切换职责）
 *   phi    = 边界层宽度（原 ε 的边界层职责）
 *   q/p    = 终端面指数 α，须 0 < q/p < 1；q/p/e_lin 留 0 时退化为纯线性
 * EXPONENT e_lin  = 线性切换阈值 (rad)，|e|<e_lin 用线性面（复旦原版硬编码
 * 1°≈0.0175） θ̈_t    = target_ddot 前馈，由调用方算好传入（AI 传
 * yaw_ddot；遥控默认 0）
 *
 * 与 2026 复旦 slidingmodec.cpp 的差异（都是修正，不是照抄）：
 *   1. 等价控制项用正确的 |e|^(α−1)；复旦在 e<0 时多乘了个
 * sign(e)，方向是反的；
 *   2. target_dot/ddot 不在类内差分（复旦没除 dt 单位错），改为调用方传入；
 *   3. ε 一物两用拆成 eta/phi；Sat 带边界层 φ；
 *   4. 死区/切换阈值用 rad（复旦用 °），e_lin 可配。
 *
 * 注意：
 *   - 终端面 s 量纲混了 rad/s 与 rad^α/s，K/phi 在两段里的等效带宽不同，
 *     这是复旦原设计的固有属性，调参时两段要分别照顾；
 *   - |e|=e_lin 处是硬切换，u 有一次小跳变；e_lin 取小时跳变可忽略，
 *     若实测在切换点抖，再改成平滑过渡。
 */
class TSMC : public LibXR::Application {
 public:
  typedef struct {
    float J;       /*转动惯量 kg·m²，复用 Gimbal 的 j_yaw_*/
    float K;       /*指数趋近律增益 (s⁻¹)*/
    float c;       /*滑模面斜率 (s⁻¹)*/
    float eta;     /*切换增益 (rad/s²)*/
    float phi;     /*Sat 边界层宽度（滑模面 s 量纲）*/
    float u_max;   /*输出力矩限幅*/
    float pos_eps; /*位置死区 (rad)*/
    float q;       /*终端面指数分子，0 表示不用终端面*/
    float p;       /*终端面指数分母，须 q/p ∈ (0,1)*/
    float e_lin;   /*线性切换阈值 (rad)*/
  } Param;

  TSMC(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
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
    const float e_dot = vol - target_vol;

    // 死区：位置误差小于阈值直接输出 0
    // TODO 边界处力矩不连续（死区内 u 突跳为 0），若实测边界振荡再改平滑
    if (std::fabs(e) < param_.pos_eps) {
      return 0.0f;
    }

    // 终端面有效性：0 < q/p < 1 且切换阈值在死区之外，否则退化为线性 EXPONENT
    const bool terminal_ok = (param_.q > 1e-6f) && (param_.p > 1e-6f) &&
                             (param_.q < param_.p) &&
                             (param_.e_lin > param_.pos_eps);
    const float alpha = terminal_ok ? (param_.q / param_.p) : 1.0f;

    float s;       /*滑模面*/
    float eq_ctrl; /*等价控制里的 −c·g'(e)·ė 项*/

    if (terminal_ok && std::fabs(e) >= param_.e_lin) {
      // 终端滑模面：s = ė + c·sign(e)·|e|^α
      const float sig = std::copysign(std::pow(std::fabs(e), alpha), e);
      s = e_dot + param_.c * sig;
      // 等价控制：−c·α·|e|^(α−1)·ė（|e|≥e_lin>0，无奇点）
      eq_ctrl =
          -param_.c * alpha * std::pow(std::fabs(e), alpha - 1.0f) * e_dot;
    } else {
      // 线性滑模面：s = c·e + ė（原 EXPONENT）
      s = param_.c * e + e_dot;
      eq_ctrl = -param_.c * e_dot;
    }

    const float u =
        param_.J * (target_ddot + eq_ctrl - param_.K * s - param_.eta * Sat(s));

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
