#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 滑模控制器集合，提供 ESMC、EISMC、TSMC、STSMC 和 VGESMC
constructor_args: []
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include "EISMC.hpp"
#include "ESMC.hpp"
#include "STSMC.hpp"
#include "TSMC.hpp"
#include "VGESMC.hpp"
#include "app_framework.hpp"
#include "libxr_def.hpp"

/**
 * @brief xrobot 模块入口，汇总五种滑模控制器
 *
 * 控制器由调用方分别构造，入口不持有控制器实例或控制状态。
 */
class SMC : public LibXR::Application {
 public:
  SMC(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app) {
    UNUSED(hw);
    UNUSED(app);
  }

  void OnMonitor() override {}
};
