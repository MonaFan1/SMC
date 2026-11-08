# SMC

基于 LibXR 的滑模控制算法集合。各控制器是独立的头文件类，可同时包含。
本仓库收录各源工程现有控制律；类名和模块入口统一，参数、死区、限幅及积分逻辑沿用源实现。

| 头文件 / 类名 | 算法 | 来源工程 | 状态 |
| --- | --- | --- | --- |
| `ESMC.hpp` / `ESMC` | 线性滑模面 + 指数趋近律 | SMC1.0 | 无积分状态 |
| `EISMC.hpp` / `EISMC` | PI 滑模面 + 指数趋近律 | SMC1.0 | 误差积分 |
| `TSMC.hpp` / `TSMC` | 终端/线性滑模面分段切换 + 指数趋近律 | SMC2.0 | 无积分状态 |
| `STSMC.hpp` / `STSMC` | 超螺旋趋近律（Super-Twisting） | SMC3.0 - 副本 | 超螺旋积分 |
| `VGESMC.hpp` / `VGESMC` | 线性滑模面 + 状态相关变增益指数趋近律 | SMC4.0 | 无积分状态 |

缩写作为本仓库的类名约定，具体实现以对应头文件为准。来源及原文件校验值见 [SOURCES.md](SOURCES.md)。

## 集成

依赖 LibXR 的 `app_framework.hpp`、`cycle_value.hpp`、`libxr_def.hpp`，使用 C++17 或更新标准。
将仓库放在工程的 `Modules/SMC/`，由 LibXR 的模块 CMake 扫描加载。

```cpp
#include "SMC.hpp"  // 汇总五个控制器；也可单独包含对应头文件
```

控制器统一使用 `(hw, app, param)` 构造；`SMC.hpp` 中的同名入口类供 xrobot 自动发现和生成使用。
调用方分别创建所需的控制器实例，模块入口不持有控制状态。Gimbal 不是本仓库的一部分。

## 接口与参数

误差约定为 `e = pos - target`，角速度误差为 `vol - target_vol`。
`CycleValue` 接口会环绕位置误差。调用方传入目标角度、反馈角度、反馈角速度、目标角速度及目标角加速度。
EISMC 还提供非循环浮点角度接口，适用于限位后的 Pitch。

| 类名 | Param 字段（按声明顺序） | Calculate 是否需要 dt |
| --- | --- | --- |
| ESMC | J, K, c, eta, phi, pos_eps | 否 |
| EISMC | J, K, c1, c2, eta, phi, pos_eps, integral_limit, output_limit | 是 |
| TSMC | J, K, c, eta, phi, u_max, pos_eps, q, p, e_lin | 否 |
| STSMC | J, c, lambda, alpha, phi, u_max, pos_eps | 是 |
| VGESMC | J, K, c, eta, phi, u_max, pos_eps | 否 |

- EISMC、STSMC 在失能或重新对齐目标时调用 `Reset()`，清除内部积分状态。
- ESMC 本身不限制输出；其余控制器的限幅语义见对应头文件。
- TSMC、STSMC、VGESMC 的 `u_max` 是直接输出限幅，需要显式设置正值。
- 控制律已包含 `J * target_ddot` 前馈；重力、摩擦等额外补偿由调用方按模型叠加。

## 实现说明

TSMC 保留源实现的终端/线性面硬切换。STSMC 保留源实现的饱和边界层、输出饱和停积和位置死区清积分行为。
VGESMC 的切换增益按 `eta * (abs(e) + abs(e_dot))` 计算。
本次整理不改变这些实验实现，也不额外声称有限时间收敛、无抖振或跨机构参数可直接复用。
参数应根据目标机构和驱动接口选取。
