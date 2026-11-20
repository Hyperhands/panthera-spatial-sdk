# panthera-spatial-sdk

Panthera-HT 双臂 + A2/L1 灵巧手的上位机空间控制 SDK。设计目标：
- 500 Hz 的 acados QP（HPIPM）任务空间控制，带双臂自碰撞约束；
- C++ 核心 + pybind11 Python 接口；
- 固件、wirelink、WLC 及外部依赖全部以源码固化在本仓库。

> C++ 双臂控制器已通过 UDP 回环运动验收。左右板 host-control 镜像已构建，实机验收列在 WP-H。见[实施计划](docs/PLAN.md)。

| 目录 | 内容 | 状态 |
| --- | --- | --- |
| [docs/PLAN.md](docs/PLAN.md) | 总体计划、架构、阶段与待确认问题 | ✅ |
| [model/panthera_dual_description](model/panthera_dual_description/README.md) | 整机 URDF/SRDF、网格、装配参数、校验 | ✅ |
| [tools/model](tools/model) | 模型生成（`build_description.py`）与校验（`verify_description.py`） | ✅ |
| [third_party/](third_party/VERSIONS.md)、[protocol/](protocol)、[generated/protocol/](generated/protocol) | 固定依赖、板端协议和编解码器 | 已接入主机构建 |
| [src/](src)、[examples/](examples) | 双板遥测、管理 RPC、租约、姿态保持与手部透传 | 已通过本机回环验收 |
| [sim/](sim) | UDP 回环双板遥测与接管模拟器 | 已接入会话回归测试 |
| [generated/model/](generated/model)、[tools/codegen/](tools/codegen) | 双臂 FK、雅可比、重力、碰撞几何及逐对阈值 | 已生成并校验 |
| [controller.hpp](include/panthera/controller.hpp) | 双臂任务空间 QP、关节目标、手部透传与专家模式 | 已通过闭环仿真 |
| [firmware/](firmware/README.md) | 固定源版本、左右板配置、构建与烧录工具及预编译镜像 | 已通过原生测试和重复构建 |
| `python/` | Python 接口与 wheel | WP-F 待实现 |

## 构建主机代码

需要 C 编译器、支持 C++20 的 C++ 编译器、Python ≥ 3.9 及开发头文件、Git、CMake 和 Ninja。首次检出后运行：

```bash
git submodule update --init --recursive
cmake --workflow --preset host
```

该预设使用 CMake ≥ 3.25，依次配置、构建并运行测试。CMake 3.22–3.24 可使用：

```bash
cmake -S . -B build/host -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/host
ctest --test-dir build/host --output-on-failure
```

## 读取两板状态

主机网卡配置为 `192.168.10.1` 后，运行 `build/host/panthera-read-state`（Windows 加 `.exe`）。示例绑定 UDP 5005、5015，收到两板新鲜的六轴反馈后输出原始编码器角度（rad）并退出。该入口只订阅遥测，不申请控制租约。板端启动回零由固件配置决定。

本机联调时，在一个终端运行 `python3 sim/telemetry.py`，另一个终端运行 `build/host/panthera-read-state --loopback`。模拟器以 2 ms 目标周期发布两臂的静止零位，另外发布手、故障和诊断遥测。

C++ 接口见 [telemetry.hpp](include/panthera/telemetry.hpp)。每个接收器由一个线程调用 `poll()`；快照包含臂、手、故障和诊断各自的最近一帧及序号缺口计数。`active_hand_angles(now, max_age)` 按手指令的六个主动关节顺序返回固件弧度值，数据过期、离线或无效时返回空值。原始 11 项反馈仍可从手部样本读取。重启事件由 `restarts` 与新的 `boot_id` 标识。

## 双板控制会话

`DualSession::enable(timeout)` 等待两板回零、申请租约、配置策略，并用当前姿态预发保持帧。两板都确认主机控制后才返回。调用方提供六轴重力矩函数，输入是扣除整圈编码器偏移后的关节弧度，输出单位为 N·m。会话接口见 [session.hpp](include/panthera/session.hpp)。

同一线程以 2 ms 为目标周期调用 `hold()`。任一侧反馈年龄达到 10 ms 时返回 `HoldStatus::frozen`，两臂均保持原参考。会话有效且反馈新鲜时，`send_hand()` 可透传六个主动手关节目标与持续时间。显式调用 `stop()` 会等待板端软停结果。反馈年龄达到 50 ms、板端故障或单板重启会中止会话并抛出异常。重启侧由固件自动回零，在线侧收到软停请求，恢复需要再次调用 `enable()`。

运行 `ctest --test-dir build/host -R sim_session -V` 可复现接管、回滚与故障注入。测试只绑定本机回环端口，使用指定测试力矩核对帧内容。板端模拟器覆盖通信与状态切换，关节运动仿真见下节。实际回零轨迹在 WP-H 验证。

## C++ 空间控制

链接 CMake 目标 `panthera_controller`。构造 `panthera::DualArm` 时可传入两板地址和 `ControlConfig`。配置包含关节速度、加速度、参考超前量、任务权重、位姿增益和插值速率。默认 J5 软限位为 ±60°。

`enable()` 完成双板接管并启动主机 2 ms 线程。调用方可从其他线程调用 `setPoseTarget`、`setTwistTarget`、`setJointTarget` 和 `setHandTarget`。位姿使用世界坐标系，TCP 速度按线速度 m/s 和角速度 rad/s 排列。臂关节角度使用扣除整圈编码器偏移后的弧度。用 `state()` 读取测量与参考关节角、两臂 TCP、碰撞间隙、QP 用时、冻结状态和故障。用 `softStop()` 停止控制并等待板端确认。

**专家模式会绕过主机运动监控、QP、碰撞检查和关节限幅。** 显式调用 `enterExpertMode(ExpertAck::IUnderstandNoSafety)` 后，正常控制线程停止。`sendMit()` 按原始编码器坐标发送单侧的 30 个 MIT 浮点数，调用方负责两侧指令的发送节奏。板端继续执行既有租约校验和看门狗。

```bash
ctest --test-dir build/host -R 'geometry|motion_qp|control_law|sim_controller' -V
```

`sim_controller` 自动启动两个本机 UDP 模拟板和 C++ 控制器，验证直线与阶跃跟踪、运动中的单向断链、重启、跟踪滞后和专家指令。模拟板使用简化的重力平衡关节伺服模型。数值验收记录见[实机前验证](docs/pre-hardware.md)。

## 模型

```bash
# 重新生成并校验模型（需要 numpy、PyYAML、pinocchio>=3.4、matplotlib）
python3 tools/model/build_description.py
python3 tools/model/verify_description.py
```
