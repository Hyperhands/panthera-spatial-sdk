# panthera-spatial-sdk

Panthera-HT 双臂 + A2/L1 灵巧手的上位机空间控制 SDK。设计目标：
- 500 Hz 的 acados QP（HPIPM）任务空间控制，带双臂自碰撞约束；
- C++ 核心 + pybind11 Python 接口；
- 固件、wirelink、WLC 及外部依赖全部以源码固化在本仓库。

> 状态：已接入主机构建、双板只读遥测、协议编解码器和 HPIPM，双臂模型（M0）已完成。工作包与验收条件见 [docs/PLAN.md](docs/PLAN.md)。

| 目录 | 内容 | 状态 |
| --- | --- | --- |
| [docs/PLAN.md](docs/PLAN.md) | 总体计划、架构、阶段与待确认问题 | ✅ |
| [model/panthera_dual_description](model/panthera_dual_description/README.md) | 整机 URDF/SRDF、网格、装配参数、校验 | ✅ |
| [tools/model](tools/model) | 模型生成（`build_description.py`）与校验（`verify_description.py`） | ✅ |
| [third_party/](third_party/VERSIONS.md)、[protocol/](protocol)、[generated/protocol/](generated/protocol) | 固定依赖、板端协议和编解码器 | 已接入主机构建 |
| [src/](src)、[examples/](examples) | 双板只读遥测与读状态示例 | WP-B 进行中 |
| [sim/](sim) | UDP 回环双板遥测模拟器 | WP-B 进行中 |
| `firmware/`、`python/` | 固件、Python 接口 | 待实现 |

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

## 模型

```bash
# 重新生成并校验模型（需要 numpy、PyYAML、pinocchio>=3.4、matplotlib）
python3 tools/model/build_description.py
python3 tools/model/verify_description.py
```
