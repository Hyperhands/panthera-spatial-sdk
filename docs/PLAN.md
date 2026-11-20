# Panthera 双臂空间控制 SDK：执行计划（v4，2026-11-20，交接版）

本文是交给执行 agent 的唯一计划来源。§1 是已拍板的决策，不要再改；有疑问先查 §9 的资料位置，仍不确定就停下来问人。

## 1. 已确认决策

| # | 决策 |
| --- | --- |
| D1 | 只做 **acados QP（HPIPM）**，500 Hz 任务空间控制；**不做 NMPC**，也不做非线性 MPC |
| D2 | 交付**独立仓库** `panthera-spatial-sdk`；C++ 核心 + **pybind11** Python 接口；上位机不限平台 |
| D3 | **双臂自碰撞**进入 QP 约束；最小距离低于硬阈值时主机软停 |
| D4 | 模型：Panthera-HT 六轴 + A2/L1 灵巧手。两手都相对工具轴 **pitch 向上 45°**，两臂中心距 **480 mm**，平行安装。已完成，见 `model/panthera_dual_description` |
| D5 | 转接件实际存在，暂按**厚度 0、质量不计** |
| D6 | 交付固件：左臂 **node11** / `192.168.10.11` / UDP 5005-5006；右臂 **node12** / `.12` / 5015-5016；主机 node10 / `192.168.10.1` |
| D7 | 固件基线：**dual-gello 11.20 源码**，新增 **host-control 构建配置**；遥操构建保留，但不作为交付默认 |
| D8 | 依赖固化：Zephyr 侧固化 `west.yml` 精确提交和补丁；主机侧**能用 git submodule 固定提交的就用 submodule**，做不到才 vendor |
| D9 | acados 固定到**最新正式版 tag**（2026-11 查到 v0.6.0，执行时再核对一次） |
| D10 | 专家模式：**关节 MIT 完全直通，不做任何监控**（不跑 QP、不检查碰撞、不限幅）；默认关闭，文档里明确标注风险 |
| D11 | 灵巧手：SDK **透传 6 个主动关节的目标和状态**；碰撞模型里手部用**覆盖全部手指行程的保守包络**，与手指当前状态无关 |
| D12 | 重力：先用 URDF 名义惯量（含真实手部 0.617 kg）。旧的辨识参数是在“link6 集中 620 g”模型上拟合的，不能直接套用，以后重新辨识 |

## 2. 系统现状（执行者必须知道的事实）

### 2.1 链路与协议
- PC ↔ 每块 NXP 走 UDP + wirelink（CRC32C），schema revision **7**。协议源是 `protocol/robot_messages.wl` 和 `*.bind.wl`；两套固件里的协议与生成代码逐字节一致。
- **实时通道**（左 5005 / 右 5015）：
  - 主机 → 板：`ArmCommand`(257)，30 个 float，即 6 关节 × `[q, dq, kp, kd, tau_ff]`，加 `joint_mask`；`HandCommand`(258)；`HandComplianceCommand`(259)。不可靠投递，最新值覆盖。
  - 板 → 主机：`ArmState`(513)，默认周期 2000 µs；`HandState`(514)，10 ms；`FaultSnapshot`(515)，50 ms；`Diagnostics`(516)，100 ms。
- **管理通道**（左 5006 / 右 5016）：可靠 RPC，`Operation`(1025/1026)、`Configure`(1027/1028)，以及 `FaultEvent`(769) 队列。
- **主机端口绑定**：固件把遥测发到主机 IP 的**同号端口**。主机必须 bind 5005/5006/5015/5016 四个本地端口，可参考 `robot_sdk.py`。
- **会话流程**（参考实现：Panthera-DualNXP 的 `source/host/robot_sdk.py`，语义按 1:1 移植）：
  1. discover：收到 `ArmState` 后取 `source_boot_id` 作为板子的启动 ID。
  2. `op1` 申请租约：板端回零未结束时返回 -16；交付配置 `BOOT_HOME=y`，上电先自动回零。
  3. `Configure`：`expected_policy_revision` → `new_policy_revision`。
  4. `op3` 激活。
  5. 每 2 ms 发一帧 `ArmCommand`，必须带 lease、policy、`target_boot_id = 板子启动 ID`、递增的 sequence。
- **其余操作码**：op2 释放，op9 失能关节，op10/11 开关状态流，op17 主机停止，op18 软停，op19–22 能力探测。
- **板端兜底**：主机指令 ≥ 50 ms 未更新就自动软停（需要 `LOCAL_HOME=y`）。固件本身没有应用层限幅（D10 的直通即依赖这一点）。
- **固件时序**：控制任务 500 Hz；CAN 每 333 µs 轮询一个电机，每个电机 500 Hz。网络往返（旧 Python 测量）p50 1.8 ms、p99 6.1 ms。
- **单位**：q rad、dq rad/s、kp Nm/rad、kd Nm·s/rad、tau Nm。默认增益取 `dual-nxp.json`：kp `[24.48, 61.2, 61.2, 12.24, 12.24, 12.24]`，kd `[1.2, 1.92, 1.92, 0.48, 0.48, 0.48]`。
- **关节坐标**：编码器坐标 = URDF 坐标（现有重力映射 sign 全 1、offset 全 0）。J3 有 `encoder_modulo_joints: [2]`，即编码器取模回绕处理，参考 `运行参数.py`；需要在实机上验证。
- **灵巧手**：`HandCommand.target_q_rad[6]` 的顺序是拇指旋转、拇指弯曲、食指、中指、无名指、小指（固件 `hand_joints.h`），对应 URDF 的 `*_thumb_cmc/thumb_pip/index_mcp/middle_mcp/ring_mcp/little_mcp`。**符号、比例、零位未核对**，属于 WP-B/WP-H 的任务。`HandState.joint_q_rad[11]` 是含被动关节的完整反馈。

### 2.2 固件 host-control 构建（新增）
- 配置组合：`local-workspace.conf; dual-common.conf; boards/{left,right}.conf`。不叠加 `gello-standalone*`、`gello-radio*`、`gello-web`；右板要显式设 `CONFIG_ROBOT_GELLO_USB=n`。保留 `LOCAL_HOME=y`、`BOOT_HOME=y`、`PEER_LINK=y`。
- 不带 `GELLO_STANDALONE_ACTUATE` 时，板端才接受主机申请租约。
- **风险**：dual-gello 基线从未在 host-control 模式下验收过；11.6 冰封版是 PC 主控、已经在实物上跑过。差异约 20 个文件，需要在实机上回归。

## 3. 架构

```mermaid
flowchart LR
  U["用户 API（C++/Python）<br/>位姿/速度/关节目标"] --> T["目标缓冲（seqlock）"]
  X["专家 API：MIT 直通"] -.->|"D10：绕过一切"| W
  T --> Q["500 Hz 双臂联合 QP<br/>acados ocp_qp + HPIPM"]
  S["ArmState L/R"] --> K["运动学/胶囊<br/>生成的 C 代码"]
  K --> Q
  Q --> I["q_ref 积分 + g(q) 前馈"]
  I --> W["wirelink 会话 ×2"]
  W -- "UDP" --> B["左/右 NXP"]
  B --> S
  H["手部透传 API"] --> W
```

### 3.1 控制环
- 主机时钟驱动 500 Hz，每周期取两臂最新的 `ArmState`。
- 某臂状态超过 10 ms 未更新：该臂保持上一帧 `q_ref`，`dq = 0`，并上报警告。超过 50 ms 由板端自己软停。
- 输出：`q_ref ← q_ref + q̇*·dt`，下发 MIT `[q_ref, q̇*, kp, kd, g(q)]`。

### 3.2 QP 形式（acados `ocp_qp`，N = 1）
- **变量**：`x0 = q`（12 维，固定为测量值），`u = q̇`（12 维），`x1 = x0 + dt·u`（dt = 2 ms）。
- **代价**：

$$
\sum_{i\in\{L,R\}} \big\|J_i\,\dot q_i - v_i^*\big\|^2_{W_i} + \lambda\|\dot q\|^2 + \mu\|\dot q - \dot q_{\text{prev}}\|^2 + \text{零空间姿态项}
$$

  其中 `v* = Kp·log(T⁻¹T*) + v_ff`。
- **约束**：
  - `x1` 的关节限位取 URDF 值，并按速度阻尼收紧。
  - `|u| ≤ v_max`，`|u − u_prev| ≤ a_max·dt`。
  - 碰撞：固定 **K = 16** 个通用约束槽位：

$$
n^\top\big(J_a - J_b\big)\,\dot q \ge -\xi\,\frac{d - d_s}{d_i - d_s}
$$

    默认 `d_s = 0.02 m`，`d_i = 0.10 m`，`ξ = 0.5 m/s`。每周期按距离选最近的 K 对；空槽位填宽松界。
- **求解器**：`FULL_CONDENSING_HPIPM`，热启动；目标 p99 < 100 µs。
- **QP 不可行或超时**：本周期保持 `q_ref`、`dq = 0`；连续 5 次失败则软停。

### 3.3 碰撞模型
- **配对来源**：`srdf/panthera_dual_a2l1.srdf` 中保留的配对（819 对，其中臂间 573 对），合并到胶囊级别。
- **臂连杆**：每个连杆 1–2 个胶囊，用离线工具从 STL 拟合（PCA 定轴 + 最小包围半径优化），要求**完全包住网格**。
- **手（D11）**：在 mimic 兼容范围内采样手指构型，把手掌坐标系下的点云取并集，拟合 2–3 个胶囊（掌、四指扫掠、拇指扫掠）。
- **运行时**：解析的线段–线段距离，不链接 coal/FCL；coal 只用于离线校验。
- **硬阈值**：任意保留配对的距离 < `d_hard = 0.005 m` 时主机执行软停（op18）。

## 4. 仓库结构（目标）

```text
panthera-spatial-sdk/
├─ docs/                     PLAN.md（本文）、API、协议、安全说明
├─ model/panthera_dual_description/   ✅ URDF/SRDF/网格/配置/校验
├─ tools/
│  ├─ model/                 ✅ build_description.py、verify_description.py
│  └─ codegen/               urdf→C（Pinocchio+CasADi）、胶囊拟合、wlc 重生成（仅离线使用）
├─ protocol/                 robot_messages.wl + *.bind.wl（从固件同步，CI 校验一致）
├─ generated/                已提交：wlc 编解码器、运动学/重力/雅可比 C 代码、胶囊参数
├─ firmware/                 dual-gello 11.20 应用源码 + host-control 配置 + 构建/烧录脚本
│  └─ zephyr/                west.yml（精确提交）+ zephyr-network.patch + gello_usb_ehci.cmake
├─ third_party/              git submodules（见 §5），VERSIONS.md 记录提交/哈希/许可
├─ include/panthera/  src/   transport / session / kinematics / collision / qp / controller
├─ python/                   pybind11 模块 panthera_sdk（scikit-build-core 打包）
├─ sim/                      模拟板：同一协议的 UDP 回环与简化动力学
├─ examples/  tests/
└─ CMakeLists.txt            CMake ≥ 3.22，C++20
```

## 5. 依赖固定方式（D8）

| 依赖 | 方式 | 固定点 | 备注 |
| --- | --- | --- | --- |
| wirelink 核心 | submodule `starwey604/wirelink` | **`608551b3`**（2026-09-22） | 该提交的 `include/wirelink` 和 `src` 两棵树（`ccaa46c3` / `3b3e9ad5`）与固件内置副本**完全一致**，已核对。主机和固件都改用 submodule，CI 校验树哈希 |
| WLC 0.8.0 | submodule `starwey604/wlc` | `758d12a2`（源码包 SHA256 `c97fa894…`） | 只在重生成协议时需要；生成物已提交；CI 断言摘要与 `robot_messages_manifest.json` 一致 |
| acados | submodule | 最新正式 tag | 它自带的 hpipm/blasfeo 子模块按 tag 固定；构建时只开 HPIPM |
| Eigen | submodule | 最新 3.4.x 或 5.x 稳定 tag | 头文件库 |
| pybind11 | submodule | 最新稳定 tag | |
| Zephyr / hal_nxp / CMSIS / segger | `firmware/zephyr/west.yml` | 精确提交（取自 dual-gello `toolchain/west.yml`） | 外加网络补丁；不入库 |
| Pinocchio / CasADi | 不入库 | `tools/codegen/requirements.txt` 固定版本 | 只在离线生成时用 |

> [!WARNING]
> `starwey604/wirelink` 和 `starwey604/wlc` 对外部用户是否可访问需要核实。不可访问时，按 D8 退回 vendor：拷贝上面固定提交的源码，并在 VERSIONS.md 记录来源。

## 6. 公共 API（草案，执行时可细化，语义不变）

```cpp
namespace panthera {
auto robot = DualArm::connect(Config::fromFile("config/dual_arm.yaml"));
robot.enable();                                   // discover → 等回零 → lease → configure → activate
robot.setPoseTarget(Arm::Left, T, twist_ff);      // 任意线程、任意频率
robot.setTwistTarget(Arm::Right, twist);
robot.setJointTarget(Arm::Left, q);               // 仍经过 QP 和碰撞约束
robot.setHandTarget(Arm::Left, q6, duration_ms);  // 透传（D11）
State s = robot.state();  // q/dq/tau、TCP 位姿、最小碰撞距离与配对、QP 状态、时延统计、故障
robot.softStop(); robot.disable();
// 专家模式（D10）：显式开启，完全直通，不做任何监控
auto raw = robot.enterExpertMode(ExpertAck::IUnderstandNoSafety);
raw.sendMit(Arm::Left, q, dq, kp, kd, tau);
}
```

Python 版本 `panthera_sdk` 与 C++ 一一对应，命名用 snake_case，位姿用 numpy 4×4 矩阵，6 维向量用 numpy 数组。

## 7. 工作包（可并行，括号内为依赖）

| WP | 内容 | 验收 |
| --- | --- | --- |
| A 骨架与依赖 | §5 的全部 submodule、VERSIONS.md、顶层 CMake、CI（Linux x86_64 + Windows x64 为一级平台，macOS arm64 / Linux aarch64 为二级）、clang-format | 全新克隆加 `--recursive` 后一条命令构建通过；CI 绿；wirelink 树哈希断言通过 |
| B 通信与会话（A） | C++ 传输层与 wirelink 会话（移植 robot_sdk.py 全部语义）、双板、时延统计、`sim/` 模拟板、手部透传 | 对模拟板跑会话全流程和异常测试（板重启、丢包、旧包、RPC 超时）；模拟板 500 Hz 下无丢周期 |
| C 模型代码生成（A） | 从单臂 URDF 生成 CasADi C：FK、雅可比、g(q)；胶囊拟合（臂 + 手包络）；`generated/` 入库 | 与 Pinocchio 误差 < 1e-9；胶囊完全包住网格（逐顶点检查）；手包络覆盖全部采样的手指构型 |
| D 碰撞与 QP（C） | 胶囊距离、最近 K 对选择、acados ocp_qp 封装、不可行处理 | 求解 p99 < 100 µs（x86_64）；仿真里两臂对冲最终停在 `d_s` 以外；随机目标下从不穿透 |
| E 控制器（B、D） | 500 Hz 环、目标插值、q_ref 积分、g(q) 前馈、状态陈旧处理、软停策略、专家模式 | 模拟板闭环跟踪误差达标；所有降级路径都有测试 |
| F Python 与文档（E） | pybind11 绑定、wheel、示例（读状态、笛卡尔移动、双臂靠近自停、手部透传、专家模式）、文档 | wheel 在一级平台可安装；示例对模拟板跑通 |
| G 固件导入（A） | 导入 dual-gello 11.20 应用源码，改为引用 wirelink submodule；新增 host-control 配置；构建与烧录脚本；预编译镜像 + SHA256 | 两块板的 host-control 镜像在固定 west 版本下可复现构建；原生单元测试全过 |
| H 实机验收（E、G，需人在场） | 只读 500 Hz 时延/丢包；单臂单关节小幅；手部映射符号核对；J3 取模验证；双臂靠近自停；1 小时稳定性 | 结果记录在 `validation/` |

## 8. 仍待定（不阻塞开工，执行中按默认处理）

1. **TCP**：默认取 `palm_base_link` +z 0.100 m（占位值），可在 `dual_arm.yaml` 调整。
2. **两底座共面且平行**：默认按是处理。
3. **重新辨识重力**：放在 WP-H 之后，作为独立任务。
4. **QP 参数**（权重、`d_s`/`d_i`/`ξ`、`v_max`/`a_max`）：先用 §3 的默认值，实机调参后写回配置。

## 9. 资料位置

| 内容 | 路径 |
| --- | --- |
| 固件基线 | `/home/ww/codings/dual-gello`（README 交接速查、`boards/`、`toolchain/west.yml`、`source/`） |
| 11.6 PC 主控固件与 Python SDK 参考 | `/home/ww/codings/Panthera-DualNXP-Hyperhands`（`source/host/robot_sdk.py`、`robot_protocol.py`、`dual-nxp.json`、`运行参数.py`） |
| wirelink / WLC | `/home/ww/codings/wirelink`（固定提交 `608551b3`）；WLC：`github.com/starwey604/wlc@758d12a2` |
| 前作 C++ SDK（acados 用法、URDF→C、pybind11 结构） | Panthera-DualNXP 中 `source/host/examples/遥操资料_20261030_完整版(2)/…/Ragtel_遥操作GUI/sdk/fci`；`/home/ww/codings/libflorid` |
| 灵巧手原始模型 | `/home/ww/codings/L1_Hand/A2L1Description`（已拷入 `model/…/sources/a2l1_hand`） |
| 工控机环境 | 见 dual-gello README“交接速查”：绿盾 DLP、内核固定、ASan 绕过方法 |
