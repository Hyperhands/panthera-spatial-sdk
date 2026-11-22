# API 与控制参考

## 1. Python 快速示例

```python
import numpy as np
import panthera_sdk as sdk

config = sdk.ControlConfig()
config.translation_speed = 0.15
config.joints.max_speed = np.full(12, 0.8)

with sdk.DualArm(sdk.BoardAddress.left(), sdk.BoardAddress.right(), config) as arm:
    arm.enable(timeout_ms=30000)

    state = arm.state()
    left_tcp = state.tcp[0].copy()
    left_tcp[0, 3] += 0.05  # 沿世界坐标系 +x 前移 50 mm
    arm.set_pose_target(sdk.Side.left, left_tcp)

    # 读取模型最小臂间距离（单位：米）
    dist = sdk.collision_state(state.measured).inter_arm
    print(f"inter_arm = {dist * 1000:.1f} mm, qp = {state.qp_time * 1e6:.1f} us")
```

- `with sdk.DualArm(...) as arm` 退出上下文时自动调用 `soft_stop()`，板端切入柔顺重力保持。
- 接管等待期间按 `Ctrl+C` 会自动回滚已取得的租约。
- `arm.set_pose_target`、`arm.set_twist_target`、`arm.set_joint_target`、`arm.set_hand_target`、`arm.set_task_weights` 均可从任意线程安全调用，由内部 500 Hz 控制线程以无锁快照读取。

---

## 2. 核心接口一览

### `DualArm`（Python: `panthera_sdk.DualArm` / C++: `panthera::DualArm`）

- **头文件**：[include/panthera/controller.hpp](../include/panthera/controller.hpp)
- **Python 类型声明**：[python/panthera_sdk/_core.pyi](../python/panthera_sdk/_core.pyi)

| 方法（Python / C++） | 说明 |
| --- | --- |
| `enable(timeout_ms)` / `enable(timeout)` | 等待两板就绪、申请控制租约、同步编码器整圈零位并启动 500 Hz 控制线程 |
| `set_pose_target(side, pose, feedforward)` / `setPoseTarget` | 设置世界坐标系下的 4×4 TCP 位姿目标与可选 6D 前馈速度 `[vx, vy, vz, wx, wy, wz]` |
| `set_twist_target(side, twist)` / `setTwistTarget` | 设置世界坐标系 6D TCP 速度目标（线速度 m/s，角速度 rad/s） |
| `set_joint_target(side, q)` / `setJointTarget` | 设置单臂 6 轴关节角目标（rad） |
| `set_hand_target(side, q6, duration_ms)` / `setHandTarget` | 透传 6 个主动手指关节目标（rad）与插值时长（ms） |
| `set_task_weights(w6)` / `set_arm_task_weights(side, w6)` | 动态调整任务空间 6D 权重（例如设后三项为 0 切换为纯 3D 位置追踪） |
| `state()` | 获取当前 `ControlState` 快照（实测/参考关节角、双臂 TCP、碰撞间隙、QP 耗时、告警/错误） |
| `soft_stop()` / `softStop()` | 停止主机控制线程并请求两板进入柔顺保持 |

### `BoardTelemetry`（只读遥测）

- **头文件**：[include/panthera/telemetry.hpp](../include/panthera/telemetry.hpp)
- 不申请控制租约，仅绑定本地 UDP 端口接收板端遥测。
- `poll()` 返回独立快照，包含 `arm`、`hand`、`faults`、`diagnostics` 四路最新帧、序号缺口计数 `missing`、`boot_id` 与重启计数 `restarts`。
- `active_hand_angles(max_age_us)` 按固件顺序返回 6 个主动手关节角（rad）；数据过期或无效时返回 `None` / `std::nullopt`。

### `GelloReader`（USB 舵机轮询）

- **头文件**：[include/panthera/gello.hpp](../include/panthera/gello.hpp)
- 基于 `third_party/astrial` 后台线程轮询单串口 14 路飞特舵机，提供 `list_gello_ports()` 与 `poll()`（返回 `connected`、`scan_hz` 与 14 路舵机的 `raw`、`ticks`、`velocity`、`age_s`、`valid`）。

### 几何与碰撞查询

- `arm_geometry(side, q6)`：计算单臂各连杆变换、TCP 位姿、6×6 雅可比与 6 轴重力矩 `gravity`（N·m）。
- `collision_state(q12)`：计算双臂碰撞状态，返回：
  - `inter_arm`：双臂之间最小距离（m）；
  - `closest`：全局最小距离（含同臂近距对，m）；
  - `hard_margin`：相对硬停阈值的最小余量（m）；
  - `pair`：最近碰撞对索引。

---

## 3. 坐标系、关节限位与默认参数

- **世界坐标系**：两底座连线中点为原点，**+x 向前**（零位工具指向）、**+y 向左**（指向左臂）、**+z 向上**。两底座中心距 480 mm，平行共面。
- **手部顺序**：`[thumb_cmc, thumb_pip, index_mcp, middle_mcp, ring_mcp, little_mcp]`。
- **默认软限位**（双臂一致）：
  - J1：`±155°`
  - J2：`0° ~ 180°`
  - J3：`0° ~ 240.6°`（`4.2 rad`）
  - J4：`±90°`
  - J5：`±60°`（相对 URDF ±100° 收紧以留出腕部安全余量）
  - J6：`±160°`
- **默认运动限制**（`default_bounds()`）：速度 `1.0 rad/s`、加速度 `8.0 rad/s²`、参考超前量 `max_lead = 0.08 rad`。
- **下发 MIT 增益**（[src/session.cpp](../src/session.cpp#L41-L42)）：
  - `kp = [24.48, 90.0, 90.0, 36.72, 12.24, 12.24]` N·m/rad
  - `kd = [1.2, 2.5, 2.5, 0.8314, 0.48, 0.48]` N·m·s/rad

---

## 4. 安全与保护行为

- **温和限幅（不停机）**：
  - 达到软限位、同臂间隙边界或跟踪滞后超 `max_lead` 时，控制器自动钳位参考并写入 `state.warning`，不会直接停机；
  - 臂间距离进入 35 mm 影响区后 QP 施加几何速度阻尼，平稳停在 **20 mm** 附近；
  - 单拍 QP 不可行时自动切入安全优先最小制动投影，若仍无解则本拍冻结保持。
- **底线保护（触发 `softStop`）**：
  - 实测关节超出机械硬限位（默认限位外 ±10°）或反馈含 NaN；
  - 臂间距离低于 **5 mm**；
  - 严重物理堵转（跟踪误差超过 `tracking_stop_rad`，默认 `max(3 * max_lead, 0.45 rad ≈ 25.8°)`）；
  - 通信断链 ≥ 50 ms（与板端 50 ms 看门狗对齐）、编码器单拍跳变 > π、板端重启或上报电机/MCU 故障。

---

## 5. 专家模式（原始 MIT 直通）

> 专家模式会停止正常 500 Hz 控制线程，**绕过 QP、自碰撞检查与关节限幅**，直接按原始编码器坐标向单侧发送 30 个 MIT 浮点数（6 轴 × `[q, dq, kp, kd, tau_ff]`）。板端仍保留租约校验与 50 ms 看门狗。

```python
arm.enter_expert_mode(sdk.ExpertAck.IUnderstandNoSafety)
arm.send_mit(sdk.Side.left, mit_30_floats)
```

完整用法见 [examples/expert_mit.py](../examples/expert_mit.py)。
