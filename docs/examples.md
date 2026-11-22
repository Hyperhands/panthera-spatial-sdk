# 示例使用指南

所有示例脚本均位于 `examples/` 目录，默认运行在本机仿真回环模式下；连接实机时显式传入 `--real`。

---

## 1. 空间控制示例：`examples/mujoco_mocap_qp.py`

通过 MuJoCo 交互窗口拖动左右末端目标球（蓝色 = 左臂，橙色 = 右臂），驱动 500 Hz acados QP 任务空间控制器，实时展示臂间碰撞距离、限位告警与 QP 求解耗时。

### 常用运行方式

```bash
# 本机 MuJoCo 仿真交互（默认从 ready 姿态启动）
python examples/mujoco_mocap_qp.py

# 本机仿真，从折叠零位启动
python examples/mujoco_mocap_qp.py --start-zero

# 实机双臂控制（MuJoCo 同步显示真实双臂与手部姿态，拖动目标球控制实机）
python examples/mujoco_mocap_qp.py --real

# 实机启动后先自动移动到 ready 姿态，再进入拖拽控制
python examples/mujoco_mocap_qp.py --real --move-ready

# 实机默认开启 6D 姿态锁定模式
python examples/mujoco_mocap_qp.py --real --lock-orientation

# 无界面回环回归验证（校验重力矩、跟踪误差与双臂对向自停）
python examples/mujoco_mocap_qp.py --headless
```

### 窗口交互与快捷键

- **选中目标球**：双击蓝色（左）或橙色（右）目标球。
- **拖动平移**：按住 `Ctrl` + 鼠标右键拖动。
- **拖动旋转**：按住 `Ctrl` + 鼠标左键拖动（在 6D 模式下生效）。
- **`O`**：切换 **3D 位置模式**（默认，手腕姿态自由）与 **6D 位姿锁定模式**。
- **`S` 或 `R`**：将两个目标球立即吸附回当前 TCP 位姿。
- **`E`**：若控制器因保护机制停止，按下即可重新使能并同步句柄，无需重启窗口。

### 常用参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--real` / `--loopback` | `--loopback` | 连接实机或本机 MuJoCo 仿真 |
| `--move-ready` | 关闭 | 仅 `--real` 有效，启动后先将双臂平滑移至 `ready` 姿态 |
| `--lock-orientation` | 关闭 | 启动时直接进入 6D 位姿锁定模式 |
| `--speed` | `0.25` | 笛卡尔平移速度上限（m/s） |
| `--joint-speed` | `1.0` | 单关节角速度上限（rad/s） |
| `--pose-gain` | `6.0` | TCP 位姿跟踪比例增益 |
| `--leash` | `0.15` | 拖拽目标球相对当前 TCP 的最大弹性牵引距离（m） |
| `--snap-delay` | `0.40` | 松开鼠标且手臂停稳后，目标球自动吸回 TCP 的延迟（s，`0` 为关闭） |

---

## 2. 双臂 GELLO USB 遥操示例：`examples/dual_gello_teleop.py`

通过单个 USB 转串口适配器直接轮询 14 路飞特 STS/SCS 舵机（左臂 ID 1–7、右臂 ID 11–17，1 Mbps 8N1），在主机侧解算双臂 12 轴关节角与五指灵巧手抓取角，经 `DualArm` 500 Hz QP（带限位、限速、重力补偿与双臂自碰撞保护）驱动仿真或实机双臂。

### 常用运行方式

```bash
# 1. 列出可用串口（跨平台）
python examples/dual_gello_teleop.py --list-ports

# 2. 独立舵机可视化面板（无需连接机械臂/NXP，用于检查舵机连接、方向、扳机与零位）
python examples/dual_gello_teleop.py --monitor --port /dev/ttyUSB0
# 无硬件预览面板：
python examples/dual_gello_teleop.py --monitor --mock-gello

# 3. 本机仿真回环（接真实 GELLO 核对 12 轴方向与防自撞；加 --viewer 打开 14 路舵机面板）
python examples/dual_gello_teleop.py --port /dev/ttyUSB0 --viewer

# 4. 实机双臂遥操
python examples/dual_gello_teleop.py --real --port /dev/ttyUSB0

# 5. 实机遥操 + 图形面板（若已装 MuJoCo 会同时打开 3D 实时孪生窗口）
python examples/dual_gello_teleop.py --real --port /dev/ttyUSB0 --viewer

# 6. 单臂遥操或关闭灵巧手
python examples/dual_gello_teleop.py --real --port /dev/ttyUSB0 --side left --no-hand

# 7. 无硬件离线自检
python examples/dual_gello_teleop.py --mock-gello --headless
```

### 零位与保护行为

- **启动自动取零（默认 `--zero-mode startup`）**：启动后保持主手静止 0.5 s 自动锁定零位；也可用 `--zero-mode saved` 使用标定零位。
- **失联/陈旧保护**：主手串口数据超过 `--stale-ms`（默认 50 ms）未更新时，机械臂自动冻结在当前姿态，主手恢复后平滑追赶。
- **在线恢复**：若 `DualArm` 触发保护停机，面板状态显示 `STOPPED` 且窗口保持打开，点击面板右上角 **Re-enable** 即可重新使能并重新取零。

### 常用参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--port` | 自动检测 | GELLO USB 串口路径（如 `/dev/ttyUSB0`、`COM5`） |
| `--baud` | `1000000` | 舵机总线波特率 |
| `--side` | `both` | 控制哪一侧：`both` / `left` / `right` |
| `--zero-mode` | `startup` | `startup`（静止 0.5 s 取零）或 `saved`（使用标定零位） |
| `--anchor` | `zero` | 主手零位对应的机械臂姿态：`zero`（零位）或 `current`（接管时当前姿态） |
| `--config` | 内置标定 | 自定义 JSON 标定文件路径，覆盖默认符号、零位与扳机范围 |
| `--max-speed` | `0.5` | 关节速度上限（rad/s） |
| `--max-accel` | `4.0` | 关节加速度上限（rad/s²） |
| `--no-hand` | 关闭 | 不下发五指灵巧手目标 |
| `--viewer` | 关闭 | 打开 14 路舵机实时面板（已装 MuJoCo 时同步打开 3D 孪生） |

---

## 3. 基础 API 示例

每个示例都是独立的精简脚本，直接演示单一 SDK 能力：

| 脚本 | 功能 | 运行命令（仿真 / 实机） |
| --- | --- | --- |
| `examples/read_state.py` | 只读订阅两板 500 Hz 遥测（不申请控制租约） | `python examples/read_state.py [--real]` |
| `examples/cartesian_move.py` | 双臂 TCP 沿 +x 直线往返移动 | `python examples/cartesian_move.py [--real]` |
| `examples/approach_stop.py` | 双臂朝中平面 `y=0` 对向接近，演示 20 mm 自动减速自停 | `python examples/approach_stop.py [--real]` |
| `examples/hand_passthrough.py` | 透传 6 个主动手关节目标并回读实测手指角 | `python examples/hand_passthrough.py [--real]` |
| `examples/expert_mit.py` | 专家模式：直接下发 30 浮点原始 MIT 帧（实机需加 `--i-understand-no-safety`） | `python examples/expert_mit.py [--real --i-understand-no-safety]` |

### C++ 只读遥测示例

构建 C++ 目标后可直接运行：

```bash
# 连接实机（主机 IP 需配置为 192.168.10.1）
./build/host/panthera-read-state

# 本机回环测试：终端 1 启动模拟板，终端 2 读取状态
python3 sim/telemetry.py
./build/host/panthera-read-state --loopback
```
