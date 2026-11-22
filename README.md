# panthera-spatial-sdk

Panthera-HT 双臂（2×6 轴）+ A2/L1 五指灵巧手的上位机空间控制 SDK。

- **500 Hz 双臂空间控制**：基于 acados QP（HPIPM），内置重力补偿、关节限位/限速与双臂自碰撞约束（臂间默认停在 20 mm 外）。
- **C++20 核心 + Python 接口**：核心库与 pybind11 绑定同仓维护，支持 Linux、Windows（MSYS2 UCRT64）与 macOS。
- **开箱即用的仿真与遥操**：内置 MuJoCo 双臂拖拽交互与单 USB 口双臂 GELLO 遥操示例，所有示例默认跑本机仿真，传入 `--real` 即可切到实机。

---

## 核心示例速查

### 1. 空间控制示例（500 Hz acados QP + MuJoCo 拖拽交互）

脚本路径：`examples/mujoco_mocap_qp.py`（注：控制器采用 500 Hz acados QP 任务空间控制，非非线性 NMPC）。

```bash
# 本机 MuJoCo 仿真交互（双击蓝色/橙色目标球选中，Ctrl+右键平移，Ctrl+左键旋转）
python examples/mujoco_mocap_qp.py

# 实机双臂控制（MuJoCo 作为实时数字孪生与拖拽手柄；可选 --move-ready 先抬至准备姿态）
python examples/mujoco_mocap_qp.py --real
python examples/mujoco_mocap_qp.py --real --move-ready

# 无界面回环自检（验证重力矩、轨迹跟踪与双臂靠近自停）
python examples/mujoco_mocap_qp.py --headless
```

窗口快捷键：`O` 切换 3D 位置 / 6D 位姿锁定，`S` 或 `R` 将目标球吸附回当前 TCP，`E` 在控制器停止后重新使能。

### 2. 双臂 GELLO USB 遥操示例

脚本路径：`examples/dual_gello_teleop.py`（单 USB 转串口直连 14 路飞特舵机：左臂 ID 1–7、右臂 ID 11–17，经 500 Hz QP 与自碰撞保护驱动双臂及五指灵巧手）。

```bash
# 1. 查看检测到的串口
python examples/dual_gello_teleop.py --list-ports

# 2. 独立舵机状态面板（无需连接机械臂/NXP，实时查看 14 路舵机原始值、角度、扳机与延迟）
python examples/dual_gello_teleop.py --monitor --port /dev/ttyUSB0

# 3. 本机仿真遥操（接真实 GELLO 核对 12 轴方向与自碰撞；已装 MuJoCo 时同步打开 3D 窗口）
python examples/dual_gello_teleop.py --port /dev/ttyUSB0 --viewer

# 4. 实机双臂遥操（可选 --viewer 打开面板/3D 孪生，--no-hand 关闭手部，--side left|right 单臂）
python examples/dual_gello_teleop.py --real --port /dev/ttyUSB0

# 5. 无硬件离线自检
python examples/dual_gello_teleop.py --mock-gello --headless
```

---

## 快速开始

完整安装教程见 [Getting Started](docs/getting-started.md)。Linux 下最快 3 步跑通仿真：

```bash
# 1. 拉取子模块
git submodule update --init --recursive

# 2. 安装 Python 包（含 MuJoCo 仿真依赖）
python -m pip install '.[mujoco]'

# 3. 启动 MuJoCo 双臂空间控制交互窗口
python examples/mujoco_mocap_qp.py
```

如需构建 C++ 核心库并运行全部 C++ 测试：

```bash
cmake --workflow --preset host
```

---

## 文档导航

| 文档 | 内容 |
| --- | --- |
| [Getting Started](docs/getting-started.md) | 环境安装（Linux / Windows MSYS2 / macOS）、C++ 与 Python 构建、主机网卡与串口配置、首次运行验证 |
| [示例使用指南](docs/examples.md) | MuJoCo 空间控制、双臂 GELLO 遥操、5 个基础 Python 示例与 C++ 遥测示例的完整参数与操作说明 |
| [API 与控制参考](docs/api.md) | Python & C++ 核心接口、世界坐标系约定、关节限位与默认增益、自碰撞保护与专家模式 |
| [硬件、固件与模型](docs/hardware-and-firmware.md) | 双板网络拓扑、MCU-Link 固件构建与烧录、URDF/胶囊碰撞模型重新生成 |
| [模型说明](model/panthera_dual_description/README.md) | 整机 URDF/SRDF、装配参数与碰撞矩阵详情 |
| [固件说明](firmware/README.md) | MCXN947 host-control 固件 Zephyr 构建步骤与预编译镜像校验 |
| [第三方依赖清单](third_party/VERSIONS.md) | 固化的子模块版本、提交哈希与许可证 |

---

## 仓库结构

```text
panthera-spatial-sdk/
├─ docs/                 使用文档（从 README 与 Getting Started 进入）
├─ examples/             Python & C++ 示例（核心：mujoco_mocap_qp.py、dual_gello_teleop.py）
├─ include/panthera/     C++ 公开头文件（controller / session / telemetry / geometry / gello）
├─ src/                  C++ 核心实现（500 Hz QP、会话管理、碰撞几何、GELLO 串口轮询）
├─ python/panthera_sdk/  pybind11 绑定与类型存根（_core.pyi）
├─ sim/                  本机 UDP 模拟板与 MuJoCo 物理仿真桥接
├─ model/                双臂 + 灵巧手 URDF/SRDF 与网格文件
├─ generated/            离线生成的协议编解码与运动学/重力/碰撞 C 代码
├─ protocol/             wirelink / WLC 通信协议定义
├─ firmware/             NXP MCXN947 固件源码、配置、构建/烧录脚本与预编译镜像
├─ tools/                模型生成、碰撞胶囊拟合与 CI 脚本
└─ tests/                C++ 与 Python 回归测试
```
