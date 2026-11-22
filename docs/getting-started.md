# Getting Started

本指南帮助你从零配置 `panthera-spatial-sdk` 开发与运行环境，并在几分钟内跑通本机仿真与实机通信。

---

## 1. 环境要求

- **编译器**：C 编译器 + 支持 C++20 的 C++ 编译器（GCC ≥ 11 / Clang ≥ 14 / MinGW-w64 UCRT64）
- **构建工具**：Git、CMake ≥ 3.22（推荐 ≥ 3.25，可使用 preset 工作流）、Ninja
- **Python**：Python ≥ 3.10（推荐 3.10–3.13）及开发头文件、`pip`、`numpy >= 1.23`
- **可选（3D 仿真与拖拽交互）**：`mujoco >= 3.2`

### Linux (Ubuntu 22.04 / 24.04)

```bash
sudo apt update
sudo apt install -y git build-essential cmake ninja-build python3-dev python3-venv python3-tk
```

> `python3-tk` 用于双臂 GELLO 遥操的 14 路舵机实时图形面板；未安装时会自动回退到终端面板。

### Windows (MSYS2 UCRT64)

在 **MSYS2 UCRT64** 终端中安装工具链：

```bash
pacman -S --needed git \
  mingw-w64-ucrt-x86_64-gcc \
  mingw-w64-ucrt-x86_64-cmake \
  mingw-w64-ucrt-x86_64-ninja \
  mingw-w64-ucrt-x86_64-python \
  mingw-w64-ucrt-x86_64-python-numpy \
  mingw-w64-ucrt-x86_64-python-pip
```

### macOS

```bash
brew install cmake ninja python
```

> macOS 上运行 MuJoCo 图形窗口时，请用 `mjpython examples/mujoco_mocap_qp.py` 替代 `python`。

---

## 2. 克隆仓库并拉取子模块

所有底层依赖（`acados` / `HPIPM` / `BLASFEO`、`Eigen`、`wirelink`、`Asio`、`pybind11`、`astrial`）均以 Git submodule 固化在 `third_party/` 中：

```bash
git clone <repo-url> panthera-spatial-sdk
cd panthera-spatial-sdk
git submodule update --init --recursive
```

---

## 3. 安装 Python SDK

建议在独立虚拟环境中安装：

```bash
python3 -m venv .venv
source .venv/bin/activate          # Windows UCRT64: source .venv/Scripts/activate

# 安装核心 SDK + MuJoCo 仿真依赖
python -m pip install --upgrade pip
python -m pip install '.[mujoco]'
```

如仅需核心控制接口（不装 MuJoCo）：

```bash
python -m pip install .
```

如直接安装预编译 wheel：

```bash
python -m pip install <path-to-panthera_sdk.whl>
```

---

## 4. 构建 C++ 核心与运行测试（可选）

CMake ≥ 3.25 可直接使用一键工作流 preset：

```bash
cmake --workflow --preset host
```

CMake 3.22–3.24 可分步执行：

```bash
cmake -S . -B build/host -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/host
ctest --test-dir build/host --output-on-failure
```

构建完成后，C++ 可执行示例位于 `build/host/panthera-read-state`（Windows 为 `.exe`），静态库目标为 `panthera_controller`。

---

## 5. 首次离线验证（无需硬件）

运行以下任意一条命令即可验证环境完全就绪：

```bash
# 1. 启动 MuJoCo 双臂空间控制交互窗口
python examples/mujoco_mocap_qp.py

# 2. 运行无界面 MuJoCo 跟踪与防碰撞自检
python examples/mujoco_mocap_qp.py --headless

# 3. 运行双臂 GELLO 遥操离线自检
python examples/dual_gello_teleop.py --mock-gello --headless

# 4. 运行基础 API 示例（默认本机 UDP 模拟板）
python examples/read_state.py
python examples/cartesian_move.py
python examples/approach_stop.py
```

---

## 6. 连接实机前的系统配置

### 6.1 主机以太网配置

两块 NXP 控制板与主机通过以太网直连或交换机互联，固定 IP 如下：

| 节点 | IP 地址 | 实时 UDP 端口 | 管理 UDP 端口 |
| --- | --- | --- | --- |
| 主机（node 10） | `192.168.10.1/24` | 绑定 5005 / 5015 | 绑定 5006 / 5016 |
| 左臂控制板（node 11） | `192.168.10.11` | 5005 | 5006 |
| 右臂控制板（node 12） | `192.168.10.12` | 5015 | 5016 |

在 Linux 上将对应网卡设为 `192.168.10.1/24`，并确保 5005/5006/5015/5016 端口未被其他程序占用：

```bash
# 示例：检查端口占用
ss -ulnp | grep -E '5005|5006|5015|5016'

# 上电后先只读检查两板 500 Hz 遥测（不接管控制权）
python examples/read_state.py --real
```

### 6.2 GELLO USB 串口权限（Linux）

双臂 GELLO 通过单个 USB 转串口适配器连接主机（1 Mbps 8N1）。Linux 下将当前用户加入 `dialout` 组后重新登录即可免 `sudo` 访问 `/dev/ttyUSB*`：

```bash
sudo usermod -aG dialout "$USER"
# 重新登录后验证
python examples/dual_gello_teleop.py --list-ports
```

---

## 下一步

- 查看 [示例使用指南](examples.md) 了解 MuJoCo 空间控制与双臂 GELLO 遥操的全部参数与用法。
- 查看 [API 与控制参考](api.md) 将 SDK 集成到你自己的 Python 或 C++ 程序中。
