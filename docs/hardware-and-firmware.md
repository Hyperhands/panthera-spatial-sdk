# 硬件、固件与模型

## 1. 网络与协议拓扑

上位机与两块 NXP MCXN947 控制板通过以太网 UDP + wirelink（CRC32C，schema revision 7）通信：

| 设备 | 节点号 | IP 地址 | 实时端口（500 Hz 控/遥测） | 管理端口（可靠 RPC） |
| --- | --- | --- | --- | --- |
| 上位机 | 10 | `192.168.10.1` | 绑定 5005（左）/ 5015（右） | 绑定 5006（左）/ 5016（右） |
| 左臂板 | 11 | `192.168.10.11` | 5005 | 5006 |
| 右臂板 | 12 | `192.168.10.12` | 5015 | 5016 |

协议源码位于 `protocol/robot_messages.wl` 与 `protocol/*.bind.wl`，由 `third_party/wlc` 生成到 `generated/protocol/`，主机与固件共享同一份协议定义。

---

## 2. 固件构建与烧录

仓库在 `firmware/images/` 中附带了预编译并校验好的左右板 host-control 镜像（`left.bin`、`right.bin` 与 `SHA256SUMS`），默认开启上电自动回零、双板对端链路与 50 ms 主机看门狗。

### 校验预编译镜像

```bash
(cd firmware/images && sha256sum -c SHA256SUMS)
```

### 使用 MCU-Link 烧录

`firmware/tools/flash-dual.py` 会在烧录前自动备份原固件、写入新镜像并执行逐字节回读校验：

```bash
python3 firmware/tools/flash-dual.py left  --probe <LEFT_PROBE_SERIAL>  --pack <MCXN947_CMSIS_PACK>
python3 firmware/tools/flash-dual.py right --probe <RIGHT_PROBE_SERIAL> --pack <MCXN947_CMSIS_PACK>
```

如需从源码重新编译固件或运行固件原生单元测试，请见 [firmware/README.md](../firmware/README.md)。

---

## 3. 模型与离线代码生成

仓库已提交所有离线生成物（`model/`、`generated/model/`、`sim/assets/`），**常规编译与运行 SDK 无需安装 Pinocchio / CasADi**。

仅在修改装配参数（如底座间距、手部安装角、转接件厚度或 TCP 偏置）时需要重新生成：

### 3.1 修改装配参数并重建 URDF / SRDF

编辑唯一参数文件 [model/panthera_dual_description/config/dual_arm.yaml](../model/panthera_dual_description/config/dual_arm.yaml)，然后运行：

```bash
python3 tools/model/build_description.py
python3 tools/model/verify_description.py
```

### 3.2 重新生成运动学、重力与碰撞胶囊 C 代码

```bash
python -m pip install -r tools/codegen/requirements.txt
python tools/codegen/model.py
python tools/codegen/capsules.py
```

生成结果写入 `generated/model/`（`panthera_left.{c,h}`、`panthera_right.{c,h}`、`geometry.inc`、`capsules.json`）。

### 3.3 重新生成 MuJoCo 仿真场景

```bash
python tools/sim/build_mujoco_scene.py
```

生成结果写入 `sim/assets/panthera_dual_mocap.xml`。
