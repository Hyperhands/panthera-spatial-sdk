# WP-H 实机验收操作指南

本文用于指导现场实机验收（对应 [PLAN.md](PLAN.md) WP-H）。**所有带电与运动阶段必须有人在场，急停开关触手可及。** 验收记录与原始日志统一归档至 `validation/<YYYYMMDD>/`。

> 工具入口为 `python examples/wp_h_accept.py <stage> --smoke`（回环）或 `--real`（现场）。用 `--help` 查看 Stage 1–6 的子命令。各阶段 JSON 的 `automated_checks` 记录脚本结果，人工判定与尺寸读数填入现场报告。

---

## 0. 实机前门禁与安全准备

上电前逐项确认：

- [ ] **离线验收全绿**：在目标工控机上确认构建与测试通过：
  ```bash
  ctest --test-dir build/host --output-on-failure
  (cd firmware/images && sha256sum -c SHA256SUMS)
  ```
- [ ] **固件镜像摘要核对**：
  - `left.bin` (node11 / `192.168.10.11`): `3694ed3bf0301815459c697ef6b5dfb4fb0fba0f81387dff6c8239f3d7b767d5`
  - `right.bin` (node12 / `192.168.10.12`): `339c4ecda6e8d57b827c6d81f2835060dfadd682328179ffa0d62e7ef8c7dbb8`
- [ ] **主机网络配置**：主机网卡固定为 `192.168.10.1/24`；确认 `5005/5006/5015/5016` 端口未被旧进程占用（`ss -ulnp | grep -E '5005|5006|5015|5016'`）。
- [ ] **物理安装核对**：两底座中心距 480 mm、平行共面；两手相对工具轴 pitch 向上 45°；周边运动包络内清空杂物；急停有效。
- [ ] **应急原则**：
  - 正常停止用 `softStop()`（发 `op18`，板端进入 `HOME_SOFT_HOLD` 重力+阻尼柔顺保持），**不要**在柔顺保持后发 `op11`。
  - 任何异常姿态、异响或失控风险，**人员须在双臂运动包络外直接拍急停开关，严禁徒手进入工作区拦截或托扶机械臂**。

---

## Stage 0：固件刷写与回滚验证（host-control）

1. 沿现场机械臂的接线核对左右板，记录各自的完整 MCU-Link 序列号及 MCXN947 CMSIS pack 路径 `<PACK>`。2026-11-21 核对的这套双臂，左臂探针尾号为 `ZEBZ`（`left.bin`，node11），右臂为 `2D5A`（`right.bin`，node12）。下列命令中的 `<LEFT_PROBE>` 和 `<RIGHT_PROBE>` 均填完整序列号。
2. 逐板刷写（工具会自动备份原固件、全字节回读校验并记录 JSON 证据）：
   ```bash
   python3 firmware/tools/flash-dual.py left  --probe <LEFT_PROBE> --pack <PACK> --backup-dir validation/$(date +%Y%m%d)/flash
   python3 firmware/tools/flash-dual.py right --probe <RIGHT_PROBE> --pack <PACK> --backup-dir validation/$(date +%Y%m%d)/flash
   ```
   > 首次上电如不想立即自动回零，可先只接控制板（不给驱动上电），或配合本地构建的 ELF 传入 `--maintenance-elf <build/firmware-side/zephyr/zephyr.elf>` 抑制本次启动回零。
3. **回滚方法**：任一阶段发现 host-control 固件异常，立即用备份文件刷回：
   ```bash
   python3 firmware/tools/flash-dual.py <side> --image <backup.bin> --probe <PROBE> --pack <PACK>
   ```
4. **通过标准**：两板均输出 `"readback_verified": true`，保留备份 `.bin` 与 `dual-flash-{left,right}.json`。

---

## Stage 1：只读 500 Hz 遥测、回零与整圈分支核对（无主机发控）

1. 给两臂驱动上电，观察板端 `BOOT_HOME` 自动回零过程（先手部就绪等待，再经 `home_waypoint` 回到零位 `HOME_HOLD`）。确认运动平稳、无碰撞、最终静止在零位。
2. 运行只读状态检查（不申请租约、不接管）：
   ```bash
   ./build/host/panthera-read-state
   ```
3. 持续只读采样 60 秒（在 WP-F / 验收工具里持续 `poll()` 统计 `ArmState`、`HandState`、`Diagnostics`），核对：
   - **链路质量**：两板 `ArmState` 实际频率 ≈ 500 Hz，记录接收间隔 p50/p99 与 `missing` 序号缺口；`Diagnostics` 中 CAN 错误计数为 0、`fault_code_raw` 全 0。
   - **整圈回绕轴号与回零分支**：记录两板在 `HOME_HOLD` 下的原始 `q_rad[0..5]`，按 [session.cpp](../src/session.cpp#L42) 与固件 `home_path.c` 的 `branch_reference = [0, π/2, 2.886410263, 0.255182391, 0, 0]` 核对：
     - 扣除 `zero[j] = 2π·floor((q_raw[j] − branch[j]) / 2π + 0.5)` 后，两板 6 轴逻辑角度应接近 `[0, 0, 0, 0, 0, 0]`（误差 < 2°）。
     - 特别核对历史上存在整圈偏移的 **J2**（旧配置 1-based `[2]`）及全部 6 轴在断电重启后是否稳定落入一致分支。
4. **通过标准**：两板 500 Hz 连续 60 s 无故障、无重启、丢包率 < 0.1%、扣除整圈偏移后零位对齐。

---

## Stage 2：接管、零位抬臂与保护路径验证

在启动运动控制前，先调低 `ControlConfig` 速度/加速度上限（建议首次将 `max_speed` 缩至 0.25 rad/s、`translation_speed` 缩至 0.05 m/s）：

1. **接管与静止保持**：调用 `DualArm::enable()`，确认两板均进入主机控制（`control_owner == 1`），无 `-16` 报错、未触发硬停（验证零位下 `link2↔link5` 与 `link4↔palm_base_link` 的分层阈值与分隔证明生效）。保持 10 s 观察关节抖动与重力前馈是否平衡。
2. **零位抬臂**：发联合关节目标将两臂从 `zero` 平稳抬至 `ready`（`[0, 1.0, 1.3, -0.5, 0, 0]`）或 `home_waypoint`，再回 `zero`。观察 `state().clearance` 与 `state().hard_margin` 全程为正且无 QP 失败。
3. **柔顺软停**：在 `ready` 姿态调用 `softStop()`，确认两臂柔和保持在当前位姿不跌落（板端 `HOME_SOFT_HOLD`），随后可再次 `enable()` 恢复接管。
4. **单向断链 / 看门狗验证**：如担心异常掉电下坠，须在**试验前**于双臂下方预置机械防坠支撑或缓冲垫，**试验期间人员退至工作区外（禁止徒手托扶）**。在低速保持状态断开一侧网线或丢弃一侧上行 UDP，按 [PLAN.md §3.1](PLAN.md#L69) 分别核对与记录：
   - 主机侧：反馈年龄达 **10 ms** 时双臂冻结参考（`frozen=True`）；达 **50 ms** 时主机停发两臂运动命令并向仍持有有效会话的两板请求 `op18`。
   - 板端侧：在线侧确认进入 `HOME_SOFT_HOLD`（若存在反馈失效/电机故障则按固件进入 `HOME_FAULT`）；失联侧在冻结期仍收到保持帧会刷新板端 `arm_at`，下行仍通时可直接收到 op18；若 op18 未送达且板仍处于主机控制，看门狗自最后接受的保持帧计满 50 ms 触发，主机缺少该侧回包时 CLI 记为 **“未确认”**（随后人工检查指示灯或恢复链路复核）。
5. **通过标准**：零位接管与抬臂零误停；软停不掉臂；单侧断链时主机按 10 ms 冻结 / 50 ms 停发并请求 `op18`，在线侧确认软停、失联侧停机结果经复核正常。

---

## Stage 3：单臂单关节小幅运动与装配核对

在 `ready` 姿态下，**每次只动单臂单轴 ±5°–10°**（左臂 6 轴、右臂 6 轴逐一检查）：

1. 核对每轴正方向与 URDF 一致（J1 左/右旋向、J2/J3 抬落方向、J4/J5/J6 腕部方向）。
2. 核对静态跟踪误差与重力前馈方向（若某轴明显下坠或上漂，记录 `tau_measured_nm` vs `g(q)` 偏差，留待 WP-H 后的重力重辨识）。
3. **重点核对 J5 与腕部间隙**：在 `ready` 下将 J5 缓慢摆至软限位边缘（默认 ±60°），目视确认 `link4` 与 `palm_base_link` / 实际转接件之间存在清晰安全余量，并实测转接件实际厚度与质量记录到验收单（用于后续更新 D5 模型）。
4. **通过标准**：12 个关节方向全部正确、无异响与超调、腕部在 J5 ±60° 内无干涉。

---

## Stage 4：灵巧手映射符号与机械端点行程核对

在两臂静止于安全姿态下，逐手、逐指测试 `setHandTarget(side, q6, duration_ms)`（`duration_ms` 取较大值如 1000 ms，动作缓慢）：

1. 按固件顺序 `[thumb_cmc, thumb_pip, index_mcp, middle_mcp, ring_mcp, little_mcp]` 逐个给小幅目标，核对：
   - 通道顺序是否与实际手指一一对应；
   - 零位（完全张开）与弯曲方向（符号）是否与 URDF 一致；
   - `active_hand_angles()` 反馈与目标的比例关系是否一致。
2. `stage4-hand --real` 按指等待操作者输入。`+` 将目标增加最多 0.1 rad，限于对应 URDF 主动关节上限；`end` 记录最新实测角并张开。确认方向后逐步核对物理端点与模型行程：
   - 确认物理包络完全落在 [PLAN.md §3.3](PLAN.md#L100) 的 URDF 主动关节全行程包络内（若实机超出模型行程或符号相反，记录修映射参数，禁止进入双臂近距测试）。
3. **通过标准**：左右手 12 通道顺序/符号/比例明确且实机扫掠包络被模型覆盖。

---

## Stage 5：双臂笛卡尔移动与靠近自停

在工作空间中央空旷区执行：

1. **笛卡尔跟踪**：两臂分别执行 ±50 mm 直线移动与 ±15° 姿态转动，核对 `state().tcp` 反馈方向与世界坐标系约定（+x 前、+y 左、+z 上）一致，记录稳态误差与 `qp_time`。
2. **双臂对向靠近自停**：初始拉开至 > 150 mm，以低速（如 `translation_speed = 0.05 m/s`）命令两臂 TCP 朝中平面 `y = 0` 对向穿越。
   - 观察随着臂间距离进入 `d_i = 100 mm`，双臂自动减速并稳定停在 **`d_s = 20 mm`** 附近（不碰撞、不震荡、不触发误硬停）。
   - 从运动包络外记录双手停止姿态，使用非接触测量或在驱动安全隔离并机械固定后实测两手最近部位物理间隙，核对模型报告的臂间距离 `collision_state(...).inter_arm` 与实测间隙的偏差（注意 `ControlState.clearance` 含同臂近距对，不直接代表臂间距离；确认 D5 零厚度转接件和手包络余量带来的偏置方向是否保守）。
3. **通过标准**：世界坐标方向正确；对向指令下稳定自停在安全距离外，物理无接触且模型间隙保守。

---

## Stage 6：1 小时连续稳定性拷机

1. 在安全工作空间内运行缓慢往复双臂轨迹（含低速避碰边界往返与周期性手部开合）连续 **60 分钟**，全程落盘统计。
2. **通过标准**：
   - 0 次非预期软停、0 次板端重启、0 次电机/CAN 故障；
   - 控制环 `missed_cycles` 占比 < 0.1%，`qp_time` p99 < 100 µs；
   - 通信无持续丢包或延迟爬升；电机与控制板温升正常。

---

## 验收归档清单（`validation/<YYYYMMDD>/`）

验收完成后在 `validation/<YYYYMMDD>/report.md` 汇总以下内容并提交：

- 软硬件版本：本仓库 commit SHA、左右固件 `SHA256`、板端刷写备份与 JSON 报告。
- Stage 1–6 逐项通过结果与实测数据（500 Hz 周期/丢包、两板零位 `q_raw` 与整圈偏移、J5 腕部实测间隙、转接件实测厚度/质量、手部 6 通道符号/端点、对向自停实测物理间隙 vs 模型 `inter_arm`、1 小时拷机统计）。
- 发现的偏差与后续待办（例如重力重辨识输入、TCP 实测偏置、D5 转接件参数修正）。
