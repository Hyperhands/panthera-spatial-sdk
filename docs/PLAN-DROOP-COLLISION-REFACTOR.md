# 重构计划：零位保持参考防漂移与双臂 `link2 ↔ link5` 距离估计修正（v2.0 收敛版）

本文根据实机回放（`build/hardware-validation/20261121/stage2-j2/stage2-j2-hold-123943-618313.json`）与离线网格核查意见，对 v1.0 计划做收敛修订，作为执行 agent 的唯一计划来源。

> **硬性边界（不可突破）**
> 1. **各碰撞对保留原硬停阈值**（不放宽同臂硬停门限，不引入 `-8 mm` 容差，不引入累积退让的预测棘轮）；`observe()` 与 `candidate_` 维持严格的 `hard_margin >= 0`。
> 2. **双侧 J2 实测下限沿用已确认的 `-1°`**，URDF 与正常运动目标下限保留 `0°`（不扩大到 J3）。
> 3. **不修改固件镜像**（`firmware/images/{left,right}.bin` 保持不变），**不修改 SRDF 碰撞矩阵**，**不新增启动/脱离业务阶段**。
> 4. **离线验收全绿 → 提交推送 → 盯 CI → 现场仅执行双臂零位保持 10 秒**；抬臂待负责人现场确认稳定后再进行。

---

## 1. 实机故障与几何根因（已用现场日志与网格复核）

1. **故障一（上次 Stage 2 保持 4.046 秒中断的直接原因）：关节目标控制律存在持续静差下的参考积分漂移**
   - 现场记录 `stage2-j2-hold-123943-618313.json` 中，最小硬停裕度全程为正（**+0.365 mm**），停机原因为 `Five consecutive QP failures`。
   - 根因在 [src/control_law.cpp](../src/control_law.cpp#L96) 与 [L100-L102](../src/control_law.cpp#L100-L102)：关节目标项用 `t.joint - measured`、姿态零空间项用 `nominal_ - measured` 生成速度，随后在 `x1 = state.reference + dt * u` 上积分。
   - 实机左 J4 存在约 `0.115°` 的持续重力/静摩擦静差，导致 `state.reference` 持续正向漂移，第 1620 拍达到 `reference - measured = 0.050096 rad > max_lead (0.05 rad)`；此时 `max_lead` 要求 `vhi <= -0.04794 rad/s`，而加速度下界要求 `vlo >= -0.008 rad/s`，[src/motion_qp.cpp](../src/motion_qp.cpp#L87-L90) 在进入求解器前即判无解，连败 5 拍软停。
2. **故障二（左右 `link2 ↔ link5` 距离估计在近零非接触姿态下过度保守且左右不对称）**
   - 离线 Pinocchio/coal 真实 STL 网格距离核查表明，现场实测近零姿态本身**并不相交**：
     - 现场左臂起始态 `[-0.406, -0.049, 0.857, -1.203, -0.352, 0.566]°`：真实网格间距 **+1.661 mm**；
     - 现场左臂软停后 `J4 = -1.489°`：真实网格间距 **+1.327 mm**；
     - 现场右臂起始态 `[-0.286, -0.022, 1.527, -0.989, -0.286, 0.483]°`：真实网格间距 **+4.307 mm**；
     - 而真正相交的姿态（如 `J2 = -0.5°, J3 = 0°, J4 = -2.5°, J5 = ±1°` 或 `J3 = 0°, J4 = -1°, J5 = 0°`）：真实网格间距为 **0.000 mm**，必须被硬停拦截。
   - 当前 [tools/codegen/capsules.py](../tools/codegen/capsules.py#L239-L299) 仅在名义 `zero`/`lift` 构造分隔证明，且 `427dd09` 只为左臂单点硬编码了 1 组额外 witness，导致右臂在同等姿态下仍误报 `-0.423 mm`，左臂 J4 再垂 `0.3°` 即误报负裕度。
3. **关联清理（`startup_pairs_` 状态冗余）**
   - 当某同臂近距对初始 `d >= safe`（未进入 `startup_pairs_`）、保持过程中因微小静差跌至 `hard < d < safe` 时，[src/geometry.cpp](../src/geometry.cpp#L136) 会给出正的 `row.lower`，迫使保持态产生推离速度。将同臂 CBF 下界统一截断至 `<= 0` 即可消除该行为并删掉 `startup_pairs_`，同时不改变任何硬停阈值。

---

## 2. 重构顺序与逐文件改动清单

### 优先级 1：修复静止保持参考漂移（使参考在持续静差下严格有界）

- **[src/control_law.cpp](../src/control_law.cpp)**
  1. 将关节目标误差与零空间姿态项从基于 `measured` 改为基于 `state.reference`（[L96](../src/control_law.cpp#L96)、[L102](../src/control_law.cpp#L102)）：
     ```cpp
     JointVector linear = -smoothing * state.velocity - posture * (nominal_ - state.reference);
     ...
     if (t.type == TargetType::joint) {
         h.block<6, 6>(index, index).diagonal().array() += 1;
         linear.segment<6>(index) -=
             config_.joint_gain * (t.joint - state.reference.segment<6>(index));
     }
     ```
     - **性质**：当保持目标 `t.joint == state.reference == nominal_` 时，无论 `measured` 存在多大持续静态偏差（只要在 `max_lead` 与硬限位内），期望速度恒为 `0`，`state.reference` 永不积分漂移；切换新关节目标时，`state.reference` 按一阶速率平滑收敛至 `t.joint` 并受 `max_speed`、`max_acceleration`、`max_lead` 与碰撞约束保护。
     - **不变量保持**：不改动 [src/motion_qp.cpp](../src/motion_qp.cpp) 的 `max_lead` 与加减速约束语义；不改动 `TargetType::pose` 的笛卡尔闭环逻辑；保留已确认的 `j ∈ {1, 7}`（双侧 J2）`-1°` 实测下限与 `step_bounds.lower[j] = std::min(0., state.reference[j])`。
  2. 清理 `startup_pairs_`：
     - 在 **[src/geometry.cpp](../src/geometry.cpp#L134-L136)** `add_row` 中对同臂行截断正下界：
       ```cpp
       double lower = -.5 * margin / (influence - safe);
       if (a.side == b.side)
           lower = std::min(0., lower);
       state.rows.push_back({gradient, lower, margin, id, a.side == b.side});
       ```
     - 在 **[src/control_law.hpp](../src/control_law.hpp#L20)** 与 **[src/control_law.cpp](../src/control_law.cpp#L75-L78)**、[L126-L133](../src/control_law.cpp#L126-L133) 中删除 `startup_pairs_`。
     - **不改变任何硬停逻辑**：`observe()` 仍严格要求 `current_.hard_margin >= 0`，`step()` 仍严格要求 `candidate_.hard_margin >= 0`。

### 优先级 2：对称改进左右 `link2 ↔ link5` 的分隔证明（零硬编码现场点、零放松阈值）

- **[tools/codegen/capsules.py](../tools/codegen/capsules.py)**
  1. **删除** `427dd09` 引入的左臂单点现场测量硬编码（[L239-L244](../tools/codegen/capsules.py#L239-L244) 与 [L288-L299](../tools/codegen/capsules.py#L288-L299)）。
  2. **保留原有四姿态阈值计算不变**：`hard`、`safe`、`capsule_hard` 仍**仅**由原有的 `zero / ready / home_waypoint / lift` 四姿态决定，**绝不**用新增网格点去放宽任何阈值。
  3. 对**左右两侧**的 `*_link2 ↔ *_link5`（对称处理 `side in SIDES`，对应 pair 30 与 pair 284），在近零 `(J3, J4, J5)` 局部网格上仅对**URDF 真实碰撞网格明确分离**的姿态做贪心补充分隔证明（每组 witness 均为任意刚体变换下严格成立的真实网格间距数学下界，不可能对相交网格给出正值）：
     - 采样范围（仅依赖 URDF 几何，与具体设备现场读数无关）：
       `J3 ∈ {0.5°, 1.0°, 1.5°, 2.0°, 3.0°}`、`J4 ∈ {-2.5°, -2.0°, -1.5°, -1.0°, -0.5°, 0.0°}`、`J5 ∈ {-1.5°, 0.0°, +1.5°}`；
     - 筛选条件：仅对真实三角网格间距 `d_true >= 0.5 mm`（避开临界接触）且当前已有证明下界 `< max(0.2 mm, 0.35 * d_true)` 的格点调用 `certify(...)`，若新证明显著提升下界（`> cur + 0.1 mm`）则加入 `proofs`；
     - **按 witness 数量升序排列 `proofs`**（`proofs.sort(key=len)`）：运行时 [src/geometry.cpp](../src/geometry.cpp#L52-L70) 先命中几十个 witness 的小组并迅速拉高 `best.distance`，使 450 个 witness 的 `zero` 组在首条即触发 `if (gap <= best.distance) break;` 早退，进一步压缩运行开销。
  4. 使用 `build/codegen-env/bin/python tools/codegen/capsules.py` 重新生成 [generated/model/geometry.inc](../generated/model/geometry.inc) 与 [generated/model/capsules.json](../generated/model/capsules.json)。
     - **已验证离线预期**（实测仅新增 3 组/侧，约 184 条 witness/侧，左右完全对称）：
       - `nominal zero`：真实 `0.204 mm`，认证 `+0.084 mm`，连杆对认证距离减去其硬停阈值为 `+0.050 mm`；
       - 现场左臂起始态：真实 `1.661 mm`，认证 **`+1.203 mm`**，连杆对认证距离减去其硬停阈值为 `+1.169 mm`（该连杆对的认证距离左右对称）；
       - 现场左臂软停后（`J4 = -1.489°`）：真实 `1.327 mm`，认证 **`+0.923 mm`**，连杆对认证距离减去其硬停阈值为 `+0.890 mm`；
       - 现场右臂起始态：真实 `4.307 mm`，认证 **`+3.413 mm`**，连杆对认证距离减去其硬停阈值为 `+3.380 mm`；
       - 网格相交态 `J2=-0.5°, J3=0°, J4=-2.5°, J5=±1°`：认证 **`-3.50 / -3.43 mm`**，`hard_margin < 0`（严格硬停）；
       - 网格相交态 `J3=0°, J4=-1°, J5=0°` 与 `J2=-0.5°, J3=0°, J4=0°, J5=+2°`：认证均为负，`hard_margin < 0`（严格硬停）。

### 优先级 3（独立可选，待负责人确认后再决定是否同批合入）：主机 J4 `kp/kd` 与固件对齐
- 现场记录显示：离开固件 `HOME_HOLD`（`kp[3]=36.72, kd[3]=0.831384`）由主机接管（`kp[3]=12.24, kd[3]=0.48`）后 95 ms 内，左 J4 由 `-1.203°` 下垂至 `-1.318°`。
- **执行原则**：**不要用调高刚度掩盖漂移问题**。默认本轮先不改 [src/session.cpp](../src/session.cpp#L40-L41) 增益，确保在现有 `kp[3]=12.24` 与实测静差下即可稳定保持 10 秒；若负责人明确确认同步对齐固件 J4 增益，再单独提交并在 `mujoco_mocap_qp.py --headless` 回归确认无数值发散。

---

## 3. 离线测试与验证矩阵（必须全部通过后再提交推送）

### 3.1 新增/更新的单元测试

1. **[tests/geometry.cpp](../tests/geometry.cpp)**（覆盖“现场姿态误报消除 + 左右对称 + 网格相交硬停”）：
   - 现场左臂起始态 `[-0.406, -0.049, 0.857, -1.203, -0.352, 0.566]°`、软停后态 `[-0.406, -0.055, 0.857, -1.489, -0.423, 0.571]°`、现场右臂起始态 `[-0.286, -0.022, 1.527, -0.989, -0.286, 0.483]°`，分别置于**左臂与右臂**，验证两侧全局 `c.hard_margin > 0`，并检查该连杆对的认证距离左右一致（全局裕度还受其他胶囊约束影响）；
   - 明确校验网格相交姿态**稳定触发硬停（`c.hard_margin < 0`）**（左右两侧均测）：
     - `J2 = -0.5°, J3 = 0°, J4 = -2.5°, J5 = +1°`
     - `J2 = -0.5°, J3 = 0°, J4 = -2.5°, J5 = -1°`
     - `J2 = -0.5°, J3 = 0°, J4 = 0°, J5 = +2°`
     - `J2 = 0°, J3 = 0°, J4 = -1°, J5 = 0°`
   - 保留 `zero`、`5° lift`、`ready`、`home_waypoint` 全部原有检查。

2. **[tests/control_law.cpp](../tests/control_law.cpp)**（覆盖“持续静差下参考有界 + 现场轨迹回放 10 秒 + 网格相交硬停”）：
   - **持续静态偏差 10 秒保持（5000 拍）**：以现场左/右近零姿态 `reset(q0)` 并设保持目标 `t.joint = q0`，在全部 5000 拍中对 `measured` 施加持续非零静态下垂（如左 J4 恒定低于 `q0` `0.15°` 与 `0.30°`——同时保持 `hard_margin > 0`），验证：
     - 全程零 QP 失败、零 `frozen`、零异常；
     - `(law.state.reference - q0).lpNorm<Infinity>() < 1e-4`（参考严格有界、不漂向 `max_lead`）。
   - **现场记录轨迹离线回放 10 秒**：按 `stage2-j2-hold-123943-618313.json` 中实测的 `q_deg` 演进特征（`t=0 → 0.1s` 左 J4 从 `-1.203°` 垂至 `-1.318°` 并叠加 `±0.0055°` 编码器量化跳动，右 J2 从 `-0.022°` 移至 `+0.088°`、右 J4 从 `-0.989°` 垂至 `-1.044°`，持续 5000 拍），验证零 QP 失败且 `|reference - measured|` 远小于 `max_lead`。
   - **网格相交硬停**：向 `ControlLaw::reset()` / `step()` 喂入上述 `J2 = -0.5°, J3 = 0°, J4 = -2.5°, J5 = ±1°`（左右两侧各测），验证必抛 `Measured collision hard threshold`。
   - **既有回归全保留**：`check_j2_takeover`（含 J2 `-1°` 接受、`-1.01°` 拒绝、J3 `-0.05°` 拒绝、显式收紧限位不被放宽）、零位抬臂、双臂对向接近停在 `>= 20 mm`、3000 步随机目标、`lag` 5 拍 QP 失败软停、`Trajectory solver p99 < 100 us`。

### 3.2 完整离线验证命令

```bash
# 1. 重新生成对称几何表并检查仅预期文件变化
build/codegen-env/bin/python tools/codegen/capsules.py

# 2. C++ 构建与全套 CTest
cmake --preset host
cmake --build build/host
ctest --test-dir build/host --output-on-failure

# 3. 重点检查 QP 时延与控制律回归
ctest --test-dir build/host -V -R 'geometry|motion_qp|control_law|sim_controller'

# 4. 重建 Python 绑定并跑 Python 集成测试与 WP-H 烟测
cmake -S . -B build -DPANTHERA_BUILD_PYTHON=ON -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release -DPython_EXECUTABLE="$PWD/build/phase2-env/bin/python"
cmake --build build --target _core
PYTHONPATH=build/python build/phase2-env/bin/python -m unittest tests/test_python_sdk.py tests/test_python_examples.py
for stage in stage1-telemetry stage2-takeover stage3-single-joint stage4-hand stage5-approach stage6-soak; do
  PYTHONPATH=build/python build/phase2-env/bin/python examples/wp_h_accept.py "$stage" --smoke
done

# 5. MuJoCo 无头闭环回归
PYTHONPATH=build/python build/phase2-env/bin/python examples/mujoco_mocap_qp.py --headless
PYTHONPATH=build/python build/phase2-env/bin/python examples/mujoco_mocap_qp.py --headless --start-zero

# 6. 校验固件摘要未动
(cd firmware/images && sha256sum -c SHA256SUMS)
```

---

## 4. 交付与现场推进门禁

1. 离线三项核心验收（**现场姿态可保持、网格相交会硬停、持续静差下参考有界**）及第 3.2 节全部命令全绿；
2. 提交并推送至远端，盯紧 GitHub Actions CI（Linux x86_64 + Windows x64 wheel 与 CTest）全绿；
3. 现场先仅执行**双臂零位保持 10 秒**验收并归档日志，确认无漂移、无 QP 失败、无误硬停、CAN 正常后，再由负责人确认进入 Stage 2 抬臂步骤。

> 独立后续备忘（不混入本次重构）：离线已观测到 Stage 3 腕部 `link4 ↔ palm_base_link`（pair 46/332）及 `link4 ↔ thumb`（pair 57）在 J5/J6 摆动时受单姿态 witness / 扫掠包络影响会触发硬停，留待 Stage 2 完成后在进入 Stage 3 前独立处理。
