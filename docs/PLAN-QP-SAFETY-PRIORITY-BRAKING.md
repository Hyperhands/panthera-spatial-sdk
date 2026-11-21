# 重构计划：QP 约束分级与安全优先最小急刹回退（v1.0）

本文针对实机推进 Stage 2 中连续暴露的同类 QP 无解冻结问题（`stage2-j2-hold` 第 1620 拍、`stage2-ready-retest` 第 139 拍、`stage2-return-hold` 第 679 拍以及零位抬臂仿真第 24 拍），给出**一次性根治**的精简方案，作为执行 agent 的唯一计划来源。

> **硬性边界（不可突破）**
> 1. **零放松安全红线**：关节位置限位 `[lower, upper]`、速度上限 `[-max_speed, +max_speed]`、跟踪超前窗 `max_lead`、限位减速 `slow/fast`、全部碰撞速度阻尼行 `g^T u >= lower`、以及 `ControlLaw` 的 `hard_margin >= 0` 和 `tracking_stop_rad` **全部保持严格硬约束**。
> 2. **零新增业务阶段、零新增 HPIPM 维度、零改公有头/ABI**：继续沿用现有 `ng = 16, ns = 0` 与 `warm_start = 0, t0_init = 0, split_step = 1`，求解 `p99` 保持在 ~35 µs（远低于 100 µs 目标），不改固件、不改 SRDF。
> 3. **正常运动零行为变化**：只要平滑加速度盒 `previous ± a_max * dt` 与安全约束有交集，第 1 拍（正常分支）直接返回与现有代码完全一致的解。

---

## 1. 根因总结（为什么连续在不同地方触发“上限无解”）

查看 [src/motion_qp.cpp](../src/motion_qp.cpp#L74-L112)，当前 QP 把**运动平滑约束**（`|u - previous| <= a_max * dt`）与三类**基于外部实测反馈、天然非光滑的一阶安全约束**硬求交集：
1. **碰撞速度阻尼** `g^T u >= lower`（`certificate()` 多分隔平面 `max-min` 切换或 `raw < 0.1` 切换时，法向与下界存在分段跳变；如 `stage2-return-hold` 第 679 拍 `right J3` 仅动了 `0.02°`、`margin = +28.0 mm`，下界跳到 `-0.1415 m/s` 而加速度盒只能降到 `-0.1672 m/s`）；
2. **跟踪超前窗** `|reference + dt*u - measured| <= max_lead`（如第 139 拍起步滞后；当前仅在 `TargetType::joint` 做了提前减速，进入 Stage 5 的 `pose / twist` 仍会重现）；
3. **关节限位线性阻尼** `slow / fast = 8 * (limit - measured)`（高速接近限位且叠加实测抖动时斜率逼近 `a_max`）。

**优先级倒置后果**：只要上述任一非光滑安全约束在单拍内多要一点点减速度，QP 直接判无解 → `ControlLaw::freeze()` **一拍把速度强行清零**（第 679 拍等效 `337 rad/s²` 急刹冲击，比放宽平滑上限猛 **6 倍**）并导致验收脚本整轮中止。

---

## 2. 方案设计：两段式安全优先投影（仅改 `src/motion_qp.cpp` 核心实现）

> 离线已对比过“把加速度放进 acados `ng=28, ns=12` 软约束”与“现有 `ng=16, ns=0` + 仅在冲突时触发安全优先最小急刹回退”两种方案：
> - 增大 `ng/ns` 会使 `p99` 涨到 `121–194 µs`（超过 100 µs 预算）且引入罚权量级与 `idxs_rev` 复杂度；
> - **安全优先回退方案**零额外维度、零魔数罚权、正常帧 `p99 ≈ 34 µs`，并在第 679 拍、第 139 拍、5,000 组压力测试和全往返 `Zero <-> Ready` 均实现 **0 失败、0 冻结**。

### 2.1 数学定义

将 `MotionQp::solve` 的速度盒拆为两层：

1. **安全硬盒（Tier 0，绝不松弛）**：
   ```cpp
   JointVector lo = limits.lower.cwiseMax(measured - limits.max_lead);
   JointVector hi = limits.upper.cwiseMin(measured + limits.max_lead);
   JointVector slow = (8 * (limits.lower - measured)).cwiseMin(0);
   JointVector fast = (8 * (limits.upper - measured)).cwiseMax(0);
   JointVector safe_lo = (-limits.max_speed).cwiseMax(slow).cwiseMax((lo - reference) / control_dt);
   JointVector safe_hi = limits.max_speed.cwiseMin(fast).cwiseMin((hi - reference) / control_dt);
   ```
   - 若 `lo > hi` 或 `safe_lo > safe_hi`（例如实测严重跳变超限、或超过 `max_lead + v_max * dt` 的真实滞后故障），立即返回失败，**完整保留既有 `lag` 滞后故障检测**。

2. **Pass 1（正常平滑分支，覆盖 99.9%+ 周期）**：
   - 将安全硬盒与平滑加速度盒求交：
     ```cpp
     JointVector acc_lo = previous - control_dt * limits.max_acceleration;
     JointVector acc_hi = previous + control_dt * limits.max_acceleration;
     JointVector vlo = safe_lo.cwiseMax(acc_lo);
     JointVector vhi = safe_hi.cwiseMin(acc_hi);
     ```
   - 调用内部辅助方法 `solve_box(reference, lo, hi, vlo, vhi, hessian, linear, rows, max_solves, start, result)`。
   - 若成功直接返回（沿用现有目标与平滑盒，不增加 HPIPM 调用）。

3. **Pass 2（安全优先最小急刹回退，仅当 Pass 1 因平滑盒冲突失败且仍在 `2 ms` 预算和 `max_solves` 额度内时触发）**：
   - 仅释放 `acc_lo / acc_hi` 平滑盒，**所有硬安全约束（`safe_lo / safe_hi` 与全部碰撞行 `rows`）保持 100% 硬满足**；
   - 目标函数切为**纯最小加速度欧氏投影**（条件数恒为 `1.0`，零待调罚权）：
     $$\min_{u}\ \frac{1}{2}\|u - u_{\text{prev}}\|^2 \quad \text{s.t.}\quad u \in [\text{safe\_lo}, \text{safe\_hi}],\quad g_i^\top u \ge l_i\ (\forall i)$$
     即 `brake_h = JointMatrix::Identity()`、`brake_lin = -previous`；
   - 关键性质：
     - **无关轴**：上一拍速度在硬盒内且不参与任何起作用的碰撞行时，投影保持该轴速度。第 679 拍完整回放中，另一条左臂碰撞行也要求左 J2/J3 分别调整 +0.000334/+0.009196 rad/s，均在常规单拍 0.016 rad/s 内；只有右 J3/J4 超出常规加速度盒。
     - **最小急刹**：冲突轴（如第 679 拍的右臂 J3/J4/J5）沿约束法向给出满足碰撞阻尼的数学最小速度修正（实测第 679 拍 `du = [+0.108, +0.034, +0.003] rad/s`，比 `freeze()` 的 `+0.674 rad/s` 温和 **6 倍**，碰撞残差 `col_slack = +8.4e-13 >= 0`）；
     - **全模式统一生效**：`joint`、`pose`、`twist` 三种模式下的 `collision`、`max_lead`、`slow/fast` 与 `a_max` 冲突全部一次性覆盖；
     - **预算严格受控**：传入 `max_solves - result.solves` 并共享同一个 `start` 2 ms 截止计时，绝不突破总调用次数与实时预算。

> **说明（关于 `control_law.cpp:105-119` 既有的 `joint` 模式 `max_lead` 提前减速）**：保留即可。它负责在常规 `joint` 运动中提前平稳减速（让绝大多数拍在 Pass 1 就通过，连 Pass 2 都无需触发）；Pass 2 则作为全模式、全约束的统一安全兜底。

---

## 3. 逐文件改动清单

### Step 1：重构 [src/motion_qp.cpp](../src/motion_qp.cpp)
- 在 `MotionQp::Impl` 中抽出私有辅助函数 `bool solve_box(...)`，封装：
  - `vlo > vhi` 检查；
  - 碰撞行活跃过滤与最坏/最优角点预判（`at_lo.cwiseMax(at_hi).sum() < row.lower - tolerance` 直接返回 `false`，不浪费 HPIPM 调用，让调用方不调用 HPIPM 即可切入 Pass 2）；
  - 静止最优解 `u = 0` 快路径（当 `linear.isZero(0.)` 且 `0` 满足盒与全部行时直接成功）；
  - 16 槽 HPIPM 求解 + 全对复核与违约换入循环（严格遵守传入的 `remaining_solves` 与 `2 ms` 截止时间）。
- 在 `MotionQp::solve(...)` 中：
  1. 计算 `lo, hi, slow, fast, safe_lo, safe_hi` 并做有限值、`lo <= hi`、`safe_lo <= safe_hi` 及 `rows` 有限值检查；
  2. Pass 1 用 `vlo = safe_lo.cwiseMax(acc_lo)`、`vhi = safe_hi.cwiseMin(acc_hi)` 调用 `solve_box`；
  3. 若空盒、碰撞行最优角点或碰撞行的非负线性组合证明平滑盒无解，且未超 `2 ms` 且 `result.solves < max_solves`，执行 Pass 2。超时、未证实无解的求解错误和非有限结果继续走冻结路径。以 `safe_lo, safe_hi` 与 `JointMatrix::Identity(), -previous` 调用 `solve_box(..., max_solves - result.solves, start, result)`。

### Step 2：补齐专项回归测试
- **[tests/motion_qp.cpp](../tests/motion_qp.cpp)**
  1. **碰撞阻尼 vs 加速度冲突（复现第 679 拍）**：构造 `previous` 速度在加速度盒内最优角点仍不满足碰撞行（`best < lower`）、但在 `safe_lo..safe_hi` 内可行，断言：
     - `result.success == true`；
     - 所有碰撞行严格满足 `g.dot(u) >= lower - 1e-7`；
     - 未参与冲突的无关轴（`g[j] == 0`）满足 `|u[j] - previous[j]| < 1e-7`（零牵连）；
     - 冲突轴确实小于 `freeze` 的急刹幅度（`|u - previous| < |previous|`）。
  2. **`max_lead` vs 加速度冲突（复现第 139 拍）**：`prev[3] = 0.25, a_max = 4, max_lead = 0.05, ref[3] - meas[3] = 0.04984`（`safe_hi[3] = 0.08 < acc_lo[3] = 0.242`），断言：
     - `result.success == true`；
     - `u[3] <= 0.08 + 1e-7` 且 `x1[3] - meas[3] <= 0.05 + 1e-7`；
     - 其余无关轴保持 `previous`。
  3. **不可恢复大滞后仍正确失败**：当 `ref[0] - meas[0] = max_lead + 0.02`（要求回拉速度 `10 rad/s > max_speed`）时，断言 `result.success == false`（验证 `safe_lo > safe_hi` 保护未退化）。
  4. **正常可行情况严格保持 `a_max`**：在有解样本上验证 `|u - previous| <= a_max * dt + 1e-7` 与原有 20k 窄盒测试零退化。
- **[tests/control_law.cpp](../tests/control_law.cpp)**
  1. **全速 `Zero -> Ready -> Zero` 往返无冻结**：在默认 `1 rad/s, 8 rad/s²` 下从 `zero` 抬至 `ready`（3000 拍）再回 `zero`（3000 拍），断言全程 `frozen == 0`、`hard_margin >= 0`、`p99 < 100 us`。
  2. **第 679 拍实机状态点闭环测试**：以 `stage2-return-hold` 第 679 拍的真实 `(measured, reference, velocity)` 状态构造 `ControlLaw` 单步与后续回零，断言不冻结且平稳减速进入零位。
  3. **`pose / twist` 模式下的 `max_lead` 保护**：在 `TargetType::twist` 下模拟单轴反馈滞后逼近 `max_lead`，验证通过安全优先回退平滑减速、不因 `a_max` 冲突误冻结。
  4. **既有安全回归通过，折叠姿态的预期结果更新为最小制动**：特别核对既有的 `lag` 5 拍 QP 失败停机测试（`q[0] += max_lead[0] + 0.02`）、`Crossing clearance >= 20 mm`、3000 步随机目标、10 s 持续静差保持全部绿灯。

---

## 4. 离线验收命令与通过判据

执行 agent 实现后，须依次跑通以下全部验证，确认全绿后再提交推送并盯四平台 CI：

```bash
# 1. C++ 构建与全部 CTest（含 motion_qp / control_law / sim_controller 等全部 11 项）
cmake --build build/host
ctest --test-dir build/host --output-on-failure
ctest --test-dir build/host -V -R 'motion_qp|control_law|sim_controller'

# 2. 用实机故障现场数据直接离线复测（确认历史两次故障均 0 冻结）
#    - stage2-return-hold 第 679 拍：确认 ok=1、col_slack >= 0、candidate hard_margin >= 0
#    - stage2-ready-retest 第 139 拍整段日志回放：确认 0 冻结

# 3. 重建 Python 绑定并跑 Python 回归与 WP-H 6 阶段烟测
cmake --build build  # 本工作区启用 Python 绑定的构建树，扩展输出到 build/python
ctest --test-dir build --output-on-failure -R test_python
for stage in stage1-telemetry stage2-takeover stage3-single-joint stage4-hand stage5-approach stage6-soak; do
  PYTHONPATH=build/python build/phase2-env/bin/python examples/wp_h_accept.py "$stage" --smoke
done

# 4. MuJoCo 无头闭环回归（含零位抬臂与对向靠近自停）
PYTHONPATH=build/python build/phase2-env/bin/python examples/mujoco_mocap_qp.py --headless
PYTHONPATH=build/python build/phase2-env/bin/python examples/mujoco_mocap_qp.py --headless --start-zero

# 5. 校验固件镜像未改动
(cd firmware/images && sha256sum -c SHA256SUMS)
```

### 通过判据
- `ctest` 全绿，且 `control_law` `Trajectory solver p99 < 100 µs`；
- `Zero -> Ready -> Zero`（`1 rad/s, 8 rad/s²`）全往返 **0 次冻结**；
- 第 679 拍、第 139 拍复现用例均 **0 失败、0 约束违反**，无关轴加速度 **0 越界**；
- 既有 `lag` 故障注入、臂间对向自停（`>= 20 mm` / MuJoCo `>= 19 mm`）、碰撞硬停（`hard_margin >= 0`）无任何退化；
- 提交推送后四平台 GitHub Actions CI 全绿，再回现场复测 **回零 → 零位保持 10 秒 →（人工确认稳定后）抬臂**。
