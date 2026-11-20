# panthera-spatial-sdk

Panthera-HT 双臂 + A2/L1 灵巧手的上位机空间控制 SDK。设计目标：
- 500 Hz 的 acados QP（HPIPM）任务空间控制，带双臂自碰撞约束；
- C++ 核心 + pybind11 Python 接口；
- 固件、wirelink、WLC 及外部依赖全部以源码固化在本仓库。

> 状态：已接入主机构建、协议编解码器和 HPIPM，双臂模型（M0）已完成。工作包与验收条件见 [docs/PLAN.md](docs/PLAN.md)。

| 目录 | 内容 | 状态 |
| --- | --- | --- |
| [docs/PLAN.md](docs/PLAN.md) | 总体计划、架构、阶段与待确认问题 | ✅ |
| [model/panthera_dual_description](model/panthera_dual_description/README.md) | 整机 URDF/SRDF、网格、装配参数、校验 | ✅ |
| [tools/model](tools/model) | 模型生成（`build_description.py`）与校验（`verify_description.py`） | ✅ |
| [third_party/](third_party/VERSIONS.md)、[protocol/](protocol)、[generated/protocol/](generated/protocol) | 固定依赖、板端协议和编解码器 | 已接入主机构建 |
| `firmware/`、`src/`、`python/`、`sim/` | 固件、控制器、Python 接口与模拟板 | 待实现 |

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

## 模型

```bash
# 重新生成并校验模型（需要 numpy、PyYAML、pinocchio>=3.4、matplotlib）
python3 tools/model/build_description.py
python3 tools/model/verify_description.py
```
