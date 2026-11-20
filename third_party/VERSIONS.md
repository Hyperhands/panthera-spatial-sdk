# 主机依赖

以下提交由 SDK 的 Git 子模块固定。使用 `git submodule update --init --recursive` 检出。

| 组件 | 版本 | Git 提交 | 许可 |
| --- | --- | --- | --- |
| wirelink | 固件 11.20 的协议核心 | `608551b3caac52fe498ed6a677ddd6392a793dd8` | ISC |
| WLC | v0.8.0 | `758d12a2466e254cb4d913fce578fb7deae766ec` | 此提交的仓库根目录及 Cargo.toml 未声明编译器许可；模板附带各自的声明 |
| acados | v0.6.1 | `de42a54bf31d95bcea907ebc280b6ec678c7bd36` | BSD-2-Clause |
| HPIPM（acados 子模块） | 随 v0.6.1 | `bad1123a96136fce3cdd8bbcbe24b853d012b840` | BSD-2-Clause |
| BLASFEO（acados 子模块） | 随 v0.6.1 | `0a3df8683c6bf9d98168d616e0c5d05938c9d51c` | BSD-2-Clause |
| Eigen | 5.0.1 | `bc3b39870ecb690a623a3f49149a358b95c5781d` | 主要为 MPL-2.0，组件许可见 COPYING.README |
| pybind11 | v3.1.0 | `97bf890db679505a14dfe547a5e77bb2bd05dc90` | BSD-3-Clause |

`protocol/` 与 `generated/protocol/` 来自 dual-gello 提交
`49981be63a0aeccd9d2a5d52249cf8e84de17718` 的 `source/protocol/` 与
`source/src/generated/`。WLC 只在重新生成时使用。CTest 检查固件 wirelink 的
`include/wirelink` 和 `src` 的 Git 树哈希，并按 WLC 清单逐文件检查生成物的大小与摘要。Linux CI 使用固定版本的 WLC 和
`board_rt.bind.wl` 再生成并逐字节比对。`value_init.h` 是固件提供的初始化辅助头。
