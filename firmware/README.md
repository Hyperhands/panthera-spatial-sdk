# MCXN947 host-control firmware

Application baseline: dual-gello 49981be63a0aeccd9d2a5d52249cf8e84de17718. The SDK shares generated protocol sources and the pinned wirelink submodule with the host. Node 11 is left and node 12 is right. Both use board-owned boot homing and the peer link. Standalone GELLO source and overlays are retained.

Use Zephyr SDK 1.0.1, west and Zephyr's Python build dependencies. Initialize a dedicated west workspace and install the pinned modules:

```sh
mkdir -p build/west/manifest
cp firmware/zephyr/west.yml build/west/manifest/
(cd build/west && west init -l manifest && west update)
export ZEPHYR_BASE="$PWD/build/west/zephyr"
git -C "$ZEPHYR_BASE" apply "$PWD/firmware/zephyr/zephyr-network.patch"
export ZEPHYR_SDK_INSTALL_DIR="$HOME/zephyr-sdk-1.0.1"
firmware/build-host.sh left
firmware/build-host.sh right
python3 firmware/tools/test-native.py
(cd firmware/images && sha256sum -c SHA256SUMS)
cmp build/firmware-left/zephyr/zephyr.bin firmware/images/left.bin
cmp build/firmware-right/zephyr/zephyr.bin firmware/images/right.bin
```

The two checked-in binaries were compared byte-for-byte with clean rebuilds in different output directories. The offline native suite includes takeover, watchdog, homing, CAN, hand, peer and standalone logic. For attended hardware acceptance, the separate flash tool accepts the board side, explicit probe serial and CMSIS device pack, backs up the application, programs and verifies readback, then resumes the board. See the arguments in `tools/flash-dual.py`.
