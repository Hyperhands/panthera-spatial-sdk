"""Build and execute the imported MCU logic tests on a native C11 toolchain."""

import os
from pathlib import Path
import subprocess as sp
import tempfile

FW = Path(__file__).resolve().parents[1]
SRC = FW / "source/src"
TESTS = FW / "source/tests"
SDK = FW.parent
UNITS = """devices/can_codec devices/stopped_can devices/hand_current devices/hand_startup
devices/hand_transaction homing/paired_home homing/home_path homing/home_still homing/homing
homing/gravity peer/peer_codec gello/gello_codec gello/gello_usb_desc gello/gello_radio_codec
gello/gello_action gello/gello_action_bank gello/gello_web_codec gello/gello_settings
standalone/gello_follow""".split()


def run(*args):
    sp.run(args, check=True)


def main():
    cc = os.environ.get("CC", "cc")
    flags = [
        "-std=c11",
        "-O1",
        "-Wall",
        "-Wextra",
        "-Wno-unused-function",
        "-I" + str(TESTS / "gello_fakes"),
        "-I" + str(SRC),
        "-I" + str(SDK / "third_party/wirelink/include"),
        "-I" + str(SDK / "generated/protocol"),
    ]
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp)
        lib = out / "logic.a"
        for src in [SRC / (name + ".c") for name in UNITS] + [
            SDK / "generated/protocol/robot_messages.c"
        ]:
            obj = out / (src.stem + ".o")
            run(cc, *flags, "-c", src, "-o", obj)
            run(os.environ.get("AR", "ar"), "rcs", lib, obj)
        for src in sorted(TESTS.glob("test_*.c")):
            web = src.stem in (
                "test_gello_web",
                "test_gello_standalone",
                "test_gello_settings_home",
            )
            extra = ["-DCONFIG_ROBOT_GELLO_WEB=1"] if web else []
            if src.stem == "test_gello_settings_home":
                extra += [
                    "-DCONFIG_ROBOT_LOCAL_HOME=1",
                    "-DCONFIG_ROBOT_BOOT_HOME=1",
                    "-DCONFIG_ROBOT_BOOT_HAND_FIRST=1",
                    "-DTEST_HOME_BUTTON=1",
                ]
            modes = (
                [(11, 0), (11, 1), (12, 0), (12, 1)]
                if src.stem in ("test_gello_web", "test_gello_standalone")
                else [(11, 0)]
            )
            for node, actuate in modes:
                exe = out / src.stem
                run(
                    cc,
                    *flags,
                    *extra,
                    f"-DCONFIG_ROBOT_NODE_ID={node}",
                    f"-DCONFIG_ROBOT_GELLO_STANDALONE_ACTUATE={actuate}",
                    "-DCONFIG_ROBOT_GELLO_RADIO_GATEWAY=0",
                    src,
                    lib,
                    "-lm",
                    "-o",
                    exe,
                )
                run(exe)


if __name__ == "__main__":
    main()
