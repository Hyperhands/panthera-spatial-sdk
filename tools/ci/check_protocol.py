"""Check the firmware's wirelink source identity and WLC output manifest."""

import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]
WIRELINK = ROOT / "third_party" / "wirelink"
GENERATED = ROOT / "generated" / "protocol"

# Git trees of the two directories compiled into the 11.20 firmware.
for directory, expected in {
    "include/wirelink": "ccaa46c33630a43b0aa0aa85830a4e89ff4f4a2e",
    "src": "3b3e9ad562ff9fb5d3f474bae56bdca1675e3703",
}.items():
    actual = subprocess.check_output(
        ["git", "-C", str(WIRELINK), "rev-parse", f"HEAD:{directory}"], text=True
    ).strip()
    if actual != expected:
        raise SystemExit(f"wirelink {directory}: expected {expected}, got {actual}")

manifest = json.loads((GENERATED / "robot_messages_manifest.json").read_text())
if manifest["artifact_digest_algorithm"] != "fnv1a64-domain-bytes-v1":
    raise SystemExit("Unrecognized WLC artifact digest")
for artifact in manifest["artifacts"]:
    data = (GENERATED / artifact["path"]).read_bytes()
    digest = 0xCBF29CE484222325
    for octet in b"wlc.generated-artifact.v1\xff" + data:
        digest = ((digest ^ octet) * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    if len(data) != artifact["size"] or digest != int(artifact["digest"], 16):
        raise SystemExit(f"WLC manifest mismatch: {artifact['path']}")
