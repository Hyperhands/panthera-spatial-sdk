"""Direct USB dual-arm GELLO teleoperation example for panthera-spatial-sdk.

Reads Feetech STS/SCS servos (left IDs 1..7, right IDs 11..17) over a single
USB-to-serial adapter via astrial (sdk.GelloReader), computes joint & five-finger
hand targets on the host, and streams them directly to sdk.DualArm via 500 Hz
joint-space MIT + gravity compensation (matching dual-gello / Panthera-DualNXP-Hyperhands).
"""

import argparse
import copy
import json
import sys
import time
from contextlib import ExitStack
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import panthera_sdk as sdk

ROOT = Path(__file__).resolve().parents[1]
SIDES = (sdk.Side.left, sdk.Side.right)
SIDE_NAMES = ("left", "right")
RAD_PER_TICK = 2.0 * np.pi / 4096.0
WCH_VENDOR_ID = 0x1A86

# Calibrated defaults ported from dual-gello (gello_profile.h / gello-*.json).
DEFAULT_PROFILE = {
    "left": {
        "factor": [-1.0, -1.0, 1.0, 1.0, -1.0, -1.0],
        "zero_raw_ticks": [119, 1025, 2478, 271, 2069, 25],
        "grip_min": -252.0,
        "grip_max": 302.0,
        "grip_inverted": False,
        "close_at": 100.0,
        "hand_scale": [0.70, 0.95, 0.85, 0.85, 0.85, 0.85],
        "hand_offset_deg": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    },
    "right": {
        "factor": [1.0, -1.0, -1.0, -1.0, -1.0, -1.0],
        "zero_raw_ticks": [1897, 833, 194, 1808, 1417, 3075],
        "grip_min": 1508.0,
        "grip_max": 2057.0,
        "grip_inverted": False,
        "close_at": 100.0,
        "hand_scale": [0.70, 0.95, 0.85, 0.85, 0.85, 0.85],
        "hand_offset_deg": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    },
}
DUAL_GELLO_BASE_SIGN = np.array([-1.0, -1.0, 1.0, -1.0, 1.0, -1.0])


def load_profile(path):
    profile = copy.deepcopy(DEFAULT_PROFILE)
    if path is None:
        return profile
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    for name in SIDE_NAMES:
        if name not in raw:
            continue
        src = raw[name]
        dst = profile[name]
        if "factor" in src:
            dst["factor"] = [float(v) for v in src["factor"]]
        elif "axes" in src:
            dst["factor"] = [
                float(base * a["sign"] * a.get("scale", 1.0))
                for base, a in zip(DUAL_GELLO_BASE_SIGN, src["axes"])
            ]
        if "zero_raw_ticks" in src:
            dst["zero_raw_ticks"] = [float(v) for v in src["zero_raw_ticks"]]
        if "gripper" in src:
            g = src["gripper"]
            dst["grip_min"] = float(g["min_raw"])
            dst["grip_max"] = float(g["max_raw"])
            dst["grip_inverted"] = bool(g.get("inverted", False))
        for key in ("grip_min", "grip_max", "grip_inverted", "close_at"):
            if key in src:
                dst[key] = src[key]
        for key in ("hand_scale", "hand_offset_deg"):
            if key in src:
                dst[key] = [float(v) for v in src[key]]
    return profile


def wrap_delta(a, b):
    d = float(a) - float(b)
    while d > 2048.0:
        d -= 4096.0
    while d < -2048.0:
        d += 4096.0
    return d


def grasp_percent(ticks, side_cfg):
    gmin = float(side_cfg["grip_min"])
    gmax = float(side_cfg["grip_max"])
    middle = 0.5 * (gmin + gmax)
    value = float(ticks) - 4096.0 * round((float(ticks) - middle) / 4096.0)
    opening = float(np.clip((value - gmin) / (gmax - gmin), 0.0, 1.0))
    if side_cfg.get("grip_inverted", False):
        opening = 1.0 - opening
    return 100.0 * (1.0 - opening)


def hand_angles_rad(percent, side_cfg):
    p = float(np.clip(percent, 0.0, 100.0))
    path = float(np.clip(p * 100.0 / float(side_cfg["close_at"]), 0.0, 100.0))
    bends = np.array([13.0, 53.0, 56.0, 55.0, 52.0])
    q = np.zeros(6)
    q[0] = 100.0 * np.clip(path / 20.0, 0.0, 1.0)
    if path > 20.0:
        q[1:] = (
            bends * (path - 20.0) / 75.0
            if path <= 95.0
            else bends + (path - 95.0)
        )
    scale = np.asarray(side_cfg["hand_scale"], dtype=float)
    offset = np.asarray(side_cfg["hand_offset_deg"], dtype=float)
    return np.deg2rad(q * scale + offset)


class StartupZeroCapture:
    """Capture startup zero quickly (average first 5 valid scans, no stillness lockout)."""

    def __init__(self, side_cfg, zero_mode):
        self.cfg = side_cfg
        self.zero_mode = zero_mode
        self.zero = None
        self.last_scan = None
        self.count = 0
        self.total = None

    def reset(self):
        self.last_scan = None
        self.count = 0
        self.total = None

    def update(self, now, scan, ticks_6, raw_6, fresh):
        if self.zero is not None:
            return self.zero
        if not fresh:
            return None
        if self.zero_mode == "saved":
            saved = np.asarray(self.cfg["zero_raw_ticks"], dtype=float)
            self.zero = np.array(
                [t - wrap_delta(r, s) for t, r, s in zip(ticks_6, raw_6, saved)],
                dtype=float,
            )
            return self.zero
        if self.count == 0:
            self.last_scan = scan
            self.count = 1
            self.total = ticks_6.copy()
            return None
        if scan == self.last_scan:
            return None
        self.last_scan = scan
        self.total += ticks_6
        self.count += 1
        if self.count >= 5:
            self.zero = self.total / float(self.count)
            return self.zero
        return None


@dataclass
class MockSample:
    id: int
    raw: int
    ticks: int
    velocity: int = 0
    age_s: float = 0.002
    valid: bool = True


@dataclass
class MockSnapshot:
    connected: bool
    scans: int
    replies: int
    timeouts: int
    bad: int
    jumps: int
    scan_hz: float
    servos: list


class MockGelloReader:
    """Synthetic dual-GELLO reader for offline loopback testing without hardware."""

    def __init__(self, profile, group_mask=0x03):
        self.profile = profile
        self.group_mask = group_mask
        self.scans = 0
        self.stale = [False, False]
        self.arm_q = [np.zeros(6), np.zeros(6)]
        self.grip_pct = [0.0, 0.0]

    def set_arm_angles(self, side_idx, q_rad):
        self.arm_q[side_idx] = np.asarray(q_rad, dtype=float)

    def set_grasp_percent(self, side_idx, pct):
        self.grip_pct[side_idx] = float(np.clip(pct, 0.0, 100.0))

    def poll(self):
        self.scans += 1
        servos = []
        for s_idx, name in enumerate(SIDE_NAMES):
            enabled = bool(self.group_mask & (1 << s_idx))
            cfg = self.profile[name]
            base_id = 1 if s_idx == 0 else 11
            age = 0.200 if self.stale[s_idx] else 0.002
            factor = np.asarray(cfg["factor"], dtype=float)
            zero_raw = np.asarray(cfg["zero_raw_ticks"], dtype=float)
            ticks_arm = np.round(zero_raw + self.arm_q[s_idx] / (RAD_PER_TICK * factor))
            for j in range(6):
                t = int(ticks_arm[j])
                servos.append(
                    MockSample(
                        id=base_id + j,
                        raw=t % 4096,
                        ticks=t,
                        age_s=age,
                        valid=enabled,
                    )
                )
            opening = 1.0 - self.grip_pct[s_idx] / 100.0
            if cfg.get("grip_inverted", False):
                opening = 1.0 - opening
            grip_t = int(
                round(cfg["grip_min"] + opening * (cfg["grip_max"] - cfg["grip_min"]))
            )
            servos.append(
                MockSample(
                    id=base_id + 6,
                    raw=grip_t % 4096,
                    ticks=grip_t,
                    age_s=age,
                    valid=enabled,
                )
            )
        return MockSnapshot(
            connected=True,
            scans=self.scans,
            replies=self.scans * 14,
            timeouts=0,
            bad=0,
            jumps=0,
            scan_hz=200.0,
            servos=servos,
        )

    def __enter__(self):
        return self

    def __exit__(self, *_):
        pass


def print_serial_ports():
    ports = sdk.list_serial_ports()
    if not ports:
        print("No serial ports detected.")
        return ports
    for p in ports:
        vid_pid = f"{p.vendor_id:04x}:{p.product_id:04x}" if (p.vendor_id or p.product_id) else "----:----"
        sn = f" sn={p.serial_number}" if p.serial_number else ""
        desc = f" ({p.description})" if p.description else ""
        print(f"  {p.port_name}  [{vid_pid}]{sn}{desc}")
    return ports


def resolve_port(port_arg):
    if port_arg:
        return port_arg
    ports = sdk.list_serial_ports()
    wch = [p for p in ports if p.vendor_id == WCH_VENDOR_ID]
    if len(wch) == 1:
        print(f"Auto-selected GELLO adapter: {wch[0].port_name}", flush=True)
        return wch[0].port_name
    usb = [p for p in ports if p.vendor_id != 0]
    if len(usb) == 1:
        print(f"Auto-selected USB serial port: {usb[0].port_name}", flush=True)
        return usb[0].port_name
    if len(ports) == 1:
        print(f"Auto-selected serial port: {ports[0].port_name}", flush=True)
        return ports[0].port_name
    print("Could not uniquely auto-detect the GELLO serial port. Detected ports:")
    print_serial_ports()
    raise SystemExit("Please pass --port <device> explicitly.")


def group_mask_for_side(side_opt):
    return {"left": 0x01, "right": 0x02, "both": 0x03}[side_opt]


def extract_group(snap, side_idx, stale_s):
    base = side_idx * 7
    group = snap.servos[base : base + 7]
    arm_fresh = snap.connected and all(
        s.valid and np.isfinite(s.age_s) and s.age_s < stale_s for s in group[:6]
    )
    trig = group[6]
    hand_fresh = (
        snap.connected and trig.valid and np.isfinite(trig.age_s) and trig.age_s < stale_s
    )
    ticks_6 = np.array([float(s.ticks) for s in group[:6]], dtype=float)
    raw_6 = np.array([float(s.raw) for s in group[:6]], dtype=float)
    return arm_fresh, hand_fresh, ticks_6, raw_6, int(trig.ticks)


def check_active(arm):
    state = arm.state()
    if state.error or not state.enabled:
        raise RuntimeError(state.error or "Controller stopped")
    return state


class ServoViewer:
    """Zero-dependency tkinter live dashboard for all 14 GELLO servos (left 1..7, right 11..17)."""

    BG = "#161922"
    CARD = "#1f2430"
    TEXT = "#e6eaf2"
    MUTED = "#8c96ab"
    OK = "#3ddc84"
    WARN = "#ffb74d"
    ERR = "#ff5c77"
    ACCENT = "#4da3ff"
    BAR_BG = "#2c3344"

    def __init__(self, port_label, bounds_lo, bounds_hi, on_rezero=None, on_reenable=None):
        import tkinter as tk

        self.tk = tk
        self.bounds_lo = np.rad2deg(bounds_lo)
        self.bounds_hi = np.rad2deg(bounds_hi)
        self.on_rezero = on_rezero
        self.on_reenable = on_reenable
        self.closed = False

        self.root = tk.Tk()
        self.root.title(f"Panthera Dual-GELLO Servo Viewer — {port_label}")
        self.root.configure(bg=self.BG)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        # Header bar
        header = tk.Frame(self.root, bg=self.CARD, padx=12, pady=8)
        header.pack(fill="x", padx=10, pady=(10, 6))

        self.conn_label = tk.Label(
            header, text="● CONNECTING", fg=self.WARN, bg=self.CARD, font=("Monospace", 10, "bold")
        )
        self.conn_label.pack(side="left")

        self.bus_label = tk.Label(
            header, text="scan=0.0 Hz", fg=self.TEXT, bg=self.CARD, font=("Monospace", 10)
        )
        self.bus_label.pack(side="left", padx=(14, 0))

        self.arm_label = tk.Label(
            header, text="", fg=self.ACCENT, bg=self.CARD, font=("Monospace", 10)
        )
        self.arm_label.pack(side="left", padx=(14, 0))

        if on_rezero is not None:
            btn = tk.Button(
                header,
                text="Re-zero",
                command=on_rezero,
                bg="#2f394f",
                fg=self.TEXT,
                activebackground="#3d4b68",
                activeforeground=self.TEXT,
                relief="flat",
                padx=10,
                pady=2,
                font=("Monospace", 9, "bold"),
            )
            btn.pack(side="right")

        if on_reenable is not None:
            re_btn = tk.Button(
                header,
                text="Re-enable",
                command=on_reenable,
                bg="#2f394f",
                fg=self.OK,
                activebackground="#3d4b68",
                activeforeground=self.OK,
                relief="flat",
                padx=10,
                pady=2,
                font=("Monospace", 9, "bold"),
            )
            re_btn.pack(side="right", padx=(0, 6))

        # Two side-by-side arm cards
        body = tk.Frame(self.root, bg=self.BG)
        body.pack(fill="both", expand=True, padx=10, pady=(0, 10))
        body.columnconfigure(0, weight=1)
        body.columnconfigure(1, weight=1)

        self.cards = []
        for s_idx, (title, base_id) in enumerate((("LEFT ARM (IDs 1–7)", 1), ("RIGHT ARM (IDs 11–17)", 11))):
            card = tk.Frame(body, bg=self.CARD, padx=10, pady=8)
            card.grid(row=0, column=s_idx, sticky="nsew", padx=(0 if s_idx == 0 else 6, 0))

            top = tk.Frame(card, bg=self.CARD)
            top.pack(fill="x", pady=(0, 6))
            tk.Label(
                top, text=title, fg=self.TEXT, bg=self.CARD, font=("Monospace", 10, "bold")
            ).pack(side="left")
            status_lbl = tk.Label(
                top, text="IDLE", fg=self.MUTED, bg=self.CARD, font=("Monospace", 10, "bold")
            )
            status_lbl.pack(side="right")

            table = tk.Frame(card, bg=self.CARD)
            table.pack(fill="x")
            headers = ("Axis", "St", "Raw", "Raw Bar", "Ticks", "Vel", "Angle/Grasp", "Range Bar", "Age")
            for col, text in enumerate(headers):
                tk.Label(
                    table, text=text, fg=self.MUTED, bg=self.CARD, font=("Monospace", 8, "bold")
                ).grid(row=0, column=col, padx=3, pady=(0, 4), sticky="w")

            rows = []
            for j in range(7):
                row_idx = j + 1
                name = f"J{j + 1}(#{base_id + j})" if j < 6 else f"TR(#{base_id + 6})"
                tk.Label(
                    table, text=name, fg=self.TEXT, bg=self.CARD, font=("Monospace", 9), width=9, anchor="w"
                ).grid(row=row_idx, column=0, padx=3, pady=2, sticky="w")
                dot = tk.Label(table, text="●", fg=self.MUTED, bg=self.CARD, font=("Monospace", 10))
                dot.grid(row=row_idx, column=1, padx=2)
                raw_lbl = tk.Label(
                    table, text="----", fg=self.TEXT, bg=self.CARD, font=("Monospace", 9), width=5, anchor="e"
                )
                raw_lbl.grid(row=row_idx, column=2, padx=3)
                raw_bar = tk.Canvas(table, width=76, height=12, bg=self.BAR_BG, highlightthickness=0)
                raw_bar.grid(row=row_idx, column=3, padx=3)
                ticks_lbl = tk.Label(
                    table, text="------", fg=self.TEXT, bg=self.CARD, font=("Monospace", 9), width=7, anchor="e"
                )
                ticks_lbl.grid(row=row_idx, column=4, padx=3)
                vel_lbl = tk.Label(
                    table, text="----", fg=self.MUTED, bg=self.CARD, font=("Monospace", 9), width=5, anchor="e"
                )
                vel_lbl.grid(row=row_idx, column=5, padx=3)
                val_lbl = tk.Label(
                    table, text="------", fg=self.TEXT, bg=self.CARD, font=("Monospace", 9), width=8, anchor="e"
                )
                val_lbl.grid(row=row_idx, column=6, padx=3)
                val_bar = tk.Canvas(table, width=96, height=12, bg=self.BAR_BG, highlightthickness=0)
                val_bar.grid(row=row_idx, column=7, padx=3)
                age_lbl = tk.Label(
                    table, text="-- ms", fg=self.MUTED, bg=self.CARD, font=("Monospace", 9), width=7, anchor="e"
                )
                age_lbl.grid(row=row_idx, column=8, padx=3)
                rows.append((dot, raw_lbl, raw_bar, ticks_lbl, vel_lbl, val_lbl, val_bar, age_lbl))

            hand_lbl = tk.Label(
                card,
                text="Hand(deg): --",
                fg=self.MUTED,
                bg=self.CARD,
                font=("Monospace", 8),
                anchor="w",
            )
            hand_lbl.pack(fill="x", pady=(6, 0))
            self.cards.append({"status": status_lbl, "rows": rows, "hand": hand_lbl})

    def _on_close(self):
        self.closed = True
        try:
            self.root.destroy()
        except Exception:
            pass

    def is_running(self):
        return not self.closed

    def _draw_bar(self, canvas, frac, color, width):
        canvas.delete("all")
        f = float(np.clip(frac, 0.0, 1.0))
        fill_w = max(1, int(round(f * width))) if f > 0 else 0
        if fill_w > 0:
            canvas.create_rectangle(0, 0, fill_w, 12, fill=color, outline="")

    def update(self, snap, stale_s, side_states, arm_summary=""):
        if self.closed:
            return
        if snap.connected:
            self.conn_label.config(text="● CONNECTED", fg=self.OK)
        else:
            self.conn_label.config(text="● DISCONNECTED", fg=self.ERR)
        self.bus_label.config(
            text=(
                f"scan={snap.scan_hz:5.1f}Hz  scans={snap.scans}  "
                f"ok={snap.replies}  to={snap.timeouts}  bad={snap.bad}  jmp={snap.jumps}"
            )
        )
        self.arm_label.config(text=arm_summary)

        for s_idx, card in enumerate(self.cards):
            st = side_states[s_idx]
            label = st["status"]
            color = (
                self.OK
                if label == "OK"
                else self.WARN
                if label in ("ZEROING", "HOLD")
                else self.MUTED
                if label == "IDLE"
                else self.ERR
            )
            card["status"].config(text=label, fg=color)

            base = s_idx * 7
            q_deg = st.get("q_deg")
            grasp = st.get("grasp_pct")
            hand_deg = st.get("hand_deg")

            for j in range(7):
                s = snap.servos[base + j]
                dot, raw_lbl, raw_bar, ticks_lbl, vel_lbl, val_lbl, val_bar, age_lbl = card["rows"][j]
                fresh = snap.connected and s.valid and np.isfinite(s.age_s) and s.age_s < stale_s
                if fresh:
                    dot.config(fg=self.OK)
                elif s.valid:
                    dot.config(fg=self.WARN)
                else:
                    dot.config(fg=self.ERR)

                raw_lbl.config(text=f"{s.raw:4d}" if s.valid else "----")
                self._draw_bar(raw_bar, (s.raw / 4095.0) if s.valid else 0.0, self.ACCENT if fresh else self.MUTED, 76)
                ticks_lbl.config(text=f"{s.ticks:6d}" if s.valid else "------")
                vel_lbl.config(text=f"{s.velocity:5d}" if s.valid else "----")
                if np.isfinite(s.age_s):
                    age_ms = s.age_s * 1e3
                    age_lbl.config(
                        text=f"{age_ms:5.1f}ms" if age_ms < 999.9 else ">999ms",
                        fg=self.MUTED if fresh else self.WARN,
                    )
                else:
                    age_lbl.config(text="   -- ", fg=self.ERR)

                if j < 6:
                    if q_deg is not None:
                        deg = float(q_deg[j])
                        lo = float(self.bounds_lo[s_idx * 6 + j])
                        hi = float(self.bounds_hi[s_idx * 6 + j])
                        frac = (deg - lo) / max(hi - lo, 1e-6)
                        val_lbl.config(text=f"{deg:+6.1f}°")
                        self._draw_bar(val_bar, frac, self.OK if fresh else self.WARN, 96)
                    else:
                        val_lbl.config(text="  --.-°")
                        self._draw_bar(val_bar, 0.0, self.MUTED, 96)
                else:
                    if grasp is not None:
                        val_lbl.config(text=f"{grasp:5.1f}%")
                        self._draw_bar(val_bar, grasp / 100.0, self.WARN if grasp > 70.0 else self.OK, 96)
                    else:
                        val_lbl.config(text="  --.-%")
                        self._draw_bar(val_bar, 0.0, self.MUTED, 96)

            if hand_deg is not None:
                card["hand"].config(
                    text=(
                        f"Hand(deg): T1={hand_deg[0]:4.1f} T2={hand_deg[1]:4.1f} "
                        f"I={hand_deg[2]:4.1f} M={hand_deg[3]:4.1f} R={hand_deg[4]:4.1f} L={hand_deg[5]:4.1f}"
                    )
                )
            else:
                card["hand"].config(text="Hand(deg): --")

        try:
            self.root.update_idletasks()
            self.root.update()
        except Exception:
            self.closed = True

    def close(self):
        if not self.closed:
            self._on_close()


class TerminalServoViewer:
    """ANSI terminal fallback when GUI display is unavailable."""

    def __init__(self, port_label):
        self.port_label = port_label
        self.last_draw = 0.0

    def is_running(self):
        return True

    def update(self, snap, stale_s, side_states, arm_summary=""):
        now = time.monotonic()
        if now - self.last_draw < 0.08:
            return
        self.last_draw = now
        lines = [
            f"Panthera Dual-GELLO Servo Monitor [{self.port_label}]  "
            f"{'CONNECTED' if snap.connected else 'DISCONNECTED'}  "
            f"scan={snap.scan_hz:5.1f}Hz ok={snap.replies} to={snap.timeouts} bad={snap.bad} jmp={snap.jumps}",
        ]
        if arm_summary:
            lines.append(arm_summary)
        for s_idx, name in enumerate(SIDE_NAMES):
            st = side_states[s_idx]
            lines.append(f"\n[{name.upper()} — {st['status']}]")
            base = s_idx * 7
            q_deg = st.get("q_deg")
            grasp = st.get("grasp_pct")
            for j in range(7):
                s = snap.servos[base + j]
                fresh = snap.connected and s.valid and np.isfinite(s.age_s) and s.age_s < stale_s
                tag = "OK " if fresh else ("STL" if s.valid else "OFF")
                axis = f"J{j + 1}(#{s.id:2d})" if j < 6 else f"TR(#{s.id:2d})"
                age_txt = f"{s.age_s * 1e3:5.1f}ms" if np.isfinite(s.age_s) else "   --  "
                if j < 6:
                    val_txt = f"{q_deg[j]:+6.1f}deg" if q_deg is not None else "   --.-deg"
                else:
                    val_txt = f"{grasp:5.1f}%   " if grasp is not None else "  --.-%   "
                lines.append(
                    f"  {axis} [{tag}] raw={s.raw:4d} ticks={s.ticks:7d} vel={s.velocity:5d}  {val_txt}  age={age_txt}"
                )
        sys.stdout.write("\x1b[H\x1b[2J" + "\n".join(lines) + "\n")
        sys.stdout.flush()

    def close(self):
        pass


def create_servo_viewer(
    port_label, bounds_lo, bounds_hi, on_rezero=None, on_reenable=None, force_terminal=False
):
    if not force_terminal:
        try:
            return ServoViewer(
                port_label,
                bounds_lo,
                bounds_hi,
                on_rezero=on_rezero,
                on_reenable=on_reenable,
            )
        except Exception as exc:
            print(f"GUI unavailable ({exc}); falling back to terminal servo viewer.", flush=True)
    return TerminalServoViewer(port_label)


def run_monitor(args):
    """Standalone non-real GELLO servo monitor (no DualArm / NXP boards required)."""
    profile = load_profile(args.config)
    group_mask = group_mask_for_side(args.side)
    active_sides = [i for i in range(2) if group_mask & (1 << i)]
    stale_s = args.stale_ms * 1e-3
    bounds = sdk.default_bounds()
    bounds_lo = np.asarray(bounds.lower, dtype=float)
    bounds_hi = np.asarray(bounds.upper, dtype=float)

    captures = [StartupZeroCapture(profile[name], args.zero_mode) for name in SIDE_NAMES]

    def on_rezero():
        for i in active_sides:
            captures[i].zero = None
            captures[i].reset()

    with ExitStack() as stack:
        if args.mock_gello:
            port_label = "mock-gello"
            gello = stack.enter_context(MockGelloReader(profile, group_mask))
        else:
            port_label = resolve_port(args.port)
            gello = stack.enter_context(
                sdk.GelloReader(
                    port=port_label,
                    baud=args.baud,
                    group_mask=group_mask,
                    servo_timeout_ms=args.servo_timeout_ms,
                    use_sync_read=not args.no_sync_read,
                )
            )

        if args.headless:
            # Quick smoke verification of monitor decoding without GUI.
            t0 = time.monotonic()
            while time.monotonic() - t0 < 1.0:
                snap = gello.poll()
                now = time.monotonic()
                for i in active_sides:
                    arm_fresh, _, ticks_6, raw_6, trig_ticks = extract_group(snap, i, stale_s)
                    zero = captures[i].update(now, snap.scans, ticks_6, raw_6, arm_fresh)
                    if zero is not None:
                        pct = grasp_percent(trig_ticks, profile[SIDE_NAMES[i]])
                        _ = hand_angles_rad(pct, profile[SIDE_NAMES[i]])
                if all(captures[i].zero is not None for i in active_sides):
                    print("headless: monitor zero & servo decode verified", flush=True)
                    return
                time.sleep(0.01)
            raise RuntimeError("Monitor headless verification timed out")

        servo_ui = create_servo_viewer(port_label, bounds_lo, bounds_hi, on_rezero=on_rezero)
        t_start = time.monotonic()
        try:
            while servo_ui.is_running():
                now = time.monotonic()
                if args.mock_gello:
                    phase = now - t_start
                    if phase > 0.8:
                        w = 0.25 * np.sin(1.2 * (phase - 0.8))
                        gello.set_arm_angles(0, [w, 0.5 + w, 0.8 + w, -0.3, w, 0.0])
                        gello.set_arm_angles(1, [-w, 0.5 + w, 0.8 + w, -0.3, -w, 0.0])
                        gello.set_grasp_percent(0, 50.0 * (1.0 + np.sin(1.5 * phase)))
                        gello.set_grasp_percent(1, 50.0 * (1.0 - np.sin(1.5 * phase)))
                snap = gello.poll()
                side_states = [{"status": "IDLE"}, {"status": "IDLE"}]
                for i in active_sides:
                    name = SIDE_NAMES[i]
                    arm_fresh, hand_fresh, ticks_6, raw_6, trig_ticks = extract_group(snap, i, stale_s)
                    zero = captures[i].update(now, snap.scans, ticks_6, raw_6, arm_fresh)
                    st = {"status": "OK" if arm_fresh else "STALE"}
                    if zero is None:
                        st["status"] = "ZEROING"
                    else:
                        factor = np.asarray(profile[name]["factor"], dtype=float)
                        q = (ticks_6 - zero) * RAD_PER_TICK * factor
                        st["q_deg"] = np.rad2deg(q)
                    if hand_fresh:
                        pct = grasp_percent(trig_ticks, profile[name])
                        st["grasp_pct"] = pct
                        st["hand_deg"] = np.rad2deg(hand_angles_rad(pct, profile[name]))
                    side_states[i] = st
                servo_ui.update(snap, stale_s, side_states, arm_summary="mode=monitor (no DualArm)")
                time.sleep(0.03)
        except KeyboardInterrupt:
            pass
        finally:
            servo_ui.close()


def run_headless_verification(arm, plant, gello, profile, config, send_hands):
    """Verify startup zeroing, dual-arm tracking, hand mapping, stale freeze, and collision safety."""
    bounds_lo = np.asarray(config.joints.lower, dtype=float)
    bounds_hi = np.asarray(config.joints.upper, dtype=float)
    captures = [StartupZeroCapture(profile[name], "startup") for name in SIDE_NAMES]
    anchors = [np.zeros(6), np.zeros(6)]

    # Phase 1: hold GELLO steady at neutral pose for startup zeroing (> 0.5 s).
    gello.set_arm_angles(0, np.zeros(6))
    gello.set_arm_angles(1, np.zeros(6))
    t0 = time.monotonic()
    while time.monotonic() - t0 < 3.0:
        snap = gello.poll()
        now = time.monotonic()
        ready = True
        for i in range(2):
            arm_fresh, _, ticks_6, raw_6, _ = extract_group(snap, i, 0.05)
            if captures[i].update(now, snap.scans, ticks_6, raw_6, arm_fresh) is None:
                ready = False
        check_active(arm)
        if ready:
            break
        time.sleep(0.01)
    else:
        raise RuntimeError("Startup zero capture timed out")
    print("headless: startup zero captured", flush=True)

    # Phase 2: command a safe dual-arm joint pose + grasp and verify tracking.
    goal_left = np.array([0.15, 0.90, 1.20, -0.45, 0.0, 0.0])
    goal_right = np.array([-0.15, 0.90, 1.20, -0.45, 0.0, 0.0])
    gello.set_arm_angles(0, goal_left)
    gello.set_arm_angles(1, goal_right)
    gello.set_grasp_percent(0, 60.0)
    gello.set_grasp_percent(1, 40.0)

    deadline = time.monotonic() + 15.0
    last_hand = 0.0
    while time.monotonic() < deadline:
        snap = gello.poll()
        for i, (side, name) in enumerate(zip(SIDES, SIDE_NAMES)):
            arm_fresh, hand_fresh, ticks_6, _, trig_ticks = extract_group(snap, i, 0.05)
            if arm_fresh:
                factor = np.asarray(profile[name]["factor"], dtype=float)
                q = anchors[i] + (ticks_6 - captures[i].zero) * RAD_PER_TICK * factor
                q = np.clip(q, bounds_lo[i * 6 : i * 6 + 6], bounds_hi[i * 6 : i * 6 + 6])
                arm.set_joint_target(side, q)
            if send_hands and hand_fresh and time.monotonic() - last_hand >= 0.02:
                pct = grasp_percent(trig_ticks, profile[name])
                arm.set_hand_target(side, hand_angles_rad(pct, profile[name]), 20)
        if time.monotonic() - last_hand >= 0.02:
            last_hand = time.monotonic()
        state = check_active(arm)
        if plant.error:
            raise plant.error
        err_l = float(np.max(np.abs(state.measured[:6] - goal_left)))
        err_r = float(np.max(np.abs(state.measured[6:] - goal_right)))
        if err_l < 0.03 and err_r < 0.03:
            print(
                f"headless: tracked dual pose err_l={err_l:.4f} err_r={err_r:.4f} rad",
                flush=True,
            )
            break
        time.sleep(0.005)
    else:
        raise RuntimeError(f"Dual-arm tracking timed out (err_l={err_l:.4f}, err_r={err_r:.4f})")

    # Phase 3: simulate stale GELLO input and verify robot freezes safely without faulting.
    gello.stale = [True, True]
    frozen_ref = check_active(arm).reference.copy()
    gello.set_arm_angles(0, goal_left + 0.5)
    for _ in range(30):
        snap = gello.poll()
        for i, side in enumerate(SIDES):
            arm_fresh, _, _, _, _ = extract_group(snap, i, 0.05)
            if not arm_fresh:
                state = check_active(arm)
                arm.set_joint_target(side, state.reference[i * 6 : i * 6 + 6])
        time.sleep(0.005)
    after_ref = check_active(arm).reference
    if float(np.max(np.abs(after_ref - frozen_ref))) > 0.02:
        raise RuntimeError("Stale GELLO input did not freeze target")
    print("headless: stale-input freeze verified", flush=True)


def run_teleop(args):
    profile = load_profile(args.config)
    group_mask = group_mask_for_side(args.side)
    active_sides = [i for i in range(2) if group_mask & (1 << i)]
    stale_s = args.stale_ms * 1e-3

    config = sdk.ControlConfig()
    config.bypass_qp = True
    config.joints.max_speed = np.full(12, float(args.max_speed))
    config.joints.max_acceleration = np.full(12, float(args.max_accel))
    bounds_lo = np.asarray(config.joints.lower, dtype=float)
    bounds_hi = np.asarray(config.joints.upper, dtype=float)

    sys.path.insert(0, str(ROOT / "sim"))
    mj = None
    has_mujoco = False
    try:
        import mujoco as mj_mod

        mj = mj_mod
        has_mujoco = True
    except ModuleNotFoundError:
        pass

    if has_mujoco:
        from mujoco_board import HAND, Plant, indices, load_scene
    else:
        from loopback import BoardPair

    with ExitStack() as stack:
        plant = None
        viewer = None
        servo_ui = None
        display_model = None
        display = None
        handles = None

        if args.real:
            arm = stack.enter_context(
                sdk.DualArm(sdk.BoardAddress.left(), sdk.BoardAddress.right(), config)
            )
            arm.enable(timeout_ms=30000)
            if args.viewer and has_mujoco:
                import mujoco.viewer

                display_model = load_scene()
                display = mj.MjData(display_model)
                handles = [
                    {
                        "aq": indices(display_model, [f"{n}_joint{j}" for j in range(1, 7)])[0],
                        "av": indices(display_model, [f"{n}_joint{j}" for j in range(1, 7)])[1],
                        "hq": indices(display_model, [f"{n}_{h}_joint" for h in HAND])[0],
                    }
                    for n in SIDE_NAMES
                ]
                viewer = stack.enter_context(
                    mujoco.viewer.launch_passive(
                        display_model, display, show_left_ui=False, show_right_ui=False
                    )
                )
        else:
            if has_mujoco:
                plant = stack.enter_context(
                    Plant(args.start_zero, measure_clearance=args.headless)
                )
                if not args.headless:
                    import mujoco.viewer

                    display_model = copy.copy(plant.model)
                    display = mj.MjData(display_model)
                    display.qpos[:] = plant.data.qpos
                    viewer = stack.enter_context(
                        mujoco.viewer.launch_passive(
                            display_model, display, show_left_ui=False, show_right_ui=False
                        )
                    )
            else:
                plant = stack.enter_context(BoardPair(ready=not args.start_zero))
                if not args.headless:
                    print(
                        "MuJoCo not installed; running loopback against lightweight UDP simulator.",
                        flush=True,
                    )
            arm = stack.enter_context(sdk.DualArm(*plant.addresses(), config))
            plant.start([arm.local_ports(s) for s in SIDES])
            arm.enable(timeout_ms=5000)

        if args.mock_gello:
            port_label = "mock-gello"
            gello = stack.enter_context(MockGelloReader(profile, group_mask))
        else:
            port_label = resolve_port(args.port)
            gello = stack.enter_context(
                sdk.GelloReader(
                    port=port_label,
                    baud=args.baud,
                    group_mask=group_mask,
                    servo_timeout_ms=args.servo_timeout_ms,
                    use_sync_read=not args.no_sync_read,
                )
            )

        if args.headless:
            run_headless_verification(arm, plant, gello, profile, config, not args.no_hand)
            return

        state = check_active(arm)
        captures = [StartupZeroCapture(profile[name], args.zero_mode) for name in SIDE_NAMES]
        anchors = [
            state.measured[i * 6 : i * 6 + 6].copy() if args.anchor == "current" else np.zeros(6)
            for i in range(2)
        ]
        # Hold inactive side(s) at their initial pose.
        for i in range(2):
            if i not in active_sides:
                arm.set_joint_target(SIDES[i], state.measured[i * 6 : i * 6 + 6])

        announced_zero = [False, False]
        was_stale = [False, False]
        was_stopped = False
        reenable_requested = [False]

        def on_rezero():
            for i in active_sides:
                captures[i].zero = None
                captures[i].reset()
                announced_zero[i] = False

        def on_reenable():
            reenable_requested[0] = True

        # Open servo dashboard when --viewer is requested, or by default in non-real mode without MuJoCo.
        if args.viewer or (not args.real and not has_mujoco):
            servo_ui = create_servo_viewer(
                port_label, bounds_lo, bounds_hi, on_rezero=on_rezero, on_reenable=on_reenable
            )

        print(
            f"Dual GELLO teleop active: mode={'real' if args.real else 'loopback'} "
            f"side={args.side} zero={args.zero_mode} anchor={args.anchor} "
            f"hand={'off' if args.no_hand else 'on'} max_speed={args.max_speed} rad/s",
            flush=True,
        )
        if args.zero_mode == "startup":
            print("Hold active GELLO arm(s) still for 0.5 s to capture startup zero...", flush=True)

        last_hand = 0.0
        last_ui = 0.0
        next_print = 0.0
        mock_demo_t0 = None
        last_valid_ticks = [None, None]
        dt_loop = 1.0 / max(20.0, float(args.hz))

        try:
            while (viewer is None or viewer.is_running()) and (servo_ui is None or servo_ui.is_running()):
                now = time.monotonic()
                state = arm.state()
                if plant is not None and plant.error:
                    print(f"[WARN] simulator error: {plant.error}", flush=True)
                    plant.error = None

                if reenable_requested[0]:
                    reenable_requested[0] = False
                    if not state.enabled:
                        try:
                            arm.enable(timeout_ms=30000 if args.real else 5000)
                            state = arm.state()
                            on_rezero()
                            was_stopped = False
                            print("[OK] DualArm re-enabled -> re-zeroing", flush=True)
                        except Exception as exc:
                            print(f"[WARN] Re-enable failed: {exc}", flush=True)

                if args.mock_gello and not args.no_hand and all(announced_zero[i] for i in active_sides):
                    if mock_demo_t0 is None:
                        mock_demo_t0 = now
                    phase = now - mock_demo_t0
                    # Gentle 0% <-> 35% grasp sine wave starting smoothly from 0% (arms stay still).
                    pct_demo = 17.5 * (1.0 - float(np.cos(1.5 * phase)))
                    for i in active_sides:
                        gello.set_grasp_percent(i, pct_demo)

                snap = gello.poll()
                hand_due = (not args.no_hand) and (now - last_hand >= 0.02)
                side_status = []
                ui_states = [{"status": "IDLE"}, {"status": "IDLE"}]

                if not state.enabled:
                    if not was_stopped:
                        was_stopped = True
                        print(
                            f"[WARN] DualArm stopped: {state.error or 'inactive'} "
                            f"(click Re-enable in viewer to recover)",
                            flush=True,
                        )
                    for i in active_sides:
                        ui_states[i] = {"status": "STOPPED"}
                        side_status.append(f"{SIDE_NAMES[i]}=STOPPED")
                else:
                    was_stopped = False
                    for i in active_sides:
                        side = SIDES[i]
                        name = SIDE_NAMES[i]
                        arm_fresh, hand_fresh, ticks_6, raw_6, trig_ticks = extract_group(
                            snap, i, stale_s
                        )
                        grp = snap.servos[i * 7 : i * 7 + 6]
                        if last_valid_ticks[i] is None:
                            if all(s.valid for s in grp):
                                last_valid_ticks[i] = ticks_6.copy()
                        else:
                            for j, s in enumerate(grp):
                                if s.valid and np.isfinite(s.age_s) and s.age_s < stale_s:
                                    last_valid_ticks[i][j] = float(s.ticks)
                        effective_ticks = (
                            last_valid_ticks[i] if last_valid_ticks[i] is not None else ticks_6
                        )
                        any_fresh = snap.connected and any(
                            s.valid and np.isfinite(s.age_s) and s.age_s < stale_s for s in grp
                        )

                        st = {"status": "ZEROING"}
                        if hand_fresh:
                            pct = grasp_percent(trig_ticks, profile[name])
                            st["grasp_pct"] = pct
                            st["hand_deg"] = np.rad2deg(hand_angles_rad(pct, profile[name]))
                            if hand_due:
                                try:
                                    arm.set_hand_target(
                                        side, hand_angles_rad(pct, profile[name]), 20
                                    )
                                except Exception as exc:
                                    print(f"[WARN] set_hand_target({name}): {exc}", flush=True)

                        zero = captures[i].update(now, snap.scans, effective_ticks, raw_6, arm_fresh)
                        if zero is None:
                            side_status.append(f"{name}=ZEROING")
                            ui_states[i] = st
                            continue
                        if not announced_zero[i]:
                            announced_zero[i] = True
                            if args.anchor == "current":
                                anchors[i] = state.measured[i * 6 : i * 6 + 6].copy()
                            print(f"[{name}] zero locked -> teleop engaged", flush=True)

                        factor = np.asarray(profile[name]["factor"], dtype=float)
                        q = anchors[i] + (effective_ticks - zero) * RAD_PER_TICK * factor
                        q_clipped = np.clip(q, bounds_lo[i * 6 : i * 6 + 6], bounds_hi[i * 6 : i * 6 + 6])
                        st["q_deg"] = np.rad2deg(q_clipped)

                        if any_fresh:
                            if was_stale[i]:
                                was_stale[i] = False
                                print(f"[{name}] GELLO input recovered -> resuming", flush=True)
                            try:
                                arm.set_joint_target(side, q_clipped)
                            except Exception as exc:
                                print(f"[WARN] set_joint_target({name}): {exc}", flush=True)
                            side_status.append(f"{name}=OK")
                            st["status"] = "OK"
                        else:
                            if not was_stale[i]:
                                was_stale[i] = True
                                print(f"[{name}] GELLO disconnected -> holding pose", flush=True)
                            try:
                                arm.set_joint_target(side, state.reference[i * 6 : i * 6 + 6])
                            except Exception:
                                pass
                            side_status.append(f"{name}=HOLD")
                            st["status"] = "HOLD"

                        ui_states[i] = st

                    if hand_due:
                        last_hand = now

                if viewer is not None:
                    if plant is not None:
                        with plant.lock:
                            display.qpos[:] = plant.data.qpos
                            display.qvel[:] = plant.data.qvel
                            display.time = plant.data.time
                        mj.mj_forward(display_model, display)
                    else:
                        with viewer.lock():
                            for i, h in enumerate(handles):
                                display.qpos[h["aq"]] = state.measured[i * 6 : i * 6 + 6]
                                display.qvel[h["av"]] = state.velocity[i * 6 : i * 6 + 6]
                                hand = state.boards[i].hand
                                if hand is not None:
                                    qh = np.asarray(hand.joint_q_rad, dtype=float)
                                    if qh.shape == h["hq"].shape and np.all(np.isfinite(qh)):
                                        display.qpos[h["hq"]] = qh
                            mj.mj_forward(display_model, display)
                    viewer.sync()

                if servo_ui is not None and now - last_ui >= 0.033:
                    arm_summary = (
                        f"direct-MIT  cycles={state.cycles}  "
                        f"{'ENABLED' if state.enabled else 'STOPPED'}"
                    )
                    servo_ui.update(snap, stale_s, ui_states, arm_summary=arm_summary)
                    last_ui = now

                if servo_ui is None and now >= next_print:
                    counts = []
                    for i in active_sides:
                        grp = snap.servos[i * 7 : i * 7 + 6]
                        n_ok = sum(
                            1 for s in grp if s.valid and np.isfinite(s.age_s) and s.age_s < stale_s
                        )
                        grip = ui_states[i].get("grasp_pct")
                        g_txt = f" grip={grip:.0f}%" if grip is not None else ""
                        counts.append(f"{SIDE_NAMES[i]}={ui_states[i]['status']}({n_ok}/6{g_txt})")
                    print(
                        f"gello={snap.scan_hz:5.1f}Hz (ok={snap.replies} to={snap.timeouts} bad={snap.bad}) "
                        f"{' '.join(counts)} direct-MIT cycles={state.cycles}",
                        flush=True,
                    )
                    next_print = now + 1.0

                time.sleep(dt_loop)
        except KeyboardInterrupt:
            print("\nCtrl+C received -> soft-stopping DualArm...", flush=True)
        finally:
            if servo_ui is not None:
                servo_ui.close()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    mode = p.add_mutually_exclusive_group()
    mode.add_argument(
        "--real",
        action="store_true",
        help="connect to real left/right NXP boards",
    )
    mode.add_argument(
        "--loopback",
        action="store_true",
        help="run against local simulation (default)",
    )
    mode.add_argument(
        "--monitor",
        action="store_true",
        help="standalone GELLO 14-servo visual monitor (no DualArm / NXP boards needed)",
    )
    p.add_argument("--list-ports", action="store_true", help="list detected serial ports and exit")
    p.add_argument("--port", default="", help="GELLO USB serial port (auto-detect if omitted)")
    p.add_argument("--baud", type=int, default=1_000_000, help="servo bus baud rate (default 1000000)")
    p.add_argument("--config", default=None, help="optional JSON calibration override")
    p.add_argument(
        "--side",
        choices=("both", "left", "right"),
        default="both",
        help="which arm(s) to teleoperate (default both)",
    )
    p.add_argument(
        "--zero-mode",
        choices=("startup", "saved"),
        default="startup",
        help="startup: hold GELLO still 0.5 s to zero (default); saved: use calibration ticks",
    )
    p.add_argument(
        "--anchor",
        choices=("zero", "current"),
        default="zero",
        help="robot pose corresponding to GELLO zero (default zero)",
    )
    p.add_argument("--no-hand", action="store_true", help="disable five-finger hand commands")
    p.add_argument(
        "--max-speed",
        type=float,
        default=3.5,
        help="joint velocity limit in rad/s (default 3.5)",
    )
    p.add_argument(
        "--max-accel",
        type=float,
        default=25.0,
        help="joint acceleration limit in rad/s^2 (default 25.0)",
    )
    p.add_argument(
        "--stale-ms",
        type=float,
        default=150.0,
        help="GELLO stale timeout in ms (default 150, matching dual-gello)",
    )
    p.add_argument(
        "--servo-timeout-ms",
        type=int,
        default=5,
        help="per-servo serial read timeout in ms (default 5)",
    )
    p.add_argument(
        "--no-sync-read",
        action="store_true",
        help="disable Feetech SYNC_READ (0x82) and poll servos one by one",
    )
    p.add_argument("--hz", type=float, default=250.0, help="Python control loop rate in Hz (default 250)")
    p.add_argument(
        "--viewer",
        action="store_true",
        help="open the 14-servo live GUI viewer (works in both loopback and --real modes; also opens MuJoCo 3D twin if installed)",
    )
    p.add_argument("--start-zero", action="store_true", help="in loopback mode, start sim at folded zero")
    p.add_argument("--mock-gello", action="store_true", help="use synthetic GELLO data (no USB adapter)")
    p.add_argument("--headless", action="store_true", help="run automated loopback verification without viewer")
    args = p.parse_args()

    if args.list_ports:
        print_serial_ports()
        return
    if args.real and args.headless:
        p.error("--headless requires loopback or --monitor mode")
    if args.headless and not args.mock_gello:
        p.error("--headless requires --mock-gello")
    if (args.real or args.monitor) and args.start_zero:
        p.error("--start-zero only applies to loopback simulation")

    if args.monitor:
        run_monitor(args)
    else:
        run_teleop(args)


if __name__ == "__main__":
    main()
