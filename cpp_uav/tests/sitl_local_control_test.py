"""
SITL test harness for the feature/local-control fork of ArduPlane.

Connects to a SITL via UDP, exposes a single-keystroke menu that fires the
nine acceptance scenarios for the new SET_POSITION_TARGET_LOCAL_NED handler.

Run:
    py tests/sitl_local_control_test.py                       # default tcp:5760 (direct SITL)
    py tests/sitl_local_control_test.py 14550                 # udpin:127.0.0.1:14550
    py tests/sitl_local_control_test.py tcp:127.0.0.1:5762    # full conn string

Prereqs (Windows native python):
    py -m pip install pymavlink

Companion notes:
    sim_vehicle.py (via run_arduplane_sitl.bat) already MAVProxy-forwards
    to 14550 (cpp_uav) and 14552 (this script). They run side by side.
"""

from __future__ import annotations

import math
import sys
import threading
import time
from dataclasses import dataclass

from pymavlink import mavutil


# ---------- MAVLink helpers ----------

# POSITION_TARGET_TYPEMASK bits (1 = ignore)
T_VX, T_VY, T_VZ = 1 << 3, 1 << 4, 1 << 5
T_AX, T_AY, T_AZ = 1 << 6, 1 << 7, 1 << 8
T_FORCE = 1 << 9
T_YAW = 1 << 10
T_YAW_RATE = 1 << 11

MASK_POS_ONLY = T_VX | T_VY | T_VZ | T_AX | T_AY | T_AZ | T_FORCE | T_YAW | T_YAW_RATE
MASK_POS_YAW = MASK_POS_ONLY & ~T_YAW

FRAME_LOCAL_NED = mavutil.mavlink.MAV_FRAME_LOCAL_NED
FRAME_LOCAL_OFFSET_NED = mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED
FRAME_BODY_OFFSET_NED = mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED
FRAME_BODY_NED = mavutil.mavlink.MAV_FRAME_BODY_NED


EARTH_R = 6_378_137.0  # m, WGS-84


def latlon_to_ne(lat0: float, lon0: float, lat: float, lon: float) -> tuple[float, float]:
    """Local-flat-Earth NE offset (m) of (lat,lon) relative to (lat0,lon0)."""
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    n = dlat * EARTH_R
    e = dlon * EARTH_R * math.cos(math.radians(lat0))
    return n, e


@dataclass
class TelemetrySnapshot:
    lat: float = float("nan")
    lon: float = float("nan")
    alt_rel: float = float("nan")
    heading_deg: float = float("nan")
    airspeed: float = float("nan")
    armed: bool = False
    mode: str = "?"
    last_statustext: str = ""
    nav_bearing_deg: float = float("nan")        # current GUIDED target bearing
    nav_alt_target_m: float = float("nan")       # current GUIDED target altitude
    wp_dist_m: float = float("nan")              # distance to current target
    min_wp_dist_m: float = float("inf")          # smallest wp_dist since last reset

    def fmt(self) -> str:
        return (
            f"mode={self.mode:<10} arm={'Y' if self.armed else 'N'}  "
            f"hdg={self.heading_deg:6.1f}deg  alt={self.alt_rel:6.1f}m  "
            f"airspd={self.airspeed:5.1f}m/s  "
            f"wp_dist={self.wp_dist_m:6.1f}m  wp_brg={self.nav_bearing_deg:6.1f}deg"
        )


class SitlClient:
    def __init__(self, conn_str: str):
        print(f"[*] Connecting to {conn_str} ...")
        self.master = mavutil.mavlink_connection(
            conn_str, source_system=255, source_component=0
        )
        hb = self.master.wait_heartbeat(timeout=10)
        if hb is None or self.master.target_system == 0:
            raise SystemExit(
                f"\n[!] No heartbeat from {conn_str} within 10s.\n"
                f"    SITL up? Try:  py tests\\sitl_local_control_test.py tcp:127.0.0.1:5760\n"
            )
        print(
            f"[+] Heartbeat from sys={self.master.target_system} "
            f"comp={self.master.target_component}"
        )
        self.tlm = TelemetrySnapshot()
        self.last_cmd: dict | None = None
        self._last_printed_statustext = ""
        self._stop = threading.Event()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def close(self) -> None:
        self._stop.set()
        self._rx_thread.join(timeout=1.0)

    # ----- background receiver -----
    def _rx_loop(self) -> None:
        while not self._stop.is_set():
            msg = self.master.recv_match(blocking=True, timeout=0.5)
            if msg is None:
                continue
            t = msg.get_type()
            if t == "HEARTBEAT":
                # SITL emits heartbeats from multiple components (e.g. comp=0
                # router stub vs comp=1 real autopilot). Only the autopilot
                # carries the real mode/armed state; ignore the rest so the
                # display doesn't flicker to MANUAL/disarmed mid-flight.
                src_comp = msg.get_srcComponent()
                is_autopilot = (
                    msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID
                    and src_comp == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
                )
                if not is_autopilot:
                    continue
                self.tlm.armed = bool(
                    msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )
                ms = mavutil.mode_string_v10(msg)
                # mode_string_v10 returns 'Mode(0x00000000)' for unknown
                # custom modes (or when CUSTOM bit not set). Fall back to a
                # plane-specific table.
                if ms.startswith("Mode("):
                    plane_modes = {
                        0: "MANUAL", 1: "CIRCLE", 2: "STABILIZE", 3: "TRAINING",
                        4: "ACRO", 5: "FBWA", 6: "FBWB", 7: "CRUISE", 8: "AUTOTUNE",
                        10: "AUTO", 11: "RTL", 12: "LOITER", 13: "TAKEOFF",
                        14: "AVOID_ADSB", 15: "GUIDED", 17: "QSTABILIZE",
                        18: "QHOVER", 19: "QLOITER", 20: "QLAND", 21: "QRTL",
                        22: "QAUTOTUNE", 23: "QACRO", 24: "THERMAL",
                    }
                    ms = plane_modes.get(msg.custom_mode, f"M{msg.custom_mode}")
                self.tlm.mode = ms
            elif t == "GLOBAL_POSITION_INT":
                self.tlm.lat = msg.lat / 1e7
                self.tlm.lon = msg.lon / 1e7
                self.tlm.alt_rel = msg.relative_alt / 1000.0
                self.tlm.heading_deg = msg.hdg / 100.0
            elif t == "VFR_HUD":
                self.tlm.airspeed = msg.airspeed
            elif t == "NAV_CONTROLLER_OUTPUT":
                # ArduPlane fills target_bearing/alt_error/wp_dist in GUIDED.
                # alt_error positive = need to climb. We reconstruct target alt.
                self.tlm.nav_bearing_deg = float(msg.target_bearing)
                self.tlm.wp_dist_m = float(msg.wp_dist)
                if msg.wp_dist > 0 and msg.wp_dist < self.tlm.min_wp_dist_m:
                    self.tlm.min_wp_dist_m = float(msg.wp_dist)
                if not math.isnan(self.tlm.alt_rel):
                    self.tlm.nav_alt_target_m = self.tlm.alt_rel + msg.alt_error
            elif t == "STATUSTEXT":
                text = msg.text.strip()
                self.tlm.last_statustext = text
                # de-dup: don't reprint the same line back-to-back. The fork's
                # ArduPlane fence patch already silences repeating "fence
                # breached" texts when FENCE_ACTION=REPORT_ONLY, so this is
                # only a guard against generic duplicate notices.
                if text != self._last_printed_statustext:
                    print(f"\n  [STATUSTEXT sev={msg.severity}] {text}")
                    self._last_printed_statustext = text

    # ----- command helpers -----
    def set_mode(self, mode_name: str) -> None:
        mode_id = self.master.mode_mapping().get(mode_name)
        if mode_id is None:
            print(f"  ! unknown mode: {mode_name}")
            return
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id, 0, 0, 0, 0, 0,
        )
        print(f"  -> SET_MODE {mode_name}")

    def arm(self) -> None:
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0,
        )
        print("  -> ARM")

    def takeoff(self, altitude_m: float) -> None:
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            10.0, 0, 0, float("nan"), 0, 0, altitude_m,
        )
        print(f"  -> TAKEOFF alt={altitude_m}m")

    def send_local_ned_target(
        self, frame: int, x: float, y: float, z: float,
        yaw_deg: float | None = None,
    ) -> None:
        """SET_POSITION_TARGET_LOCAL_NED, position-only with optional yaw.

        Records the command into self.last_cmd so `track` can verify the
        actual displacement that follows.
        """
        if yaw_deg is None or math.isnan(yaw_deg):
            mask = MASK_POS_ONLY
            yaw_rad = 0.0
        else:
            mask = MASK_POS_YAW
            yaw_rad = math.radians(yaw_deg)

        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system, self.master.target_component,
            frame, mask,
            x, y, z,
            0, 0, 0,
            0, 0, 0,
            yaw_rad, 0,
        )
        frame_name = {
            FRAME_BODY_OFFSET_NED: "BODY_OFFSET_NED",
            FRAME_BODY_NED: "BODY_NED",
            FRAME_LOCAL_OFFSET_NED: "LOCAL_OFFSET_NED",
            FRAME_LOCAL_NED: "LOCAL_NED",
        }.get(frame, f"frame={frame}")
        yaw_s = "NaN" if yaw_deg is None or math.isnan(yaw_deg) else f"{yaw_deg:.1f}deg"
        print(f"  -> LOCAL_NED [{frame_name}] x={x:+.1f} y={y:+.1f} z={z:+.1f} yaw={yaw_s}")

        # reset min-wp-dist tracker so the next observation window measures
        # how close we got to THIS waypoint, not a previous one.
        self.tlm.min_wp_dist_m = float("inf")

        # remember last commanded offset (resolved into NED) for `track`
        if frame in (FRAME_BODY_OFFSET_NED, FRAME_BODY_NED):
            yaw_now = math.radians(self.tlm.heading_deg) if not math.isnan(self.tlm.heading_deg) else 0.0
            n = math.cos(yaw_now) * x - math.sin(yaw_now) * y
            e = math.sin(yaw_now) * x + math.cos(yaw_now) * y
            d = z
        else:
            n, e, d = x, y, z
        self.last_cmd = {
            "frame": frame_name,
            "raw": (x, y, z),
            "ned": (n, e, d),
            "yaw_deg": yaw_deg,
            "lat0": self.tlm.lat,
            "lon0": self.tlm.lon,
            "alt0": self.tlm.alt_rel,
            "hdg0": self.tlm.heading_deg,
        }

    # ----- SITL parameter helpers (for fence test, etc.) -----
    def set_param(self, name: str, value: float, ptype: int | None = None) -> None:
        if ptype is None:
            ptype = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        self.master.mav.param_set_send(
            self.master.target_system, self.master.target_component,
            name.encode("ascii"), float(value), ptype,
        )
        print(f"  -> PARAM_SET {name} = {value}")

    def configure_fence(self, radius_m: float = 5000.0) -> None:
        """Configure (but do NOT enable) a circular fence. Lets us toggle
        FENCE_ENABLE on/off later without re-sending all params."""
        self.set_param("FENCE_TYPE",   2, mavutil.mavlink.MAV_PARAM_TYPE_INT8)  # circle
        self.set_param("FENCE_RADIUS", radius_m)
        self.set_param("FENCE_ACTION", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT8)  # report only
        self.set_param("FENCE_ENABLE", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT8)  # OFF by default
        time.sleep(0.3)

    def fence_enable(self, enable: bool) -> None:
        self.set_param("FENCE_ENABLE", 1 if enable else 0,
                       mavutil.mavlink.MAV_PARAM_TYPE_INT8)
        time.sleep(0.2)

    def track(self, seconds: float = 8.0) -> None:
        """After the last send, watch position for `seconds` and report
        actual N/E/D displacement vs. the commanded offset."""
        if not getattr(self, "last_cmd", None):
            print("  ! no command sent yet -- run a scenario first, then 'tr'.")
            return
        cmd = self.last_cmd
        if math.isnan(cmd["lat0"]) or math.isnan(cmd["lon0"]):
            print("  ! no GPS fix when command was sent; cannot measure displacement.")
            return

        n_cmd, e_cmd, d_cmd = cmd["ned"]
        print(f"  Tracking for {seconds:.1f}s. Commanded NED: "
              f"N={n_cmd:+.1f} E={e_cmd:+.1f} D={d_cmd:+.1f}m  "
              f"(frame {cmd['frame']}, yaw_at_send={cmd['hdg0']:.1f}deg)")

        t_end = time.time() + seconds
        while time.time() < t_end:
            time.sleep(1.0)
            if math.isnan(self.tlm.lat):
                continue
            n, e = latlon_to_ne(cmd["lat0"], cmd["lon0"], self.tlm.lat, self.tlm.lon)
            d = -(self.tlm.alt_rel - cmd["alt0"])  # down positive
            remaining = t_end - time.time()
            print(f"   t-{remaining:4.1f}s  actual NED: "
                  f"N={n:+6.1f} E={e:+6.1f} D={d:+6.1f}m   "
                  f"hdg={self.tlm.heading_deg:6.1f}deg  alt={self.tlm.alt_rel:6.1f}m  "
                  f"wp_dist={self.tlm.wp_dist_m:6.1f}m")

        # final summary
        if math.isnan(self.tlm.lat):
            print("  ! lost telemetry, no final summary")
            return
        n, e = latlon_to_ne(cmd["lat0"], cmd["lon0"], self.tlm.lat, self.tlm.lon)
        d = -(self.tlm.alt_rel - cmd["alt0"])
        print()
        print(f"  ===== summary =====")
        print(f"  cmd  NED : N={n_cmd:+7.1f}  E={e_cmd:+7.1f}  D={d_cmd:+7.1f}  m")
        print(f"  real NED : N={n:+7.1f}  E={e:+7.1f}  D={d:+7.1f}  m   ({seconds:.0f}s after send)")
        # signed projection of actual onto commanded direction
        norm_cmd = math.sqrt(n_cmd*n_cmd + e_cmd*e_cmd + d_cmd*d_cmd)
        if norm_cmd > 0.1:
            proj = (n*n_cmd + e*e_cmd + d*d_cmd) / norm_cmd
            print(f"  along-cmd projection: {proj:+.1f} m  ({100*proj/norm_cmd:+.0f}% of commanded magnitude)")
        if cmd["yaw_deg"] is not None and not math.isnan(cmd["yaw_deg"]):
            dyaw = (self.tlm.heading_deg - cmd["yaw_deg"] + 540) % 360 - 180
            print(f"  heading  : commanded {cmd['yaw_deg']:.1f}deg, now {self.tlm.heading_deg:.1f}deg, error {dyaw:+.1f}deg")


# ---------- nine-scenario menu ----------

def scenario_1_forward(c: SitlClient) -> None:
    """Sanity: 50 m forward in body frame."""
    c.send_local_ned_target(FRAME_BODY_OFFSET_NED, 50, 0, 0)


def scenario_2_right(c: SitlClient) -> None:
    """30 m to the right."""
    c.send_local_ned_target(FRAME_BODY_OFFSET_NED, 0, 30, 0)


def scenario_2b_left(c: SitlClient) -> None:
    """30 m to the left."""
    c.send_local_ned_target(FRAME_BODY_OFFSET_NED, 0, -30, 0)


def scenario_3_climb(c: SitlClient) -> None:
    """20 m climb (down=-20)."""
    c.send_local_ned_target(FRAME_BODY_OFFSET_NED, 0, 0, -20)


def scenario_3b_descend(c: SitlClient) -> None:
    """20 m descent."""
    c.send_local_ned_target(FRAME_BODY_OFFSET_NED, 0, 0, 20)


def scenario_4_compound(c: SitlClient) -> None:
    """80 m forward, 40 m right, 10 m climb, yaw 045."""
    c.send_local_ned_target(FRAME_BODY_OFFSET_NED, 80, 40, -10, yaw_deg=45.0)


def scenario_5_fence(c: SitlClient) -> None:
    """10 km forward -> should be REJECTED if a fence is set."""
    c.send_local_ned_target(FRAME_BODY_OFFSET_NED, 10000, 0, 0)


def scenario_6_local_offset(c: SitlClient) -> None:
    """LOCAL_OFFSET_NED: 100 m north, 50 m east."""
    c.send_local_ned_target(FRAME_LOCAL_OFFSET_NED, 100, 50, 0)


def scenario_7_local_ned(c: SitlClient) -> None:
    """LOCAL_NED (alias of LOCAL_OFFSET in our handler): 100 m north."""
    c.send_local_ned_target(FRAME_LOCAL_NED, 100, 0, 0)


def scenario_8_polar_check(c: SitlClient) -> None:
    """Bearing 045 / dist 141 m / +20 m alt -> equivalent to (N=100, E=100, D=-20)."""
    bearing = 45.0
    dist = 141.0
    alt_off = 20.0
    n = math.cos(math.radians(bearing)) * dist
    e = math.sin(math.radians(bearing)) * dist
    d = -alt_off
    print(f"  (polar check: bearing={bearing} dist={dist} -> N={n:.1f} E={e:.1f} D={d:.1f})")
    c.send_local_ned_target(FRAME_LOCAL_OFFSET_NED, n, e, d)


def scenario_10_qr_perception_lock(c: SitlClient) -> None:
    """Simulate the perception-driven QR_DIVE terminal phase end-to-end.

    Mirrors what NavigationEngine.dispatchPerceptionTarget produces when
    SimpleQRDetector emits a fresh lock:
      pulse 1 -> VISION_LOS-style BODY_OFFSET (yaw_off=+10deg, pitch=-5deg, range=80m)
      pulse 2 -> world-pose LOCAL_NED  (north=+60, east=+20, down=-5)
    The aircraft should steer toward each in turn (no stale yaw bleed-over),
    proving that frame-aware dispatch correctly switches between body and
    local frames mid-flight (the key Phase 1f integration test).
    """
    yaw_off = 10.0
    pitch_off = -5.0
    rng = 80.0
    forward = math.cos(math.radians(yaw_off)) * math.cos(math.radians(pitch_off)) * rng
    right   = math.sin(math.radians(yaw_off)) * math.cos(math.radians(pitch_off)) * rng
    down    = -math.sin(math.radians(pitch_off)) * rng
    print(f"  (perception pulse 1: VISION_LOS yaw=+{yaw_off:.0f} pitch={pitch_off:.0f} rng={rng:.0f}m"
          f"  -> body fwd={forward:+.1f} right={right:+.1f} down={down:+.1f})")
    c.send_local_ned_target(FRAME_BODY_OFFSET_NED, forward, right, down)


def scenario_11_qr_perception_world(c: SitlClient) -> None:
    """Second half of the perception sequence: same target now reported in
    LOCAL_NED (world-pose). dispatchPerceptionTarget routes this through
    flyLocalOffset, which must reach the firmware as LOCAL_OFFSET_NED with
    yaw cleared so the previous BODY-frame yaw target does not leak in."""
    n, e, d = 60.0, 20.0, -5.0
    print(f"  (perception pulse 2: LOCAL_NED N={n:+.0f} E={e:+.0f} D={d:+.0f})")
    c.send_local_ned_target(FRAME_LOCAL_OFFSET_NED, n, e, d)


def scenario_12_follow_vision_pulse(c: SitlClient) -> None:
    """FollowMode vision-fallback pulse: pretends world-model lost the
    target, vision sees it +20deg right of bore-sight, ~150m out, slightly
    high. Equivalent to one pursueVisionTarget call from
    NavigationEngine.dispatchPerceptionTarget."""
    yaw_off = 20.0
    pitch_off = -3.0
    rng = 150.0
    forward = math.cos(math.radians(yaw_off)) * math.cos(math.radians(pitch_off)) * rng
    right   = math.sin(math.radians(yaw_off)) * math.cos(math.radians(pitch_off)) * rng
    down    = -math.sin(math.radians(pitch_off)) * rng
    print(f"  (follow vision pulse: yaw=+{yaw_off:.0f} pitch={pitch_off:.0f} rng={rng:.0f}m"
          f"  -> body fwd={forward:+.1f} right={right:+.1f} down={down:+.1f})")
    c.send_local_ned_target(FRAME_BODY_OFFSET_NED, forward, right, down)


def scenario_13_release_guided_hold(c: SitlClient) -> None:
    """Reproduces NavigationEngine.releaseGuidedHold(): zero-magnitude
    BODY_OFFSET_NED with YAW_IGNORE so the firmware drops any sticky
    guided_state.target_heading from the previous scenario. After this the
    next regular waypoint should be tracked cleanly (no stale yaw fight)."""
    print("  (releaseGuidedHold: 0/0/0 BODY_OFFSET, yaw cleared)")
    c.send_local_ned_target(FRAME_BODY_OFFSET_NED, 0.0, 0.0, 0.0)


def scenario_9_qr_dive_step(c: SitlClient) -> None:
    """Single step like QRDiveMode.tickDiveVelocity: forward + descent."""
    pitch_deg = 45.0
    speed = 25.0
    step_s = 0.5
    fwd = math.cos(math.radians(pitch_deg)) * speed * step_s
    down = math.sin(math.radians(pitch_deg)) * speed * step_s
    print(f"  (qr_dive step: fwd={fwd:.1f} down={down:.1f} (45° dive @ 25 m/s, 0.5s slice))")
    c.send_local_ned_target(FRAME_BODY_OFFSET_NED, fwd, 0, down)


SCENARIOS = {
    "1": ("forward 50m (body)", scenario_1_forward),
    "2": ("right 30m (body)", scenario_2_right),
    "2b": ("left 30m (body)", scenario_2b_left),
    "3": ("climb 20m", scenario_3_climb),
    "3b": ("descend 20m", scenario_3b_descend),
    "4": ("compound: fwd+right+climb+yaw45", scenario_4_compound),
    "5": ("FENCE test: 10km forward (expect REJECT)", scenario_5_fence),
    "6": ("LOCAL_OFFSET: 100N 50E", scenario_6_local_offset),
    "7": ("LOCAL_NED: 100N", scenario_7_local_ned),
    "8": ("polar 045/141m equivalence check", scenario_8_polar_check),
    "9": ("qr_dive single step (body fwd+down)", scenario_9_qr_dive_step),
    "10": ("qr_dive perception VISION_LOS pulse", scenario_10_qr_perception_lock),
    "11": ("qr_dive perception LOCAL_NED pulse", scenario_11_qr_perception_world),
    "12": ("follow vision-fallback pulse", scenario_12_follow_vision_pulse),
    "13": ("releaseGuidedHold (0/0/0 + yaw clear)", scenario_13_release_guided_hold),
}


# ---------- automated test runner ----------
#
# Per-scenario acceptance:
#   "kind"        : how to evaluate
#                       lateral  -> projection along commanded NE >= min_proj_frac of |NE_cmd|
#                       vertical -> alt change in commanded D direction >= min_alt_frac of |D_cmd|
#                       compound -> lateral progress AND heading approaches target yaw
#                       reject   -> STATUSTEXT containing one of reject_keys within reject_window_s
#                       polar    -> like lateral, then we'll cross-check vs scenario 1/7
#   "wait_s"      : how long to observe after sending the command
#   "min_proj_frac": fraction of commanded magnitude considered "moving correctly"
#                    (fixed-wing has ~80-150 m turn radius, so we don't expect 100%)
#
AUTO_TESTS = [
    # name, scenario_key, kind, wait_s, min_frac, reject_keys
    ("forward 50m (body)",                "1",  "lateral",  15.0, 0.30, None),
    ("right 30m (body)",                  "2",  "lateral",  18.0, 0.20, None),
    ("left 30m (body)",                   "2b", "lateral",  18.0, 0.20, None),
    ("climb 20m",                         "3",  "vertical", 18.0, 0.40, None),
    ("descend 20m",                       "3b", "vertical", 18.0, 0.40, None),
    ("compound 80/40/-10 yaw45",          "4",  "compound", 25.0, 0.25, None),
    ("FENCE 10km (expect REJECT)",        "5",  "reject",    3.0, 0.0,  ("fence", "outside", "reject", "denied")),
    ("LOCAL_OFFSET 100N 50E",             "6",  "lateral",  20.0, 0.30, None),
    ("LOCAL_NED 100N",                    "7",  "lateral",  20.0, 0.30, None),
    ("polar 045/141 equivalence",         "8",  "lateral",  22.0, 0.30, None),
    ("qr_dive single step",               "9",  "lateral",   8.0, 0.20, None),
    # ---- Phase 1f perception integration ----
    # qr_dive perception_lock: BODY pulse, then LOCAL pulse - verifies
    # frame-aware dispatchPerceptionTarget switches frames cleanly without
    # leaking yaw/state from the previous one.
    ("qr_dive_perception_lock VISION",    "10", "lateral",  18.0, 0.20, None),
    ("qr_dive_perception_lock LOCAL",     "11", "lateral",  20.0, 0.30, None),
    # follow vision fallback pulse: single VISION_LOS-style command,
    # verifying pursueVisionTarget routing.
    ("follow_vision_fallback pulse",      "12", "lateral",  22.0, 0.20, None),
]


def _ensure_airborne(c: SitlClient, takeoff_alt: float = 100.0) -> bool:
    """Make sure aircraft is in GUIDED + armed + above 50 m.
    Returns True if ready, False if it gave up after timeout."""
    if c.tlm.mode != "GUIDED":
        c.set_mode("GUIDED")
        time.sleep(1.0)
    if not c.tlm.armed:
        c.arm()
        time.sleep(2.0)
    if c.tlm.alt_rel < 50.0 or math.isnan(c.tlm.alt_rel):
        c.takeoff(takeoff_alt)
        # wait up to 60s to climb past 80% of target
        t_end = time.time() + 60.0
        while time.time() < t_end:
            time.sleep(2.0)
            if c.tlm.alt_rel >= takeoff_alt * 0.8:
                break
            print(f"   ... climbing, alt={c.tlm.alt_rel:.1f}m mode={c.tlm.mode}")
    return (c.tlm.mode == "GUIDED" and c.tlm.armed and c.tlm.alt_rel >= 50.0)


def _evaluate(c: SitlClient, kind: str, wait_s: float, min_frac: float,
              reject_keys: tuple | None) -> tuple[str, str]:
    """Watch the aircraft for `wait_s` and judge whether it tracked the
    last commanded offset. Returns (verdict, detail).

    Methodology for fixed-wing (non-trivial because the plane can't hover):
      lateral  -> PASS if either
                    (a) min wp_dist during window <= max(0.4*|cmd_NE|, 30 m)
                        i.e. the plane actually got close to the synthetic WP, OR
                    (b) projection along commanded NE >= min_frac of |cmd_NE|
                        (handles the case where the WP was reached and overshot)
      vertical -> PASS if alt change in commanded D direction >= min_frac
      compound -> PASS if (lateral PASS) AND heading approached target yaw
      reject   -> PASS if any reject_key appears in STATUSTEXT within wait_s
    """
    cmd = c.last_cmd
    if cmd is None:
        return ("FAIL", "no command was sent")

    n_cmd, e_cmd, d_cmd = cmd["ned"]

    if kind == "reject":
        # watch for STATUSTEXT containing any of the keys, also reset baseline
        keys = reject_keys or ()
        baseline = c.tlm.last_statustext
        t_end = time.time() + wait_s
        while time.time() < t_end:
            time.sleep(0.2)
            seen = c.tlm.last_statustext
            if seen != baseline:
                low = seen.lower()
                if any(k in low for k in keys):
                    return ("PASS", f"rejected: '{seen}'")
        return ("FAIL", f"no reject statustext within {wait_s}s (last='{c.tlm.last_statustext}')")

    time.sleep(wait_s)
    if math.isnan(c.tlm.lat) or math.isnan(cmd["lat0"]):
        return ("FAIL", "no GPS fix")

    n, e = latlon_to_ne(cmd["lat0"], cmd["lon0"], c.tlm.lat, c.tlm.lon)
    d = -(c.tlm.alt_rel - cmd["alt0"])
    min_wp = c.tlm.min_wp_dist_m

    if kind == "lateral":
        ne_norm = math.hypot(n_cmd, e_cmd)
        if ne_norm < 1.0:
            return ("FAIL", "commanded NE near zero")
        proj = (n * n_cmd + e * e_cmd) / ne_norm
        frac = max(min(proj / ne_norm, 1.5), -1.5)  # clamp display to +/-150%
        reach_thr = max(0.4 * ne_norm, 30.0)
        reached = (min_wp <= reach_thr)
        ok = reached or (proj >= min_frac * ne_norm)
        detail = (f"NE_cmd={ne_norm:.0f}m  along={proj:+.0f}m ({frac*100:+.0f}%)  "
                  f"min_wp={min_wp:.0f}m  reach<={reach_thr:.0f}m -> "
                  f"{'reached' if reached else 'not-reached'}")
        return ("PASS" if ok else "FAIL", detail)

    if kind == "vertical":
        if abs(d_cmd) < 1.0:
            return ("FAIL", "commanded D near zero")
        frac = (d * d_cmd) / (d_cmd * d_cmd)
        detail = (f"D_cmd={d_cmd:+.1f}m  actual={d:+.1f}m ({frac*100:+.0f}%)  "
                  f"alt={c.tlm.alt_rel:.1f}m")
        return ("PASS" if frac >= min_frac else "FAIL", detail)

    if kind == "compound":
        ne_norm = math.hypot(n_cmd, e_cmd)
        proj = (n * n_cmd + e * e_cmd) / ne_norm if ne_norm > 1.0 else 0.0
        lat_frac = proj / ne_norm if ne_norm > 1.0 else 0.0
        lat_frac_disp = max(min(lat_frac, 1.5), -1.5)
        reach_thr = max(0.4 * ne_norm, 30.0)
        reached = (min_wp <= reach_thr)
        lat_ok = reached or (lat_frac >= min_frac)
        yaw_ok = True
        yaw_detail = ""
        if cmd["yaw_deg"] is not None and not math.isnan(cmd["yaw_deg"]):
            err = (c.tlm.heading_deg - cmd["yaw_deg"] + 540) % 360 - 180
            yaw_ok = abs(err) < 30.0
            yaw_detail = f"  yaw_err={err:+.0f}deg"
        detail = (f"NE={ne_norm:.0f}m along={proj:+.0f}m ({lat_frac_disp*100:+.0f}%)  "
                  f"min_wp={min_wp:.0f}m -> {'reached' if reached else 'not-reached'}  "
                  f"D={d:+.1f}/{d_cmd:+.1f}m{yaw_detail}")
        return ("PASS" if (lat_ok and yaw_ok) else "FAIL", detail)

    return ("FAIL", f"unknown kind '{kind}'")


def _settle(c: SitlClient, max_wait_s: float = 8.0) -> None:
    """Between tests: send a 'go back toward home' command so the previous
    WP overshoot momentum is cleared and the plane stays inside the fence.

    The home point is captured the first time this is called.
    """
    if math.isnan(c.tlm.lat):
        time.sleep(2.0)
        return
    if not hasattr(c, "_home_lat") or c._home_lat is None:
        c._home_lat = c.tlm.lat
        c._home_lon = c.tlm.lon
        c._home_alt = c.tlm.alt_rel
    # vector from current pos back to home in NE (meters)
    n, e = latlon_to_ne(c.tlm.lat, c.tlm.lon,
                        c._home_lat, c._home_lon)
    dist = math.hypot(n, e)
    if dist > 80.0:
        # head toward home (refresh halfway through so plane keeps committing
        # to the inbound vector even after it turns)
        scale = min(1.0, 200.0 / dist)
        c.send_local_ned_target(FRAME_LOCAL_OFFSET_NED, n * scale, e * scale, 0.0)
        time.sleep(max_wait_s * 0.5)
        # second nudge with fresh n,e from new position
        if not math.isnan(c.tlm.lat):
            n2, e2 = latlon_to_ne(c.tlm.lat, c.tlm.lon, c._home_lat, c._home_lon)
            d2 = math.hypot(n2, e2)
            if d2 > 80.0:
                s2 = min(1.0, 200.0 / d2)
                c.send_local_ned_target(FRAME_LOCAL_OFFSET_NED, n2 * s2, e2 * s2, 0.0)
        time.sleep(max_wait_s * 0.5)
    else:
        # near home -- just keep flying current heading
        c.send_local_ned_target(FRAME_BODY_OFFSET_NED, 100.0, 0.0, 0.0)
        time.sleep(max_wait_s)


def run_auto_tests(c: SitlClient, repeat: int = 1, only: list | None = None) -> None:
    """Full PASS/FAIL matrix. `only` can restrict to a subset of scenario keys."""
    print("\n" + "=" * 64)
    print("  AUTO TEST RUN")
    print("=" * 64)

    if not _ensure_airborne(c):
        print("  ! could not get GUIDED + airborne; aborting auto run")
        return

    # configure a fence (radius 5km, report-only) but leave it DISABLED.
    # We only flip it ON for the FENCE-reject scenario, so the rest of the
    # run is silent (no 'Circle fence breached' spam).
    print("\n[setup] configuring 5000 m fence (off by default, on only for FENCE test)")
    c.configure_fence(5000.0)

    results = []
    tests = AUTO_TESTS if not only else [t for t in AUTO_TESTS if t[1] in only]
    for cycle in range(repeat):
        if repeat > 1:
            print(f"\n--- cycle {cycle + 1} / {repeat} ---")
        for idx, (name, key, kind, wait_s, min_frac, reject_keys) in enumerate(tests):
            if idx > 0:
                print(f"  [settle] holding pattern between tests ...")
                _settle(c, max_wait_s=6.0)
            print(f"\n[{name}]  ({kind}, {wait_s:.0f}s)")
            print(f"  state: {c.tlm.fmt()}")
            desc, fn = SCENARIOS[key]

            # toggle fence ON only for the reject scenario
            if kind == "reject":
                print("  [fence ON] enabling fence to verify reject path")
                c.fence_enable(True)

            fn(c)
            verdict, detail = _evaluate(c, kind, wait_s, min_frac, reject_keys)

            if kind == "reject":
                c.fence_enable(False)
                print("  [fence OFF] disabled to silence breach warnings")

            mark = "[ OK ]" if verdict == "PASS" else "[FAIL]"
            print(f"  {mark} {detail}")
            results.append((cycle, name, verdict, detail))

    print("\n" + "=" * 64)
    print("  RESULTS")
    print("=" * 64)
    n_pass = sum(1 for r in results if r[2] == "PASS")
    n_fail = len(results) - n_pass
    for cyc, name, verdict, detail in results:
        prefix = f"c{cyc + 1} " if repeat > 1 else ""
        print(f"  {prefix}{verdict:4}  {name:38} | {detail}")
    print("-" * 64)
    print(f"  TOTAL: {n_pass} PASS / {n_fail} FAIL  ({len(results)} runs)")
    print("=" * 64)


def run_perception_sequence(c: SitlClient) -> None:
    """End-to-end perception scenario chain that mimics Phase 1f flow:

      1. forward (steady cruise reference)
      2. VISION_LOS pulse  (BODY frame  -- pursueVisionTarget path)
      3. LOCAL_NED pulse   (world-pose  -- flyLocalOffset path)
      4. releaseGuidedHold (resets sticky guided_state.target_heading)
      5. follow vision pulse (BODY frame again, no leakover from step 4)

    PASS criteria: every step except (4) reaches its WP or projects >= 20%
    along the commanded vector. Step (4) is a state-reset, not motion, and
    is only checked for "no STATUSTEXT error" within 1 s.
    """
    print("\n" + "=" * 64)
    print("  PERCEPTION INTEGRATION SEQUENCE")
    print("=" * 64)

    if not _ensure_airborne(c):
        print("  ! could not get GUIDED + airborne; aborting")
        return

    c.configure_fence(5000.0)

    sequence = [
        ("baseline forward 50m",         "1",  "lateral",  12.0, 0.20),
        ("perception VISION_LOS pulse",  "10", "lateral",  18.0, 0.20),
        ("perception LOCAL_NED pulse",   "11", "lateral",  20.0, 0.25),
        ("releaseGuidedHold",            "13", None,        2.0, 0.0),
        ("follow vision-fallback pulse", "12", "lateral",  22.0, 0.20),
    ]

    results = []
    for idx, (name, key, kind, wait_s, min_frac) in enumerate(sequence):
        if idx > 0:
            print(f"  [settle]")
            _settle(c, max_wait_s=5.0)
        print(f"\n[seq {idx + 1}] {name}")
        print(f"  state: {c.tlm.fmt()}")
        desc, fn = SCENARIOS[key]
        baseline_status = c.tlm.last_statustext
        fn(c)
        if kind is None:
            time.sleep(wait_s)
            new_status = c.tlm.last_statustext
            err = (new_status != baseline_status and
                   any(k in new_status.lower() for k in ("error", "rejected", "fail", "denied")))
            verdict = "FAIL" if err else "PASS"
            detail = f"post-status='{new_status}'"
        else:
            verdict, detail = _evaluate(c, kind, wait_s, min_frac, None)
        mark = "[ OK ]" if verdict == "PASS" else "[FAIL]"
        print(f"  {mark} {detail}")
        results.append((name, verdict, detail))

    print("\n" + "=" * 64)
    print("  PERCEPTION SEQUENCE RESULTS")
    print("=" * 64)
    n_pass = sum(1 for r in results if r[1] == "PASS")
    for name, verdict, detail in results:
        print(f"  {verdict:4}  {name:36} | {detail}")
    print("-" * 64)
    print(f"  TOTAL: {n_pass}/{len(results)} PASS")
    print("=" * 64)


def run_loop_tests(c: SitlClient) -> None:
    """Sürekli döngüde senaryo gönderir; her birini doğrular. Ctrl+C ile dur."""
    print("\n" + "=" * 64)
    print("  LOOP MODE — Ctrl+C to stop")
    print("=" * 64)
    if not _ensure_airborne(c):
        print("  ! could not get GUIDED + airborne; aborting loop")
        return

    n_total = 0
    n_pass = 0
    try:
        while True:
            for name, key, kind, wait_s, min_frac, reject_keys in AUTO_TESTS:
                if kind == "reject":
                    continue  # skip fence test in loop, it doesn't move the plane
                print(f"  [settle]")
                _settle(c, max_wait_s=5.0)
                print(f"\n[loop #{n_total + 1}] {name}")
                print(f"  state: {c.tlm.fmt()}")
                desc, fn = SCENARIOS[key]
                fn(c)
                verdict, detail = _evaluate(c, kind, wait_s, min_frac, reject_keys)
                mark = "[ OK ]" if verdict == "PASS" else "[FAIL]"
                print(f"  {mark} {detail}")
                n_total += 1
                if verdict == "PASS":
                    n_pass += 1
                print(f"  running tally: {n_pass}/{n_total} pass "
                      f"({100 * n_pass / n_total:.0f}%)")
    except KeyboardInterrupt:
        print()
        print(f"\n[stopped] final tally: {n_pass}/{n_total} pass")


HELP = """
================================================================
  SITL local-control test harness
================================================================
  Setup:
    g    GUIDED mode
    a    ARM throttle
    t    TAKEOFF to 100m
    s    show telemetry snapshot

  Scenarios (need GUIDED + airborne):
    1    forward 50m (body)
    2    right 30m (body)              2b  left 30m
    3    climb 20m                      3b  descend 20m
    4    compound 80/40/-10 yaw 045
    5    FENCE test 10km forward (expect REJECT)
    6    LOCAL_OFFSET 100N 50E
    7    LOCAL_NED 100N
    8    polar 045/141m equivalence check
    9    qr_dive single step

  Phase 1f perception integration:
    10   QR_DIVE perception VISION_LOS pulse (BODY frame)
    11   QR_DIVE perception LOCAL_NED pulse  (world-pose)
    12   FOLLOW vision-fallback pulse        (BODY frame)
    13   releaseGuidedHold                   (state reset)

  Verification:
    tr   track last command for 8s and report actual N/E/D drift

  Automated:
    auto         run full PASS/FAIL test matrix once (auto-arm + takeoff)
    auto N       run matrix N times and aggregate
    perception   run end-to-end perception sequence (steps 1-12 chained)
    loop         continuously cycle scenarios (Ctrl+C to stop)

    h    help        q    quit
================================================================
"""


def main() -> int:
    conn_str = "tcp:127.0.0.1:5760"
    if len(sys.argv) > 1:
        a = sys.argv[1]
        if a.isdigit():
            conn_str = f"udpin:127.0.0.1:{a}"
        else:
            conn_str = a

    client = SitlClient(conn_str)
    print(HELP)

    try:
        while True:
            cmd = input(f"\n[{client.tlm.fmt()}]\n> ").strip().lower()
            if not cmd:
                continue
            if cmd == "q":
                break
            elif cmd == "h":
                print(HELP)
            elif cmd == "g":
                client.set_mode("GUIDED")
            elif cmd == "a":
                client.arm()
            elif cmd == "t":
                client.takeoff(100.0)
            elif cmd == "s":
                print(f"  {client.tlm.fmt()}")
                print(f"  lat={client.tlm.lat} lon={client.tlm.lon}")
                if client.tlm.last_statustext:
                    print(f"  last STATUSTEXT: {client.tlm.last_statustext}")
            elif cmd in ("tr", "track"):
                client.track(8.0)
            elif cmd == "auto" or cmd.startswith("auto "):
                parts = cmd.split()
                repeat = 1
                if len(parts) > 1 and parts[1].isdigit():
                    repeat = max(1, int(parts[1]))
                run_auto_tests(client, repeat=repeat)
            elif cmd == "perception":
                run_perception_sequence(client)
            elif cmd == "loop":
                run_loop_tests(client)
            elif cmd in SCENARIOS:
                desc, fn = SCENARIOS[cmd]
                print(f"-- {desc}")
                fn(client)
                # short wait so we surface any STATUSTEXT (e.g. fence reject)
                time.sleep(0.6)
            else:
                print(f"  ? unknown command '{cmd}'   (h for help)")
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        client.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
