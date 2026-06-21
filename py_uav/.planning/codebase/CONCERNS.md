# Concerns & Technical Debt

**Analysis Date:** 2026-06-21

Severity reflects impact on the project's **Core Value** — *competition-valid,
server-time-stamped scoring packets*. A perfectly-flown mission scores **zero**
if the packet is malformed or wrongly timestamped.

## CRITICAL — Scoring Data Contract

### C1. Server clock is never synced in production
**Files:** `src/uav/comm/downlink.py:36-60`, `src/uav/comm/statustext_parser.py`
`DownlinkEncoder.sync_server_clock()` exists and is unit-tested
(`tests/test_tracking.py:209,228,631`) — but **no production code path ever calls
it.** `grep` finds callers only in tests. Worse, the **uplink parser
(`statustext_parser.py`) has no message type that carries a server timestamp** —
it handles only `RIV / HSS / BND / FREE / ARDUPILOT / LOCK / KAMIKAZE`. So there
is no channel by which the server clock could reach the encoder even if a call
site existed.
**Consequence:** `_ServerClock.offset_ms` stays `0.0`, so every `encode_*`
timestamp is **local `time.time()`**, not server time. Şartname 6.1.3 requires
server-clock event times → packets risk being scored invalid.
**Fix direction:** define an uplink server-time message, parse it, route it to
`DownlinkEncoder.sync_server_clock()`, and assert `clock.synced` before emitting
any scoring packet.

### C2. Downlink packet formats are PROVISIONAL
**File:** `src/uav/comm/downlink.py:1-20`
Module docstring states the `KLT / KBX / QRR / MOD` formats are *"PROVISIONAL
(gecici) bizim formatlarimizdir"* — placeholders invented before the official
"Haberlesme Dokumani" was available. The competition spec PDF
(`SIHA_Haberlesme_Dokumani_2026_*.pdf`) is now in the repo but the encoder bodies
have **not been reconciled** against it.
**Consequence:** field order/labels may not match what the server expects → zero
score. The encoder *interface* is stable by design, so the fix is localized to
`encode_*` bodies — but it must actually be done.

### C3. No end-to-end scoring-path test or send retry
**Files:** `src/uav/comm/downlink.py:101-119`, `tests/test_tracking.py`
`DownlinkEncoder.send()` returns `False` on oversize or missing `send_fn` and is
**fire-and-forget — no retry, no ack.** Şartname allows essentially one packet at
lock/kamikaze completion; a single dropped STATUSTEXT = unscored mission. There is
no integration test asserting the full `CombatVisionTracker → DownlinkEncoder →
MavlinkBridge.send_statustext` chain produces a valid, server-stamped frame.

## HIGH — Safety & Flight Correctness

### H1. Min-altitude floor must hold during the dive
**Files:** `src/uav/app/application.py:435-452` (`_qr_dive_floor`),
`src/uav/safety/safety_gate.py`
The dive floor is computed as `max(cfg_floor, safety_floor + pullout_sag)` with a
0.5 g pull-out estimate. This is a *model*, not a guarantee. Şartname treats a
min-altitude breach (even during kamikaze) as an out-of-area penalty. FBWA dive
commands (`RC_CHANNELS_OVERRIDE`) bypass ArduPlane's L1/TECS altitude protection,
so the companion-side floor is the **only** guard — it needs explicit per-tick
verification under realistic descent rates (SITL + hardware), not just at command
issue.

### H2. RC override silently dropped on sysid mismatch
**File:** `src/uav/app/application.py:633-638`, `src/uav/comm/mavlink_io.py:218`
ArduPlane accepts `RC_CHANNELS_OVERRIDE` **only** from `MAV_GCS_SYSID`. The code
sets the signing sysid from `param_reader.gcs_sysid()` on `PARAMS_READY`. If
params are not ready, or the value is wrong, FBWA stick injection is dropped
**silently** and the aircraft flies straight in FBWA — a failure mode with no
loud signal. Needs a guard/alarm when FBW is requested but sysid is unconfirmed.

### H3. Boundary recovery vs. loiter geometry edge case
**File:** `src/uav/app/application.py:509-548`
Comments document a real bug history: when the loiter radius exceeds
`boundary_buffer`, an edge target's orbit circle spills outside the fence, so a
"valid" pursuit route keeps the aircraft outside. The continuous
`_enforce_boundary_recovery` mitigates this, but the interaction between
loiter-radius, boundary buffer, and FOLLOW/PURSUIT near the edge remains fragile
and config-sensitive.

## MEDIUM — Robustness

### M1. Multipart STATUSTEXT buffer key can collide
**File:** `src/uav/comm/statustext_parser.py:96-108`
Reassembly key is `f"{prefix}_{total}"` — it does **not** include any message id.
Two interleaved multipart messages with the same prefix and same part-count
(e.g. two `RIV[1/2]` streams) would write into the same buffer and corrupt each
other. Low probability at 1–2 Hz uplink, but possible. Stale entries are dropped
after 10 s (`_cleanup_stale`), which bounds but does not prevent the collision.

### M2. Vision divergence: SITL vs production
**Files:** `src/uav/perception/simple_vision_tracker.py` vs
`src/uav/perception/tracking/combat_tracker.py`
SITL uses synthetic `SimpleVisionTracker` (world-model fallback); production uses
`CombatVisionTracker` (YOLO + Kalman + PID over a real camera). Lock timing, miss
behavior, and confidence semantics differ. Anything validated only in SITL has
**not** exercised the real lock/scoring path. cv2 tests auto-skip when the lib is
absent (see TESTING.md), so "green" locally can hide the prod path entirely.

### M3. Optional-dependency failures are swallowed
**Files:** `src/uav/app/application.py:255-274,276-331`
`_register_combat_module` / `_register_opencv_module` wrap registration in
`except Exception: # noqa: BLE001` and only log. If ultralytics/opencv/torch is
missing or a model fails to load, the system starts **without vision** and the
only signal is a log line — easy to miss before a competition run.

## LOW — Maintainability

### L1. Large, complex hotspots
`navigation_engine.py` (1123 LOC), `qr_dive_mode.py` (698), `planner.py` (659),
`follow_mode.py` (621), `combat_tracker.py` (519) concentrate most of the logic
and most of the risk. No size limits; methods are split by `# === ... ===`
dividers rather than extraction. Changes here warrant extra review.

### L2. No linter / formatter / CI gates
No `.flake8`, `ruff`, `black`, or CI config detected. Style is hand-maintained;
`# noqa: BLE001` is used deliberately for loop guards, but there is nothing
enforcing it or catching regressions automatically.

### L3. One private-attribute reach-through
**File:** `src/uav/mission/qr_dive_mode.py:270` — `self._nav._att_law.cfg`
(`# noqa: SLF001`, marked "gecici zarf genisletme"). Couples QR dive to nav
internals; a candidate for a proper accessor.

## Notes / Non-Issues

- Broad `except Exception: # noqa: BLE001` in async loops is **intentional** fault
  isolation, not debt (see CONVENTIONS.md). Flagged only where it hides a
  *capability* failure (M3).
- `WorldModel` deepcopy getters + RLock and `EventBus` `call_soon_threadsafe`
  appear to correctly bridge the RX-thread / asyncio boundary.

---

*Concerns analysis: 2026-06-21*
