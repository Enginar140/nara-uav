# Architecture

**Analysis Date:** 2026-06-21

## Architectural Pattern

**Single asyncio event loop + daemon-thread MAVLink RX.** One `Application`
(`src/uav/app/application.py`) owns orchestration. MAVLink receive runs in
background **daemon threads** inside `MavlinkBridge`; data crosses into the async
world via two lock-protected channels: `WorldModel` (RLock, deep-copy getters)
and `EventBus.publish()` (marshalled onto the loop via `loop.call_soon_threadsafe`).

Philosophy: **"Python plans, ArduPlane flies."** The companion decides *where /
in what order* to go (detection, tracking, planning, dive geometry); ArduPlane
firmware does straight-line L1/TECS following. There is **no velocity command** —
only `DO_REPOSITION`, `GUIDED_CHANGE_*`, and `RC_CHANNELS_OVERRIDE` (FBWA stick
injection).

Mission behavior is a **state machine** (`MissionDirector`) delegating to swappable
**mode objects**. Perception is a **registry with a single active slot**. Every
subsystem is wired top-down via **constructor dependency injection** in
`Application._init_*` — no global singletons except module loggers and capability
flags (`_HAS_CV2`).

## Layers

Init order is strict (`Application.initialize`): **core → comm → safety → nav →
mission → perception**.

| Layer | Dir | Key classes | Responsibility |
|-------|-----|-------------|----------------|
| Core | `src/uav/core/` | `types.py`, `geo.py` (`LocalFrame`), `time_utils.py` | Data types, WGS84↔local projection, clocks. No external deps. |
| Infra | `src/uav/infra/` | `EventBus`, `TraceRecorder`, `log` | Thread-safe pub/sub, JSONL replay snapshots, logging |
| World | `src/uav/world_model/` | `WorldModel` | Shared truth (own state, opponents, HSS, boundary). RLock + deepcopy. Written only from RX thread. |
| Comm | `src/uav/comm/` | `MavlinkBridge`, `FlightCommander`, `DownlinkEncoder`, `StatusTextParser`, `ParamReader` | Pixhawk + GCS links, command emission, 50-char STATUSTEXT contract |
| Safety | `src/uav/safety/` | `SafetyMonitor` (20 Hz), `SafetyGate`, `EscapeManager`, `ZoneChecker` | Continuous risk monitoring + per-command HARD gate + boundary/HSS escape |
| Nav | `src/uav/nav/` | `NavigationEngine`, `PathPlanner`, `WaypointChain`, `AttitudeDirectLaw`, `dubins`, `hybrid_astar` | Channel arbitration, 3-stage planning, FBWA attitude law |
| Mission | `src/uav/mission/` | `MissionDirector`, `PatrolMode`, `FollowMode`, `QRDiveMode`, `ArduPilotPassthrough` | Mode state machine |
| Perception | `src/uav/perception/` | `PerceptionEngine`, `CombatVisionTracker`, `SimpleVisionTracker`, `SimpleQRDetector` | Detection/lock registry, YOLO+Kalman+PID combat tracker |

## Data Flow

**Inbound (Pixhawk → companion):**
```
Pixhawk UDP ─► MavlinkBridge RX thread ─► WorldModel.set_own_state (RLock)
                                       └─► EventBus (PARAMS_READY, HEARTBEAT_*)
```

**Inbound (GCS → companion):**
```
GCS TCP ─► MavlinkBridge._handle_statustext ─► StatusTextParser.parse
        ├─ RIV ► WorldModel.update_opponent
        ├─ HSS ► WorldModel.update_hss_zone ► EventBus HSS_ZONE_UPDATED
        ├─ BND ► WorldModel.set_boundary ► EventBus BOUNDARY_UPDATED
        └─ FREE/ARDUPILOT/LOCK/KAMIKAZE ► EventBus MODE_COMMAND_RECEIVED
                                        ► MissionDirector.handle_mode_command
```

**Main tick (5 Hz, `Application._main_loop`):**
```
boundary recovery check ─► if outside: fly_escape_direct (SKIP mission tick)
                          else: MissionDirector.tick()
                                  └► active mode handler.tick(own)
                                       └► NavigationEngine (ROUTE/COURSE/FBW)
                                            └► SafetyGate.validate ─► FlightCommander ─► Pixhawk
```

**FBW stream (20 Hz, separate task `_fbw_stream_loop`):** re-emits the last FBWA
stick setpoint via `nav.restream_fbw()` so RC override does not time out (5 Hz is
too slow for stick injection). Control law updates the setpoint at 5 Hz.

**Outbound scoring downlink (companion → GCS → server):**
```
CombatVisionTracker (lock 4s / QR result) ─► DownlinkEncoder.encode_* (50-char)
                                          ─► MavlinkBridge.send_statustext ─► GCS TCP
```

## Channel Arbitration

`NavigationEngine` enforces **exactly one actuator owner** at a time via
`Channel` (`ROUTE` = waypoint chain / `DO_REPOSITION`, `COURSE` = heading hold,
`FBW` = FBWA stick override). Mode handlers request a channel; the engine
serializes commands and rate-limits them (`min_cmd_interval_s`,
`min_plan_interval_s`). `release_guided_hold()` / `release_fbw()` clear stale
state on mode change.

## Safety Override Priority

Out-of-boundary recovery **preempts** the mission tick: `_enforce_boundary_recovery`
runs first every tick and, if the aircraft is outside the fence, flies an escape
target and returns `True` so `mission.tick()` is skipped entirely. Event-driven
escapes (`SAFETY_VIOLATION`, `COLLISION_WARNING`) are a second layer in case the
continuous check misses a transition. `ARDUPILOT` mode is exempt (operator owns
the aircraft).

## Key Abstractions

- **Mode objects** (`src/uav/mission/mode_base.py`, `IMissionMode`) — each mission
  mode is a unit with `on_enter` / `tick(own)` / `on_exit`; `MissionDirector`
  swaps them and enforces transition rules (`QR_DIVE → only PATROL`, etc.).
- **Perception modules** (`src/uav/perception/module_base.py`) — uniform
  register/activate interface; one active slot at a time (`PerceptionEngine`).
- **Parameter adaptation** — on `PARAMS_READY`, real ArduPlane params reshape
  subsystems (`Application._apply_ardupilot_params`): stall/airspeed → `SpeedManager`,
  turn radius → `PathPlanner`, FENCE alt → `SafetyGate`, RCMAP → `FlightCommander`.
- **Dependency injection** — collaborators passed via constructor in `_init_*`.
- **Telemetry/replay** — `TraceRecorder` snapshots full structural state to JSONL
  for the dashboard / replay tooling.

## Entry Points

- `python -m uav.main --config config/<name>.yaml` (`src/uav/main.py`) — parses
  args, installs signal handlers (best-effort on Windows), runs `Application.run()`.
- Launchers: `run.bat` (default config), `run_sitl.bat` (SITL config).
- Dev tooling under `tools/` (dashboard HTTP server, SITL pilot, replay).

## Cross-Cutting Concerns

- **Threading model:** MAVLink RX daemon threads + single asyncio loop. The only
  thread-safe crossings are `WorldModel` (RLock) and `EventBus` (`call_soon_threadsafe`).
- **Fault isolation:** every async loop guards handlers with `except Exception:
  # noqa: BLE001` so one failure cannot kill the loop.
- **Config-driven:** all tunables come from YAML via `ConfigManager`; dataclasses
  carry defaults, then `PARAMS_READY` overlays real vehicle params.

---

*Architecture analysis: 2026-06-21*
