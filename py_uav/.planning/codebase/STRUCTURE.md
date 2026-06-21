# Project Structure

**Analysis Date:** 2026-06-21

## Top-Level Layout

```
py_uav/
├── src/uav/            # All autonomy source (PYTHONPATH=src)
├── tests/              # pytest suite (asyncio_mode=auto)
├── config/             # YAML runtime config (default / sitl / sitl_nocam)
├── tools/              # Dev tooling (dashboard, SITL pilot, replay)
├── models/             # YOLO weights (best.pt — legacy YOLOv5 pickle)
├── samples/            # Sample media/data
├── logs/               # Runtime logs + telemetry JSONL (gitignored)
├── pyproject.toml      # Package metadata, deps, pytest config
├── requirements.txt    # Flat pip list (not a lockfile)
├── run.bat / run_sitl.bat   # Windows launchers
├── CLAUDE.md / AGENTS.md / README.md
├── YOL_PLANLAMA_VE_GOREV.md          # Path-planning + mission design notes (TR)
├── SIHA_Haberlesme_Dokumani_*.pdf    # Competition comms spec (reference)
└── .planning/          # GSD planning artifacts (this map lives in codebase/)
```

## Source Tree (`src/uav/`, ~11.9k LOC)

| Module | Files (LOC) | Notes |
|--------|-------------|-------|
| `app/` | `application.py` (897) | Orchestrator: full DI wiring, 5 Hz loop, 20 Hz FBW stream, dashboard, param overlay |
| `core/` | `types.py` (193), `geo.py` (195), `time_utils.py` (22) | Pure data + math, no external deps |
| `infra/` | `event_bus.py` (162), `telemetry.py` (327), `log.py` (62) | Pub/sub, replay, logging |
| `world_model/` | `world_model.py` (115) | Shared state, RLock |
| `comm/` | `flight_commander.py` (485), `mavlink_bridge.py` (323), `mavlink_io.py` (269), `param_reader.py` (232), `statustext_parser.py` (217), `downlink.py` (151) | MAVLink + data contract |
| `safety/` | `safety_monitor.py` (197), `safety_gate.py` (126), `escape_manager.py` (106), `zone_checker.py` (70) | Risk + gating |
| `nav/` | `navigation_engine.py` (1123), `planner.py` (659), `dubins.py` (249), `hybrid_astar.py` (234), `waypoint_chain.py` (178), `cost_field.py` (176), `attitude_law.py` (166), `fbw_guidance.py` (144), `speed_manager.py` (34) | Channel arbitration + 3-stage planning |
| `mission/` | `qr_dive_mode.py` (698), `follow_mode.py` (621), `mission_director.py` (254), `patrol_mode.py` (118), `qr_dive_geometry.py` (114), `ardupilot_mode.py` (29), `mode_base.py` (23) | Mode state machine |
| `perception/` | `opencv_vision_tracker.py` (267), `perception_engine.py` (152), `simple_vision_tracker.py` (96), `simple_qr_detector.py` (91), `perception_types.py` (42), `module_base.py` (26) | Registry + stubs |
| `perception/tracking/` | `combat_tracker.py` (519), `yolo_detector.py` (249), `video_recorder.py` (246), `lock_evaluator.py` (221), `detection.py` (166), `qr_pro.py` (150), `target_estimator.py` (139), `factory.py` (139), `roi_tracker.py` (133), `centering_pid.py` (106) | Production YOLO+Kalman+PID combat stack |

**Largest / highest-risk files:** `navigation_engine.py` (1123), `qr_dive_mode.py`
(698), `planner.py` (659), `follow_mode.py` (621), `combat_tracker.py` (519),
`flight_commander.py` (485) — these are where complexity (and bugs) concentrate.

## Key Locations — "Where do I find…"

| Need | Location |
|------|----------|
| App entry / wiring | `src/uav/app/application.py`, `src/uav/main.py` |
| Mission mode logic | `src/uav/mission/<mode>_mode.py` |
| Add a mission mode | implement `IMissionMode` (`mission/mode_base.py`), register in `MissionDirector` |
| Path planning | `src/uav/nav/planner.py` (+ `dubins.py`, `hybrid_astar.py`, `cost_field.py`) |
| Actuator commands | `src/uav/comm/flight_commander.py` |
| **Scoring downlink (data contract)** | `src/uav/comm/downlink.py` (`DownlinkEncoder`) |
| **Uplink parsing (GCS→companion)** | `src/uav/comm/statustext_parser.py` |
| MAVLink RX/TX plumbing | `src/uav/comm/mavlink_bridge.py`, `mavlink_io.py` |
| Safety margins / gate | `src/uav/safety/safety_gate.py`, `safety_monitor.py` |
| Vehicle param overlay | `src/uav/comm/param_reader.py` + `Application._apply_ardupilot_params` |
| Shared world state | `src/uav/world_model/world_model.py` |
| Config schema (defaults) | `config/default.yaml`; dataclasses in `Application._init_*` |
| Tests | `tests/test_<area>.py`; sim harness `tests/sim/` |

## Naming Conventions

- Files: always `snake_case.py`; mode files `<name>_mode.py`.
- Config dataclasses: `*Config` suffix (`PlannerConfig`, `QRDiveConfig`).
- Physical-unit suffixes on names are mandatory: `_m`, `_ms`, `_deg`, `_s`,
  `_us`, `_dps` (see CONVENTIONS.md).
- Tests mirror source areas: `tests/test_nav.py`, `tests/test_mission.py`,
  `tests/test_comm.py`, etc.
- No barrel `__init__.py` re-exports; imports use full relative paths (`..core`).

## Config Files

- `config/default.yaml` — production (real HW, `combat_vision` YOLO, USB camera)
- `config/sitl.yaml` — SITL + debug (synthetic `vision_tracker`)
- `config/sitl_nocam.yaml` — SITL, perception disabled

---

*Structure analysis: 2026-06-21*
