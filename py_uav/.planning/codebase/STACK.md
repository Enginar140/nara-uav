# Technology Stack

**Analysis Date:** 2026-06-21

## Languages

**Primary:**
- Python 3.10+ — All autonomy logic, MAVLink communication, vision/perception, navigation, mission management, safety monitoring (`src/uav/`)
- HTML/JavaScript — Web dashboard UI (`tools/dashboard/index.html`); no framework, served by stdlib `http.server`

## Runtime

**Environment:**
- CPython 3.10+ (enforced in `pyproject.toml` via `requires-python = ">=3.10"`)
- asyncio event loop — single-threaded application orchestration; MAVLink RX in daemon threads

**Package Manager:**
- pip / setuptools >= 68 (declared in `pyproject.toml` `[build-system]`)
- Lockfile: None checked in (`requirements.txt` is a flat dependency list, not a lock file)

## Frameworks

**Core:**
- asyncio (stdlib) — Application main loop, safety monitor, perception engine, event bus (`src/uav/app/application.py`, `src/uav/safety/safety_monitor.py`, `src/uav/perception/perception_engine.py`, `src/uav/infra/event_bus.py`); `asyncio_mode = "auto"` in pytest

**Testing:**
- pytest >= 8.0 — Test runner; config in `pyproject.toml` `[tool.pytest.ini_options]`; `testpaths = ["tests"]`, `pythonpath = ["src"]`
- pytest-asyncio >= 0.23 — Async test support; `asyncio_mode = "auto"` globally

**Build/Dev:**
- setuptools >= 68 + wheel — Package build backend declared in `pyproject.toml`
- No linter/formatter config detected (no `.flake8`, no `ruff.toml`, no `.prettierrc`, no `black.toml`)

## Key Dependencies

**Critical (core cannot run without these):**
- `pymavlink >= 2.4` — All ArduPlane communication: HEARTBEAT, GLOBAL_POSITION_INT, VFR_HUD, ATTITUDE, DO_REPOSITION, RC_CHANNELS_OVERRIDE, GUIDED_CHANGE_* ext commands. `mavutil.mavlink_connection` provides UDP/TCP/serial abstraction. Used in `src/uav/comm/mavlink_io.py`, `src/uav/comm/flight_commander.py`, `src/uav/comm/mavlink_bridge.py`.
- `shapely >= 2.0` — 2D polygon operations (`Polygon`, `LineString`, `Point`) for boundary, HSS zone inflation, path safety checks. Removing it breaks the planner entirely. Used in `src/uav/nav/planner.py`, `src/uav/safety/zone_checker.py`.
- `networkx >= 3.0` — Visibility graph (Stage 1 planner). Used in `src/uav/nav/planner.py`.
- `pyproj >= 3.5` — WGS84 geodetic projection (Azimuthal-Equidistant) for the planning `LocalFrame`. Used in `src/uav/core/geo.py`.
- `numpy >= 1.24` — Cost-field arrays (`src/uav/nav/cost_field.py`), Kalman matrices (`src/uav/perception/tracking/target_estimator.py`), nav calculations.
- `PyYAML >= 6.0` — YAML config file parsing. Used in `src/uav/config/config_manager.py`.

**Vision / ML (optional; import-guarded):**
- `ultralytics >= 8.0` — Primary YOLO inference backend. `from ultralytics import YOLO` wrapped in try/except in `src/uav/perception/tracking/yolo_detector.py`. Gracefully disabled if absent.
- `yolov5 >= 7.0` — Legacy YOLOv5 checkpoint fallback backend for `models/best.pt` (classic YOLOv5 pickle format). Loaded via `yolov5.load()` in `src/uav/perception/tracking/yolo_detector.py`. Windows: patches `PosixPath -> WindowsPath` at runtime.
- `opencv-python >= 4.8` — Camera I/O (V4L2 device, RTSP, file) and vision processing (HoughCircles, ROI crop, preview). Guarded with `_HAS_CV2` in `src/uav/perception/opencv_vision_tracker.py`, `src/uav/perception/tracking/combat_tracker.py`, `src/uav/perception/tracking/video_recorder.py`.

**Implicit (pulled by ultralytics/yolov5):**
- `torch` — Pulled in by ultralytics/yolov5; used directly in `src/uav/perception/tracking/yolo_detector.py` for the legacy checkpoint loader (`torch.load` monkey-patched for `weights_only=False`).

## Configuration

**Environment:**
- No `.env` file or environment variable injection; no secrets in code
- All runtime config via YAML files in `config/`
- Config loaded at startup via `ConfigManager(config_path)` (`src/uav/config/config_manager.py`)
- Access pattern: `config.get("nav.planner.hss_buffer_m", 40.0)` (dotted path, typed getters)

**Config files:**
- `config/default.yaml` — Production config (real hardware, `combat_vision` YOLO module, USB camera device_index 1)
- `config/sitl.yaml` — SITL + debug (same MAVLink ports; synthetic `vision_tracker`; YOLO enabled but camera points at laptop device_index 1; debug logging, higher telemetry rate)
- `config/sitl_nocam.yaml` — SITL without camera (perception module disabled)

**ArduPlane parameter overlay:**
- Critical params (stall speed, max airspeed, RCMAP, FENCE_ALT, WP_LOITER_RAD) fetched via `PARAM_VALUE` MAVLink messages at connection time (`src/uav/comm/param_reader.py`)
- Applied to subsystems on `PARAMS_READY` event; supports both ArduPlane 4.4 and 4.5+ param name aliases

**Build:**
- `pyproject.toml` — single source of truth for package metadata, dependencies, and pytest config
- `requirements.txt` — flat pip install list including vision extras and comments (not a lock file)

## Platform Requirements

**Development / Companion Computer:**
- Windows 10/11 (primary development platform; `run.bat`, `run_sitl.bat` launchers; YOLOv5 legacy loader conditionally patches `PosixPath -> WindowsPath`)
- Linux also supported (no Windows-only APIs in core code; Windows path patch is conditional on `os.name == "nt"`)
- `PYTHONPATH=src` must be set — handled by `run.bat`/`run_sitl.bat` and `pyproject.toml` `pytest.ini_options.pythonpath`
- Python 3.10+ required

**Hardware:**
- Onboard companion computer alongside ArduPlane autopilot (Pixhawk)
- MAVLink connections: UDP inbound from Pixhawk (`udpin:0.0.0.0:14550`), TCP inbound from GCS (`tcpin:0.0.0.0:14552`)
- Camera: USB device or RTSP stream (configurable via `perception.combat.camera` or `perception.opencv_vision.camera` in YAML)

**Production:**
- Deployed as companion process alongside ArduPlane firmware (Pixhawk/CubeOrange)
- No web server in production; `tools/dashboard/server.py` is a development-only tool (stdlib `http.server`)
- Launched via: `python -m uav.main --config config/default.yaml` (or `run.bat`)

---

*Stack analysis: 2026-06-21*
