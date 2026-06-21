<!-- GSD:project-start source:PROJECT.md -->
## Project

**Nara v10 — Savaşan İHA Companion Otonomi (py_uav)**

TEKNOFEST Savaşan İHA yarışması için sabit-kanat İHA'nın **companion bilgisayar otonomi sistemi**. Felsefe: *"Python plans, ArduPlane flies"* — companion nereye/hangi sırayla gidileceğine karar verir (algılama, takip, planlama, dalış geometrisi), düz-hat takibini (L1/TECS) ArduPlane firmware yapar. Companion, takımın Mission Planner tabanlı yer kontrol istasyonu (GCS) üzerinden yarışma sunucusuyla dolaylı haberleşir.

Bu milestone'un odağı: mevcut olgun otonomi yığınını **şartname-uyumlu hale getirmek** — companion↔GCS veri kontratını sonlandırmak, operasyonel mod mantığını düzeltmek ve uçtan uca doğrulamak.

**Core Value:** **Kilitlenme ve kamikaze görevlerinin şartname-geçerli veri akışıyla puanlanması** — İHA hedefe kilitlense/dalsa bile, doğru formatta + sunucu saatiyle damgalanmış paket gönderilmezse puan YOK. Veri yolu doğru çalışmadan otonominin hiçbir değeri yok.

### Constraints

- **Tech stack**: Python 3.10+, pymavlink, ArduPlane GUIDED/FBWA — velocity komutu YOK; yalnız DO_REPOSITION / GUIDED_CHANGE_* / RC_CHANNELS_OVERRIDE
- **Haberleşme**: Companion ↔ GCS yalnız MAVLink STATUSTEXT (50 karakter gövde, `:` ayraçlı, çok-parçalı chunk); GCS ↔ sunucu HTTP/JSON (companion'a şeffaf)
- **Şartname**: Telemetri 1–2 Hz; tüm olay zamanları sunucu saati; kilitlenme/kamikaze bitiminde 1 paket; paket gönderilmeyen görev puanlanmaz
- **Emniyet**: SafetyGate HARD marj; min irtifa dalışta da geçerli (ihlal = ceza); HSS dairelerinden kaçınma
- **Dependencies**: GCS (ayrı repo) ile iki-taraflı kontrat — bir tarafın değişimi diğerini etkiler
- **Compatibility**: SITL `vision_tracker` (sentetik) ≠ prod `combat_vision` (YOLO+kamera)
<!-- GSD:project-end -->

<!-- GSD:stack-start source:codebase/STACK.md -->
## Technology Stack

## Languages
- Python 3.10+ - All autonomy logic, MAVLink communication, vision, navigation, mission management
- HTML/JavaScript - Web dashboard UI (`tools/dashboard/index.html`); no framework, served by stdlib HTTP server
## Runtime
- CPython 3.10+ (required by pyproject.toml `requires-python = ">=3.10"`)
- asyncio event loop — single main loop; MAVLink RX runs in daemon threads
- pip / setuptools >= 68
- Lockfile: None checked in (requirements.txt is a flat dependency list, not a lock file)
## Frameworks
- asyncio (stdlib) — Application main loop, safety monitor, perception engine, event bus; `asyncio_mode = "auto"` in pytest
- pymavlink >= 2.4 — MAVLink protocol; UDP/TCP/serial connection string abstraction via `mavutil.mavlink_connection`
- numpy >= 1.24 — Array math in cost-field, Kalman estimator, nav calculations
- shapely >= 2.0 — 2D polygon operations (boundary, HSS zone inflation, path safety checks) in `src/uav/nav/planner.py` and `src/uav/safety/zone_checker.py`
- networkx >= 3.0 — Visibility graph construction for comfort-layer path planning in `src/uav/nav/planner.py`
- pyproj >= 3.5 — WGS84 geodetic projections in `src/uav/core/geo.py`
- opencv-python >= 4.8 — Camera capture (V4L2/RTSP/file), HoughCircles detector, ROI crop, preview (`src/uav/perception/opencv_vision_tracker.py`, `src/uav/perception/tracking/combat_tracker.py`)
- ultralytics >= 8.0 — Primary YOLO inference backend (`src/uav/perception/tracking/yolo_detector.py`)
- yolov5 >= 7.0 — Legacy YOLOv5 checkpoint fallback backend (`models/best.pt` is in classic YOLOv5 pickle format)
- pytest >= 8.0 — Test runner; config in `pyproject.toml` `[tool.pytest.ini_options]`
- pytest-asyncio >= 0.23 — Async test support; `asyncio_mode = "auto"` globally
- setuptools >= 68 + wheel — Package build backend declared in `pyproject.toml`
## Key Dependencies
- `pymavlink >= 2.4` — All ArduPlane communication (HEARTBEAT, GLOBAL_POSITION_INT, VFR_HUD, ATTITUDE, DO_REPOSITION, RC_CHANNELS_OVERRIDE, GUIDED_CHANGE_* ext commands). Cannot run without it.
- `shapely >= 2.0` — Path planning uses `Polygon`, `LineString`, `Point` for boundary and HSS collision math. Removing it breaks the planner entirely.
- `networkx >= 3.0` — Visibility graph (Stage 1 planner). Used in `src/uav/nav/planner.py`.
- `pyproj >= 3.5` — Geodetic distance and bearing math in `src/uav/core/geo.py`.
- `numpy >= 1.24` — Cost-field arrays, Kalman matrices.
- `PyYAML >= 6.0` — Config file parsing in `src/uav/config/config_manager.py`.
- `ultralytics >= 8.0` — YOLO model inference (optional, gracefully disabled if absent)
- `yolov5 >= 7.0` — Legacy checkpoint loader; needed when `models/best.pt` is the pre-trained aircraft model in old YOLOv5 pickle format
- `opencv-python >= 4.8` — Camera I/O and vision processing (optional; import-guarded with `_HAS_CV2`)
## Configuration
- No `.env` file or environment variable injection detected
- All runtime config via YAML files in `config/`
- Key configs: `config/default.yaml` (production), `config/sitl.yaml` (SITL + debug), `config/sitl_nocam.yaml` (SITL without camera)
- Config loaded at startup via `ConfigManager(config_path)` in `src/uav/config/config_manager.py`
- ArduPlane parameter overlay: critical params (stall speed, max airspeed, RCMAP, FENCE_ALT) fetched via PARAM_VALUE MAVLink messages at connection time and applied to subsystems
- `pyproject.toml` — single source of truth for package metadata and dependencies
- `requirements.txt` — flat pip install list (includes vision extras and comments)
## Platform Requirements
- Windows 10/11 (project developed on Windows; `run.bat`, `run_sitl.bat` launchers; YOLOv5 legacy loader patches `PosixPath -> WindowsPath` at runtime)
- Linux also supported (no Windows-only APIs in core code; Windows path patch is conditional)
- PYTHONPATH must include `src/` — set by `run.bat` and `pytest.ini_options.pythonpath`
- Onboard companion computer alongside ArduPlane autopilot (Pixhawk)
- MAVLink connections: UDP to Pixhawk (`udpin:0.0.0.0:14550`), TCP to GCS/competition server (`tcpin:0.0.0.0:14552`)
- Camera: USB device or RTSP stream (configurable via `perception.combat.camera` / `perception.opencv_vision.camera`)
- Python 3.10+ required on companion
<!-- GSD:stack-end -->

<!-- GSD:conventions-start source:CONVENTIONS.md -->
## Conventions

## Language and Bilingualism
- **Module docstrings:** Turkish (explains purpose in Turkish domain language)
- **Class names, variable names, function names:** English `snake_case`
- **Comments inside functions:** Turkish prose explaining logic
- **Commit messages:** Turkish
- **Log messages:** Turkish (e.g., `"Log dosyasi acilamadi, sadece konsol kullanilacak"`)
- **C++ port references:** Turkish docstrings reference `C++ X portu` pattern
## Naming Patterns
- `snake_case.py` always (e.g., `flight_commander.py`, `mission_director.py`, `safety_gate.py`)
- No hyphens, no uppercase filenames
- `PascalCase` (e.g., `PathPlanner`, `NavigationEngine`, `MissionDirector`, `SafetyGate`)
- Config/settings dataclasses use `*Config` suffix: `PlannerConfig`, `SafetyGateConfig`, `FollowConfig`, `QRDiveConfig`, `VehicleConfig`
- `snake_case` (e.g., `plan_route`, `fly_to_position`, `validate_segment`)
- Private helpers prefixed with `_` (e.g., `_active_guidance`, `_dispatch_sync`, `_handoff_floor`)
- Boolean-returning methods often named as assertions: `is_path_safe`, `is_valid`, `has_boundary`, `course_active`, `fbw_active`
- `snake_case` (e.g., `hss_buffer_m`, `turn_rate_dps`, `arrive_radius_m`)
- Suffixes encode physical units: `_m` for metres, `_ms` for m/s, `_deg` for degrees, `_s` for seconds, `_us` for microseconds, `_dps` for degrees-per-second
- `UPPER_SNAKE_CASE` at module level (e.g., `EARTH_RADIUS_M`, `DEG_TO_RAD`, `_LL_EPS`)
- Private module-level constants prefixed with `_`: `_HANDOFF_LOITER_MARGIN_M`, `_HANDOFF_MAX_CORNER_DEG`
- Class: `PascalCase` (e.g., `MissionMode`, `RiskLevel`, `Channel`, `EventType`)
- Members: `UPPER_SNAKE_CASE` (e.g., `MissionMode.QR_DIVE`, `Channel.ROUTE`, `RiskLevel.CRITICAL`)
- `PascalCase` (e.g., `EventHandler`)
## Code Style
- No explicit formatter config detected (no `.prettierrc`, no `ruff.toml`, no `black.toml`)
- Style observed in code: consistent 4-space indentation, lines kept under ~100 chars, aligned closing parens on multi-line calls
- No `.flake8` or `ruff` config found in `pyproject.toml`
- `# noqa: BLE001` used to suppress broad exception catches in async loop guards:
## Import Organization
## Data Modelling
- `field(default_factory=...)` for mutable defaults
- `from __future__ import annotations` enables forward references in dataclass fields
- `Enum` for string/symbolic values (`MissionMode`, `GoalType`, `Channel`)
- `IntEnum` for ordered values that get compared numerically (`RiskLevel`, `BoundaryStatus`)
- `auto()` for internal tracking enums (`EventType`, `Channel`)
## Error Handling
## Logging
- `_log.debug(...)` for internal state
- `_log.info(...)` for lifecycle events
- `_log.warning(...)` for recoverable issues
- `_log.exception(...)` inside `except` blocks (includes traceback automatically)
## Function Design
- `bool` for commands/actions (success/failure)
- Dataclass instances for query results
- `None` for void operations
- `Optional[T]` for nullable results
## Module Design
## Comments
- Section dividers with `# === ... ===` (e.g., `# =============================================================================\n# Komut & Gorev tipleri`)
- Key invariants and non-obvious decisions (Turkish prose)
- Test case intent: Docstring or first-line comment on what the bug/invariant is
## Thread Safety
<!-- GSD:conventions-end -->

<!-- GSD:architecture-start source:ARCHITECTURE.md -->
## Architecture

## Overview
## Architectural Pattern
- One asyncio event loop owns orchestration (`Application`).
- MAVLink receive runs in **daemon threads**; data crosses into the async world
- Subsystems are constructed top-down with explicit dependency injection in
- Mission behavior is a **state machine** (`MissionDirector`) delegating to mode
## Layers
## Data Flow
```
```
```
```
```
```
## Key Abstractions
- **Channel arbitration** (`NavigationEngine.Channel`) — exactly one actuator
- **Mode objects** (`mode_base.py`) — each mission mode is a swappable unit with
- **Perception modules** (`module_base.py`) — uniform register/activate interface;
- **Dependency injection** — every subsystem receives its collaborators via
## Entry Points
- `python -m uav.main --config config/<name>.yaml` (`src/uav/main.py`) — parses
- Launchers: `run.bat` (default config), `run_sitl.bat` (SITL config).
- Tooling entry points under `tools/` (dashboard server, SITL pilot, replay,
## Cross-Cutting Concerns
- **Threading model:** MAVLink RX threads + asyncio loop; `WorldModel` (RLock)
- **Safety override priority:** out-of-boundary recovery preempts mission tick
- **Parameter adaptation:** on `PARAMS_READY`, real ArduPlane params reshape
- **Telemetry/replay:** `TraceRecorder` snapshots full structural state for the
<!-- GSD:architecture-end -->

<!-- GSD:skills-start source:skills/ -->
## Project Skills

No project skills found. Add skills to any of: `.claude/skills/`, `.agents/skills/`, `.cursor/skills/`, `.github/skills/`, or `.codex/skills/` with a `SKILL.md` index file.
<!-- GSD:skills-end -->

<!-- GSD:workflow-start source:GSD defaults -->
## GSD Workflow Enforcement

Before using Edit, Write, or other file-changing tools, start work through a GSD command so planning artifacts and execution context stay in sync.

Use these entry points:
- `/gsd-quick` for small fixes, doc updates, and ad-hoc tasks
- `/gsd-debug` for investigation and bug fixing
- `/gsd-execute-phase` for planned phase work

Do not make direct repo edits outside a GSD workflow unless the user explicitly asks to bypass it.
<!-- GSD:workflow-end -->



<!-- GSD:profile-start -->
## Developer Profile

> Profile not yet configured. Run `/gsd-profile-user` to generate your developer profile.
> This section is managed by `generate-claude-profile` -- do not edit manually.
<!-- GSD:profile-end -->
