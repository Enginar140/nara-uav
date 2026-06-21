# Coding Conventions

**Analysis Date:** 2026-06-21

## Naming Patterns

**Files:**
- Always `snake_case.py` — no hyphens, no uppercase filenames
- Examples: `flight_commander.py`, `mission_director.py`, `safety_gate.py`, `zone_checker.py`
- Config/settings dataclasses use `*Config` suffix: `PlannerConfig`, `SafetyGateConfig`, `FollowConfig`, `QRDiveConfig`, `VehicleConfig`

**Classes:**
- `PascalCase` — `PathPlanner`, `NavigationEngine`, `MissionDirector`, `SafetyGate`
- Enums: `PascalCase` — `MissionMode`, `RiskLevel`, `Channel`, `EventType`
- Enum members: `UPPER_SNAKE_CASE` — `MissionMode.QR_DIVE`, `Channel.ROUTE`, `RiskLevel.CRITICAL`
- `auto()` used for internal tracking enums: `EventType`, `Channel` (in `src/uav/infra/event_bus.py`, `src/uav/nav/navigation_engine.py`)
- `IntEnum` for ordered values compared numerically: `RiskLevel`, `BoundaryStatus` (in `src/uav/core/types.py`)

**Functions / Methods:**
- `snake_case` — `plan_route`, `fly_to_position`, `validate_segment`
- Private helpers prefixed with `_` — `_active_guidance`, `_dispatch_sync`, `_handoff_floor`
- Boolean-returning predicates named as assertions — `is_path_safe`, `is_valid`, `has_boundary`, `course_active`, `fbw_active`
- Command/action methods return `bool` (success/failure): `fly_to_position`, `set_mode`, `plan_route`
- Void operations return `None`; nullable results return `Optional[T]`; queries return dataclass instances

**Variables:**
- `snake_case` — `hss_buffer_m`, `turn_rate_dps`, `arrive_radius_m`
- Physical unit suffixes are mandatory: `_m` metres, `_ms` m/s, `_deg` degrees, `_s` seconds, `_us` microseconds, `_dps` degrees-per-second
- Module-level constants: `UPPER_SNAKE_CASE` — `EARTH_RADIUS_M`, `DEG_TO_RAD`, `RAD_TO_DEG` (in `src/uav/core/geo.py`)
- Private module-level constants prefixed `_`: `_HANDOFF_LOITER_MARGIN_M`, `_HANDOFF_MAX_CORNER_DEG`, `_LL_EPS`, `_ALT_EPS`

## Bilingual Convention

This is the single most important convention — violation breaks team norms:

- **Module docstrings:** Turkish prose (domain language of the competition)
- **Class names, variable names, function names, parameters:** English `snake_case`
- **Comments inside functions:** Turkish prose explaining logic
- **Commit messages:** Turkish
- **Log messages:** Turkish — e.g., `"Log dosyasi acilamadi, sadece konsol kullanilacak"` (in `src/uav/infra/log.py`)
- **Exception to module-level docstrings:** C++ port references written as: `"C++ NavigationEngine portu"`, `"C++ SafetyGate portu"`

## Code Style

**Formatting:**
- No explicit formatter config (no `.prettierrc`, no `ruff.toml`, no `black` config in `pyproject.toml`)
- Consistent 4-space indentation observed throughout
- Lines kept under approximately 100 characters
- Aligned closing parens on multi-line calls

**Linting:**
- No `.flake8`, no `ruff` config
- `# noqa: BLE001` used to suppress broad `except Exception:` catches in async loop guards — this is an intentional pattern for fault-isolated loops (see `src/uav/app/application.py`, `src/uav/infra/event_bus.py`, `src/uav/infra/telemetry.py`)
- `# noqa: SLF001` used for one explicit private-attribute access between subsystems (`src/uav/mission/qr_dive_mode.py:270`)
- `# type: ignore` used at optional `cv2` import in `src/uav/perception/tracking/combat_tracker.py`

## Import Organization

**Order within a file (observed pattern):**
1. `from __future__ import annotations` (always first when present)
2. Standard library imports (`import math`, `import threading`, `from dataclasses import ...`, `from enum import ...`, `from typing import ...`)
3. Third-party imports (`from pymavlink import mavutil`, `import numpy`)
4. Intra-package relative imports (`from ..core.types import ...`, `from ..infra.log import ...`, `from .mode_base import ...`)

**Path Aliases:** None — pure relative imports using `..` and `.` dot notation.

**Late / lazy imports:** Used deliberately to avoid heavy import cost at module load:
- `pyproj.Transformer` in `src/uav/core/geo.py` `LocalFrame.__init__`
- MAVLink numeric constants imported inside functions in test helpers

## Optional Dependency Guards

Pattern used whenever an optional library may be absent:

```python
try:
    import cv2  # noqa: F401
    _HAS_CV2 = True
except ImportError:
    _HAS_CV2 = False
```

Used in: `src/uav/perception/opencv_vision_tracker.py`, `src/uav/perception/tracking/combat_tracker.py`, `src/uav/perception/tracking/qr_pro.py`, `src/uav/perception/tracking/roi_tracker.py`. Code paths guard on `_HAS_CV2` before calling cv2 APIs — module imports never crash even without vision extras.

## Data Modelling

- `@dataclass` with `from __future__ import annotations` for forward refs
- `field(default_factory=...)` for mutable defaults — no bare `list` or `dict` as default
- Example from `src/uav/core/types.py`:
  ```python
  @dataclass
  class UAVState:
      position: Position = field(default_factory=Position)
      velocity: Velocity = field(default_factory=Velocity)
  ```
- Config dataclasses carry all tunable values with sensible defaults; passed to constructors, never read from globals
- `Enum` for string/symbolic values; `IntEnum` for ordered values that need `<` / `>` comparison

## Error Handling

**Pattern:** Async loop guards use broad `except Exception:` with explicit `# noqa: BLE001` suppression. This is intentional — a single handler failure must not crash the dispatch loop.

```python
try:
    res = sub.handler(ev)
    if asyncio.iscoroutine(res):
        await res
except Exception:  # noqa: BLE001
    _log.exception("Handler hatasi: %s", ev.type.name)
```

**Inside business logic:** Specific exceptions only. No bare `except:`. Errors propagate as `bool` return from command methods (e.g., `fly_to_position` returns `False` on failure).

**No custom exception classes detected** — failure communicated via `bool` returns and log messages, not exceptions.

## Logging

**Framework:** stdlib `logging`; single hierarchy rooted at `"uav"` logger. Setup in `src/uav/infra/log.py` (RotatingFileHandler + StreamHandler).

**Usage pattern (every module):**
```python
from ..infra.log import get_logger
_log = get_logger("module_name")
```

**Level usage:**
- `_log.debug(...)` — internal state, tick-level values
- `_log.info(...)` — lifecycle events, mode transitions, connection events
- `_log.warning(...)` — recoverable issues, invalid transitions, suppressed commands
- `_log.exception(...)` — inside `except` blocks only (includes traceback automatically)

**Log messages in Turkish.** No f-strings in log calls — `%s` positional formatting used (standard logging pattern).

## Comments

**Section dividers:**
```python
# =============================================================================
# Komut & Gorev tipleri
# =============================================================================
```

**Invariant comments:** Turkish prose explaining non-obvious decisions inside functions. Example from `src/uav/mission/mission_director.py`:
```python
# Dalis KURULAMAZSA (entry'ye varilamaz) RTL degil PATROL: gorevde kal
```

**Test case intent:** First-line docstring or comment on what bug/invariant is covered. Example from `src/uav/nav/navigation_engine.py`:
```python
# Devir tabani = loiter yaricapi + bu marj: ArduPlane GUIDED hedefi loiter
# merkezi olarak ucar; ucagin orbit'e YATMAYA BASLAMASINDAN once sonraki
# bacaga gecilmeli (yalpalamanin 1 numarali kaynagi buydu).
```

**No JSDoc/TSDoc** — Python docstrings are module-level only (Turkish); class and function bodies rely on comments for explanation.

## Function Design

- **Size:** No observed size limit; long methods split with `# === ... ===` section dividers
- **Parameters:** Prefer explicit keyword arguments for config; dependency injection via constructor, not global lookup
- **Return Values:**
  - `bool` for commands/actions (success/failure)
  - Dataclass instances for query results
  - `None` for void operations
  - `Optional[T]` for nullable results

## Module Design

**Exports:** No `__all__` declarations observed — all public names are implicitly exported.

**Barrel Files:** No barrel `__init__.py` re-exports found — imports use full module paths.

**Dependency Injection:** Every subsystem receives its collaborators via constructor; no module-level singletons except `_log` loggers and optional capability flags (`_HAS_CV2`).

## Thread Safety

**Pattern:** `WorldModel` (`src/uav/world_model/world_model.py`) and `FlightCommander` (`src/uav/comm/flight_commander.py`) use `threading.RLock`. All getters return `copy.deepcopy` — callers own their copy, no aliasing to internal state.

MAVLink receive runs in daemon threads; data enters the asyncio world through `WorldModel` (lock-protected writes) and `EventBus.publish()` (thread-safe via `loop.call_soon_threadsafe`).

---

*Convention analysis: 2026-06-21*
