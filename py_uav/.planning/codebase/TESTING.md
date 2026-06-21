# Testing

**Analysis Date:** 2026-06-21

## Framework

- **pytest >= 8.0** — runner; config in `pyproject.toml` `[tool.pytest.ini_options]`
  (`testpaths = ["tests"]`, `pythonpath = ["src"]`).
- **pytest-asyncio >= 0.23** with `asyncio_mode = "auto"` — async test functions
  run without explicit `@pytest.mark.asyncio`.
- No coverage tooling configured (no `.coveragerc`, no `pytest-cov` in deps).
- No `conftest.py` — fixtures/helpers are defined inline or in `tests/sim/`.

Run: `pytest` from `py_uav/` (pythonpath wired in `pyproject.toml`), or via the
launchers which already set `PYTHONPATH=src`.

## Test Layout

```
tests/
├── __init__.py
├── sim/
│   ├── __init__.py
│   └── fake_vehicle.py        # SimVehicle + RecordingIO closed-loop harness
├── test_attitude.py           # AttitudeDirectLaw / FBWA stick law
├── test_comm.py               # MAVLink bridge, statustext parse/encode
├── test_combat_replay.py      # CombatVisionTracker replay-driven
├── test_fbw_guidance.py       # FBW guidance blending
├── test_geo.py                # LocalFrame projection round-trips
├── test_infra.py              # EventBus, TraceRecorder
├── test_mission.py            # MissionDirector transition rules
├── test_mission_sim.py        # End-to-end mission against SimVehicle
├── test_nav.py                # NavigationEngine channel arbitration
├── test_planner_hybrid.py     # Hybrid A* + Dubins
├── test_planner_scenarios.py  # Planner boundary/HSS scenarios
├── test_qr_dive_geometry.py   # Dive sight geometry math
├── test_safety.py             # SafetyGate / SafetyMonitor / escape
├── test_telemetry.py          # Snapshot/replay format
├── test_tracking.py           # Perception, lock evaluator, DOWNLINK encoding
├── test_tracking_cv.py        # OpenCV-dependent vision (auto-skipped)
└── test_types.py              # Core dataclasses
```

Tests mirror source modules one-to-one by area.

## Mocking Strategy

**Hand-written fakes, not `unittest.mock`.** The codebase favors explicit fake
objects that implement the same interface as the real collaborator:

- `tests/sim/fake_vehicle.py` — `SimVehicle` (closed-loop kinematic plane,
  `VehicleConfig`) + `RecordingIO` (captures commands sent through the IO layer).
  Used by `test_mission_sim.py` for end-to-end mode behavior without a real
  Pixhawk.
- Lightweight in-test fakes/stubs and scripted detectors are constructed inline
  in the relevant `test_*.py` (e.g. scripted detection sources feeding the
  perception engine, fake nav for follow logic).
- Dependency injection makes this clean: every subsystem takes its collaborators
  via constructor, so a test passes a fake in place of `WorldModel`,
  `FlightCommander`, `NavigationEngine`, etc.

## Optional-Dependency Handling

cv2/numpy-dependent tests **auto-skip** when the library is absent rather than
failing:

```python
cv2 = pytest.importorskip("cv2")        # tests/test_tracking_cv.py:8
```

Also used in `tests/test_tracking.py` (lines 566, 585, 609). Some paths skip on
missing contrib features: `pytest.skip("cv2.QRCodeEncoder yok (contrib gerekli)")`.
This means vision tests are **not guaranteed to run in CI** unless the vision
extras are installed — green locally without cv2 ≠ vision verified.

## Downlink / Data-Contract Tests

`tests/test_tracking.py` exercises `DownlinkEncoder` directly, including server
clock sync:

```python
enc.sync_server_clock(1_718_000_000_000)                       # :209, :631
enc.sync_server_clock(2_000_000_000_000, local_epoch_ms=...)   # :228 (deterministic)
```

**Important gap:** `sync_server_clock` is called *only in tests*. No production
code path feeds a server timestamp into the encoder, so the unit tests validate a
code path that never runs in flight (see CONCERNS.md — server-clock provenance).
There is no end-to-end integration test asserting that a lock/QR event produces a
correctly-formatted, server-time-stamped STATUSTEXT through the full
tracker→encoder→bridge chain.

## Conventions for New Tests

- Async tests need no decorator (`asyncio_mode=auto`).
- Prefer hand-written fakes injected via constructor over `mock.patch`.
- Guard cv2/torch usage with `pytest.importorskip`.
- First-line docstring/comment states the bug or invariant under test
  (project convention — see CONVENTIONS.md).
- Test names and identifiers in English `snake_case`; explanatory comments in
  Turkish, matching source style.

---

*Testing analysis: 2026-06-21*
