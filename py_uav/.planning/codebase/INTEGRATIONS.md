# External Integrations

**Analysis Date:** 2026-06-21

## MAVLink / ArduPlane Autopilot Link

**Pixhawk (primary flight link):**
- Protocol: MAVLink 2 over UDP (`udpin:0.0.0.0:14550` production; `udpin:127.0.0.1:14550` SITL)
- SDK/Client: `pymavlink >= 2.4` — `mavutil.mavlink_connection()` in `src/uav/comm/mavlink_io.py`
- Receive: daemon thread (`_rx_loop`) calls `master.recv_match(blocking=True, timeout=1.0)`, dispatches to `MavlinkBridge._on_pixhawk_msg()`
- Send (thread-safe): `send_command_int`, `send_command_long`, `send_rc_channels_override`, `send_statustext` in `src/uav/comm/mavlink_io.py`
- Config: `mavlink.pixhawk.connection` and `mavlink.pixhawk.heartbeat_timeout` in YAML configs

**Messages received from ArduPlane:**
- `HEARTBEAT` — mode decode (`ArduPilotMode` enum), armed state; triggers `PARAMS_READY` sequence
- `GLOBAL_POSITION_INT` — lat/lon/alt/velocity → `WorldModel.set_own_state()`
- `VFR_HUD` — airspeed, groundspeed overlay
- `GPS_RAW_INT` — fix type, HDOP, satellites visible
- `SYS_STATUS` — battery voltage/remaining
- `ATTITUDE` — roll/pitch/yaw (degrees) for FBW control law
- `COMMAND_ACK` — command acknowledgment for `FlightCommander`
- `PARAM_VALUE` — ArduPlane parameter readback for `ParamReader`
- `STATUSTEXT` — routed to `StatusTextParser` (same pipeline as GCS messages)

**Commands sent to ArduPlane (`src/uav/comm/flight_commander.py`):**
- `DO_REPOSITION` (COMMAND_INT) — navigate to lat/lon/alt at ground speed; primary nav command
- `GUIDED_CHANGE_SPEED` (43000), `GUIDED_CHANGE_ALTITUDE` (43001), `GUIDED_CHANGE_HEADING` (43002) — ArduPilot extension commands for course-channel guidance
- `DO_SET_MODE` — mode transitions (GUIDED, FBWA, RTL, etc.)
- `COMPONENT_ARM_DISARM` — arming control
- `RC_CHANNELS_OVERRIDE` — FBW/FBWA stick injection for attitude-law guidance; **must be sent with GCS sysid (MAV_GCS_SYSID / SYSID_MYGCS, default 255)** — ArduPlane silently ignores overrides from non-GCS sysids

**ArduPlane parameter fetch (`src/uav/comm/param_reader.py`):**
- Critical params fetched individually on first HEARTBEAT; list fetch fallback after 3s if incomplete
- Params read: `AIRSPEED_MIN/MAX/CRUISE`, `ROLL_LIMIT_DEG`, `WP_RADIUS`, `WP_LOITER_RAD`, `FENCE_ALT_MAX/MIN`, pitch limits, all RC calibration params
- Supports both ArduPlane 4.4 (old names, centidegree/cm units) and 4.5+ (new names, degree/m units) via alias groups

## GCS / Competition Server Link

**GCS (Mission Planner-based Ground Control Station):**
- Protocol: MAVLink 2 over TCP (`tcpin:0.0.0.0:14552` production; `tcpin:127.0.0.1:14552` SITL)
- SDK/Client: `pymavlink >= 2.4` — second `MavlinkIO` instance in `src/uav/comm/mavlink_bridge.py`
- Passthrough: all Pixhawk messages are forwarded to GCS (`_on_pixhawk_msg` → `gcs.send_message(msg)`)
- The GCS communicates with the competition server via HTTP/JSON (transparent to companion)

**Incoming from GCS (STATUSTEXT uplink) — parsed by `src/uav/comm/statustext_parser.py`:**

| Message prefix | Format | Effect |
|---|---|---|
| `RIV:` | `RIV:ID:LAT:LON:ALT:HDG:SPD:TS` | Opponent UAV state → `WorldModel.update_opponent()` |
| `HSS:` | `HSS:COUNT:ID:LAT:LON:RAD:...` | HSS no-fly zones → `WorldModel.replace_hss_zones()` |
| `BND:` | `BND:COUNT:LAT:LON:...` | Competition boundary polygon → `WorldModel.set_boundary()` |
| `FREE` | literal | Mode command → `MissionMode.PATROL` |
| `ARDUPILOT` | literal | Mode command → `MissionMode.ARDUPILOT` (operator takeover) |
| `LOCK:<id>` | `LOCK:9` | Mode command → `MissionMode.FOLLOW` with target id |
| `KAMIKAZE:<lat>:<lon>` | `KAMIKAZE:37.82:-122.35` | Mode command → `MissionMode.QR_DIVE` with target position |

Multi-part: large messages chunk as `PREFIX[PART/TOTAL]:content`; reassembled by `StatusTextParser._accumulate()` with 10s stale buffer timeout.

**Outgoing to GCS (STATUSTEXT downlink) — encoded by `src/uav/comm/downlink.py`:**

| Format | Trigger | Content |
|---|---|---|
| `KLT:<srv_ms>:<end_ms>:<state>` | Lock acquire/break | Lock result packet; state=1 active/4s, 0=broke |
| `KBX:<cx>:<cy>:<w>:<h>:<iw>:<ih>` | Lock active | Lock bounding box in pixels (optional supplementary) |
| `QRR:<srv_ms>:<payload>` | QR/kamikaze sequence end | QR decode result |
| `MOD:<srv_ms>:<mode>` | Mode transition | Current flight/mission mode |

- Constraint: MAVLink STATUSTEXT body = 50 chars max; `DownlinkEncoder` clips at 50 and rejects oversized
- Long payloads (QR): auto-chunked via `DownlinkEncoder.chunk()` → `<PFX>[i/n]:content` frames
- Server clock sync: `DownlinkEncoder._ServerClock` tracks `offset_ms = server_ms - local_ms`; all timestamps use `server_ms()` → spec requirement (section 6.1.3)
- Route: `DownlinkEncoder.send_fn` is bound to `MavlinkBridge.send_statustext()` which prefers the GCS link, falls back to Pixhawk link
- Implementation: `src/uav/comm/downlink.py`; NOTE: packet format is PROVISIONAL — official competition comms document not yet received; interface stable, implementation body will change

## Camera / RTSP / Video

**Combat Vision Tracker (`src/uav/perception/tracking/combat_tracker.py`):**
- Primary production perception module (id: `"combat_vision"`)
- Camera input: USB device (`type: "device"`, `device_index: 1`), RTSP (`type: "rtsp"`, `url: "rtsp://..."`), video file (`type: "file"`, `file_path: "..."`)
- SDK: `opencv-python >= 4.8` — `cv2.VideoCapture` for all camera types
- Config section: `perception.combat.camera` in YAML
- Resolution: 640×480 default at 30 fps; writer opens at actual first-frame size
- Preview: `cv2.imshow` live preview when `perception.combat.preview.enabled: true` (debug only)
- Worker thread: blocking `cv2.VideoCapture.read()` runs in daemon thread; `poll()` returns last detection without blocking asyncio

**OpenCV Vision Tracker (`src/uav/perception/opencv_vision_tracker.py`):**
- Legacy/backup round-object detector using `HoughCircles`
- Camera input: same `CameraConfig` (device/rtsp/http/file); `device_index: 2` default in production, `device_index: 1` in SITL
- Config section: `perception.opencv_vision` in YAML; `enabled: false` by default in both production and SITL (single-camera design)

**Video Recording (`src/uav/perception/tracking/video_recorder.py`):**
- Output: `logs/flight_record.mp4` (configurable via `perception.combat.video.path`)
- Codec: H264/avc1 (spec requirement); falls back to mp4v with warning
- Rate: fixed 15 fps with wall-clock pacing (server ms timestamp overlay on every frame, spec section 6.1.1)
- Safety: never overwrites existing file (timestamp-suffix fallback); stops if free disk < 500 MB (prod) or 1000 MB (SITL)
- Spec overlays: AV reference zone (red rectangle), AH lock rectangle (red, ≤3px), server-time timestamp top-right corner
- Max duration: 3600s production, 1800s SITL (leak guard)

## YOLO Model

**Primary model file:** `models/best.pt`
- Format: classic YOLOv5 repo checkpoint (pickle with `models.yolo.*` references)
- Detection target: opponent fixed-wing UAV (trained on aircraft)
- Class IDs: configurable via `perception.combat.yolo.class_ids` (empty = all classes)
- Confidence threshold: 0.35 default; 0.20 in ROI search mode
- Image size: 640px (configurable `imgsz`)
- Backend selection (`src/uav/perception/tracking/yolo_detector.py`):
  1. Detects legacy YOLOv5 checkpoint via ZIP inspection (`data.pkl` contains `b"models.yolo"`)
  2. If legacy: loads via `yolov5.load()` with `weights_only=False` patch and `PosixPath → WindowsPath` patch (Windows)
  3. If not legacy: loads via `ultralytics.YOLO()`; falls back to yolov5 backend on error
- ROI search: Kalman-predicted small window YOLO at 0.20 conf for tracking locked targets; full-frame re-acquisition every 30 frames

## Telemetry / Dashboard (Development Tool)

**TraceRecorder (`src/uav/infra/telemetry.py`):**
- Output: JSONL files `logs/trace_<timestamp>.jsonl`
- Record kinds: `meta`, `state`, `plan`, `event`
- Ring buffer: last 4000 records (prod) / 8000 (SITL) in memory for live read
- Rate: 5 Hz (prod), 10 Hz (SITL)
- Thread-safe: `threading.Lock` on ring buffer; EventBus callbacks may come from threads

**Web Dashboard (`tools/dashboard/server.py` + `tools/dashboard/index.html`):**
- Standalone process; NOT embedded in the autonomy application
- Reads JSONL trace files incrementally (tail / byte-offset based)
- Serves via stdlib `http.server.ThreadingHTTPServer` — no external dependency
- Default port: 8080 (launched by `run_sitl.bat`)
- Features: boundary polygon, HSS circles, planned route, UAV track, opponents, lock status, mode/channel, plan summary, event log
- Live mode: `--follow` flag polls for newest `trace_*.jsonl` in `logs/`

## ArduPlane SITL

**SITL Setup (`tools/sitl_pilot.py`, `tools/gcs_feeder.py`):**
- ArduPlane SITL: `sim_vehicle.py -v ArduPlane --out=udp:127.0.0.1:14550` (external, not managed by py_uav)
- `tools/sitl_pilot.py` — sends MAVLink commands to SITL (arm, mode, takeoff, goto)
- `tools/gcs_feeder.py` — simulates GCS/competition server; sends STATUSTEXT commands (`FREE`, `LOCK:<id>`, `KAMIKAZE:<lat>:<lon>`, `RIV:...`) via TCP to companion's GCS port
- Synthetic vision (`"vision_tracker"` module): converts world-model opponent state into synthetic `VISION_LOS` targets, bypassing real camera — enables end-to-end testing of the guidance/lock pipeline without a camera
- Config: `config/sitl.yaml` uses `vision_module: "vision_tracker"` (FOLLOW) and `perception_module: "vision_tracker"` (QR_DIVE)

## Logging

**Framework:** Python stdlib `logging` with `RotatingFileHandler`
- Implementation: `src/uav/infra/log.py`
- Output: console + `logs/uav.log` (rotating, max 10 MB prod / 50 MB SITL, 5/10 backup files)
- Logger hierarchy: `uav.<module>` (e.g., `uav.mavlink_bridge`, `uav.combat_tracker`)
- Level: `info` (prod), `debug` (SITL) — set via `logging.level` in YAML

## Data Storage

**Databases:** None — no database dependency
**File Storage:** Local filesystem only
- Logs: `logs/uav.log` (text), `logs/trace_*.jsonl` (structured telemetry), `logs/flight_record.mp4` (spec evidence video)
- Models: `models/best.pt` (YOLO checkpoint, checked into repo)
**Caching:** None

## Authentication & Identity

**Auth Provider:** None — no authentication layer
**MAVLink sysid:** Companion uses sysid=1, component=191 (MAV_COMP_ID_ONBOARD_COMPUTER); GCS simulation uses sysid=255
**RC_CHANNELS_OVERRIDE exception:** ArduPlane only accepts RC overrides from GCS sysid (`MAV_GCS_SYSID`/`SYSID_MYGCS`, default 255); companion temporarily spoofs GCS sysid for this specific message type (see `src/uav/comm/mavlink_io.py:send_rc_channels_override`)

## Webhooks & Callbacks

**Incoming:** None — no HTTP/webhook endpoints; all inbound data arrives via MAVLink STATUSTEXT from GCS

**Outgoing:** None — all outbound data goes via MAVLink STATUSTEXT to GCS; the GCS layer (separate repo, Mission Planner-based) handles HTTP communication with the competition server

---

*Integration audit: 2026-06-21*
