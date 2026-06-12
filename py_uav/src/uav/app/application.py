"""Application -- asyncio orchestration. C++ Application portu.

Init sirasi: core -> comm -> safety -> nav -> mission.
Olay abonelikleri (failsafe/replan/mode), ~5Hz mission tick + dashboard.
"""
from __future__ import annotations

import asyncio
import math
from typing import List, Optional

from ..comm.mavlink_bridge import MavlinkBridge
from ..config.config_manager import ConfigManager
from ..core.time_utils import now_monotonic
from ..core.types import Boundary, HSSZone, MissionMode, Position, RiskLevel
from ..infra.event_bus import Event, EventBus, EventType
from ..infra.log import get_logger, setup_logging
from ..nav.attitude_law import AttitudeLawConfig
from ..nav.navigation_engine import NavigationEngine
from ..nav.planner import PathPlanner, PlannerConfig
from ..nav.speed_manager import SpeedConfig, SpeedManager
from ..perception.perception_engine import PerceptionEngine, PerceptionEngineConfig
from ..perception.simple_qr_detector import QRDetectorConfig, SimpleQRDetector
from ..perception.simple_vision_tracker import SimpleVisionTracker, VisionTrackerConfig
from ..safety.escape_manager import EscapeManager
from ..safety.safety_gate import SafetyGate, SafetyGateConfig
from ..safety.safety_monitor import SafetyMonitor, SafetyMonitorConfig
from ..safety.zone_checker import ZoneChecker
from ..mission.follow_mode import FollowConfig
from ..mission.mission_director import (
    MissionConfig,
    MissionDirector,
    parse_post_dive_action,
)
from ..mission.patrol_mode import PatrolConfig
from ..mission.qr_dive_mode import QRDiveConfig
from ..world_model.world_model import WorldModel

_log = get_logger("application")


class Application:
    def __init__(self, config_path: str):
        self.config_path = config_path
        self._running = False
        self._shutdown = asyncio.Event()
        self.config: Optional[ConfigManager] = None
        self.bus: Optional[EventBus] = None
        self.wm: Optional[WorldModel] = None
        self.bridge: Optional[MavlinkBridge] = None
        self.commander = None
        self.downlink = None
        self.zone_checker: Optional[ZoneChecker] = None
        self.safety_gate: Optional[SafetyGate] = None
        self.safety_monitor: Optional[SafetyMonitor] = None
        self.escape: Optional[EscapeManager] = None
        self.planner: Optional[PathPlanner] = None
        self.speed: Optional[SpeedManager] = None
        self.nav: Optional[NavigationEngine] = None
        self.perception: Optional[PerceptionEngine] = None
        self.mission: Optional[MissionDirector] = None

    # ===================== init =====================
    def initialize(self) -> bool:
        self.config = ConfigManager(self.config_path)
        setup_logging(
            level=self.config.get_str("logging.level", "info"),
            log_dir=self.config.get_str("logging.dir", "logs"),
            max_size_mb=self.config.get_int("logging.max_size_mb", 10),
            backup_count=self.config.get_int("logging.backup_count", 5),
        )
        _log.info("Nara v10 (Python) baslatiliyor: %s", self.config_path)
        self._init_core()
        self._init_comm()
        self._init_safety()
        self._init_nav()
        self._init_perception()
        self._init_mission()
        self._load_boundary_from_config()
        self._load_hss_from_config()
        return True

    def _init_core(self) -> None:
        self.bus = EventBus()
        self.wm = WorldModel()

    def _init_comm(self) -> None:
        self.bridge = MavlinkBridge(
            pixhawk_conn=self.config.pixhawk_connection(),
            world_model=self.wm,
            event_bus=self.bus,
            gcs_conn=self.config.gcs_connection(),
            target_sys=self.config.get_int("mavlink.target_system", 1),
            target_comp=self.config.get_int("mavlink.target_component", 1),
            heartbeat_timeout_s=self.config.get_float("mavlink.pixhawk.heartbeat_timeout", 5.0),
        )
        self.commander = self.bridge.commander
        # Downlink (companion -> sunucu/GCS) STATUSTEXT kodlayici: kilit/QR paketleri
        from ..comm.downlink import DownlinkEncoder
        self.downlink = DownlinkEncoder(send_fn=self.bridge.send_statustext)

    def _init_safety(self) -> None:
        c = self.config
        self.zone_checker = ZoneChecker()
        self.safety_gate = SafetyGate(self.zone_checker, self.wm, SafetyGateConfig(
            hss_buffer_m=c.get_float("safety.hss_buffer_m", 40.0),
            boundary_buffer_m=c.get_float("safety.boundary_buffer_m", 20.0),
            min_altitude_m=c.get_float("safety.min_altitude_m", 30.0),
            max_altitude_m=c.get_float("safety.max_altitude_m", 400.0),
        ))
        self.safety_monitor = SafetyMonitor(self.wm, self.bus, SafetyMonitorConfig(
            rate_hz=c.get_float("safety.monitoring_rate_hz", 20.0),
            hss_buffer_m=c.get_float("safety.hss_buffer_m", 40.0),
            boundary_warn_m=c.get_float("safety.boundary_warn_m", 30.0),
            collision_critical_m=c.get_float("safety.collision_critical_m", 30.0),
            collision_high_m=c.get_float("safety.collision_high_m", 80.0),
            collision_medium_m=c.get_float("safety.collision_medium_m", 200.0),
            collision_low_m=c.get_float("safety.collision_low_m", 500.0),
            ttc_critical_s=c.get_float("safety.ttc_critical_s", 5.0),
            ttc_high_s=c.get_float("safety.ttc_high_s", 10.0),
        ))
        self.escape = EscapeManager(self.zone_checker, self.wm, self.safety_gate)

    def _init_nav(self) -> None:
        c = self.config
        self.planner = PathPlanner(PlannerConfig(
            hss_buffer_m=c.get_float("nav.planner.hss_buffer_m", 40.0),
            boundary_buffer_m=c.get_float("nav.planner.boundary_buffer_m", 20.0),
            min_turn_radius_m=c.get_float("nav.planner.min_turn_radius_m", 50.0),
            max_bank_deg=c.get_float("nav.planner.max_bank_deg", 35.0),
            turn_safety_margin_m=c.get_float("nav.planner.turn_safety_margin_m", 50.0),
            escape_prevention_margin_m=c.get_float("nav.planner.escape_prevention_margin_m", 80.0),
            tracking_error_m=c.get_float("nav.planner.tracking_error_m", 15.0),
            direct_clear_margin_m=c.get_float("nav.planner.direct_clear_margin_m", 25.0),
            max_segment_length_m=c.get_float("nav.planner.max_segment_length_m", 80.0),
            max_arc_step_deg=c.get_float("nav.planner.max_arc_step_deg", 25.0),
            polygon_segments=c.get_int("nav.planner.polygon_segments", 16),
            dubins_smoothing_enabled=c.get_bool("nav.planner.dubins_smoothing_enabled", True),
            soft_cost_max=c.get_float("nav.planner.soft_cost_max", 4.0),
            boundary_soft_band_m=c.get_float("nav.planner.boundary_soft_band_m", 60.0),
            boundary_cost_max=c.get_float("nav.planner.boundary_cost_max", 1.5),
            side_bias_cost=c.get_float("nav.planner.side_bias_cost", 1.0),
            cell_size_m=c.get_float("nav.planner.cell_size_m", 15.0),
            heading_bins=c.get_int("nav.planner.heading_bins", 16),
            max_expansions=c.get_int("nav.planner.max_expansions", 30000),
            time_budget_ms=c.get_float("nav.planner.time_budget_ms", 600.0),
        ))
        self.speed = SpeedManager(SpeedConfig(
            cruise=c.get_float("nav.speed.cruise", 20.0),
            max_speed=c.get_float("nav.speed.max", 35.0),
            min_speed=c.get_float("nav.speed.min", 12.0),
            patrol=c.get_float("nav.speed.patrol", 18.0),
            pursuit=c.get_float("nav.speed.pursuit", 30.0),
            dive=c.get_float("nav.speed.dive", 25.0),
        ))
        self.nav = NavigationEngine(
            self.planner, self.commander, self.safety_gate, self.speed, self.wm,
            min_cmd_interval_s=c.get_int("nav.command.min_interval_ms", 250) / 1000.0,
            min_plan_interval_s=c.get_int("nav.command.min_plan_interval_ms", 2000) / 1000.0,
            waypoint_acceptance_radius_m=c.get_float("nav.waypoint.acceptance_radius_m", 80.0),
            waypoint_lookahead_factor=c.get_float("nav.waypoint.lookahead_factor", 1.0),
            attitude_config=AttitudeLawConfig(
                enabled=c.get_bool("nav.attitude.enabled", False),
                roll_kp=c.get_float("nav.attitude.roll_kp", 1.6),
                roll_kd=c.get_float("nav.attitude.roll_kd", 0.35),
                pitch_kp=c.get_float("nav.attitude.pitch_kp", 0.9),
                pitch_kd=c.get_float("nav.attitude.pitch_kd", 0.20),
                max_bank_deg=c.get_float("nav.attitude.max_bank_deg", 45.0),
                pitch_min_deg=c.get_float("nav.attitude.pitch_min_deg", -30.0),
                pitch_max_deg=c.get_float("nav.attitude.pitch_max_deg", 18.0),
                pitch_trim_deg=c.get_float("nav.attitude.pitch_trim_deg", 2.0),
                min_interval_s=c.get_float("nav.attitude.min_interval_s", 0.08),
                horizon_s=c.get_float("nav.attitude.horizon_s", 4.0),
                horizon_step_s=c.get_float("nav.attitude.horizon_step_s", 0.8),
                floor_margin_m=c.get_float("nav.attitude.floor_margin_m", 15.0),
                ceiling_margin_m=c.get_float("nav.attitude.ceiling_margin_m", 10.0),
                min_airspeed_margin=c.get_float("nav.attitude.min_airspeed_margin", 2.0),
            ),
        )

    def _init_perception(self) -> None:
        c = self.config
        self.perception = PerceptionEngine(self.bus, self.wm, PerceptionEngineConfig(
            tick_rate_hz=c.get_float("perception.tick_rate_hz", 10.0),
            min_confidence=c.get_float("perception.min_confidence", 0.4),
            lock_timeout_ms=c.get_float("perception.lock_timeout_ms", 1000.0),
            publish_misses=c.get_bool("perception.publish_misses", True),
        ))
        # kamerasiz varsayilan moduller (her zaman kayitli)
        self.perception.register_module(SimpleVisionTracker(self.wm, VisionTrackerConfig(
            detection_timeout_s=c.get_float("perception.vision.detection_timeout_s", 1.0),
            default_range_m=c.get_float("perception.vision.default_range_m", 100.0),
            min_confidence=c.get_float("perception.vision.min_confidence", 0.5),
            target_id=c.get_int("perception.vision.target_id", -1),
            use_world_fallback=c.get_bool("perception.vision.use_world_fallback", True),
        )))
        self.perception.register_module(SimpleQRDetector(QRDetectorConfig(
            detection_timeout_s=c.get_float("perception.qr.detection_timeout_s", 0.7),
            default_range_m=c.get_float("perception.qr.default_range_m", 60.0),
            min_confidence=c.get_float("perception.qr.min_confidence", 0.6),
            expected_payload=c.get_str("perception.qr.expected_payload", ""),
        )))
        # opsiyonel: gercek OpenCV kamera modulu (config'te enabled ise)
        if c.get_bool("perception.opencv_vision.enabled", False):
            self._register_opencv_module()
        # opsiyonel: YOLO+ROI+Kalman+PID kilitlenme/QR modulu (Savasan IHA)
        if c.get_bool("perception.combat.enabled", False):
            self._register_combat_module()

    def _register_combat_module(self) -> None:
        try:
            from ..perception.tracking.factory import (attach_downlink,
                                                       build_combat_tracker)
            tracker = build_combat_tracker(
                self.config,
                server_ms_fn=self.downlink.server_ms if self.downlink else None,
                autonomous_fn=(self.commander.is_in_guided_mode
                               if self.commander else None),
            )
            if self.downlink is not None:
                attach_downlink(tracker, self.downlink, self.bus)
            self.perception.register_module(tracker)
            _log.info("Combat vision modulu kayitli (mode=%s)", tracker.cfg.mode)
        except Exception:  # noqa: BLE001
            _log.exception("Combat vision modulu kaydedilemedi (ultralytics/opencv?)")

    def _register_opencv_module(self) -> None:
        try:
            from ..perception.opencv_vision_tracker import (
                CameraConfig,
                OpenCvVisionConfig,
                OpenCvVisionTracker,
                RoundDetectorConfig,
            )
            cam = self.config.get_node("perception.opencv_vision.camera") or {}
            det = self.config.get_node("perception.opencv_vision.detector") or {}
            rng = self.config.get_node("perception.opencv_vision.range") or {}
            cfg = OpenCvVisionConfig(
                id=self.config.get_str("perception.opencv_vision.id", "vision_tracker_opencv"),
                h_fov_deg=self.config.get_float("perception.opencv_vision.h_fov_deg", 60.0),
                v_fov_deg=self.config.get_float("perception.opencv_vision.v_fov_deg", 45.0),
                detection_timeout_s=self.config.get_float("perception.opencv_vision.detection_timeout_s", 1.0),
                min_confidence=self.config.get_float("perception.opencv_vision.min_confidence", 0.55),
                worker_sleep_ms=self.config.get_int("perception.opencv_vision.worker_sleep_ms", 5),
                range_mode=str(rng.get("mode", "fixed")),
                fixed_range_m=float(rng.get("fixed_m", 50.0)),
                real_diameter_m=float(rng.get("real_diameter_m", 0.30)),
                camera=CameraConfig(
                    type=str(cam.get("type", "device")),
                    device_index=int(cam.get("device_index", 0)),
                    url=str(cam.get("url", "")),
                    file_path=str(cam.get("file_path", "")),
                    width=int(cam.get("width", 640)),
                    height=int(cam.get("height", 480)),
                    fps=float(cam.get("fps", 30.0)),
                    loop_file=bool(cam.get("loop_file", True)),
                ),
                detector=RoundDetectorConfig(
                    dp=float(det.get("dp", 1.2)),
                    min_dist_factor=float(det.get("min_dist_factor", 0.25)),
                    param1=float(det.get("param1", 100.0)),
                    param2=float(det.get("param2", 30.0)),
                    min_radius_px=int(det.get("min_radius_px", 15)),
                    max_radius_px=int(det.get("max_radius_px", 200)),
                    gaussian_kernel=int(det.get("gaussian_kernel", 9)),
                    use_hsv_mask=bool(det.get("use_hsv_mask", False)),
                    base_confidence=float(det.get("base_confidence", 0.7)),
                ),
            )
            self.perception.register_module(OpenCvVisionTracker(cfg))
            _log.info("OpenCV vision modulu kayitli")
        except Exception:  # noqa: BLE001
            _log.exception("OpenCV vision modulu kaydedilemedi (opencv-python?)")

    def _init_mission(self) -> None:
        c = self.config
        mc = MissionConfig(
            patrol=PatrolConfig(
                boundary_inset_m=c.get_float("mission.patrol.boundary_inset_m", 50.0),
                cruise_altitude_m=c.get_float("mission.patrol.cruise_altitude_m", 100.0),
                patrol_speed_ms=c.get_float("nav.speed.patrol", 18.0),
            ),
            follow=FollowConfig(
                pursuit_range_m=c.get_float("mission.follow.pursuit_range_m", 200.0),
                reacquire_timeout_s=c.get_float("mission.follow.reacquire_timeout_s", 5.0),
                max_reacquire_s=c.get_float("mission.follow.max_reacquire_s", 30.0),
                prediction_horizon_s=c.get_float("mission.follow.prediction_horizon_s", 3.0),
                target_stale_s=c.get_float("mission.follow.target_stale_s", 3.0),
                plan_update_interval_s=c.get_float("mission.follow.plan_update_interval_s", 3.0),
                route_lock_s=c.get_float("mission.follow.route_lock_s", 8.0),
                route_goal_update_m=c.get_float("mission.follow.route_goal_update_m", 120.0),
                route_force_replan_m=c.get_float("mission.follow.route_force_replan_m", 350.0),
                commit_route_until_complete=c.get_bool("mission.follow.commit_route_until_complete", True),
                route_prediction_min_s=c.get_float("mission.follow.route_prediction_min_s", 3.0),
                route_prediction_max_s=c.get_float("mission.follow.route_prediction_max_s", 10.0),
                approach_standoff_m=c.get_float("mission.follow.approach_standoff_m", 150.0),
                route_chain_min_spacing_m=c.get_float("mission.follow.route_chain_min_spacing_m", 120.0),
                target_position_alpha=c.get_float("mission.follow.target_position_alpha", 0.30),
                target_velocity_alpha=c.get_float("mission.follow.target_velocity_alpha", 0.35),
                target_filter_reset_m=c.get_float("mission.follow.target_filter_reset_m", 500.0),
                direct_command_interval_s=c.get_float("mission.follow.direct_command_interval_s", 1.5),
                direct_goal_update_m=c.get_float("mission.follow.direct_goal_update_m", 70.0),
                approach_speed_ms=c.get_float("mission.follow.approach_speed_ms", 20.0),
                pursuit_speed_ms=c.get_float("mission.follow.pursuit_speed_ms", 30.0),
                pn_gain=c.get_float("mission.follow.pn_gain", 3.0),
                pn_max_lead_deg=c.get_float("mission.follow.pn_max_lead_deg", 45.0),
                vision_fallback=c.get_bool("mission.follow.vision_fallback", False),
                vision_module_id=c.get_str("mission.follow.vision_module", "vision_tracker"),
                vision_lock_age_s=c.get_float("mission.follow.vision_lock_age_s", 0.5),
                attitude_pursuit=c.get_bool("mission.follow.attitude_pursuit", False),
            ),
            qr_dive=QRDiveConfig(
                approach_altitude_m=c.get_float("mission.qr_dive.approach_altitude_m", 120.0),
                min_altitude_m=self._qr_dive_floor(),
                climb_altitude_m=c.get_float("mission.qr_dive.climb_altitude_m", 100.0),
                entry_corridor_m=c.get_float("mission.qr_dive.entry_corridor_m", 500.0),
                align_heading_tolerance_deg=c.get_float("mission.qr_dive.align_heading_tolerance_deg", 15.0),
                align_altitude_tolerance_m=c.get_float("mission.qr_dive.align_altitude_tolerance_m", 10.0),
                dive_pitch_deg=c.get_float("mission.qr_dive.dive_pitch_deg", -45.0),
                dive_speed_ms=c.get_float("mission.qr_dive.dive_speed_ms", 25.0),
                target_pass_distance_m=c.get_float("mission.qr_dive.target_pass_distance_m", 15.0),
                climb_timeout_s=c.get_float("mission.qr_dive.climb_timeout_s", 30.0),
                dive_step_interval_s=c.get_float("mission.qr_dive.dive_step_interval_s", 0.5),
                use_perception_terminal=c.get_bool("mission.qr_dive.use_perception_terminal", True),
                perception_lock_age_s=c.get_float("mission.qr_dive.perception_lock_age_s", 0.5),
                perception_module_id=c.get_str("mission.qr_dive.perception_module", "qr_simple"),
                attitude_dive=c.get_bool("mission.qr_dive.attitude_dive", False),
            ),
            qr_dive_on_complete=parse_post_dive_action(
                c.get_str("mission.qr_dive.on_complete", "rtl")),
        )
        self.mission = MissionDirector(self.nav, self.commander, self.wm, self.bus, mc,
                                       perception=self.perception)
        self.mission.initialize()

    def _qr_dive_floor(self) -> float:
        """Dalis komut tabani: guvenlik tabani + pull-out irtifa kaybi.

        Sartname geregi kamikaze dalisinda da min irtifa siniri gecerli
        (ihlal = alan disi cezasi). Toparlanmada kaybedilen irtifa
        v_dusey^2 / (2*a_pull) ile kestirilir (a_pull ~ 0.5g).
        """
        c = self.config
        cfg_floor = c.get_float("mission.qr_dive.min_altitude_m", 20.0)
        safety_floor = c.get_float("safety.min_altitude_m", 30.0)
        speed = c.get_float("mission.qr_dive.dive_speed_ms", 25.0)
        pitch = abs(c.get_float("mission.qr_dive.dive_pitch_deg", -45.0))
        v_down = speed * math.sin(math.radians(pitch))
        pullout_sag = v_down * v_down / (2.0 * 0.5 * 9.80665)
        floor = max(cfg_floor, safety_floor + pullout_sag)
        _log.info("QR dalis tabani: %.0fm (guvenlik %.0f + pull-out %.0f)",
                  floor, safety_floor, pullout_sag)
        return floor

    def _load_boundary_from_config(self) -> None:
        verts_node = self.config.get_node("boundary.vertices") or []
        verts: List[Position] = []
        for v in verts_node:
            verts.append(Position(float(v["lat"]), float(v["lon"]), 0.0))
        if len(verts) >= 3:
            b = Boundary(
                vertices=verts,
                min_altitude=self.config.get_float("safety.min_altitude_m", 30.0),
                max_altitude=self.config.get_float("safety.max_altitude_m", 400.0),
            )
            self.wm.set_boundary(b)
            _log.info("Boundary config'ten yuklendi: %d vertex", len(verts))

    def _load_hss_from_config(self) -> None:
        zones_node = self.config.get_node("hss_zones") or []
        for z in zones_node:
            self.wm.update_hss_zone(HSSZone(
                id=int(z["id"]),
                center=Position(float(z["lat"]), float(z["lon"]), 0.0),
                radius=float(z["radius"]),
                active=bool(z.get("active", True)),
            ))

    # ===================== olay abonelikleri =====================
    def _subscribe_events(self) -> None:
        b = self.bus
        b.subscribe(EventType.GPS_QUALITY_CRITICAL, lambda e: self._trigger_rtl("gps_critical"))
        b.subscribe(EventType.BATTERY_CRITICAL, lambda e: self._trigger_rtl("battery_critical"))
        b.subscribe(EventType.HEARTBEAT_TIMEOUT, self._on_heartbeat_timeout)
        b.subscribe(EventType.HEARTBEAT_RECOVERED, lambda e: self.safety_monitor.notify_heartbeat())
        b.subscribe(EventType.HEARTBEAT_RECEIVED, lambda e: self.safety_monitor.notify_heartbeat())
        b.subscribe(EventType.HSS_ZONE_UPDATED, lambda e: self.nav.revalidate_route())
        b.subscribe(EventType.BOUNDARY_UPDATED, self._on_boundary_updated)
        b.subscribe(EventType.MODE_COMMAND_RECEIVED, self._on_mode_command)
        b.subscribe(EventType.SAFETY_VIOLATION, self._on_safety_violation)
        b.subscribe(EventType.COLLISION_WARNING, self._on_collision_warning)
        b.subscribe(EventType.PARAMS_READY, lambda e: self._apply_ardupilot_params())

    def _on_heartbeat_timeout(self, ev: Event) -> None:
        self.safety_monitor.notify_heartbeat_lost()
        self._trigger_rtl("heartbeat_timeout")

    def _on_boundary_updated(self, ev: Event) -> None:
        boundary = self.wm.get_boundary()
        if boundary is not None:
            boundary.min_altitude = self.config.get_float("safety.min_altitude_m", boundary.min_altitude)
            boundary.max_altitude = self.config.get_float("safety.max_altitude_m", boundary.max_altitude)
            self.wm.set_boundary(boundary)
        self.nav.revalidate_route()

    def _on_mode_command(self, ev: Event) -> None:
        if ev.payload is not None:
            self.mission.handle_mode_command(ev.payload)

    def _on_safety_violation(self, ev: Event) -> None:
        if self.mission.get_current_mode() == MissionMode.ARDUPILOT:
            return
        own = self.wm.get_own_state()
        if not own.is_valid():
            return
        if self.nav.has_active_route() and self.nav.revalidate_route():
            return
        target = self.escape.generate_escape_target(own)
        if target is not None:
            self.nav.fly_escape_direct(target)

    def _on_collision_warning(self, ev: Event) -> None:
        if self.mission.get_current_mode() == MissionMode.ARDUPILOT:
            return
        own = self.wm.get_own_state()
        if not own.is_valid() or ev.payload is None:
            return
        if ev.payload.risk_level < RiskLevel.CRITICAL:
            return
        target = self.escape.generate_collision_escape(own, ev.payload)
        if target is not None:
            self.nav.fly_escape_direct(target)

    def _trigger_rtl(self, reason: str) -> None:
        self.mission.trigger_rtl(reason)

    def _apply_ardupilot_params(self) -> None:
        """PARAMS_READY: gercek arac parametrelerini alt sistemlere uygula.

        Hiz zarfi -> SpeedManager; tahmini donus yaricapi -> PathPlanner koridoru;
        FENCE irtifa -> SafetyGate. C++ Application::applyArduPilotParams portu.
        """
        pr = self.bridge.param_reader
        if not pr.are_critical_ready():
            return
        _log.info("ArduPilot parametreleri alt sistemlere uygulaniyor")

        # SpeedManager
        sc = self.speed.cfg
        sc.min_speed = pr.stall_speed()
        sc.max_speed = pr.max_airspeed()
        sc.cruise = pr.cruise_speed()
        sc.patrol = min(max(sc.patrol, sc.min_speed), sc.max_speed)
        sc.pursuit = min(max(sc.pursuit, sc.min_speed), sc.max_speed)
        sc.dive = min(max(sc.dive, sc.min_speed), sc.max_speed)
        _log.info("  SpeedManager: cruise=%.1f min=%.1f max=%.1f",
                  sc.cruise, sc.min_speed, sc.max_speed)

        # PathPlanner: donus yaricapi (x1.2 emniyet) + koridor marji
        pc = self.planner.cfg
        estimated_r = pr.estimate_turn_radius()
        pc.min_turn_radius_m = estimated_r * 1.2
        pc.turn_safety_margin_m = max(pc.turn_safety_margin_m, pc.min_turn_radius_m)
        _log.info("  PathPlanner: turn_radius=%.0fm (tahmin %.0fm x1.2) koridor=%.0fm",
                  pc.min_turn_radius_m, estimated_r, pc.turn_safety_margin_m)

        # SafetyGate: fence irtifa limitleri
        gc = self.safety_gate.cfg
        if pr.has_param("FENCE_ALT_MAX") and pr.fence_alt_max() > 10.0:
            gc.max_altitude_m = pr.fence_alt_max()
        if pr.has_param("FENCE_ALT_MIN") and pr.fence_alt_min() > 0.0:
            gc.min_altitude_m = pr.fence_alt_min()
        _log.info("  SafetyGate: alt [%.0f, %.0f]", gc.min_altitude_m, gc.max_altitude_m)

    # ===================== calistir =====================
    async def run(self) -> None:
        self.initialize()
        self._subscribe_events()
        await self.bus.start()
        if not await self.bridge.start():
            _log.error("MAVLink baslatilamadi, cikiliyor")
            return
        await self.safety_monitor.start()
        await self.perception.start()
        self.bus.publish(EventType.SYSTEM_STARTED)
        self._running = True
        _log.info("Sistem calisiyor (5Hz ana dongu)")
        await self._main_loop()

    async def _main_loop(self) -> None:
        period = 1.0 / 5.0
        last_cleanup = 0.0
        last_dash = 0.0
        stale_timeout = self.config.get_float("system.opponent_stale_timeout_s", 10.0)
        while self._running and not self._shutdown.is_set():
            tick_start = now_monotonic()
            try:
                self.mission.tick()
            except Exception:  # noqa: BLE001
                _log.exception("mission.tick hatasi")
            if tick_start - last_cleanup >= 2.0:
                self.wm.remove_stale_opponents(stale_timeout)
                last_cleanup = tick_start
            if tick_start - last_dash >= 2.0:
                self._print_dashboard()
                last_dash = tick_start
            elapsed = now_monotonic() - tick_start
            await asyncio.sleep(max(0.0, period - elapsed))

    def _print_dashboard(self) -> None:
        own = self.wm.get_own_state()
        mission_stats = self.mission.get_stats()
        ap_mode = self.commander.get_current_mode().name
        mav_stats = self.bridge.get_stats()
        safety = self.safety_monitor.get_safety_state()
        opponents = sorted(self.wm.get_all_opponents(), key=lambda o: o.id)
        hss_zones = self.wm.get_active_hss_zones()
        param_reader = self.bridge.param_reader

        phase = mission_stats["active_phase"] or "-"
        lines = [
            "=== UAV v10 Dashboard ===",
            "Mode: %s [%s] | AP=%s | Transitions: %d" % (
                mission_stats["current_mode"],
                phase,
                ap_mode,
                mission_stats["mode_transitions"],
            ),
        ]
        if own.is_valid():
            lines.append(
                "Pos: %.6f,%.6f | Alt: %.1fm | Spd: %.1fm/s | Hdg: %.0fdeg" % (
                    own.position.latitude,
                    own.position.longitude,
                    own.position.altitude,
                    own.groundspeed,
                    own.heading,
                )
            )
            lines.append(
                "GPS: fix=%d sat=%d hdop=%s | Bat=%s%% %.1fV" % (
                    own.gps_fix_type,
                    own.satellites_visible,
                    own.gps_hdop,
                    own.battery_remaining if own.battery_remaining >= 0 else "?",
                    own.battery_voltage,
                )
            )
        else:
            lines.append("Telemetry: NO DATA | Pixhawk heartbeat/global position bekleniyor")

        lines.append(
            "Safety: %s | Collision: %s | Boundary: %s | CommLoss: %s | HSS violations: %d" % (
                "OK" if safety.is_safe() else "WARNING",
                safety.collision_risk.name,
                safety.boundary_status.name,
                "YES" if safety.comm_loss else "NO",
                len(safety.hss_violations),
            )
        )
        lines.append(
            "World: Opponents=%d | HSS=%d | Boundary=%s" % (
                len(opponents),
                len(hss_zones),
                "YES" if self.wm.has_boundary() else "NO",
            )
        )
        for opp in opponents[:6]:
            lines.append(
                "  OPP[%d] %.6f,%.6f alt=%.0f spd=%.1f hdg=%.0f age=%.1fs" % (
                    opp.id,
                    opp.position.latitude,
                    opp.position.longitude,
                    opp.position.altitude,
                    opp.groundspeed or opp.velocity.horizontal_speed(),
                    opp.heading,
                    opp.age_seconds(),
                )
            )
        if len(opponents) > 6:
            lines.append("  ... %d rakip daha" % (len(opponents) - 6))

        hb_state = "WAIT"
        if mav_stats.last_heartbeat_us:
            hb_state = "TIMEOUT" if mav_stats.heartbeat_timeout else "OK"
        lines.append(
            "MAVLink: PIX=%s GCS=%s HB=%s | Mode=%s | Armed=%s | RX=%d TX=%d" % (
                "OK" if mav_stats.pixhawk_connected else "LOST",
                "OK" if mav_stats.gcs_connected else "LOST",
                hb_state,
                mav_stats.flight_mode,
                "YES" if mav_stats.armed else "NO",
                mav_stats.messages_received,
                mav_stats.messages_sent,
            )
        )

        total = param_reader.total_count()
        total_text = str(total) if total >= 0 else "?"
        lines.append(
            "Params: %d/%s ready=%s | TurnR=%.0fm" % (
                param_reader.received_count(),
                total_text,
                "YES" if param_reader.are_critical_ready() else "NO",
                param_reader.estimate_turn_radius(),
            )
        )
        _log.info("\n%s", "\n".join(lines))

    async def stop(self) -> None:
        self._running = False
        self._shutdown.set()
        self.bus.publish(EventType.SYSTEM_STOPPING)
        if self.perception:
            await self.perception.stop()
        if self.safety_monitor:
            await self.safety_monitor.stop()
        if self.bridge:
            await self.bridge.stop()
        if self.bus:
            await self.bus.stop()
