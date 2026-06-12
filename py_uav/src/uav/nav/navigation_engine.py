"""NavigationEngine -- GUIDED DO_REPOSITION sequencing facade.

Planner + WaypointChain + SafetyGate + FlightCommander'i baglar.
GUIDED-only: yalniz pozisyon hedefleri (DO_REPOSITION) ve body/local pozisyon
ofsetleri. Velocity komutu YOK. C++ NavigationEngine'in mantigi korunur ama
hiz tabanli yollar cikarildi.
"""
from __future__ import annotations

import math
import threading
from typing import List, Optional

from ..comm.flight_commander import FlightCommander
from ..core import geo
from ..core.time_utils import now_monotonic
from ..core.types import Position, UAVState
from ..infra.log import get_logger
from ..safety.safety_gate import SafetyGate
from ..world_model.world_model import WorldModel
from .attitude_law import AttitudeDirectLaw, AttitudeLawConfig
from .planner import PathPlanner
from .speed_manager import SpeedManager
from .waypoint_chain import ChainStatus, Waypoint, WaypointChain

_log = get_logger("nav_engine")


class NavigationEngine:
    def __init__(self, planner: PathPlanner, commander: FlightCommander,
                 safety_gate: SafetyGate, speed_manager: SpeedManager,
                 world_model: WorldModel,
                 min_cmd_interval_s: float = 0.25,
                 min_plan_interval_s: float = 2.0,
                 waypoint_acceptance_radius_m: float = 80.0,
                 waypoint_lookahead_factor: float = 1.0,
                 keepalive_interval_s: float = 4.0,
                 velocity_lookahead_s: float = 3.0,
                 attitude_config: Optional[AttitudeLawConfig] = None):
        self._planner = planner
        self._cmd = commander
        self._gate = safety_gate
        self._speed = speed_manager
        self._wm = world_model
        self._chain = WaypointChain()
        self._lock = threading.RLock()
        self._min_cmd = min_cmd_interval_s
        self._min_plan = min_plan_interval_s
        self._wp_acceptance = max(10.0, waypoint_acceptance_radius_m)
        self._wp_lookahead = max(0.0, waypoint_lookahead_factor)
        # Komut suskunsa (paket kaybi vb.) mevcut hedefi bu araliklarla tazele.
        # Stabilite icin her tick re-send YOK: komut yalniz "degisim"de gider.
        self._keepalive_s = max(0.0, keepalive_interval_s)
        # Hiz (velocity) demand'ini guvenlik icin ileri yansitirken kullanilan
        # ufuk: hedef = mevcut + hiz * bu sure; o segment HSS/sinir guvenli mi?
        self._vel_lookahead_s = max(0.5, velocity_lookahead_s)
        self._last_cmd_t = 0.0
        self._last_plan_t = 0.0
        # Replan DENEME zamani (plan-before-swap throttle'i): basarisiz da olsa
        # 5Hz'de tekrar tekrar planlamayi engeller.
        self._last_replan_attempt_t = 0.0
        self._last_sent: Optional[Position] = None
        self._escape_active = False
        # Aktif rotanin homotopi imzasi: replan ayni yani tercih eder
        # (HSS etrafinda yan flip-flop'unu planner seviyesinde keser).
        self._last_signature: tuple = ()
        # Sanal FBWA: GUIDED icinde dogrudan roll/pitch kanunu (opsiyonel)
        self._att_law = AttitudeDirectLaw(attitude_config)
        self._last_att_t = 0.0

    # ===================== Rota planlama =====================
    def plan_route(self, goal: Position, airspeed: float = -1.0,
                   goal_heading_deg: Optional[float] = None) -> bool:
        with self._lock:
            return self._plan_to(goal, airspeed, goal_heading_deg)

    def make_safe_route(self, waypoints: List[Position], airspeed: float = -1.0) -> bool:
        """Verilen waypoint listesini HSS-guvenli zincire cevir ve yukle."""
        with self._lock:
            if self._should_block_normal():
                return False
            own = self._wm.get_own_state()
            if not own.is_valid():
                return False
            boundary = self._wm.get_boundary()
            zones = self._wm.get_active_hss_zones()
            full = [own.position] + list(waypoints)
            speed = airspeed if airspeed > 0 else self._speed.cruise_speed()
            result = self._planner.make_safe(full, boundary, zones,
                                             start_heading_deg=own.heading,
                                             speed_ms=speed)
            if not result.success:
                _log.warning("make_safe_route basarisiz: %s", result.error)
                return False
            self._load_chain(result.waypoints, airspeed)
            _log.info("Guvenli rota: %d waypoint, %.0fm", self._chain.size(),
                      result.total_distance_m)
            return self._send_current(force=True)

    def _plan_to(self, goal: Position, airspeed: float,
                 goal_heading_deg: Optional[float] = None) -> bool:
        # Periyodik/normal planlama: yakinda planladiysak ve aktif rota varsa atla.
        if self._should_block_normal():
            return False
        now = now_monotonic()
        if (now - self._last_plan_t < self._min_plan
                and not self._chain.is_empty() and not self._chain.is_complete()):
            return True
        return self._do_plan(goal, airspeed, goal_heading_deg)

    def _do_plan(self, goal: Position, airspeed: float,
                 goal_heading_deg: Optional[float] = None) -> bool:
        """Cekirdek planlama (plan-before-swap): plan BASARISIZ olursa mevcut
        zincire DOKUNMAZ. Yeni zincir yalniz basarida yuklenir.
        """
        if self._should_block_normal():
            return False
        boundary = self._wm.get_boundary()
        if boundary is None or not boundary.is_valid():
            return False
        own = self._wm.get_own_state()
        if not own.is_valid():
            return False
        if self._gate.cfg and self._inside_hss_buffer(own.position):
            _log.warning("HSS buffer icinde, rota reddedildi (escape bekleniyor)")
            return False
        zones = self._wm.get_active_hss_zones()
        goal_safe = goal if self._gate.validate_position(goal) else self._gate.clamp_to_safe(goal)
        speed = airspeed if airspeed > 0 else self._speed.cruise_speed()
        result = self._planner.plan(own.position, goal_safe, boundary, zones,
                                    start_heading_deg=own.heading,
                                    goal_heading_deg=goal_heading_deg,
                                    speed_ms=speed,
                                    prefer_signature=self._last_signature)
        if not result.success:
            _log.warning("Planlama basarisiz, mevcut rota korunuyor: %s", result.error)
            return False
        self._load_chain(result.waypoints, airspeed)
        self._last_signature = result.signature
        _log.info("Rota planlandi [%s]: %d waypoint, %.0fm", result.stage,
                  self._chain.size(), result.total_distance_m)
        if self._send_current(force=True):
            self._last_plan_t = now_monotonic()
            return True
        return False

    def _load_chain(self, positions: List[Position], airspeed: float) -> None:
        speed = airspeed if airspeed > 0 else self._speed.cruise_speed()
        wps: List[Waypoint] = []
        for i, p in enumerate(positions):
            if i == 0 and len(positions) > 1:
                continue  # ilk nokta = mevcut konum
            wps.append(Waypoint(
                position=p,
                airspeed=speed,
                acceptance_radius_m=self._wp_acceptance,
                label=f"WP{i}",
            ))
        if not wps:
            wps.append(Waypoint(
                position=positions[-1],
                airspeed=speed,
                acceptance_radius_m=self._wp_acceptance,
                label="GOAL",
            ))
        self._chain.replace(wps)

    # ===================== Zincir ilerleme =====================
    def update(self, own_pos: Position) -> ChainStatus:
        with self._lock:
            if self._should_block_normal():
                return ChainStatus.IN_PROGRESS
            if self._chain.is_empty():
                return ChainStatus.EMPTY
            status = self._chain.update(own_pos)
            if status == ChainStatus.WAYPOINT_REACHED:
                # Yeni bacak: hedefi bir kez gonder (degisim).
                if not self._send_current(force=True):
                    self.revalidate_route()
            elif status == ChainStatus.IN_PROGRESS:
                self._track(own_pos)
            return status

    def _track(self, own_pos: Position) -> None:
        """Tek karar otoritesi: bacak ucarken ne yapilacagi.

        Stabilite:
          - Tier-1 (ucuz): mevcut bacak HARD-guvenli mi? Degilse replan
            (plan-before-swap + deneme throttle'i revalidate icinde).
          - look-ahead (yumusak): sonraki WP'yi erken benimse; basarisizsa
            SESSIZCE mevcut bacakta kal (revalidate'e dusurme -> cirpinma yok).
          - Aksi halde komut gonderme; yalniz keepalive ile tazele.
        """
        wp = self._chain.current_target()
        if wp is None:
            return
        own = self._wm.get_own_state()
        # Tier-1: mevcut bacak hala HARD-guvenli mi?
        if own.is_valid() and not self._gate.validate_segment(own.position, wp.position):
            self.revalidate_route()
            return
        # look-ahead: yaklasinca sonraki WP'ye puruzsuz gecis (throttle'li)
        nxt = self._chain.peek_next()
        if nxt is not None and not self._throttled():
            dist = geo.haversine_distance(own_pos, wp.position)
            if dist < wp.acceptance_radius_m * self._wp_lookahead:
                if self._send_target(nxt.position, nxt.airspeed,
                                     require_soft_clear=True, force=True):
                    self._chain.advance_to_next()
                    return
        # keepalive: komut suskunsa (paket kaybi) mevcut hedefi tazele
        if self._keepalive_s > 0.0 and \
                now_monotonic() - self._last_cmd_t >= self._keepalive_s:
            self._send_current(force=True)

    # ===================== Direkt komutlar =====================
    def fly_direct(self, target: Position, airspeed: float = -1.0) -> bool:
        with self._lock:
            return self._fly_direct(target, airspeed, escape=False)

    def fly_escape_direct(self, target: Position, airspeed: float = -1.0) -> bool:
        with self._lock:
            self._chain.clear()
            self._last_sent = None
            sent = self._fly_direct(target, airspeed, escape=True)
            if sent:
                self._escape_active = True
            return sent

    def _fly_direct(self, target: Position, airspeed: float, escape: bool) -> bool:
        if not escape and self._should_block_normal():
            return False
        if not escape and self._throttled():
            return False
        speed = airspeed if airspeed > 0 else self._speed.cruise_speed()
        safe = target if self._gate.validate_position(target) else self._gate.clamp_to_safe(target)
        own = self._wm.get_own_state()
        if not own.is_valid():
            return False
        seg_ok = (self._gate.validate_segment(own.position, safe) if not escape
                  else self._gate.validate_escape_segment(own.position, safe))
        if not seg_ok:
            if not escape:
                # koridor dar -> HSS etrafinda yeniden planla (plan-before-swap)
                _log.warning("Direkt segment guvensiz, HSS etrafinda yeniden planlaniyor")
                return self._do_plan(safe, speed)
            return False
        # tek-bacak klerans kontrolu: hard + direct_margin (konfor DEGIL --
        # koridor icinde takip/pursuit'i engellemez, SafetyGate ayrica korur)
        if not escape:
            boundary = self._wm.get_boundary()
            zones = self._wm.get_active_hss_zones()
            if boundary is not None and not self._planner.is_direct_clear(
                    own.position, safe, boundary, zones,
                    margin_m=self._planner.direct_margin()):
                return self._do_plan(safe, speed)
        return self._send_target(safe, speed, force=True)

    # body/local pozisyon ofsetleri (terminal dalis vb.) -- velocity DEGIL
    def fly_body_offset(self, forward_m: float, right_m: float, down_m: float,
                        yaw_deg: float = math.nan) -> bool:
        with self._lock:
            if self._should_block_normal() or self._throttled():
                return False
            own = self._wm.get_own_state()
            if not own.is_valid():
                return False
            if not all(math.isfinite(v) for v in (forward_m, right_m, down_m)):
                return False
            target = self._body_offset_to_position(own, forward_m, right_m, down_m)
            if not self._gate.validate_segment(own.position, target):
                _log.warning("Guvensiz body-offset komutu reddedildi")
                return False
            sent = self._cmd.fly_to_body_offset(forward_m, right_m, down_m, yaw_deg)
            if sent:
                self._last_cmd_t = now_monotonic()
            return sent

    def fly_bearing(self, bearing_deg: float, distance_m: float,
                    altitude_offset_m: float = 0.0, yaw_deg: float = math.nan) -> bool:
        with self._lock:
            if self._should_block_normal() or self._throttled():
                return False
            own = self._wm.get_own_state()
            if not own.is_valid() or distance_m < 0:
                return False
            br = math.radians(bearing_deg)
            north = math.cos(br) * distance_m
            east = math.sin(br) * distance_m
            target = geo.ned_to_position(geo.NED(north, east, -altitude_offset_m), own.position)
            if not self._gate.validate_segment(own.position, target):
                return False
            sent = self._cmd.fly_to_bearing_offset(bearing_deg, distance_m,
                                                   altitude_offset_m, yaw_deg)
            if sent:
                self._last_cmd_t = now_monotonic()
            return sent

    def pursue_vision_target(self, yaw_off_deg: float, pitch_off_deg: float,
                             range_m: float, yaw_deg: float = math.nan) -> bool:
        with self._lock:
            if self._should_block_normal() or self._throttled():
                return False
            own = self._wm.get_own_state()
            if not own.is_valid() or range_m <= 0:
                return False
            yo, po = math.radians(yaw_off_deg), math.radians(pitch_off_deg)
            fwd = math.cos(po) * math.cos(yo) * range_m
            right = math.cos(po) * math.sin(yo) * range_m
            down = math.sin(po) * range_m
            target = self._body_offset_to_position(own, fwd, right, down)
            if not self._gate.validate_segment(own.position, target):
                return False
            sent = self._cmd.fly_to_vision_target(yaw_off_deg, pitch_off_deg, range_m, yaw_deg)
            if sent:
                self._last_cmd_t = now_monotonic()
            return sent

    def dispatch_perception_target(self, t) -> bool:
        """PerceptionTarget'i cercevesine gore dogru wrapper'a yonlendir."""
        from ..perception.perception_types import PerceptionFrame
        yaw = t.yaw_deg if t.yaw_deg is not None else math.nan
        if t.frame == PerceptionFrame.BODY:
            return self.fly_body_offset(t.a, t.b, t.c, yaw)
        if t.frame == PerceptionFrame.LOCAL_NED:
            return self.fly_local_offset(t.a, t.b, t.c, yaw)
        if t.frame == PerceptionFrame.POLAR:
            return self.fly_bearing(t.a, t.b, t.c, yaw)
        if t.frame == PerceptionFrame.VISION_LOS:
            # sanal FBWA aciksa once dogrudan attitude kanunu dene; emniyet
            # superviszoru reddederse pozisyon-adimi yoluna dus.
            # (perception pitch_off +asagi; kanun +yukari bekler -> isaret cevir)
            if self._att_law.cfg.enabled and self.fly_attitude_los(t.a, -t.b):
                return True
            return self.pursue_vision_target(t.a, t.b, t.c, yaw)
        if t.frame == PerceptionFrame.VELOCITY_NED:
            return self.fly_velocity_ned(t.a, t.b, t.c, yaw)
        if t.frame == PerceptionFrame.VELOCITY_BODY:
            return self.fly_velocity_body(t.a, t.b, t.c, yaw)
        return False

    def fly_local_offset(self, north_m: float, east_m: float, down_m: float,
                         yaw_deg: float = math.nan) -> bool:
        with self._lock:
            if self._should_block_normal() or self._throttled():
                return False
            own = self._wm.get_own_state()
            if not own.is_valid():
                return False
            target = geo.ned_to_position(geo.NED(north_m, east_m, down_m), own.position)
            if not self._gate.validate_segment(own.position, target):
                return False
            sent = self._cmd.fly_to_local_offset(north_m, east_m, down_m, yaw_deg)
            if sent:
                self._last_cmd_t = now_monotonic()
            return sent

    # body/local hiz (Katman 2) -- velocity'yi ileri yansitip segmenti dogrula
    def fly_velocity_ned(self, vn: float, ve: float, vd: float,
                         yaw_deg: float = math.nan) -> bool:
        """Yer-cercevesi NED hiz demand'i (m/s). Hedef ufuk kadar ileri
        yansitilip HSS/sinir guvenligi kontrol edilir; firmware'de timeout->
        loiter failsafe'i ayrica korur."""
        with self._lock:
            if self._should_block_normal() or self._throttled():
                return False
            own = self._wm.get_own_state()
            if not own.is_valid():
                return False
            if not all(math.isfinite(v) for v in (vn, ve, vd)):
                return False
            t = self._vel_lookahead_s
            target = geo.ned_to_position(geo.NED(vn * t, ve * t, vd * t), own.position)
            if not self._gate.validate_segment(own.position, target):
                _log.warning("Guvensiz NED hiz demand'i reddedildi")
                return False
            sent = self._cmd.fly_velocity_ned(vn, ve, vd, yaw_deg)
            if sent:
                self._last_cmd_t = now_monotonic()
            return sent

    def fly_velocity_body(self, vx: float, vy: float, vz: float,
                          yaw_deg: float = math.nan) -> bool:
        """Govde-cercevesi hiz demand'i (m/s; vx ileri, vy sag, vz asagi)."""
        with self._lock:
            if self._should_block_normal() or self._throttled():
                return False
            own = self._wm.get_own_state()
            if not own.is_valid():
                return False
            if not all(math.isfinite(v) for v in (vx, vy, vz)):
                return False
            t = self._vel_lookahead_s
            target = self._body_offset_to_position(own, vx * t, vy * t, vz * t)
            if not self._gate.validate_segment(own.position, target):
                _log.warning("Guvensiz govde hiz demand'i reddedildi")
                return False
            sent = self._cmd.fly_velocity_body(vx, vy, vz, yaw_deg)
            if sent:
                self._last_cmd_t = now_monotonic()
            return sent

    # ===================== sanal FBWA (attitude direct law) =====================
    def attitude_law_enabled(self) -> bool:
        return self._att_law.cfg.enabled

    def reset_attitude_law(self) -> None:
        with self._lock:
            self._att_law.reset()

    def fly_attitude_los(self, yaw_err_deg: float, pitch_err_deg: float,
                         thrust: float = -1.0) -> bool:
        """LOS hatalarindan dogrudan roll/pitch talebi (GUIDED, SET_ATTITUDE_TARGET).

        Emniyet superviszoru: komut edilen yatisla ufuk boyunca yorunge
        HSS buffer/sinir/irtifa-bandina giriyorsa komut GONDERILMEZ ->
        cagiran ayni tick'te pozisyon gudumune duser. Stall korumasi pitch'i
        kirpar. Stream kesilirse firmware GUIDED timeout failsafe'i devrede.
        """
        with self._lock:
            if not self._att_law.cfg.enabled:
                return False
            if self._should_block_normal():
                return False
            now = now_monotonic()
            if now - self._last_att_t < self._att_law.cfg.min_interval_s:
                return True  # akis zaten guncel; cagirana basari raporla
            own = self._wm.get_own_state()
            if not own.is_valid():
                return False
            if not (math.isfinite(yaw_err_deg) and math.isfinite(pitch_err_deg)):
                return False
            roll, pitch = self._att_law.compute(yaw_err_deg, pitch_err_deg, now)
            pitch = self._att_law.clamp_for_airspeed(
                pitch, own.airspeed, self._speed.cfg.min_speed)
            if not self._attitude_projection_safe(own, roll, pitch):
                # bir kez daha kanatlari duzleterek dene: hedef manevrasi
                # yuzunden degil cevre yuzunden guvensizsek duz ucus gecebilir
                if not self._attitude_projection_safe(own, 0.0, min(pitch, 0.0)):
                    _log.warning("attitude talebi emniyet projeksiyonunu gecemedi, "
                                 "pozisyon gudumune donulmeli")
                    return False
                roll = 0.0
                pitch = min(pitch, 0.0)
            sent = self._cmd.set_attitude(roll, pitch, math.nan, thrust)
            if sent:
                self._last_att_t = now
                self._last_cmd_t = now
            return sent

    def _attitude_projection_safe(self, own, roll_deg: float,
                                  pitch_deg: float) -> bool:
        cfg = self._att_law.cfg
        floor = self._gate.cfg.min_altitude_m + cfg.floor_margin_m
        ceiling = self._gate.cfg.max_altitude_m - cfg.ceiling_margin_m
        for p in self._att_law.project_trajectory(own, roll_deg, pitch_deg):
            if not (floor <= p.altitude <= ceiling):
                return False
            # irtifa bandini yukarida kendimiz kontrol ettik; gate'in kendi
            # bandina takilmamasi icin orta deger ver
            flat = Position(p.latitude, p.longitude,
                            (self._gate.cfg.min_altitude_m
                             + self._gate.cfg.max_altitude_m) * 0.5)
            if not self._gate.validate_position(flat):
                return False
        return True

    def release_guided_hold(self) -> bool:
        """Stale GUIDED durumunu (yaw hedefi/son waypoint) temizle.

        Mod gecislerinde ve QR dive sonrasi cagrilir: body-offset(0,0,0) gonderir,
        boylece onceki modun yaw hedefi yeni komutu ezmez. Throttle bypass.
        """
        with self._lock:
            if self._should_block_normal():
                return False
            self._last_cmd_t = now_monotonic()
            self._last_sent = None
            self._chain.clear()
            return self._cmd.fly_to_body_offset(0.0, 0.0, 0.0, math.nan)

    # ===================== Rota yonetimi =====================
    def clear_route(self) -> None:
        with self._lock:
            self._chain.clear()
            self._last_sent = None
            self._last_signature = ()

    def has_active_route(self) -> bool:
        with self._lock:
            return not self._chain.is_empty() and not self._chain.is_complete()

    def can_fly_direct(self, target: Position, escape: bool = False) -> bool:
        """Mevcut konumdan hedefe tek komutluk segment guvenli mi?"""
        with self._lock:
            own = self._wm.get_own_state()
            if not own.is_valid():
                return False
            safe = target if self._gate.validate_position(target) else self._gate.clamp_to_safe(target)
            seg_ok = (self._gate.validate_segment(own.position, safe) if not escape
                      else self._gate.validate_escape_segment(own.position, safe))
            if not seg_ok:
                return False
            if escape:
                return True
            boundary = self._wm.get_boundary()
            zones = self._wm.get_active_hss_zones()
            if boundary is None or not boundary.is_valid():
                return True
            return self._planner.is_direct_clear(
                own.position, safe, boundary, zones,
                margin_m=self._planner.direct_margin())

    def revalidate_route(self) -> bool:
        """Mevcut rotanin HARD-guvenligini dogrula; bozuksa plan-before-swap.

        Stabilite icin:
          - Yalniz HARD ihlali replan tetikler (soft koridor/marj DEGIL): uçak
            L1 ile kose keserken soft marja girse bile rota korunur.
          - Replan basarisizsa mevcut zincir SILINMEZ; eski rotada ucmaya devam.
          - Deneme throttle'i (_min_plan): planlama tutarli sekilde basarisiz
            olsa bile 5Hz'de tekrar tekrar denenmez.
        """
        with self._lock:
            if self._chain.is_empty():
                return True
            boundary = self._wm.get_boundary()
            if boundary is None:
                return True
            zones = self._wm.get_active_hss_zones()
            own = self._wm.get_own_state()
            current = self._chain.current_target()
            hard_buffer = self._gate.cfg.hss_buffer_m
            seg_ok = (not own.is_valid() or current is None
                      or self._gate.validate_segment(own.position, current.position))
            chain_ok = self._chain.validate(self._gate.checker, boundary, zones, hard_buffer)
            if seg_ok and chain_ok:
                return True  # rota HARD-guvenli -> replan yok
            # Rota HARD-bozuk: plan-before-swap (deneme throttle'i)
            now = now_monotonic()
            if now - self._last_replan_attempt_t < self._min_plan:
                return False  # cok sik deneme yok; eski zincir korunur
            self._last_replan_attempt_t = now
            remaining = self._chain.remaining_waypoints()
            if not remaining:
                return False
            goal = remaining[-1].position
            speed = current.airspeed if current is not None else remaining[0].airspeed
            return self._do_plan(goal, speed)

    # ===================== ic yardimcilar =====================
    def _send_current(self, force: bool = False) -> bool:
        # Taahhutlu bacak: yalniz HARD guvenlik (soft-clear DEGIL). Soft koridor
        # bir PLANLAMA tercihidir; ucarken zorlamak cirpinma yaratir.
        wp = self._chain.current_target()
        if wp is None:
            return False
        return self._send_target(wp.position, wp.airspeed, force=force)

    def _send_target(self, target: Position, airspeed: float,
                     require_soft_clear: bool = False, force: bool = False) -> bool:
        safe = target if self._gate.validate_position(target) else self._gate.clamp_to_safe(target)
        own = self._wm.get_own_state()
        if own.is_valid() and not self._gate.validate_segment(own.position, safe):
            _log.warning("Guvensiz waypoint segmenti reddedildi")
            return False
        if require_soft_clear and own.is_valid() and not self._is_direct_soft_clear(own.position, safe):
            _log.warning("Waypoint look-ahead soft koridoru kesiyor, current WP korunuyor")
            return False
        # send-on-change: ayni hedef tekrar gonderilmez (force ile bypass)
        if (not force and self._last_sent is not None
                and geo.haversine_distance(safe, self._last_sent) < 1.0):
            return True
        sent = self._cmd.fly_to_position(safe, airspeed)
        if sent:
            self._last_cmd_t = now_monotonic()
            self._last_sent = safe
        return sent

    def _should_block_normal(self) -> bool:
        if not self._escape_active:
            return False
        own = self._wm.get_own_state()
        if own.is_valid() and self._gate.validate_position(own.position):
            _log.info("Escape tamamlandi, normal komutlar tekrar acik")
            self._escape_active = False
            return False
        return True

    def _throttled(self) -> bool:
        return (now_monotonic() - self._last_cmd_t) < self._min_cmd

    def _inside_hss_buffer(self, pos: Position) -> bool:
        zones = self._wm.get_active_hss_zones()
        return self._gate.checker.is_inside_any_hss(pos, zones, self._gate.cfg.hss_buffer_m)

    def _is_direct_soft_clear(self, frm: Position, target: Position) -> bool:
        # look-ahead erken gecis: hard + direct_margin yeterli (konfor marji
        # koridorda erken gecisi tamamen kilitler)
        boundary = self._wm.get_boundary()
        if boundary is None or not boundary.is_valid():
            return True
        zones = self._wm.get_active_hss_zones()
        return self._planner.is_direct_clear(
            frm, target, boundary, zones,
            margin_m=self._planner.direct_margin())

    def _body_offset_to_position(self, own: UAVState, fwd: float, right: float,
                                 down: float) -> Position:
        yaw = math.radians(own.heading)
        north = math.cos(yaw) * fwd - math.sin(yaw) * right
        east = math.sin(yaw) * fwd + math.cos(yaw) * right
        return geo.ned_to_position(geo.NED(north, east, down), own.position)
