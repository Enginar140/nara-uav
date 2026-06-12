"""LockOnEvaluator -- Savasan IHA kilitlenme durum makinesi (saf mantik).

Her frame icin (zaman, tespit, otonomi, hedef-kimlik, havada-mi) girdisi alir;
sartname kilit kurallarini uygular ve kilit basarildiginda/koptugunda LockEvent
uretir. Kamerasiz, deterministik test edilebilir.

Kurallar (bkz. detection.py + sartname 6.1.1 / 7.2.2):
  - Frame kilit-uygun: hedef merkezi AV icinde + eksen orani >= lock_min_frac +
    (otonom kilit isteniyorsa) otonom modda.
  - Kilit: kesintisiz >= min_lock_s (4 sn). Kisa frame kayiplari (<= max_gap_s)
    pencereyi kirmaz; toplam kayip <= invalid_tolerance_s.
  - Otonomi: hedef AV'ye GIRMEDEN once otonom olunmali -> pencere yalniz
    "AV'ye giris" kenarinda (onceki frame AV disindayken) baslar. Hedef zaten
    AV icindeyken otonoma gecmek kilit baslatmaz.
  - Ayni IHA'ya pes pese kilit yok: bir kilit tamamlaninca o hedef kimligi
    'blokeli' olur; farkli bir kimlige kilitlenene kadar tekrar baslatamaz.
  - Yerdeki IHA'ya kilit yok (airborne=False frame'leri gecersiz).

Konservatif tasarim: sartname toleranslari (5sn pencere, +-1sn) cömerttir; biz
yanlis-vurus cezasindan (-30) kacinmak icin daha siki esikler kullaniriz.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional

from .detection import Detection, FrameGeometry, frame_lock_valid


class LockState(Enum):
    IDLE = auto()       # aktif pencere yok
    TRACKING = auto()   # gecerli frame'ler birikiyor (henuz 4 sn degil)
    LOCKED = auto()     # 4 sn saglandi, kilit onaylandi (uzayabilir)


class LockEventKind(Enum):
    ACHIEVED = auto()   # 4 sn ilk kez saglandi
    LOST = auto()       # kilit/pencere koptu (LOCKED iken -> tamamlanmis kilit)


@dataclass
class LockEvent:
    kind: LockEventKind
    target_id: int
    start_t: float
    end_t: float
    box_cx: float = 0.0
    box_cy: float = 0.0
    box_w: float = 0.0
    box_h: float = 0.0
    frame_w: int = 0
    frame_h: int = 0

    @property
    def duration_s(self) -> float:
        return max(0.0, self.end_t - self.start_t)


@dataclass
class LockEvaluatorConfig:
    lock_min_frac: float = 0.06        # eksen oran esigi (sartname %5 + tolerans)
    min_lock_s: float = 4.0            # kesintisiz kilit suresi
    # Sartname kesinti butcesi: 4 sn kilitte %5 = 200 ms ('eksik/hatali frame
    # toleransi'); baslangic/bitiste gecerli degil. Esikler bunun ALTINDA
    # tutulur ki gonderdigimiz her paket hakem incelemesini gecsin.
    max_gap_s: float = 0.15            # tek seferde tolere edilen kayip
    invalid_tolerance_s: float = 0.18  # pencere boyunca toplam kayip butcesi
    require_autonomous: bool = True
    prevent_same_target_twice: bool = True
    require_airborne: bool = True


@dataclass
class _Window:
    target_id: int
    start_t: float
    last_valid_t: float
    invalid_acc: float = 0.0
    last_box: Optional[Detection] = None


class LockOnEvaluator:
    def __init__(self, config: Optional[LockEvaluatorConfig] = None):
        self.cfg = config or LockEvaluatorConfig()
        self._state = LockState.IDLE
        self._win: Optional[_Window] = None
        self._prev_in_av = False
        self._prev_tid: Optional[int] = None
        self._last_t: Optional[float] = None
        self._blocked_target_id: Optional[int] = None  # pes pese ayni hedef engeli

    # ---- durum sorgu ----
    @property
    def state(self) -> LockState:
        return self._state

    def current_lock(self) -> Optional[LockEvent]:
        """Aktif (LOCKED) kilit icin anlik olay; degilse None."""
        if self._state != LockState.LOCKED or self._win is None:
            return None
        return self._make_event(LockEventKind.ACHIEVED, self._win.last_valid_t)

    def reset(self) -> None:
        self._state = LockState.IDLE
        self._win = None
        self._prev_in_av = False
        self._prev_tid = None
        self._last_t = None

    # ---- ana giris ----
    def update(self, t: float, det: Optional[Detection], geom: FrameGeometry,
               is_autonomous: bool = True, target_id: int = -1,
               airborne: bool = True) -> Optional[LockEvent]:
        """Bir frame isle. Onemli gecis varsa LockEvent dondurur (yoksa None)."""
        in_av = bool(det is not None and geom.center_in_av(det.box.cx, det.box.cy))
        valid = self._frame_valid(det, geom, is_autonomous, airborne)
        # bu hedef icin 'pes pese ayni' engeli?
        tid = det.track_id if (det is not None and det.track_id >= 0) else target_id
        blocked = (self.cfg.prevent_same_target_twice
                   and self._blocked_target_id is not None
                   and tid == self._blocked_target_id)

        event: Optional[LockEvent] = None
        if valid:
            event = self._on_valid(t, det, tid, blocked)
        else:
            event = self._on_invalid(t)

        self._prev_in_av = in_av
        self._prev_tid = tid if det is not None else self._prev_tid
        self._last_t = t
        return event

    # ---- ic mantik ----
    def _frame_valid(self, det: Optional[Detection], geom: FrameGeometry,
                     is_autonomous: bool, airborne: bool) -> bool:
        if det is None:
            return False
        if self.cfg.require_airborne and not airborne:
            return False
        if self.cfg.require_autonomous and not is_autonomous:
            return False
        return frame_lock_valid(det, geom, self.cfg.lock_min_frac)

    def _on_valid(self, t: float, det: Detection, tid: int,
                  blocked: bool) -> Optional[LockEvent]:
        if self._state == LockState.IDLE:
            # pencere yalniz AV'ye GIRIS kenarinda baslar (otonomi kurali) ve
            # hedef 'pes pese ayni' degilse. Yeni bir hedef kimligi de 'giris'
            # sayilir (onceki frame'de baska/yok hedef AV'deydi).
            fresh_entry = (not self._prev_in_av) or (self._prev_tid != tid)
            if not fresh_entry or blocked:
                return None
            self._win = _Window(target_id=tid, start_t=t, last_valid_t=t,
                                last_box=det)
            self._state = LockState.TRACKING
            return None

        assert self._win is not None
        # ayni pencere icinde hedef kimligi degistiyse pencereyi yenile
        if tid != self._win.target_id and self._win.target_id >= 0 and tid >= 0:
            self._break_window(self._win.last_valid_t)
            if not (self._prev_in_av or blocked):
                self._win = _Window(target_id=tid, start_t=t, last_valid_t=t,
                                    last_box=det)
                self._state = LockState.TRACKING
            return None

        self._win.last_valid_t = t
        self._win.last_box = det
        # Basari kosulu GECERLI sure uzerinden: gecen sure - kayip butcesi.
        # Boylece 4 sn'lik pencerede kayip varsa kilit, kaybin telafi edildigi
        # ana ertelenir (hakem 4 sn KESINTISIZ kilit arar).
        if self._state == LockState.TRACKING and \
                (t - self._win.start_t - self._win.invalid_acc) \
                >= self.cfg.min_lock_s - 1e-6:
            self._state = LockState.LOCKED
            return self._make_event(LockEventKind.ACHIEVED, t)
        return None

    def _on_invalid(self, t: float) -> Optional[LockEvent]:
        if self._state == LockState.IDLE or self._win is None:
            return None
        single_gap = t - self._win.last_valid_t
        if self._last_t is not None:
            self._win.invalid_acc += max(0.0, t - self._last_t)
        if single_gap > self.cfg.max_gap_s or \
                self._win.invalid_acc > self.cfg.invalid_tolerance_s:
            return self._break_window(self._win.last_valid_t)
        return None

    def _break_window(self, end_t: float) -> Optional[LockEvent]:
        ev: Optional[LockEvent] = None
        if self._state == LockState.LOCKED:
            ev = self._make_event(LockEventKind.LOST, end_t)
            if self.cfg.prevent_same_target_twice and self._win is not None:
                self._blocked_target_id = self._win.target_id
        self._state = LockState.IDLE
        self._win = None
        return ev

    def _make_event(self, kind: LockEventKind, end_t: float) -> LockEvent:
        w = self._win
        box = w.last_box if w else None
        return LockEvent(
            kind=kind,
            target_id=w.target_id if w else -1,
            start_t=w.start_t if w else end_t,
            end_t=end_t,
            box_cx=box.box.cx if box else 0.0,
            box_cy=box.box.cy if box else 0.0,
            box_w=box.box.w if box else 0.0,
            box_h=box.box.h if box else 0.0,
            frame_w=box.frame_w if box else 0,
            frame_h=box.frame_h if box else 0,
        )
