"""Downlink STATUSTEXT formatlari (companion -> sunucu/GCS).

NOT: Yarisma 'Haberlesme Dokumani' henuz elimizde olmadigindan bu formatlar
PROVISIONAL (gecici) bizim formatlarimizdir; resmi paket formati yayinlaninca
encode_* govdeleri ona gore degistirilir. Arayuz (DownlinkEncoder.*) sabit
kalir, boylece mission/app kodu etkilenmez.

Tasarim kisitlari:
  - MAVLink STATUSTEXT govdesi 50 karakter (mavlink_io 50'ye kirpiyor).
  - ':' ayracli, mevcut yukari-yon (StatusTextParser) konvansiyonuyla uyumlu.
  - Sunucu sistem saati her pakete eklenir (sartname 6.1.3) -> ms epoch.
  - 50 char'i asabilecek payload'lar (uzun QR) icin chunk() ile cok-parcali.

Formatlar (hepsi ms epoch = sunucu saati):
  KLT:<srv_ms>:<end_ms>:<state>            kilitlenme paketi (state 1=aktif/4sn, 0=koptu)
  KBX:<cx>:<cy>:<w>:<h>:<iw>:<ih>          kilit dortgeni konumu (piksel) [opsiyonel ek]
  QRR:<srv_ms>:<payload>                   QR/kamikaze sonucu
  MOD:<srv_ms>:<mode>                      anlik ucus/gorev modu
Cok-parcali: <PFX>[<part>/<total>]:<content>  (yukari-yon ile ayni desen)
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import List, Optional

STATUSTEXT_MAX = 50


@dataclass
class _ServerClock:
    """Yerel saat <-> sunucu ms epoch eslemesi."""
    offset_ms: float = 0.0   # server_ms - local_ms
    synced: bool = False

    def sync(self, server_ms: int, local_epoch_ms: Optional[float] = None) -> None:
        local = local_epoch_ms if local_epoch_ms is not None else time.time() * 1000.0
        self.offset_ms = float(server_ms) - local
        self.synced = True

    def now_ms(self) -> int:
        return int(time.time() * 1000.0 + self.offset_ms)


class DownlinkEncoder:
    """STATUSTEXT downlink kodlayici. send_fn(text, severity) ile gonderir."""

    def __init__(self, send_fn=None):
        self._send = send_fn
        self.clock = _ServerClock()
        self.sent = 0
        self.dropped_oversize = 0

    # ---- sunucu saati ----
    def sync_server_clock(self, server_ms: int,
                          local_epoch_ms: Optional[float] = None) -> None:
        self.clock.sync(server_ms, local_epoch_ms)

    def server_ms(self) -> int:
        return self.clock.now_ms()

    # ---- encode ----
    def encode_lock(self, active: bool, lock_end_ms: Optional[int] = None,
                    srv_ms: Optional[int] = None) -> str:
        s = srv_ms if srv_ms is not None else self.server_ms()
        end = lock_end_ms if lock_end_ms is not None else s
        return f"KLT:{s}:{end}:{1 if active else 0}"

    def encode_lock_box(self, cx: float, cy: float, w: float, h: float,
                        iw: int, ih: int) -> str:
        return f"KBX:{int(cx)}:{int(cy)}:{int(w)}:{int(h)}:{int(iw)}:{int(ih)}"

    def encode_qr(self, payload: str, srv_ms: Optional[int] = None) -> str:
        s = srv_ms if srv_ms is not None else self.server_ms()
        return f"QRR:{s}:{payload}"

    def encode_mode(self, mode: str, srv_ms: Optional[int] = None) -> str:
        s = srv_ms if srv_ms is not None else self.server_ms()
        return f"MOD:{s}:{mode}"

    # ---- cok-parcali ----
    @staticmethod
    def chunk(prefix: str, content: str,
              limit: int = STATUSTEXT_MAX) -> List[str]:
        """content'i <PFX>[i/n]:... cerceveler halinde boler (>limit ise).

        Tek parca limite sigiyorsa onek/parca eklenmez (oldugu gibi doner).
        """
        full = f"{prefix}:{content}" if prefix else content
        if len(full) <= limit:
            return [full]
        # her parca: '<PFX>[i/n]:' + govde <= limit
        # onek uzunlugunu en kotu (n>=10) icin tahmin et
        head_len = len(prefix) + len("[99/99]:")
        body_room = max(1, limit - head_len)
        bodies = [content[i:i + body_room] for i in range(0, len(content), body_room)]
        n = len(bodies)
        return [f"{prefix}[{i + 1}/{n}]:{b}" for i, b in enumerate(bodies)]

    # ---- gonder ----
    def send(self, text: str, severity: int = 6) -> bool:
        if len(text) > STATUSTEXT_MAX:
            self.dropped_oversize += 1
            return False
        if self._send is None:
            return False
        self._send(text, severity)
        self.sent += 1
        return True

    def send_chunked(self, prefix: str, content: str, severity: int = 6) -> int:
        frames = self.chunk(prefix, content)
        ok = 0
        for f in frames:
            if self._send is not None and len(f) <= STATUSTEXT_MAX:
                self._send(f, severity)
                ok += 1
        self.sent += ok
        return ok

    # ---- ust seviye kisayollar ----
    def send_lock(self, active: bool, lock_end_ms: Optional[int] = None) -> bool:
        return self.send(self.encode_lock(active, lock_end_ms))

    def send_lock_box(self, cx, cy, w, h, iw, ih) -> bool:
        return self.send(self.encode_lock_box(cx, cy, w, h, iw, ih))

    def send_qr(self, payload: str) -> int:
        # QR payload uzun olabilir -> srv_ms onekiyle chunk
        s = self.server_ms()
        return self.send_chunked("QRR", f"{s}:{payload}")

    def send_mode(self, mode: str) -> bool:
        return self.send(self.encode_mode(mode))


# ---- test/diagnostik icin cozucu (downlink'i geri okur) ----
@dataclass
class DecodedDownlink:
    kind: str
    fields: List[str]


def decode_downlink(text: str) -> Optional[DecodedDownlink]:
    if not text or ":" not in text:
        return None
    head, _, rest = text.partition(":")
    if head not in ("KLT", "KBX", "QRR", "MOD"):
        return None
    return DecodedDownlink(kind=head, fields=rest.split(":"))
