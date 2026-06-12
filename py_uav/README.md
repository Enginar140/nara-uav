# Nara v10 — Python İHA Otonomi Sistemi

TEKNOFEST sabit-kanat İHA otonomi sistemi. `cpp_uav/` C++ sürümünün Python'da,
**sabit-kanat gerçeklerine göre yeniden tasarlanmış** hâli.

## Tasarım felsefesi: "Python plans, ArduPlane flies"

Companion bilgisayar (bu kod) yalnız **üst düzey** kararlar verir; gerçek uçuş
ArduPlane içinde olur. Üst düzey hata yüzeyi minimumdur:

- **Companion'ın işi:** verilen waypoint listesini HSS/fence-güvenli hâle getirir,
  sabit-kanat dönüş yarıçapına göre güvenli **koridor marjı** bırakır, GUIDED modda
  `DO_REPOSITION` ile waypoint'leri sırayla gönderir.
- **ArduPlane'in işi:** L1/NPFG + TECS ile gerçek uçuş, dönüş, loiter, geofence,
  bank limit, stall önleme — hepsi her zaman aktif.
- **Velocity (hız) komutu YOK.** ArduPlane sabit-kanatta hız setpoint'ini yok sayar
  (kaynak: `cpp_uav_external/ardupilot_repo` nara/local-control fork'u). Sistem
  yalnız **pozisyon hedefleri** gönderir: `DO_REPOSITION` (global) ve
  `SET_POSITION_TARGET_LOCAL_NED` BODY/LOCAL **pozisyon ofseti** (terminal dalış).

## Mimari (katmanlar)

```
core → config → infra → world_model → comm → safety → nav → mission → app
```

| Katman | Modüller | İş |
|--------|----------|-----|
| core | types, geo, time_utils | Veri yapıları, geo matematiği (+ pyproj LocalFrame) |
| config | config_manager | YAML dotted-get |
| infra | log, event_bus | Loglama, asyncio pub/sub (thread-güvenli publish) |
| world_model | world_model | Thread-güvenli dünya durumu (own/opponents/HSS/boundary) |
| comm | mavlink_io, flight_commander, statustext_parser, mavlink_bridge | pymavlink; GUIDED komut yüzeyi; RIV/HSS/BND/MODE parse |
| safety | zone_checker, safety_gate, safety_monitor, escape_manager | Komut clamp, 20Hz watchdog, kaçış |
| nav | planner, waypoint_chain, speed_manager, navigation_engine | **shapely visibility-graph + turn-radius koridor**; GUIDED sequencing |
| mission | patrol, follow (PN), qr_dive, mission_director | Görev modları + durum makinesi |
| app | application, main | asyncio orchestration |

## Yol planlama

`nav/planner.py` — shapely + networkx + pyproj:

- Tüm geometri tek bir `LocalFrame` (pyproj AEQD) metre-düzleminde.
- **İki katman marj:** HARD = HSS yarıçapı + `hss_buffer_m` (asla ihlal edilmez);
  SOFT = HARD + `turn_safety_margin_m` (detour'lar buradan geçer, böylece uçak
  WP'leri keserken HARD koridora girmez).
- Düz segment soft-clear ise direkt; değilse HSS dairelerini soft poligona genişlet
  → visibility graph → Dijkstra → line-of-sight simplify → densify → HARD doğrulama.
- `make_safe(waypoint_list)`: GCS'ten gelen waypoint listesini bacak bacak güvenli
  hâle getirir ("sürekli waypoint listesi" senaryosu).
- Arc/Dubins **waypoint üretmez** — ArduPlane kendi yumuşatmasını yapar.

## Kurulum

```bash
pip install -r requirements.txt   # pymavlink numpy shapely networkx pyproj PyYAML
```

## Çalıştırma

```bash
cd py_uav
PYTHONPATH=src python -m uav.main --config config/sitl.yaml
```

### SITL ile

```bash
# ArduPlane SITL (ayrı terminal)
sim_vehicle.py -v ArduPlane --out=udp:127.0.0.1:14550

# companion
PYTHONPATH=src python -m uav.main --config config/sitl.yaml
```

GCS'ten STATUSTEXT ile komut: `BND:...`, `HSS:...`, `FREE` (PATROL),
`LOCK:<id>` (FOLLOW), `KAMIKAZE:<lat>:<lon>` (QR_DIVE), `ARDUPILOT`.

## Test

```bash
cd py_uav && python -m pytest -q          # 64 test
```

## STATUSTEXT formatları (GCS → companion)

```
RIV:ID:LAT:LON:ALT:HDG:SPD:TS
HSS:COUNT:ID:LAT:LON:RAD:...
BND:COUNT:LAT:LON:LAT:LON:...
FREE | ARDUPILOT | LOCK:ID | KAMIKAZE:LAT:LON
Çok parçalı: PREFIX[PART/TOTAL]:içerik
```
