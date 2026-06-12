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
- **Sanal FBWA (`nav/attitude_law.py`):** Takip/dalış terminal fazlarında GUIDED
  içinde `SET_ATTITUDE_TARGET` ile doğrudan roll/pitch açı talebi — FBWA'nın
  kontrol yetkisi, ama mod GUIDED kaldığı için şartnameye göre **otonom** sayılır
  (gerçek FBWA yarı-oto = manuel, 6.1.7). Her komut öncesi ileri-projeksiyon
  emniyet süpervizörü (HSS/sınır/irtifa bandı + stall koruması); ihlalde komut
  gönderilmez ve görev modu pozisyon güdümüne düşer; akış kesilirse firmware
  GUIDED timeout failsafe'i devrededir. `nav.attitude.enabled` ile açılır.

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
| nav | planner, dubins, cost_field, hybrid_astar, waypoint_chain, speed_manager, navigation_engine | **3 kademeli planlayıcı (konfor grafı → hybrid A\*/Dubins → indirgenmiş graf)**; GUIDED sequencing |
| mission | patrol, follow (PN), qr_dive, mission_director | Görev modları + durum makinesi |
| app | application, main | asyncio orchestration |

## Yol planlama (3 kademe)

`nav/planner.py` + `dubins.py` + `cost_field.py` + `hybrid_astar.py`
(shapely + networkx + pyproj; tüm geometri `LocalFrame` AEQD metre-düzleminde):

- **İki güvenlik kavramı:** HARD = HSS yarıçapı + `hss_buffer_m` (SafetyGate ile
  aynı metrik, asla ihlal edilmez); KONFOR = HARD + dönüş + escape marjı.
- **Kademe 0/1 (açık alan):** direkt konfor-temiz ya da konforlu visibility
  graph — bugüne kadarki geniş-klerans davranışı aynen.
- **Kademe 2 (hybrid A\*/Dubins):** konfor marjı rota seçeneklerini mühürlerse
  (dar HSS koridoru) ya da **varış heading'i** istenirse devreye girer. Sert
  duvar yalnız HARD + izleme payı; konfor bandı **maliyettir** — koridor tek
  seçenekse koridordan geçilir. Arama (x, y, heading) uzayında, hıza bağlı
  dönüş yarıçapıyla (`v²/(g·tanφ)`) — çıkan yol uçulabilirdir. Hedefe analitik
  **Dubins shot** varış heading'ini garanti eder (QR dalış girişi).
- **Kademe 3 (son çare):** indirgenmiş marjlı graf — hybrid bütçeyi aşarsa
  "yol yok" yerine heading-kör ama HARD-güvenli rota.
- **Homotopi imzası:** plan hangi HSS'i hangi yandan geçtiğini taşır; replan
  önceki yanı tercih eder (yan flip-flop önlenir).
- `make_safe(waypoint_list)`: bacak bacak güvenli hâle getirir; bacaklar arası
  heading sürekliliği korunur.
- Çıktı her zaman düz waypoint listesidir (arc'lar `max_arc_step_deg` ile
  örneklenir) — ArduPlane yine yalnız `DO_REPOSITION` hedefleri uçar.

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
cd py_uav && python -m pytest -q          # 140 test
```

## STATUSTEXT formatları (GCS → companion)

```
RIV:ID:LAT:LON:ALT:HDG:SPD:TS
HSS:COUNT:ID:LAT:LON:RAD:...
BND:COUNT:LAT:LON:LAT:LON:...
FREE | ARDUPILOT | LOCK:ID | KAMIKAZE:LAT:LON
Çok parçalı: PREFIX[PART/TOTAL]:içerik
```
