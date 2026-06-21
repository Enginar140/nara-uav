# Nara v10 — Savaşan İHA Companion Otonomi (py_uav)

## What This Is

TEKNOFEST Savaşan İHA yarışması için sabit-kanat İHA'nın **companion bilgisayar otonomi sistemi**. Felsefe: *"Python plans, ArduPlane flies"* — companion nereye/hangi sırayla gidileceğine karar verir (algılama, takip, planlama, dalış geometrisi), düz-hat takibini (L1/TECS) ArduPlane firmware yapar. Companion, takımın Mission Planner tabanlı yer kontrol istasyonu (GCS) üzerinden yarışma sunucusuyla dolaylı haberleşir.

Bu milestone'un odağı: mevcut olgun otonomi yığınını **şartname-uyumlu hale getirmek** — companion↔GCS veri kontratını sonlandırmak, operasyonel mod mantığını düzeltmek ve uçtan uca doğrulamak.

## Core Value

**Kilitlenme ve kamikaze görevlerinin şartname-geçerli veri akışıyla puanlanması** — İHA hedefe kilitlense/dalsa bile, doğru formatta + sunucu saatiyle damgalanmış paket gönderilmezse puan YOK. Veri yolu doğru çalışmadan otonominin hiçbir değeri yok.

## Requirements

### Validated

<!-- Mevcut koddan çıkarıldı; birim testlerle (~207 pytest) doğrulanmış, SITL E2E PENDING -->

- ✓ 3 kademeli yol planlama (direkt → görünürlük grafı → hybrid A* + Dubins → indirgenmiş graf) — existing
- ✓ Seyrek waypoint çıktısı + dönüş-öngörülü devir (yalpalama önleme) — existing
- ✓ NavigationEngine tek-kanal arbitrasyon (ROTA/KURS/FBW) — existing
- ✓ 4 mod görev durum makinesi (ARDUPILOT/PATROL/FOLLOW/QR_DIVE) — existing
- ✓ Dört katmanlı emniyet (SafetyGate inline + SafetyMonitor ~20Hz + EscapeManager + ileri-projeksiyon) — existing
- ✓ Perception engine + combat tracker (YOLO/ROI/Kalman/PID/lock FSM) — existing
- ✓ MAVLink köprü + STATUSTEXT uplink parse (RIV/HSS/BND/MODE) + ArduPilot param adaptasyonu — existing
- ✓ Kapalı-döngü sim (fake_vehicle) + senaryo testleri + trace/dashboard araçları — existing

### Active

<!-- Bu milestone'da inşa edilecek. Şartname-uyumu + operasyonel mantık + doğrulama. -->

**A. Şartname-uyumlu haberleşme kontratı (companion ↔ GCS)**
- [ ] Companion→GCS downlink: `KBX` (kilit + hedef bbox) sürekli yayını
- [ ] Companion→GCS downlink: `MOD` (görev modu + gerçek otonom bayrağı) yayını
- [ ] Companion→GCS downlink: `KLT` (kilitlenme bitiş zamanı + otonom) olay paketi
- [ ] Companion→GCS downlink: `QRR` (kamikaze başlangıç + bitiş + qrMetni) olay paketi — format düzeltmesi
- [ ] GCS→Companion uplink: `SRV` (sunucu saati) periyodik mesaj + `_ServerClock` senkronu
- [ ] Companion sunucu saatiyle olayları kendi damgalar (`{gun,saat,dakika,saniye,milisaniye}` üretimi)
- [ ] `KAMIKAZE:LAT:LON[:ALT]` format hizalaması (GCS 3-alan ↔ companion 2-alan)
- [ ] Çok-parçalı STATUSTEXT chunk öneki GCS `ChunkSTATUSTEXT` ile hizalama doğrulaması

**B. Operasyonel mod mantığı düzeltmeleri**
- [ ] FOLLOW re-target: `LOCK:başkaID` → doğrudan hedef değiştir (`FOLLOW→FOLLOW` meşru geçiş)
- [ ] FOLLOW amaç: kilidi olabildiğince uzun tut (arkada istasyon-tutma, sürekli kilit süresi maks)
- [ ] QR_DIVE abort: min irtifaya kadar QR okunamazsa → CLIMB/abort
- [ ] QR_DIVE her zaman entry koridorundan giriş (yakın hedefte atlama yok)
- [ ] Global öncelik hiyerarşisi netliği (sınır-dışı > çarpışma escape > RTL > ARDUPILOT > görev)
- [ ] Mod komutları operatör/GCS kaynaklı (sunucu değil) — tetik omurgası teyidi

**C. Çapraz-sistem (GCS / Mission Planner tarafı — ayrı repo `mp v2`)**
- [ ] GCS companion downlink parse (KBX/MOD/KLT/QRR receiver) ekle
- [ ] GCS telemetri stub'larını kaldır: `iha_kilitlenme=0`, `hedef_merkez_*=0`, `iha_otonom` AP-mod-string yerine companion verisi
- [ ] GCS `SendLockingInfo` / `SendKamikazeInfo` companion olaylarıyla tetiklensin
- [ ] GCS `SRV` sunucu-saati uplink'i companion'a yollasın

**D. Uçtan uca doğrulama**
- [ ] SITL E2E: telemetri ≥1Hz (≤2Hz), kilit→kilitlenme paketi→puan akışı
- [ ] SITL E2E: kamikaze→QR metni okuma→kamikaze paketi akışı
- [ ] Gerçek FBWA RC override firmware teyidi (uplink/stall/failsafe)

### Out of Scope

- Yarışma sunucusu HTTP/JSON API'sinin companion'da implementasyonu — **GCS hallediyor** (companion sadece STATUSTEXT)
- Boundary'nin sunucudan çekilmesi — şartnamede API yok; uçuş alanı önceden duyurulur, config'ten yüklenir
- Velocity-tabanlı güdüm — sabit-kanat tasarım kuralı: yalnız pozisyon/kurs/FBWA
- Yeni planlayıcı/mimari paralel sistem — mevcut katmanlar genişletilir (AGENTS.md §12)
- ArduPlane firmware değişikliği — stock GUIDED + FBWA kullanılır

## Context

- **Mevcut sistem:** C++ `cpp_uav/` portunun Python yeniden yazımı; ~64 kaynak dosya, ~207 pytest testi. Birim seviyesi sağlam; **SITL uçtan uca doğrulama PENDING** (AGENTS.md §13).
- **GCS:** Takımın kendi Mission Planner fork'u (`C:\Users\Engin\Desktop\Nara\mp v2`, ArduPilot/MissionPlanner master üzerine untracked değişiklikler). `GCSViews/SavasanIHA.cs` (2057 satır) sunucu API'sini implemente ediyor; `SavasanIHAService.cs` boş placeholder. `GCS_COMMUNICATION_GUIDE.md` uplink kontratını belgeliyor.
- **Şartname:** `SIHA_Haberlesme_Dokumani_2026` — sunucu HTTP/JSON REST API; telemetri ≥1Hz/≤2Hz; sunucu saati `{gun,saat,dakika,saniye,milisaniye}`; kilitlenme/kamikaze olayları sunucu saatiyle damgalı tek paket; QR Versiyon 1, qrMetni okunmalı; HSS yalnız hakem duyurusunda.
- **Tespit edilen boşluk:** GCS sunucuya `iha_kilitlenme=0`, `hedef_merkez_*=0` (sabit stub) gönderiyor; `iha_otonom` AP mod string'inden türetiliyor. Companion bu verileri üretiyor ama GCS'e bir kontratla iletmiyor → puan riski.

## Constraints

- **Tech stack**: Python 3.10+, pymavlink, ArduPlane GUIDED/FBWA — velocity komutu YOK; yalnız DO_REPOSITION / GUIDED_CHANGE_* / RC_CHANNELS_OVERRIDE
- **Haberleşme**: Companion ↔ GCS yalnız MAVLink STATUSTEXT (50 karakter gövde, `:` ayraçlı, çok-parçalı chunk); GCS ↔ sunucu HTTP/JSON (companion'a şeffaf)
- **Şartname**: Telemetri 1–2 Hz; tüm olay zamanları sunucu saati; kilitlenme/kamikaze bitiminde 1 paket; paket gönderilmeyen görev puanlanmaz
- **Emniyet**: SafetyGate HARD marj; min irtifa dalışta da geçerli (ihlal = ceza); HSS dairelerinden kaçınma
- **Dependencies**: GCS (ayrı repo) ile iki-taraflı kontrat — bir tarafın değişimi diğerini etkiler
- **Compatibility**: SITL `vision_tracker` (sentetik) ≠ prod `combat_vision` (YOLO+kamera)

## Key Decisions

| Decision | Rationale | Outcome |
|----------|-----------|---------|
| Companion↔GCS yalnız STATUSTEXT; GCS sunucu JSON API'sini sahiplenir | GCS zaten JSON paketliyor; companion'da HTTP yükü gereksiz | — Pending |
| Sunucu saatini companion damgalar (GCS `SRV` yollar) | Kilit/dalış bitiş anı hassasiyeti companion'da en doğru | — Pending |
| FOLLOW kilit hedefini operatör (GCS `LOCK:ID`) seçer | Şartnamede sunucu hedef vermez; seçim takım taktiği, operatör kontrolü | — Pending |
| Kamikaze hedefi GCS `qr_koordinati`→`KAMIKAZE:LAT:LON` ile gelir | QR konumu sunucu API'sinden; GCS çekip companion'a yollar | — Pending |
| `FOLLOW→FOLLOW` re-target geçişi eklenir | `LOCK:başkaID`'de PATROL ara durağı mantıksız | — Pending |
| QR_DIVE→PATROL korunur (on_complete=patrol) | PATROL her komuta açık; sunucu hemen yönlendirebilir | — Pending |
| Mantıklı faz sırası: kontrat → companion downlink → GCS parse → mod mantığı → E2E | Veri yolu olmadan mod mantığı/doğrulama anlamsız | — Pending |
| GSD kapsamı = py_uav (companion); GCS değişiklikleri entegrasyon gereksinimi | Ayrı repo; kod orada uygulanır, burada izlenir | — Pending |

## Evolution

This document evolves at phase transitions and milestone boundaries.

**After each phase transition** (via `/gsd-transition`):
1. Requirements invalidated? → Move to Out of Scope with reason
2. Requirements validated? → Move to Validated with phase reference
3. New requirements emerged? → Add to Active
4. Decisions to log? → Add to Key Decisions
5. "What This Is" still accurate? → Update if drifted

**After each milestone** (via `/gsd:complete-milestone`):
1. Full review of all sections
2. Core Value check — still the right priority?
3. Audit Out of Scope — reasons still valid?
4. Update Context with current state

---
*Last updated: 2026-06-21 after initialization*
