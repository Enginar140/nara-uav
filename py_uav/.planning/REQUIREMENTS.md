# Requirements: Nara v10 — Savaşan İHA Companion Otonomi

**Defined:** 2026-06-21
**Core Value:** Kilitlenme ve kamikaze görevlerinin şartname-geçerli veri akışıyla puanlanması — doğru formatta + sunucu saatiyle damgalı paket gönderilmezse puan yok.

## v1 Requirements

İlk sürüm için gereksinimler. Her biri roadmap fazlarına eşlenir.

### Haberleşme — Companion Downlink/Uplink (COMM)

- [ ] **COMM-01**: Companion, kilit durumu + hedef kutusunu `KBX:<kilit>:<cx>:<cy>:<w>:<h>:<iw>:<ih>` paketiyle GCS'e sürekli (telemetri hızında) yayınlar
- [ ] **COMM-02**: Companion, anlık görev modu + gerçek otonom bayrağını `MOD:<srv_ms>:<mod>:<otonom>` paketiyle yayınlar
- [ ] **COMM-03**: Companion, kilitlenme penceresi bitince `KLT:<bitis_srv_ms>:<otonom>` olay paketini yalnızca **bir kez** gönderir
- [ ] **COMM-04**: Companion, kamikaze bitince `QRR:<basla_srv_ms>:<bitis_srv_ms>:<qrMetni>` olay paketini yalnızca **bir kez** gönderir (başlangıç+bitiş+metin)
- [ ] **COMM-05**: Companion, GCS'ten gelen `SRV:<gun>:<saat>:<dakika>:<saniye>:<milisaniye>` ile sunucu saatini senkronlar ve tüm olayları sunucu saatiyle damgalar
- [ ] **COMM-06**: Companion, `KAMIKAZE:LAT:LON[:ALT]` formatını kabul eder (3. alan opsiyonel ALT; GCS 3-alan ↔ companion hizalaması)
- [ ] **COMM-07**: 50 karakteri aşan downlink paketleri, GCS `ChunkSTATUSTEXT` ile aynı `[part/total]` deseninde çok-parçalı gönderilir

### Operasyonel Mod Mantığı (MODE)

- [ ] **MODE-01**: `LOCK:başkaID` geldiğinde FOLLOW, PATROL ara durağı olmadan doğrudan yeni hedefe geçer (`FOLLOW→FOLLOW`)
- [ ] **MODE-02**: FOLLOW PURSUIT, hedefin arkasında sabit mesafede istasyon tutarak kesintisiz kilit süresini maksimize eder
- [ ] **MODE-03**: QR_DIVE, minimum irtifaya kadar QR okunamazsa dalışı abort edip CLIMB'e geçer
- [ ] **MODE-04**: QR_DIVE her zaman entry koridorundan girer (hedef yakın olsa bile koridor atlanmaz)
- [ ] **MODE-05**: Global öncelik (sınır-dışı kurtarma > çarpışma escape > RTL > ARDUPILOT > görev) her modu deterministik ezer
- [ ] **MODE-06**: Mod komutları operatör/GCS kaynaklıdır (FREE/LOCK/KAMIKAZE/ARDUPILOT); sunucu mod komutu vermez

### GCS Entegrasyonu — Mission Planner (çapraz-sistem, ayrı repo `mp v2`) (GCS)

- [ ] **GCS-01**: GCS, companion downlink paketlerini (KBX/MOD/KLT/QRR) parse eden bir receiver içerir
- [ ] **GCS-02**: GCS telemetrisinde `iha_kilitlenme` ve `hedef_merkez/genislik/yukseklik` stub'ları (sabit 0) companion `KBX` verisiyle doldurulur
- [ ] **GCS-03**: GCS, `iha_otonom`'u companion `MOD` verisinden üretir (AP mod string'i yerine)
- [ ] **GCS-04**: GCS `SendLockingInfo`/`SendKamikazeInfo`, companion `KLT`/`QRR` olaylarıyla tetiklenir ve sunucuya POST eder
- [ ] **GCS-05**: GCS, sunucu saatini companion'a `SRV` paketiyle periyodik yollar

### Uçtan Uca Doğrulama (VALID)

- [ ] **VALID-01**: SITL'de telemetri 1–2 Hz aralığında akar; kilit anında `iha_kilitlenme=1` + geçerli bbox üretilir
- [ ] **VALID-02**: SITL'de tam kilitlenme akışı doğrulanır (LOCK → PURSUIT → kilit penceresi → KLT → sunucu kabul)
- [ ] **VALID-03**: SITL'de tam kamikaze akışı doğrulanır (KAMIKAZE → dalış → QR okuma → QRR → sunucu kabul)
- [ ] **VALID-04**: Gerçek FBWA RC override firmware'de teyit edilir (uplink/stall/failsafe davranışı)

## v2 Requirements

Gelecek sürüme ertelendi. İzleniyor ama mevcut roadmap'te değil.

### Otonomi Geliştirmeleri

- **AUTO-01**: Companion, operatör ID vermezse `konumBilgileri`'nden en uygun rakibi otonom seçer (hibrit hedefleme)
- **AUTO-02**: `zaman_farki` ile rakip konum tahmini/lead iyileştirmesi (FOLLOW APPROACH)

## Out of Scope

Açıkça hariç. Kapsam kaymasını önlemek için belgelenmiştir.

| Feature | Reason |
|---------|--------|
| Sunucu HTTP/JSON API'sinin companion'da implementasyonu | GCS hallediyor; companion yalnız STATUSTEXT |
| Boundary'nin sunucudan çekilmesi | Şartnamede API yok; uçuş alanı önceden duyurulur, config'ten yüklenir |
| Velocity-tabanlı güdüm | Sabit-kanat tasarım kuralı: yalnız pozisyon/kurs/FBWA |
| Yeni planlayıcı/paralel mimari | Mevcut katmanlar genişletilir (AGENTS.md §12) |
| ArduPlane firmware değişikliği | Stock GUIDED + FBWA kullanılır |

## Traceability

Hangi faz hangi gereksinimi kapsıyor. Roadmap oluşturulurken doldurulur.

| Requirement | Phase | Status |
|-------------|-------|--------|
| COMM-01 | TBD | Pending |
| COMM-02 | TBD | Pending |
| COMM-03 | TBD | Pending |
| COMM-04 | TBD | Pending |
| COMM-05 | TBD | Pending |
| COMM-06 | TBD | Pending |
| COMM-07 | TBD | Pending |
| MODE-01 | TBD | Pending |
| MODE-02 | TBD | Pending |
| MODE-03 | TBD | Pending |
| MODE-04 | TBD | Pending |
| MODE-05 | TBD | Pending |
| MODE-06 | TBD | Pending |
| GCS-01 | TBD | Pending |
| GCS-02 | TBD | Pending |
| GCS-03 | TBD | Pending |
| GCS-04 | TBD | Pending |
| GCS-05 | TBD | Pending |
| VALID-01 | TBD | Pending |
| VALID-02 | TBD | Pending |
| VALID-03 | TBD | Pending |
| VALID-04 | TBD | Pending |

**Coverage:**
- v1 requirements: 22 total
- Mapped to phases: 0 (roadmap pending)
- Unmapped: 22 ⚠️ (roadmapper dolduracak)

---
*Requirements defined: 2026-06-21*
*Last updated: 2026-06-21 after initial definition*
