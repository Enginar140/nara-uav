# Roadmap: Nara v10 — Savaşan İHA Companion Otonomi (Şartname Uyumu)

## Overview

Mevcut, birim-test düzeyinde sağlam bir Python companion otonomi yığınını
TEKNOFEST şartname-uyumlu hale getirme yolculuğu. Önce veri yolu kontratı
kurulur (companion↔GCS STATUSTEXT şeması + sunucu saati senkronu), ardından
companion downlink paketleri üretilir, GCS tarafı entegre edilir, operasyonel
mod mantığı düzeltilir ve son olarak uçtan uca SITL + firmware doğrulamasıyla
tümü onaylanır. Veri yolu doğru çalışmadan diğer hiçbir şeyin puanlama değeri
yoktur.

## Phases

**Phase Numbering:**
- Integer phases (1, 2, 3): Planned milestone work
- Decimal phases (2.1, 2.2): Urgent insertions (marked with INSERTED)

Decimal phases appear between their surrounding integers in numeric order.

- [ ] **Phase 1: Haberleşme Kontratı** - Companion↔GCS STATUSTEXT şeması, sunucu saati senkronu ve chunk protokolünü belgele ve sabitle
- [ ] **Phase 2: Companion Downlink Üretimi** - Companion tarafında KBX/MOD/KLT/QRR paketlerini üret ve yayınla
- [ ] **Phase 3: GCS Entegrasyonu** - Mission Planner (mp v2) tarafında downlink parser, stub kaldırma ve SRV uplink ekle (çapraz-sistem, ayrı repo)
- [ ] **Phase 4: Operasyonel Mod Mantığı** - FOLLOW re-target, kilit-tutma, QR_DIVE abort/koridor, global öncelik ve komut-kaynak düzeltmeleri
- [ ] **Phase 5: Uçtan Uca Doğrulama** - SITL E2E telemetri + kilitlenme + kamikaze akışları ve gerçek FBWA firmware teyidi

## Phase Details

### Phase 1: Haberleşme Kontratı
**Goal**: Companion ile GCS arasındaki tüm STATUSTEXT paketi biçimleri, sunucu saati senkron mekanizması ve chunk protokolü netleştirilmiş, belgelenmiş ve uygulamaya hazırdır
**Depends on**: Nothing (first phase)
**Requirements**: COMM-05, COMM-06, COMM-07
**Success Criteria** (what must be TRUE):
  1. `SRV:<gun>:<saat>:<dakika>:<saniye>:<milisaniye>` mesajı companion tarafından parse edilip yerel sunucu saati referansı olarak saklanır; tüm olay damgaları bu referanstan türer
  2. Companion, `KAMIKAZE:LAT:LON` (2 alan) ile `KAMIKAZE:LAT:LON:ALT` (3 alan) formatlarının her ikisini de hatasız parse eder; eski 2-alan gelen paket artık düşmez
  3. 50 karakteri aşan companion çıkışı, `[part/total]` önekiyle GCS `ChunkSTATUSTEXT` deseniyle aynı şekilde bölümlere ayrılır; GCS tek parçaymış gibi yeniden birleştirir
  4. Kontrat belgesi (`GCS_COMMUNICATION_GUIDE.md` güncellemesi veya eşdeğeri) tüm paket biçimlerini, alan sınırlarını ve chunk kuralını tek yerde tanımlar
**Plans**: TBD

### Phase 2: Companion Downlink Üretimi
**Goal**: Companion, şartname olay paketlerinin tamamını (KBX/MOD/KLT/QRR) doğru format ve sunucu saatiyle GCS'e üretip yayınlar
**Depends on**: Phase 1
**Requirements**: COMM-01, COMM-02, COMM-03, COMM-04
**Success Criteria** (what must be TRUE):
  1. GCS log'larında her telemetri döngüsünde `KBX:<kilit>:<cx>:<cy>:<w>:<h>:<iw>:<ih>` paketi görünür; kilit=1 olduğunda bbox alanları sıfır değildir
  2. GCS log'larında her döngüde `MOD:<srv_ms>:<mod>:<otonom>` paketi görünür; `<otonom>` companion misyon durumundan gelir, ArduPlane mod string'inden değil
  3. Kilitlenme penceresi kapandığında GCS tam olarak bir `KLT:<bitis_srv_ms>:<otonom>` paketi alır; aynı kilitlenme olayı için ikinci paket gönderilmez
  4. Kamikaze tamamlandığında GCS tam olarak bir `QRR:<basla_srv_ms>:<bitis_srv_ms>:<qrMetni>` paketi alır; `<qrMetni>` boş değildir ve `<bitis_srv_ms>` >= `<basla_srv_ms>` koşulunu sağlar
**Plans**: TBD

### Phase 3: GCS Entegrasyonu
**Goal**: Mission Planner (mp v2) companion downlink paketlerini (KBX/MOD/KLT/QRR) parse eder, sabit stub değerlerini companion verisiyle doldurur ve SRV saatini companion'a geri yollar
**Depends on**: Phase 2
**Requirements**: GCS-01, GCS-02, GCS-03, GCS-04, GCS-05
**Notes**: Çapraz-sistem entegrasyon fazı. Kod `mp v2` reposunda (C#) uygulanır; bu fazda companion tarafı sadece kontrat uyumunu korur. Bağımlılık yönetimi kritik.
**Success Criteria** (what must be TRUE):
  1. GCS telemetrisinde `iha_kilitlenme` alanı, companion `KBX` verisiyle gerçek zamanlı güncellenir; artık sabit 0 değildir
  2. GCS telemetrisinde `hedef_merkez_x/y`, `hedef_genislik`, `hedef_yukseklik` alanları companion `KBX` bbox değerlerini yansıtır
  3. GCS'teki `iha_otonom` bayrağı companion `MOD` paketindeki `<otonom>` alanından türer; ArduPlane mod string karşılaştırması artık kullanılmaz
  4. GCS `SendLockingInfo` yalnızca companion `KLT` olayı geldiğinde tetiklenir ve sunucuya POST eder; aynı şekilde `SendKamikazeInfo` yalnızca `QRR` olayıyla tetiklenir
  5. GCS periyodik olarak `SRV:<gun>:<saat>:<dakika>:<saniye>:<milisaniye>` paketini companion'a yollar; companion bu paketi alıp sunucu saatini günceller (Faz 1 senkronu canlı döngüde çalışır)
**Plans**: TBD

### Phase 4: Operasyonel Mod Mantığı
**Goal**: FOLLOW, QR_DIVE ve global öncelik mekanizmaları şartname senaryolarında doğru davranır; mod komutları operatör/GCS kaynaklı olarak kilitlenmiştir
**Depends on**: Phase 2
**Requirements**: MODE-01, MODE-02, MODE-03, MODE-04, MODE-05, MODE-06
**Success Criteria** (what must be TRUE):
  1. `LOCK:yeniID` komutu FOLLOW modundayken alındığında sistem PATROL ara durağına geçmeden doğrudan yeni hedefe geçer; takip kaybı ≤ 1 tick
  2. FOLLOW PURSUIT aşamasında companion hedefin arkasında sabit mesafede konumlanır; hedef hareket ettikçe istasyon tutulur ve kilit süresi kesintisiz uzar
  3. QR_DIVE dalışında minimum irtifaya kadar QR okunamazsa sistem CLIMB'e geçer ve PATROL'e döner; zemine çarpma riski oluşmaz
  4. QR_DIVE entry koridoru, hedef mevcut konuma yakın olsa dahi atlanmaz; tüm dalışlar koridor girişinden başlar
  5. Sınır-dışı veya çarpışma escape tetiklendiğinde görev modundan bağımsız olarak öncelik hiyerarşisi devreye girer ve hedef modu ezer; ARDUPILOT operatör override'ı tüm otonom escape'i bastırır
  6. Companion `FREE`, `LOCK`, `KAMIKAZE`, `ARDUPILOT` komutlarını yalnızca GCS/operatör kaynaklı STATUSTEXT'ten kabul eder; sunucu kaynaklı mod geçişi yoktur
**Plans**: TBD

### Phase 5: Uçtan Uca Doğrulama
**Goal**: SITL ortamında kilitlenme ve kamikaze görev akışlarının tamamı şartname uyumlu veri yolundan geçtiği doğrulanır; gerçek FBWA firmware davranışı teyit edilir
**Depends on**: Phase 3, Phase 4
**Requirements**: VALID-01, VALID-02, VALID-03, VALID-04
**Success Criteria** (what must be TRUE):
  1. SITL'de companion MAVLink köprüsü üzerinden ≥1 Hz / ≤2 Hz aralığında sürekli telemetri üretir; ölçülen frekans bu bant içinde kalır
  2. SITL'de tam kilitlenme döngüsü çalışır: LOCK komutu → FOLLOW PURSUIT → kilit penceresi dolumu → `KLT` paketi GCS'e ulaşır → GCS `SendLockingInfo` sunucuya POST eder ve sunucu 200 OK döner
  3. SITL'de tam kamikaze döngüsü çalışır: `KAMIKAZE:LAT:LON` komutu → QR_DIVE → dalış → QR metni okunur → `QRR` paketi GCS'e ulaşır → GCS `SendKamikazeInfo` sunucuya POST eder ve sunucu 200 OK döner
  4. Gerçek donanımda FBWA mod geçişi RC override uplink ile teyit edilir; stall/failsafe durumunda `release_fbw()` çağrılarak GUIDED'a dönüş gözlemlenir
**Plans**: TBD

## Progress

**Execution Order:**
Phases execute in numeric order: 1 → 2 → 3 → 4 → 5

| Phase | Plans Complete | Status | Completed |
|-------|----------------|--------|-----------|
| 1. Haberleşme Kontratı | 0/TBD | Not started | - |
| 2. Companion Downlink Üretimi | 0/TBD | Not started | - |
| 3. GCS Entegrasyonu | 0/TBD | Not started | - |
| 4. Operasyonel Mod Mantığı | 0/TBD | Not started | - |
| 5. Uçtan Uca Doğrulama | 0/TBD | Not started | - |
