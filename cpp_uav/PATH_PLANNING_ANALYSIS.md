# v10 Path Planning - Detaylı Analiz Raporu

## ÖZET
v10'un path planning sisteminde HSS engelinden kaçınmada kritik tasarım hataları ve bug'lar tespit edildi. v9.1 CostMap+A* sistemi ile karşılaştırıldı.

---

## 1. BULUNAN KRİTİK BUG'LAR

### 1.1 `path_planner.cpp:191` - Obstacle Skip Logic BUG
```cpp
// ÖNCEKI (YANLIŞ):
if (obs_idx == skip_obs_a && obs_idx == skip_obs_b) continue;

// ŞİMDİ (DOĞRU):
if (obs_idx == skip_obs_a || obs_idx == skip_obs_b) continue;
```
**Problem:** VE (&&) operatörü sadece iki parametre EŞİT olduğunda skip ediyordu. Bu demek ki obstacle kontrolü tamamen bypass oluyordu.

**Etki:** Visibility graph yanlış bağlantılar oluşturuyordu, HSS içinden geçen rotalar üretiyordu.

### 1.2 `path_planner.cpp:214-220` - Boundary Check BUG
```cpp
// ÖNCEKI (YETERSİZ):
core::Position mid{(a.latitude + b.latitude) / 2.0, ...};
if (!geo::isPointInPolygon(mid, boundary.vertices)) return false;

// ŞİMDİ (DOĞRU):
if (!geo::isPointInPolygon(a, boundary.vertices) ||
    !geo::isPointInPolygon(b, boundary.vertices)) return false;
```
**Problem:** Sadece orta nokta kontrol ediliyordu. Segment dışarı çıkıp tekrar içine girebiliyordu.

### 1.3 `isDirectPathClear` - Her Zaman True Dönüyor
```cpp
// path_planner.cpp:15-27
if (isDirectPathClear(start, goal, boundary, hss_zones)) {
    result.waypoints = {start, goal};  // Sadece 2 waypoint!
    return result;
}
```
**Problem:** `isDirectPathClear` her zaman `true` dönüyordu çünkü:
1. HSS zone'ların altitude range'i tanımsız (INFINITY)
2. `segmentIntersectsCircle` sadece 2D mesafe kontrolü yapıyor
3. Her seferinde visibility graph atlanıyordu

---

## 2. TASARIM EKSİKLİKLERİ (v9.1'de Var, v10'da Yok)

### 2.1 Altitude-Aware HSS Kontrolü
**v9.1:** Her zone için altitude range kontrolü yok (ama HSS mesajında da yok)
**v10:** Eklendi ama kullanılmıyor:
```cpp
// types.hpp - YENİ
struct HSSZone {
    double min_altitude_m = -INFINITY;
    double max_altitude_m = INFINITY;
    bool conflictsWithAltitude(double alt) const;
};
```

### 2.2 Path Sampling (Kritik!)
**v9.1:**
```cpp
// strategic_planner.cpp:948-981 - isPathSafe()
const int samples = 10;
for (int i = 0; i <= samples; ++i) {
    double t = static_cast<double>(i) / samples;
    core::Position sample = interpolate(from, to, t);
    double dist = core::geo::hashesineDistance(sample, zone.center);
    if (dist < safe_dist) return false;  // Path crosses HSS!
}
```

**v10:** Sampling YOK!
```cpp
// zone_checker.cpp:segmentIntersectsAnyHSS() - Sadece endpoint kontrolü
if (geo::segmentIntersectsCircle(from, to, zone.center, zone.radius + buffer)) {
    return true;
}
```

**Problem:** Uzun segment'ler HSS içinden geçebilir ama `segmentIntersectsCircle` bunu yakalamayabilir!

### 2.3 HSS İçindeki Goal Durumu
**v9.1:** Goal HSS içindeyse HESAPLAMAZIYOR (reject ediyor)
**v10:** Goal HSS içindeyse NE YAPILACAK BELİRLİ DEĞİL!

---

## 3. MİMARİ KARŞILAŞTIRMASI

| Özellik | v9.1 CostMap+A* | v10 Visibility Graph |
|---------|-----------------|---------------------|
| Algoritma | Grid A* | Geometric Visibility Graph |
| HSS Temsili | Grid marking | Polygon expansion |
| Path Smoothing | Adaptive sampling | turn-based smoothing |
| HSS Check | Cost-based sampling | Segment intersection |
| Waypoint Count | Grid resolution'a bağlı | Vertex sayısına bağlı |
| Memory | Grid size (1000x1000) | Minimal |
| 3D Support | Sınırlı | Teorik olarak var |
| Garanti | Grid kapsamıyorsa yok | Graph bağlıysa yok |

**V9.1 Avantajı:**
- Grid her zaman çözüm üretir (henüz yoksa)
- Sampling ile path validation çok güvenilir

**V10 Avantajı:**
- Daha az memory
- Daha temiz kod yapısı
- Optimal path (geometric olarak)

---

## 4. KRİTİK SORUN: 1 Waypoint Problem

Log analizi:
```
NavigationEngine: route planned, 1 waypoints, 1676m
NavigationEngine: waypoint WP1 unsafe, clamping
```

Bu `isDirectPathClear`'in true döndüğü anlamına geliyor! Çünkü:
1. HSS altitude range INFINITY
2. Segment intersection check yetersiz
3. Clamping waypoint'i HSS kenarına atıyor ama UAV giderken HSS içinden geçiyor

---

## 5. ÖNERİLER

### Seçenek A: v10 Visibility Graph'ü Tamamen Düzelt
**Artıları:**
- Mevcut mimariyi korur
- Daha az memory
- Daha hızlı (potansiyel)

**Eksileri:**
- Karmaşık geometric hesaplamalar
- Boundary case'ler için sampling gerekli

### Seçenek B: v9.1 CostMap+A* Sistemi Entegre Et
**Artıları:**
- Zaten test edilmiş ve çalışıyor
- Garantili path finding
- Adaptive path smoothing

**Eksileri:**
- Daha fazla memory
- Grid resolution sınırlamaları
- Daha karmaşık kod

### Seçenek C: Hibrit - İkisini Birleştir
- CostMap ile ön-çalışırma
- Visibility graph ile detaylı optimizasyon

---

## 6. ACİL YAPILMASI GEREKEN DÜZELTMELER (Her İki Seçenek İçin)

### 6.1 Path Sampling Ekle (Kritik!)
```cpp
// zone_checker.cpp'de veya path_planner.cpp'de
bool isSegmentSafeWithSampling(const Position& from, const Position& to,
                               const vector<HSSZone>& zones, double buffer) {
    // Önce basit kontrol
    if (segmentIntersectsAnyHSS(from, to, zones, buffer)) return false;
    
    // Sonra sampling ile doğrula (v9.1'den)
    constexpr int SAMPLES = 10;
    for (int i = 1; i < SAMPLES; ++i) {
        double t = static_cast<double>(i) / SAMPLES;
        Position sample;
        sample.latitude = from.latitude + t * (to.latitude - from.latitude);
        sample.longitude = from.longitude + t * (to.longitude - from.longitude);
        sample.altitude = from.altitude + t * (to.altitude - from.altitude);
        
        for (const auto& zone : zones) {
            if (!zone.active) continue;
            double dist = haversineDistance(sample, zone.center);
            if (dist < zone.radius + buffer) return false;
        }
    }
    return true;
}
```

### 6.2 Goal Validation Ekle
```cpp
// Goal HSS içindeyse RED et
if (isInsideAnyHSS(goal, hss_zones, hss_buffer)) {
    result.error = "Goal inside HSS zone - cannot safely reach";
    return result;
}
```

### 6.3 SimplifyPath'te Sampling Ekle
```cpp
// path_planner.cpp - simplifyPath fonksiyonunda
// Her bir waypoint çifti için sampling kontrolü ekle
```

---

## 7. KARAR: SEÇENEK A + 6.x Düzeltmeleri

1. v10 mimarisini koru (temiz, hızlı, memory-efficient)
2. Path sampling ekle (v9.1'den)
3. Goal validation ekle
4. Zaten yapılmış düzeltmeleri uygula
5. Test et ve sonuçlara göre daha fazla ekle

### Sonraki Adımlar:
1. `isSegmentSafeWithSampling` fonksiyonu ekle
2. `plan()` fonksiyonuna goal validation ekle
3. Build et ve test et
4. Log'lardaki waypoint sayısını kontrol et
5. Hala sorun varsa CostMap entegrasyonu değerlendir
