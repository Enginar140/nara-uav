# Path Planning Dubins/Corridor Plan

## Karar

Mevcut v10 visibility-graph mimarisi korunuyor. Bunun ustune fixed-wing icin
donus yaricapi bilen bir corridor katmani eklendi:

1. Hard safety: HSS radius + `hss_buffer_m`. Bu alan asla ihlal edilmez.
2. Soft corridor: hard safety + `turn_safety_margin_m`. Planner normalde bu
   koridorun disindan rota uretmeye calisir.
3. Turn smoothing: keskin visibility-graph koseleri `min_turn_radius_m` ile
   arc waypoint'lerine cevrilir.
4. Densify: uzun bacaklar `max_segment_length_m` ile ara waypoint'lere bolunur.
5. Final validation: uretilen rota hard safety ile tekrar dogrulanir.

Bu, tam Dubins solver degil; cunku mevcut sistem start/end heading'i rota
kontratinda tasimiyor. Su anki degisiklik fixed-wing uyumlu "Dubins-style"
koridor ve arc smoothing katmanidir. Gercek Dubins ya da Hybrid A* icin rota
state'i `(lat, lon, heading)` seviyesine cikarilmalidir.

## Neden Bu Yol

- Mevcut kodun temiz visibility-graph yapisini bozmadan HSS etrafinda daha
  genis ve ucabilir koridor uretir.
- Costmap/A*'a gore daha az bellek kullanir ve kucuk HSS listelerinde hizlidir.
- HSS icine girmeme garantisi SafetyGate ve PathPlanner hard validation ile
  iki yerde korunur.
- ArduPilot GUIDED/DO_REPOSITION akisi degismeden kalir; yalnizca direct
  komut HSS koridoru cok darsa route planning'e duser.

## Akis

```text
start/goal
  -> direct segment soft corridor clear ise direkt rota
  -> HSS circle'larini soft corridor ile polygon'a genislet
  -> visibility graph kur
  -> hard HSS crossing yapan graph edge'lerini ele
  -> Dijkstra shortest path
  -> line-of-sight simplify
  -> turn-radius arc smoothing
  -> waypoint densify
  -> hard HSS/boundary final validation
```

## Konfig

```yaml
nav:
  planner:
    hss_buffer_m: 40.0
    boundary_buffer_m: 20.0
    min_turn_radius_m: 50.0
    turn_safety_margin_m: 50.0
    max_arc_step_deg: 10.0
    max_segment_length_m: 80.0
    dubins_smoothing_enabled: true
    polygon_segments: 12
```

ArduPilot parametreleri geldikten sonra `min_turn_radius_m` tahmini yaricapla
guncellenir. `turn_safety_margin_m`, bu yaricaptan kucuk kalmayacak sekilde
korunur.

## Sonraki Esik

Bu mimari yeterli olmazsa bir sonraki mantikli adim:

1. `PathPlanner` kontratina heading eklemek.
2. HSS/boundary uzerinde Hybrid A* veya Dubins state lattice kullanmak.
3. Visibility graph'i yalnizca global koridor ureteci olarak tutup, son
   segmentleri Dubins primitive'leriyle doldurmak.

Bu daha guclu olur ama butun mission/navigation kontratini etkiler. Mevcut
branch once en dusuk riskli, en uyumlu iyilestirmeyi uygular.
