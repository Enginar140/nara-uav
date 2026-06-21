# Project State

## Project Reference

See: .planning/PROJECT.md (updated 2026-06-21)

**Core value:** Kilitlenme ve kamikaze görevlerinin şartname-geçerli veri akışıyla puanlanması — doğru formatta + sunucu saatiyle damgalı paket gönderilmezse puan yok.
**Current focus:** Phase 1 — Haberleşme Kontratı

## Current Position

Phase: 1 of 5 (Haberleşme Kontratı)
Plan: 0 of TBD in current phase
Status: Ready to plan
Last activity: 2026-06-21 — Roadmap oluşturuldu, Faz 1 planlamaya hazır

Progress: [░░░░░░░░░░] 0%

## Performance Metrics

**Velocity:**
- Total plans completed: 0
- Average duration: -
- Total execution time: 0 hours

**By Phase:**

| Phase | Plans | Total | Avg/Plan |
|-------|-------|-------|----------|
| - | - | - | - |

**Recent Trend:**
- Last 5 plans: —
- Trend: —

*Updated after each plan completion*

## Accumulated Context

### Decisions

Decisions are logged in PROJECT.md Key Decisions table.
Recent decisions affecting current work:

- [Roadmap]: Veri yolu önce: kontrat sabitleme → downlink üretimi → GCS parse → mod mantığı → E2E doğrulama sırası benimsendi
- [Roadmap]: GCS (Phase 3) ayrı repo (mp v2 / C#) — izleme fazı olarak tutuldu, companion tarafı yalnız kontrat uyumunu korur
- [Roadmap]: Phase 3 ve Phase 4 bağımlılığı Phase 2'dir (ikisi paralel yürütülebilir), Phase 5 her ikisine bağlı

### Pending Todos

None yet.

### Blockers/Concerns

- Phase 3 çapraz-sistem: GCS repo'suna erişim ve C# değişiklikleri companion timeline'ını etkileyebilir — GCS değişiklikleri erken başlatılmalı
- VALID-04 (gerçek FBWA teyidi) gerçek donanım gerektirir; SITL'de kısmen doğrulanabilir

## Deferred Items

| Category | Item | Status | Deferred At |
|----------|------|--------|-------------|
| v2 | AUTO-01: Otonom hedef seçimi | v2 backlog | Roadmap |
| v2 | AUTO-02: Konum tahmini / lead | v2 backlog | Roadmap |

## Session Continuity

Last session: 2026-06-21
Stopped at: Roadmap ve State dosyaları oluşturuldu; REQUIREMENTS.md traceability güncellendi
Resume file: None
