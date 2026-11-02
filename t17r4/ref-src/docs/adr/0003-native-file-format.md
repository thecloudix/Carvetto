# ADR-0003: Native `.carv` file is a zip of JSON plus binary blobs

Date: 2026-10-24 · Status: accepted · Decided by: project owner (2026-10-24)

## Decision
A `.carv` file is a zip archive containing `manifest.json` (format version, app version), `job.json` (document tree: job, sheets, layers, objects with creation parameters, toolpath definitions, tool snapshots, document variables), and optional binary members for large data (`components/*.hf` zstd heightfields, `bitmaps/*`, `toolpaths/*.bin` cache). Templates (`.carvt`) are the same container without computed caches.

## Rationale
Human-diffable where practical, compact where it matters, and trivially extensible with new members. Mirrors how ODF/OOXML/KiCad-style tools solved the same problem.

## Alternatives
- Single JSON: too large with heightfields and bitmaps.
- SQLite: robust but opaque to diff tools and heavier for a desktop save.
