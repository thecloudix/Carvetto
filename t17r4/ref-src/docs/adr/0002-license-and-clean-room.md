# ADR-0002: GPL-3.0-or-later and clean-room development

Date: 2026-10-24 · Status: accepted · Decided by: project owner

## Decision
Carvetto is licensed GPL-3.0-or-later. Development is clean-room: functionality is reproduced from documented behaviour, never from proprietary code, assets, or file formats. Rules are in `CONTRIBUTING.md`.

## Consequences
- Third-party dependencies must be GPL-compatible (MIT, BSD, Apache-2.0, LGPL, Boost, zlib are fine).
- Proprietary file formats are not read or written; interchange is via open formats.
- Trademarked product names do not appear in UI, identifiers, or the project name.
