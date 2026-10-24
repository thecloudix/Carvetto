# Changelog

## Unreleased

- T-0001: geometry primitives — `Box2`, `Transform2` (2×3 affine: compose/invert/apply), `Span` variant of `LineSpan`/`ArcSpan`/`BezierSpan`, `Contour`, `Polyline`, `Tolerance`; length, bounds, point-at-parameter, reverse, transform and polygonisation at a chordal tolerance, with Catch2 tests (ADR-0005).
- Repository scaffold: CMake project, `carvetto_core` library with basic 2D primitives, Catch2 tests, CI, agent workflow and backlog.
