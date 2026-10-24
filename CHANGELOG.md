# Changelog

## Unreleased

- T-0040: toolpath core — `Move` (rapid/feed/plunge/retract/arcCW/arcCCW/dwell, 3D endpoint, arc centre, feed override, dwell seconds), `Vec3`, `Toolpath` with `ToolSnapshot`, bounds and length by kind, `PassPlan` (exact-step / equalised passes, last-pass thickness), contour linking with safe-Z rapids and plunge/retract, and a deterministic trapezoidal-acceleration time estimate, with Catch2 tests (ADR-0008).
- T-0001: geometry primitives — `Box2`, `Transform2` (2×3 affine: compose/invert/apply), `Span` variant of `LineSpan`/`ArcSpan`/`BezierSpan`, `Contour`, `Polyline`, `Tolerance`; length, bounds, point-at-parameter, reverse, transform and polygonisation at a chordal tolerance, with Catch2 tests (ADR-0005).
- Repository scaffold: CMake project, `carvetto_core` library with basic 2D primitives, Catch2 tests, CI, agent workflow and backlog.
