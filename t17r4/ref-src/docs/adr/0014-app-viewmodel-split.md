# ADR-0014: Qt-free view-model library under the app

Date: 2026-10-24 · Status: accepted (director, 2026-10-24)

## Context
T-0060 adds the Qt desktop shell. Qt is not available in the agents' workspace, so anything
written only against Qt can be checked only by CI, and only by a smoke test. The card requires
the non-trivial view logic (view transform, rulers, contour polygonisation for drawing, cursor
formatting, New Job validation) to be unit-testable headless everywhere.

## Decision
1. **`carvetto_viewmodel`** is a static library in `app/viewmodel/` (namespace
   `carvetto::viewmodel`, headers under `include/carvetto/viewmodel/`). It depends on
   `carvetto::core` only and never on Qt. It is configured by the top-level `CMakeLists.txt`
   whenever `CARVETTO_BUILD_VIEWMODEL` (default ON) or `CARVETTO_BUILD_APP` is on, independent
   of whether Qt is found, and its Catch2 tests (`app/viewmodel/tests/`, reusing core's Catch2)
   run under `ctest` with `CARVETTO_BUILD_TESTS`.
2. **The Qt layer (`app/src/`) only paints and forwards events.** Each frame the canvas asks
   `buildCanvasScene(document, view)` for screen-space material rectangle, datum and
   polygonised paths (curves polygonised at 0.25 px), and strokes them; rulers draw what
   `rulerTicks` returns; the New Job dialog hands its fields to `validateNewJob`.
3. **View transform:** uniform scale (px/mm, clamped to 0.01…1000) plus translation, world Y up
   and screen Y down. Zoom-to-fit margin is a fraction of the viewport *per side* (5% → the
   material spans 90% of the limiting axis).
4. **Rulers:** the major step is the smallest 1/2/5×10ⁿ step in the display unit that is at
   least 50 px on screen (so ≤ 125 px); the step is kept as integers (mantissa, exponent) and
   labels are built by decimal string scaling, so they are exact. Minor divisions: 10 / 4 / 5.
5. **No `Q_OBJECT`.** Widgets expose `std::function` callbacks and connect Qt signals to
   lambdas, so no moc output is compiled under our warnings-as-errors flags.
6. **Self-test** (`carvetto --self-test`): runs in-process against the real `MainWindow`, renders
   via `CanvasWidget::renderFitted` (QPainter on a `QImage`, same paint routine as the widget),
   checks pixels with the Qt-free `countPixelsDiffering`, and round-trips the dock layout through
   `QSettings(path, QSettings::IniFormat)` in a `QTemporaryDir`. It defaults `QT_QPA_PLATFORM`
   to `offscreen` when unset and never touches the user's settings.
7. **Windows subsystem:** the executable is a GUI-subsystem program (`WIN32_EXECUTABLE ON`),
   so users get no console window. CI's PowerShell self-test step launches it with
   `Start-Process -Wait -PassThru` and redirected stdout/stderr, which waits for GUI-subsystem
   programs and captures the exit code and the one-line failure reason. (Amended by the director
   at merge; the coder's console-subsystem workaround predates that CI change.)

## Consequences
- View behaviour is regression-tested on every platform, including where Qt is missing; a
  `viewmodel.qt_free` ctest fails if `core/` or `app/viewmodel/` include a Qt header or if
  `carvetto_core`/`carvetto_viewmodel` link a Qt target.
- Later UI cards (drawing tools, selection, snapping) put their maths in `carvetto_viewmodel`
  first and keep `app/src` thin.
- On Windows, launching `carvetto.exe` from Explorer shows a console window until item 7 is
  revisited.
