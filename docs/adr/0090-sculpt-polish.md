# ADR-0090: Sculpt polish (app)

Status: accepted (T-0309; completes the T-0302 follow-up list of ADR-0084 except the merged preview)

## Decisions
- **Custom brush from an image.** `viewmodel::customBrushFromImage` turns RGBA pixels into a
  `relief::CustomBrush`: height = Rec. 709 luminance (bright = high; "Invert image" flips it), a cell
  is transparent when fewer than half of its source pixels have alpha >= 128. Pictures over 256 per
  side are box-averaged down (the stamp is sampled bilinearly, so more resolution is wasted).
  `BrushSettings::custom` carries the stamp for every brush kind; "Rotate with stroke" sets
  `dynamicRotation`. A stamp is a session choice: it is not part of a preset and not persisted.
- **Overlay image.** A reference picture is laid on the design plane at the model's top height, as
  the largest rectangle with the picture's aspect ratio that fits the model, centred
  (`overlayPlacement`). The camera is orthographic, so the rectangle is a parallelogram on screen and
  is drawn with one affine `QTransform` from three projected corners (`projectModelPoint`, the
  inverse of the framing `pickModelPoint` uses). It is drawn over the surface, not occluded by relief
  (a deliberate simplification: it is a tracing aid). Opacity 0..1, a panel slider; a view setting,
  never stored in the document.
- **Brush ring cursor.** While sculpting, hovering the Model view draws a 48-segment ring of the
  brush diameter draped on the surface (`brushRingOutline`, each point at the surface height under
  it). `ModelView3D` tracks the mouse and asks `brushDiameter()`; unset outside Sculpt mode.
- **Persisted presets.** `BrushPresets::serialiseUser/loadUser`: a versioned text
  (`carvetto-brush-presets 1`, one tab-separated line per user preset, numbers in the C locale at
  full precision). Loading is tolerant: wrong header, malformed or out-of-range lines, built-in names,
  control characters in names, more than 256 user presets or text over 1 MiB are skipped, never an
  error. The panel only exposes `loadPresets` and `onPresetsChanged`; `MainWindow::setPersistLayout`
  wires them to `QSettings` key `sculpt/brushPresets`, so self-tests and headless runs never touch the
  user's settings.
- **Cheaper long strokes.** The controller still re-applies the open stroke's whole tail on every
  move (so the result ignores how often the mouse reported), but a tail longer than
  `kChunkDiameters` (30) brush diameters is frozen and a new chunk starts at the pointer. A move now
  costs at most ~150 dabs however long the stroke. A chunk join changes the dab phase there, so a
  long stroke equals the single-piece result only to within the spacing (deposit per cell within a
  few percent; tested), not bit for bit; strokes under 30 diameters are unchanged. The controller
  records how many session steps each stroke owns so "Undo Stroke" still reverts a whole stroke.

## Not done
- Sculpt preview merged with the other components stays unmerged (ADR-0084): composing the whole
  model on every mouse move is O(model cells) per move and would undo the cost saving above. A
  later task could compose the rest of the model once at Start and combine only the working
  component per move.
