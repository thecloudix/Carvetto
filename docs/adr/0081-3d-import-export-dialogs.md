# ADR-0081: 3D import and export dialogs

Status: accepted (T-0301, first part)

## Decisions
- Qt-free logic lives in `viewmodel/model_io.hpp`: `MeshImportSettings` (units, top face, quarter
  turns, resize with ratio lock and driving axis, centring, zero plane, cell size),
  `prepareMesh`, `previewMeshImport`, `importMeshIntoDocument`, `exportSheetModel`. The app adds
  `MeshImportDialog`, `ModelExportDialog` and the File menu actions (`main_window_model_io.cpp`).
- Order of operations on import: units, orientation, resize, centring (XY only). Z is settled by the
  zero plane, which is "the bottom of the model" by default or a typed model Z; heights below it
  are discarded by default (T-0286). Centring puts the model on the origin; it is not moved to the
  sheet's material.
- Undercut highlight: the dialog shows a top-down picture (grey by height) with cells that have
  material hidden below the top surface in red, plus their count. The preview rasterises at a cell
  size raised to at most 256 cells on the longer side; the import uses the requested cell size.
- Import adds one component on top of the sheet's topmost level (a level is created when there is
  none) as one undo step (macro) and selects it. Only the top side is imported; the both-sides
  option of the rasteriser is not exposed yet.
- Export: File > Export 3D Model as STL, OBJ or 16-bit heightmap (PNG or PGM) of the composed model
  of the active sheet at full resolution. `backZ` of T-0287 is an absolute Z: the dialog default is
  automatic, 0 when the whole model is above 0, otherwise one millimetre below the lowest cell;
  the user can type another value or choose the open surface. Heightmaps use the model's own
  range.
- Not in this task (follow-up card): the Two Rail Sweep, Extrude/Weave, Turn/Spin and Textured Area
  forms.
