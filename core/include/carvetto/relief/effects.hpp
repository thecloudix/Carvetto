#pragma once

#include "carvetto/relief/relief.hpp"

// Level effects (T-0295, ADR-0079): clipping and mirror mode, applied to a composed level field.
// They never touch stored component fields; `compose` calls `applyEffects` per level.

namespace carvetto::relief {

/// Finite points, every polygon at least 3 vertices, within the clip caps, finite mirror centre.
/// BadParameter otherwise.
Status validateEffects(const LevelEffects& e);

/// Makes every cell whose centre is outside the clip transparent. Even-odd rule; a cell centre
/// exactly on a boundary is inside on a left or bottom edge and outside on a right or top edge.
/// No polygons: no change.
Status applyClip(const Clip& clip, HeightField& field);

/// Replaces cells on the non-source side of the axes through `center` with the bilinear sample of
/// their mirror image. Cells on an axis keep their value. A source sample that is transparent
/// makes the target transparent.
Status applyMirror(MirrorMode mode, Point center, HeightField& field);

/// Folds the field round the circumference (T-0321, ADR-0114). Cells whose circumference coordinate lies in
/// [start, start + circumference) take the highest opaque value of the field at that coordinate plus any whole
/// number of circumferences (so a relief that runs past the window edge wraps in on the other side, and
/// overlapping turns merge high); every other cell becomes transparent. Linear interpolation along the
/// circumference, exact on cell centres. No change when not enabled. BadParameter for a non-finite or
/// non-positive circumference, or when the field spans more than kMaxWrapPeriods turns.
Status applyWrapping(const Wrapping& wrapping, HeightField& field);

/// Mirror, wrapping, then clip. BadParameter when the effects are invalid.
Status applyEffects(const LevelEffects& e, HeightField& field);

/// A level's visible components composed on their union grid with effects applied (level base
/// height not included: it stays on the level), as one component named "<level> (baked)".
/// Mirrored content outside the union grid is cut off. Error when nothing is visible.
Status bakeLevel(const Level& level, Component& out);

}  // namespace carvetto::relief
