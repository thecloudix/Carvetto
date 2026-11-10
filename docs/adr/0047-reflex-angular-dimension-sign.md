# ADR-0047: reflex angular dimensions are encoded in the sign of the offset

Status: accepted (T-0255)

## Context

T-0246 stores an angular dimension as vertex plus two arm points and an `offsetMm` (the radius of
the dimension arc, used as `|offsetMm|`). It always measures the smaller angle (0..pi). The
inventory's dimension tool also dimensions the larger (reflex) angle, chosen by where the arc is
placed. A stored flag would be a new `.carv` field and a new format minor.

## Decision

An angular dimension with a negative `offsetMm` measures the reflex angle (2*pi minus the smaller
angle); the radius is `|offsetMm|`. `offsetForPlacement` returns a negative offset when the placing
click lies outside the smaller angle; the handle and options edits keep the sign (the edit form
shows a "reflex" check box and the magnitude).

## Consequences

- No file-format change: the existing `offset_mm` field carries it, older readers draw the arc the
  short way (the sign was unused for angular dimensions before, so no existing file changes meaning).
- Scaling, moving and mirroring keep the sign (`offsetMm *= scale` for a positive scale).
- Other dimension kinds treat a negative offset as before (linear/aligned: the other side of the
  part; radial: clamped to a zero-length leader).
