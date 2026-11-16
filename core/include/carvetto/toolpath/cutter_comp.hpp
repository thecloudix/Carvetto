#pragma once

#include <string>
#include <vector>

#include "carvetto/geometry/vec3.hpp"
#include "carvetto/toolpath/move.hpp"

// Controller-side cutter compensation (G41/G42 style; T-0327, ADR-0100). A compensated toolpath
// keeps the tool-centre path in its moves (so simulation, preview and every other consumer are
// unchanged) and flags the moves that run under compensation (`Move::comp`). The post writes the
// programmed path, which is the tool-centre path moved back by the tool radius towards the part;
// `cutterCompProgram` derives it.

namespace carvetto::toolpath {

/// One move as the post writes it.
struct ProgramMove {
    /// True when the programmed path has no move here (a round corner of the tool-centre path
    /// that is a single point of the programmed path): write nothing.
    bool omit{false};
    /// The move to write: the input move with the programmed end point (and, for arcs, the same
    /// centre); moves outside compensated runs are returned unchanged.
    Move move;
};

/// The programmed path of a toolpath's move list.
struct CutterCompProgram {
    /// One entry per input move. Empty when `error` is set.
    std::vector<ProgramMove> moves;
    /// Empty when the programmed path was derived; otherwise why a compensated run cannot be
    /// written (the message names the 1-based move).
    std::string error;
    bool ok() const { return error.empty(); }
};

/// Derives the programmed path of `moves`, a toolpath that starts at `from` (the position before
/// its first move; only used when a run starts at move 0), for a tool of the given `radius` (mm).
///
/// A run is a maximal stretch of moves with the same `comp` other than `Off`. The tool is on the
/// `Left` or `Right` of the programmed path looking along the travel. Each line moves back by the
/// radius along its normal, each arc keeps its centre and changes its radius by the tool radius;
/// where consecutive moves then no longer meet (an inside corner of the tool-centre path) they are
/// joined at the intersection of the two moved moves. The engaging move starts at the tool
/// position and ends where the compensation vector is perpendicular to the next move; the move
/// after a run (the releasing move) starts from the compensated end of the last one. Every rule
/// is the controller's start-up/release behaviour of ADR-0100; the tool-centre path the
/// controller then produces is the one in `moves`.
///
/// Errors (all reported, nothing guessed): a run that is not engaged by a straight move in the
/// plane, not released by a straight move in the plane, a move in a run that is a rapid or a
/// dwell, a switch from left to right without a release, an arc that would have no radius left
/// (smaller than the tool radius), corners whose moved moves do not meet or meet farther than
/// ten radii away, and a non-finite or non-positive `radius`.
CutterCompProgram cutterCompProgram(const std::vector<Move>& moves, double radius, geometry::Vec3 from);

/// True when some move carries a compensation flag.
bool usesCutterComp(const std::vector<Move>& moves);

}  // namespace carvetto::toolpath
