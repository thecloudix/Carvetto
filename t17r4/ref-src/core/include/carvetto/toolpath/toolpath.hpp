#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/toolpath/move.hpp"

namespace carvetto::toolpath {

/// The cutting parameters a toolpath needs from the tool it was computed with, copied at
/// calculation time so later edits to the tool database do not silently change the output.
/// T-0020 (tool database) provides the conversion from its `Tool` + `CuttingData`.
struct ToolSnapshot {
    std::string name;
    /// Tool number sent to the post-processor (1-based; 0 = unset).
    int toolNumber{0};
    /// Cutting diameter in mm (for V-bits the diameter at the top of the flutes).
    double diameter{0.0};
    /// Default feed rate for Feed and Arc moves, mm/min.
    double feedRate{0.0};
    /// Default rate for Plunge (and Retract) moves, mm/min.
    double plungeRate{0.0};
    /// Spindle speed in rpm (0 = not set).
    double spindleSpeed{0.0};

    bool operator==(const ToolSnapshot&) const = default;
};

/// Travel length broken down by move kind, in millimetres. Arcs count under `feed`.
struct LengthByKind {
    double rapid{0.0};
    double feed{0.0};
    double plunge{0.0};
    double retract{0.0};

    double total() const { return rapid + feed + plunge + retract; }
    bool operator==(const LengthByKind&) const = default;
};

/// A computed toolpath: an ordered move list in absolute machine coordinates (mm, Z up, tool
/// tip), plus the name and tool it was computed with. See ARCHITECTURE.md "Toolpaths are move
/// lists". The position before the first move is not stored; queries that need it take an
/// optional `from` and otherwise treat the first move as having zero length.
struct Toolpath {
    std::string name;
    ToolSnapshot tool;
    std::vector<Move> moves;

    bool operator==(const Toolpath&) const = default;

    bool isEmpty() const { return moves.empty(); }
    std::size_t moveCount() const { return moves.size(); }
    void append(Move m) { moves.push_back(m); }
    /// Appends all moves of `other` (its tool/name are ignored).
    void append(const Toolpath& other);

    /// End point of the last move; `std::nullopt` when empty.
    std::optional<Vec3> endPoint() const;
    /// Start point of move `i`: the end of move i-1, or `from` (or move 0's own end point when
    /// `from` is empty) for the first move.
    Vec3 startOf(std::size_t i, std::optional<Vec3> from = {}) const;

    /// Travel length per kind. The first move is measured from `from` when supplied.
    LengthByKind lengthByKind(std::optional<Vec3> from = {}) const;
    /// Sum of all move lengths.
    double length(std::optional<Vec3> from = {}) const { return lengthByKind(from).total(); }
    /// Exact XY extent swept by the moves (arcs included); empty for an empty toolpath.
    Box2 bounds(std::optional<Vec3> from = {}) const;
    /// Lowest end-point Z over all moves; `std::nullopt` when empty.
    std::optional<double> minZ() const;
    /// Highest end-point Z over all moves; `std::nullopt` when empty.
    std::optional<double> maxZ() const;
    /// Number of moves of the given kind.
    std::size_t count(MoveKind kind) const;
};

}  // namespace carvetto::toolpath
