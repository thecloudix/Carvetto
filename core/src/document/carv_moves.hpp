#pragma once

// Internal: the binary move cache `toolpaths/NNNN.bin` of a `.carv` file (T-0132, ADR-0021).
// Tests include this header directly.
//
// Layout (all integers and doubles little-endian, doubles as their IEEE-754 bits):
//   header   "CVTP" | u32 version (1) | u64 move count
//   per move one tag byte, then the values it calls for:
//            bits 0-2  move kind (toolpath::MoveKind, 0..6)
//            bit 3     a feed override follows (cutting kinds only)
//            bit 4/5/6 x / y / z equal (bit for bit) to the previous move's end point, not
//                      written (the "previous" point of the first move is (0, 0, 0))
//            bit 7     the feed override equals the last one written, not written (with bit 3)
//            then x, y, z (unless equal), centre x and y (arcs), seconds (dwell), feed (bit 3
//            without bit 7): 1 to 49 bytes per move.
// Fields a move kind ignores (the centre of a line, the seconds of a non-dwell, a feed override
// on a rapid or dwell) are not stored and load as zero / empty. Every stored value must be
// finite. Equal input gives equal bytes.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "carvetto/toolpath/move.hpp"

namespace carvetto::document::detail {

class CharSource;

/// Bytes of the header.
inline constexpr std::size_t kMovesHeaderBytes = 16;
/// Most bytes one move can take (an arc with a feed override, plus in versions 2 and 3 the laser flag byte
/// and power).
inline constexpr std::size_t kMaxMoveBytes = 58;
/// Format version of a cache without laser or compensated moves (the only one written before T-0323).
inline constexpr std::uint32_t kMovesFormatVersion = 1;
/// Format version of a cache with at least one laser move and no compensated move (T-0323, ADR-0099): every
/// move is followed by one flag byte (bit 0 = a power follows, bit 1 = the power equals the last one written)
/// and the power (a double) unless it is repeated. `encodeMoves` writes version 1 when no move has power or
/// compensation, so existing files are byte-identical.
inline constexpr std::uint32_t kMovesFormatVersionLaser = 2;
/// Format version of a cache with at least one compensated move (T-0327, ADR-0100): exactly the version 2
/// layout (the per-move flag bytes and powers, which a cutter-compensated toolpath leaves at zero but a
/// toolpath that is both laser and compensated fills in), followed by the cutter-compensation runs: a u64
/// run count (at least 1), then per run a u64 first move (0-based), a u64 move count (at least 1) and a byte
/// for the side (1 = left, 2 = right). Runs are in order, do not overlap, and a run never follows another of
/// the same side without a gap (so every move list has one encoding). Written only when some move is
/// compensated; a version 2 cache never carries a trailer.
inline constexpr std::uint32_t kMovesFormatVersionCutterComp = 3;
/// Bytes after the moves of a version 3 cache: the run count, and at most one run per move.
inline constexpr std::size_t kMovesRunCountBytes = 8;
inline constexpr std::size_t kMovesRunBytes = 17;

/// Appends the cache encoding of `moves` to `out`. Returns an empty string on success, otherwise
/// what is wrong (a non-finite value or an invalid kind, naming the move); `out` is then
/// unspecified.
std::string encodeMoves(const std::vector<toolpath::Move>& moves, std::vector<std::uint8_t>& out);

/// Reads a whole cache from `source`: the header must declare `expectedCount` moves, every move
/// must be well formed with finite values, and nothing may follow the last move. Appends to
/// `out`, reserving exactly `expectedCount` more moves first (the caller bounds it by the move
/// budget). Returns an empty string on success, otherwise the defect.
std::string decodeMoves(CharSource& source, std::uint64_t expectedCount, std::vector<toolpath::Move>& out);

/// Checks a whole cache exactly as `decodeMoves` does (same defects, same messages) without
/// keeping the moves: memory use does not depend on the count. `saveCarv` verifies its output
/// with it (T-0136, ADR-0021 §3).
std::string validateMoves(CharSource& source, std::uint64_t expectedCount);

}  // namespace carvetto::document::detail
