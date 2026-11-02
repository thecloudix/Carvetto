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
/// Most bytes one move can take (an arc with a feed override).
inline constexpr std::size_t kMaxMoveBytes = 49;
/// Format version in the header.
inline constexpr std::uint32_t kMovesFormatVersion = 1;

/// Appends the cache encoding of `moves` to `out`. Returns an empty string on success, otherwise
/// what is wrong (a non-finite value or an invalid kind, naming the move); `out` is then
/// unspecified.
std::string encodeMoves(const std::vector<toolpath::Move>& moves, std::vector<std::uint8_t>& out);

/// Reads a whole cache from `source`: the header must declare `expectedCount` moves, every move
/// must be well formed with finite values, and nothing may follow the last move. Appends to
/// `out` (growing as data arrives, never reserving more than `expectedCount`). Returns an empty
/// string on success, otherwise the defect.
std::string decodeMoves(CharSource& source, std::uint64_t expectedCount, std::vector<toolpath::Move>& out);

}  // namespace carvetto::document::detail
