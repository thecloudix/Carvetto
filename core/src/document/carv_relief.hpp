#pragma once

// Internal: the height-field member `components/<id>.hf` of a `.carv` file (T-0282, ADR-0061).
// Tests include this header directly.
//
// Layout (integers and doubles little-endian, doubles as their IEEE-754 bits):
//   header   "CVHF" | u32 version (1) | u32 width | u32 height | f64 cell size | f64 origin x |
//            f64 origin y                                                   (40 bytes)
//   payload  width*height float32 bit patterns, byte-planar: all the lowest bytes of every cell
//            in row-major order, then all the second bytes, the third, the fourth (this groups
//            the slowly varying exponent bytes, which the archive's deflate then packs well).
// A transparent cell is the quiet NaN 0x7FC00000 (every NaN is written as that); infinities are
// refused. The member is exactly 40 + 4*width*height bytes. Equal fields give equal bytes.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "carv_zip.hpp"
#include "carvetto/relief/relief.hpp"

namespace carvetto::document::detail {

inline constexpr std::size_t kHeightFieldHeaderBytes = 40;
inline constexpr std::uint32_t kHeightFieldVersion = 1;

/// "components/<id>.hf".
std::string reliefFieldMemberName(std::uint64_t componentId);

/// Appends the encoding of `field` to `out`. Empty string on success, otherwise the defect
/// (empty field, infinite cell); `out` is then unspecified.
std::string encodeHeightField(const relief::HeightField& field, std::vector<std::uint8_t>& out);

/// Decodes a member of `declaredSize` bytes from `stream`. The header is validated and the
/// declared size must equal 40 + 4*cells exactly before any cell storage is allocated, and
/// storage never exceeds `declaredSize` (so never more than `relief::kMaxCells` floats).
/// Empty string on success, otherwise the defect.
std::string decodeHeightField(ZipArchive::Stream& stream, std::uint64_t declaredSize, relief::HeightField& out);

}  // namespace carvetto::document::detail
