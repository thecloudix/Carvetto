#pragma once

#include <cstdint>
#include <span>

namespace carvetto::viewmodel {

/// The font the Text tool starts with: `data/fonts/DejaVuSerif.ttf` (DejaVu Serif 2.37, Regular;
/// Bitstream Vera / DejaVu licence, see data/fonts/DEJAVU-LICENSE.txt and THIRD_PARTY_NOTICES.md),
/// compiled into the program so a fresh install can place text without any system font. The file is
/// never modified. The bytes are generated at build time from the data file.
std::span<const std::uint8_t> starterFontBytes();

/// Display name of the starter font.
const char* starterFontName();

}  // namespace carvetto::viewmodel
