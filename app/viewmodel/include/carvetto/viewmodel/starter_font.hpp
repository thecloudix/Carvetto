#pragma once

#include <cstdint>
#include <span>

namespace carvetto::viewmodel {

/// The font the Text tool starts with: `data/fonts/CarvettoStarter-Regular.ttf` (SIL OFL 1.1, an
/// original Carvetto font), compiled into the program so a fresh install can place text without any
/// system font. It is a minimal glyph set (see data/fonts/README); users pick their own TTF/OTF file
/// for real work. The bytes are generated at build time from the data file.
std::span<const std::uint8_t> starterFontBytes();

/// Display name of the starter font.
const char* starterFontName();

}  // namespace carvetto::viewmodel
