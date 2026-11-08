#pragma once

// FreeType configuration used by Carvetto (T-0130, ADR-0033): FreeType's generated ftoption.h with
// two options switched off, selected with the FT_CONFIG_OPTIONS_H macro (core/CMakeLists.txt).
//  - FT_CONFIG_OPTION_ENVIRONMENT_PROPERTIES: otherwise FT_Init_FreeType reads the FREETYPE_PROPERTIES
//    environment variable and can change driver behaviour (hinting engine, ...), so the same font
//    bytes could give different outlines on different machines.
//  - FT_CONFIG_OPTION_USE_ZLIB: no gzip-compressed fonts and no WOFF; the bundled zlib is not used.
#include <freetype/config/ftoption.h>

#undef FT_CONFIG_OPTION_ENVIRONMENT_PROPERTIES
#undef FT_CONFIG_OPTION_USE_ZLIB
