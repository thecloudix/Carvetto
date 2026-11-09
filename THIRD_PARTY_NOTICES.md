# Third-party notices

Carvetto is GPL-3.0-or-later. It uses the following third-party components, each under its own
GPL-compatible licence. Their full licence texts are fetched with the sources at build time
(see `core/CMakeLists.txt`) and must be shipped alongside any binary distribution.

| Component | Version | Licence | Used for |
|---|---|---|---|
| Qt | 6.x | LGPL-3.0 (dynamically linked) | Desktop UI (`app/`) |
| Clipper2 | 1.4.0 (one upstream fix backported, ADR-0006 §6; plus a local cancel-poll hook, T-0228) | Boost Software License 1.0 | Polygon offsets and booleans |
| nlohmann/json | 3.11.3 | MIT | JSON import/export |
| miniz | 3.0.2 | MIT | `.carv` zip container |
| FreeType | 2.14.3 | FreeType License (FTL; used under this option, which is GPLv3 compatible, ADR-0033) | Font parsing and glyph outlines (`core/text`) |
| HarfBuzz | 15.1.0 | "Old MIT" | Text shaping (`core/text`) |
| Catch2 | 3.7.1 | Boost Software License 1.0 | Unit tests only (not distributed) |
| DejaVuSerif.ttf (DejaVu Serif Regular) | 2.37 (`dejavu-fonts-ttf-2.37.tar.bz2`, SHA-256 `fa9ca4d1…17d7`) | Bitstream Vera Fonts licence + DejaVu public-domain changes (`data/fonts/DEJAVU-LICENSE.txt`); redistribution allowed, GPL-compatible as a separate font data file | Starter font of the Text tool and DXF text, compiled into the app |
| CarvettoTest-Regular.ttf | 1.0 | SIL Open Font License 1.1 (original Carvetto work, `core/tests/fixtures/text/OFL.txt`) | Text tests only (not distributed) |

**Required credit (FreeType License, clause 2):** "Portions of this software are copyright © The
FreeType Project (www.freetype.org). All rights reserved." Binary distributions must include this line
in their documentation, together with `docs/FTL.TXT` of the FreeType sources and HarfBuzz's `COPYING`.

**DejaVu Serif licence conditions:** the copyright and trademark notices and the permission notice
(`data/fonts/DEJAVU-LICENSE.txt`) must be included with every copy of the font, binary distributions
included. "Bitstream Vera" is a trademark of Bitstream, Inc. The font file is never modified; a
modified font would have to be renamed to a name containing neither "Bitstream" nor "Vera". The font
may be sold as part of a larger package, not by itself. The names of Gnome, the Gnome Foundation and
Bitstream must not be used to promote the software. The font is data embedded in `.carv` files the
user creates; those files are the user's documents.

When adding a dependency: pin it (tag or URL + SHA-256), confirm GPL compatibility, add a row here.
