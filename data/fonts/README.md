# Bundled fonts (T-0211, T-0233)

`DejaVuSerif.ttf` is the font the Text tool starts with and the font used for DXF text import. It is
**DejaVu Serif 2.37 (Regular)**, taken unmodified from the official release tarball
`dejavu-fonts-ttf-2.37.tar.bz2`
(<https://github.com/dejavu-fonts/dejavu-fonts/releases/tag/version_2_37>,
tarball SHA-256 `fa9ca4d13871dd122f61258a80d01751d603b4d3ee14095d65453b4e846e17d7`,
`ttf/DejaVuSerif.ttf` SHA-256 `42d1edeb7952f31b1f96d767ed7030b08a39e0c372b0071641518864e2bffb51`,
380132 bytes). `DEJAVU-LICENSE.txt` is the upstream `LICENSE` file of that tarball.

**Never modify this file.** The Bitstream Vera / DejaVu licence allows redistribution, but a modified
font must be renamed to a name containing neither "Bitstream" nor "Vera"; we ship it as-is under its
own name. The font may not be sold by itself (we ship it inside a larger program).

The font is compiled into the program (`app/viewmodel`, generated source), not read from disk. Test
fixtures keep using the tiny original `core/tests/fixtures/text/CarvettoTest-Regular.ttf`.
