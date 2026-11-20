# ADR-0131: Application icon

Status: accepted (T-0363)

The project owner chose round-3 design "E-c-cut v2" (a letter C lifting out of a plywood sheet).

- **Provenance and licence.** Original artwork made for Carvetto, no third-party or proprietary-product material;
  licensed under the project licence (GPL-3.0-or-later). Sources and generation notes: `packaging/icons/README.md`.
- **Committed renders.** CI has no SVG renderer, so the PNG sizes and the multi-size `.ico` are committed;
  `tools/make_ico.py` (stdlib only) builds the `.ico` from the PNGs with PNG-compressed entries and
  `packaging.app_icon` verifies sizes and freshness.
- **Small sizes.** At 16 px the master art loses the C inside the wood grain and shadow, so 16 and 24 px use a
  simplified variant of the same design (`carvetto-small.svg`: C and pocket only). The SVG hicolor icon stays the master.
- **Use.** Qt resource (`app/resources/icons.qrc`, PNGs only, so no Qt Svg dependency) for the window icon and the
  About logo; `packaging/windows/carvetto.rc` for the exe icon (WIN32 only); NSIS installer, uninstaller and `.carv`
  association icons from the installed `.ico`; Linux hicolor scalable SVG plus 16..256 px PNGs; the AppImage root
  icon is the 256 px PNG.
