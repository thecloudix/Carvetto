# Application icon

Own artwork ("a letter C lifting out of a plywood sheet", round 3 "E-c-cut" variant v2), created for this project and
licensed like the rest of the repository (GPL-3.0-or-later). It contains no third-party artwork. See ADR-0131.

| File | What |
|---|---|
| `carvetto.svg` | Master artwork, 256 x 256 viewBox. Also the Linux scalable and MIME-type icon. |
| `carvetto-small.svg` | Simplified variant (C and pocket only: no wood grain or shadow, bolder strokes, art enlarged) for 16 and 24 px, where the master turns muddy. |
| `carvetto-{32,48,64,128,256}.png` | Renders of `carvetto.svg`. |
| `carvetto-small-{16,24}.png` | Renders of `carvetto-small.svg`. |
| `carvetto.ico` | Windows icon, built from the PNGs above by `tools/make_ico.py` (PNG-compressed entries). |

CI has no SVG renderer, so the PNGs and the .ico are committed; `packaging.app_icon` checks that they are complete,
correctly sized and that the .ico is not stale. After editing an SVG, regenerate the renders with Qt's `QSvgRenderer`
(Qt6 Gui + Svg), for example:

```cpp
QSvgRenderer r(QStringLiteral("carvetto.svg"));
QImage im(size, size, QImage::Format_ARGB32);
im.fill(Qt::transparent);
QPainter p(&im);
p.setRenderHint(QPainter::Antialiasing);
r.render(&p, QRectF(0, 0, size, size));
p.end();
im.save(QStringLiteral("carvetto-%1.png").arg(size));
```

then rebuild the icon: `python3 tools/make_ico.py packaging/icons/carvetto.ico packaging/icons/carvetto-small-16.png
packaging/icons/carvetto-small-24.png packaging/icons/carvetto-{32,48,64,128,256}.png`.
