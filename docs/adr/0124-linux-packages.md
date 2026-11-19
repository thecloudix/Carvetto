# ADR-0124: Linux packages (DEB, RPM, AppImage)

Date: 2026-11-17 · Status: accepted (T-0353)

## Context
T-0353 asks for Linux packages with desktop integration, built in CI on Ubuntu 24.04. The app needs Qt 6.5+, but
Ubuntu 24.04 ships Qt 6.4, and CI builds against Qt 6.7 from the Qt installer, so the packages cannot rely on the
distribution's Qt.

## Decision
1. **Bundle Qt** in all Linux packages under `lib/carvetto/{lib,plugins}` (option `CARVETTO_BUNDLE_QT`, default OFF so a
   distribution build links the system Qt). Only Qt and its private ICU are bundled, found at install time with
   `file(GET_RUNTIME_DEPENDENCIES)`; the executable gets RUNPATH `$ORIGIN/../lib/carvetto/lib` and `bin/qt.conf` points Qt
   at the plugins. Qt plugins already search `$ORIGIN/../../lib`, which is exactly that directory, so nothing is needed for those; when `patchelf` is available the install step also pins the plugin RUNPATH (other Qt builds, e.g. conda, search elsewhere) and CI checks where the xcb plugin resolves Qt from.
2. **DEB and RPM through CPack**, `Depends`/`Requires` written by hand (dpkg-shlibdeps and rpm's auto-requires cannot
   resolve the bundled libraries): system GL/EGL, xkbcommon, xcb-util, fontconfig, freetype, dbus, glib and a few small libs.
3. **AppImage from the same install tree** (`packaging/linux/make-appimage.sh`: `DESTDIR` install + `AppRun` + appimagetool),
   not linuxdeploy: one bundling mechanism for every package, fewer downloaded tools.
4. **Desktop integration** is installed by every `cmake --install` on Linux: `carvetto.desktop`, MIME type
   `application/x-carvetto-project` (`*.carv`), and an original SVG icon (also the document icon). Only a scalable SVG is
   shipped; PNG sizes can be generated later. DEB relies on dpkg triggers for the caches, the RPM runs a post script.
5. Resources are found relative to the executable (`../share/carvetto`), so the same tree works under `/usr` and inside the
   AppImage; the headless test `packaging.linux_install` checks the install layout without Qt.
6. The CI job `linux-packages` runs on main, tags, on demand, PRs from `task/T-0353*` and PRs labelled `packaging`.
   The DEB is installed with apt (which proves the Depends names) and `carvetto --self-test` runs from `/usr`; the AppImage
   is run with `--appimage-extract-and-run`. The RPM is built and inspected (`rpm -qpl`) but not installed (Ubuntu runner).

## Consequences
Packages are large (Qt ~60 MB). No security updates for Qt from the distribution; rebuilds are needed. x86_64 only.
