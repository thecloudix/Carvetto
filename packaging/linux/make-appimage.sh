#!/usr/bin/env bash
# T-0353: build Carvetto-<version>-x86_64.AppImage from a configured, built tree configured with
# -DCARVETTO_BUNDLE_QT=ON (so the install tree already carries Qt).
#   packaging/linux/make-appimage.sh <build-dir> <output-dir> [appimagetool]
# appimagetool is looked up on PATH unless given. APPIMAGE_EXTRACT_AND_RUN=1 lets an appimagetool that is
# itself an AppImage run without FUSE (CI containers).
set -euo pipefail
build=${1:?build dir}
out=${2:?output dir}
tool=${3:-appimagetool}
here=$(cd "$(dirname "$0")" && pwd)
version=$(sed -n 's/^CMAKE_PROJECT_VERSION:STATIC=//p' "$build/CMakeCache.txt")
[ -n "$version" ] || version=0
appdir=$(mktemp -d)/Carvetto.AppDir
trap 'rm -rf "$(dirname "$appdir")"' EXIT
DESTDIR=$appdir cmake --install "$build" --prefix /usr
install -m 755 "$here/AppRun" "$appdir/AppRun"
cp "$appdir/usr/share/applications/carvetto.desktop" "$appdir/carvetto.desktop"
# T-0363: the AppImage root icon is the PNG (appimagetool reads carvetto.png; .DirIcon is the file-manager thumbnail).
cp "$appdir/usr/share/icons/hicolor/256x256/apps/carvetto.png" "$appdir/carvetto.png"
cp "$appdir/usr/share/icons/hicolor/scalable/apps/carvetto.svg" "$appdir/carvetto.svg"
ln -s carvetto.png "$appdir/.DirIcon"
mkdir -p "$out"
ARCH=${ARCH:-x86_64} APPIMAGE_EXTRACT_AND_RUN=1 "$tool" "$appdir" "$out/Carvetto-$version-x86_64.AppImage"
