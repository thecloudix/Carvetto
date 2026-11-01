#!/usr/bin/env bash
# Installs Qt 6.7 (conda-forge) into a private prefix for building the Carvetto app in
# environments where download.qt.io is unreachable (e.g. the agents' cloud workspace).
# Usage: tools/setup_qt.sh <prefix-dir>
# Then:  cmake -S . -B build-qt -G Ninja -DCMAKE_PREFIX_PATH=<prefix-dir>/env
#        LD_LIBRARY_PATH=<prefix-dir>/env/lib QT_PLUGIN_PATH=<prefix-dir>/env/lib/qt6/plugins \
#          QT_QPA_PLATFORM=offscreen build-qt/app/carvetto --self-test
set -euo pipefail
PREFIX="${1:?usage: tools/setup_qt.sh <prefix-dir>}"
mkdir -p "$PREFIX"
PREFIX="$(cd "$PREFIX" && pwd)"
MM="$PREFIX/micromamba"
if [ ! -x "$MM" ]; then
  # Static micromamba binary from GitHub releases (api.anaconda.org may be blocked; GitHub is not).
  curl -fsSL -o "$MM" \
    https://github.com/mamba-org/micromamba-releases/releases/download/2.0.5-0/micromamba-linux-64
  chmod +x "$MM"
fi
export MAMBA_ROOT_PREFIX="$PREFIX/root"
# libgl/libegl/libopengl -devel are needed or Qt6Gui's CMake config fails to find OpenGL.
"$MM" create -y -q -p "$PREFIX/env" -c conda-forge \
  "qt6-main=6.7" libgl-devel libegl-devel libopengl-devel
echo "Qt installed. CMAKE_PREFIX_PATH=$PREFIX/env"
