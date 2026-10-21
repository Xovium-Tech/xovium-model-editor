#!/usr/bin/env bash
set -euo pipefail
EDITOR_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
if [[ ${1:-} == --help || ${1:-} == -h ]]; then
    cat <<'HELP'
Build Xovium Model Editor for Ubuntu 24.04 x86-64.
  ./build.sh
Environment:
  XOVIUM_BUILD_DIR   Build directory (default: .build/cmake)
  XOVIUM_DEPS_DIR    Pinned dependency cache (default: .build/deps)
  XOVIUM_BUILD_JOBS  Parallel compilation jobs (default: 4)
Prerequisites:
  build-essential cmake git python3 pkg-config libassimp-dev libpng-dev
  libx11-dev libxrandr-dev libxi-dev libxcursor-dev libxinerama-dev libgl1-mesa-dev
Missing packages are reported; this script does not run apt or sudo.
HELP
    exit 0
fi
if (( $# )); then printf 'Unknown build argument: %s\n' "$1" >&2; exit 2; fi
for tool in cmake git python3 c++ pkg-config; do
    command -v "$tool" >/dev/null || { printf 'Missing tool: %s. See ./build.sh --help for packages.\n' "$tool" >&2; exit 1; }
done
missing=()
for library in assimp libpng x11 xrandr xi xcursor xinerama gl; do
    pkg-config --exists "$library" || missing+=("$library")
done
if (( ${#missing[@]} )); then
    printf 'Missing development libraries: %s\nSee ./build.sh --help for Ubuntu package names.\n' "${missing[*]}" >&2
    exit 1
fi
export XOVIUM_BUILD_DIR=${XOVIUM_BUILD_DIR:-$EDITOR_ROOT/.build/cmake}
export XOVIUM_DEPS_DIR=${XOVIUM_DEPS_DIR:-$EDITOR_ROOT/.build/deps}
XOVIUM_BUILD_DIR=$(python3 -c 'import os,sys; print(os.path.abspath(sys.argv[1]))' "$XOVIUM_BUILD_DIR")
XOVIUM_DEPS_DIR=$(python3 -c 'import os,sys; print(os.path.abspath(sys.argv[1]))' "$XOVIUM_DEPS_DIR")
export XOVIUM_BUILD_DIR XOVIUM_DEPS_DIR
export PYTHONDONTWRITEBYTECODE=1
python3 -B "$EDITOR_ROOT/tools/bootstrap.py"
cmake -S "$EDITOR_ROOT" -B "$XOVIUM_BUILD_DIR" -DCMAKE_BUILD_TYPE=Release -DXOVIUM_DEPS_DIR="$XOVIUM_DEPS_DIR"
cmake --build "$XOVIUM_BUILD_DIR" --parallel "${XOVIUM_BUILD_JOBS:-4}"
printf 'Built editor: %s/xovium-model-editor\nMesh importer: %s/mesh-import\n' "$XOVIUM_BUILD_DIR" "$XOVIUM_BUILD_DIR"
