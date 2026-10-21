#!/usr/bin/env bash
set -euo pipefail
EDITOR_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
EDITOR_BUILD_GUI=ON
if [[ ${1:-} == --backend-only ]]; then EDITOR_BUILD_GUI=OFF; shift; fi
if [[ ${1:-} == --help || ${1:-} == -h ]]; then
    cat <<'HELP'
Build Xovium Model Editor for Ubuntu 24.04 x86-64.
  ./build.sh                 Desktop editor and native command line tools
  ./build.sh --backend-only  Native command line tools without display libraries
Environment:
  XOVIUM_BUILD_DIR   Build directory (default: .build/cmake or .build/backend)
  XOVIUM_DEPS_DIR    Pinned dependency cache (default: .build/deps)
  XOVIUM_BUILD_JOBS  Parallel compilation jobs (default: 4)
Common prerequisites:
  build-essential cmake git python3 pkg-config libassimp-dev libpng-dev
  libtinyxml2-dev libarchive-dev
Desktop prerequisites:
  libx11-dev libxrandr-dev libxi-dev libxcursor-dev libxinerama-dev libgl1-mesa-dev
Python is used only for dependency bootstrap, not the editor/runtime backend.
Missing packages are reported; this script does not run apt or sudo.
HELP
    exit 0
fi
if (( $# )); then printf 'Unknown build argument: %s\n' "$1" >&2; exit 2; fi
for tool in cmake git python3 c++ pkg-config; do
    command -v "$tool" >/dev/null || { printf 'Missing tool: %s. See ./build.sh --help for packages.\n' "$tool" >&2; exit 1; }
done
missing=()
libraries=(assimp libpng tinyxml2 libarchive)
if [[ $EDITOR_BUILD_GUI == ON ]]; then libraries+=(x11 xrandr xi xcursor xinerama gl); fi
for library in "${libraries[@]}"; do
    pkg-config --exists "$library" || missing+=("$library")
done
if (( ${#missing[@]} )); then
    printf 'Missing development libraries: %s\nSee ./build.sh --help for Ubuntu package names.\n' "${missing[*]}" >&2
    exit 1
fi
EDITOR_DEFAULT_BUILD=$EDITOR_ROOT/.build/cmake
bootstrap_args=()
if [[ $EDITOR_BUILD_GUI == OFF ]]; then
    EDITOR_DEFAULT_BUILD=$EDITOR_ROOT/.build/backend
    bootstrap_args+=(--backend-only)
fi
export XOVIUM_BUILD_DIR=${XOVIUM_BUILD_DIR:-$EDITOR_DEFAULT_BUILD}
export XOVIUM_DEPS_DIR=${XOVIUM_DEPS_DIR:-$EDITOR_ROOT/.build/deps}
XOVIUM_BUILD_DIR=$(python3 -c 'import os,sys; print(os.path.abspath(sys.argv[1]))' "$XOVIUM_BUILD_DIR")
XOVIUM_DEPS_DIR=$(python3 -c 'import os,sys; print(os.path.abspath(sys.argv[1]))' "$XOVIUM_DEPS_DIR")
export XOVIUM_BUILD_DIR XOVIUM_DEPS_DIR
export PYTHONDONTWRITEBYTECODE=1
EDITOR_BUILD_MODE=gui
if [[ $EDITOR_BUILD_GUI == OFF ]]; then EDITOR_BUILD_MODE=backend; fi
EDITOR_BUILD_SOURCE=$(bash "$EDITOR_ROOT/tools/build-state.sh" fingerprint "$EDITOR_ROOT" "$EDITOR_BUILD_MODE")
python3 -B "$EDITOR_ROOT/tools/bootstrap.py" "${bootstrap_args[@]}"
cmake -S "$EDITOR_ROOT" -B "$XOVIUM_BUILD_DIR" -DCMAKE_BUILD_TYPE=Release \
    -DXOVIUM_DEPS_DIR="$XOVIUM_DEPS_DIR" -DXOVIUM_BUILD_GUI="$EDITOR_BUILD_GUI"
cmake --build "$XOVIUM_BUILD_DIR" --parallel "${XOVIUM_BUILD_JOBS:-4}"
bash "$EDITOR_ROOT/tools/build-state.sh" record "$EDITOR_ROOT" "$XOVIUM_BUILD_DIR" "$EDITOR_BUILD_MODE" "$EDITOR_BUILD_SOURCE"
printf 'Model tool: %s/xovium-model-tool\nMesh importer: %s/mesh-import\n' "$XOVIUM_BUILD_DIR" "$XOVIUM_BUILD_DIR"
if [[ $EDITOR_BUILD_GUI == ON ]]; then printf 'Built editor: %s/xovium-model-editor\n' "$XOVIUM_BUILD_DIR"; fi
