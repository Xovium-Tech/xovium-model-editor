#!/usr/bin/env bash
set -euo pipefail
EDITOR_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
EDITOR_CLI=false
EDITOR_BUILD_MODE=gui
EDITOR_BUILD_ARGS=()
EDITOR_DEFAULT_BUILD=$EDITOR_ROOT/.build/cmake
if [[ ${1:-} == --cli ]]; then
    shift
    EDITOR_CLI=true
    EDITOR_BUILD_MODE=backend
    EDITOR_BUILD_ARGS+=(--backend-only)
    EDITOR_DEFAULT_BUILD=$EDITOR_ROOT/.build/backend
    # A desktop build already contains both native command line tools.
    if [[ -z ${XOVIUM_BUILD_DIR:-} ]] && bash "$EDITOR_ROOT/tools/build-state.sh" check "$EDITOR_ROOT" "$EDITOR_ROOT/.build/cmake" backend; then
        EDITOR_DEFAULT_BUILD=$EDITOR_ROOT/.build/cmake
    fi
fi
export XOVIUM_BUILD_DIR=${XOVIUM_BUILD_DIR:-$EDITOR_DEFAULT_BUILD}
export XOVIUM_DEPS_DIR=${XOVIUM_DEPS_DIR:-$EDITOR_ROOT/.build/deps}
XOVIUM_BUILD_DIR=$(realpath -m -- "$XOVIUM_BUILD_DIR")
if ! bash "$EDITOR_ROOT/tools/build-state.sh" check "$EDITOR_ROOT" "$XOVIUM_BUILD_DIR" "$EDITOR_BUILD_MODE"; then
    "$EDITOR_ROOT/build.sh" "${EDITOR_BUILD_ARGS[@]}" >&2
fi
export XOVIUM_MESH_IMPORT="$XOVIUM_BUILD_DIR/mesh-import"
export XOVIUM_MODEL_TOOL="$XOVIUM_BUILD_DIR/xovium-model-tool"
if $EDITOR_CLI; then
    exec "$XOVIUM_MODEL_TOOL" --root "$EDITOR_ROOT" "$@"
fi
exec "$XOVIUM_BUILD_DIR/xovium-model-editor" --root "$EDITOR_ROOT" "$@"
