#!/usr/bin/env bash
set -euo pipefail
EDITOR_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
export XOVIUM_BUILD_DIR=${XOVIUM_BUILD_DIR:-$EDITOR_ROOT/.build/cmake}
export XOVIUM_DEPS_DIR=${XOVIUM_DEPS_DIR:-$EDITOR_ROOT/.build/deps}
"$EDITOR_ROOT/build.sh" >&2
XOVIUM_BUILD_DIR=$(cd -- "$XOVIUM_BUILD_DIR" && pwd -P)
export XOVIUM_MESH_IMPORT="$XOVIUM_BUILD_DIR/mesh-import"
export PYTHONDONTWRITEBYTECODE=1
if [[ ${1:-} == --cli ]]; then
    shift
    exec python3 -B "$EDITOR_ROOT/backend/editor.py" "$@"
fi
exec "$XOVIUM_BUILD_DIR/xovium-model-editor" --root "$EDITOR_ROOT" "$@"
