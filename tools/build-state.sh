#!/usr/bin/env bash
# Content fingerprints for native builds. Checking a build needs only coreutils.
set -euo pipefail
export LC_ALL=C
usage() {
    printf 'Usage: build-state.sh fingerprint ROOT gui|backend\n       build-state.sh check|record ROOT BUILD gui|backend [EXPECTED_SOURCE_HASH]\n' >&2
    exit 2
}
file_hash() { local hash rest; read -r hash rest < <(sha256sum < "$1"); printf '%s\n' "$hash"; }
fingerprint() (
    cd -- "$1"
    {
        printf '%s\0' CMakeLists.txt build.sh run.sh tools/build-state.sh tools/bootstrap.py tools/mesh_import.cpp app/dependencies.json
        find backend schema -type d \( -name .git -o -name tests -o -name test -o -name __pycache__ \) -prune -o -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' -o -name '*.json' \) -print0
        if [[ $2 == gui ]]; then
            find app -type d \( -name .git -o -name tests -o -name test -o -name __pycache__ \) -prune -o -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' -o -name '*.json' \) -print0
        fi
    } | sort -zu | xargs -0 sha256sum --zero -- | sha256sum | cut -d ' ' -f 1
)
check_state() {
    local root=$1 build=$2 mode=$3 tag saved_mode source tool mesh editor extra
    local stamp=$build/.xovium-build-$mode
    [[ -f $stamp && -x $build/xovium-model-tool && -x $build/mesh-import ]] || return 1
    IFS=' ' read -r tag saved_mode source tool mesh editor extra < "$stamp" || return 1
    [[ $tag == xovium-build-v1 && $saved_mode == "$mode" && -z $extra ]] || return 1
    [[ $source =~ ^[0-9a-f]{64}$ && $tool =~ ^[0-9a-f]{64}$ && $mesh =~ ^[0-9a-f]{64}$ ]] || return 1
    [[ $source == "$(fingerprint "$root" "$mode")" ]] || return 1
    [[ $tool == "$(file_hash "$build/xovium-model-tool")" && $mesh == "$(file_hash "$build/mesh-import")" ]] || return 1
    if [[ $mode == gui ]]; then
        [[ -x $build/xovium-model-editor && $editor =~ ^[0-9a-f]{64}$ ]] || return 1
        [[ $editor == "$(file_hash "$build/xovium-model-editor")" ]] || return 1
    else
        [[ $editor == - ]] || return 1
    fi
}
write_stamp() {
    local build=$1 mode=$2 source=$3 tool=$4 mesh=$5 editor=$6 temporary
    temporary=$(mktemp "$build/.xovium-build-$mode.XXXXXX")
    printf 'xovium-build-v1 %s %s %s %s %s\n' "$mode" "$source" "$tool" "$mesh" "$editor" > "$temporary"
    mv -f -- "$temporary" "$build/.xovium-build-$mode"
}
record_state() {
    local root=$1 build=$2 mode=$3 expected=${4:-} backend_source selected_source tool mesh editor=-
    [[ -x $build/xovium-model-tool && -x $build/mesh-import ]] || { printf 'Native build is incomplete.\n' >&2; return 1; }
    backend_source=$(fingerprint "$root" backend)
    selected_source=$backend_source
    if [[ $mode == gui ]]; then
        [[ -x $build/xovium-model-editor ]] || { printf 'Desktop build is incomplete.\n' >&2; return 1; }
        selected_source=$(fingerprint "$root" gui)
        editor=$(file_hash "$build/xovium-model-editor")
    fi
    tool=$(file_hash "$build/xovium-model-tool")
    mesh=$(file_hash "$build/mesh-import")
    if [[ ( -n $expected && $expected != "$selected_source" ) || $selected_source != "$(fingerprint "$root" "$mode")" ]]; then
        printf 'Sources changed during the build; run build.sh again before launching.\n' >&2
        return 1
    fi
    write_stamp "$build" backend "$backend_source" "$tool" "$mesh" -
    if [[ $mode == gui ]]; then write_stamp "$build" gui "$selected_source" "$tool" "$mesh" "$editor"; fi
}
(( $# >= 3 )) || usage
operation=$1 root=$2
case "$operation" in
    fingerprint)
        (( $# == 3 )) || usage
        [[ $3 == gui || $3 == backend ]] || usage
        fingerprint "$root" "$3" ;;
    check|record)
        [[ $# == 4 || ( $# == 5 && $operation == record ) ]] || usage
        [[ $4 == gui || $4 == backend ]] || usage
        if [[ $operation == check ]]; then check_state "$root" "$3" "$4"; else record_state "$root" "$3" "$4" "${5:-}"; fi ;;
    *) usage ;;
esac
