#!/usr/bin/env bash
#
# build.sh — Build the WSPR smoke test and run it.
#
# This script lives in test/ and drives the Makefile one level up.
#
# Usage:
#   ./build.sh            # debug build (ASan + UBSan), then run smoke test
#   ./build.sh debug      # same as above
#   ./build.sh release    # release build (-O2), then run smoke test
#   ./build.sh clean      # clean build artifacts
#   ./build.sh both       # build debug AND release, run smoke test on debug
#
# Environment overrides:
#   CXX=clang++ ./build.sh
#   JOBS=8 ./build.sh

set -euo pipefail

# Move to the directory containing this script (test/) so all relative
# paths below are anchored there regardless of where the caller is.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
cd "$SCRIPT_DIR"

# The Makefile lives one level up, in the project root.
PROJECT_ROOT="$(cd .. && pwd)"

JOBS="${JOBS:-$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 2)}"
MODE="${1:-debug}"

# If any tracked file has a future mtime (e.g., because it was extracted
# from an archive on a system with a different clock, or copied from a VM),
# `make` warns "File 'X' has modification time N s in the future" and may
# do spurious rebuilds. Touch any future-dated files back to "now".
fix_future_mtimes() {
    local now
    now=$(date +%s)
    local fixed=0
    # Search the project root (headers, Makefile) and this test/ directory
    # (.cc, .sh). -maxdepth 1 in each pass avoids descending further.
    while IFS= read -r -d '' file; do
        touch "$file"
        fixed=1
    done < <(find "$PROJECT_ROOT" "$SCRIPT_DIR" \
                    \( -name '*.h' -o -name '*.cc' \
                       -o -name 'Makefile' -o -name '*.sh' \) \
                    -maxdepth 1 -newermt "@$now" -print0 2>/dev/null)
    if [[ $fixed -eq 1 ]]; then
        echo "(reset future-dated file mtimes to now)"
    fi
}
fix_future_mtimes

color_green() { printf '\033[1;32m%s\033[0m\n' "$*"; }
color_red()   { printf '\033[1;31m%s\033[0m\n' "$*"; }
color_blue()  { printf '\033[1;34m%s\033[0m\n' "$*"; }

build_one() {
    local build_mode="$1"
    color_blue "==> Building ($build_mode)"
    make -C "$PROJECT_ROOT" -j"$JOBS" BUILD="$build_mode" all
}

run_smoke_test() {
    color_blue "==> Running smoke test"
    if ./test_main; then
        color_green "==> Smoke test PASSED"
    else
        color_red "==> Smoke test FAILED"
        exit 1
    fi
}

case "$MODE" in
    debug|release)
        build_one "$MODE"
        run_smoke_test
        ;;
    both)
        build_one debug
        run_smoke_test
        # Save the debug binary, do a release build to verify it compiles
        # cleanly under -O2 too. The Makefile produces test/test_main; we
        # rename siblings of it to keep both flavours around.
        mv test_main test_main.debug
        make -C "$PROJECT_ROOT" clean >/dev/null
        build_one release
        mv test_main test_main.release
        # Restore the debug binary as the default for subsequent runs
        cp test_main.debug test_main
        color_green "==> Both builds succeeded"
        color_blue "    test/test_main.debug   (ASan + UBSan)"
        color_blue "    test/test_main.release (-O2)"
        color_blue "    test/test_main         (= debug copy, default)"
        ;;
    clean)
        color_blue "==> Cleaning"
        make -C "$PROJECT_ROOT" clean
        rm -f test_main.debug test_main.release
        color_green "==> Clean complete"
        ;;
    -h|--help|help)
        sed -n '2,/^$/p' "$0" | sed 's/^# \{0,1\}//'
        ;;
    *)
        color_red "Unknown mode: $MODE"
        echo "Usage: $0 [debug|release|both|clean|help]" >&2
        exit 2
        ;;
esac
