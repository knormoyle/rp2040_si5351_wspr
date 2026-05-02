#!/usr/bin/env bash
#
# runTests.sh — Run the WSPR smoke test.
#
# Builds the test binary first if it doesn't exist or if any source/header
# is newer than the binary. Exits non-zero if any check fails.
#
# Usage:
#   ./runTests.sh
#   ./runTests.sh -v    # also print a summary line and exit code
#
# Environment overrides:
#   BUILD=release ./runTests.sh   # run against release build instead of debug

set -euo pipefail

# This script lives in test/, but the Makefile lives at the project root,
# one directory up. cd to the root so `make` and the binary path both work.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." &>/dev/null && pwd)"
cd "$PROJECT_ROOT"

VERBOSE=0
if [[ "${1:-}" == "-v" || "${1:-}" == "--verbose" ]]; then
    VERBOSE=1
fi

color_green() { printf '\033[1;32m%s\033[0m\n' "$*"; }
color_red()   { printf '\033[1;31m%s\033[0m\n' "$*"; }
color_blue()  { printf '\033[1;34m%s\033[0m\n' "$*"; }

# Build (or rebuild) the test binary. Make handles up-to-date checks via
# the auto-generated .d dependency files.
color_blue "==> Building"
make all >/dev/null

# Run the test. Forward -v / --verbose so the C++ side can show its
# detailed encode/decode output too (otherwise the script's own -v just
# prints a build-mode footer at the end).
color_blue "==> Running smoke test"
set +e
if [[ $VERBOSE -eq 1 ]]; then
    ./test/test_main -v
else
    ./test/test_main
fi
status=$?
set -e

if [[ $status -eq 0 ]]; then
    color_green "==> PASSED (exit $status)"
else
    color_red "==> FAILED (exit $status)"
fi

if [[ $VERBOSE -eq 1 ]]; then
    echo "Build mode: ${BUILD:-debug}"
    echo "Compiler:   ${CXX:-g++}"
fi

exit "$status"
