#!/bin/sh
# Host tests for N2's connection-state / relay logic. Needs only gcc.
# Usage (from the repo root or anywhere):  sh tests/host/run_tests.sh [trials]
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)
OUT=${TMPDIR:-/tmp}/n2_host_tests
mkdir -p "$OUT"
CFLAGS="-std=c11 -O2 -Wall -Wextra -Werror -I$ROOT/src -I$HERE/stubs"
gcc $CFLAGS "$HERE/test_conn_state.c" "$ROOT/src/core/conn_state.c" -o "$OUT/test_conn_state"
gcc $CFLAGS "$HERE/sim_network.c"     "$ROOT/src/core/conn_state.c" -o "$OUT/sim_network"
"$OUT/test_conn_state"
"$OUT/sim_network" "${1:-2000}"
