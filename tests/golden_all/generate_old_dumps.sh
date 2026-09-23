#!/usr/bin/env bash
# Generates the OLD code's expansion dumps (ground truth for
# nas_expansion_differential) for every scene of environments.hpp.
#
# Prerequisites (old tree, conda env "rwa" active):
#   cmake --build build --target old_expansion_dump -j1
# Usage: tests/golden_all/generate_old_dumps.sh <out_dir> [extra env, e.g. NAS_SCRAMBLE=1]
#
# Every run is capped (ulimit -v, timeout): the old code allocates freely.
# Start positions are the ones tests/capture_golden_references.sh used.
set -uo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT="${1:?usage: $0 <out_dir>}"
mkdir -p "$OUT"
BIN="$REPO_ROOT/build/old_expansion_dump"
[ -x "$BIN" ] || { echo "build it first: cmake --build build --target old_expansion_dump -j1" >&2; exit 1; }

SCENES=(
  "NarrowPassage:0 0 0" "Stairs:0.1 0 0" "TwoFlatSurfaces:2.2 0.7 0" "LongStairs:0 0 0"
  "LongLongStairs:0 0 0" "Flat:0 0 0" "LongStairsComplete:0 0 0" "LongStairsExp:0 0 0"
  "ThreePathsScene:0 0 0" "Stairs_Up_Down:0 0 0" "ThreePathsNAS:0 0 0"
)
for entry in "${SCENES[@]}"; do
  name="${entry%%:*}"; start="${entry#*:}"
  ( ulimit -v 5000000; unset DISPLAY
    timeout 240 "$BIN" "$name" $start 150 "$OUT/$name.json" > "$OUT/$name.log" 2>&1 ) \
    && echo "ok   $name" || echo "FAIL $name (see $OUT/$name.log)"
done
