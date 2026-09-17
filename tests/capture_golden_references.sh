#!/usr/bin/env bash
# tests/capture_golden_references.sh — Stage A phase 0e (see PLAN.md).
#
# Cycles through every scenario in include/environments.hpp, editing the
# active surf_list / current_foot_pos / goal_offset in constants.hpp,
# rebuilding golden_capture, and running it. A scenario that fails to
# build, times out, or finds no path is logged in capture_summary.txt and
# skipped — not debugged, per PLAN.md phase 0's "keep whatever works"
# scope. constants.hpp is restored to its original content on exit either
# way, so this script is safe to re-run.
#
# Usage: source the rwa conda env, then run from anywhere:
#   tests/capture_golden_references.sh
#
# Starting positions/goal offsets below come from README.md / the comments
# already in constants.hpp where documented; scenarios without a documented
# starting point use the (0,0,0) default and may simply fail to find a
# path, which is fine.

set -uo pipefail  # no -e: one scenario failing must not abort the loop

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

CONSTANTS=include/constants.hpp
BACKUP=$(mktemp)
cp "$CONSTANTS" "$BACKUP"
restore_constants() { cp "$BACKUP" "$CONSTANTS"; rm -f "$BACKUP"; }
trap restore_constants EXIT

if [ -z "${CONDA_PREFIX:-}" ]; then
  echo "CONDA_PREFIX not set — activate the 'rwa' conda env before running this script." >&2
  exit 1
fi

GIT_SHA=$(git rev-parse HEAD)
OUT_DIR="$REPO_ROOT/tests/golden"
mkdir -p "$OUT_DIR"
SUMMARY="$OUT_DIR/capture_summary.txt"
{
  echo "Golden capture run — $(date -u +%Y-%m-%dT%H:%M:%SZ), git_sha=$GIT_SHA"
  echo ""
} > "$SUMMARY"

# name:current_foot_pos:goal_offset
SCENARIOS=(
  "NarrowPassage:0.0, 0.0, 0.0:0.0, 0.0, 0.0"
  "Stairs:0.1, 0.0, 0.0:0.0, 0.0, 0.0"
  "TwoFlatSurfaces:2.2, 0.7, 0.0:0.0, 0.0, 0.0"
  "LongStairs:0.0, 0.0, 0.0:0.0, 0.0, 0.0"
  "LongLongStairs:0.0, 0.0, 0.0:0.0, 0.0, 0.0"
  "Flat:0.0, 0.0, 0.0:0.0, 0.0, 0.0"
  "LongStairsComplete:0.0, 0.0, 0.0:0.0, 0.0, 0.0"
  "LongStairsExp:0.0, 0.0, 0.0:0.0, 0.0, 0.0"
  "ThreePathsScene:0.0, 0.0, 0.0:0.0, 0.0, 0.0"
  "Stairs_Up_Down:0.0, 0.0, 0.0:0.0, 0.0, 0.0"
  "ThreePathsNAS:0.0, 0.0, 0.0:0.0, 1.0, 0.0"
)

for entry in "${SCENARIOS[@]}"; do
  IFS=':' read -r name pos offset <<< "$entry"
  echo "=== $name (start: $pos | goal_offset: $offset) ==="

  cp "$BACKUP" "$CONSTANTS"
  sed -i -E "s/^const std::vector<std::vector<Point_3>> surf_list = .*/const std::vector<std::vector<Point_3>> surf_list = ${name};/" "$CONSTANTS"
  sed -i -E "s/^const Point_3 current_foot_pos\(.*\);.*/const Point_3 current_foot_pos(${pos});/" "$CONSTANTS"
  sed -i -E "s/^const Vector_3 goal_offset\(.*\);.*/const Vector_3 goal_offset(${offset});/" "$CONSTANTS"

  if ! cmake --build build --target golden_capture -j3 > "/tmp/build_${name}.log" 2>&1; then
    echo "$name: BUILD FAILED (see /tmp/build_${name}.log)" | tee -a "$SUMMARY"
    continue
  fi

  if timeout 120s ./build/golden_capture "$name" "$GIT_SHA" "$OUT_DIR" > "/tmp/run_${name}.log" 2>&1; then
    astar_ok=$(python3 -c "import json;print(json.load(open('$OUT_DIR/${name}_astar.json')).get('success'))" 2>/dev/null || echo "?")
    nas_ok=$(python3 -c "import json;print(json.load(open('$OUT_DIR/${name}_nas.json')).get('success'))" 2>/dev/null || echo "?")
    echo "$name: ran — astar success=$astar_ok, nas success=$nas_ok" | tee -a "$SUMMARY"
  else
    echo "$name: RUN FAILED/TIMEOUT (see /tmp/run_${name}.log)" | tee -a "$SUMMARY"
  fi
done

restore_constants
trap - EXIT
echo ""
echo "Done. Summary: $SUMMARY"
cat "$SUMMARY"
