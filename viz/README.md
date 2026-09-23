# viz

- `meshcat_view.py <plan.json> <zmq_url>`: interactive 3D view of one plan dump in a running `meshcat-server`.
- `plans_report.py <dumps_dir> <golden_dir> <out.html>`: one self-contained HTML page with the plans of all 11 scenarios (top view of surfaces, patches and footsteps, height profile for stairs, the old code's plan in dashed violet, and a comparison table). No server needed.

Minimal workflow, from the repository root (conda env `rwa`):

```sh
mkdir -p /tmp/plans
for sc in NarrowPassage Stairs TwoFlatSurfaces LongStairs LongLongStairs Flat LongStairsComplete LongStairsExp ThreePathsScene Stairs_Up_Down ThreePathsNAS; do
  cfg=apps/astar_plan/examples/$sc.json
  [ $sc = NarrowPassage ] && cfg=apps/astar_plan/examples/narrow_passage.json
  [ $sc = ThreePathsNAS ] && cfg=apps/astar_plan/examples/three_paths_nas.json
  apps/astar_plan/build/astar_plan $sc $cfg talosReachability/data/reachability_constraints /tmp/plans/$sc.json
done
python3 viz/plans_report.py /tmp/plans tests/golden /tmp/plans.html
```
