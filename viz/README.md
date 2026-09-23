# viz

- `meshcat_view.py <plan.json> <zmq_url>`: interactive 3D view of one plan dump in a running `meshcat-server`.
- `plans_report.py <dumps_dir> <golden_dir> <out.html>`: one self-contained HTML page with the plans of all 11 scenarios (top view of surfaces, patches and footsteps, height profile for stairs, the old code's plan in dashed violet, and a comparison table). No server needed. Give one dumps directory per variant (for instance the default cost and `astar.yaw_change_weight = 0.1`): the page then has a selector to compare them.

Minimal workflow, from the repository root (conda env `rwa`):

```sh
mkdir -p /tmp/plans
for sc in NarrowPassage Stairs TwoFlatSurfaces LongStairs LongLongStairs Flat LongStairsComplete LongStairsExp ThreePathsScene Stairs_Up_Down ThreePathsNAS; do
  cfg=apps/astar_plan/examples/$sc.json
  [ $sc = NarrowPassage ] && cfg=apps/astar_plan/examples/narrow_passage.json
  [ $sc = ThreePathsNAS ] && cfg=apps/astar_plan/examples/three_paths_nas.json
  apps/astar_plan/build/astar_plan $sc $cfg talosReachability/data/reachability_constraints /tmp/plans/$sc.json
done
python3 viz/plans_report.py tests/golden /tmp/plans.html "Coût 1 (papier)" /tmp/plans
```

- `check_plan_feasibility.py <talos_reachability_dir> <plan.json>...`: checks plan dumps against the QP's constraints with no QP involved (reachability against the true polytope, patch plane and polygon, start, goal). Needs numpy and scipy.
