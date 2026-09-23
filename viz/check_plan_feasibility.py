"""Independent feasibility check of plan dumps, using neither footstep QP.

Reads plan JSONs written by apps/astar_plan and the reachability polytopes (.obj), and tests
every constraint the footstep QP is supposed to satisfy, directly:
  - reachability: for each step i, A . R(yaw_{i-1})^T (x_i - x_{i-1}) <= b, with the polytope of
    the stepping foot in the support foot's frame (LF in RF or RF in LF), as in the old code;
  - surface: every intermediate footstep lies on its patch (on the patch plane, inside its polygon);
  - the first footstep is the start, the last is the goal.
A plan passes when the largest violation of each family is below --tol (default 1e-6 m).

Usage: python3 viz/check_plan_feasibility.py <talos_reachability_dir> <plan.json> [<plan.json> ...]
"""
import json
import math
import sys

import numpy as np
from scipy.spatial import ConvexHull

RF_IN_LF = "RF_constraints_in_LF_quasi_flat_REDUCED.obj"
LF_IN_RF = "LF_constraints_in_RF_quasi_flat_REDUCED.obj"


def hrep(path):
    """H-representation A x <= b of the convex hull of an .obj's vertices."""
    pts = np.array([[float(t) for t in l.split()[1:4]] for l in open(path) if l.startswith("v ")])
    eq = ConvexHull(pts).equations          # normal . x + offset <= 0 inside
    return eq[:, :3], -eq[:, 3]


def check(plan, polys, tol):
    feet = np.array([f["position"] for f in plan["footsteps"]])
    path = plan["path"]
    n = len(path)
    out = {"reach": 0.0, "plane": 0.0, "inside": 0.0, "start": 0.0, "goal": 0.0}
    out["start"] = float(np.abs(feet[0] - np.array(plan["start"])).max())
    out["goal"] = float(np.abs(feet[-1] - np.array(plan["goal"])).max())
    for i in range(1, n):
        A, b = polys[path[i]["stance_foot"]]          # stance 0 = LF -> LF in RF ; 1 = RF -> RF in LF
        yaw = path[i - 1]["foot_yaw"]
        R = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
        local = R.T @ (feet[i] - feet[i - 1])
        out["reach"] = max(out["reach"], float((A @ local - b).max()))
    for i in range(1, n - 1):
        v = np.array(path[i]["patch_vertices"])
        # patch plane through its (coplanar) vertices
        nrm = np.cross(v[1] - v[0], v[2] - v[0]); nrm /= np.linalg.norm(nrm)
        out["plane"] = max(out["plane"], float(abs(nrm @ (feet[i] - v[0]))))
        # inside the convex polygon (xy), either winding
        c = v[:, :2].mean(axis=0)
        worst = 0.0
        for k in range(len(v)):
            a, bb = v[k, :2], v[(k + 1) % len(v), :2]
            e = bb - a; ln = np.linalg.norm(e)
            if ln < 1e-12: continue
            nn = np.array([-e[1], e[0]]) / ln
            if nn @ (c - a) > 0: nn = -nn          # outward normal
            worst = max(worst, float(nn @ (feet[i, :2] - a)))
        out["inside"] = max(out["inside"], worst)
    return out


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    tol = 1e-6
    d = sys.argv[1]
    polys = {0: hrep(f"{d}/{LF_IN_RF}"), 1: hrep(f"{d}/{RF_IN_LF}")}
    bad = 0
    print(f"{'scene':20s} {'steps':>5s} {'reach':>10s} {'plane':>10s} {'inside':>10s} {'start':>10s} {'goal':>10s}  verdict")
    for p in sys.argv[2:]:
        plan = json.load(open(p))
        if not plan.get("path_found"):
            print(f"{plan['scenario']:20s} {'no path':>5s}"); continue
        if not plan.get("qp_success"):
            print(f"{plan['scenario']:20s} QP FAILED: no footsteps to check"); bad += 1; continue
        r = check(plan, polys, tol)
        ok = all(v <= tol for v in r.values())
        bad += 0 if ok else 1
        print(f"{plan['scenario']:20s} {len(plan['footsteps']):5d} {r['reach']:10.2e} {r['plane']:10.2e} {r['inside']:10.2e} {r['start']:10.2e} {r['goal']:10.2e}  {'FEASIBLE' if ok else 'VIOLATED'}")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
