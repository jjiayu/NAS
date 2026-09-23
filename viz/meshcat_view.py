"""Interactive 3D view of a plan dump in meshcat (see PLAN.md phase 11).

Reads the JSON that apps/astar_plan (or tests/viz_dump/dump_plan.cpp) writes
- {"surfaces", "path", "footsteps", ...} - and draws it in a running
meshcat-server: walkable patches, footsteps as feet (blue = left stance,
orange = right stance, rotated by their yaw), and the stride line.

Uses the meshcat-python client that's already in the conda env rather than
the C++ meshcat-cpp (whose dependency chain was judged too heavy for this
machine, see PLAN.md) - the planner never needs to import this, it only
consumes the JSON, so viz stays decoupled from the core.

Usage:
    meshcat-server &            # prints its zmq:// and http:// URLs
    python3 meshcat_view.py <plan.json> <zmq_url>
"""

import json
import sys

import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf
import numpy as np

# Cosmetic foot box (m). Not the planner's foot_length/foot_width, which
# describe the margin used to shrink surfaces, not the sole's outline.
FOOT_SIZE = (0.22, 0.12, 0.02)

LEFT_STANCE, RIGHT_STANCE = 0, 1
COLORS = {
    LEFT_STANCE: 0x2f6fd6,
    RIGHT_STANCE: 0xe8892b,
}
SURFACE_COLOR = 0x8a9aa8


def draw_surfaces(vis, surfaces):
    for i, verts in enumerate(surfaces):
        v = np.array(verts, dtype=float)
        if len(v) < 3 or not np.isfinite(v).all():
            continue
        # Patches are convex and CCW-ordered, so a triangle fan is exact.
        faces = np.array([[0, k, k + 1] for k in range(1, len(v) - 1)], dtype=np.uint32)
        material = g.MeshLambertMaterial(color=SURFACE_COLOR, opacity=0.55, transparent=True, side=2)
        vis[f"surfaces/{i}"].set_object(g.TriangularMeshGeometry(v, faces), material)


def draw_footsteps(vis, footsteps):
    points = []
    for i, fs in enumerate(footsteps):
        x, y, z = fs["position"]
        points.append([x, y, z])
        foot = g.Box(list(FOOT_SIZE))
        material = g.MeshLambertMaterial(color=COLORS[fs["stance_foot"]])
        transform = tf.translation_matrix([x, y, z + FOOT_SIZE[2] / 2]) @ tf.rotation_matrix(fs["foot_yaw"], [0, 0, 1])
        vis[f"footsteps/{i:03d}"].set_object(foot, material)
        vis[f"footsteps/{i:03d}"].set_transform(transform)

    if len(points) >= 2:
        line = g.Line(g.PointsGeometry(np.array(points, dtype=float).T), g.LineBasicMaterial(color=0x222222))
        vis["stride_line"].set_object(line)


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        return 1
    with open(sys.argv[1]) as f:
        plan = json.load(f)

    vis = meshcat.Visualizer(zmq_url=sys.argv[2])
    vis.delete()
    draw_surfaces(vis, plan["surfaces"])
    draw_footsteps(vis, plan.get("footsteps", []))

    print(f"{plan.get('scenario', '?')}: {len(plan['surfaces'])} surfaces, "
          f"{len(plan.get('footsteps', []))} footsteps, qp_success={plan.get('qp_success')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
