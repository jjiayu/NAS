"""nas_bindings tests — plain assert, no pytest dependency (matches the
rest of the repo's C++ tests, which don't use an external framework
either). Run with: python3 test_bindings.py <path to built nas_bindings.so's dir>
"""

import glob
import importlib.util
import math
import os
import sys

if len(sys.argv) != 2:
    print(f"Usage: {sys.argv[0]} <build_dir_containing_nas_bindings.so>", file=sys.stderr)
    sys.exit(1)

# Load the .so from the given build dir by explicit file path, NOT `sys.path.insert` + plain
# `import nas_bindings` — a `pip install -e .` editable install (bindings/README.md's own documented
# setup) registers a sys.meta_path finder that intercepts `import nas_bindings` before sys.path is
# even consulted, silently shadowing this build dir with whatever was last `pip install`-ed
# (discovered when this exact mechanism made both ctest -R nas_bindings and this script test a stale
# build with none of a session's changes in it, no error, just wrong results).
def _load_nas_bindings(build_dir):
    candidates = glob.glob(os.path.join(build_dir, "nas_bindings*"))
    if not candidates:
        raise RuntimeError(f"no nas_bindings module found in {build_dir}")
    spec = importlib.util.spec_from_file_location("nas_bindings", candidates[0])
    module = importlib.util.module_from_spec(spec)
    sys.modules["nas_bindings"] = module
    spec.loader.exec_module(module)
    return module


nb = _load_nas_bindings(sys.argv[1])

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
BINDINGS_DIR = os.path.dirname(THIS_DIR)
REPO_ROOT = os.path.dirname(BINDINGS_DIR)
TALOS_DATA_DIR = os.path.join(REPO_ROOT, "talosReachability", "data", "reachability_constraints")
EXAMPLES_DIR = os.path.join(REPO_ROOT, "apps", "astar_plan", "examples")


def near(a, b, eps=1e-6):
    return abs(a - b) < eps


def test_available_scenarios_lists_all():
    names = nb.available_scenarios()
    # 11 of the old environments.hpp + Ramp/SteepRamp/SlopedGround/SideSlope + StairsGap/
    # BoxRoomStairs added later -- this count drifted stale before (was hardcoded 15, missing the
    # last two), check src/config/scenario_library.cpp's own registry() if this fails again rather
    # than just bumping the number.
    assert len(names) == 17, len(names)
    assert len(set(names)) == 17
    print("test_available_scenarios_lists_all passed")


def test_goal_offset_config_is_resolved():
    # The per-scenario configs give the goal as an offset from the last surface's centroid; the bindings must resolve it
    # (they once left the goal at the origin, so the "plan" was a trivial one-step path).
    result = nb.plan("Stairs", os.path.join(EXAMPLES_DIR, "Stairs.json"), TALOS_DATA_DIR)
    assert result.success
    assert len(result.positions) == 6, len(result.positions)
    last = result.positions[-1]
    assert all(abs(a - b) < 1e-6 for a, b in zip(last, (1.35, 0.22, 0.4))), last  # centroid of Stairs' last step
    print("test_goal_offset_config_is_resolved passed")


def test_goal_as_a_surface():
    # goal_surface instead of a goal position: the last footstep is free on the last patch, not pinned to a point
    result = nb.plan("Stairs", os.path.join(EXAMPLES_DIR, "Stairs_goal_surface.json"), TALOS_DATA_DIR)
    assert result.success and len(result.positions) > 2
    last = result.positions[-1]
    assert 1.2 - 1e-6 <= last[0] <= 1.5 + 1e-6 and -0.16 - 1e-6 <= last[1] <= 0.6 + 1e-6 and abs(last[2] - 0.4) < 1e-6, last  # on Stairs' last step
    assert not all(abs(a - b) < 1e-6 for a, b in zip(last, (1.35, 0.22, 0.4))), last  # not pinned to its centroid
    print("test_goal_as_a_surface passed")


# --- plan_with_config / PlannerConfig / FootGoal: build a config directly in Python, no JSON file.
# Every case below mirrors one of the JSON-based tests above/an existing example config, to prove the
# two front-ends converge on the same nas::config::PlannerConfig (bindings/src/module.cpp's own
# planner_config_from_py()) rather than testing a second, independently-behaving code path.

def _rotation_config(start_position):
    # Shared knobs every case below needs: rotation on both sides (search AND QP -- two separate
    # flags, astar.expansion.rotation_enabled and qp.rotation_enabled in the JSON schema, easy to
    # set only one and get a spurious QP infeasibility).
    cfg = nb.PlannerConfig(start_position=start_position)
    cfg.rotation_enabled = True
    cfg.yaw_discretization_num = 3
    cfg.yaw_angle_increment_deg = 10.0
    cfg.qp_rotation_enabled = True
    return cfg


def test_plan_with_config_point_goal_matches_plan():
    cfg = _rotation_config((0.0, 0.0, 0.0))
    cfg.foot_goals.left = nb.FootGoal.point((8.0, 0.0, 0.0))
    direct = nb.plan_with_config("NarrowPassage", cfg, TALOS_DATA_DIR)
    from_json = nb.plan("NarrowPassage", os.path.join(EXAMPLES_DIR, "narrow_passage.json"), TALOS_DATA_DIR)
    assert direct.success and from_json.success
    assert direct.expansion_count == from_json.expansion_count == 115
    assert len(direct.positions) == len(from_json.positions) == 34
    for a, b in zip(direct.positions, from_json.positions):
        assert all(near(x, y) for x, y in zip(a, b)), (a, b)
    print("test_plan_with_config_point_goal_matches_plan passed")


def test_plan_with_config_surface_goal():
    cfg = _rotation_config((0.1, 0.0, 0.0))
    cfg.foot_goals.left = nb.FootGoal.surface(4)  # Stairs' last step, see Stairs_goal_surface.json
    result = nb.plan_with_config("Stairs", cfg, TALOS_DATA_DIR)
    assert result.success and len(result.positions) > 2
    last = result.positions[-1]
    assert 1.2 - 1e-6 <= last[0] <= 1.5 + 1e-6 and -0.16 - 1e-6 <= last[1] <= 0.6 + 1e-6 and near(last[2], 0.4), last
    print("test_plan_with_config_surface_goal passed")


def test_plan_with_config_offset_goal():
    cfg = _rotation_config((0.1, 0.0, 0.0))
    cfg.foot_goals.left = nb.FootGoal.offset((0.0, 0.0, 0.0))  # last surface centroid, see Stairs.json
    result = nb.plan_with_config("Stairs", cfg, TALOS_DATA_DIR)
    assert result.success and len(result.positions) == 6
    assert all(near(a, b) for a, b in zip(result.positions[-1], (1.35, 0.22, 0.4))), result.positions[-1]
    print("test_plan_with_config_offset_goal passed")


def test_plan_with_config_polytope_and_yaw_range():
    cfg = _rotation_config((0.0, 0.0, 0.0))
    goal = nb.FootGoal.polytope([(0.1, -0.3, 0.0), (0.3, -0.3, 0.0), (0.3, -0.05, 0.0), (0.1, -0.05, 0.0)])
    goal.yaw_range_deg = (-90.0, 0.0)
    cfg.foot_goals.right = goal
    result = nb.plan_with_config("Flat", cfg, TALOS_DATA_DIR)
    assert result.success
    assert result.stance_feet[-1] == 1  # Right -- FootstepResult.stance_feet stays a plain int (0=Left/1=Right)
    yaw_deg = result.foot_yaws[-1] * 180.0 / math.pi
    assert -90.0 - 1e-6 <= yaw_deg <= 0.0 + 1e-6, yaw_deg
    print("test_plan_with_config_polytope_and_yaw_range passed")


def test_plan_with_config_requires_a_foot_goal():
    cfg = nb.PlannerConfig(start_position=(0.0, 0.0, 0.0))  # foot_goals.left/right both left unset
    threw = False
    try:
        nb.plan_with_config("Flat", cfg, TALOS_DATA_DIR)
    except ValueError:
        threw = True
    assert threw
    print("test_plan_with_config_requires_a_foot_goal passed")


def test_narrow_passage_matches_golden():
    result = nb.plan("NarrowPassage", os.path.join(EXAMPLES_DIR, "narrow_passage.json"), TALOS_DATA_DIR)
    assert result.success
    # 33 steps + start node (34 nodes), 115 expansions -- was 29 steps/30 nodes before the
    # PatchIndex yaw-bin fix (2026-09-27, commit 540e9f3): that fix corrected a truncation bug that
    # doubled the width of the yaw~=0 bucket, which had been silently merging NarrowPassage's true
    # optimum away (see docs/patchindex-scalability-note.md). This test's own expectation was never
    # updated when that commit landed (NAS_BUILD_BINDINGS is OFF by default, so nobody ran it) --
    # astar_search_golden_test.cpp/golden_all_scenes_test.cpp already document NarrowPassage as the
    # one accepted exception to the paper's Table I; this file was the odd one out.
    assert len(result.positions) == 34, len(result.positions)
    assert result.expansion_count == 115, result.expansion_count
    assert near(result.positions[0][0], 0.0)
    assert near(result.positions[-1][0], 8.0)
    print("test_narrow_passage_matches_golden passed")


def test_three_paths_nas_matches_golden():
    result = nb.plan("ThreePathsNAS", os.path.join(EXAMPLES_DIR, "three_paths_nas.json"), TALOS_DATA_DIR)
    assert result.success
    # 19 steps + start node, matching the paper's "local minima" scenario.
    assert len(result.positions) == 20
    print("test_three_paths_nas_matches_golden passed")


def test_unknown_scenario_raises():
    # config::load_scenario throws std::out_of_range, which nanobind maps
    # to Python's IndexError (not RuntimeError) — see nanobind's built-in
    # exception translation table.
    threw = False
    try:
        nb.plan("DoesNotExist", os.path.join(EXAMPLES_DIR, "narrow_passage.json"), TALOS_DATA_DIR)
    except IndexError:
        threw = True
    assert threw
    print("test_unknown_scenario_raises passed")


if __name__ == "__main__":
    test_available_scenarios_lists_all()
    test_goal_offset_config_is_resolved()
    test_goal_as_a_surface()
    test_plan_with_config_point_goal_matches_plan()
    test_plan_with_config_surface_goal()
    test_plan_with_config_offset_goal()
    test_plan_with_config_polytope_and_yaw_range()
    test_plan_with_config_requires_a_foot_goal()
    test_narrow_passage_matches_golden()
    test_three_paths_nas_matches_golden()
    test_unknown_scenario_raises()
    print("All bindings tests passed.")
