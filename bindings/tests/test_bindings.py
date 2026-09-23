"""nas_bindings tests — plain assert, no pytest dependency (matches the
rest of the repo's C++ tests, which don't use an external framework
either). Run with: python3 test_bindings.py <path to built nas_bindings.so's dir>
"""

import math
import os
import sys

if len(sys.argv) != 2:
    print(f"Usage: {sys.argv[0]} <build_dir_containing_nas_bindings.so>", file=sys.stderr)
    sys.exit(1)

sys.path.insert(0, sys.argv[1])
import nas_bindings as nb

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
BINDINGS_DIR = os.path.dirname(THIS_DIR)
REPO_ROOT = os.path.dirname(BINDINGS_DIR)
TALOS_DATA_DIR = os.path.join(REPO_ROOT, "talosReachability", "data", "reachability_constraints")
EXAMPLES_DIR = os.path.join(REPO_ROOT, "apps", "astar_plan", "examples")


def near(a, b, eps=1e-6):
    return abs(a - b) < eps


def test_available_scenarios_lists_all():
    names = nb.available_scenarios()
    assert len(names) == 15  # the 11 of the old environments.hpp + Ramp, SteepRamp, SlopedGround, SideSlope
    assert len(set(names)) == 15
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


def test_narrow_passage_matches_golden():
    result = nb.plan("NarrowPassage", os.path.join(EXAMPLES_DIR, "narrow_passage.json"), TALOS_DATA_DIR)
    assert result.success
    # 29 steps + start node, matching the paper's Table I (NarrowPassage,
    # with rotation) and the golden reference captured in phase 0.
    assert len(result.positions) == 30
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
    test_narrow_passage_matches_golden()
    test_three_paths_nas_matches_golden()
    test_unknown_scenario_raises()
    print("All bindings tests passed.")
