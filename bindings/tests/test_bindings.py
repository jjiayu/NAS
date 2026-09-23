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
    test_narrow_passage_matches_golden()
    test_three_paths_nas_matches_golden()
    test_unknown_scenario_raises()
    print("All bindings tests passed.")
