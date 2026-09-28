"""B6: memory of a long-lived process that calls nas_bindings.plan() in a loop (the Python live usage).

Every call reloads the scenario, the config and the two reachability polytopes, runs the search and the
footstep QP, and frees everything. The resident size must not grow: after a warm-up, RSS (read from
/proc/self/statm after malloc_trim, so allocator retention is not counted) may grow by at most 2 MB over
the remaining calls. (Measured: 0.3 MB over 300 plans of Stairs, nothing visible on the others.)
Run with: python3 test_memory.py <build_dir_containing_nas_bindings.so>
"""
import ctypes
import glob
import importlib.util
import os
import sys

if len(sys.argv) != 2:
    print(f"Usage: {sys.argv[0]} <build_dir_containing_nas_bindings.so>", file=sys.stderr)
    sys.exit(1)

# Load the .so from the given build dir by explicit file path, NOT `sys.path.insert` + plain
# `import nas_bindings` — see test_bindings.py's own comment on this same helper: an editable
# `pip install -e .` shadows this build dir via a sys.meta_path finder that sys.path.insert cannot
# override.
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
REPO_ROOT = os.path.dirname(os.path.dirname(THIS_DIR))
TALOS = os.path.join(REPO_ROOT, "talosReachability", "data", "reachability_constraints")
EXAMPLES = os.path.join(REPO_ROOT, "apps", "astar_plan", "examples")
libc = ctypes.CDLL("libc.so.6")

MAX_GROWTH_KB = 2048


def rss_kb():
    libc.malloc_trim(0)
    with open("/proc/self/statm") as f:
        return int(f.read().split()[1]) * (os.sysconf("SC_PAGE_SIZE") // 1024)


def check(scene, config, plans, warmup):
    baseline = None
    for i in range(plans):
        r = nb.plan(scene, os.path.join(EXAMPLES, config), TALOS)
        assert r.success and len(r.positions) > 2, (scene, i)  # a real plan every time, not a trivial one
        if i == warmup:
            baseline = rss_kb()
    growth = rss_kb() - baseline
    print(f"{scene}: {plans} plans, RSS growth after {warmup} warm-up plans: {growth} kB (limit {MAX_GROWTH_KB} kB)")
    assert growth <= MAX_GROWTH_KB, f"{scene}: resident memory grew by {growth} kB over {plans - warmup} plans"


check("Stairs", "Stairs.json", plans=150, warmup=20)
check("NarrowPassage", "narrow_passage.json", plans=40, warmup=10)
check("ThreePathsScene", "ThreePathsScene.json", plans=20, warmup=5)
print("test_memory passed")
