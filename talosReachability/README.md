# talosReachability

`.obj` files describing the kinematic reachability constraints of the Talos robot, used by the [NAS/CASSR](../) footstep planner.

Packaging structure mirrors [go2Reachability](https://github.com/) — same CMake/Python config pattern — but this package does **not** generate the constraint files: they are the ones already shipped in this repo's `data/constraints_files/`, just repackaged as an installable, `find_package`-able unit. Currently nested inside the NAS repo; intended to move to its own sibling repo later.

---

## Installation

1. **Build and install using CMake**:

```bash
mkdir build
cd build
cmake ..
cmake --install . --prefix /desired/install/path
```

This will install:

- `.obj` files in `share/talosReachability/reachability_constraints`
- CMake config: `share/talosReachability/cmake/talosReachabilityConfig.cmake`
- Python config: `share/talosReachability/talosReachabilityConfig.py`

2. **Add Python config to your PYTHONPATH**:

```bash
export PYTHONPATH=/desired/install/path/share/talosReachability:$PYTHONPATH
```

## Usage

1. **Using the files in Python**:
```python
from talosReachabilityConfig import TALOSREACHABILITY_CONSTRAINTS_DIR
import os, glob

obj_files = glob.glob(os.path.join(TALOSREACHABILITY_CONSTRAINTS_DIR, "*.obj"))
print("Found OBJ files:", obj_files)
```

2. **Using the files in C++**:
```cmake
find_package(talosReachability REQUIRED PATHS /desired/install/path/share/talosReachability/cmake)
message(STATUS "Reachability OBJ dir = ${TALOSREACHABILITY_CONSTRAINTS_DIR}")
```

## Naming convention

Unlike go2Reachability (which only ever needs the "forward" direction — where the moving foot can go next), Talos's files come in two directions because the NAS planner searches backward from the goal while CASSR searches forward from the start:

- `<Foot>_constraints_in_<Foot>[...].obj` — **forward**: reachable positions for the named foot, in the other foot's frame. Used by CASSR and the footstep QP.
- `<Foot>_antecedent[...].obj` — **antecedent**: where the previous foot must have been, given the current one. Used by NAS/Tree's backward search.

## Known gaps

- No `LF_antecedent_*.obj` exists in this package (only `RF_antecedent_*.obj` variants) — `constants.hpp` in the main NAS repo references a nonexistent `LF_antecedent_CUTZ.obj`, which blocks NAS/Tree entirely regardless of scenario. Not fixed here (no confident replacement candidate). See the main repo's `docs/paper-deltas.md`.
- No `COM_constraints_in_*.obj` files exist — the CoM QP constraints referenced in `constants.hpp` and commented out in `footstep_planner.cpp` have no backing asset either.
