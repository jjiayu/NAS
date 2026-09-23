# legacy

The original NAS/CASSR implementation, kept for reference — it compiles but is not part of the
maintained package (see `../docs/history/PLAN.md` and `../docs/history/PROGRESS.md` for how the
CASSR rewrite superseded it; the rewrite's own proofs of equivalence against this code live in the
git tags `legacy-replay-verified`, `cassr-stage-a-validated`, `cassr-validated-v2`/`v3`).

CASSR was removed from this tree once validated — its old implementation is only reachable via
those tags, not as live code here. NAS is different: still genuinely useful as a reference/baseline
(the old A* over a discretized grid, and the CasADi/qpOASES footstep QP this repo's own `nas`
library was rewritten from), so it's kept live, just isolated from the maintained package.

## Building

Own standalone CMake project, own dependencies (VTK, freetype, yaml-cpp, CasADi — none of these are
needed by the maintained `nas` library at the repo root):

```bash
cmake -S legacy -B legacy/build -DCMAKE_BUILD_TYPE=Release
cmake --build legacy/build -j1   # -j1: see the repo owner's build-parallelism note, low-RAM machine
```

`test_bench_operations`/`test_print_paths` are the repo owner's own scratch tools (never committed,
not moved by any tooling) — those two targets only appear once the corresponding `.cpp` files are
placed in this directory; the rest of the build configures and succeeds either way.
