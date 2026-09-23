# tests/golden_all

Checks of the rewritten CASSR on every scene of the old `environments.hpp`.

## `nas_golden_all_scenes` (ctest)
Replays each `tests/golden/*_astar.json` scene with the old `constants.hpp` configuration and compares against the plan the old code stored: path length, depth, stance foot and surface id per node (yaw is reported only: equal-cost plans differ by yaw ties, which the old code broke arbitrarily), and, where the old QP succeeded, the QP (footstep count, distance walked within 6%). Also checks determinism: each search runs twice, the second after a heap fragmentation, and must be identical.

## `nas_expansion_oracle` (ctest)
`core/expansion` against an exact-arithmetic oracle (`exact_clip.hpp`) on the states a real search expands, on every scene: for each expanded parent and each surface the child patch must equal the patch recomputed from the plane cut with the 2D projection, hull and clip done in exact arithmetic, children must exist exactly when that patch is non-empty and the cycle detection allows them, and every patch must be a clean convex polygon. This is what proves the corrected 2D clip (the old code dropped an intersection point on near-parallel edges, `docs/paper-deltas.md`).

## Tools (not tests)
- `nas_perf_all_scenes [runs]`: search + QP timing on every scene. The same source builds in a checkout of an older commit (e.g. tag `legacy-replay-verified`) to get an old-behaviour baseline on the same machine.
- `nas_trace_divergence <scene> <seed>`: runs a scene twice (the second after a heap fragmentation) and reports the first divergence between the two searches; uses the `on_expand` / `on_child` hooks of `AstarSearchConfig`. The search used to diverge on 7 of 11 scenes and is now deterministic, so this is a regression diagnostic.

## Where the old-code proofs went
The old CASSR was removed from the tree. Its differential test (`old_expansion_dump` + replay of the old `get_children`'s exact output: bit-identical on ~18 000 children on 11 scenes) and the tools that captured the golden plans live in git history: tags `legacy-replay-verified` and `cassr-stage-a-validated`. The stored plans in `tests/golden/` remain the reference.
