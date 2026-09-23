# footstep_qp

Footstep QP formulation and solver backend. Builds a `QPProblem` once, backend-agnostic, from a CASSR result path. Direct Eigen matrix construction replaces the old code's CasADi symbolic expression tree (never actually needed for autodiff — every Jacobian there was already hand-built).

## API

- [`include/nas/footstep_qp/qp_backend.hpp`](include/nas/footstep_qp/qp_backend.hpp): `QPProblem`/`QPSolution`, abstract `QPBackend`. Convention: minimize `0.5 x'Hx + g'x` s.t. `A_eq x = b_eq`, `A_ineq x <= b_ineq` (bounds folded into inequality rows).
- [`include/nas/footstep_qp/quadprog_backend.hpp`](include/nas/footstep_qp/quadprog_backend.hpp): `QuadprogBackend`, wraps [eiquadprog](https://github.com/stack-of-tasks/eiquadprog) (LAAS/Gepetto, Eigen-native, same lineage as Pinocchio/coal).
- [`include/nas/footstep_qp/footstep_qp.hpp`](include/nas/footstep_qp/footstep_qp.hpp): `solve_footstep_qp(path_nodes, start_position, goal_position, reachability, config, backend)` and `FootstepQPConfig`/`FootstepPlan`.

Only reproduces the constraint blocks the old `FootstepPlanner` actually used — `reachability_constraints_prev`/`com_constraints_next` were dead code in the old implementation, not ported (see `docs/paper-deltas.md`).

Numerical note: the stride-length objective is a graph Laplacian (only positive *semi*-definite, 3D translational null space). `eiquadprog`'s Cholesky-based method needs strict positive-definiteness, unlike the old qpOASES pipeline — fixed with a small Tikhonov regularization (`FootstepQPConfig::hessian_regularization = 1e-8`), harmless since the start/goal equality constraints already pin down the null space.

Validated against golden: max deviation 3.8e-08m on `ThreePathsNAS`. ~53x faster than the old CasADi/qpOASES pipeline, and succeeds on `NarrowPassage` where the old QP failed.

## Dependencies

`core/geometry`, `core/node`, `core/reachability`, Eigen, eiquadprog.

## Testing standalone

```sh
mkdir build && cd build && cmake .. && cmake --build . -j2 && ctest
```
