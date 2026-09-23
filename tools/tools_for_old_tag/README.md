# tools/tools_for_old_tag

Tools that only build in a checkout of an older commit, not at HEAD.

- `check_old_astar.cpp`: is a plan dump of the rewritten CASSR valid for the OLD A*? Needs `ExpansionParams::legacy_clip` and `legacy_node_keys`, which exist at the tag `cassr-stage-a-validated` (bit-identical to the old `get_children`, proved at tag `legacy-replay-verified`) and were removed afterwards. Build it there: `git worktree add <dir> cassr-stage-a-validated`, copy the file to `tests/golden_all/`, add an `add_executable(nas_check_old_astar ...)` linking `nas_config` like the other tools, run `nas_check_old_astar <scene> <plan.json>`.
