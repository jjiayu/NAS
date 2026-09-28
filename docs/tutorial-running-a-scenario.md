# Tutoriel : lancer un scénario

Ce document montre comment lancer une planification de bout en bout — par le CLI, par Python, et en
écrivant un fichier de config JSON à la main. Pour comprendre CE QUE ça calcule, voir
`docs/footstep-planning-mechanism.md`. La section cube (tout en bas) suppose ce document lu d'abord.

## Prérequis

Build C++ fait (`cmake -S . -B build && cmake --build build -j4` — voir le `README.md` racine), et
les données d'atteignabilité Talos disponibles (`talosReachability/data/reachability_constraints`,
sous-paquet du dépôt).

## Méthode 1 : le CLI `astar_plan`

```sh
./build/apps/astar_plan/astar_plan <scenario> <config.json> <talos_reachability_data_dir> <output.json>
```

Exemple concret, un des scénarios livrés :

```sh
./build/apps/astar_plan/astar_plan NarrowPassage \
    apps/astar_plan/examples/narrow_passage.json \
    talosReachability/data/reachability_constraints \
    /tmp/out.json
```

`<scenario>` est un nom de `config::available_scenarios()` (`Flat`, `Stairs`, `NarrowPassage`, ... —
voir `src/config/scenario_library.cpp` pour la liste complète). `apps/astar_plan/examples/` contient
un fichier de config par scénario livré, à utiliser tel quel ou comme point de départ.

## Méthode 2 : Python

```python
import nas_bindings

result = nas_bindings.plan("NarrowPassage", "apps/astar_plan/examples/narrow_passage.json",
                            "talosReachability/data/reachability_constraints")
if result.success:
    print(result.positions)       # [[x, y, z], ...]
    print(result.expansion_count, result.search_ms, result.qp_ms)
```

Nécessite les bindings compilés (`NAS_BUILD_BINDINGS=ON`, ou `pip install --no-build-isolation -e .`
— voir le `README.md` racine).

`plan()` prend un fichier de config JSON, comme le CLI. Pour construire le but (et le reste de la
config) directement en Python, sans fichier — pratique pour itérer depuis un REPL/notebook —
`plan_with_config()` :

```python
config = nas_bindings.PlannerConfig(start_position=(0.0, 0.0, 0.0))
config.foot_goals.left = nas_bindings.FootGoal.point((8.0, 0.0, 0.0))
config.rotation_enabled = True          # astar.expansion.rotation_enabled
config.qp_rotation_enabled = True       # qp.rotation_enabled -- séparé, penser aux deux

result = nas_bindings.plan_with_config("NarrowPassage", config,
                                        "talosReachability/data/reachability_constraints")
```

`nas_bindings.FootGoal` a une usine statique par forme de région (`.point()`, `.surface()`,
`.polytope()`, `.offset()`, `.polygon_2d()`), même 5 formes que le JSON (section suivante) —
`goal.yaw_range_deg = (min, max)` optionnel sur chacune. `PlannerConfig` couvre les mêmes champs que
les sections JSON `"astar"`/`"qp"`, avec les mêmes valeurs par défaut. Détails complets :
`bindings/README.md`.

## Écrire un fichier de config

Squelette minimal :

```json
{
  "astar": {
    "start_position": [0.0, 0.0, 0.0],
    "foot_goals": {
      "left": { "point": [1.0, 0.0, 0.0] }
    }
  }
}
```

Deux clés obligatoires sous `astar` : `start_position` et `foot_goals` (au moins `"left"` ou
`"right"`, ou les deux). Tout le reste a une valeur par défaut raisonnable (voir
`AstarSearchConfig`/`ExpansionParams` dans `include/nas/planners/astar_search.hpp` pour la liste
complète et leur signification — `heuristic_weight`, `node_similarity_threshold`,
`patch_index_cell_size`, `expansion.rotation_enabled`, etc.).

### La forme du but : `foot_goals.left`/`foot_goals.right`

Chaque slot (`left`/`right`) accepte exactement une des 4 formes suivantes pour sa région, plus un
`yaw_range_deg` optionnel (`[min, max]` en degrés — la plage doit être "dépliée" si elle traverse
±180°, ex. `[170, 190]`, pas `[170, -170]`).

| Forme | Exemple | Sens |
|---|---|---|
| `point` | `{"point": [1.0, 0.0, 0.0]}` | Un point précis (monde). |
| `surface` | `{"surface": 2}` | N'importe où sur cette surface (indice dans la scène) — pas de géométrie, juste "avoir posé ce pied dessus". |
| `polytope` | `{"polytope": [[x,y,z], ...]}` (≥3 sommets) | Une région 3D arbitraire (monde). |
| `surface` + `polygon_2d` | `{"surface": 0, "polygon_2d": [[u,v], ...]}` | Un polygone dans le repère 2D local de cette surface, converti en 3D au chargement (voir plus bas). |
| `offset` | `{"offset": [0.0, 1.0, 0.0]}` | Centroïde de la DERNIÈRE surface de la scène + ce vecteur (pratique pour un but "au bout du scénario", indépendant de ses coordonnées exactes). |

`surface`/`offset`/`polygon_2d` ont besoin de la scène pour être résolus (indice de surface, ou
transform monde↔surface) — c'est fait une fois, après le chargement du scénario, par
`config::resolve_goal()` (déjà appelé par le CLI et les bindings Python ; à appeler soi-même si on
écrit un nouvel appelant C++). `qp_goal()` ne renvoie un point au QP que pour la forme `point` — les
3 autres laissent le dernier pas libre sur son patch (voir `docs/footstep-planning-mechanism.md`,
section QP, pour ce que ça implique).

**Exemple : but sur une surface entière** (`apps/astar_plan/examples/Stairs_goal_surface.json`) :

```json
"foot_goals": { "left": { "surface": 4 } }
```

**Exemple : polygone 2D sur une surface** (`apps/astar_plan/examples/Flat_polygon_2d.json`) : pour
une surface horizontale, le repère local a ses axes alignés avec le monde (`x_axis=(1,0,0)`,
`y_axis=(0,1,0)`, origine au centroïde de la surface — voir
`Surface::establish_surface_coordinate_system`, `src/core/surface.cpp`) ; les coordonnées locales
`(u, v)` d'un point valent donc simplement `(x_monde − x_centroïde, y_monde − y_centroïde)`. Pour une
surface inclinée, calculer les coordonnées locales à la main n'est pas pratique — passer par
`polytope` (coordonnées monde directes) est plus simple dans ce cas.

**2 slots remplis** ("closing stance") : le chemin ne termine que quand les 2 DERNIERS pas
consécutifs satisfont chacun leur propre slot en même temps (voir le mécanisme, déjà documenté). Pas
un simple "et" de 2 buts atteints à des moments différents.

### Options courantes

```json
{
  "astar": {
    "start_position": [0, 0, 0],
    "start_stance_foot": "Right",
    "foot_goals": { "left": { "point": [4, 0, 0] } },
    "distance_metric": "Epa",
    "heuristic_weight": 10.0,
    "goal_yaw_weight": 0.5,
    "expansion": {
      "rotation_enabled": true,
      "yaw_discretization_num": 3,
      "yaw_angle_increment_deg": 10.0
    }
  },
  "qp": {
    "alpha_weight": 10.0,
    "rotation_enabled": true
  }
}
```

`goal_yaw_weight` (biais souple vers le centre du `yaw_range` du slot ciblé) exige exactement 1 slot
rempli ET que ce slot ait un `yaw_range_deg` — sinon `AstarSearch` lève une erreur claire à la
construction.

## Lire la sortie

Le CLI écrit un JSON avec, entre autres : `path_found`, `foot_goals` (la forme résolue, pour
vérifier ce qui a été effectivement recherché), `expansions`, `search_ms`, `qp_ms`, `path` (un nœud
par pas : `centroid`, `patch_vertices`, `stance_foot`, `surface_id`, `foot_yaw`), `footsteps` (la
sortie du QP : une position concrète par pas). Les bindings Python renvoient l'équivalent aplati
(`FootstepResult`, voir `bindings/README.md`).

## Erreurs courantes

- **`AstarSearch: foot_goals has neither slot set`** — ni `left` ni `right` n'a de région valide.
- **`... 's surface index N is not a surface of the scenario`** — indice hors bornes pour `surface`
  (direct ou via `polygon_2d`) ; vérifier le nombre de surfaces du scénario choisi.
- **`... needs exactly one of "point"/"surface"/"polytope"/"offset"`** — 0 ou ≥2 formes de région
  données pour le même slot.
- **Un chemin non trouvé, `path_found: false`** — le but est peut-être hors d'atteinte depuis
  `start_position` avec le modèle d'atteignabilité chargé ; augmenter `expansion.yaw_discretization_num`
  ou vérifier que le but est bien à l'intérieur d'une surface (pensez à l'empreinte **rétrécie** par
  la moitié des dimensions du pied, pas les sommets bruts de la surface).

---

## Extension cube

Voir `docs/cube-extension-mechanism.md` pour le mécanisme (ramassage, pose, enjambement). Pour le
lancer : mêmes commandes CLI/Python ci-dessus, avec `astar.cube_half_extent`/`cube_height` (cube
porté dès le départ) ou `astar.scene_cubes` (cube au repos dans la scène, à ramasser) en plus dans le
JSON — pas encore de clé JSON pour `scene_cubes`/`pickup_affordance` aujourd'hui (JSON/CLI non câblé,
voir `docs/cube-extension-mechanism.md`, section "Ce qui manque") : cette partie se configure en C++
direct, voir `tests/golden/cube_pickup_and_placement_test.cpp` pour un exemple complet.
