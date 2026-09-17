# NAS/CASSR — plan de réécriture

Statut : planification terminée, implémentation non démarrée. Voir [PROGRESS.md](PROGRESS.md) pour l'avancement au jour le jour.

Ce document est la référence stable de l'architecture cible et du séquencement. Il ne bouge que quand une décision structurelle change. Pour "où on en est maintenant", voir PROGRESS.md, pas ce fichier.

## Pourquoi une réécriture

- `Tree::get_children` (NAS) et `AstarSearch::get_children` (CASSR) dupliquent ~90% du même code (somme de Minkowski → intersection surfaces → clip 2D → nœud(s) enfant(s)).
- Toute la config est en globals compile-time (`constants.hpp`) — a forcé la création de `test_bench_operations.cpp`/`test_print_paths.cpp` (non trackés) qui réimplémentent les classes en local pour pouvoir les paramétrer.
- Chemins d'assets absolus personnels en dur (`/media/stonneau/...`, `/Users/jiayu/...`, `$HOME/Desktop/...`).
- VTK forcé en dépendance de tout le cœur de recherche (pas de build/usage headless possible).
- `yaml-cpp` + `config/*.yaml` présents mais jamais branchés (morts).
- Contraintes CoM commentées/désactivées dans le QP.

## Architecture cible

```
core/geometry      # types + opérations géométriques (CGAL), agnostique au robot
core/surface        # Surface — inchangé dans l'esprit
core/node           # Node généralisé à N effecteurs (pas juste 2 pieds), possession explicite (pool/arena)
core/reachability   # ReachabilityModel — query(moving_effector, support_effector) -> Polyhedron
                     #   couche 0 SEULEMENT pour l'instant : chemins/répertoire de fichiers .obj explicites en entrée.
                     #   Pas de découverte automatique de package (find_package/import générique) — écarté pour l'instant.
core/expansion      # fonction d'expansion unifiée (le get_children commun à NAS et CASSR)
core/gait           # GaitSequencer — quel effecteur bouge ensuite (trivial L/R pour biped aujourd'hui)
planners/
  tree_search        # NAS — BFS exhaustif arrière, multi-parents
  astar_search        # CASSR — best-first avant, heuristique EPA/GJK
  grid_astar_search   # baseline discrétisée (portage direct, priorité basse)
footstep_qp         # étage QP partagé — interface QPBackend solveur-agnostique
config/             # RobotModel / Scenario / PlannerConfig chargés à l'exécution (plus de globals)
apps/                # drivers CLI minces (nas_plan / astar_plan / astar_grid_plan)
viz/                 # découplée du cœur, jamais une dépendance obligatoire
bindings/            # Python (nanobind)
talosReachability/  # package séparé (nested pour l'instant), packaging des assets Talos existants
```

## Décisions clés

- **QP** : interface `QPBackend` (H,g,A,b → x,status). `quadprog` = défaut/sûr (déjà utilisé dans l'écosystème sibling go2Motion). `ProxQP` = optionnelle, isolée derrière un flag de build (jugée encore trop jeune pour s'y engager). `CasADi` = gardé temporairement, uniquement pour valider la parité de formulation, retiré ensuite.
- **Viz** : VTK découplé du cœur. Cible probable meshcat-cpp pour le dev interactif (convention déjà en usage dans go2Reachability/go2Motion), export minimal séparé pour figures d'article.
- **Reachability loading** : couche 0 uniquement — chemins explicites. Pas de couche de découverte de package pour l'instant (conçue puis explicitement écartée : "j'aime pas trop le sucre pour l'instant"). Plus tard, en Python seulement : possibilité de construire un planner en lui passant un package qui sait extraire ses propres fichiers — forme exacte non spécifiée, à concevoir le moment venu.
- **Bindings Python** : nanobind pressenti par défaut (pas Boost.Python+eigenpy comme Pinocchio/coal — acceptable tant qu'il n'y a pas d'interop directe avec des objets Pinocchio/coal vivants dans le même process).
- **Node ownership** : pool/arena explicite (plus de `new` orphelin comme dans le code actuel) — condition nécessaire pour un usage "live" depuis un process Python longue durée.
- **talosReachability** : structure calquée sur `/media/stonneau/data/dev/linux/go2Reachability` (CMakeLists.txt glob+install, `.cmake.in` généré, config Python générée exposant `TALOSREACHABILITY_CONSTRAINTS_DIR`) mais SANS pipeline de génération Pinocchio — juste install des `.obj` déjà dans `data/constraints_files/`. Nested dans NAS pour l'instant, pensé pour extraction en repo sibling plus tard. Garder la distinction forward/antecedent dans le nommage (Talos en a besoin, Go2 non).
- **Quadrupède (Go2)** : explicitement différé, Talos d'abord. Repos siblings pertinents pour plus tard : `go2Reachability` (génération Pinocchio, 4 effecteurs, 12 paires + CoM/pied, pas d'antecedent) et `go2Motion` (SL1M/Gurobi pour la combinatoire, Pink/TSID/ndcurves/meshcat pour le reste ; `sl1m.Problem(constraint_paths=...)` est un bon précédent d'API). Piste notée : remplacer SL1M par CASSR dans `go2Motion/test_sl1m.py` comme comparaison directe.

## Convention de travail

Commits unitaires par classe/feature/test ajoutée pendant l'implémentation, messages clairs — pas de gros commit monolithique par phase.

## Stage A — Atteindre l'état actuel (parité fonctionnelle)

Critère de sortie **unique** : séquence de nœuds golden (égalité stricte : surface_id, stance_foot, yaw), positions QP (tolérance, validé via casadi d'abord), perf comparable (temps + nombre d'expansions). Rien du Stage B n'entre dans ce critère.

0. Golden references : séquences de nœuds (`nas_plan` tous chemins, `astar_plan` result_path) + positions QP (CasADi actuel) + temps de référence (minkowski/clipping/intersect/expansions déjà instrumentés dans `AstarSearch`/`Tree`), sur les 3 scénarios du papier pour démarrer.
1. `talosReachability` (packaging, cf. ci-dessus).
2. `core/geometry` + `core/surface` — port dé-globalisé ; remplacer le clip 2D fait main (Sutherland-Hodgman) par `CGAL::intersection` sur `Polygon_2`/`Polygon_with_holes_2` (déjà typedef `Polygon_set_2` dans `types.hpp`, jamais utilisé).
3. `core/reachability` — `ReachabilityModel`, couche 0 seulement.
4. `core/node` (possession explicite) + `core/expansion` (fonction `get_children` unifiée).
5. Port CASSR (`planners/astar_search`) → diff vs golden.
6. Port NAS (`planners/tree_search`) → diff vs golden.
7. `footstep_qp` — interface `QPBackend` (quadprog défaut, proxqp optionnelle isolée, casadi temporaire).
8. Parité QP : new+casadi vs ancien casadi (doit matcher) puis new+quadprog/proxqp vs golden (tolérance). Retrait de casadi une fois validé.
9. `config/` — RobotModel/Scenario/PlannerConfig runtime, migration d'`environments.hpp`.
10. `apps/` — drivers CLI minces.
11. `viz/` découplée — prototype meshcat-cpp + export minimal pour figures.
12. `bindings/` Python (nanobind) — DTO plat de résultat, GIL relâché pendant le solve, couche 0 seulement.
13. `planners/grid_astar_search` — baseline, priorité basse.

## Stage B — Rajouter les tests qu'il manque (après Stage A seulement)

- B1. Comparaison géométrique de la reachability : aire de différence symétrique sur les patches enfants (`CGAL::Polygon_set_2`), comparaison de `P_union` isolé, round-trip H-rep de `convert_polytope_to_half_space_constraint`.
- B2. Cas dégénérés/limites : quasi-coplanaire, aire~0, bord de patch/surface, lacet ±π.
- B3. Golden étendu aux 11 scénarios d'`environments.hpp` (pas juste les 3 du papier).
- B4. Complétude d'énumération NAS : comparer tout l'ensemble de chemins à profondeur minimale (`find_paths_to_root`), pas juste un chemin.
- B5. Correction de l'infaisabilité QP : mêmes chemins précis jugés infaisables ancien/nouveau, pas juste un taux similaire.
- B6. Mémoire longue durée de vie : boucle de `plan()` en process long, surveillance fuite/croissance (nouveau besoin, motivé par l'usage Python live).
- B7. Déterminisme cross-machine des golden references (pertinent vu le portage Linux en cours).
- B8. Loader testé contre les vraies données go2Reachability (N=4 effecteurs, 12 paires + CoM) — sans intégration Go2 complète.

## Nettoyage final (après Stage B)

- Suppression de l'ancien code C++ (implémentation actuelle NAS/CASSR) une fois le nouveau seul à faire foi.
- Suppression des tests devenus inutiles : `test_bench_operations.cpp`/`test_print_paths.cpp` une fois remplacés par un vrai `tests/perf/`, binaires/scripts de capture des golden une fois plus nécessaires, `test_grid_visualization.cpp`/`test_auto_plot.cpp` si non repris.

## Documentation continue (à chaque phase de Stage A et B, jamais une phase à part)

- README par module (`core/*`, `planners/*`, `footstep_qp/`, `config/`, `bindings/`, `viz/`, `talosReachability/`).
- `docs/paper-deltas.md` — journal style ADR des choix d'implémentation non spécifiés par les papiers NAS/CASSR, + section séparée "écarts déjà identifiés à corriger". Déjà pré-rempli pendant l'analyse initiale :
  - **Choix non spécifiés par le papier** : seuil de fusion de nœuds `node_similarity_threshold=0.02m` ; poids ×10 sur l'heuristique EPA/GJK dans le A* ; discrétisation du lacet (±3 pas de 10°) ; `cycle_path_detection` (anti-cycle 2 pas) ; poids `alpha_weight=10` dans l'objectif QP ; dimensions de pied (`foot_length`/`foot_width`) ; workaround `Build_prism` pour hull CGAL sur points coplanaires ; incohérence entre les deux algos de dédoublonnage de nœuds (Tree exact O(n) vs AstarSearch hash quantifié).
  - **Écarts déjà identifiés à corriger** : `total_num_steps` jamais initialisé dans `AstarSearch` (pas de cap de profondeur réel) ; `node_search_method="knn"` documentée mais non implémentée ; dépendance yaml-cpp morte ; clip 2D fait main au lieu de CGAL natif ; contraintes CoM commentées.

## Différé (hors Stage A/B, noté pour plus tard)

- Intégration Go2/go2Motion, comparaison CASSR vs SL1M/MIP sur Go2 réel.
- Extension Python : construire un planner à partir d'un package qui extrait ses propres fichiers (au-delà de la couche 0).
