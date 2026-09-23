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
core/node           # Node — stance_foot binaire (2 effecteurs), possession explicite (pool/arena).
                     #   Pas de généralisation N-effecteurs pour l'instant (décision 2026-09-18, voir Décisions clés).
core/reachability   # ReachabilityModel — query(moving_effector, support_effector) -> Polyhedron
                     #   couche 0 SEULEMENT pour l'instant : chemins/répertoire de fichiers .obj explicites en entrée.
                     #   Pas de découverte automatique de package (find_package/import générique) — écarté pour l'instant.
core/expansion      # fonction d'expansion unifiée (le get_children commun à NAS et CASSR), paramétrée par
                     #   rotation (on/off) et direction (forward/antecedent). Alternance L/R codée en dur —
                     #   pas de GaitSequencer (descoped, décision 2026-09-18).
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
tests/
  golden/            # références figées capturées sur l'ancien repo (phase 0)
  perf/               # harness de benchmark (évolution de test_bench_operations.cpp)
```

## Décisions clés

- **QP** : interface `QPBackend` (H,g,A,b → x,status). `quadprog` = défaut/sûr (déjà utilisé dans l'écosystème sibling go2Motion). `ProxQP` = optionnelle, isolée derrière un flag de build (jugée encore trop jeune pour s'y engager). `CasADi` = gardé temporairement, uniquement pour valider la parité de formulation, retiré ensuite.
- **Viz** : VTK découplé du cœur. Cible probable meshcat-cpp pour le dev interactif (convention déjà en usage dans go2Reachability/go2Motion), export minimal séparé pour figures d'article.
- **Reachability loading** : couche 0 uniquement — chemins explicites. Pas de couche de découverte de package pour l'instant (conçue puis explicitement écartée : "j'aime pas trop le sucre pour l'instant"). Plus tard, en Python seulement : possibilité de construire un planner en lui passant un package qui sait extraire ses propres fichiers — forme exacte non spécifiée, à concevoir le moment venu.
- **Bindings Python** : nanobind pressenti par défaut (pas Boost.Python+eigenpy comme Pinocchio/coal — acceptable tant qu'il n'y a pas d'interop directe avec des objets Pinocchio/coal vivants dans le même process).
- **Node ownership** : pool/arena explicite (plus de `new` orphelin comme dans le code actuel) — condition nécessaire pour un usage "live" depuis un process Python longue durée.
- **Portée effecteurs (2026-09-18)** : design à 2 effecteurs seulement (biped) pour l'instant — pas de généralisation N-effecteurs anticipée dans `core/node`/`core/expansion`, pas de `GaitSequencer`. On ne sait pas encore comment une extension future (quadrupède ou autre) se présenterait, donc pas la peine de la deviner — ça simplifie le design. `core/reachability` garde ses clés en `std::string` (déjà fait, pas remis en cause), mais rien d'autre n'anticipe N>2.
- **Rotation dans `core/expansion` (2026-09-18)** : la fonction d'expansion unifiée prend la rotation en paramètre (fan-out sur lacets discrétisés, comme CASSR aujourd'hui), mais l'appel côté NAS/Tree la laisse désactivée pour l'instant — comportement identique à l'actuel (NAS ne fait pas de rotation). La rotation est explicitement voulue pour NAS à terme, d'où le paramètre dès maintenant plutôt qu'un comportement figé par planner.
- **talosReachability** : structure calquée sur `/media/stonneau/data/dev/linux/go2Reachability` (CMakeLists.txt glob+install, `.cmake.in` généré, config Python générée exposant `TALOSREACHABILITY_CONSTRAINTS_DIR`) mais SANS pipeline de génération Pinocchio — juste install des `.obj` déjà dans `data/constraints_files/`. Nested dans NAS pour l'instant, pensé pour extraction en repo sibling plus tard. Garder la distinction forward/antecedent dans le nommage (Talos en a besoin, Go2 non).
- **Quadrupède (Go2)** : explicitement différé, Talos d'abord. Repos siblings pertinents pour plus tard : `go2Reachability` (génération Pinocchio, 4 effecteurs, 12 paires + CoM/pied, pas d'antecedent) et `go2Motion` (SL1M/Gurobi pour la combinatoire, Pink/TSID/ndcurves/meshcat pour le reste ; `sl1m.Problem(constraint_paths=...)` est un bon précédent d'API). Piste notée : remplacer SL1M par CASSR dans `go2Motion/test_sl1m.py` comme comparaison directe.

## Convention de travail

Commits unitaires par classe/feature/test ajoutée pendant l'implémentation, messages clairs — pas de gros commit monolithique par phase.

## Stage A — Atteindre l'état actuel (parité fonctionnelle)

Critère de sortie **unique** : séquence de nœuds golden (égalité stricte : surface_id, stance_foot, yaw), positions QP (tolérance, validé via casadi d'abord), perf comparable (temps + nombre d'expansions). Rien du Stage B n'entre dans ce critère.

**Ordre d'exécution révisé (2026-09-18)** : les numéros de phase ci-dessous reflètent la conception, pas l'ordre d'exécution. Décidé avec l'utilisateur : terminer CASSR complètement (5 → 7 → 8 → 8b) plutôt que de passer à NAS (6) tout de suite, puisque NAS est de toute façon bloqué et que ça donne une implémentation de référence complète et propre à suivre pour NAS plus tard. Séquence réelle : **5 (fait) → 7 → 8 → 8b → 8c (commit propre) → 8d (refactor qualité, en cours) → 9 → 10 → ... → 6 (NAS) en dernier, quand débloqué.**

**Décision (2026-09-18, suite)** : phase 6 (NAS) est mise de côté explicitement pour l'instant, pas seulement "après 8d" — on ne teste pas NAS à ce stade du tout, indépendamment du blocage antecedent. Conséquence directe : le **Nettoyage final reste différé** lui aussi (il suppose l'ancien code entièrement remplacé, NAS inclus) — voir section "Nettoyage final" plus bas. On continue sur 9/10/11/12/13 (CASSR/infra, pas NAS-dépendants) en attendant.

0. Golden references : séquences de nœuds (`nas_plan` tous chemins, `astar_plan` result_path) + positions QP (CasADi actuel) + temps de référence (minkowski/clipping/intersect/expansions déjà instrumentés dans `AstarSearch`/`Tree`), sur les 3 scénarios du papier pour démarrer. **Détail des sous-étapes ci-dessous.**
1. `talosReachability` (packaging, cf. ci-dessus).
2. `core/geometry` + `core/surface` — port dé-globalisé. **Note (2026-09-18) :** un remplacement du clip 2D fait main par `CGAL::intersection` natif a été tenté puis **abandonné** (segfault sur `NarrowPassage`, voir docs/paper-deltas.md) — `compute_2d_polygon_intersection` est revenu à l'implémentation Sutherland-Hodgman d'origine.
3. `core/reachability` — `ReachabilityModel`, couche 0 seulement.
4. `core/node` (possession explicite) + `core/expansion` (fonction `get_children` unifiée).
5. Port CASSR (`planners/astar_search`) → diff vs golden. **FAIT (2026-09-18)** : match exact sur `NarrowPassage` (30/30 nœuds) et `ThreePathsNAS` (20/20 nœuds).
6. Port NAS (`planners/tree_search`) → diff vs golden. **Repoussé en dernier, après 9-13** (voir "Ordre d'exécution révisé" ci-dessus) — mis de côté explicitement par décision utilisateur (2026-09-18), pas seulement bloqué par le fichier antecedent manquant.
7. `footstep_qp` — interface `QPBackend` (quadprog défaut, proxqp optionnelle isolée, casadi temporaire). **En cours.**
8. Parité QP : new+casadi vs ancien casadi (doit matcher) puis new+quadprog/proxqp vs golden (tolérance). Retrait de casadi une fois validé.
8b. Comparaison de performance : CASSR neuf vs golden (temps de recherche, nombre d'expansions) + comparaison des backends QP entre eux (quadprog/proxqp/casadi).
8c. Checkpoint : une fois 7/8/8b satisfaisants, commit propre de tout avant de passer au refactor.
8d. Refactor qualité de code. **Plan détaillé (2026-09-18), à valider avant exécution :**
  - 8d-1. `core/geometry` — remplacer les 8 `std::cerr`/`std::cout` (dont codes ANSI) par une gestion d'erreur cohérente : exception (`invalid_argument`/`runtime_error`) pour les entrées invalides plutôt que print-et-continue. Le fallback silencieux de `calculate_gjk/epa_distance_point_to_patch` (catch-all → distance au centroïde) reste tel quel mais est documenté explicitement comme un choix assumé, pas retiré sans discussion. Zéro `std::cout`/`std::cerr` dans une lib après ce point.
  - 8d-2. Dédupliquer les fixtures de test — les scénarios `NarrowPassage`/`ThreePathsNAS` + la construction du `ReachabilityModel` forward sont recopiés dans 5 fichiers de test différents. Extraire dans un header de fixtures partagé, mettre à jour les 5 call sites.
  - 8d-3. **Point à trancher avec l'utilisateur avant de toucher quoi que ce soit** : `Node` reste-t-il un struct plat à champs spécifiques par planner (assumé en phase 4), ou sépare-t-on maintenant que 3 modules en aval l'utilisent réellement (`core/expansion`, `planners/astar_search`, `footstep_qp`) ? C'est la seule décision de ce refactor qui a un effet d'entraînement sur du code déjà testé — pas une simple passe de nettoyage.
  - 8d-4. Audit const-correctness / passage par valeur vs référence (ex: `ReachabilityModel` copié par valeur dans le constructeur d'`AstarSearch` — voir si `const&`/possession partagée serait plus approprié).
  - 8d-5. Revue de cohérence de nommage/style à travers tous les modules. **FAIT (2026-09-18)** : rien à corriger, le code neuf suit déjà des conventions cohérentes de bout en bout (PascalCase types/enums, snake_case fonctions/variables, suffixe `_` sur les membres privés, commentaires de fermeture de namespace uniformes, ordre d'include uniforme, nommage CMake `nas_<couche>_<module>[_tests]`) — contrairement à `core/geometry` en 8d-1, rien ici n'a été copié verbatim de l'ancien code. Écart réel trouvé en marge (pas du nommage/style, de la doc) : aucun README par module, voir section "Documentation continue" ci-dessous.
  - 8d-6. `ctest` complet après **chaque** sous-étape ci-dessus, pas seulement à la fin — aucune régression tolérée.
  - **Explicitement hors scope** : la duplication de boilerplate CMake entre modules (chaque `CMakeLists.txt` répète la détection CONDA_PREFIX, etc.) — c'est un artefact temporaire du choix de garder chaque module buildable seul (voir décisions clés) qui disparaît naturellement à la phase 9/10 (build racine consolidé), pas la peine de le corriger deux fois.
8e. **Rattrapage README** (ajouté 2026-09-18, suite au constat de 8d-5) — un `README.md` par module déjà porté avant d'en ajouter d'autres en phase 9+ : `core/geometry`, `core/surface`, `core/node`, `core/reachability`, `core/expansion`, `planners/astar_search`, `footstep_qp`, `tests/fixtures`. Contenu attendu par module : rôle/périmètre, API publique principale, dépendances, comment tester isolément (`cmake`/`ctest` depuis le dossier). Court, pas un tutoriel.
9. `config/` — RobotModel/Scenario/PlannerConfig runtime, migration d'`environments.hpp`. **Sous-étapes (2026-09-18) :**
  - 9a. `RobotModel` — struct simple (`foot_length`, `foot_width`, `com_z_height`), valeurs par défaut = celles de l'ancien `constants.hpp` (0.22/0.22/0.75). Pas de chargement fichier pour l'instant (couche 0, comme `core/reachability` — un loader viendra avec `bindings/` phase 12 si besoin). **FAIT (2026-09-18)**.
  - 9b. `Scenario` + registre de scènes — migrer les 11 listes de surfaces brutes d'`environments.hpp` (actuellement des globals `const std::vector<std::vector<Point_3>>`) vers `config::load_scenario(name, robot_model) -> Scenario{name, std::vector<Surface>}`. Distinct de `tests/fixtures` : celui-ci reste le harnais de test (2 scénarios, `AstarSearchConfig` inclus), `config/` est la version "production" qui construit juste les `Surface` à partir du nom, pour toutes les scènes connues. Pas de dédup immédiat entre les deux copies de vertices `NarrowPassage`/`ThreePathsNAS` (fixtures vs config) — à revisiter si ça devient gênant, pas avant. **FAIT (2026-09-18)** : les 11 scènes migrées verbatim dans `config/src/scenario_library.cpp`.
  - 9c. `PlannerConfig` — pas un nouveau type dupliquant les champs : un simple bundle chargé depuis un fichier (JSON, nlohmann déjà dépendance) qui remplit `AstarSearchConfig`/`ExpansionParams`/`FootstepQPConfig` existants, remplaçant la sélection par commentaire dans `constants.hpp`. **FAIT (2026-09-18)** : `load_planner_config(json_path)` — seuls `start_position`/`goal_location` sont requis (pas de défaut sensé indépendant du scénario), tout le reste retombe sur les défauts déjà définis dans les structs existants si absent du JSON. 5 tests (override complet, fallback sur les défauts, 3 cas d'erreur).
  - 9d. Import de scène depuis fichier STL (parser + regroupement des triangles coplanaires en surfaces convexes) plutôt que des listes de points codées en dur — feature à part, plus grosse, après 9a-9c. **Pas encore fait.**
  - 9e. Tests : chaque scénario migré se construit sans exception (11 scènes) ; `NarrowPassage`/`ThreePathsNAS` de `config/` comparés aux vertices d'`environments.hpp` (pas seulement "ça compile"). **FAIT (2026-09-18)** : 6 tests dans `config/tests/test_scenario_library.cpp`, dont vérification de centroïde (calculé avant le rétrécissement par la taille du pied, donc directement comparable aux vertices bruts) sur `NarrowPassage`/`ThreePathsNAS`.
10. `apps/` — drivers CLI minces. **FAIT partiellement (2026-09-18)** : `apps/astar_plan` (CASSR uniquement, `astar_grid_plan`/`nas_plan` viendront avec les phases 13/6). Prend un nom de scène (`config::available_scenarios()`), un `PlannerConfig` JSON, un dossier `talosReachability`, et dump le même JSON que `tests/viz_dump/dump_plan.cpp` (surfaces/chemin/pas) — pas de tests unitaires propres (wrapper fin sur du code déjà testé), testé manuellement sur `NarrowPassage`/`ThreePathsNAS` : tailles de chemin exactes (30/20, soit 29/19 pas) et QP réussi sur les deux.
11. `viz/` découplée — prototype meshcat-cpp + export minimal pour figures. **`tests/viz_dump/dump_plan.cpp` (2026-09-18) est déjà la graine de la moitié "export statique"** — outil permanent (pas supprimé), généralisé à n'importe quel scénario de `tests/fixtures`, dump JSON (surfaces/chemin/pas) consommé par un rendu SVG à la demande. Reste à faire à cette phase : la moitié meshcat-cpp interactive.
12. `bindings/` Python (nanobind) — DTO plat de résultat, GIL relâché pendant le solve, couche 0 seulement.
13. `planners/grid_astar_search` — baseline, priorité basse.

### Phase 0 en détail — Golden references

**Prérequis (à faire avant tout le reste) :**
- 0a. Vérifier que le repo actuel compile proprement sur cette machine — pas encore vérifié dans cette session ; le portage Linux est en cours avec des changements non commités (`CMakeLists.txt`, `geometry.*`, `types.hpp`, `visualizer.hpp`, `astar_search.cpp`, `surface.cpp`, `tree.cpp`). Si ça ne build pas, rien d'autre n'est possible.
- 0b. **Résolu (papier obtenu, 2026-09-17)** — pas besoin de restreindre à exactement 3 scénarios : on capture golden pour **tous les scénarios d'`environments.hpp` qui marchent** (produisent un résultat, succès ou échec propre — pas de crash/hang), on garde ce qui marche, on ignore ce qui ne marche pas sans chercher à le réparer maintenant. Ça absorbe ce qui était prévu en B3 (golden étendu) — supprimé de Stage B, fait ici directement.

  Repères du papier (section VII-D, Table I) pour sanity-check une fois les scénarios identifiés — CASSR, colonnes A*/QP/Total en ms, Nodes, Steps :
  - **Stairs** : sans rotation 4.67±0.17 / 8.51±0.88 / 13.18±0.91, 16 nodes, 15 steps. Avec rotation 10.00±0.15 / 4.74±0.60 / 14.75±0.53, 33 nodes, 11 steps.
  - **Local Minima** : sans rotation 8.08±0.26 / 33.18±3.03 / 41.27±2.85, 42 nodes, 25 steps. Avec rotation 33.66±0.9 / 14.61±0.92 / 48.27±0.70, 92 nodes, 19 steps.
  - **Narrow Passage** : sans rotation **Fail** (pas de solution — attendu, le papier dit que la rotation est nécessaire pour progresser). Avec rotation 60.41±0.46 / 64.72±1.22 / 125.13±1.43, 88 nodes, 29 steps.
  - Correspondance avec `environments.hpp` (noms exacts) — **confirmée empiriquement le 2026-09-17** par nombre de pas exact sur la capture golden : `NarrowPassage` = 29 pas = narrow passage papier (avec rotation, exact). `ThreePathsNAS` = 19 pas = local minima papier (avec rotation, exact). Aucune variante stairs (`Stairs`=5, `LongStairs`=9, `LongLongStairs`=19, `LongStairsComplete`=9, `LongStairsExp`=5) ne matche le "stairs" du papier (15 pas attendu) — pas de scénario stairs équivalent identifié dans `environments.hpp`, non bloquant, on garde ce qu'on a.

**Outil de capture** (temporaire, vit uniquement dans le repo actuel, pas repris dans la réécriture — même statut que `test_bench_operations.cpp`) :
- 0c. Nouvel exécutable `golden_capture.cpp`, ajouté à `CMakeLists.txt` comme les autres binaires de test. Pour le scénario actif (compile-time, comme le reste du repo aujourd'hui) :
  - `AstarSearch::search()` → dump du `result_path` (depth, surface_id, stance_foot, foot_yaw, centroid) + les compteurs déjà accumulés (`total_minkowski_time`, `total_clipping_time`, `total_plane_polytope_intersect_time`, `total_polygon_2d_intersect_time`, `expansion_coount`) + temps de recherche total.
  - `Tree::expand` + `find_nodes_containing_current_stance_foot_brute_force` (`node_search_method` forcé à `"bruteforce"` — pas kdtree/knn, ce dernier n'étant pas implémenté) + `find_paths_to_root` → dump de **tous** les chemins à la profondeur minimale, pas juste un (gratuit à capturer maintenant, rend le golden déjà exploitable pour B4 plus tard sans re-capture) + temps de `Tree::expand`.
  - `FootstepPlanner::plan()` sur le `result_path` CASSR et sur chaque chemin NAS à profondeur minimale → dump des `computed_footsteps`, succès/échec, temps de résolution QP.
  - Inclut le SHA du commit du repo au moment de la capture, dans le fichier produit, pour traçabilité.
- 0d. Format JSON (nlohmann déjà dépendance, déjà le pattern de `footstep_planner.cpp::saveFootsteps`) — un fichier par (scénario × planner) dans `tests/golden/`.

**Exécution :**
- 0e. Petit script shell qui édite les lignes actives de `constants.hpp` (`surf_list`, `current_foot_pos`) pour chacun des scénarios d'`environments.hpp`, rebuild, lance `golden_capture`, sauve le JSON si ça marche (skip proprement sinon, sans debugger le scénario cassé) — rend la capture re-jouable (utile pour B7, déterminisme cross-machine).
- 0f. Lancer, relire les fichiers produits pour un sanity check (chemin plausible, pas de NaN/valeurs aberrantes).
- 0g. Commit des fichiers golden + de l'outil de capture, mise à jour de PROGRESS.md.

## Stage B — Rajouter les tests qu'il manque (après Stage A seulement)

- B1. Comparaison géométrique de la reachability : aire de différence symétrique sur les patches enfants (`CGAL::Polygon_set_2`), comparaison de `P_union` isolé, round-trip H-rep de `convert_polytope_to_half_space_constraint`.
- B2. Cas dégénérés/limites : quasi-coplanaire, aire~0, bord de patch/surface, lacet ±π.
- ~~B3. Golden étendu aux 11 scénarios~~ — absorbé dans la phase 0 (2026-09-17) : on capture déjà tous les scénarios qui marchent dès le départ, pas seulement les 3 du papier.
- B4. Complétude d'énumération NAS : comparer tout l'ensemble de chemins à profondeur minimale (`find_paths_to_root`), pas juste un chemin.
- B5. Correction de l'infaisabilité QP : mêmes chemins précis jugés infaisables ancien/nouveau, pas juste un taux similaire.
- B6. Mémoire longue durée de vie : boucle de `plan()` en process long, surveillance fuite/croissance (nouveau besoin, motivé par l'usage Python live).
- B7. Déterminisme cross-machine des golden references (pertinent vu le portage Linux en cours).
- B8. Loader testé contre les vraies données go2Reachability (N=4 effecteurs, 12 paires + CoM) — sans intégration Go2 complète.

## Nettoyage final (après Stage B)

**Différé tant que la phase 6 (NAS) n'est pas faite** (décision 2026-09-18) : l'ancien code reste la seule implémentation NAS qui marche, il ne peut pas être supprimé avant que `planners/tree_search` existe côté nouveau code.

- Suppression de l'ancien code C++ (implémentation actuelle NAS/CASSR) une fois le nouveau seul à faire foi.
- Suppression des tests devenus inutiles : `test_bench_operations.cpp`/`test_print_paths.cpp` une fois remplacés par un vrai `tests/perf/`, binaires/scripts de capture des golden une fois plus nécessaires, `test_grid_visualization.cpp`/`test_auto_plot.cpp` si non repris.

## Documentation continue (à chaque phase de Stage A et B, jamais une phase à part)

**Constat (2026-09-18, audit 8d-5)** : cet engagement n'a pas été tenu jusqu'ici — aucun des 8 modules déjà portés (`core/geometry`, `core/surface`, `core/node`, `core/reachability`, `core/expansion`, `planners/astar_search`, `footstep_qp`, `tests/fixtures`) n'a de `README.md`. À rattraper (pas fait dans 8d-5, qui portait sur le nommage/style du code, pas la doc — un README par module reste une tâche à part entière, pas glissée dedans sans le signaler).

- README par module (`core/*`, `planners/*`, `footstep_qp/`, `config/`, `bindings/`, `viz/`, `talosReachability/`).
- `docs/paper-deltas.md` — journal style ADR des choix d'implémentation non spécifiés par les papiers NAS/CASSR, + section séparée "écarts déjà identifiés à corriger". Déjà pré-rempli pendant l'analyse initiale :
  - **Choix non spécifiés par le papier** : seuil de fusion de nœuds `node_similarity_threshold=0.02m` ; poids ×10 sur l'heuristique EPA/GJK dans le A* ; discrétisation du lacet (±3 pas de 10°) ; `cycle_path_detection` (anti-cycle 2 pas) ; poids `alpha_weight=10` dans l'objectif QP ; dimensions de pied (`foot_length`/`foot_width`) ; workaround `Build_prism` pour hull CGAL sur points coplanaires ; incohérence entre les deux algos de dédoublonnage de nœuds (Tree exact O(n) vs AstarSearch hash quantifié).
  - **Écarts déjà identifiés à corriger** : `total_num_steps` jamais initialisé dans `AstarSearch` (pas de cap de profondeur réel) ; `node_search_method="knn"` documentée mais non implémentée ; dépendance yaml-cpp morte ; clip 2D fait main au lieu de CGAL natif ; contraintes CoM commentées.

## Différé (hors Stage A/B, noté pour plus tard)

- Intégration Go2/go2Motion, comparaison CASSR vs SL1M/MIP sur Go2 réel.
- Extension Python : construire un planner à partir d'un package qui extrait ses propres fichiers (au-delà de la couche 0).
