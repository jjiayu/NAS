# Évaluation de l'API et du fonctionnement

Avis d'ingénierie sur l'état actuel de `AstarSearchConfig`/`AstarSearch`/`solve_footstep_qp` et de
leur usage (CLI, JSON, Python), après le travail de cette session (unification de `foot_goals`) —
pas un sondage neutre, un jugement avec des recommandations concrètes là où il y en a. Structuré en
"ce qui marche bien" / "points de friction" / "à trancher si on itère encore". Section cube à la fin
de ce fichier (`docs/cube-extension-mechanism.md` d'abord si pas déjà lu).

## Ce qui marche bien

- **Un seul mécanisme de but.** Avant cette session, `goal_location`/`goal_surface_id`/
  `goal_stance_foot`/`goal_yaw_target` coexistaient avec `foot_goals`, mutuellement exclusifs,
  dupliquant la même logique à 4 endroits de `astar_search.cpp` (heuristique de départ, `goal_point()`,
  terminaison, heuristique des enfants). Unifié sur `foot_goals`, chacun de ces 4 endroits est
  descendu de 3 cas à 2 — moins de code, moins de chemins à tester. Bénéfice annexe : l'exclusion
  cube s'est resserrée d'elle-même (1 slot + cube est maintenant légal, seul "2 slots" reste exclu),
  sans qu'on ait eu à y penser explicitement — un signe que l'unification était la bonne simplification,
  pas juste un renommage.
- **Validation à la construction, pas au premier usage.** Chaque incohérence de config (indice de
  surface hors limites, `yaw_range` sans rotation, `goal_yaw_weight` sans cible claire, 2 slots +
  cube) lève une exception claire dans le constructeur d'`AstarSearch`, avant qu'une seule expansion
  n'ait lieu — pas un plantage géométrique obscur 50 nœuds plus tard.
- **Pas de config dupliquée entre couches.** `PlannerConfig` (JSON) ne fait que peupler
  `AstarSearchConfig`/`FootstepQPConfig` — pas de 3ᵉ struct à maintenir en synchro (voir le
  commentaire en tête de `planner_config.hpp`, une décision de conception assumée et qui tient).
- **`FootGoal` réutilisé tel quel par le ramassage de cube** (`SceneCube::pickup_affordance`, même
  forme région+yaw_range) — le nouveau variant `surface` (indice) et le mécanisme de résolution
  `offset`/`polygon_2d` en bénéficient gratuitement, sans code cube-spécifique à écrire.
- **Le QP ne force jamais le dernier pas vers un point pour un but `surface`/`polytope`** — quand
  `qp_goal()` renvoie `nullopt`, le dernier pas reste libre sur son patch, avec la même contrainte que
  les pas intermédiaires (voir `docs/footstep-planning-mechanism.md`, section QP). Sur un terrain très
  ouvert (`Flat`, patchs qui restent grands tout le long), ça peut faire choisir au QP de ne quasiment
  pas bouger (son objectif ne minimise que la longueur des foulées — "ne pas bouger" est optimal dès
  que c'est faisable). **Confirmé comme le comportement voulu, pas un gap à combler** : la recherche
  A* a fait son travail (trouver une séquence de patchs valide vers la région demandée), le QP fait le
  sien (satisfaire les contraintes de la manière la moins coûteuse) — ajouter une incitation
  artificielle à se déplacer changerait ce que le QP optimise réellement pour tout appelant
  `goal_surface`/`polytope` existant, pas juste ce cas limite.
- **Construction directe en Python, sans fichier JSON** (`nas_bindings.PlannerConfig`/`FootGoal`,
  `plan_with_config()`) — point de friction de la session précédente, résolu à la demande de
  l'utilisateur. Même astuce que côté C++/JSON : les deux façades (fichier, Python) convergent sur
  le même `nas::config::PlannerConfig` interne (`planner_config_from_py()` dans
  `bindings/src/module.cpp`), aucune 2ᵉ logique de résolution de but écrite à part. `FootGoal` va
  plus loin que le JSON sur un point : ses 5 usines statiques rendent une région invalide
  **non représentable** (pas de champ "region" brut à valider), alors que le parseur JSON doit se
  défendre activement contre du texte non typé (`parse_foot_goal_slot`, "exactement une clé").
  Trouvaille en marge, sans rapport avec `foot_goals` : ce travail a révélé que `ctest -R
  nas_bindings` testait silencieusement une installation pip `-e .` obsolète au lieu du build local
  fraîchement compilé (un `sys.meta_path` d'installation éditable prend le pas sur `sys.path.insert`,
  sans erreur ni avertissement) — corrigé dans `bindings/tests/test_bindings.py`/`test_memory.py`
  (chargement par chemin de fichier explicite). Deux assertions de `test_bindings.py` étaient déjà
  fausses avant même ce correctif (15 scénarios au lieu de 17, `NarrowPassage` à 30 nœuds au lieu de
  34 depuis le fix PatchIndex du 2026-09-27) — jamais détecté car `NAS_BUILD_BINDINGS` est `OFF` par
  défaut et personne n'avait relancé `ctest -R nas_bindings` depuis.

## Points de friction

- **Deux niveaux pour la forme de région, pas évident au premier abord.** `FootGoal::region` n'a que
  3 variants runtime (`Point_3`/`vector<Point_3>`/`int`) ; `offset` et `surface`+`polygon_2d` sont un
  4ᵉ et 5ᵉ cas qui n'existent **qu'en JSON**, résolus en un des 3 variants réels par
  `config::resolve_goal()` avant que `AstarSearch` ne les voie. Cohérent avec le précédent
  `goal_offset` d'avant cette session, mais quelqu'un qui construit un `AstarSearchConfig` en C++
  direct (comme tous les tests le font) n'a PAS accès à `offset`/`polygon_2d` — seulement au JSON. Pas
  documenté ailleurs que dans le commentaire de `PendingFootGoalRegion` et ce tutoriel.
- **`goal_yaw_weight` reste un champ global unique**, pas par slot — décision prise cette session pour
  rester au périmètre demandé (généraliser par slot n'a jamais été testé ni demandé). Ça veut dire
  qu'en mode "2 slots", il n'y a aujourd'hui aucun moyen de biaiser doucement l'orientation de CHAQUE
  pied indépendamment pendant le chemin — seulement la contrainte dure (`yaw_range`) par slot. Pas un
  problème tant que personne n'en a besoin ; documenté comme limitation assumée dans le commentaire du
  champ, pas caché.
- **`AstarSearch`/`GridAstarSearch` ont chacun leur propre `..Config` avec `goal_location`/
  `goal_stance_foot`** (`grid_astar_search.hpp`) — l'unification de cette session ne les touche pas
  (planners distincts, décision explicite). Quelqu'un qui connaît `foot_goals` et passe au baseline
  grid retrouvera l'ancien style à deux champs séparés, sans prévenance particulière au-delà de ce
  document.
## À trancher si on itère encore

- Exposer `scene_cubes`/`pickup_affordance`/`cube_half_extent` en JSON **et** en Python direct (voir
  `docs/cube-extension-mechanism.md`, section "Ce qui manque", et le point suivant) — mécanique mais
  un vrai morceau de travail (même style que `foot_goals`/`PlannerConfig`, avec le même genre de
  sucre potentiel pour la position du cube). Explicitement reporté par l'utilisateur.
- Généraliser `goal_yaw_weight` par slot, seulement si un vrai cas d'usage à 2 pieds + biais
  d'orientation indépendant se présente — pas avant, pour ne pas ajouter une capacité jamais exercée.

## Extension cube

- **Bonnes frontières.** `expand_cube_pickup` vit délibérément dans `planners/astar_search.cpp`, pas
  `core/expansion.*` (`SceneCube` est un concept de planificateur, pas de `core/` — voir
  `cube-pickup-spec.md` §1). `pickup_affordance` réutilise `FootGoal` tel quel, sans code dupliqué.
  L'état joint `(x, c)` (`CubeState`/`Node::cube`) est un ajout propre : un chemin séparé dans
  `expand_node` pour le transport (`cube_state == PlacedActive`), donc zéro risque pour tout appelant
  qui n'active jamais l'extension — déjà vérifié par la suite existante avant cette session.
- **Pas de plomberie JSON/CLI** — `cube_half_extent`/`cube_height`/`scene_cubes`/`pickup_affordance`
  se configurent seulement en C++ direct aujourd'hui, contrairement à tout le reste
  d'`AstarSearchConfig`. Écart d'ergonomie net avec `foot_goals` (qui, lui, vient d'être exposé en
  JSON cette session) : quelqu'un qui veut explorer un scénario cube sans écrire de C++ ne peut pas.
  Mécanique à ajouter (même style de schéma que `foot_goals`, avec potentiellement le même genre de
  sucre pour une position de cube au repos), pas commencé.
- ~~Un piège subtil : la pose du cube réutilisait l'érosion du pied et ignorait `cube_half_extent`~~
  **Résolu** : le cube suit la même logique de marge que les surfaces (`docs/cube-extension-mechanism.md`,
  « Marge et taille du cube »). La pose érode la surface brute de la demi-diagonale du cube, le
  dessus praticable est érodé de `inner_margin`, et `cube_half_extent <= inner_margin` est refusé au
  constructeur.
- **v1 = un seul cube en jeu, usage unique**, assumé et bien documenté partout (spec §6, commentaires
  de `CubeState`) — pas une surprise si `docs/cube-extension-mechanism.md` est lu avant d'essayer
  d'enchaîner deux poses ou de reprendre un cube déjà posé.
