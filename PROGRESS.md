# Progression — réécriture NAS/CASSR

Dernière mise à jour : 2026-09-17.

But de ce fichier : reprendre exactement où on s'est arrêté si la session s'interrompt — mis à jour à chaque étape franchie, pas seulement en fin de phase. Voir [PLAN.md](PLAN.md) pour le pourquoi/l'architecture, qui ne change pas à chaque commit.

## Où on en est là, maintenant

**Étape courante : phases 0-5 faites. `AstarSearch` porté et validé contre le golden (match exact sur 2 scénarios). Passage à la phase 6 (port NAS, `planners/tree_search`) — mais NAS reste bloqué (fichiers antecedent manquants, voir avertissement plus haut), donc à voir avec l'utilisateur comment traiter cette phase.**
**Prochaine action : décider avec l'utilisateur si on attaque la phase 6 malgré le blocage NAS (ex: porter Tree quand même et le documenter comme non testé), ou si on saute à la phase 7 (`footstep_qp`) en attendant.**

**Note pour la suite (pas encore fait) : les modules `core/*` sont pour l'instant des projets CMake indépendants, pas raccordés entre eux ni au build racine de NAS — chacun teste sa propre pièce isolément. Le raccordement en un seul build cohérent est repoussé à la phase 9/10 (`config/`/`apps/`), pas avant.**

**Décision (2026-09-18)** : le blocage NAS (fichiers antecedent manquants/ambigus, cf. avertissement plus haut) ne concerne que `Tree`/NAS — `AstarSearch`/CASSR ne charge jamais les chemins antecedent. Donc les phases 4+ continuent à être validées contre **CASSR uniquement** tant que le blocage NAS n'est pas résolu ; NAS reste un gap tracké et différé, pas un blocage sur le reste du plan.

Rien n'a encore été créé dans le repo pour le cœur de la réécriture (pas de `core/`, `planners/`, etc.) — seul l'outillage de la phase 0 existe (`tests/golden_capture.cpp`, `docs/paper-deltas.md`).

**⚠️ Bloqueur pour vous au réveil** : NAS/`Tree` ne peut pas tourner du tout actuellement — `constants.hpp` pointe vers `LF_antecedent_CUTZ.obj`/`RF_antecedent_CUTZ.obj`, qui n'existent pas dans `data/constraints_files/` (aucun fichier `LF_antecedent_*` du tout). Je n'ai pas deviné de remplacement. Détail dans `docs/paper-deltas.md`. Les golden CASSR/astar ne sont pas affectées.

**Rappel environnement** : le shell ne garde pas l'activation conda entre mes appels — toujours `source .../conda.sh && conda activate rwa` dans la même commande qu'un `cmake`/build.

## Stage A — parité fonctionnelle

- [x] 0. Golden references — CASSR/astar complet (10/11 scénarios). **NAS incomplet, bloqué** (cf. avertissement ci-dessus) : golden NAS = fichiers d'erreur uniquement pour l'instant, à re-capturer une fois le fichier `.obj` manquant résolu.
  - [x] 0a. Vérifier que le repo actuel compile proprement ici — OK avec `cmake --build build -j2` (jamais plus de 3-4 jobs sur cette machine, RAM très contrainte : ~1.9 Gi libres sur 14 Gi, swap plein — `-j$(nproc)`=12 a été tué par l'OOM killer)
  - [x] 0b. Papier CASSR obtenu — pas de mapping strict à 3 scénarios requis, on capture tous les scénarios d'`environments.hpp` qui marchent (cf. PLAN.md pour les chiffres de référence Table I)
  - [x] 0c. Écrire `golden_capture.cpp` — fait, testé sur NarrowPassage (astar OK, 29 steps = chiffre exact du papier ; nas échoue proprement, cf. bloqueur ci-dessus)
  - [x] 0d. Format JSON dans `tests/golden/` — un fichier par (scénario × planner)
  - [x] 0e. Script de bascule de tous les scénarios d'`environments.hpp` — écrit (`tests/capture_golden_references.sh`), lancé en arrière-plan sur les 11 scénarios
  - [x] 0f. Lancé sur les 11 scénarios — 10/11 astar OK (`TwoFlatSurfaces` ne trouve pas de chemin, gardé tel quel, non débuggé). Sanity check fort : `NarrowPassage`=29 pas et `ThreePathsNAS`=19 pas matchent exactement Table I du papier (narrow passage / local minima avec rotation)
  - [x] 0g. Committé (`071c4be`, `5faaceb`)
- [x] 1. `talosReachability` — créé, testé (build+install+find_package C++ et import Python OK, 14 fichiers), committé (`3671112`)
- [x] 2. `core/geometry` + `core/surface` — les deux portés, testés (9 + 7 tests dirigés, tous passent), committés (`e56dd1f`, `04db66f`). Clip 2D remplacé par CGAL natif, validé sur 4 cas (overlap partiel, aucun, contenu, identique). Modules autonomes (add_subdirectory entre eux), pas encore raccordés au build racine.
- [x] 3. `core/reachability` (couche 0) — `ReachabilityModel` avec manifeste explicite `{chemin, effecteur mobile, effecteur support, direction}`, pas de parsing de noms de fichiers (décision : la sémantique antecedent Talos est ambiguë, voir `docs/paper-deltas.md`). 7 tests, testés contre les vrais assets `talosReachability`. Committé (`992f065`).
- [x] 4. `core/node` + `core/expansion` — `Node` (2 effecteurs, `NodePool` avec node_id séquentiel), `expand_node` unifié (paramétré rotation + direction forward/antecedent, pas de GaitSequencer). Testé contre les vrais assets forward de `talosReachability` (rotation fan-out, alternance, cycle detection). 23 tests passent sur les 5 modules `core/*` via ctest. Committé (`267f3d8`, `4103dc9`, `cad540a`).
- [x] 5. Port CASSR (`planners/astar_search`) — `AstarSearch` dé-globalisé, expansion déléguée à `core/expansion`. **Match exact contre le golden sur 2 scénarios réels** (NarrowPassage 30/30 nœuds, ThreePathsNAS 20/20 nœuds, tous les champs). A révélé et corrigé un crash du clip CGAL natif (voir ci-dessous). Committé (`9d17fcd`, `9172f91`).
- [ ] 6. Port NAS (`planners/tree_search`)
- [ ] 7. `footstep_qp` + `QPBackend`
- [ ] 8. Parité QP (casadi puis quadprog/proxqp)
- [ ] 9. `config/`
- [ ] 10. `apps/`
- [ ] 11. `viz/` découplée
- [ ] 12. `bindings/` Python
- [ ] 13. `planners/grid_astar_search`

## Stage B — tests manquants (après Stage A)

- [ ] B1. Comparaison géométrique reachability
- [ ] B2. Cas dégénérés/limites
- [ ] B3. Golden étendu aux 11 scénarios
- [ ] B4. Complétude d'énumération NAS
- [ ] B5. Correction infaisabilité QP
- [ ] B6. Mémoire longue durée de vie
- [ ] B7. Déterminisme cross-machine
- [ ] B8. Loader vs go2Reachability (N=4)

## Nettoyage final

- [ ] Suppression ancien code C++
- [ ] Suppression tests obsolètes

## Décisions ouvertes / en attente d'arbitrage

_(aucune pour l'instant — tout ce qui a été tranché est dans PLAN.md)_

## Journal

- **2026-09-18** (suite) : phase 5 faite — `planners/astar_search` porté, `AstarSearch` dé-globalisé (config par constructeur), expansion déléguée à `core/expansion`. Validation contre le golden de la phase 0 : match exact sur `NarrowPassage` (30/30 nœuds) et `ThreePathsNAS` (20/20 nœuds), tous les champs (depth, stance_foot, foot_yaw, surface_id). Cette validation a immédiatement payé : elle a révélé un **segfault** du clip CGAL natif introduit en phase 2 (`compute_2d_polygon_intersection`) sur la surface "Passage" du scénario NarrowPassage, qui devient quasi-dégénérée (~2cm de large) une fois rétrécie par la taille du pied — CGAL plante dans son code d'arrangement interne. Revert vers l'implémentation Sutherland-Hodgman originale (prouvée correcte, c'est elle qui a produit le golden), documenté comme tentative abandonnée dans `docs/paper-deltas.md` plutôt que silencieusement retiré. Committé (`9d17fcd`, `9172f91`). Suite à discuter avec l'utilisateur : phase 6 (NAS) reste bloquée par le fichier antecedent manquant.
- **2026-09-18** : scope de phase 4 clarifié avec l'utilisateur (2 effecteurs seulement, pas de GaitSequencer ; rotation paramétrée dans `core/expansion` mais désactivée côté NAS pour l'instant). `core/node` porté : `Node` biped avec `surface_id` par défaut à -1 (corrige à la source le bug trouvé en phase 0) et `NodePool` (deque, pointeurs stables, node_id séquentiel) — 14 tests. `core/expansion` ajouté : `expand_node` unifie enfin `Tree::get_children`/`AstarSearch::get_children`, paramétré par `ReachabilityDirection` et `rotation_enabled` — testé contre les vrais assets forward de `talosReachability` (rotation fan-out 2n+1, alternance de pied, cycle detection bloque/autorise correctement) — 9 tests. Les 5 modules `core/*` passent tous via ctest (23 tests). Phase 4 terminée, Stage A a maintenant tout son socle commun. Passage à la phase 5.
- **2026-09-17** (soir, session autonome, suite) : phase 3 faite — `core/reachability` ajouté : `ReachabilityModel::load()` prend un manifeste explicite (pas de parsing de noms de fichiers) suite à la découverte que le nommage antecedent de Talos est réellement ambigu, pas juste "pas encore mappé" — `RF_antecedent_CUTZ_2.obj` sert à la direction `lf_in_rf` dans l'ancien `constants.hpp`, et le fichier `LF_antecedent` correspondant n'existe même pas. Plutôt que deviner, la couche 0 fait porter cette décision à l'appelant. Testé contre les vrais fichiers de `talosReachability` (pas seulement des données synthétiques) — 7 tests, tous passent. Committé (`992f065`). Passage à la phase 4.
- **2026-09-17** (soir, session autonome, suite) : phase 2 faite — `core/geometry` porté (types + fonctions géométriques, quasi verbatim, aucune ne lisait de global) avec le clip 2D remplacé par `CGAL::intersection` natif sur `Polygon_2` (9 tests dirigés, tous passent : overlap partiel, aucun overlap, contenu, identique, + minkowski_sum/get_centroid). `core/surface` porté (foot_length/foot_width en paramètres de constructeur au lieu de globals, `surface_idx` corrigé en passage par valeur, include `utils.hpp` inutilisé retiré — 7 tests dirigés, tous passent). Deux modules CMake autonomes pour l'instant (surface dépend de geometry via `add_subdirectory`), pas encore raccordés au build racine — décision délibérée pour ne pas risquer de casser le build existant (validé par les golden references) pendant que le reste de `core/` se construit. Committés (`e56dd1f`, `04db66f`). Journal des écarts enrichi de 3 nouvelles entrées trouvées en portant. Passage à la phase 3.
- **2026-09-17** (soir, session autonome, suite) : phase 1 faite — `talosReachability/` créé (structure identique à go2Reachability, 14 `.obj` copiés tels quels depuis `data/constraints_files/`, pas de renommage). Sanity-check complet : build+install dans un préfixe de test, `find_package` C++ et `import` Python résolvent correctement les 14 fichiers. Un piège trouvé et contourné : `.gitignore` a une règle générique `*.obj` (probablement pensée pour des objets compilés Windows) qui bloquait le `git add` normal — les fichiers originaux de `data/constraints_files/` étaient déjà trackés en force pour la même raison, j'ai fait pareil (`git add -f`) pour rester cohérent. Committé (`3671112`). `docs/paper-deltas.md` affiné : confirmé que les 4 fichiers CoM manquent réellement (pas juste du code commenté par prudence). Passage à la phase 2.
- **2026-09-17** (soir, session autonome) : phase 0 menée à bien côté CASSR/astar — `golden_capture.cpp` écrit et testé (`231095a`), `docs/paper-deltas.md` créé et seedé (`626daba`), script de bascule multi-scénarios écrit (`071c4be`), capture lancée sur les 11 scénarios d'`environments.hpp` et committée (`5faaceb`) : 10/11 réussissent côté astar, `TwoFlatSurfaces` ne trouve pas de chemin (gardé tel quel). Deux scénarios confirment le mapping papier par match exact du nombre de pas : `NarrowPassage`=29, `ThreePathsNAS`=19. **NAS/Tree reste bloqué** sur toute la phase 0 : `constants.hpp` référence un fichier `LF_antecedent_CUTZ.obj` inexistant dans `data/constraints_files/` — pas de correction tentée (aucun candidat de remplacement fiable), documenté dans `docs/paper-deltas.md`, à trancher par l'utilisateur. Aussi trouvé et corrigé au passage : `AstarSearch`'s start node avait `surface_id` jamais initialisé (mémoire non déterministe) — neutralisé dans la capture, logué. Passage à la phase 1.
- **2026-09-17** : analyse complète du repo actuel (libs, pipeline de reachability, NAS vs CASSR) + analyse de `go2Reachability`/`go2Motion`. Plan complet discuté et arbitré (QP backends, viz, bindings Python, talosReachability, tests, stages A/B, cleanup). `PLAN.md` et `PROGRESS.md` créés et commités (`db33476`). Phase 0 détaillée en sous-étapes 0a-0g dans PLAN.md. Implémentation pas encore démarrée — prochaine action concrète : 0a.
- **2026-09-17** : papier CASSR fourni par l'utilisateur (attaché en conversation, pas de fichier local). Scope de la capture golden élargi : tous les scénarios d'`environments.hpp` qui marchent, pas juste les 3 du papier — B3 de Stage B absorbé dans la phase 0. Repères Table I du papier ajoutés dans PLAN.md pour sanity-check.
- **2026-09-17** : trouvé `/media/stonneau/data/dev/linux/cassr/` et `/media/stonneau/data/dev/linux/cursor/nas/` (essai antérieur non versionné, ~avril 2026, structure proche du plan actuel — geometry2d/3d, a_star, qp_footsteps avec quadprog vendorisé, viz SVG). **Tranché par l'utilisateur : ignorer complètement, ça ne marchait pas.** Ne pas ré-explorer ces dossiers dans une session future.
