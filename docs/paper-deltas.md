# Journal des écarts — implémentation vs papiers NAS/CASSR

Format court style ADR : quoi, où, pourquoi (si connu). Alimenté en continu à chaque phase de [PLAN.md](../PLAN.md), pas seulement en fin de projet.

## Choix d'implémentation non spécifiés par les papiers

- **Seuil de fusion de nœuds** `node_similarity_threshold = 0.02` m (centroïde + périmètre) — [constants.hpp](../include/constants.hpp). Le papier CASSR (`nodeAlreadyExpanded()`, Sec. V-B.3) dit juste "set empirically to 2 cm", donc la valeur est confirmée par le papier, mais reste un choix empirique à documenter comme tel.
- **Poids ×10 sur l'heuristique EPA/GJK** dans le A* (`tentative_h_score = 10.0*calculate_epa_distance_point_to_patch(...)` dans [astar_search.cpp](../src/astar_search.cpp)) — le papier dit "weighted A*, no admissibility guarantee" mais ne précise pas la valeur du poids. Non confirmé par le papier, à traiter comme un paramètre de tuning propre à cette implémentation.
- **Discrétisation du lacet** : ±3 pas de 10° autour du lacet parent (`foot_yaw_angle_discretization_num=3`, `foot_yaw_angle_increment=10°`) — le papier dit utiliser des incréments de 10° de −30° à 30°, donc **confirmé et cohérent** avec l'implémentation (Sec. VII-A : "10° increments from −30° to 30°").
- **`cycle_path_detection`** (interdiction de revisiter la même surface 2 pas plus tard) — correspond à `hasContactSurfaceNotBeenLeft()` du papier (Sec. V-B.4), confirmé conceptuellement, mais l'implémentation exacte (comparaison sur l'historique `pred_surface_ids`) est un détail non spécifié par le papier.
- **Poids `alpha_weight=10` dans l'objectif QP** entre régularisation de foulée et marge de robustesse — le papier confirme explicitement "we empirically scale α by a factor 10" (Sec. VI, Eq. 6) — **confirmé par le papier**.
- **Dimensions de pied** (`foot_length=0.22`, `foot_width=0.22` dans `constants.hpp`, mais le commentaire à côté dit `//0.22` vs `0.12` — incohérence à vérifier, voir section "écarts à corriger") — paramètres robot (Talos), pas donnés par le papier.
- **Workaround `Build_prism`/`convex_hull_3_from_coplanar_points`** ([geometry.cpp](../src/geometry.cpp)) — pur artefact d'implémentation CGAL (`convex_hull_3` plante sur points coplanaires), invisible dans le papier mais nécessaire à la correction.
- **Deux algorithmes différents de dédoublonnage de nœuds** entre NAS (`Tree::check_node_similarity`, comparaison exacte O(n) sur la couche courante) et CASSR (`NodeHash`/`NodeEqual`, hash quantifié) — le papier ne distingue pas les deux algorithmes, ils implémentent le même concept (`nodeAlreadyExpanded`) différemment selon le contexte (DAG multi-parents pour NAS vs open-set à dédoublonner pour CASSR). Incohérence à trancher explicitement dans la réécriture plutôt que reproduite par accident.

## Écarts déjà identifiés à corriger

- **`AstarSearch`'s start node a `surface_id` jamais initialisé** dans le constructeur ([astar_search.cpp](../src/astar_search.cpp)) — contrairement à `Tree`'s root qui le fait explicitement (`root_ptr->surface_id = this->surfaces.back().surface_id;`). Lecture de mémoire non initialisée, non reproductible. Trouvé le 2026-09-17 en écrivant `golden_capture.cpp` — la capture golden neutralise ce champ (`null`) pour le nœud de départ plutôt que d'enregistrer du bruit.
- **`total_num_steps` jamais initialisé dans `AstarSearch`** — pas de cap de profondeur réel côté CASSR, contrairement à `Tree::num_steps` qui borne réellement le BFS.
- **`node_search_method="knn"`** documentée (README, `constants.hpp`) mais non implémentée (branche vide dans `nas_plan.cpp`).
- **Dépendance yaml-cpp / `config/*.yaml` morte** — jamais branchée malgré le lien CMake et l'include dans `nas_plan.cpp`.
- **Clip 2D fait main** (Sutherland-Hodgman dans `compute_2d_polygon_intersection`) au lieu des opérations booléennes natives de CGAL (`Polygon_set_2` typedef dans `types.hpp` mais jamais utilisé).
- **Contraintes CoM commentées/désactivées** dans le QP (`footstep_planner.cpp`) — les fichiers `.obj` sont référencés dans `constants.hpp` mais le bloc de contraintes CasADi correspondant est commenté.
- **BLOQUANT — fichier de contrainte manquant pour NAS** (trouvé le 2026-09-17, phase 0) : `constants.hpp` référence `rf_in_lf_path_antecedent = ".../LF_antecedent_CUTZ.obj"` et `lf_in_rf_path_antecedent = ".../RF_antecedent_CUTZ.obj"`, mais **aucun des deux fichiers n'existe** dans `data/constraints_files/` (casse exacte non trouvée ; il existe `RF_antecedent_CUTZ_2.obj`, `RF_antecedent_cutZ.obj`, `RF_antecedent.obj`, mais **aucun `LF_antecedent_*` du tout**). Conséquence : `Tree` (et donc `nas_plan`) plante à la construction (`std::runtime_error`, "Failed to load polyhedron") — **indépendamment du scénario choisi**. Golden capture le gère proprement (exception catchée, JSON d'erreur écrit) mais ne peut pas produire de référence NAS tant que ce n'est pas résolu. Je n'ai pas deviné de fichier de remplacement (aucun candidat évident, risque de fausser silencieusement la référence) — **à trancher par l'utilisateur**. Bloque les golden references NAS de la phase 0 ; les golden CASSR/astar ne sont pas affectées (elles utilisent les polytopes *forward*, qui existent).

## À vérifier

- Incohérence `foot_width` dans `constants.hpp` : valeur active `0.22`, commentaire à côté dit `0.12` — laquelle est correcte ?
