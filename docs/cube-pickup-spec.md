# Extension de CASSR : action de manipulation "ramasser un cube"

Contexte : `docs/cube-extension-spec.md` et `docs/cube-implementation-plan.md` spécifient et
implémentent l'action "poser un cube" (`InHand -> PlacedActive`) et "marcher dessus"
(`PlacedActive -> PlacedInactive`), pour un cube porté dès le départ de la recherche
(`AstarSearchConfig::cube_half_extent/cube_height`, un seul cube, `Node::cube_state`/`Node::cube`
scalaires). Ce document spécifie le mécanisme inverse : un cube qui repose déjà dans la scène, à
une position fixe et connue, que la recherche doit ramasser (`None`/`PlacedInactive -> InHand`)
avant de pouvoir le poser/l'utiliser comme précédemment. Une scène peut contenir plusieurs cubes
de ce type.

## 1. Pourquoi aucune primitive géométrique taguée n'est nécessaire

`expand_cube_placement`/`expand_onto_cube` résolvent un problème à deux inconnues couplées : la
position du pied (`x`) ET la position du cube (`c`) sont toutes deux encore des familles de
candidats (des polygones, pas des points) au moment de l'expansion — d'où `minkowski_sum_tagged`
et les primitives 2D/3D taguées qui l'accompagnent (`core/geometry.hpp`), pour ne jamais réapparier
un `x` d'un candidat à un `c` d'un candidat différent et non lié.

Un cube qui repose dans la scène dès le départ n'a **pas** cette propriété : sa position est fixe
et connue à la construction de la recherche. Il n'y a rien à coupler — seul `x` (le patch du pied
courant) reste une famille de candidats face à une cible fixe. C'est exactement le problème que
`AstarSearchConfig::foot_goals`/`goal_satisfied` résolvent déjà (découpe de plan + clip 2D contre
un polytope cible fixe, ou containment direct pour une cible ponctuelle). Le ramassage réutilise
donc cette même géométrie (généralisée en fonctions libres `foot_goal_satisfied`/
`foot_goal_distance`, `include/nas/planners/astar_search.hpp`) et n'a besoin d'aucun
`minkowski_sum`, d'aucune primitive taguée, ni d'aucune requête de reachability.

## 2. Modèle : `SceneCube` et affordance de ramassage

```cpp
struct SceneCube {
    std::array<std::optional<FootGoal>, 2> pickup_affordance;
};
std::vector<SceneCube> scene_cubes;  // dans AstarSearchConfig, PAS dans surfaces_
```

`pickup_affordance` réutilise `FootGoal` telle quelle (même forme que `AstarSearchConfig::
foot_goals` : point ou polytope, plage de yaw optionnelle, par pied). Mêmes 3 modes selon le
nombre de cases remplies :
- **1 case** : ce pied seul doit atteindre la région pour ramasser le cube.
- **2 cases** : posture symétrique fermée, testée sur les deux derniers pas consécutifs (nœud
  courant + son parent), exactement comme le mode "fermeture de posture" de `foot_goals` — mais
  ici comme déclencheur évalué à CHAQUE expansion, pas comme condition de terminaison.

v1 : tous les cubes d'une scène partagent `cube_half_extent`/`cube_height` (pas de géométrie par
cube). `scene_cubes` non vide exige `cube_half_extent > 0` (validé au constructeur) : un cube
ramassé doit pouvoir être posé/enjambé par le mécanisme existant, inchangé.

Inspiration (pas consommation) : le README de g1motion (dépôt sœur) décrit un prototype non
committé, non fini ("grasp affordance polytopes") — une région (x, y, yaw) par pied autour d'une
posture symétrique. C'est la même forme que `FootGoal`, mais aucune donnée g1motion n'est
consommée telle quelle (le prototype n'écrit même pas de fichier `.obj`).

## 3. État par nœud, pas par scène (contrainte : plusieurs branches de recherche en parallèle)

Un cube ramassé sur un chemin ne doit pas redevenir disponible sur ce même chemin, mais la
recherche explore plusieurs branches simultanément : la disponibilité ne peut donc pas être une
mutation partagée de la scène. `Node` porte un `std::vector<bool> cubes_picked_up` (index-aligné
avec `scene_cubes`, vide par défaut), copié tel quel à chaque construction d'enfant dans
`core/expansion.cpp` — exactement comme `pred_surface_ids` porte déjà l'historique des surfaces
visitées par nœud. `expand_cube_pickup` ne mute jamais le vecteur du parent : il copie, positionne
un bit sur la copie, et c'est cette copie qui devient l'état de l'enfant.

Conséquence sur le dédoublonnage (`PatchIndex::Cell`/`CellHash`/`similar`, `astar_search.cpp`) :
deux nœuds identiques par ailleurs (patch/surface/pied/yaw/`cube_state`) mais avec des ensembles
différents de cubes déjà pris ne sont pas interchangeables — `cubes_picked_up` a donc rejoint la
clé de dédup, à coût nul quand `scene_cubes` est vide.

## 4. Affordances hors scène (contrainte : pas des `Surface`)

`scene_cubes`/`pickup_affordance` vivent directement dans `AstarSearchConfig`, jamais dans la
liste `surfaces_` du scénario — même précédent que `FootGoal::region`, qui n'a jamais été injecté
dans les surfaces enregistrées. Une affordance n'a ni `surface_id`, ni participation à
`cycle_path_detection` : ce n'est pas un endroit où poser le pied, seulement une zone de test.

## 5. Le déclencheur : `expand_cube_pickup`

Fonction libre (`include/nas/planners/astar_search.hpp`/`.cpp`), pas une méthode privée
d'`AstarSearch` : elle doit rester directement appelable depuis un test unitaire isolé, comme
`expand_cube_placement`/`expand_onto_cube`. Elle ne vit pas non plus dans `core/expansion.*`, qui
ne doit pas dépendre d'`AstarSearchConfig`/`SceneCube` (mauvais sens de dépendance).

États éligibles : `cube_state ∈ {None, PlacedInactive}`. `PlacedInactive` signifie déjà "mains
libres, simple trace historique qu'un cube a été utilisé" (voir le commentaire de `CubeState` dans
`node.hpp`) — l'inclure permet le ramassage **séquentiel** d'un second cube de la scène après en
avoir fini avec le premier, sans quoi "plusieurs cubes dans une scène" resterait un cube unique
utilisable une seule fois au total.

Pourquoi le mode "2 cases" du ramassage ne casse pas l'invariant d'alternance (contrairement à
`expand_cube_placement`, d'où l'exclusion existante avec `foot_goals` mode 2, voir
`astar_search.hpp`) : aucune action ne produit jamais un enfant `None`/`PlacedInactive` en gardant
le même pied que son propre parent — `expand_cube_placement` ne produit que du `PlacedActive` ;
`expand_onto_cube`/`expand_node` alternent toujours réellement le pied quand ils produisent
`PlacedInactive`/`None`. Un nœud éligible au ramassage a donc structurellement un vrai pas alterné
derrière lui, contrairement au cas que l'exclusion `foot_goals` protège.

Aucune exclusion supplémentaire n'a donc été ajoutée entre `scene_cubes` et `foot_goals` mode 2 :
elle découle transitivement de l'exclusion déjà existante `foot_goals mode 2 x cube_half_extent >
0`, puisque `scene_cubes` non vide exige `cube_half_extent > 0`. Testé explicitement (non-régression
si l'une des deux règles est un jour refactorée séparément).

Nœud de départ : `scene_cubes` non vide ⇒ démarre `cube_state = None` (mains vides, doit ramasser)
au lieu de `InHand` — avec `scene_cubes` vide, l'expression se réduit exactement au comportement
d'avant cette extension.

## 6. Hors scope

- **Reprendre un cube que la recherche elle-même a posé plus tôt sur ce chemin**
  (`PlacedActive -> InHand` pour un cube placé, pas un cube de scène) : question restée ouverte
  dans `docs/cube-extension-spec.md` §8, distincte de ce mécanisme. Ce cas-là nécessiterait le
  couplage tagué (§1 ci-dessus), puisque la position d'un cube placé PAR LA RECHERCHE est
  elle-même une famille de candidats couplée au pied qui l'a posé — contrairement à un cube de
  scène, dont la position est fixe dès le départ.
- Cubes de géométries différentes entre eux (v1 : tous partagent `cube_half_extent`/`cube_height`).
- Modélisation de l'encombrement physique du cube au repos comme obstacle de collision pour
  d'autres pas.
- Plomberie JSON/CLI (`planner_config.cpp`, `apps/astar_plan`, `nas_tools/cube_plan.cpp` côté
  g1motion) — même exclusion que le plan `foot_goals`/`multitarget` précédent.
