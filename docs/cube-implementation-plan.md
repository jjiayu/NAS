# Plan d'implémentation : action "poser un cube"

Doc compagnon de `docs/cube-extension-spec.md` (la spec reçue de l'utilisateur,
commit initial de la branche `cube`). Ce document répond à la spec : d'abord
un avis sur la théorie, puis un plan d'exécution ancré dans le code réel
(lu en détail avant d'écrire ce plan : `core/node.hpp`, `core/geometry.cpp`,
`core/expansion.cpp`, `core/reachability.hpp`, `footstep_qp/footstep_qp.cpp`,
`planners/astar_search.hpp`).

## 1. Avis sur la théorie de la spec

**D'accord avec le diagnostic et la solution.** Le piège du §2 est réel et
bien identifié : c'est une perte de couplage classique (la propriété de
Markov du DP sous-jacent à CASSR — chaque patch ne dépend que de son parent
immédiat — est cassée dès qu'une contrainte future doit se souvenir d'un
paramètre fixé plus d'un pas en arrière). Le contre-exemple 1D est correct
et minimal : pour un `z` fixé, `[z, z+0.4] ∩ [z+0.5, z+0.6] = ∅` toujours,
alors que `T ∩ pas = [0.5, 1.4] ≠ ∅` une fois les sommes de Minkowski
calculées indépendamment. C'est exactement le genre de faux positif qui
rendrait le QP infaisable a posteriori (§2) — sur un chemin que la recherche
croyait valide.

La solution (état joint `(x, c)`, §3) est la bonne réponse standard à ce
type de problème : augmenter l'état pour restaurer la propriété de Markov,
plutôt que de la contourner. Points vérifiés en particulier :

- **Dimension 4, pas 6** (§3.1) : correct, `x` et `c` vivent chacun sur un
  plan (2D intrinsèque chacun).
- **La projection §3.4 est bien triviale en V-rep** : projeter un polytope
  V-rep sur un sous-ensemble de coordonnées = jeter les coordonnées en trop
  sur chaque sommet + reprendre l'enveloppe convexe (l'image d'une
  application linéaire d'un polytope est l'enveloppe convexe des images de
  ses sommets). C'est le sens inverse (H-rep → projection, qui demanderait
  une élimination de Fourier-Motzkin) qui serait dur — et ce n'est jamais
  ce qu'il faut faire ici. Bon choix de représentation.
- **Le QP reste linéaire** (§5.5) : confirmé, `x_i - c - h·n ∈ carré_cube`
  est linéaire en `(x_i, c)`.
- **Le coût heuristique ne progresse jamais en posant le cube** (§5.4) :
  confirmé et à surveiller (voir §5 ci-dessous), pas un défaut de la
  théorie, juste un vrai coût de perf à mesurer.
- **La remarque sur la marge α (§5.5)** : vérifiée dans le code actuel
  (`src/footstep_qp/footstep_qp.cpp`). Bonne nouvelle partielle : la marge
  `α` n'est aujourd'hui appliquée qu'aux lignes de bord du polygone de
  surface (`generate_surface_constraint`, lignes `r >= 1`), jamais aux
  lignes du polytope de reachability (`hrep`, aucune marge du tout). Donc
  le risque concret aujourd'hui est plus étroit que "toutes les lignes de
  `B_i`" : il porte sur une surface étroite des deux côtés (typiquement
  `NarrowPassage`, dont `core/geometry.cpp` documente déjà le cas limite à
  ~2cm). Reste une vraie question à trancher séparément (voir §6),
  indépendante du cube.

**Une subtilité non explicitée à régler dans l'implémentation** : le yaw
d'alignement du cube (`Q` dans `c ∈ z ⊕ Q·K_cube`, §3.1) doit être mémorisé
au moment de la pose et rester figé pour `carré_cube` (§3.3), même si le
pied continue à tourner pour les pas suivants qui ne touchent pas le cube.
Le nœud doit donc porter deux yaws distincts une fois le cube actif : celui,
courant, du pied (`foot_yaw`, déjà là) et celui, figé, de la pose du cube.

## 2. Constat d'architecture clé : pas besoin de géométrie n-dimensionnelle

C'est le point qui change le profil de risque du plan. Le pipeline actuel
(`expand_node`, `core/expansion.cpp`) est : somme de Minkowski 3D +
`CGAL::convex_hull_3` (`minkowski_sum`) → coupe par le plan de la surface
candidate (`compute_polytope_plane_intersection`) → projection 2D → hull
2D → clip Sutherland-Hodgman fait maison contre le polygone de la surface
(`compute_2d_polygon_intersection`) → hull/nettoyage final.

Vérifié étape par étape : **chaque étape est soit une sélection de
sous-ensemble parmi les points d'entrée (hull = sous-ensemble des sommets
donnés, jamais de nouveau point synthétisé), soit une interpolation
affine explicite déjà paramétrée par un `t`** (`compute_polytope_plane_intersection`
via `CGAL::intersection(plane, segment)` ; `compute_2d_polygon_intersection`
via `push_crossing`, qui calcule `t` explicitement). Dans les deux cas, on
peut faire porter à chaque point d'entrée une charge utile (`payload`, ici
la coordonnée `c` ou `z` selon l'étape) et la faire suivre : recopiée telle
quelle à travers une sélection de sous-ensemble, interpolée avec le même
`t` à travers une interpolation affine.

Conséquence concrète : **`J` n'a jamais besoin d'être manipulé comme un
vrai polytope 4D** (pas de hull ni d'intersection de demi-espaces en
dimension 4, donc pas besoin de CGAL `Convex_hull_d`/`Epick_d` ni de qhull).
Il suffit d'ajouter, à côté de chaque primitive géométrique existante, une
variante "taguée" qui fait exactement le même calcul géométrique sur la
coordonnée qui pilote réellement le hull/clip à cette étape (`x` pour un
pas normal, `c` pour la pose, `x - c` pour un pas sur le cube) tout en
faisant suivre l'autre comme simple charge utile. C'est un ajout borné et
mécanique aux fonctions de `core/geometry.cpp`, pas une nouvelle brique de
géométrie computationnelle. Ça vaut aussi la peine d'être dit dans l'autre
sens : la tentation "on réutilise `expand_node` tel quel pour poser le
cube" (en traitant le cube comme un effecteur de plus) **reproduirait
exactement l'approche naïve du §2** — ce n'est pas une réutilisation
gratuite possible, il faut bien les primitives taguées.

## 3. Plan d'exécution (étapes commitées séparément, chacune vérifiée)

**Étape 1 — Données et config, sans toucher à la recherche.**
Un premier polytope `K_cube` conservateur construit à la main (boîte/prisme,
même niveau d'approximation que ce que le papier fait déjà pour les pieds à
l'origine), chargé via `ReachabilityModel::load` existant sans aucune
modification de cette classe — elle généralise déjà à un effecteur nommé
`"Cube"` (`ReachabilityEntry{path, moving_effector="Cube", support_effector,
direction}`). Nouveau fichier `talosReachability/data/reachability_constraints/
Cube_constraints_in_*F.obj`, plus une petite `struct CubeConfig` (demi-emprise,
épaisseur `h`) dans `config/`. Vérif : test unitaire qui charge le fichier et
vérifie un polytope non dégénéré, comme les tests reachability existants.

**Étape 2 — Primitives géométriques taguées, géométrie pure.**
Ajoute, à côté des fonctions existantes dans `core/geometry.*` (sans changer
leurs signatures, pour ne pas perturber tous les appelants actuels) : des
variantes qui font porter une charge utile `Point_2`/`Point_3` à travers
`minkowski_sum`, `compute_polytope_plane_intersection`,
`compute_2d_polygon_intersection`, en suivant exactement le raisonnement du
§2 ci-dessus. Premier test : **porter tel quel le contre-exemple 1D de la
spec** (ou son équivalent 2D) comme test unitaire — assertion que l'approche
naïve (deux sommes de Minkowski indépendantes puis intersection) donne un
faux positif non vide, et que la version taguée/jointe donne correctement
vide. Vérif : uniquement des tests unitaires nouveaux, rien d'autre dans le
code touché — revue et merge isolés du reste.

**Étape 3 — `Node` + action de pose (§3.1).**
Nouveau champ sur `Node`, un seul struct optionnel plutôt que plusieurs
champs épars :
```cpp
struct CubePlacement {
    std::vector<Point_3> vertices_3d;  // c_i, alignés index à index avec patch_vertices (x_i)
    Polygon_2 polygon_2d;
    Transformation to_2d, to_3d;       // repère de la surface du cube, peut différer de celle du pied
    int surface_id = -1;
    double placement_yaw = 0.0;        // figé à la pose, distinct de foot_yaw qui continue à évoluer
};
enum class CubeState { None, InHand, PlacedActive, PlacedInactive };
CubeState cube_state = CubeState::None;
std::optional<CubePlacement> cube;
```
Nouvelle fonction `expand_cube_placement()` (soeur de `expand_node`, dans
`core/expansion.*`), construite avec les primitives taguées de l'étape 2 —
candidate seulement si `cube_state == InHand`. Vérif : scénario plat simple,
comparaison du polygone joint obtenu à un calcul de référence à la main, plus
un invariant systématique (`∀i, c_i - x_i ∈ K_cube, c_i ∈ S̃_j`).

**Étape 4 — Pas normal et pas sur le cube (§3.2, §3.3, §3.4).**
`expand_node` (ou une branche dédiée) transporte `cube` sans le modifier
quand le parent est `PlacedActive` (3.2, primitives taguées avec `c` comme
charge). Nouvelle branche "pas sur le cube", candidate seulement vers la
surface du cube du parent, avec la coupe additionnelle sur `x' - c`
(3.3, variante de `compute_2d_polygon_intersection` taguée qui classe sur
`x - c` mais fait suivre `(x, c)`). Transition `PlacedActive →
PlacedInactive` (puis chute de `cube`) en réutilisant la logique
"surface quittée" déjà là pour `cycle_path_detection` (3.4). Vérif :
reprise du contre-exemple de l'étape 2 mais posé "en vrai" dans une mini
scène à 2 pas (pose puis pas impossible) — le nœud correspondant doit être
absent des enfants, pas juste un polytope vide en théorie.

**Étape 5 — Dédoublonnage et heuristique (§5.3, §5.4).**
`PatchIndex` (utilisé par `AstarSearch`, `astar_search.hpp` /
`node_similarity_threshold`) doit aussi discriminer sur `cube_state` et, si
actif, la proximité du patch `c` — pas seulement `(surface_id, stance_foot,
yaw_bin, patch x)` comme aujourd'hui. `heuristic()` reste sur la projection
`x` (déjà vrai si elle ne lit que `patch_vertices`) — ajouter un test qui
vérifie explicitement qu'une pose de cube n'améliore jamais `h_score`, pour
que ce comportement (exploré seulement à `f` égal une fois les options
moins chères épuisées, potentiellement lent) soit surveillé et documenté,
pas juste subi.

**Étape 6 — QP (§5.5) + scénario de bout en bout.**
Variable `c` (3 composantes) par cube posé-et-utilisé sur le chemin retenu
dans `footstep_qp.cpp`, avec les deux familles de contraintes du §5.5,
construites sur le même modèle que le bloc `hrep` existant (lignes ~90-101)
et `generate_surface_constraint` (déjà lu, déjà réutilisable tel quel pour
`c ∈ S̃_j`). **Ne pas étendre la marge `α` aux nouvelles contraintes sans
décision explicite** — laisser α à 0 sur les lignes cube pour l'instant,
trancher séparément (§6 des questions ouvertes). Nouveau scénario golden
avec un passage prouvé infaisable sans cube (vérifié en confirmant d'abord
"pas de chemin" avec le planner actuel) et faisable avec, jusqu'au QP
résolu et faisable. Vérif finale : suite golden existante toujours 100%
verte + ce nouveau scénario dedans.

**Étape 7 — Docs.** Entrée dans `docs/paper-deltas.md` (extension, pas
correction du papier), pointant vers `cube-extension-spec.md` et ce plan,
avec le contre-exemple et le raisonnement "pourquoi la somme de Minkowski
naïve casse le couplage" conservés pour la postérité.

## 4. Questions ouvertes à trancher avant (ou au fil de) l'implémentation

Reprises du §8 de la spec, plus celles trouvées en lisant le code :

- **`K_cube` réel** : commencer par une boîte conservatrice à la main
  (étape 1), explicitement documentée comme approximation — construire une
  vraie reachability double-appui depuis la cinématique du robot est un
  travail séparé, hors scope v1.
- **Reprise d'un cube déjà posé** (`PlacedActive → InHand`) : spec la laisse
  ouverte. Proposition : **exclue du v1** — toute la mécanique ci-dessus
  suppose qu'un cube posé-puis-quitté est définitivement inactif. Simplifie
  beaucoup ; à confirmer.
- **Cube qui obstrue la surface qu'il recouvre** : explicitement mis de
  côté par la spec. Proposition : au moins un commentaire "limitation
  connue" explicite dans le code v1, pas une omission silencieuse.
  L'obstruction et la reprise se recoupent : traiter les deux plus tard.
  ensemble si besoin.
- **Un seul cube actif à la fois** : confirmé par le §6 de la spec
  (complexité `2+2n`). `cube_state`/`cube` en scalaire sur `Node` (pas une
  collection) verrouille cette limite au niveau du type — à confirmer que
  c'est bien voulu pour le v1.
- **Comment le robot a le cube "en main" au départ** : hors scope, le cube
  démarre `InHand` — à confirmer que le v1 ne modélise pas d'action de
  ramassage.

## 5. Ce qui ne change pas

Le reste de CASSR (recherche normale sans cube, dédoublonnage existant,
QP existant) : zéro changement de comportement quand aucun cube n'est en
jeu — toutes les nouvelles branches sont conditionnées à
`cube_state != None`. `ReachabilityModel`, `generate_surface_constraint`,
`minkowski_sum`/`compute_2d_polygon_intersection` existants : signatures
inchangées, nouvelles variantes à côté.

## 6. Item séparé trouvé en cours de lecture (pas le cube, à vérifier un jour)

La marge `α` (Eq. 6) n'est appliquée aujourd'hui qu'aux lignes de bord de
`generate_surface_constraint`, jamais à `hrep` (le polytope de
reachability) — donc pas le risque "toutes les lignes de `B_i`" tel quel,
mais toujours le risque plus étroit qu'une surface étroite des deux côtés
(type `NarrowPassage`, déjà documentée comme cas limite ailleurs dans le
code) devienne infaisable dès que `α` dépasse sa demi-largeur. Indépendant
du cube, pas bloquant, mentionné ici pour ne pas le perdre.
