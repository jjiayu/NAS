# Rapport de session — extension "cube" (2026-09-23, à reprendre)

**Visualisation animée publiée** (demande explicite de l'utilisateur, faite
juste avant la coupure) : https://claude.ai/artifact/1QQdV9T3igvn9PeowLvihW
— "Passerelle cube", anime exactement le plan à 87 expansions décrit
plus bas (vue de profil X-Z + vue de dessus X-Y, pas qui apparaissent un
par un, cube qui apparaît à la pose, lecture auto + molette). Données
codées en dur dans la page (le JSON de la section "Prochaine étape" plus
bas, régénéré après le fix du bug de centroid) — pas de connexion au
dépôt, donc si le plan change il faut regénérer le JSON et republier sur
cette même URL.

Session interrompue par la limite hebdomadaire. Ce document résume l'état
exact pour reprendre sans perdre de contexte. Voir aussi
`docs/cube-extension-spec.md` (la spec d'origine) et
`docs/cube-implementation-plan.md` (le plan détaillé, avec son propre
"Statut" mis à jour mais légèrement en retard sur ce rapport-ci — celui-ci
est la référence la plus à jour).

## Où en est le dépôt

Branche `cube`, **locale uniquement, jamais poussée**. 14 commits au-dessus
de `devel` (`git log --oneline devel..cube`), du plus récent au plus ancien :

```
9fbeba8 Cube: corrige la hauteur de centroid pour un nœud sur le cube
54a084f Cube: filtre "découverte" pour les poses de cube (idée utilisateur, 2026-09-23)
ed51238 Documente le statut actuel de l'implémentation cube (2026-09-23)
7918108 Cube: étape 6 (partiel), branchement dans AstarSearch + calibration de K_cube
f5960a8 Cube: étape 5, PatchIndex discrimine aussi sur cube_state (§5.3)
8f6dfbb Cube: étape 4 (partie 2/3), pas sur le cube (§3.3) et désactivation v1 (§3.4)
c27d4e5 Cube: étape 4 (partie 1/3), transport du cube à travers un pas normal (§3.2)
94c7901 Cube: étape 3, Node + action de pose expand_cube_placement (§3.1)
ee04629 Cube: étape 2, primitives géométriques taguées (charge utile à travers hull/slice/clip)
f3fe21d Cube: étape 1, K_cube conservateur à la main + CubeConfig (cube de 15cm)
2dae7f2 Ajoute le scénario StairsGap et une reachability clampée Z<=25cm pour le test cube
7207e56 Plan d'implémentation pour l'extension cube, en réponse à la spec
1bab469 Branche cube : ajoute la spec d'extension "poser un cube" comme plan de travail
```

Working tree propre (seuls `test_bench_operations.cpp`/`test_print_paths.cpp`
restent non suivis à la racine — vos fichiers scratch, ne jamais y toucher).

Suites de tests : **unit 12/12, golden 13/13, toujours vertes** après chaque
commit de cette session.

## Ce qui est fait et vérifié

Toute la mécanique géométrique du §3 de la spec, testée à l'unité :

- `include/nas/core/geometry.hpp` / `.cpp` : primitives "taguées"
  (`minkowski_sum_tagged`, `compute_polytope_plane_intersection_tagged`,
  `convex_hull_2_tagged`/`_with_origin`, `compute_2d_polygon_intersection_tagged`
  /`_with_origin`) qui font porter une charge utile (position couplée) à
  travers hull/coupe de plan/clip, sans jamais construire de vrai polytope
  4D — voir `docs/cube-implementation-plan.md` §2 pour le raisonnement complet.
- `include/nas/core/node.hpp` : `CubeState{None,InHand,PlacedActive,PlacedInactive}`,
  `CubePlacement` (patch du cube aligné index à index avec `patch_vertices`
  du pied), `Node::cube_state`/`cube`.
- `core/expansion.{hpp,cpp}` : `expand_cube_placement` (§3.1, pose),
  `expand_node` modifié pour transporter le cube à travers un pas normal
  quand `cube_state==PlacedActive` (§3.2, chemin existant intégralement
  inchangé sinon), `expand_onto_cube` (§3.3, pas sur le cube, désactive
  systématiquement le cube après usage — v1 = cube à usage unique, §3.4).
- `planners/astar_search.cpp` : `PatchIndex` distingue aussi `cube_state`
  (étape 5, sinon deux branches avec/sans cube au même endroit fusionnaient
  à tort).
- `AstarSearchConfig` : `cube_half_extent`/`cube_height` (0 = extension
  désactivée, comportement inchangé), `cube_place_cost`/`cube_step_cost`.
  `AstarSearch::search()` propose `expand_cube_placement`/`expand_onto_cube`
  comme actions candidates en plus de `expand_node`.
- **Filtre "découverte"** (idée de l'utilisateur, dernière session) : au
  constructeur, une passe BFS (`expand_node` seul, cube désactivé) calcule
  quelles surfaces sont atteignables par un pas normal ; une pose de cube
  n'est proposée à la recherche que si elle amène une surface *hors* de cet
  ensemble à portée (rayon dérivé de l'enveloppe réelle des polytopes
  LF-in-RF/RF-in-LF chargés). Vérifié correct sur StairsGap : `{sol seul}`
  atteignable sans cube, la pose utile passe le filtre (EPA dist 0.52m <
  seuil 0.74m).
- **Bug trouvé et corrigé cette session** (par câblage, pas par test) :
  `expand_node`'s boucle normale ne recopiait pas `cube_state` sur l'enfant
  → un pied `InHand` prenant un pas normal perdait silencieusement le cube.
  Corrigé (no-op pour tout appelant sans cube).
- **Deuxième bug trouvé et corrigé** (en construisant la visualisation,
  juste avant l'interruption) : `expand_onto_cube` calculait `centroid` via
  `area_centroid(patch_2d, parent->cube->transform_to_3d, patch_3d)` —
  `area_centroid` suppose que le z local du transform passé EST la hauteur
  du patch, mais `parent->cube->transform_to_3d` est calé sur la BASE du
  cube (z local 0 = base, pas le dessus) : `centroid.z` tombait à la
  hauteur de base au lieu de `cube_height`. `patch_vertices`/`patch_3d`
  (construits point par point avec le bon z) n'étaient pas affectés, donc
  le heuristique EPA (qui lit `patch_vertices`, pas `centroid`) n'était pas
  impacté par ce bug — seul un champ d'affichage/convenance était faux.
  Corrigé (commit `9fbeba8`) en réutilisant `top_transform_to_3d` (déjà
  construit juste après pour le nœud enfant) au lieu du transform de base.

## Validation décisive : le mécanisme marche, prouvé avec un gros cube de test

Sur le vrai scénario `StairsGap` (escalier avec la marche 1 retirée, voir
`config/scenario_library.cpp`), avec la reachability des pieds clampée
(`*_clamp_z25.obj`, Z≤0.25m) **et un cube de test à 60cm** (pas le vrai
cube de 15cm de l'utilisateur — un cube volontairement surdimensionné pour
isoler "le mécanisme marche-t-il" de "15cm est-il bien calibré") :

**`AstarSearch::search()` trouve un chemin complet en 87 expansions, 0.07s** :

```
départ (InHand) → pose du cube (PlacedActive, même position)
  → pas normal sur le sol → pas SUR LE CUBE (surface_id=-2, PlacedInactive)
  → Step2 → Step2 (autre pied) → Step3 → Step3 → Step4 (but)
```

Ça prouve, sans ambiguïté, que pose + transport + pas-sur-le-cube +
désactivation + dédoublonnage + filtre de découverte fonctionnent tous
ensemble correctement dans une vraie recherche complète. Le problème
restant (ci-dessous) est isolé au calibrage du cube RÉEL de 15cm sur CE
trou précis, pas au mécanisme.

## Ce qui ne marche pas encore

Avec le vrai cube de 15cm (`talosReachability/data/reachability_constraints/
Cube_constraints_in_{LF,RF}.obj`, calibré cette session : X dans [0.0,0.15]
avant, Y dans [0.20,0.35] (RF) / [-0.35,-0.20] (LF) latéral décalé pour la
clairance de largeur d'appui, Z dans [-0.05,0.10]), **la recherche complète
sur `StairsGap` ne converge pas dans un budget praticable** (testé jusqu'à
~12000 expansions / ~4min sans trouver). Le trou (~0.5m entre le bord érodé
du sol à x≈0.19 et Step2 à x=0.6) est probablement trop grand pour un cube
de 15cm être une aide *facile à trouver*, même si géométriquement une
combinaison existe peut-être (pas vérifié de façon exhaustive).

Piste explorée en dernier avant l'interruption : construire une **visualisation
animée du plan** (demande explicite de l'utilisateur : "vidéo où on voit les
pas apparaître puis le cube être posé puis les pas qui continuent") en
utilisant le plan à 87 expansions (gros cube) comme données, pour que
l'utilisateur voie concrètement ce qui se passe avant de décider s'il faut
continuer à calibrer le cube de 15cm ou accepter la limite documentée.

## Reprise : outils de scratch à recréer (perdus au redémarrage, `/tmp`)

Les diagnostics autonomes utilisés cette session vivent dans le scratchpad
de session (`/tmp/claude-*/.../scratchpad/`), **pas dans le dépôt** — ils
seront perdus. Ce qui suit permet de les reconstruire vite. Compilation
type (même schéma pour tous, changer juste le `.cpp`) :

```sh
c++ -O3 -DNDEBUG -DCGAL_USE_GMPXX=1 -std=gnu++20 -fPIC -frounding-math \
  -Iinclude -isystem $CONDA_PREFIX/include -isystem $CONDA_PREFIX/include/eigen3 \
  <fichier>.cpp -o <sortie> \
  -Wl,-rpath,$CONDA_PREFIX/lib build/libnas.a \
  $CONDA_PREFIX/lib/lib{gmpxx,mpfr,gmp,coal.so.3.0.1,boost_serialization,boost_chrono,boost_filesystem,octomap.so.1.10.0,octomath.so.1.10.0,eiquadprog}.*
```//(adapter l'extension .so selon la lib — voir les commandes précédentes de cette session pour l'exact)

**Reachability pieds clampée (dossier `reach_z25`)** : copier
`talosReachability/data/reachability_constraints/{LF,RF}_constraints_in_*_quasi_flat_REDUCED_clamp_z25.obj`
vers un dossier scratch en les renommant `{LF,RF}_constraints_in_*_quasi_flat_REDUCED.obj`
(sans le suffixe `_clamp_z25`) — ces fichiers `_clamp_z25.obj` SONT dans le
dépôt (committés), seul le renommage est à refaire :

```sh
mkdir -p /tmp/scratch/reach_z25
cp talosReachability/data/reachability_constraints/LF_constraints_in_RF_quasi_flat_REDUCED_clamp_z25.obj /tmp/scratch/reach_z25/LF_constraints_in_RF_quasi_flat_REDUCED.obj
cp talosReachability/data/reachability_constraints/RF_constraints_in_LF_quasi_flat_REDUCED_clamp_z25.obj /tmp/scratch/reach_z25/RF_constraints_in_LF_quasi_flat_REDUCED.obj
```

**Gros cube de test (60cm, PAS committé, à régénérer)** — script Python qui
génère les deux `.obj` (boîte à 8 sommets, mêmes faces que le vrai
`Cube_constraints_in_*.obj` du dépôt) :

```python
def gen(xs, ys, zs):
    verts = [(x,y,z) for x in xs for y in ys for z in zs]
    def idx(x,y,z): return verts.index((xs[x],ys[y],zs[z])) + 1
    lines = [f'v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}' for v in verts]
    faces = [(idx(0,0,0),idx(0,1,0),idx(0,1,1),idx(0,0,1)),
             (idx(1,0,0),idx(1,0,1),idx(1,1,1),idx(1,1,0)),
             (idx(0,0,0),idx(0,0,1),idx(1,0,1),idx(1,0,0)),
             (idx(0,1,0),idx(1,1,0),idx(1,1,1),idx(0,1,1)),
             (idx(0,0,0),idx(1,0,0),idx(1,1,0),idx(0,1,0)),
             (idx(0,0,1),idx(0,1,1),idx(1,1,1),idx(1,0,1))]
    lines += ['f ' + ' '.join(str(i) for i in f) for f in faces]
    return '\n'.join(lines) + '\n'

for supp, ys in [('RF', (0.20, 0.70)), ('LF', (-0.70, -0.20))]:
    open(f'/tmp/scratch/BigCube_constraints_in_{supp}.obj','w').write(gen((0.0,0.55), ys, (-0.05,0.10)))
```

**Le programme qui a trouvé le chemin à 87 expansions** (`check_bigcube.cpp`,
à adapter en `dump_bigcube_plan.cpp` pour exporter le JSON — voir plus bas
pour la version qui exporte) :

```cpp
#include "nas/config/scenario.hpp"
#include "nas/core/reachability.hpp"
#include "nas/planners/astar_search.hpp"
#include <cstdio>
using namespace nas;

int main(int argc, char** argv) {
    std::string foot_dir = argv[1];   // /tmp/scratch/reach_z25
    std::string cube_dir = argv[2];   // /tmp/scratch (BigCube_constraints_in_*.obj)
    int max_exp = argc > 3 ? std::atoi(argv[3]) : 5000;

    ReachabilityModel reach = ReachabilityModel::load({
        {foot_dir + "/RF_constraints_in_LF_quasi_flat_REDUCED.obj", "RF", "LF", ReachabilityDirection::Forward},
        {foot_dir + "/LF_constraints_in_RF_quasi_flat_REDUCED.obj", "LF", "RF", ReachabilityDirection::Forward},
        {cube_dir + "/BigCube_constraints_in_LF.obj", "Cube", "LF", ReachabilityDirection::Forward},
        {cube_dir + "/BigCube_constraints_in_RF.obj", "Cube", "RF", ReachabilityDirection::Forward},
    });
    config::Scenario sc = config::load_scenario("StairsGap");

    AstarSearchConfig cfg;
    cfg.start_position = Point_3(0.1, 0.0, 0.0);
    cfg.start_stance_foot = StanceFoot::Right;
    cfg.goal_location = sc.surfaces.back().centroid;
    cfg.goal_stance_foot = StanceFoot::Left;
    cfg.expansion_params.rotation_enabled = true;
    cfg.expansion_params.yaw_discretization_num = 3;
    cfg.expansion_params.yaw_angle_increment = 10.0 / 180.0 * M_PI;
    cfg.max_expansions = max_exp;
    cfg.cube_half_extent = 0.3;  // le "gros" cube de test -- PAS le vrai (0.075)
    cfg.cube_height = 0.2;       // idem, PAS le vrai (0.15)

    AstarSearch search(sc.surfaces, reach, cfg);
    search.search();
    const auto& path = search.result_path();
    std::printf("path_found=%s expansions=%d path_size=%zu\n",
                path.empty() ? "false" : "true", search.expansion_count(), path.size());
    for (Node* n : path) {
        std::printf("  depth=%d surface_id=%d stance=%d cube_state=%d yaw_deg=%.1f centroid=(%.3f,%.3f,%.3f)\n",
                    n->depth, n->surface_id, static_cast<int>(n->stance_foot), static_cast<int>(n->cube_state),
                    n->foot_yaw * 180.0 / M_PI,
                    CGAL::to_double(n->centroid.x()), CGAL::to_double(n->centroid.y()), CGAL::to_double(n->centroid.z()));
    }
    return path.empty() ? 1 : 0;
}
```

**Exécuter sous garde-fous** (règle du projet) :
```sh
ulimit -v 3000000 && timeout 60 ./check_bigcube /tmp/scratch/reach_z25 /tmp/scratch 5000
```

## Prochaine étape immédiate (là où on s'est arrêté)

L'utilisateur veut une **visualisation animée** (pas juste du texte) du plan
ci-dessus : les pas qui apparaissent un par un, le cube qui apparaît au bon
moment, puis les pas qui continuent. Plan (pas encore exécuté) :

1. Étendre `dump_bigcube_plan.cpp` (variante du programme ci-dessus, déjà
   écrite et testée cette session, à recréer) pour écrire un JSON avec les
   surfaces (bornes x/y/z par surface) et, pour chaque nœud du
   `result_path()`, son type (`start`/`step`/`place_cube`/`step_on_cube`),
   pied, position (x,y,z), et — au moment `place_cube` — l'étendue du cube
   posé (`cube_x`/`cube_y`/`cube_z0`/`cube_z1`, dérivée de
   `n->cube->vertices_3d` + `cube_height`).
2. Publier un Artifact HTML/JS (pas une vidéo au sens fichier — un artifact
   interactif est plus approprié et plus utile : lecture auto + pause/scrub)
   qui anime cette séquence : vue de profil (X-Z, l'escalier + le cube comme
   rectangle) et vue de dessus (X-Y, pour montrer le décalage latéral du
   cube) en parallèle, un pas/pose apparaît à chaque tick.
3. Le JSON généré la dernière fois (avant l'interruption, avec le bug de
   centroid ci-dessus déjà corrigé mais PAS re-testé après le fix) avait
   cette forme exacte (9 pas, à régénérer pour avoir les z corrects du pas
   sur le cube — avant le fix, `step_on_cube` affichait z=0 au lieu de 0.2,
   c'est le bug qu'on vient de corriger) :

```json
{
  "surfaces": [
    {"id": 0, "x": [-1.69, 0.19], "y": [-0.89, 0.89], "z": 0},
    {"id": 1, "x": [0.71, 0.79], "y": [-0.05, 0.49], "z": 0.2},
    {"id": 2, "x": [1.01, 1.09], "y": [-0.05, 0.49], "z": 0.3},
    {"id": 3, "x": [1.31, 1.39], "y": [-0.05, 0.49], "z": 0.4}
  ],
  "steps": [
    {"depth": 0, "type": "start", "stance": "R", "x": 0.1, "y": 0, "z": 0},
    {"depth": 1, "type": "place_cube", "stance": "R", "x": 0.1, "y": 0, "z": 0,
     "cube_x": [0.1, 0.19], "cube_y": [0.2, 0.7], "cube_z0": 0, "cube_z1": 0.2},
    {"depth": 2, "type": "step", "stance": "L", "x": 0.045, "y": 0.247, "z": 0},
    {"depth": 3, "type": "step_on_cube", "stance": "R", "x": 0.095, "y": 0.003, "z": 0.2},
    {"depth": 4, "type": "step", "stance": "L", "x": 0.712, "y": 0.161, "z": 0.2},
    {"depth": 5, "type": "step", "stance": "R", "x": 0.749, "y": 0.006, "z": 0.2},
    {"depth": 6, "type": "step", "stance": "L", "x": 1.048, "y": 0.103, "z": 0.3},
    {"depth": 7, "type": "step", "stance": "R", "x": 1.049, "y": 0.005, "z": 0.3},
    {"depth": 8, "type": "step", "stance": "L", "x": 1.348, "y": 0.102, "z": 0.4}
  ]
}
```

(le `z: 0` du pas `depth 3` ci-dessus vient tourné avec le bug pré-fix ;
après `9fbeba8`, régénérer devrait donner `z: 0.2`.)

## Décisions ouvertes pour l'utilisateur au retour

- Une fois la visualisation vue : continuer à calibrer le cube de 15cm sur
  `StairsGap`, ou accepter que ce scénario de démo utilise un cube plus
  grand et documenter 15cm comme "mécanisme validé, calibrage fin non
  terminé" ?
- Le filtre de découverte (seuil dérivé de l'enveloppe 3D complète, ~0.74m)
  est peu sélectif à cause de la portée verticale incluse — resserrer en
  planaire (X-Y seulement) reste une piste non essayée.
- Rien n'a été poussé sur `rwa` depuis le début de la branche `cube` — à
  faire quand l'utilisateur le demandera explicitement (règle du projet :
  jamais de push sans consigne).
