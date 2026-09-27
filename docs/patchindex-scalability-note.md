# Limite de scalabilité de `PatchIndex` (découverte et corrigée — yaw + CELL, 2026-09-27)

Contexte : trouvée en profilant le scénario StairsGap+`scene_cubes` (voir `docs/cube-pickup-spec.md`),
dont la recherche prenait ~8.8s pour 1321 expansions (contre ~0.4s pour 218 expansions sans
`scene_cubes` sur la même scène). Ni le test géométrique du ramassage (`expand_cube_pickup`, mesuré à
moins de 1ms sur le total) ni le type de `Node::cubes_picked_up` n'expliquaient l'écart. La cause
réelle, trouvée par instrumentation manuelle (temporaire, retirée avant commit — `perf` indisponible
sans droits root sur cette machine) : `PatchIndex` lui-même.

## Le mécanisme (`src/planners/astar_search.cpp`, classe `PatchIndex`)

Chaque nœud est rangé dans une case (`Cell`) déterminée par des champs **discrets, exacts** :
`surface_id`, `stance_foot`, `cube_state`, `cubes_picked_up`, plus deux approximations :
- position : centroïde arrondi à une grille de 10cm (`CELL = 0.1`) sur x, y, z ;
- yaw : `static_cast<int>(foot_yaw / yaw_increment)`, une case parmi celles utilisées à l'expansion.

`find()` scanne les 27 cases voisines (±1 en x, y, z) autour de la case du candidat, et pour chaque
case non vide, teste chaque nœud qu'elle contient via `similar()` (vrai test : mêmes champs discrets
+ `patch_distance(a,b) < node_similarity_threshold` (2cm), une distance de Hausdorff à deux sens entre
les polygones des deux patchs).

Le commentaire du fichier affirme "the cost is independent of the set size" — c'est faux dès qu'une
case accumule beaucoup de nœuds qui partagent les mêmes clés discrètes/de position grossière mais ne
sont pas assez proches (< 2cm) pour fusionner : `similar()` est alors appelé une fois par nœud déjà
présent dans la case, un coût qui grandit avec la population de la case, pas O(1).

## Mesure

Sur le scénario StairsGap+`scene_cubes` (1321 expansions) : `open_index` fait **966 667** appels à
`similar()`, avec une case atteignant **175 nœuds**. `closed_index` (nœuds déjà expansés) reste
modeste : 51 052 appels, case max 18. `process_child` (dominé par ces deux index) représente 87% du
temps total (7.6s sur 8.8s) ; `expand_cube_pickup` lui-même : 0.8ms.

Explication : démarrer mains vides force la recherche à explorer bien plus largement le sol (~6x plus
d'expansions) avant de pouvoir s'engager vers le cube. Beaucoup de candidats côté `expand_node`
(plusieurs yaws testés par surface) partagent la même case grossière (même case 10cm, même case de
yaw, même `cube_state`/`cubes_picked_up`) sans être assez proches en forme/position exacte pour
fusionner — ils s'empilent donc comme entrées séparées. Comme la recherche met longtemps à s'engager,
beaucoup de ces candidats restent longtemps **en attente** dans l'ensemble ouvert (d'où la population
qui explose côté `open_index`, pas `closed_index` : un nœud une fois expansé "sort de la compétition").

**Ce n'est pas un défaut introduit par le ramassage de cube** — c'est une caractéristique préexistante
de `PatchIndex`, simplement rendue visible par un scénario qui force une recherche inhabituellement
large avant engagement.

## Deux nuances trouvées en creusant la question "un décalage de 1cm peut-il dissocier deux surfaces proches ?"

**Position (x, y, z) : non, la marge est mathématiquement sûre.** La case fait 10cm, la tolérance de
fusion 2cm — un facteur 5. Deux centroïdes distants de ≤2cm ne peuvent jamais tomber dans des cases
dont l'indice diffère de plus de 1 (franchir 2 frontières de case exigerait une distance > 10cm), donc
le scan des 27 cases voisines les trouve toujours, quelle que soit la position de la frontière.

**Yaw : un vrai bug structurel, pas de la précision flottante.** Le yaw n'est PAS un problème de
"frontière de case tombant mal à cause d'une imprécision" — les yaws produits par l'expansion sont des
multiples entiers de `yaw_angle_increment` relatifs au yaw du parent, donc en arithmétique exacte la
classe de congruence est préservée sur tout le chemin. Le vrai problème, trouvé en creusant la question
initiale : `cell_of()`/`similar()` utilisent `static_cast<int>(foot_yaw / yaw_increment)`, qui **tronque
vers zéro**, contrairement à x/y/z qui utilisent bien `std::floor`. Pour un cast C++, `int(-0.99) == 0`
alors que `floor(-0.99) == -1` : la case "0" fait donc le **double de largeur** de toutes les autres
(elle couvre tout l'intervalle `(-increment, +increment)` au lieu d'une largeur normale d'un
`increment`). Deux nœuds à -9° et +9° (avec un pas de 10°) tombent dans la même case alors qu'ils sont
à 18° d'écart — un décalage déterministe et reproductible, pas un accident numérique. Comme yaw=0 est
une orientation de départ très commune, et que les premières expansions génèrent naturellement des
candidats symétriques autour de 0, cette case doublée pourrait contribuer directement au problème de
bucket mesuré plus haut (plus d'entrées non-fusionnables regroupées à tort). Pas encore vérifié
empiriquement — à faire avant/pendant la correction ci-dessous.

**Correction retenue (pas encore faite)** : représenter le yaw en interne comme un **entier** (indice
de multiple de `yaw_angle_increment`), pas comme un flottant re-discrétisé à chaque comparaison — élimine
la troncature/`floor` asymétrique à la racine plutôt que de la contourner. Accompagner d'une validation
dure au constructeur : `360° / yaw_angle_increment` doit être un entier (sans quoi la classe de
congruence ne boucle pas proprement à ±180°, cf. l'argument "2π est un multiple exact de l'incrément"
du §"position" ci-dessus, qui ne tient que si l'incrément divise 360° exactement).

## Expérience faite : réduire `CELL` (résultat mesuré, pas encore appliqué)

Testé `CELL = 0.05` (5cm, gardant `node_similarity_threshold` à 2cm — marge x2.5, toujours valide pour
la garantie du §"position" ci-dessus) sur StairsGap+`scene_cubes` :

- **8850ms → 5110ms (-42%)**, expansions et chemin strictement identiques (1321 expansions, même trace
  nœud par nœud).
- Suite complète (30/30) et comparaison sur les 11 scénarios standard (`nas_bench_perf`) : expansions
  identiques partout, zéro régression.

Changement non conservé tel quel (juste testé puis annulé) : `CELL` est aujourd'hui un
`static constexpr` codé en dur dans `PatchIndex` — voir la tâche "CELL en paramètre" ci-dessous, qui
rendrait cette valeur réglable plutôt que de fixer 0.05 en dur à la place de 0.1.

## Étape 1 (yaw en entier) : FAITE et appliquée (2026-09-27, après investigation approfondie)

Implémentée : `Node::foot_yaw_bin`, un entier suivi exactement (parent + même décalage entier que le
yaw flottant, jamais re-dérivé par division), plus `yaw_bins_per_revolution()` qui valide dur que
`360° / yaw_angle_increment` est entier. Élimine la troncature asymétrique décrite plus haut.

**Effet mesuré** : sur la suite golden (11 scénarios), le nombre d'expansions augmente sur la
plupart (attendu — la case yaw≈0° n'est plus en double largeur, donc fusionne moins), diminue sur
deux (pas monotone : A* pondérée, l'ordre d'exploration peut aussi bien raccourcir qu'allonger).
Sur 10 des 11 scénarios le chemin final reste identique (même longueur/surfaces/pieds que la
référence legacy). **Sur `NarrowPassage` seul**, le chemin change : 30 nœuds (29 pas) → 34 nœuds
(33 pas) — cassant le match exact avec la Table I du papier CASSR, longtemps cité comme sanity-check
de ce projet (PROGRESS.md).

**Investigation de la régression `NarrowPassage`, à la demande de l'utilisateur (qui doutait qu'elle
soit justifiée)** — chaque théorie testée par la mesure, pas juste raisonnée :
1. *Un `floor()` seul (sans champ `Node`, suggestion utilisateur) donne-t-il la même régression ?*
   Oui, bit pour bit (98→115 expansions, 30→34 nœuds) — élimine l'hypothèse "bug de mon
   implémentation bins" : n'importe quelle correction de la troncature casse `NarrowPassage` pareil.
   (`floor()` seul diffère des bins entiers sur d'autres scénarios — Stairs 49 vs 50, etc. — parce
   que les bins unifient aussi le repli ±180°, ce qu'un `floor()` seul ne fait pas ; sans effet sur
   `NarrowPassage`, qui n'approche jamais ±180°.)
2. *Le nœud partagé (identique dans les deux versions, mêmes 6 premiers pas, même `node_id`) a-t-il
   une raison de produire un enfant différent ?* Tracé via `on_expand`/`on_child` : ses 7 candidats
   de lacet sont des **égalités exactes** (même `g`, `h`, `f` à la dernière décimale — le patch ne
   dépend que de la surface, pas du lacet choisi). Aucune fusion ne les départage à ce niveau. Piste
   `SkippedClosed` (le code ne compare aucun coût sur un match contre le **closed**-set, contrairement
   à l'open-set) testée par instrumentation : **zéro occurrence** sur ce scénario — piste fausse,
   explicitement abandonnée. La vraie fusion apparaît un niveau plus bas (l'un des 7 enfants
   d'égalité, en s'étendant, rencontre un enfant d'une **autre branche** et le bat/perd) — gouvernée
   par la même règle de lacet en cours de correction.
3. *La fusion peut-elle changer l'optimum ?* Testé directement : recherche relancée avec
   `node_similarity_threshold` quasi nul (`1e-9`, fusion flitue désactivée, seuls des doublons
   géométriques exacts peuvent encore fusionner) — **tableau 2×2 décisif** :

   |                        | tolérance floue (0.02) | tolérance quasi nulle (1e-9) |
   |------------------------|:-----------------------:|:-----------------------------:|
   | code buggé (tronque)   | 98 exp / **30 nœuds**   | 103 exp / **34 nœuds**         |
   | code corrigé (bins)    | 115 exp / **34 nœuds**  | 115 exp / **34 nœuds**         |

   Les 30 nœuds n'apparaissent que dans une seule case (bug + flou) — retirer *l'un ou l'autre*
   ingrédient (corriger le bug, OU juste resserrer la tolérance) donne 34. **Conclusion tranchée** :
   30 n'était pas un optimum robuste protégé par du code correct — un artefact de l'interaction
   bug-de-troncature × tolérance floue à 2cm. 34 est la réponse stable aux quatre coins du tableau
   sauf un.

**Décision finale (2026-09-27)** : fix appliqué (bins entiers, gardés plutôt que `floor()` seul pour
traiter aussi le repli ±180°). Convergence revérifiée à froid sur les 11 scénarios (avant ≤ après
partout, égal sur 10/11) et perf toujours du même ordre de grandeur (ratios 1.00x-1.45x). Tests mis à
jour en conséquence : `merge_consistency_test.cpp` utilisait sa propre réplique périmée de l'ancien
calcul de lacet pour sa vérification indépendante (corrigé pour lire `n.foot_yaw_bin`, le champ
canonique, sinon il signalait des "fusions manquées" qui n'étaient qu'un désaccord de réplique, pas
un vrai bug) ; `astar_search_golden_test.cpp`/`golden_all_scenes_test.cpp` documentent explicitement
`NarrowPassage` comme exception acceptée (pas une comparaison stricte affaiblie partout, seulement
ce scénario, avec la justification ci-dessus) ; `dual_target_all_scenes_test.cpp` : références de
régression (`expected_*_expansions`) mises à jour aux nouvelles valeurs mesurées.

## Étape 2 (CELL configurable) : FAITE et validée (2026-09-27)

`AstarSearchConfig::patch_index_cell_size` (défaut 0.1, inchangé), transmis au constructeur de
`PatchIndex` à côté de `tol_`/`rotation_enabled_`/`yaw_increment_`, lu depuis le JSON
(`"patch_index_cell_size"`, `config/planner_config.cpp`). Testé à 0.1 (no-op : suite golden 100%
identique à l'état d'avant, ~178s) puis à 0.05 sur les 11 scénarios standard (`nas_bench_perf` +
`golden_all_scenes_test`, hook temporaire `NAS_CELL_SIZE`, retiré après coup) : **expansions
strictement identiques sur les 11 scénarios**, chemins identiques (toujours conformes à la
référence legacy), gain modeste sur ce lot (549.7ms → 530.2ms, -3.5% — bien moindre que les -42%
mesurés sur StairsGap+`scene_cubes`, cohérent : le gain dépend de la population des cases, ce lot de
11 scénarios n'a pas la pathologie qui a motivé cette note).

**`StairsGap+scene_cubes` intégré en permanence à `nas_bench_perf`** (2026-09-27, plus besoin de
hook ad hoc), avec `patch_index_cell_size` en second argument CLI (`nas_bench_perf [runs]
[cell_size]`). Comparaison complète 0.05 / 0.1 / 0.2 sur les 11 scénarios + `StairsGap+cube` :
expansions et chemins strictement identiques aux 3 tailles (golden 17/17 à 0.05 et à 0.2, comme à
0.1). Résultat net :
- **0.05 : seul gain réel**, -40% sur `StairsGap+cube` (9544ms → 5683ms), cohérent avec la mesure
  d'origine (-42%). Sur les 11 scénarios standard, dans le bruit de mesure (±5%, pas de signal net —
  leur population de case n'est jamais assez grande pour que `CELL` compte).
- **0.2 : PIRE que 0.1**, +11% sur `StairsGap+cube` (9544ms → 10567ms) — des cases plus grosses
  regroupent encore plus de nœuds par case, donc `similar()` est appelé encore plus souvent (le coût
  croît avec la population de la case, exactement le mécanisme documenté plus haut, pas O(1)).
- **0.02 (= `node_similarity_threshold`, marge nulle) testé aussi** : encore plus rapide (2030ms sur
  `StairsGap+cube`, -64% vs 0.05) et zéro missed-merge mesuré (`merge_consistency_test`, qui recalcule
  indépendamment si deux nœuds "pushed" auraient dû fusionner -- 0 sur tous ses cas, y compris les
  explosions Euclidiennes à ~13500 enfants). Empiriquement sûr sur ce jeu de scènes, mais sans la
  garantie mathématique de marge x2.5 que 0.05 a -- gardé en réserve, pas retenu comme défaut par
  prudence plutôt que par une régression mesurée.

**Décision (2026-09-27) : `patch_index_cell_size` par défaut passe à 0.05** (`include/nas/planners/
astar_search.hpp`), sur la base des mesures ci-dessus. Suite complète revalidée à ce nouveau défaut :
`nas_tests_fast`+`nas_tests_golden` 100% (129s, plus rapide qu'avant grâce aux tests cube qui en
profitent aussi).

## Prochaines étapes

1. ~~Yaw en entier~~ — fait (voir ci-dessus).
2. ~~`CELL` en paramètre~~ — fait (voir ci-dessus).
3. **Une vraie structure spatiale (k-d tree, R-tree)** : seulement si 1 et 2 ne suffisent pas. N'aurait
   de sens que pour la partie continue (position, yaw traité comme dimension circulaire) à l'intérieur
   de chaque case déjà isolée par les champs discrets (surface/pied/cube_state/cubes_picked_up) — ce
   partitionnement discret est déjà un hachage exact, qu'une structure spatiale n'améliorerait pas. Ne
   réglerait pas le problème si les nœuds d'une case sont de VRAIS quasi-voisins (à plus de 2cm les uns
   des autres mais tous dans un petit rayon) plutôt que des collisions de hachage évitables.

Touche `PatchIndex`, utilisé par tous les scénarios (pas seulement le cube) — à traiter avec la même
rigueur de non-régression que le reste de cette codebase (suite complète + `nas_bench_perf` sur les 11
scénarios avant/après, comme fait pour chaque étape de cette session).
