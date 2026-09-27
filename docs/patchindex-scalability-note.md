# Limite de scalabilité de `PatchIndex` (découverte, pas corrigée)

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

## Prochaines étapes (dans l'ordre demandé, aucune commencée)

1. **Yaw en entier + validation dure que l'incrément divise 360°** — élimine le bug de case doublée à
   la racine (voir ci-dessus). Prioritaire : à vérifier si ça réduit lui-même une partie du problème de
   bucket avant de toucher à `CELL`.
2. **`CELL` doit devenir un paramètre**, pas une constante codée en dur dans `PatchIndex` — probablement
   un nouveau champ sur `AstarSearchConfig`, transmis au constructeur de `PatchIndex` aux côtés de
   `tol_`/`rotation_enabled_`/`yaw_increment_`. Une fois fait, la valeur 5cm mesurée ci-dessus (ou plus
   fine) devient un choix de configuration à tester, pas un changement de code.
3. **Une vraie structure spatiale (k-d tree, R-tree)** : seulement si 1 et 2 ne suffisent pas. N'aurait
   de sens que pour la partie continue (position, yaw traité comme dimension circulaire) à l'intérieur
   de chaque case déjà isolée par les champs discrets (surface/pied/cube_state/cubes_picked_up) — ce
   partitionnement discret est déjà un hachage exact, qu'une structure spatiale n'améliorerait pas. Ne
   réglerait pas le problème si les nœuds d'une case sont de VRAIS quasi-voisins (à plus de 2cm les uns
   des autres mais tous dans un petit rayon) plutôt que des collisions de hachage évitables.

Touche `PatchIndex`, utilisé par tous les scénarios (pas seulement le cube) — à traiter avec la même
rigueur de non-régression que le reste de cette codebase (suite complète + `nas_bench_perf` sur les 11
scénarios avant/après, comme fait pour chaque étape de cette session).
