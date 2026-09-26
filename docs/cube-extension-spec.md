# Extension de CASSR : action de manipulation "poser un cube"

Contexte : CASSR (Wang & Tonneau) est un A* de footstep planning dont chaque
nœud porte un polytope continu de positions atteignables pour le pied actif
(cf. `NODE` : `effectorId`, `parent`, `surfaceId`, `extremePoints`, `yaw`).
L'expansion (`expandNode`) calcule la somme de Minkowski du patch parent avec
la reachability du pied, puis l'intersecte avec les surfaces de
l'environnement (Eq. 2 du papier).

Ce document spécifie une extension : le robot porte un cube qu'il peut poser
au sol pour créer une nouvelle surface de contact (par ex. combler une marche
autrement inatteignable). Il documente un piège de correction découvert par
construction d'un contre-exemple, et la solution retenue.

## 1. Action proposée

Depuis un nœud où le pied d'appui a pour patch de positions possibles `Z`
(polygone convexe, surface `S_j`, yaw du nœud) :

- le cube est posé devant/à côté du pied, aligné en yaw sur le pied ;
- l'ensemble des poses possibles du cube est calculé par la même mécanique
  que la reachability inter-pied : somme de Minkowski de `Z` avec la
  reachability du cube `K_cube` (le polytope "où peut aller le cube sachant
  la position du pied"), intersectée avec la surface support érodée de la
  demi-emprise du cube ;
- le dessus du cube devient une nouvelle surface virtuelle sur laquelle on
  peut poser le pied dans les expansions suivantes.

Intuition confirmée : ceci peut rendre atteignable un contact (ex. une marche)
qui ne l'était pas sans le cube, avec le même avantage combinatoire que le
reste de CASSR (un nœud par surface support, pas un nœud par pose discrète).

## 2. Le piège : la somme de Minkowski "aveugle" perd le couplage pied/cube

Approche naïve envisagée initialement : traiter "poser le cube" comme une
action normale de CASSR — calculer l'ensemble `T` de toutes les positions
possibles du dessus du cube par somme de Minkowski depuis `Z`, l'ajouter à la
liste des surfaces, et laisser l'expansion normale tester l'intersection
`(pas atteignable) ∩ T`.

**Ceci casse la garantie "QP toujours faisable".** Le patch `Z` du pied
d'appui contient en général plusieurs positions candidates `z`. La somme de
Minkowski `T` mélange, sans les garder associées, "le cube posé depuis
`z = 0.2`" et "le cube posé depuis `z = 0.9`". Un point du pas suivant peut
alors intersecter `T` sans qu'aucun `z` unique ne rende *simultanément*
faisables la pose du cube et le pas qui y mène.

### Contre-exemple 1D minimal

- Patch du pied d'appui : `Z = [0, 1]`.
- Cube posable entre `z+0.1` et `z+0.3`, largeur `0.2` → dessus du cube
  couvre `[z, z+0.4]` pour un `z` donné.
- Pas de l'autre pied : atteint `[z+0.5, z+0.6]`.

Pour un `z` fixé, le pas (`≥ z+0.5`) n'atteint jamais le dessus du cube posé
depuis ce même `z` (`≤ z+0.4`) : l'action est en réalité toujours infaisable.

Sommes de Minkowski (en ignorant l'origine du `z`) :
- dessus de cube possible : `T = [0, 1.4]`
- pas possible : `[0.5, 1.6]`
- intersection : `[0.5, 1.4]` — **non vide**, alors qu'aucune pose réelle ne
  le permet. Un point `x = 1` de cette intersection est "expliqué" par
  `z = 0.7` côté cube et par `z = 0.5` côté pas : deux `z` différents, donc
  incohérent. Le QP construit a posteriori peut être infaisable.

Ce problème ne se pose pas pour une expansion normale de CASSR parce que
chaque patch ne dépend que d'un seul parent, et qu'on ne réutilise jamais deux
fois la même variable `z` pour deux contraintes indépendantes. Le cube
introduit précisément cette réutilisation.

Un second couplage du même type existe si deux pas consécutifs sont posés sur
le cube : ils doivent être sur le *même* cube (même `c`), ce qu'une simple
liste de surfaces ne peut pas imposer.

## 3. Solution retenue : état joint (position du pied, position du cube)

Tant que le cube est "actif" (posé mais encore utilisable comme surface, càd
avant que la règle "on ne revient pas sur une surface quittée" ne le rende
sans objet), le nœud ne porte plus un polygone 2D de positions du pied, mais
un **polytope joint** `J ⊂ ℝ³ × ℝ³` de paires `(x, c)` :

- `x` : position du dernier pied posé (pied d'appui pour l'expansion
  suivante) ;
- `c` : position du cube.

`x` et `c` vivent chacun sur un plan (leur surface respective), donc `J` est
de dimension intrinsèque 4, pas 6.

### 3.1 Poser le cube

Depuis le patch `Z` du pied d'appui (polytope "classique", pas encore de
`c`) :

```
J0 = { (z, c) : z ∈ Z, c ∈ z ⊕ Q·K_cube, c ∈ S̃_j }
```

où `S̃_j` est la surface support érodée de la demi-emprise du cube. `Q` est la
matrice de rotation du yaw du nœud (cube aligné sur le pied). Cette
construction est linéaire — même mécanique que l'Eq. 2 du papier.

### 3.2 Pas normal (ne touche pas le cube)

Un pas classique sur une surface `S_k`, à partir d'un nœud à état joint :

```
J' = { (x', c) : ∃ x, (x, c) ∈ J, x' ∈ x ⊕ Q·K, x' ∈ S_k }
```

Identique à l'expansion actuelle (somme de Minkowski + intersection), mais
appliquée uniquement à la composante `x` ; **`c` est simplement transporté
sans être modifié**. En V-rep : enveloppe convexe des sommets de `J`
translatés par ceux de `K` (comme aujourd'hui), `c` inchangé sur chaque
sommet.

### 3.3 Pas sur le dessus du cube

Même opération que 3.2, mais avec une coupe supplémentaire qui porte sur la
**différence** `x' - c`, pas sur `x'` seul :

```
x' - c - h·n ∈ carré_cube
```

(`h` = épaisseur du cube, `n` = normale). C'est cette coupe oblique dans
l'espace joint qui empêche le faux positif du §2 : elle vérifie qu'il existe
*un* `c` sous le pied, précisément le même `c` que celui transporté depuis la
pose. Reprise du contre-exemple 1D : `J0` donne `x - c ∈ [0.1, 0.3]` ; après
le pas, `x' - c ∈ [0.2, 0.5]` ; la coupe "sur le cube" exige `x' - c ∈
[-0.1, 0.1]` → intersection vide → nœud correctement rejeté.

Un deuxième pas sur le cube impose la même coupe avec le même `c` porté par
`J`, ce qui garantit automatiquement que les deux pieds sont sur le même
cube.

### 3.4 Oubli du cube (retour en 2D)

Dès que le pied quitte le cube pour de bon (garanti simple par la règle
existante "pas de retour sur une surface quittée"), plus aucune contrainte
future ne porte sur `c`. On projette :

```
P = { x : ∃ c, (x, c) ∈ J }
```

En V-rep : on jette la composante `c` des sommets et on reprend l'enveloppe
convexe. Le nœud redevient un polygone 2D classique, et le surcoût
dimensionnel ne dure que le temps où le cube est en jeu.

## 4. Garantie de faisabilité restaurée

En remontant la chaîne des parents, tout `(x_i, c) ∈ J_i` a par construction
un antécédent `(x_{i-1}, c) ∈ J_{i-1}` **avec le même `c`**, jusqu'à
`(z, c) ∈ J0` où `c` est effectivement posable depuis `z`. Il n'y a plus deux
`z` différents pour "expliquer" séparément la pose et le pas : le QP construit
depuis n'importe quel point de `J` en bout de chaîne est faisable par
construction, comme dans le CASSR actuel.

## 5. Modifications à apporter

### 5.1 Structure `NODE`

Ajouter :
- `cubeState : ENUM { NONE, IN_HAND, PLACED_ACTIVE }`
- si `PLACED_ACTIVE` : la composante `c` fait partie de `extremePoints` (le
  polytope est alors sur `(x, c)` et non plus seulement `x`) ; garder une
  trace de la surface virtuelle créée par le cube pour les enfants.
- quand le cube redevient inactif (§3.4), redescendre `cubeState` à un état
  "posé, non contraignant" et ne garder que la projection sur `x`.

### 5.2 `expandNode()`

- Ajouter une branche d'action "poser le cube" (§3.1), disponible seulement
  si `cubeState == IN_HAND`.
- Pour les branches "pas", appliquer la somme de Minkowski **sur la
  composante `x` seulement** quand `J` est joint (§3.2).
- Ajouter la branche "pas sur le cube" avec la coupe couplée `x - c` (§3.3),
  candidate uniquement si le cube est `PLACED_ACTIVE` et si l'intersection
  résultante est non vide.
- Après un pas qui quitte définitivement le support du cube, projeter `J`
  sur `x` (§3.4).

### 5.3 `nodeAlreadyExpanded()`

Comparer les polytopes joints (ou au moins leurs projections sur `x` *et* le
`cubeState`/l'identité du cube porté) — deux nœuds avec le même patch en `x`
mais des poses de cube différentes ne sont pas équivalents.

### 5.4 Heuristique / cost-to-go

`estimateCostToGoal()` doit utiliser la projection de `J` sur `x` (distance
GJK/EPA au patch projeté, comme aujourd'hui). Noter que l'action "poser le
cube" ne rapproche jamais de la cible au sens de cette heuristique : elle est
donc explorée seulement après épuisement des options moins coûteuses à `f`
égal, ce qui peut être lent sur de grands environnements (cas
"local minima"-like). Piste d'amélioration ultérieure : heuristique
consciente du fait qu'un cube en main peut réduire le coût-à-venir face à un
obstacle de hauteur connue.

### 5.5 QP (Eq. 3/6 du papier)

- Ajouter `c` comme variable du programme (une par cube posé sur le chemin
  retenu).
- Contraintes : `c ∈ x_p ⊕ Q·K_cube`, `c ∈ S̃_j` (pose, avec `x_p` le pied
  d'appui au moment de la pose) ; `x_i - c - h·n ∈ carré_cube` pour chaque
  pas `i` marqué "sur le cube".
- Le programme reste linéaire, donc toujours un QP/LP comme aujourd'hui.
- Attention si l'on garde la marge `α` (Eq. 6) : elle ne doit s'appliquer
  qu'aux contraintes de bord de patch, pas à toutes les lignes de `B_i` (sinon
  une surface bornée par un plan des deux côtés devient infaisable dès que
  `α > 0` — remarque valable indépendamment de cette extension, à vérifier
  aussi dans le code actuel).

## 6. Complexité et limites

- Les opérations restent des sommes de Minkowski (V-rep, simple) et des
  intersections (H-rep, simple) comme dans le CASSR actuel, mais en dimension
  4 au lieu de 2 tant que le cube est actif. Le nombre de sommets peut
  croître plus vite ; prévoir un élagage si besoin.
- Avec `n` objets manipulables simultanément actifs, l'état est de dimension
  `2 + 2n`. Ça ne reste tenable que si chaque objet est reprojeté dès qu'il
  quitte la portée du problème (la règle de non-retour aide beaucoup ici),
  donc éviter de garder plusieurs objets actifs longtemps en parallèle dans
  une première implémentation.

## 7. Alternative pragmatique (si le coût de l'état joint est trop élevé)

Garder l'approche naïve du §2 (patch en `x` seul, `T` calculé par simple somme
de Minkowski) et **valider a posteriori** : quand A* atteint le but, résoudre
le QP normalement ; s'il est infaisable à cause d'une contrainte cube
incohérente, marquer la branche et relancer la recherche (ou repartir du
dernier nœud "cube" ambigu). Plus simple à implémenter, mais on perd la
propriété "QP faisable par construction", qui est un argument de correction
important du papier — à documenter clairement si ce choix est fait pour
accélérer un prototype.

## 8. Questions ouvertes

- Construction précise de `K_cube` (cinématique conservative de pose, a
  priori en double appui — même niveau d'approximation que `K` pour les pas).
- Faut-il permettre de reprendre un cube déjà posé (`PLACED_ACTIVE →
  IN_HAND`) ? Si oui, il faut aussi coupler cette reprise à `c` dans l'état
  joint, avec la même logique que la pose.
- Gestion du fait que le cube posé obstrue potentiellement la surface qu'il
  recouvre (explicitement mis de côté dans la discussion initiale, à traiter
  séparément).
