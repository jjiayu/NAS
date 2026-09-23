# Brouillon : écarts entre le papier CASSR (arXiv:2603.02989) et le code

Brouillon établi le 2026-09-19 à partir de la relecture du papier (sections IV à VII, tableau I) et de tout ce qui a été mesuré dans le portage. Chaque point donne ce que dit le papier, ce que fait le code, les preuves, et la correction proposée. **Le sens de la correction (papier ou code) est à décider par l'auteur** ; les points marqués « décision » ne sont pas tranchés. Aucun point n'a été modifié dans le papier.

Statuts : **[papier]** le pseudo-code ou le texte est à corriger ; **[code corrigé]** le code s'écartait du papier et a été corrigé ; **[décision]** écart réel, sens de la correction à choisir ; **[précision]** le papier est incomplet, à préciser ; **[à vérifier]** non contrôlé.

## 1. [papier] Algorithme 2, `expandNode`, lignes 3 à 4 et 9 : quel lacet tourne le polytope

- **Papier** : pour chaque `θ_i`, `currentYaw = yaw + θ_i`, `R_ei = rotateKinConstraint(surfaceId, currentYaw)`, région de l'enfant calculée avec, et l'enfant reçoit `currentYaw` : un patch différent par `θ_i`.
- **Règle (confirmée par l'auteur)** : un pied posé l'est avec une orientation ; celle-ci contraint tous ses enfants par le même polytope, quelle que soit l'orientation de chaque enfant.
- **Code** : polytope tourné par le lacet du parent seulement, une somme de Minkowski et un patch par surface, partagés par les 7 enfants ; `θ_i` ne donne à l'enfant que son propre lacet.
- **Cohérence** : c'est ce que disent l'Eq. 2 (« lacet courant ») et l'Eq. 4 (`A_{i-1}`, rotation du pas précédent).
- **Correction proposée** : calculer `R_e = rotateKinConstraint(surfaceId, yaw)` et la région **avant** la boucle sur les lacets ; dans la boucle, ne faire que `createChildNode(effectorId, this, j, reachableContact, yaw + θ_i)`. Remarque à ajouter : la somme de Minkowski est faite une fois par parent, non une fois par lacet.

## 2. [papier] `nodeAlreadyExpanded` (V-B.3) : le critère exact

- **Papier** : même pied, même surface, et « distance euclidienne entre les centres et le périmètre des patchs sous 2 cm » (formule absente, périmètre non défini, lacet absent).
- **Code** : même surface, même pied, même bin de lacet `int(yaw/10°)`, et `d(A,B) = max( max_{v∈A} dist(v, ∂B), max_{v∈B} dist(v, ∂A) ) < 2 cm`. L'ancien code comparait des cellules `int(x/0,02)` sur le centroïde et le périmètre du prisme.
- **Preuves** : audit de similarité (105 quasi-doublons non fusionnés par les cellules sur NarrowPassage), comparaison des trois critères (`docs/paper-deltas.md`).
- **Correction proposée** : écrire la formule du code, y mettre le lacet, et dire que le bin de lacet tronque vers zéro (le bin 0 fait 20° de large) ou le corriger dans le code.

## 3. [précision] Poids de l'heuristique (V-B.5)

Le papier dit que l'EPA est « une mise à l'échelle » de la borne inférieure sur le nombre de pas, qui rend l'heuristique non admissible, sans donner la valeur. Le code utilise **10** (`heuristic_weight`), non appliqué à l'heuristique euclidienne. À écrire.

## 4. [précision] Heuristique du nœud de départ

Le nœud de départ est un point (pas de patch, l'EPA demande au moins 3 sommets) : le code utilise la distance euclidienne au but. À écrire.

## 5. [décision prise, travail à faire] Robustesse du placement du pied (VI)

- **Papier** : « Our algorithm allows stepping on the edge of contact surfaces, where the entire foot is not guaranteed to fit. […] robustness can be obtained by avoiding these positions if possible » (le paramètre `α` de l'Eq. 6).
- **Code** : chaque surface est rétrécie de la demi-longueur et de la demi-largeur du pied (0,22 m par défaut, donc 11 cm) avant toute recherche, si bien que le centre du pied ne peut pas se poser à moins de 11 cm d'un bord, en plus de la marge `α`. Effet visible : NarrowPassage, dont le passage de 24 cm devient un passage de ~2 cm.
- **Décision de l'auteur (2026-09-20)** : ce rétrécissement devient un **paramètre de la config** : l'utilisateur donne un seuil `d` en cm et les surfaces réellement utilisées par le problème sont réduites d'autant. Les modalités (par surface ou global, isotrope ou selon x et y, valeur par défaut) sont à définir plus tard. Noté en TODO dans `PLAN.md` (section « Différé »).

## 6. [décision] Objectif du QP (Eq. 3)

- **Papier** : `c(X) = Σ_{i=2}^{l} (x_i − x_{i−2})²`. Les pas alternent (`x_0` pied droit de départ, `x_1` gauche, `x_2` droit...) : chaque terme compare un pied à **sa propre position précédente**, une foulée complète.
- **Code** : la même somme, plus un premier terme `(x_1 − x_0)²` qui compare deux pieds **différents** (le premier pas gauche au pied droit de départ), un demi-pas. Le premier pas n'a pas de pied gauche précédent connu, le papier ne le pénalise donc pas ; le code lui ajoute une pénalité qui le tire vers `x_0`, sans le faire pour les autres demi-pas.
- **Décision** : garder le terme (comportement de l'ancien code) ou le retirer pour coller au papier. En attente.

## 7. [décision prise, faite] But du QP : position ou surface (Eq. 3 et 6)

Le code fixait le dernier pas au but (égalité) et n'appliquait les contraintes de surface qu'aux pas intermédiaires ; les Eq. 3 et 6 imposent `x_i ∈ R ∩ F_i` pour tous les pas, sans égalité au but. Décision de l'auteur (2026-09-20) : laisser le choix d'une **surface** ou d'une **position** en cible, au niveau du problème et du QP. Fait : voir « But par position ou par surface » dans `docs/paper-deltas.md`. Avec une surface, le QP est exactement celui des Eq. 3 et 6 (dernier pas dans `R ∩ F_l`).

## 8. [code corrigé] Rotation de la surface de contact (Eq. 2)

- **Papier** : `Q` correspond au lacet **et à la rotation de la surface de contact**.
- **Code (ancien)** : lacet autour de z seulement.
- **Corrigé** : `Q = R_tilt · R_z(lacet)`, dans la recherche et dans le QP ; identique sur les surfaces horizontales (bit à bit). Voir la section « Surfaces inclinées » de `docs/paper-deltas.md`.

## 9. [code corrigé] Polytope du QP (Eq. 1, 4 et 5)

- **Papier** : un polytope est l'enveloppe convexe de ses points extrêmes (Eq. 1) ; le QP utilise sa forme en inégalités (VI).
- **Code (ancien)** : demi-espaces tirés du plan de chaque face du maillage `.obj` avec ses trois premiers sommets, faux sur 22 quadrilatères non plans (jusqu'à 29 cm) : les pas sortaient du polytope de 0,1 à 7 mm.
- **Corrigé** : demi-espaces de l'enveloppe convexe des sommets. Aucun changement du papier, mais **les fichiers `.obj` d'atteignabilité contiennent des quadrilatères non plans** : à signaler à qui les génère.

## 10. [code corrigé] « Succès » du QP

Un solveur à ensemble actif peut renvoyer « optimal » avec un résidu de 47 µm. Le code vérifie maintenant les résidus (≤ 1e-6 m), resserre la régularisation, sinon déclare un échec. À mentionner si le papier fait état de la faisabilité garantie.

## 11. [précision] Départage des ex æquo dans `getCheapestNode`

Des nœuds ont un f égal en exact ; le code compare f arrondi au nanomètre puis l'ordre de création (déterministe, identique d'un état de tas à l'autre). Un ordre différent change le nombre d'expansions et parfois le plan (mesuré). À dire si le papier compare des nombres de nœuds.

## 12. [à vérifier] Nombres de nœuds du tableau I

Avec rotation : NarrowPassage 88 nœuds / 29 pas dans le papier, 98 expansions / 29 pas chez nous (l'ancien code : 90) ; minima locaux (ThreePathsNAS) 92 nœuds / 19 pas dans le papier, 115 expansions / 19 pas chez nous (l'ancien : 91 à 93 selon l'état du tas). Les nombres de pas coïncident ; les nombres de nœuds dépendent de l'ordre des ex æquo et du critère de fusion.

## 13. [vérifié] `hasContactSurfaceNotBeenLeft` (V-B.4)

Implémenté comme `cycle_path_detection` (historique des surfaces par pied). Vérifié de l'extérieur, sur la chaîne des parents de chaque nœud développé : 0 cycle sur 9486 nœuds (10 scènes en EPA, 3 en euclidien, un sol dupliqué) ; témoin négatif : 466 cycles sur 3000 nœuds sans la détection. Voir « Détection des cycles » dans `docs/paper-deltas.md`. Une réserve théorique, non observée : l'historique est copié à la création d'un nœud et n'est pas mis à jour quand un nœud fusionné change de parent.

## 14. [précision] Convention du lacet

Le papier donne le lacet « en radians », sans sens ni axe. Code : positif dans le sens inverse des aiguilles d'une montre, autour de la normale de la surface après inclinaison. Le signe compte (inverser le lacet fait violer les contraintes de 0,4 à 1 m).
