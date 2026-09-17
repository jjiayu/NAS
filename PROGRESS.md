# Progression — réécriture NAS/CASSR

Dernière mise à jour : 2026-09-17.

But de ce fichier : reprendre exactement où on s'est arrêté si la session s'interrompt — mis à jour à chaque étape franchie, pas seulement en fin de phase. Voir [PLAN.md](PLAN.md) pour le pourquoi/l'architecture, qui ne change pas à chaque commit.

## Où on en est là, maintenant

**Étape courante : Stage A, phase 0 — 0a et 0b faits.**
**Prochaine action : 0c (écrire `golden_capture.cpp`).**

Rien n'a encore été créé dans le repo pour la réécriture (pas de `core/`, `planners/`, etc.). Seuls `PLAN.md` et ce fichier existent à ce stade.

## Stage A — parité fonctionnelle

- [ ] 0. Golden references (séquences + QP + perf, 3 scénarios du papier)
  - [x] 0a. Vérifier que le repo actuel compile proprement ici — OK avec `cmake --build build -j2` (jamais plus de 3-4 jobs sur cette machine, RAM très contrainte : ~1.9 Gi libres sur 14 Gi, swap plein — `-j$(nproc)`=12 a été tué par l'OOM killer)
  - [x] 0b. Papier CASSR obtenu — pas de mapping strict à 3 scénarios requis, on capture tous les scénarios d'`environments.hpp` qui marchent (cf. PLAN.md pour les chiffres de référence Table I)
  - [ ] 0c. Écrire `golden_capture.cpp`
  - [ ] 0d. Format JSON dans `tests/golden/`
  - [ ] 0e. Script de bascule des 3 scénarios (édite `constants.hpp` + rebuild)
  - [ ] 0f. Lancer, sanity check des fichiers produits
  - [ ] 0g. Commit golden + outil de capture
- [ ] 1. `talosReachability`
- [ ] 2. `core/geometry` + `core/surface`
- [ ] 3. `core/reachability` (couche 0)
- [ ] 4. `core/node` + `core/expansion`
- [ ] 5. Port CASSR (`planners/astar_search`)
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

- **2026-09-17** : analyse complète du repo actuel (libs, pipeline de reachability, NAS vs CASSR) + analyse de `go2Reachability`/`go2Motion`. Plan complet discuté et arbitré (QP backends, viz, bindings Python, talosReachability, tests, stages A/B, cleanup). `PLAN.md` et `PROGRESS.md` créés et commités (`db33476`). Phase 0 détaillée en sous-étapes 0a-0g dans PLAN.md. Implémentation pas encore démarrée — prochaine action concrète : 0a.
- **2026-09-17** : papier CASSR fourni par l'utilisateur (attaché en conversation, pas de fichier local). Scope de la capture golden élargi : tous les scénarios d'`environments.hpp` qui marchent, pas juste les 3 du papier — B3 de Stage B absorbé dans la phase 0. Repères Table I du papier ajoutés dans PLAN.md pour sanity-check.
- **2026-09-17** : trouvé `/media/stonneau/data/dev/linux/cassr/` et `/media/stonneau/data/dev/linux/cursor/nas/` (essai antérieur non versionné, ~avril 2026, structure proche du plan actuel — geometry2d/3d, a_star, qp_footsteps avec quadprog vendorisé, viz SVG). **Tranché par l'utilisateur : ignorer complètement, ça ne marchait pas.** Ne pas ré-explorer ces dossiers dans une session future.
