# Progression — réécriture NAS/CASSR

Dernière mise à jour : 2026-09-17.

But de ce fichier : reprendre exactement où on s'est arrêté si la session s'interrompt — mis à jour à chaque étape franchie, pas seulement en fin de phase. Voir [PLAN.md](PLAN.md) pour le pourquoi/l'architecture, qui ne change pas à chaque commit.

## Où on en est là, maintenant

**Étape courante : aucune — planification terminée, implémentation pas commencée.**
**Prochaine action : Stage A, phase 0 (golden references).**

Rien n'a encore été créé dans le repo pour la réécriture (pas de `core/`, `planners/`, etc.). Seuls `PLAN.md` et ce fichier existent à ce stade.

## Stage A — parité fonctionnelle

- [ ] 0. Golden references (séquences + QP + perf, 3 scénarios du papier)
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

- **2026-09-17** : analyse complète du repo actuel (libs, pipeline de reachability, NAS vs CASSR) + analyse de `go2Reachability`/`go2Motion`. Plan complet discuté et arbitré (QP backends, viz, bindings Python, talosReachability, tests, stages A/B, cleanup). `PLAN.md` et `PROGRESS.md` créés. Implémentation pas encore démarrée.
