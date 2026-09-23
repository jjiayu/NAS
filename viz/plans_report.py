"""Builds one self-contained HTML page showing CASSR plans, from the JSON dumps of
apps/astar_plan (one per scenario) and, for comparison, the plans the old code stored
in tests/golden/<scene>_astar.json.

Usage:  python3 viz/plans_report.py <golden_dir> <out.html> <label> <dumps_dir> [<label> <dumps_dir> ...]
  each <dumps_dir> holds <Scenario>.json as written by astar_plan (see apps/astar_plan/README.md), one directory per
  variant (e.g. the default cost and a rotation cost); the first one is the reference. The page has a selector.
The page needs no server and no network except two Google Fonts; it works offline with fallbacks.
"""
import json
import os
import sys

SCENES = ["NarrowPassage", "Stairs", "TwoFlatSurfaces", "LongStairs", "LongLongStairs", "Flat",
          "LongStairsComplete", "LongStairsExp", "ThreePathsScene", "Stairs_Up_Down", "ThreePathsNAS",
          "Ramp", "SteepRamp", "SlopedGround", "SideSlope"]   # the last four are inclined scenes, new (no old-code plan)

# Old code (same machine, same tool, interleaved runs; docs/paper-deltas.md, "Mesure finale"):
# search ms and expansions of the old behaviour, built from commit f97e445.
OLD_PERF = {
    "NarrowPassage": (136.2, 90), "Stairs": (14.9, 38), "TwoFlatSurfaces": (0.06, 1), "LongStairs": (13.3, 28),
    "LongLongStairs": (44.5, 75), "Flat": (28.2, 10), "LongStairsComplete": (9.8, 24), "LongStairsExp": (89.1, 249),
    "ThreePathsScene": (193.0, 416), "Stairs_Up_Down": (16.4, 33), "ThreePathsNAS": (60.4, 91),
}


def load_variant(dumps, sc):
    with open(os.path.join(dumps, sc + ".json")) as f:
        new = json.load(f)
    return {
        "found": new["path_found"], "qp": new["qp_success"],
        "start": new["start"], "goal": new["goal"],
        "expansions": new["expansions"], "search_ms": new["search_ms"], "qp_ms": new["qp_ms"],
        "surfaces": new["surfaces"],
        "path": [{"patch": n["patch_vertices"], "stance": n["stance_foot"], "yaw": n["foot_yaw"], "surf": n["surface_id"]}
                 for n in new["path"]],
        "feet": new["footsteps"],
    }


def main(golden, out, variants):
    """variants: list of (label, dumps_dir); the first is the reference (default cost)."""
    data = []
    for sc in SCENES:
        vs = {}
        for label, dumps in variants:
            v = load_variant(dumps, sc)
            vs[label] = {k: v[k] for k in v if k != "surfaces"}
        first = load_variant(variants[0][1], sc)
        gpath = os.path.join(golden, sc + "_astar.json")
        old = json.load(open(gpath)) if os.path.exists(gpath) else {"success": False, "nodes": []}
        fp = old.get("footstep_plan", {})
        old_nodes = [{"c": n["centroid"], "yaw": n["foot_yaw"], "stance": n["stance_foot"], "surf": n["surface_id"]}
                     for n in old.get("nodes", [])]
        data.append({
            "name": sc, "surfaces": first["surfaces"], "variants": vs,
            "old": {"found": old.get("success", False), "qp": fp.get("success", False), "nodes": old_nodes,
                    "feet": fp.get("footsteps", []) if fp.get("success", False) else [],
                    "ms": OLD_PERF.get(sc, (0, 0))[0], "exp": OLD_PERF.get(sc, (0, 0))[1], "new_scene": sc not in OLD_PERF},
        })
    html = TEMPLATE.replace("/*DATA*/[]", json.dumps(data, separators=(",", ":")))
    html = html.replace("/*VARIANTS*/[]", json.dumps([lab for lab, _ in variants]))
    with open(out, "w") as f:
        f.write(html)
    print("wrote", out, len(html) // 1024, "KB")


TEMPLATE = r"""<title>Plans CASSR</title>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500&family=IBM+Plex+Sans:wght@400;500;600&display=swap">
<style>
:root {
  --ground: #eef2f5; --panel: #f9fbfc; --ink: #17222d; --muted: #5b6b79; --line: #d3dce4; --grid: #dfe6ec;
  --surf: #b9c8d6; --surf-edge: #7f93a6; --patch-l: #1f6fd1; --patch-r: #d9822b;
  --left: #1f6fd1; --right: #d9822b; --old: #7a4fd6; --ok: #1f8a5b; --bad: #c2412d; --start: #17222d; --goal: #1f8a5b;
  --sans: "IBM Plex Sans", system-ui, -apple-system, "Segoe UI", sans-serif; --mono: "IBM Plex Mono", ui-monospace, Menlo, monospace;
}
@media (prefers-color-scheme: dark) {
  :root:not([data-theme="light"]) {
    --ground: #10171d; --panel: #171f27; --ink: #e3eaf0; --muted: #94a4b3; --line: #2a3541; --grid: #1f2a34;
    --surf: #3a4b5b; --surf-edge: #6d8398; --patch-l: #5aa2ff; --patch-r: #f0a04b;
    --left: #5aa2ff; --right: #f0a04b; --old: #a98cf0; --ok: #4cc38a; --bad: #ef7a66; --start: #e3eaf0; --goal: #4cc38a;
  }
}
:root[data-theme="dark"] {
  --ground: #10171d; --panel: #171f27; --ink: #e3eaf0; --muted: #94a4b3; --line: #2a3541; --grid: #1f2a34;
  --surf: #3a4b5b; --surf-edge: #6d8398; --patch-l: #5aa2ff; --patch-r: #f0a04b;
  --left: #5aa2ff; --right: #f0a04b; --old: #a98cf0; --ok: #4cc38a; --bad: #ef7a66; --start: #e3eaf0; --goal: #4cc38a;
}
* { box-sizing: border-box; }
body { background: var(--ground); color: var(--ink); font-family: var(--sans); font-size: 14px; line-height: 1.5; padding-inline: 16px; padding-block: 20px 40px; }
.wrap { max-width: 1240px; margin: 0 auto; display: grid; gap: 20px; }
header h1 { font-size: 22px; font-weight: 600; margin: 0 0 4px; text-wrap: balance; letter-spacing: -0.01em; }
header p { margin: 0; color: var(--muted); max-width: 70ch; }
.layout { display: grid; grid-template-columns: 220px minmax(0, 1fr); gap: 20px; align-items: start; }
nav { display: grid; gap: 4px; position: sticky; top: env(safe-area-inset-top, 0px); }
nav button { font: inherit; text-align: left; background: transparent; color: var(--ink); border: 1px solid transparent; border-radius: 6px; padding: 7px 10px; cursor: pointer; display: grid; grid-template-columns: 1fr auto; gap: 8px; align-items: baseline; }
nav button:hover { background: var(--panel); border-color: var(--line); }
nav button[aria-current="true"] { background: var(--panel); border-color: var(--ink); }
nav button:focus-visible, .toggles input:focus-visible { outline: 2px solid var(--left); outline-offset: 2px; }
nav button span:first-child { white-space: nowrap; }
nav .n { white-space: nowrap; font-family: var(--mono); font-size: 12px; color: var(--muted); font-variant-numeric: tabular-nums; }
nav .none .n { color: var(--bad); }
.stage { display: grid; gap: 14px; min-width: 0; }
.stage h2 { margin: 0; font-size: 18px; font-weight: 600; display: flex; gap: 10px; align-items: baseline; flex-wrap: wrap; }
.pill { font-family: var(--mono); font-size: 11px; padding: 1px 8px; border-radius: 999px; border: 1px solid var(--line); color: var(--muted); }
.pill.ok { color: var(--ok); border-color: var(--ok); } .pill.bad { color: var(--bad); border-color: var(--bad); }
.stats { display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 0; border-block: 1px solid var(--line); }
.stats div { padding: 8px 12px 8px 0; }
.stats dt { font-size: 11px; letter-spacing: .06em; text-transform: uppercase; color: var(--muted); }
.stats dd { margin: 0; font-family: var(--mono); font-size: 15px; font-variant-numeric: tabular-nums; }
.stats dd small { color: var(--muted); font-size: 12px; }
.toolbar { display: flex; flex-wrap: wrap; gap: 8px 18px; align-items: center; color: var(--muted); font-size: 13px; }
.toggles { display: flex; flex-wrap: wrap; gap: 6px 16px; }
.toggles label { display: inline-flex; gap: 6px; align-items: center; cursor: pointer; color: var(--ink); }
.key { display: inline-flex; gap: 6px; align-items: center; }
.key i { width: 14px; height: 9px; display: inline-block; border-radius: 1px; }
.plot { background: var(--panel); border: 1px solid var(--line); border-radius: 8px; padding: 8px; }
.plot svg { display: block; width: 100%; height: auto; max-height: 68vh; }
.plot .cap { font-size: 12px; color: var(--muted); padding: 2px 6px 0; }
svg text { font-family: var(--mono); fill: var(--muted); }
.grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(220px, 1fr)); gap: 10px; }
.card { font: inherit; text-align: left; color: inherit; background: var(--panel); border: 1px solid var(--line); border-radius: 8px; padding: 8px; cursor: pointer; display: grid; gap: 4px; }
.card:hover { border-color: var(--ink); }
.card:focus-visible { outline: 2px solid var(--left); outline-offset: 2px; }
.card svg { width: 100%; height: 120px; display: block; }
.ch { display: flex; justify-content: space-between; gap: 8px; font-size: 12.5px; }
.ch .n { font-family: var(--mono); color: var(--muted); white-space: nowrap; }
.tablewrap { overflow-x: auto; border: 1px solid var(--line); border-radius: 8px; background: var(--panel); }
table { border-collapse: collapse; width: 100%; font-size: 13px; }
th, td { padding: 7px 12px; text-align: right; white-space: nowrap; border-bottom: 1px solid var(--line); font-variant-numeric: tabular-nums; }
th:first-child, td:first-child { text-align: left; }
th { font-size: 11px; letter-spacing: .05em; text-transform: uppercase; color: var(--muted); font-weight: 500; }
td { font-family: var(--mono); } td:first-child { font-family: var(--sans); }
tr:last-child td { border-bottom: 0; }
tr.sel td { background: color-mix(in srgb, var(--left) 8%, transparent); }
.good { color: var(--ok); } .warn { color: var(--bad); }
h3 { font-size: 15px; margin: 8px 0 -8px; font-weight: 600; }
.note { color: var(--muted); font-size: 12.5px; max-width: 80ch; }
@media (max-width: 880px) {
  .layout { grid-template-columns: minmax(0, 1fr); }
  nav { position: static; display: flex; overflow-x: auto; gap: 6px; padding-bottom: 4px; }
  nav button { flex: 0 0 auto; grid-template-columns: auto auto; border-color: var(--line); }
}
</style>

<div class="wrap">
  <header>
    <h1>Plans CASSR sur 15 scénarios</h1>
    <p>Chaque plan est produit par l'interface minimale <code>astar_plan</code> (recherche + QP des pas) sur la version validée. Le plan de l'ancien code, quand il existe, est superposé en pointillés violets. Les quatre derniers scénarios (Ramp, SteepRamp, SlopedGround, SideSlope) sont des surfaces inclinées, nouvelles : l'ancien code n'en a pas.</p>
  </header>
  <div class="layout">
    <nav id="nav" aria-label="Scénarios"></nav>
    <section class="stage" id="stage"></section>
  </div>
  <h3>Vue d'ensemble (variante choisie)</h3>
  <div class="grid" id="overview"></div>
  <h3>Comparaison avec l'ancien code</h3>
  <div class="tablewrap"><table id="cmp"></table></div>
  <p class="note">Ancien : configuration de l'ancien code, mesurée avec le même outil sur la même machine (recherche seule, moyenne de 10 exécutions alternées). « Pas » = distance horizontale parcourue par les pieds d'après le QP. Les lacets d'un plan peuvent différer de l'ancien sans que les surfaces, les pieds ou la longueur changent : à coût égal, l'ordre de sortie des candidats est un simple départage.</p>
</div>

<script>
const DATA = /*DATA*/[];
const VARIANTS = /*VARIANTS*/[];
const FOOT_L = 0.22, FOOT_W = 0.12;   // indicative outline; the planner only uses the foot size to shrink surfaces
const el = (t, a = {}, k = []) => { const e = document.createElementNS(t === 'svg' || svgTags.has(t) ? 'http://www.w3.org/2000/svg' : 'http://www.w3.org/1999/xhtml', t); for (const [n, v] of Object.entries(a)) e.setAttribute(n, v); for (const c of k) e.append(c); return e; };
const svgTags = new Set(['g','path','rect','circle','line','text','polygon','polyline','title','defs','marker']);
const fmt = (v, d = 1) => v.toFixed(d).replace('.', ',');
// the scene as seen in the selected variant (cost setting)
const view = i => Object.assign({}, DATA[i], DATA[i].variants[state.v]);
const rotation = s => { let t = 0; for (let i = 1; i < s.path.length; i++) { let d = Math.abs(s.path[i].yaw - s.path[i - 1].yaw) % (2 * Math.PI); if (d > Math.PI) d = 2 * Math.PI - d; t += d; } return t * 180 / Math.PI; };
const walked = feet => { let d = 0; for (let i = 1; i < feet.length; i++) d += Math.hypot(feet[i][0] - feet[i-1][0], feet[i][1] - feet[i-1][1]); return d; };
const newFeet = s => s.feet.map(f => f.position);
const state = { v: VARIANTS[VARIANTS.length > 1 ? 1 : 0], i: 0, old: true, patches: true, nums: true };
try { const s = JSON.parse(localStorage.getItem('plans-state-2') || '{}'); if (Number.isInteger(s.i) && s.i < DATA.length) state.i = s.i; if (VARIANTS.includes(s.v)) state.v = s.v; for (const k of ['old','patches','nums']) if (typeof s[k] === 'boolean') state[k] = s[k]; } catch (e) {}
const save = () => { try { localStorage.setItem('plans-state-2', JSON.stringify(state)); } catch (e) {} };

function niceStep(span) { const raw = span / 6, p = Math.pow(10, Math.floor(Math.log10(raw))); const m = raw / p; return (m < 1.5 ? 1 : m < 3.5 ? 2 : m < 7.5 ? 5 : 10) * p; }

function foot(x, y, yaw, cls, label, fs, opts = {}) {
  const g = el('g', { transform: `translate(${x} ${-y}) rotate(${-yaw * 180 / Math.PI})` });
  const r = el('rect', { x: -FOOT_L / 2, y: -FOOT_W / 2, width: FOOT_L, height: FOOT_W, rx: 0.015, fill: opts.hollow ? 'none' : `var(--${cls})`, 'fill-opacity': opts.hollow ? 0 : 0.92, stroke: `var(--${cls})`, 'stroke-width': opts.hollow ? 1.6 : 1, 'vector-effect': 'non-scaling-stroke' });
  if (opts.hollow) r.setAttribute('stroke-dasharray', '4 3');
  g.append(r);
  g.append(el('line', { x1: 0, y1: 0, x2: FOOT_L / 2, y2: 0, stroke: opts.hollow ? `var(--${cls})` : 'var(--panel)', 'stroke-width': 1.4, 'vector-effect': 'non-scaling-stroke' }));
  if (label !== undefined && state.nums && !opts.hollow) { const t = el('text', { x: 0, y: fs * 0.35, 'text-anchor': 'middle', 'font-size': fs * 0.9, transform: `rotate(${yaw * 180 / Math.PI})`, fill: 'var(--panel)', style: 'fill:var(--panel);font-weight:500' }); t.textContent = label; g.append(t); }
  return g;
}

function topView(s, mini = false) {
  const pts = s.surfaces.flat();
  let xs = pts.map(p => p[0]), ys = pts.map(p => p[1]);
  const all = [...s.feet.map(f => f.position), s.start, s.goal, ...s.old.feet];
  xs = xs.concat(all.map(p => p[0])); ys = ys.concat(all.map(p => p[1]));
  const pad = 0.4, x0 = Math.min(...xs) - pad, x1 = Math.max(...xs) + pad, y0 = Math.min(...ys) - pad, y1 = Math.max(...ys) + pad;
  const W = x1 - x0, H = y1 - y0, fs = Math.max(W, H) * 0.02;
  const svg = el('svg', { viewBox: `${x0} ${-y1} ${W} ${H}`, role: 'img', 'aria-label': `Vue de dessus, ${s.name}` });
  const step = niceStep(Math.max(W, H));
  const zmean = v => v.reduce((a, p) => a + p[2], 0) / v.length;
  if (!mini) {
  for (let x = Math.ceil((x0 + fs * 3) / step) * step; x <= x1 - fs * 3; x += step) { svg.append(el('line', { x1: x, y1: -y1, x2: x, y2: -y0, stroke: 'var(--grid)', 'stroke-width': 1, 'vector-effect': 'non-scaling-stroke' })); const t = el('text', { x: x + fs * 0.2, y: -y0 - fs * 0.35, 'font-size': fs * 0.72 }); t.textContent = fmt(x, step < 1 ? 1 : 0) + ' m'; svg.append(t); }
  for (let y = Math.ceil((y0 + fs * 2) / step) * step; y <= y1 - fs; y += step) { svg.append(el('line', { x1: x0, y1: -y, x2: x1, y2: -y, stroke: 'var(--grid)', 'stroke-width': 1, 'vector-effect': 'non-scaling-stroke' })); const t = el('text', { x: x0 + fs * 0.3, y: -y - fs * 0.25, 'font-size': fs * 0.72 }); t.textContent = fmt(y, step < 1 ? 1 : 0) + ' m'; svg.append(t); }
  }
  const zs = s.surfaces.map(zmean), zmin = Math.min(...zs), zmax = Math.max(...zs);
  s.surfaces.forEach((v, k) => {
    const t = zmax > zmin ? (zmean(v) - zmin) / (zmax - zmin) : 0.5;
    svg.append(el('polygon', { points: v.map(p => `${p[0]},${-p[1]}`).join(' '), fill: 'var(--surf)', 'fill-opacity': 0.3 + 0.55 * t, stroke: 'var(--surf-edge)', 'stroke-width': 1, 'vector-effect': 'non-scaling-stroke' }, [el('title', {}, [`surface ${k}, hauteur ${fmt(Math.min(...v.map(p => p[2])), 2)} à ${fmt(Math.max(...v.map(p => p[2])), 2)} m`])]));
    if (mini || s.surfaces.length > 8) return; // many narrow surfaces (stairs): labels would pile up, the hover title has them
    const cx = v.reduce((a, p) => a + p[0], 0) / v.length, cy = v.reduce((a, p) => a + p[1], 0) / v.length;
    const tx = el('text', { x: cx, y: -cy, 'text-anchor': 'middle', 'font-size': fs * 0.95, style: 'opacity:.85' }); { const lo = Math.min(...v.map(p => p[2])), hi = Math.max(...v.map(p => p[2])); tx.textContent = `#${k}` + (zmax > zmin || hi - lo > 0.01 ? (hi - lo > 0.01 ? `  z=${fmt(lo, 2)}→${fmt(hi, 2)}` : `  z=${fmt(lo, 2)}`) : ''); } svg.append(tx);
  });
  if (state.patches && !mini) s.path.forEach((n, k) => { if (k === 0 || !n.patch.length) return; svg.append(el('polygon', { points: n.patch.map(p => `${p[0]},${-p[1]}`).join(' '), fill: `var(--patch-${n.stance ? 'r' : 'l'})`, 'fill-opacity': 0.09, stroke: `var(--patch-${n.stance ? 'r' : 'l'})`, 'stroke-opacity': 0.35, 'stroke-width': 1, 'vector-effect': 'non-scaling-stroke' })); });
  if (!mini && state.old && s.old.feet.length) {
    svg.append(el('polyline', { points: s.old.feet.map(p => `${p[0]},${-p[1]}`).join(' '), fill: 'none', stroke: 'var(--old)', 'stroke-width': 1.4, 'stroke-dasharray': '5 4', 'vector-effect': 'non-scaling-stroke' }));
    s.old.feet.forEach((p, i) => svg.append(foot(p[0], p[1], s.old.nodes[i] ? s.old.nodes[i].yaw : 0, 'old', undefined, fs, { hollow: true })));
  } else if (!mini && state.old && s.old.nodes.length > 1) {
    s.old.nodes.slice(1).forEach(n => { const c = n.c, d = fs * 0.5; svg.append(el('polygon', { points: `${c[0]},${-c[1]-d} ${c[0]+d},${-c[1]} ${c[0]},${-c[1]+d} ${c[0]-d},${-c[1]}`, fill: 'none', stroke: 'var(--old)', 'stroke-width': 1.4, 'vector-effect': 'non-scaling-stroke' })); });
  }
  if (s.feet.length) {
    svg.append(el('polyline', { points: s.feet.map(f => `${f.position[0]},${-f.position[1]}`).join(' '), fill: 'none', stroke: 'var(--muted)', 'stroke-width': 1, 'vector-effect': 'non-scaling-stroke', 'stroke-opacity': 0.8 }));
    s.feet.forEach((f, i) => svg.append(foot(f.position[0], f.position[1], f.foot_yaw, f.stance_foot ? 'right' : 'left', mini ? undefined : String(i), fs)));
  } else if (s.found) {
    s.path.slice(1).forEach(n => { const c = n.patch.reduce((a, p) => [a[0] + p[0] / n.patch.length, a[1] + p[1] / n.patch.length], [0, 0]); svg.append(foot(c[0], c[1], n.yaw, n.stance ? 'right' : 'left', undefined, fs)); });
  }
  svg.append(el('circle', { cx: s.start[0], cy: -s.start[1], r: fs * 0.45, fill: 'var(--start)', stroke: 'var(--panel)', 'stroke-width': 1.5, 'vector-effect': 'non-scaling-stroke' }, [el('title', {}, ['départ'])]));
  svg.append(el('circle', { cx: s.goal[0], cy: -s.goal[1], r: fs * 0.7, fill: 'none', stroke: 'var(--goal)', 'stroke-width': 2, 'vector-effect': 'non-scaling-stroke' }, [el('title', {}, ['but'])]));
  svg.append(el('circle', { cx: s.goal[0], cy: -s.goal[1], r: fs * 0.18, fill: 'var(--goal)' }));
  return svg;
}

function sideView(s) {
  if (!s.found || !s.feet.length) return null;
  const px = s.feet.map(f => f.position);
  const spanX = Math.max(...px.map(p => p[0])) - Math.min(...px.map(p => p[0])), spanY = Math.max(...px.map(p => p[1])) - Math.min(...px.map(p => p[1]));
  const ax = spanX >= spanY ? 0 : 1, axn = ax ? 'y' : 'x';
  const zs = [...s.surfaces.flat().map(p => p[2]), ...px.map(p => p[2])];
  const a0 = Math.min(...s.surfaces.flat().map(p => p[ax]), ...px.map(p => p[ax])) - 0.2, a1 = Math.max(...s.surfaces.flat().map(p => p[ax]), ...px.map(p => p[ax])) + 0.2;
  const zmin = Math.min(...zs), zmax = Math.max(...zs);
  if (zmax - zmin < 0.02) return null; // flat scene: nothing to see
  const zp = Math.max((zmax - zmin) * 0.25, 0.08), z0 = zmin - zp, z1 = zmax + zp, W = a1 - a0, H = z1 - z0;
  const fs = Math.max(W, H * 4) * 0.02;
  // z is stretched so steps of ~10 cm read: scale factor k on z
  const k = Math.min(W / H * 0.28, 6), Hs = H * k;
  const svg = el('svg', { viewBox: `${a0} ${-z1 * k} ${W} ${Hs}`, role: 'img', 'aria-label': `Profil de hauteur, ${s.name}` });
  s.surfaces.forEach((v, i) => {
    const zlo = Math.min(...v.map(p => p[2])), zhi = Math.max(...v.map(p => p[2]));
    svg.append(el('polygon', { points: v.map(p => `${p[ax]},${-p[2] * k}`).join(' '), fill: 'var(--surf)', 'fill-opacity': 0.35, stroke: 'var(--surf-edge)', 'stroke-width': 2.5, 'stroke-linejoin': 'round', 'vector-effect': 'non-scaling-stroke' }, [el('title', {}, [`surface ${i}, z de ${fmt(zlo, 2)} à ${fmt(zhi, 2)} m`])]));
  });
  for (const zv of [zmin, zmax]) { const t = el('text', { x: a0 + fs * 0.3, y: -zv * k - fs * 0.5, 'font-size': fs }); t.textContent = `z=${fmt(zv, 2)}`; svg.append(t); }
  if (state.old && s.old.feet.length) s.old.feet.forEach(p => svg.append(el('circle', { cx: p[ax], cy: -p[2] * k, r: fs * 0.62, fill: 'none', stroke: 'var(--old)', 'stroke-width': 1.5, 'stroke-dasharray': '3 2', 'vector-effect': 'non-scaling-stroke' })));
  s.feet.forEach((f, i) => { svg.append(el('circle', { cx: f.position[ax], cy: -f.position[2] * k, r: fs * 0.45, fill: `var(--${f.stance_foot ? 'right' : 'left'})` }, [el('title', {}, [`pas ${i}, ${axn}=${fmt(f.position[ax], 2)} m, z=${fmt(f.position[2], 2)} m`])])); });
  return { svg, cap: `Profil : ${axn} (m) en abscisse, hauteur z en ordonnée, étirée ×${fmt(k, 0)} pour lire les marches.` };
}

function renderNav() {
  const nav = document.getElementById('nav'); nav.replaceChildren();
  DATA.forEach((_, i) => { const s = view(i); const b = el('button', { type: 'button', 'aria-current': String(i === state.i), class: s.found ? '' : 'none' }, [el('span', {}, [s.name]), el('span', { class: 'n' }, [s.found ? `${s.path.length - 1} pas` : 'sans chemin'])]); b.onclick = () => { state.i = i; save(); render(); }; nav.append(b); });
}

function renderStage() {
  const s = view(state.i), st = document.getElementById('stage'); st.replaceChildren();
  const h = el('h2', {}, [s.name, el('span', { class: 'pill ' + (s.found ? 'ok' : 'bad') }, [s.found ? 'chemin trouvé' : 'aucun chemin']), el('span', { class: 'pill ' + (s.qp ? 'ok' : 'bad') }, [s.qp ? 'QP résolu' : (s.found ? 'QP en échec' : 'QP non lancé')])]);
  st.append(h);
  const wn = walked(newFeet(s)), wo = walked(s.old.feet);
  const dl = el('dl', { class: 'stats' });
  const stat = (k, v, small) => dl.append(el('div', {}, [el('dt', {}, [k]), el('dd', {}, [v, small ? el('small', {}, [' ' + small]) : ''])]));
  stat('Pas', s.found ? String(s.path.length - 1) : '–', s.old.found ? `ancien ${s.old.nodes.length - 1}` : (s.old.new_scene ? 'scène nouvelle' : 'ancien : aucun chemin'));
  stat('Expansions', String(s.expansions), s.old.new_scene ? '' : `ancien ${s.old.exp}`);
  stat('Recherche', fmt(s.search_ms) + ' ms', s.old.new_scene ? '' : `ancien ${fmt(s.old.ms)} ms`);
  stat('QP', s.found ? fmt(s.qp_ms, 2) + ' ms' : '–', s.old.qp ? 'ancien QP résolu' : (s.old.found ? 'ancien QP en échec' : ''));
  stat('Rotation totale', s.found ? fmt(rotation(s), 0) + '°' : '–', '');
  stat('Distance parcourue', wn ? fmt(wn, 2) + ' m' : '–', wo ? `ancien ${fmt(wo, 2)} m` : '');
  st.append(dl);
  const tb = el('div', { class: 'toolbar' });
  const tg = el('div', { class: 'toggles' });
  [['old', 'Plan ancien'], ['patches', 'Zones d\'appui'], ['nums', 'Numéros']].forEach(([k, l]) => { const id = 'tg-' + k; const inp = el('input', { type: 'checkbox', id }); inp.checked = state[k]; inp.onchange = () => { state[k] = inp.checked; save(); renderPlots(); }; tg.append(el('label', { for: id }, [inp, l])); });
  tb.append(tg);
  if (VARIANTS.length > 1) { const sel = el('div', { class: 'toggles', role: 'radiogroup', 'aria-label': 'Coût' }); VARIANTS.forEach((v, k) => { const id = 'var-' + k; const inp = el('input', { type: 'radio', name: 'variant', id }); inp.checked = v === state.v; inp.onchange = () => { state.v = v; save(); render(); }; sel.append(el('label', { for: id }, [inp, v])); }); tb.append(sel); }
  const key = (c, t) => el('span', { class: 'key' }, [el('i', { style: `background:var(--${c})` }), t]);
  tb.append(key('left', 'pied gauche'), key('right', 'pied droit'), key('old', 'ancien (pointillés)'));
  st.append(tb);
  st.append(el('div', { class: 'plot', id: 'plot-top' }), el('div', { class: 'plot', id: 'plot-side' }));
  renderPlots();
}

function renderPlots() {
  const s = view(state.i);
  const top = document.getElementById('plot-top'); top.replaceChildren(topView(s), el('div', { class: 'cap' }, ['Vue de dessus, mètres. Les surfaces sont rétrécies de la marge du pied (là où le centre du pied peut se poser) ; teinte plus foncée = plus haut. Pied dessiné 22 × 12 cm, indicatif ; le trait clair marque l\'avant du pied. Cercle vert = but.']));
  const sv = sideView(s), side = document.getElementById('plot-side');
  if (sv) { side.hidden = false; side.replaceChildren(sv.svg, el('div', { class: 'cap' }, [sv.cap])); } else side.hidden = true;
}

function renderOverview() {
  const box = document.getElementById('overview'); box.replaceChildren();
  DATA.forEach((_, i) => {
    const s = view(i);
    const card = el('button', { type: 'button', class: 'card', 'aria-label': `${s.name}, ouvrir` });
    card.onclick = () => { state.i = i; save(); render(); window.scrollTo({ top: 0, behavior: 'smooth' }); };
    const head = el('div', { class: 'ch' }, [el('span', {}, [s.name]), el('span', { class: 'n' }, [s.found ? `${s.path.length - 1} pas · ${fmt(rotation(s), 0)}°` : 'sans chemin'])]);
    card.append(head, s.found || s.surfaces.length ? topView(s, true) : el('div'));
    box.append(card);
  });
}

function renderTable() {
  const t = document.getElementById('cmp'); t.replaceChildren();
  const head = ['Scénario', 'Pas', 'Pas ancien', 'Expansions', 'Ancien', 'Recherche ms', 'Ancien ms', 'Rapport', 'Parcouru m', 'Ancien m', 'QP'];
  t.append(el('thead', {}, [el('tr', {}, head.map(h => el('th', {}, [h])))]));
  const tb = el('tbody'); let tn = 0, to = 0;
  DATA.forEach((_, i) => {
    const s = view(i);
    const wn = walked(newFeet(s)), wo = walked(s.old.feet), r = s.search_ms / s.old.ms; const nw = s.old.new_scene; if (s.found && !nw) { tn += s.search_ms; to += s.old.ms; }
    const cells = [s.name, s.found ? s.path.length - 1 : '–', s.old.found ? s.old.nodes.length - 1 : '–', s.expansions, nw ? '–' : s.old.exp, fmt(s.search_ms), nw ? '–' : fmt(s.old.ms), (s.found && !nw ? fmt(r, 2) + '×' : '–'), wn ? fmt(wn, 2) : '–', wo ? fmt(wo, 2) : '–', s.found ? (s.qp ? (s.old.qp ? 'ok' : 'ok (ancien : échec)') : 'échec') : '–'];
    const tr = el('tr', { class: i === state.i ? 'sel' : '' }, cells.map((c, j) => { const td = el('td', j === 7 && s.found && !nw ? { class: r > 1.15 ? 'warn' : 'good' } : {}); td.textContent = String(c); return td; }));
    tb.append(tr);
  });
  const sum = el('tr', {}, ['Total (11 scènes anciennes avec chemin)', '', '', '', '', fmt(tn), fmt(to), fmt(tn / to, 2) + '×', '', '', ''].map((c, j) => { const td = el('td', j === 7 ? { class: 'good' } : {}); td.textContent = c; td.style.fontWeight = '600'; return td; }));
  tb.append(sum); t.append(tb);
}

function render() { renderNav(); renderStage(); renderOverview(); renderTable(); }
render();
</script>
"""

if __name__ == "__main__":
    if len(sys.argv) < 5 or len(sys.argv) % 2 != 1:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2], list(zip(sys.argv[3::2], sys.argv[4::2])))
