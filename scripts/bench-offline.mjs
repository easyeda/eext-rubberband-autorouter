#!/usr/bin/env node
/**
 * bench-offline.mjs — reproducible OFFLINE baseline benchmark (no EDA client, no network).
 *
 *   (A) routing speed + completion rate of PlanningEngine.plan() on a seeded synthetic board
 *   (B) quality of the "tighten existing routing" feature, driven through the REAL production
 *       entry (src/worker/planning-worker.ts, message type 'tighten') on two inputs:
 *         b1 = the routed result of phase A run #1
 *         b2 = a hand-built "deliberately bad" trace set (90-degree staircases, 45-degree zig-zags,
 *              U-detours, a T-junction net, a via net, bottom-layer net, jittery wiggle) on a board
 *              that is DRC-clean BEFORE tightening
 *
 * USAGE
 *   node scripts/bench-offline.mjs [--seed 1] [--nets 80] [--runs 3] [--budget 60000]
 *                                  [--layers 2] [--phases a,b1,b2] [--tighten-world prod|clean]
 *                                  [--tighten-timeout 300000] [--out file.json] [--verbose]
 *   --seed N            PRNG seed for the board generators (default 1). Same seed => same board.
 *   --nets N            routed nets in the synthetic routing board, incl. one GND and one VCC net
 *                       (default 80)
 *   --runs N            repetitions of every phase (fresh engine / fresh module instance each) (default 3)
 *   --budget MS         PlanningInput.budgetMs for phase A (default 60000)
 *   --layers N          PlanningInput.copperLayers (default 2)
 *   --phases LIST       subset of a,b1,b2 (default all). b1 needs a.
 *   --tighten-world M   prod (default): baseWorld mimics topological-router.tightenExistingRouting,
 *                       i.e. static copper in "tighten" clearance口径 PLUS every existing track of
 *                       every net (as `line:` obstacles, including the runs being tightened) and
 *                       existing vias. clean: only board outline / keepouts / foreign tracks / vias.
 *   --tighten-timeout MS  after this long a 'cancel' message is sent to the tighten worker (default 300000)
 *   --out FILE          also write the JSON document to FILE
 *   --verbose           enable engine logging (stderr). Default: quiet.
 *   Stdout = one pretty-printed JSON document. Everything else goes to stderr.
 *   Exit code 0 = success; 1 = engine threw / bad arguments (partial JSON with an "error" field is still printed).
 *
 * UNITS: all coordinates, lengths, clearances and times-of-geometry are mil (1 mil = 0.0254 mm).
 *        Times are milliseconds of wall clock.
 *
 * METRICS
 *   wallMs               wall time of engine.plan() (A) or of the whole worker 'tighten' task (B)
 *   nets.routed/total    result.routedNets.length / input.networks.length ; failed/partial = engine lists
 *   pads.routed/total    sum over nets of the last onNetUpdate `pads` stats (nets that never reported
 *                        count as 0 routed of n.pads.length), same as batch-route-all.mjs
 *   vias                 result.vias.length
 *   lengthMil            sum of renderedLength() over all prims (arc = |sweep|*r)
 *   lines / arcs         primitive counts
 *   sharp45 / sharp89    SHARP CORNER = a point where exactly two prims of the SAME net and SAME layer
 *                        meet (endpoints clustered within 0.8 mil), both are LINES (no arc smoothing
 *                        the joint), and the turn angle between the two travel directions is >= 45 deg
 *                        (sharp45) / >= 89 deg (sharp89, a subset of sharp45). 0 deg = straight on.
 *                        T-junctions (degree >= 3) and layer changes are not corners.
 *   drc (drcAudit)       scripts/audit-lib.mjs drcAudit(): violated net pairs. NOTE it compares
 *                        CENTER-LINE sample distances (trace width ignored) and (for non-POLYGON pads)
 *                        a pad box with half the real half-extents, so it is lenient; kept because it is the
 *                        repo's audit.
 *   drcEdge (own)        strict edge-to-edge audit written here: track width included, real pad
 *                        shapes (RECT/ROUNDRECT/POLYGON box, ELLIPSE/OVAL capsule), vias, static
 *                        foreign tracks and board outline. Violation = clearance < rule - 0.5 mil
 *                        (same slack as drcAudit); counted per (netA,netB,kind) pair like drcAudit.
 *                        Rules: trackToTrack, trackToPad (pads+vias), trackToBoard.
 *                        Not checked: via-pad, via-via, hole-hole, keepout fills.
 *   joints (jointAudit)  audit-lib jointAudit(): gaps / sharp(>30 deg, arcs included) / bigArcs. Only
 *                        meaningful for chain-ordered prim lists (phase A).
 *   tightenDone          adopted / rounded / solved / rewritten / solveAttempts / solveMs as posted by
 *                        the worker (saved = total mil removed, total = number of runs)
 *   resultHash           sha1 of the rounded (0.01 mil) prims — equal hashes mean identical geometry
 *
 * Phase B AFTER state is built the way the main thread does: `tightenRun` with non-null prims replaces
 * that run; `netRewrite` replaces ALL runs of that net.
 *
 * The script never touches src/ ; it bundles planning-worker.ts + planning-engine.ts with esbuild in
 * memory. Every run imports a fresh copy of the bundle (new module instance => no cross-run engine state).
 */
import * as esbuild from 'esbuild';
import * as fs from 'fs';
import * as path from 'path';
import * as crypto from 'crypto';
import { fileURLToPath } from 'url';
import { drcAudit, jointAudit } from './audit-lib.mjs';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const log = (...a) => console.error(...a);
// engine code console.log()s under __RB_VERBOSE__ — keep stdout clean for the JSON document
console.log = (...a) => console.error(...a);
// engine also console.warn()s ([viaSlot] ...) from deep inside; silence unless --verbose
if (!process.argv.includes('--verbose')) { console.warn = () => {}; console.info = () => {}; console.debug = () => {}; }

// ─────────────────────────────────────────────────────────────
// CLI
// ─────────────────────────────────────────────────────────────
const argv = process.argv.slice(2);
const argOf = (k, d) => { const i = argv.indexOf(k); return i >= 0 && i + 1 < argv.length ? argv[i + 1] : d; };
const flag = (k) => argv.includes(k);
const opts = {
	seed: Number(argOf('--seed', 1)),
	nets: Number(argOf('--nets', 80)),
	runs: Number(argOf('--runs', 3)),
	budgetMs: Number(argOf('--budget', 60000)),
	layers: Number(argOf('--layers', 2)),
	phases: new Set(String(argOf('--phases', 'a,b1,b2')).split(',').map(s => s.trim()).filter(Boolean)),
	tightenWorld: String(argOf('--tighten-world', 'prod')),
	tightenTimeoutMs: Number(argOf('--tighten-timeout', 300000)),
	out: argOf('--out', null),
	verbose: flag('--verbose'),
};
if (flag('--help') || flag('-h')) {
	const src = fs.readFileSync(fileURLToPath(import.meta.url), 'utf8');
	process.stdout.write(src.slice(src.indexOf('/**'), src.indexOf('*/') + 2) + '\n');
	process.exit(0);
}
for (const k of ['seed', 'nets', 'runs', 'budgetMs', 'layers', 'tightenTimeoutMs']) {
	if (!Number.isFinite(opts[k]) || opts[k] < 0) { log(`bad --${k}`); process.exit(1); }
}
if (!['prod', 'clean'].includes(opts.tightenWorld)) { log('bad --tighten-world (prod|clean)'); process.exit(1); }
opts.nets = Math.max(2, Math.round(opts.nets));
opts.runs = Math.max(1, Math.round(opts.runs));

// ─────────────────────────────────────────────────────────────
// Small utils
// ─────────────────────────────────────────────────────────────
function mulberry32(a) {
	return () => {
		a |= 0; a = (a + 0x6D2B79F5) | 0;
		let t = Math.imul(a ^ (a >>> 15), 1 | a);
		t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
		return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
	};
}
const r1 = (v) => Math.round(v * 10) / 10;
const r2 = (v) => Math.round(v * 100) / 100;
function stats(arr) {
	const a = arr.filter(Number.isFinite).slice().sort((x, y) => x - y);
	if (!a.length) return { min: null, median: null, max: null };
	const mid = a.length % 2 ? a[(a.length - 1) / 2] : (a[a.length / 2 - 1] + a[a.length / 2]) / 2;
	return { min: r1(a[0]), median: r1(mid), max: r1(a[a.length - 1]) };
}
const sha1 = (s) => crypto.createHash('sha1').update(s).digest('hex').slice(0, 12);
const clone = (o) => structuredClone(o);

function renderedLength(prims) {
	let t = 0;
	for (const p of prims) {
		if (p.kind === 'line') t += Math.hypot((p.x2 ?? 0) - (p.x1 ?? 0), (p.y2 ?? 0) - (p.y1 ?? 0));
		else t += Math.abs(p.sweep ?? 0) * (p.r ?? 0);
	}
	return t;
}
function hashNets(nets, vias) {
	const q = (v) => Math.round((v ?? 0) * 100) / 100;
	const body = [...nets].sort((a, b) => a.name < b.name ? -1 : 1).map(n => [n.name, n.prims.map(p =>
		p.kind === 'line' ? ['l', p.layer, q(p.x1), q(p.y1), q(p.x2), q(p.y2)] : ['a', p.layer, q(p.cx), q(p.cy), q(p.r), q(p.startAngle), q(p.sweep)])]);
	const v = [...(vias ?? [])].map(x => [x.net, q(x.x), q(x.y)]).sort();
	return sha1(JSON.stringify([body, v]));
}

// ─────────────────────────────────────────────────────────────
// Engine bundles
// ─────────────────────────────────────────────────────────────
async function buildBundle() {
	const contents = `
		export * from './src/router/planning-engine.ts';
		export { buildPolylineObstacle, buildPolygonObstacle } from './src/router/rubberband-engine.ts';
		import './src/worker/planning-worker.ts';
	`;
	const built = await esbuild.build({
		stdin: { contents, resolveDir: ROOT, loader: 'ts', sourcefile: 'bench-entry.ts' },
		bundle: true, format: 'esm', write: false, platform: 'neutral', logLevel: 'error',
	});
	return built.outputFiles[0].text;
}
let bundleCode = null;
let freshCounter = 0;
/** Import a fresh instance of the bundle. The worker module grabs `self` at import time, so a
 * collecting shim is installed first; shim.onmessage is the real production worker handler. */
async function loadFresh() {
	const shim = { messages: [], onmessage: undefined, postMessage(m) { shim.messages.push(m); } };
	globalThis.self = shim;
	const code = bundleCode + `\n//# bench-instance ${++freshCounter}\n`;
	const mod = await import('data:text/javascript;base64,' + Buffer.from(code).toString('base64'));
	return { mod, shim };
}

// ─────────────────────────────────────────────────────────────
// Board model (shared by both generators)
// ─────────────────────────────────────────────────────────────
// JLC-like 2-layer rules, mil (what fetchDRCRules would return for 0.1524mm clearances).
const DRC = {
	traceWidthMil: 10, clearanceMil: 6,
	trackToTrackMil: 6, trackToLineMil: 6, trackToBoardMil: 11.81, trackToPadMil: 6,
	trackToFillMil: 10, holeToHoleMil: 11.81,
};
const VIA = { dia: 24, hole: 12 };
const padKey = (x, y) => `${x.toFixed(2)},${y.toFixed(2)}`;

/** pad factory — mirrors the EnginePad / PadShape population of scripts/eda-dump-driver.ts */
function makePad(board, { x, y, net, id, layer, kind, w, h }) {
	let shapeName = 'RECT', hole = null, padType = 'smd', holeMil = 0, lay = layer;
	if (kind === 'th') { shapeName = 'ELLIPSE'; hole = ['ROUND', 36, 36]; holeMil = 36; padType = 'th'; lay = 12; }
	const pad = {
		x, y, net, id, layer: lay, padType, holeDiameter: holeMil,
		radius: Math.max(w / 2, h / 2, holeMil / 2),
		bodyRadius: Math.max(Math.min(w, h) / 2, holeMil / 2),
		rotation: 0,
		index: board.allPads.length,
	};
	board.allPads.push(pad);
	board.padShapes[padKey(x, y)] = { pad: [shapeName, w, h], hole, rotation: 0 };
	return pad;
}

/** Static copper -> RBObstacles, mirroring topological-router.collectStaticObstacles
 * (mode 'tighten') / eda-dump-driver.dumpInput (mode 'plan').
 * `extraTracks` = existing routed copper (prod tighten world). */
function buildStaticObstacles(mod, board, mode, extraTracks = [], extraVias = []) {
	const d = board.drc;
	const lineClear = mode === 'tighten' ? d.trackToTrackMil : Math.max(d.trackToLineMil, 8);
	const copperMargin = mode === 'tighten' ? 0.5 : 2;
	const out = [];
	const o = board.outline;
	if (o) {
		const r = 10 / 2 + d.trackToBoardMil + d.traceWidthMil / 2;
		out.push(...mod.buildPolylineObstacle(o, r, undefined, undefined, 'board', true));
	}
	for (const f of board.statics.fills) {
		const r = (f.lw ?? 0) / 2 + Math.max(d.trackToFillMil, 10) + d.traceWidthMil / 2 + 2;
		out.push(...mod.buildPolygonObstacle(f.pts, r, f.net || undefined, f.layer, `fill:${f.pts[0].x},${f.pts[0].y}`, true));
	}
	const trackObs = (t) => {
		const r = (t.lw ?? 10) / 2 + lineClear + d.traceWidthMil / 2 + copperMargin;
		out.push(...mod.buildPolylineObstacle(t.pts, r, t.net || undefined, t.layer, t.group ?? `line:${t.id}`));
	};
	for (const t of board.statics.tracks) trackObs(t);
	for (const t of extraTracks) trackObs(t);
	const viaClear = Math.max(d.trackToTrackMil, d.trackToPadMil);
	for (const v of [...board.statics.vias, ...extraVias]) {
		out.push({
			id: 'via' + out.length, kind: 'circle', x: v.x, y: v.y,
			r: v.dia / 2 + viaClear + d.traceWidthMil / 2 + 0.1,
			net: v.net || undefined, layer: undefined,
			group: `via:${v.net ?? ''}:${v.x.toFixed(1)},${v.y.toFixed(1)}`, noPrune: true,
		});
	}
	return out;
}

/** routed prims -> polyline "tracks" (for the prod tighten world) */
function primsToTracks(nets) {
	const tracks = [];
	let k = 0;
	for (const n of nets) {
		for (const p of n.prims) {
			let pts;
			if (p.kind === 'line') pts = [{ x: p.x1, y: p.y1 }, { x: p.x2, y: p.y2 }];
			else {
				const steps = Math.max(6, Math.ceil(Math.abs(p.sweep ?? 0) / (Math.PI / 12)));
				pts = [];
				for (let i = 0; i <= steps; i++) {
					const a = p.startAngle + (p.sweep * i) / steps;
					pts.push({ x: p.cx + p.r * Math.cos(a), y: p.cy + p.r * Math.sin(a) });
				}
			}
			tracks.push({ id: `${n.name}#${k++}`, net: n.name, layer: p.layer ?? 1, lw: DRC.traceWidthMil, pts });
		}
	}
	return tracks;
}

// ─────────────────────────────────────────────────────────────
// Generator 1: routing board
// ─────────────────────────────────────────────────────────────
function genRoutingBoard({ seed, nets, layers }) {
	const rng = mulberry32(seed);
	const rint = (a, b) => a + Math.floor(rng() * (b - a + 1));

	// 1) net sizes: GND (many pads), VCC, then signals 70% 2-pin / 15% 3 / 10% 4 / 5% 5
	const sizes = [];
	if (nets >= 4) {
		sizes.push({ name: 'GND', k: Math.min(12, Math.max(3, Math.round(nets / 8))) });
		sizes.push({ name: 'VCC', k: Math.min(6, Math.max(3, Math.round(nets / 14))) });
	}
	const nSig = nets - sizes.length;
	for (let i = 0; i < nSig; i++) {
		const r = rng();
		sizes.push({ name: `NET_${String(i + 1).padStart(3, '0')}`, k: r < 0.70 ? 2 : r < 0.85 ? 3 : r < 0.95 ? 4 : 5 });
	}
	const totalPads = sizes.reduce((s, n) => s + n.k, 0);

	// 2) component library (local coords, y down; pads {dx,dy,w,h,kind})
	const passive = (bottom) => ({ name: 'R', pads: [-38, 38].map(dx => ({ dx, dy: 0, w: 32, h: 36, kind: 'smd', bottom })) });
	const soic = (n) => {
		const pads = [];
		for (let i = 0; i < n / 2; i++) {
			const y = (i - (n / 2 - 1) / 2) * 50;
			pads.push({ dx: -120, dy: y, w: 62, h: 24, kind: 'smd' }, { dx: 120, dy: y, w: 62, h: 24, kind: 'smd' });
		}
		return { name: `SOIC${n}`, pads };
	};
	const qfp = (n) => {
		const per = n / 4, pitch = 19.7, rowHalf = (per - 1) * pitch / 2, ring = rowHalf + 60;
		const pads = [];
		for (let i = 0; i < per; i++) {
			const t = -rowHalf + i * pitch;
			pads.push({ dx: t, dy: -ring, w: 11, h: 59, kind: 'smd' }, { dx: t, dy: ring, w: 11, h: 59, kind: 'smd' },
				{ dx: -ring, dy: t, w: 59, h: 11, kind: 'smd' }, { dx: ring, dy: t, w: 59, h: 11, kind: 'smd' });
		}
		return { name: `QFP${n}`, pads };
	};
	const header = (n, pitch = 100) => ({
		name: `J${n}`, pads: Array.from({ length: n }, (_, i) => ({ dx: 0, dy: (i - (n - 1) / 2) * pitch, w: 62, h: 62, kind: 'th' })),
	});

	const comps = [];
	let remaining = totalPads;
	while (remaining > 0) {
		const r = rng();
		if (remaining >= 48 && r < 0.03) comps.push(qfp(48));
		else if (remaining >= 32 && r < 0.06) comps.push(qfp(32));
		else if (remaining >= 16 && r < 0.12) comps.push(soic(16));
		else if (remaining >= 8 && r < 0.20) comps.push(soic(8));
		else if (r < 0.24 || remaining < 2) comps.push(header(Math.min(remaining, rint(2, 8))));
		else comps.push(passive(layers >= 2 && rng() < 0.10));
		remaining -= comps[comps.length - 1].pads.length;
	}
	// pre-routed foreign nets (kept out of input.networks; their tracks/via are static obstacles)
	const preA = { ...header(2, 100), name: 'PRE_A', pre: 'PRE_A' };
	const preB = { ...header(2, 200), name: 'PRE_B', pre: 'PRE_B' };
	comps.push(preA, preB);
	// keepouts (fills, no pads)
	const nKeep = Math.max(1, Math.round(nets / 40));
	for (let i = 0; i < nKeep; i++) comps.push({ name: 'KEEPOUT', pads: [], keepout: { w: 320, h: 200, layer: i % 2 === 0 ? 1 : (layers >= 2 ? 2 : 1) } });

	// 3) orientation + bbox + placement
	const GAP = 70, EDGE = 150;
	for (const c of comps) {
		c.rot = c.keepout ? 0 : rint(0, 3);
		const odd = c.rot % 2 === 1;
		const cs = [1, 0, -1, 0][c.rot], sn = [0, 1, 0, -1][c.rot];
		for (const p of c.pads) {
			const x = p.dx * cs - p.dy * sn, y = p.dx * sn + p.dy * cs;
			p.dx = x; p.dy = y;
			if (odd) { const t = p.w; p.w = p.h; p.h = t; }
		}
		if (c.keepout) { c.hx = c.keepout.w / 2; c.hy = c.keepout.h / 2; }
		else {
			c.hx = Math.max(...c.pads.map(p => Math.abs(p.dx) + p.w / 2));
			c.hy = Math.max(...c.pads.map(p => Math.abs(p.dy) + p.h / 2));
		}
	}
	const area = comps.reduce((s, c) => s + (2 * c.hx + GAP) * (2 * c.hy + GAP), 0) * 3.0;
	let W = Math.sqrt(area * 1.5) + 2 * EDGE, H = area / Math.sqrt(area * 1.5) + 2 * EDGE;
	const order = [...comps].sort((a, b) => b.hx * b.hy - a.hx * a.hy);
	for (let attempt = 0; attempt < 40; attempt++) {
		const placed = [];
		let ok = true;
		for (const c of order) {
			let done = false;
			for (let t = 0; t < 600 && !done; t++) {
				const x = EDGE + c.hx + rng() * Math.max(1, W - 2 * EDGE - 2 * c.hx);
				const y = EDGE + c.hy + rng() * Math.max(1, H - 2 * EDGE - 2 * c.hy);
				if (placed.every(q => Math.abs(q.cx - x) >= q.hx + c.hx + GAP || Math.abs(q.cy - y) >= q.hy + c.hy + GAP)) {
					c.cx = Math.round(x * 2) / 2; c.cy = Math.round(y * 2) / 2; placed.push(c); done = true;
				}
			}
			if (!done) { ok = false; break; }
		}
		if (ok) break;
		W *= 1.08; H *= 1.08;
		if (attempt === 39) throw new Error('component placement failed');
	}
	W = Math.round(W); H = Math.round(H);

	const board = {
		name: 'routing', widthMil: W, heightMil: H, drc: { ...DRC }, copperLayers: layers,
		allPads: [], padShapes: {}, networks: [], components: comps.length,
		outline: [{ x: 0, y: 0 }, { x: W, y: 0 }, { x: W, y: H }, { x: 0, y: H }, { x: 0, y: 0 }],
		statics: { tracks: [], vias: [], fills: [] },
	};

	// 4) pads (net assigned later); PRE nets assigned now
	const free = [];
	let compNo = 0;
	for (const c of comps) {
		compNo++;
		if (c.keepout) {
			const { w, h, layer } = c.keepout;
			const pts = [[-w / 2, -h / 2], [w / 2, -h / 2], [w / 2, h / 2], [-w / 2, h / 2], [-w / 2, -h / 2]].map(([a, b]) => ({ x: c.cx + a, y: c.cy + b }));
			board.statics.fills.push({ layer, pts, lw: 0 });
			continue;
		}
		const made = [];
		c.pads.forEach((p, i) => {
			const x = Math.round((c.cx + p.dx) * 100) / 100, y = Math.round((c.cy + p.dy) * 100) / 100;
			const lay = p.bottom ? 2 : 1;
			const id = `${c.name}${compNo}-${i + 1}`;
			if (c.pre) made.push(makePad(board, { x, y, net: c.pre, id, layer: lay, kind: p.kind, w: p.w, h: p.h }));
			else free.push({ spec: { x, y, id, layer: lay, kind: p.kind, w: p.w, h: p.h }, comp: compNo });
		});
		if (c.pre === 'PRE_A') {
			board.statics.tracks.push({ id: 'preA', net: 'PRE_A', layer: 1, lw: 10, pts: made.map(p => ({ x: p.x, y: p.y })) });
		} else if (c.pre === 'PRE_B') {
			const mx = (made[0].x + made[1].x) / 2, my = (made[0].y + made[1].y) / 2;
			board.statics.tracks.push({ id: 'preB1', net: 'PRE_B', layer: 2, lw: 10, pts: [{ x: made[0].x, y: made[0].y }, { x: mx, y: my }] });
			board.statics.tracks.push({ id: 'preB2', net: 'PRE_B', layer: 1, lw: 10, pts: [{ x: mx, y: my }, { x: made[1].x, y: made[1].y }] });
			board.statics.vias.push({ net: 'PRE_B', x: mx, y: my, dia: VIA.dia, hole: VIA.hole });
		}
	}

	// 5) assign pads to nets by locality (anchor + random among the nearest free pads,
	//    preferring other components so nets are not intra-footprint shorts)
	const netsByName = new Map();
	const order2 = [...sizes].sort((a, b) => b.k - a.k);
	for (const ns of order2) {
		const anchorIdx = rint(0, free.length - 1);
		const anchor = free.splice(anchorIdx, 1)[0];
		const chosen = [anchor];
		for (let j = 1; j < ns.k && free.length; j++) {
			const dist = (f) => Math.hypot(f.spec.x - anchor.spec.x, f.spec.y - anchor.spec.y);
			let pool = free.map((f, i) => ({ f, i, d: dist(f) })).sort((a, b) => a.d - b.d);
			const diffComp = pool.filter(q => !chosen.some(c => c.comp === q.f.comp));
			if (diffComp.length >= 2) pool = diffComp;
			pool = pool.slice(0, Math.max(2, Math.min(pool.length, 4 * (ns.k - 1) + 2)));
			const pick = pool[rint(0, pool.length - 1)];
			chosen.push(free.splice(pick.i, 1)[0]);
		}
		netsByName.set(ns.name, chosen);
	}
	for (const ns of sizes) {
		const pads = (netsByName.get(ns.name) ?? []).map(f => makePad(board, { ...f.spec, net: ns.name }));
		if (pads.length) board.networks.push({ name: ns.name, pads });
	}
	// `networks` pad objects must be the same shape as allPads entries (they are the same objects)
	return board;
}

// ─────────────────────────────────────────────────────────────
// Generator 2: "deliberately bad" traces on a DRC-clean board
// ─────────────────────────────────────────────────────────────
function genBadBoard({ seed }) {
	const rng = mulberry32(seed * 7919 + 13);
	const rint = (a, b) => a + Math.floor(rng() * (b - a + 1));
	const CW = 760, CH = 460, COLS = 6, ROWS = 4, M = 200;
	const W = COLS * CW + 2 * M, H = ROWS * CH + 2 * M;
	const board = {
		name: 'bad-traces', widthMil: W, heightMil: H, drc: { ...DRC }, copperLayers: 2,
		allPads: [], padShapes: {}, networks: [],
		outline: [{ x: 0, y: 0 }, { x: W, y: 0 }, { x: W, y: H }, { x: 0, y: H }, { x: 0, y: 0 }],
		statics: { tracks: [], vias: [], fills: [] },
		traces: [], vias: [], cases: [],
	};
	const staircase = (x0, y0, nSteps, dx, dy) => {
		const pts = [{ x: x0, y: y0 }];
		for (let k = 0; k < nSteps; k++) {
			const l = pts[pts.length - 1];
			pts.push({ x: l.x + dx, y: l.y }, { x: l.x + dx, y: l.y + dy });
		}
		return pts;
	};
	const stairs45 = () => {
		const pts = [{ x: 60, y: 100 }];
		for (let k = 0; k < 5; k++) {
			const l = pts[pts.length - 1];
			pts.push({ x: l.x + 70, y: l.y }, { x: l.x + 120, y: l.y + 50 });
		}
		return pts;
	};
	const P = (x, y) => ({ x, y });
	const KEEP = { x0: 40, y0: 360, x1: 300, y1: 440 };
	// each type: pads (net's own), layerPts: [{layer, pts}], foreign pads, via, keepout?
	const types = [
		{ id: 'stairs90', pads: [[60, 60, 1], [640, 400, 1]], segs: [{ layer: 1, pts: staircase(60, 60, 5, 116, 68) }], foreign: [[620, 60, 1, 'smd']], keep: true },
		{ id: 'zigzag45', pads: [[60, 100, 1], [660, 350, 1]], segs: [{ layer: 1, pts: stairs45() }], foreign: [[60, 400, 1, 'smd']] },
		{ id: 'detourU', pads: [[60, 230, 1], [700, 230, 1]], segs: [{ layer: 1, pts: [P(60, 230), P(160, 230), P(160, 70), P(600, 70), P(600, 230), P(700, 230)] }], foreign: [[380, 230, 12, 'th']] },
		{ id: 'dogleg', pads: [[60, 60, 1], [700, 400, 1]], segs: [{ layer: 1, pts: [P(60, 60), P(700, 60), P(700, 400)] }], foreign: [[380, 230, 12, 'th']] },
		{ id: 'tee3', pads: [[60, 60, 1], [700, 60, 1], [380, 400, 1]],
			segs: [{ layer: 1, pts: [P(60, 60), P(60, 200), P(380, 200)] }, { layer: 1, pts: [P(380, 200), P(700, 200), P(700, 60)] }, { layer: 1, pts: [P(380, 200), P(380, 400)] }],
			foreign: [[700, 400, 1, 'smd']], keep: true },
		{ id: 'wiggle', pads: [[60, 230, 1], [700, 230, 1]], jitter: true, foreign: [[380, 60, 1, 'smd'], [380, 400, 1, 'smd']] },
		{ id: 'stairs90Bottom', pads: [[60, 60, 2], [640, 400, 2]], segs: [{ layer: 2, pts: staircase(60, 60, 5, 116, 68) }], foreign: [[620, 60, 2, 'smd']] },
		{ id: 'viaNet', pads: [[60, 100, 1], [700, 360, 2]],
			segs: [{ layer: 1, pts: [P(60, 100), P(220, 100), P(220, 230), P(380, 230)] }, { layer: 2, pts: [P(380, 230), P(540, 230), P(540, 360), P(700, 360)] }],
			via: [380, 230], foreign: [[380, 60, 1, 'smd'], [380, 400, 2, 'smd']] },
	];
	for (let ci = 0; ci < COLS * ROWS; ci++) {
		const t = types[ci % types.length];
		const col = ci % COLS, row = Math.floor(ci / COLS);
		const flipX = rng() < 0.5, flipY = rng() < 0.5;
		const T = (x, y) => ({ x: M + col * CW + (flipX ? CW - x : x), y: M + row * CH + (flipY ? CH - y : y) });
		const net = `B_${String(ci + 1).padStart(2, '0')}_${t.id}`;
		let segs = t.segs;
		if (t.jitter) {
			const pts = [P(60, 230)];
			let last = 0;
			for (let k = 1; k <= 7; k++) {
				let j; do { j = rint(-70, 70); } while (Math.abs(j - last) < 40);
				last = j; pts.push(P(60 + 80 * k, 230 + j));
			}
			pts.push(P(700, 230));
			// make it an orthogonal-ish wiggle: each vertex pair stays as given (arbitrary angles)
			segs = [{ layer: 1, pts }];
		}
		const myPads = t.pads.map(([x, y, layer], i) => {
			const q = T(x, y);
			return makePad(board, { x: q.x, y: q.y, net, id: `${net}.${i + 1}`, layer, kind: 'smd', w: 32, h: 36 });
		});
		const prims = [];
		for (const s of segs) {
			for (let i = 0; i + 1 < s.pts.length; i++) {
				const a = T(s.pts[i].x, s.pts[i].y), b = T(s.pts[i + 1].x, s.pts[i + 1].y);
				prims.push({ kind: 'line', net, layer: s.layer, x1: a.x, y1: a.y, x2: b.x, y2: b.y });
			}
		}
		board.networks.push({ name: net, pads: myPads });
		board.traces.push({ name: net, prims });
		const caseInfo = { net, type: t.id, padSpanMil: myPads.length === 2 ? r1(Math.hypot(myPads[0].x - myPads[1].x, myPads[0].y - myPads[1].y)) : null };
		board.cases.push(caseInfo);
		if (t.via) {
			const q = T(t.via[0], t.via[1]);
			board.vias.push({ net, x: q.x, y: q.y, dia: VIA.dia, hole: VIA.hole });
		}
		(t.foreign ?? []).forEach(([x, y, layer, kind], i) => {
			const q = T(x, y);
			const fnet = `F_${String(ci + 1).padStart(2, '0')}_${i + 1}`;
			const p = kind === 'th'
				? makePad(board, { x: q.x, y: q.y, net: fnet, id: `${fnet}.1`, layer: 12, kind: 'th', w: 62, h: 62 })
				: makePad(board, { x: q.x, y: q.y, net: fnet, id: `${fnet}.1`, layer, kind: 'smd', w: 32, h: 36 });
			board.networks.push({ name: fnet, pads: [p] });
		});
		if (t.keep) {
			const a = T(KEEP.x0, KEEP.y0), b = T(KEEP.x1, KEEP.y1);
			const x0 = Math.min(a.x, b.x), x1 = Math.max(a.x, b.x), y0 = Math.min(a.y, b.y), y1 = Math.max(a.y, b.y);
			board.statics.fills.push({ layer: 1, lw: 0, pts: [P(x0, y0), P(x1, y0), P(x1, y1), P(x0, y1), P(x0, y0)] });
		}
	}
	// single-pad foreign nets are pads only (no tracks): not tighten inputs
	return board;
}

// ─────────────────────────────────────────────────────────────
// Metrics
// ─────────────────────────────────────────────────────────────
/** corner/prim statistics for one net's prims (see header for the definition) */
function netMetrics(prims) {
	const ends = [];
	const endpoint = (p, which) => p.kind === 'line'
		? (which ? { x: p.x2, y: p.y2 } : { x: p.x1, y: p.y1 })
		: { x: p.cx + p.r * Math.cos(p.startAngle + (which ? p.sweep : 0)), y: p.cy + p.r * Math.sin(p.startAngle + (which ? p.sweep : 0)) };
	const nodes = [];
	const nodeOf = (pt) => {
		for (let i = 0; i < nodes.length; i++) if (Math.hypot(nodes[i].x - pt.x, nodes[i].y - pt.y) < 0.8) return nodes[i];
		const n = { x: pt.x, y: pt.y, inc: [] };
		nodes.push(n);
		return n;
	};
	prims.forEach((p, i) => {
		nodeOf(endpoint(p, false)).inc.push({ i, which: false });
		nodeOf(endpoint(p, true)).inc.push({ i, which: true });
	});
	let sharp45 = 0, sharp89 = 0;
	for (const n of nodes) {
		if (n.inc.length !== 2) continue;
		const [a, b] = n.inc;
		const pa = prims[a.i], pb = prims[b.i];
		if (a.i === b.i || pa.kind !== 'line' || pb.kind !== 'line' || (pa.layer ?? 1) !== (pb.layer ?? 1)) continue;
		// direction leaving the joint along each prim
		const dir = (p, atEnd) => {
			const dx = atEnd ? p.x1 - p.x2 : p.x2 - p.x1, dy = atEnd ? p.y1 - p.y2 : p.y2 - p.y1;
			const l = Math.hypot(dx, dy);
			return l > 1e-9 ? { x: dx / l, y: dy / l } : null;
		};
		const da = dir(pa, a.which), db = dir(pb, b.which);
		if (!da || !db) continue;
		const between = Math.acos(Math.max(-1, Math.min(1, da.x * db.x + da.y * db.y))) * 180 / Math.PI;
		const turn = 180 - between;
		if (turn >= 45) sharp45++;
		if (turn >= 89) sharp89++;
	}
	return {
		lines: prims.filter(p => p.kind === 'line').length,
		arcs: prims.filter(p => p.kind === 'arc').length,
		lengthMil: renderedLength(prims), sharp45, sharp89,
	};
}
function aggMetrics(nets) {
	const m = { lines: 0, arcs: 0, lengthMil: 0, sharp45: 0, sharp89: 0 };
	for (const n of nets) { const q = netMetrics(n.prims); for (const k of Object.keys(m)) m[k] += q[k]; }
	m.lengthMil = r1(m.lengthMil);
	return m;
}

/** strict edge-to-edge clearance audit (see header) */
function edgeAudit(board, nets, vias, polygonToRect) {
	const d = board.drc, tw = d.traceWidthMil;
	const STEP = 2.5, TOL = 0.5;
	const trk = new Map(), padG = new Map(), viaG = new Map();
	const put = (g, cell, x, y, it) => { const k = `${Math.floor(x / cell)},${Math.floor(y / cell)}`; let b = g.get(k); if (!b) g.set(k, b = []); b.push({ x, y, ...it }); };
	const sampleLine = (x1, y1, x2, y2, net, layer, hw) => {
		const n = Math.max(1, Math.ceil(Math.hypot(x2 - x1, y2 - y1) / STEP));
		for (let i = 0; i <= n; i++) put(trk, 32, x1 + (x2 - x1) * i / n, y1 + (y2 - y1) * i / n, { net, layer, hw });
	};
	for (const n of nets) for (const p of n.prims) {
		if (p.kind === 'line') sampleLine(p.x1, p.y1, p.x2, p.y2, n.name, p.layer ?? 1, tw / 2);
		else {
			const steps = Math.max(2, Math.ceil(Math.abs(p.sweep) * p.r / STEP));
			for (let i = 0; i <= steps; i++) {
				const a = p.startAngle + p.sweep * i / steps;
				put(trk, 32, p.cx + p.r * Math.cos(a), p.cy + p.r * Math.sin(a), { net: n.name, layer: p.layer ?? 1, hw: tw / 2 });
			}
		}
	}
	for (const t of board.statics.tracks) for (let i = 0; i + 1 < t.pts.length; i++) sampleLine(t.pts[i].x, t.pts[i].y, t.pts[i + 1].x, t.pts[i + 1].y, t.net, t.layer, (t.lw ?? 10) / 2);
	for (const p of board.allPads) put(padG, 128, p.x, p.y, { pad: p });
	for (const v of [...board.statics.vias, ...(vias ?? [])]) put(viaG, 64, v.x, v.y, { net: v.net, r: v.dia / 2 });

	const padDist = (pad, x, y) => {
		const sh = board.padShapes[padKey(pad.x, pad.y)];
		const s = sh?.pad;
		if (!Array.isArray(s)) return Math.hypot(x - pad.x, y - pad.y) - pad.radius;
		let name = String(s[0] ?? '').toUpperCase(), w = s[1], h = s[2], rot = sh.rotation ?? 0;
		if (name === 'POLYGON') {
			const rc = polygonToRect(s);
			if (!rc) return Math.hypot(x - pad.x, y - pad.y) - pad.radius;
			name = 'RECT'; w = rc.w; h = rc.h; rot = rc.rotationDeg;
		}
		const a = -rot * Math.PI / 180, c = Math.cos(a), sn = Math.sin(a);
		const dx = x - pad.x, dy = y - pad.y;
		const lx = dx * c - dy * sn, ly = dx * sn + dy * c;
		if (name === 'RECT' || name === 'ROUNDRECT') {
			const ox = Math.abs(lx) - w / 2, oy = Math.abs(ly) - h / 2;
			return ox > 0 && oy > 0 ? Math.hypot(ox, oy) : Math.max(ox, oy);
		}
		// ELLIPSE/OVAL: capsule along the longer axis
		if (w >= h) return Math.hypot(Math.max(Math.abs(lx) - (w - h) / 2, 0), ly) - h / 2;
		return Math.hypot(lx, Math.max(Math.abs(ly) - (h - w) / 2, 0)) - w / 2;
	};
	const groups = new Map();
	const consider = (na, nb, kind, edge, need, x, y) => {
		if (edge >= need - TOL) return;
		const [a, b] = na < nb ? [na, nb] : [nb, na];
		const k = `${a}|${b}|${kind}`;
		const cur = groups.get(k);
		if (!cur || edge < cur.edgeMil) groups.set(k, { netA: a, netB: b, kind, edgeMil: r1(edge), needMil: r2(need), x: Math.round(x), y: Math.round(y) });
	};
	const neigh = (g, cell, x, y, fn) => {
		const cx = Math.floor(x / cell), cy = Math.floor(y / cell);
		for (let i = -1; i <= 1; i++) for (let j = -1; j <= 1; j++) { const b = g.get(`${cx + i},${cy + j}`); if (b) for (const it of b) fn(it); }
	};
	const W = board.widthMil, H = board.heightMil;
	for (const b of trk.values()) for (const s of b) {
		neigh(trk, 32, s.x, s.y, (o) => {
			if (o.net === s.net || o.layer !== s.layer) return;
			consider(s.net, o.net, 'trk-trk', Math.hypot(o.x - s.x, o.y - s.y) - s.hw - o.hw, d.trackToTrackMil, (o.x + s.x) / 2, (o.y + s.y) / 2);
		});
		neigh(padG, 128, s.x, s.y, ({ pad }) => {
			if (pad.net === s.net || (pad.layer !== 12 && pad.layer !== s.layer)) return;
			consider(s.net, pad.net, 'trk-pad', padDist(pad, s.x, s.y) - s.hw, d.trackToPadMil, s.x, s.y);
		});
		neigh(viaG, 64, s.x, s.y, (v) => {
			if (v.net === s.net) return;
			consider(s.net, v.net ?? '', 'trk-via', Math.hypot(v.x - s.x, v.y - s.y) - v.r - s.hw, d.trackToPadMil, s.x, s.y);
		});
		if (board.outline) consider(s.net, 'BOARD', 'trk-board', Math.min(s.x, W - s.x, s.y, H - s.y) - s.hw, d.trackToBoardMil, s.x, s.y);
	}
	// static tracks are samples too; keep only pairs involving at least one routed net to avoid
	// reporting the fixed fixture against itself
	const routed = new Set(nets.map(n => n.name));
	const list = [...groups.values()].filter(v => routed.has(v.netA) || routed.has(v.netB)).sort((a, b) => a.edgeMil - b.edgeMil);
	return { violationCount: list.length, worst: list.slice(0, 8) };
}

function audits(board, nets, vias, mod) {
	const input = { drc: board.drc, allPads: board.allPads, padShapes: board.padShapes };
	const res = { nets, vias: vias ?? [] };
	const a = drcAudit(input, res, mod.polygonToRect);
	const e = edgeAudit(board, nets, vias, mod.polygonToRect);
	return { drc: { violationCount: a.violationCount, top: a.violations.slice(0, 5) }, drcEdge: e };
}

// ─────────────────────────────────────────────────────────────
// Phase A
// ─────────────────────────────────────────────────────────────
async function runPhaseA(board, util) {
	const runs = [];
	let firstResult = null;
	for (let i = 1; i <= opts.runs; i++) {
		const { mod } = await loadFresh();
		const input = {
			drc: clone(board.drc), allPads: clone(board.allPads), networks: clone(board.networks),
			staticObstacles: buildStaticObstacles(mod, board, 'plan'), padShapes: clone(board.padShapes),
			budgetMs: opts.budgetMs, copperLayers: board.copperLayers,
			options: { ripupRounds: 5, optimize: true, allowViaFallback: true, allowViaInPad: false, netOrder: 'auto' },
		};
		const padStats = new Map();
		const phaseLog = [];
		log(`[A ${i}/${opts.runs}] routing ${input.networks.length} nets, budget ${opts.budgetMs}ms ...`);
		const t0 = Date.now();
		const eng = new mod.PlanningEngine();
		const result = await eng.plan(input, {
			isCancelled: () => false,
			onNetUpdate: (name, _p, _v, _ok, pads) => { if (pads) padStats.set(name, pads); },
			onLog: (m, level) => {
				if (/\[耗时\]/.test(m)) phaseLog.push(m.replace(/^.*?\[耗时\]\s*/, ''));
				if (opts.verbose) log(`  [${level}] ${m}`);
			},
		});
		const wallMs = Date.now() - t0;
		let padsTotal = 0, padsRouted = 0;
		for (const n of input.networks) { const st = padStats.get(n.name); padsTotal += st ? st.total : n.pads.length; padsRouted += st ? st.routed : 0; }
		const nets = result.nets.map(n => ({ name: n.name, prims: n.prims }));
		const au = audits(board, nets, result.vias, util.mod);
		const j = jointAudit({ nets });
		const m = aggMetrics(nets);
		const run = {
			run: i, wallMs,
			nets: { routed: result.routedNets.length, total: input.networks.length, failed: result.failed, partial: result.partial },
			pads: { routed: padsRouted, total: padsTotal },
			vias: result.vias.length,
			...m,
			corners: { sharp45: m.sharp45, sharp89: m.sharp89 },
			drc: au.drc, drcEdge: au.drcEdge,
			joints: { gaps: j.gaps, sharp: j.sharp, bigArcs: j.bigArcs },
			resultHash: hashNets(nets, result.vias),
			enginePhaseSeconds: phaseLog,
		};
		delete run.sharp45; delete run.sharp89;
		runs.push(run);
		if (i === 1) firstResult = { nets, vias: result.vias.map(v => ({ ...v })) };
		log(`  -> ${run.nets.routed}/${run.nets.total} nets, pads ${padsRouted}/${padsTotal}, ${run.vias} vias, ${r1(run.lengthMil)} mil, drc ${au.drc.violationCount}/edge ${au.drcEdge.violationCount}, ${wallMs}ms`);
	}
	const summary = {
		wallMs: stats(runs.map(r => r.wallMs)),
		routedNets: stats(runs.map(r => r.nets.routed)),
		routedPads: stats(runs.map(r => r.pads.routed)),
		vias: stats(runs.map(r => r.vias)),
		lengthMil: stats(runs.map(r => r.lengthMil)),
		drcViolations: stats(runs.map(r => r.drc.violationCount)),
		drcEdgeViolations: stats(runs.map(r => r.drcEdge.violationCount)),
		identicalGeometryAcrossRuns: new Set(runs.map(r => r.resultHash)).size === 1,
		resultHashes: runs.map(r => r.resultHash),
	};
	return { result: { runs, summary }, first: firstResult };
}

// ─────────────────────────────────────────────────────────────
// Phase B
// ─────────────────────────────────────────────────────────────
async function runPhaseB(label, board, nets, vias, util) {
	const { mod } = util;
	const drc = board.drc;
	// production split (topological-router.tightenExistingRouting): fresh engine, initTightenWorld with an empty world
	const split = new mod.PlanningEngine();
	split.initTightenWorld(drc, board.allPads, [], board.padShapes);
	const viasByNet = new Map();
	for (const v of vias) { if (!viasByNet.has(v.net)) viasByNet.set(v.net, []); viasByNet.get(v.net).push(v); }
	const runs = [];
	for (const n of nets) {
		if (!n.prims.length) continue;
		for (const seg of split.splitRunsForTighten(n.prims, viasByNet.get(n.name) ?? [])) {
			runs.push({ id: runs.length, net: n.name, layer: seg.layer, a: seg.a, b: seg.b, prims: seg.prims });
		}
	}
	const baseWorld = buildStaticObstacles(mod, board, 'tighten',
		opts.tightenWorld === 'prod' ? primsToTracks(nets) : [], []);
	const msg = { type: 'tighten', drc, allPads: board.allPads, baseWorld, padShapes: board.padShapes, vias, runs };

	const before = {
		...aggMetrics(nets),
		...audits(board, nets, vias, mod),
	};
	const perNetBefore = new Map(nets.map(n => [n.name, netMetrics(n.prims)]));
	const out = { runCount: runs.length, netCount: nets.length, tightenWorld: opts.tightenWorld, before, runs: [] };

	for (let i = 1; i <= opts.runs; i++) {
		const { mod: wmod, shim } = await loadFresh();
		if (typeof shim.onmessage !== 'function') throw new Error('planning-worker did not install self.onmessage');
		void wmod;
		log(`[${label} ${i}/${opts.runs}] tightening ${runs.length} runs of ${nets.length} nets (world=${opts.tightenWorld}) ...`);
		const t0 = Date.now();
		let timedOut = false;
		const timer = setTimeout(() => { timedOut = true; shim.onmessage({ data: { type: 'cancel' } }); }, opts.tightenTimeoutMs);
		await shim.onmessage({ data: clone(msg) });
		clearTimeout(timer);
		const wallMs = Date.now() - t0;
		const err = shim.messages.find(m => m.type === 'error');
		if (err) throw new Error(`tighten worker error: ${err.message}`);
		// ---- apply results the way the main thread does ----
		const cur = new Map(runs.map(r => [r.id, r.prims]));
		const override = new Map();
		let done = null, nRunMsgs = 0;
		for (const m of shim.messages) {
			if (m.type === 'tightenRun') { nRunMsgs++; if (m.prims && m.prims.length) cur.set(m.id, m.prims); }
			else if (m.type === 'netRewrite') { if (m.prims && m.prims.length) override.set(m.net, m.prims); }
			else if (m.type === 'tightenDone') done = m;
		}
		const after = nets.map(n => {
			if (override.has(n.name)) return { name: n.name, prims: override.get(n.name) };
			return { name: n.name, prims: runs.filter(r => r.net === n.name).flatMap(r => cur.get(r.id)) };
		});
		const au = audits(board, after, vias, mod);
		const m = aggMetrics(after);
		const run = {
			run: i, wallMs, timedOut,
			tightenDone: done ? { total: done.total, adopted: done.adopted, rounded: done.rounded, solved: done.solved, rewritten: done.rewritten, saved: r1(done.saved), solveAttempts: done.solveAttempts, solveMs: done.solveMs } : null,
			after: { ...m, ...au },
			deltaLengthMil: r1(m.lengthMil - before.lengthMil),
			resultHash: hashNets(after, vias),
		};
		if (label === 'b2') {
			run.perCase = board.cases.map(c => {
				const bm = perNetBefore.get(c.net);
				const am = netMetrics(after.find(n => n.name === c.net).prims);
				return { net: c.net, type: c.type, padSpanMil: c.padSpanMil, lengthBefore: r1(bm.lengthMil), lengthAfter: r1(am.lengthMil), sharp45Before: bm.sharp45, sharp45After: am.sharp45, arcsBefore: bm.arcs, arcsAfter: am.arcs };
			});
		}
		out.runs.push(run);
		log(`  -> adopted ${done?.adopted}/${done?.total} (rounded ${done?.rounded}, solved ${done?.solved}, rewritten ${done?.rewritten}), len ${r1(before.lengthMil)} -> ${m.lengthMil}, sharp45 ${before.sharp45} -> ${m.sharp45}, drc ${au.drc.violationCount}/edge ${au.drcEdge.violationCount}, ${wallMs}ms${timedOut ? ' (TIMED OUT -> cancelled)' : ''}`);
	}
	out.summary = {
		wallMs: stats(out.runs.map(r => r.wallMs)),
		solveMs: stats(out.runs.map(r => r.tightenDone?.solveMs)),
		lengthAfterMil: stats(out.runs.map(r => r.after.lengthMil)),
		sharp45After: stats(out.runs.map(r => r.after.sharp45)),
		sharp89After: stats(out.runs.map(r => r.after.sharp89)),
		arcsAfter: stats(out.runs.map(r => r.after.arcs)),
		drcAfter: stats(out.runs.map(r => r.after.drc.violationCount)),
		drcEdgeAfter: stats(out.runs.map(r => r.after.drcEdge.violationCount)),
		identicalGeometryAcrossRuns: new Set(out.runs.map(r => r.resultHash)).size === 1,
	};
	return out;
}

// ─────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────
const doc = {
	meta: {
		script: 'scripts/bench-offline.mjs', node: process.version, units: 'mil (lengths/coords/clearances), ms (times)',
		params: { seed: opts.seed, nets: opts.nets, runs: opts.runs, budgetMs: opts.budgetMs, layers: opts.layers, phases: [...opts.phases], tightenWorld: opts.tightenWorld, tightenTimeoutMs: opts.tightenTimeoutMs },
		definitions: {
			sharp45: 'two LINE prims of one net+layer meeting at a degree-2 point (0.8 mil clustering) with no arc between them, turn angle >= 45 deg',
			sharp89: 'same, turn angle >= 89 deg (subset of sharp45)',
			drc: 'audit-lib drcAudit (center-line distances, lenient); count = violated net pairs',
			drcEdge: 'own strict edge-to-edge audit (track width, real pad shapes, vias, static tracks, board edge); count = violated (netA,netB,kind) groups',
		},
	},
};
let exitCode = 0;
try {
	bundleCode = await buildBundle();
	globalThis.__RB_VERBOSE__ = opts.verbose;
	const util = await loadFresh();   // builders, splitting, polygonToRect (never runs plan())

	let first = null;
	let boardA = null;
	const needBoardA = opts.phases.has('a') || opts.phases.has('b1');
	if (needBoardA) {
		boardA = genRoutingBoard({ seed: opts.seed, nets: opts.nets, layers: opts.layers });
		doc.routingBoard = {
			widthMil: boardA.widthMil, heightMil: boardA.heightMil, components: boardA.components,
			nets: boardA.networks.length, pads: boardA.allPads.length,
			padsByLayer: { top: boardA.allPads.filter(p => p.layer === 1).length, bottom: boardA.allPads.filter(p => p.layer === 2).length, throughHole: boardA.allPads.filter(p => p.layer === 12).length },
			netSizes: Object.fromEntries([...boardA.networks.reduce((m, n) => m.set(n.pads.length, (m.get(n.pads.length) ?? 0) + 1), new Map())].sort((a, b) => a[0] - b[0])),
			staticObstacles: { fills: boardA.statics.fills.length, tracks: boardA.statics.tracks.length, vias: boardA.statics.vias.length },
			boardHash: sha1(JSON.stringify([boardA.allPads.map(p => [p.net, p.x, p.y, p.layer]), boardA.statics])),
		};
		log(`routing board ${boardA.widthMil}x${boardA.heightMil} mil, ${boardA.networks.length} nets, ${boardA.allPads.length} pads`);
	}
	if (opts.phases.has('a')) {
		const a = await runPhaseA(boardA, util);
		doc.phaseA = a.result;
		first = a.first;
	} else doc.phaseA = { skipped: 'not selected (--phases)' };

	doc.phaseB = {};
	if (opts.phases.has('b1')) {
		if (!first) doc.phaseB.b1 = { skipped: 'needs phase A (run with --phases a,b1)' };
		else doc.phaseB.b1 = await runPhaseB('b1', boardA, first.nets, first.vias, util);
	}
	if (opts.phases.has('b2')) {
		const bad = genBadBoard({ seed: opts.seed });
		const b2 = await runPhaseB('b2', bad, bad.traces, bad.vias, util);
		b2.board = { widthMil: bad.widthMil, heightMil: bad.heightMil, tracedNets: bad.traces.length, pads: bad.allPads.length, caseTypes: [...new Set(bad.cases.map(c => c.type))] };
		if (b2.before.drc.violationCount !== 0 || b2.before.drcEdge.violationCount !== 0) {
			log(`WARNING: b2 fixture is not DRC-clean before tightening (drc=${b2.before.drc.violationCount}, edge=${b2.before.drcEdge.violationCount})`);
		}
		doc.phaseB.b2 = b2;
	}
} catch (e) {
	exitCode = 1;
	doc.error = String(e?.stack ?? e);
	log('ERROR:', doc.error);
}
const text = JSON.stringify(doc, null, 2);
if (opts.out) fs.writeFileSync(opts.out, text + '\n', 'utf8');
process.stdout.write(text + '\n');
process.exit(exitCode);
