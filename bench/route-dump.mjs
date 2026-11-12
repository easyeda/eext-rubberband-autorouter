#!/usr/bin/env node
/**
 * route-dump.mjs — run PlanningEngine.plan() offline on a real-board dump and report metrics.
 *
 *   node bench/route-dump.mjs <dump.json> [--budget ms] [--ripup N] [--runs N] [--out file.json]
 *                             [--verbose] [--net-order auto|river|...] [--no-optimize]
 *                             [--no-via-fallback] [--keep-old-tracks] [--dump-options]
 *
 * Dump format (tmp/dumps/<name>-dump.json, written by scripts/batch-dump-all.mjs):
 *   { structVersion, input }   input = PlanningInput {drc, allPads, networks, staticObstacles, padShapes, copperLayers, ...}
 * A structVersion different from scripts/struct-version.mjs STRUCT_VERSION only produces a warning.
 *
 * Options: the engine gets the UI defaults the way topological-router.ts maps them (NOT the dump's own
 * `options`, which come from eda-dump-driver with ripupRounds 5): budgetMs 1800000, ripupRounds 3,
 * optimize true (finalOptimize), allowViaFallback true (viaFallback), allowViaInPad false,
 * escapeClearanceMil 0, connectivityFirst true, powerNetsFirst false, netOrder 'auto'.
 * --dump-options keeps the dump's own `options` object instead (still overriding --ripup/--budget).
 * Like scripts/batch-route-all.mjs, static `line:` obstacles of nets that are being routed are removed
 * (EDA's clearExistingTraces semantics); --keep-old-tracks disables that.
 *
 * Every run imports a fresh copy of the bundled engine (new module instance, no cross-run state).
 *
 * Reported per run: wallMs, endedBy (self | budget), routed/total nets, routed/total pad connections
 * (last onNetUpdate `pads` stats per net, same as batch-route-all; nets that never reported count
 * 0 routed of n.pads.length), vias, lengthMil (sum of rendered prim lengths), lenient drcAudit count
 * (audit-lib), jointAudit (gaps/sharp/bigArcs), resultHash (sha1/12 of prims rounded to 0.01 mil, as
 * bench-offline's hashNets), the engine's "[耗时]" phase lines, and the strict edge audit below.
 * endedBy=budget when wallMs >= 95% of the budget or the engine logged "时间预算用尽"; such runs are
 * wall-clock truncated and their hash is not reproducible.
 *
 * STRICT EDGE AUDIT (drcEdge) — same idea as bench-offline's edgeAudit, restricted to what a dump has.
 *   Checks (violation = clearance < rule - 0.5 mil, one entry per (netA, netB, kind), same layer):
 *     trk-trk  routed track edge to routed track edge       (rule drc.trackToTrackMil, width = drc.traceWidthMil)
 *     trk-pad  routed track edge to the real pad shape       (rule drc.trackToPadMil; RECT/ROUNDRECT/POLYGON
 *              box, ELLIPSE/OVAL capsule; pad.layer 12 = all layers)
 *     trk-via  routed track edge to a routed via annulus     (rule drc.trackToPadMil)
 *   NOT checked: static tracks, board outline (trackToBoard), keepouts/fills, other static obstacles
 *   (a dump only holds them as inflated staticObstacles, not as raw geometry), via-pad, via-via,
 *   hole-to-hole, trackToFill, and anything between two nets that are both pre-existing.
 *   Clearance is sampled (2.5 mil step) from centre-lines, so it is accurate to ~±1 mil on corners.
 *
 * Output: human lines on stdout; --out writes the full JSON (runs[] + summary). Exit 1 on engine error.
 */
import * as esbuild from 'esbuild';
import * as fs from 'fs';
import * as path from 'path';
import * as crypto from 'crypto';
import { fileURLToPath } from 'url';
import { STRUCT_VERSION } from '../scripts/struct-version.mjs';
import { drcAudit, jointAudit } from '../scripts/audit-lib.mjs';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const argv = process.argv.slice(2);
const argOf = (k, d) => { const i = argv.indexOf(k); return i >= 0 && i + 1 < argv.length ? argv[i + 1] : d; };
const flag = (k) => argv.includes(k);

if (flag('--help') || flag('-h') || argv.length === 0) {
	const src = fs.readFileSync(fileURLToPath(import.meta.url), 'utf8');
	process.stdout.write(src.slice(src.indexOf('/**'), src.indexOf('*/') + 2) + '\n');
	process.exit(argv.length === 0 ? 2 : 0);
}
const VALUE_FLAGS = new Set(['--budget', '--ripup', '--runs', '--out', '--net-order']);
const dumpPath = argv.find((a, i) => !a.startsWith('--') && !VALUE_FLAGS.has(argv[i - 1]));
if (!dumpPath) { console.error('usage: route-dump.mjs <dump.json> [options]'); process.exit(2); }

const verbose = flag('--verbose');
// engine code console.log()s under __RB_VERBOSE__; keep stdout for our report lines
const out = (...a) => process.stdout.write(a.join(' ') + '\n');
const realLog = console.log;
console.log = (...a) => console.error(...a);
if (!verbose) { console.warn = () => {}; console.info = () => {}; console.debug = () => {}; }
globalThis.__RB_VERBOSE__ = verbose;

const budgetMs = Number(argOf('--budget', 1800000));
const ripupRounds = Number(argOf('--ripup', 3));
const runs = Math.max(1, Math.round(Number(argOf('--runs', 1))));
const outFile = argOf('--out', null);
for (const [k, v] of Object.entries({ budget: budgetMs, ripup: ripupRounds })) {
	if (!Number.isFinite(v) || v < 0) { console.error(`bad --${k}`); process.exit(2); }
}

const sha1 = (s) => crypto.createHash('sha1').update(s).digest('hex').slice(0, 12);
const r1 = (v) => Math.round(v * 10) / 10;
const r2 = (v) => Math.round(v * 100) / 100;
const median = (a) => { const s = [...a].sort((x, y) => x - y); const m = s.length >> 1; return s.length % 2 ? s[m] : (s[m - 1] + s[m]) / 2; };
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

/** strict edge-to-edge audit restricted to routed copper vs routed copper / pads (see header) */
function edgeAudit(input, nets, vias, polygonToRect) {
	const d = input.drc, tw = d.traceWidthMil;
	const STEP = 2.5, TOL = 0.5;
	const padKey = (x, y) => `${x.toFixed(2)},${y.toFixed(2)}`;
	const trk = new Map(), padG = new Map(), viaG = new Map();
	const put = (g, cell, x, y, it) => { const k = `${Math.floor(x / cell)},${Math.floor(y / cell)}`; let b = g.get(k); if (!b) g.set(k, b = []); b.push({ x, y, ...it }); };
	for (const n of nets) for (const p of n.prims) {
		if (p.kind === 'line') {
			const k = Math.max(1, Math.ceil(Math.hypot(p.x2 - p.x1, p.y2 - p.y1) / STEP));
			for (let i = 0; i <= k; i++) put(trk, 32, p.x1 + (p.x2 - p.x1) * i / k, p.y1 + (p.y2 - p.y1) * i / k, { net: n.name, layer: p.layer ?? 1, hw: tw / 2 });
		} else {
			const k = Math.max(2, Math.ceil(Math.abs(p.sweep) * p.r / STEP));
			for (let i = 0; i <= k; i++) {
				const a = p.startAngle + p.sweep * i / k;
				put(trk, 32, p.cx + p.r * Math.cos(a), p.cy + p.r * Math.sin(a), { net: n.name, layer: p.layer ?? 1, hw: tw / 2 });
			}
		}
	}
	for (const p of input.allPads) put(padG, 128, p.x, p.y, { pad: p });
	for (const v of vias ?? []) put(viaG, 64, v.x, v.y, { net: v.net, r: v.dia / 2 });

	const padDist = (pad, x, y) => {
		const sh = input.padShapes[padKey(pad.x, pad.y)];
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
	}
	const list = [...groups.values()].sort((a, b) => a.edgeMil - b.edgeMil);
	return { violationCount: list.length, worst: list.slice(0, 8) };
}

// ── load dump ────────────────────────────────────────────────
let wrapped;
try { wrapped = JSON.parse(fs.readFileSync(dumpPath, 'utf8')); }
catch (e) { console.error(`cannot read dump ${dumpPath}: ${e.message}`); process.exit(2); }
const input0 = wrapped.input ?? wrapped;
if (!input0?.networks || !input0?.allPads || !input0?.drc) { console.error('not a dump: input.{drc,allPads,networks} missing'); process.exit(2); }
const dumpVersion = wrapped.structVersion ?? 1;
if (dumpVersion !== STRUCT_VERSION) console.error(`WARNING: dump structVersion ${dumpVersion} != current ${STRUCT_VERSION} (scripts/struct-version.mjs); results may be meaningless — re-dump the board`);

const built = await esbuild.build({
	entryPoints: [path.join(ROOT, 'src/router/planning-engine.ts')],
	bundle: true, format: 'esm', write: false, platform: 'neutral', logLevel: 'error',
});
const bundleCode = built.outputFiles[0].text;
let fresh = 0;
const loadFresh = () => import('data:text/javascript;base64,' + Buffer.from(bundleCode + `\n//# route-dump-instance ${++fresh}\n`).toString('base64'));

function buildInput() {
	const input = clone(input0);
	input.budgetMs = budgetMs;
	const uiOptions = {
		connectivityFirst: true,
		optimize: !flag('--no-optimize'),
		allowViaFallback: !flag('--no-via-fallback'),
		allowViaInPad: false,
		escapeClearanceMil: 0,
		ripupRounds,
		powerNetsFirst: false,
		netOrder: argOf('--net-order', 'auto'),
	};
	input.options = flag('--dump-options') ? { ...(input.options ?? {}), ripupRounds } : uiOptions;
	let removedOld = 0;
	if (!flag('--keep-old-tracks')) {
		const routed = new Set(input.networks.map(n => n.name));
		const before = input.staticObstacles.length;
		input.staticObstacles = input.staticObstacles.filter(o => !(o.group?.startsWith('line:') && o.net && routed.has(o.net)));
		removedOld = before - input.staticObstacles.length;
	}
	return { input, removedOld };
}

const results = [];
let exitCode = 0;
for (let i = 1; i <= runs; i++) {
	const mod = await loadFresh();
	const { input, removedOld } = buildInput();
	const padStats = new Map();
	const phaseLog = [];
	let budgetLogged = false;
	console.error(`[run ${i}/${runs}] ${input.networks.length} nets, ${input.allPads.length} pads, budget ${budgetMs}ms, ripup ${ripupRounds} ...`);
	const t0 = Date.now();
	let result;
	try {
		result = await new mod.PlanningEngine().plan(input, {
			isCancelled: () => false,
			onNetUpdate: (name, _p, _v, _ok, pads) => { if (pads) padStats.set(name, pads); },
			onLog: (m, level) => {
				if (/\[耗时\]/.test(m)) phaseLog.push(m.replace(/^.*?\[耗时\]\s*/, ''));
				if (/时间预算用尽/.test(m)) budgetLogged = true;
				if (verbose) console.error(`  [${level}] ${m}`);
			},
		});
	} catch (e) {
		exitCode = 1;
		out(`run ${i}: ENGINE ERROR ${String(e?.stack ?? e).split('\n').slice(0, 4).join(' | ')}`);
		results.push({ run: i, error: String(e?.stack ?? e) });
		continue;
	}
	const wallMs = Date.now() - t0;
	let padsTotal = 0, padsRouted = 0;
	for (const n of input.networks) { const st = padStats.get(n.name); padsTotal += st ? st.total : n.pads.length; padsRouted += st ? st.routed : 0; }
	const nets = result.nets.map(n => ({ name: n.name, prims: n.prims }));
	const drc = drcAudit(input, result, mod.polygonToRect);
	const joints = jointAudit(result);
	const edge = edgeAudit(input, nets, result.vias, mod.polygonToRect);
	const endedBy = budgetLogged || (budgetMs > 0 && wallMs >= budgetMs * 0.95) ? 'budget' : 'self';
	const run = {
		run: i, wallMs, endedBy, budgetMs,
		nets: { routed: result.routedNets.length, total: input.networks.length, failed: result.failed, partial: result.partial },
		pads: { routed: padsRouted, total: padsTotal },
		vias: result.vias.length,
		lengthMil: r1(nets.reduce((s, n) => s + renderedLength(n.prims), 0)),
		drc: { violationCount: drc.violationCount, top: drc.violations.slice(0, 5) },
		drcEdge: edge,
		joints: { gaps: joints.gaps, sharp: joints.sharp, bigArcs: joints.bigArcs },
		resultHash: hashNets(nets, result.vias),
		oldTrackObstaclesRemoved: removedOld,
		enginePhaseSeconds: phaseLog,
	};
	results.push(run);
	out(`run ${i}/${runs}: wall=${wallMs}ms endedBy=${endedBy}  nets=${run.nets.routed}/${run.nets.total}  pads=${padsRouted}/${padsTotal}  vias=${run.vias}  len=${Math.round(run.lengthMil)}mil` +
		`  drc=${drc.violationCount}  edge=${edge.violationCount}  joints(gap/sharp/bigArc)=${joints.gaps}/${joints.sharp}/${joints.bigArcs}  hash=${run.resultHash}`);
	for (const l of phaseLog) out(`    [耗时] ${l}`);
	if (run.nets.failed.length) out(`    failed: ${run.nets.failed.slice(0, 20).join(', ')}${run.nets.failed.length > 20 ? ' ...' : ''}`);
	if (run.nets.partial.length) out(`    partial: ${run.nets.partial.slice(0, 20).join(', ')}${run.nets.partial.length > 20 ? ' ...' : ''}`);
	for (const v of edge.worst.slice(0, 3)) out(`    edge ${v.kind} ${v.netA}|${v.netB} ${v.edgeMil}<${v.needMil} @${v.x},${v.y}`);
}

const ok = results.filter(r => !r.error);
const doc = {
	meta: { script: 'bench/route-dump.mjs', dump: dumpPath, dumpStructVersion: dumpVersion, expectedStructVersion: STRUCT_VERSION, node: process.version, budgetMs, ripupRounds, runs },
	runs: results,
	summary: ok.length ? {
		wallMsMedian: Math.round(median(ok.map(r => r.wallMs))),
		identicalHashAcrossRuns: new Set(ok.map(r => r.resultHash)).size === 1,
		resultHashes: ok.map(r => r.resultHash),
		endedBy: ok.map(r => r.endedBy),
	} : null,
};
if (ok.length > 1) out(`summary: median wall ${doc.summary.wallMsMedian}ms, hashes ${doc.summary.identicalHashAcrossRuns ? 'identical' : 'DIFFER'} (${doc.summary.resultHashes.join(',')})`);
if (outFile) fs.writeFileSync(outFile, JSON.stringify(doc, null, 2) + '\n', 'utf8');
void realLog;
process.exit(exitCode);
