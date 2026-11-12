#!/usr/bin/env node
/**
 * tighten-e2e-run.mjs - END-TO-END driver: REAL routing + REAL "tighten existing routing" inside the real JLCEDA Pro
 * desktop client (through its CLI), canvas geometry exported before/after, strict independent metrics computed in Node.
 *
 *   node bench/tighten-e2e-run.mjs --data-root <dir> [--case clock-pcb3] [--steps route,tighten,tighten2,drc]
 *                                  [--fault-at N]
 *                                  [--no-worker] [--keep-session] [--reuse <runDir>] [--run <name>]
 *                                  [--route-timeout-min 45] [--tighten-timeout-min 30] [--no-render] [--depot <dir>] [--force]
 *
 *   --data-root   directory holding case/<case>/ (git-ignored owner data; in a worktree point at the main checkout).
 *                 All scratch/results go to <data-root>/tmp/testwork-ask003/<run>/ (never into the repo).
 *   --steps       subset of route,tighten,tighten2,drc,fault (default: all but fault). `drc` = EDA DRC (check + checkDetailed) for the
 *                 unrouted board and after every executed stage. Snapshots are always taken (unrouted + each stage).
 *                 Optional extra step `fault` (not in the default list, e.g. --steps route,fault,tighten,drc): runs BETWEEN route
 *                 and tighten; a tighten job with globalThis.__RB_TIGHTEN_FAULT__ = { createFailAt: N } (the N-th canvas create
 *                 call throws 'fault injection'; the entry always deletes the global afterwards). Its result (aborted, adopted/
 *                 failed/verified/issues, final message, warn/error logs) is printed; snapshot-fault.json (+ drc-fault.json),
 *                 fault.json; stage column `fault` sits between routed and tight1 (compared with routed); tight1 is then
 *                 compared against `fault`, not routed.
 *   --fault-at N  N for the fault step (default 7).
 *   --no-worker   tighten calls run with globalThis.Worker = undefined (production main-thread fallback), test-only.
 *   --keep-session  do not close/destroy the session this run opened.
 *   --reuse DIR   no client: recompute metrics (+ renders) from the artefacts of an earlier run directory.
 *   --run NAME    run directory name (default run-<timestamp>). A fresh run REFUSES to start if <data-root>/tmp/testwork-ask003/<NAME>
 *                 already exists and is non-empty (old snapshots would be mixed into the metrics); pass --force to delete the stale
 *                 artefacts first (snapshot-*, drc-*, route/fault/tighten*.json, probe/doctor/metrics.json, svg/). --reuse is exempt.
 *   --force       clear stale artefacts of an existing --run directory (see above); ignored with --reuse.
 *   --depot DIR   ALSO copy the SVGs into this directory (no default). Without --depot the SVGs go only to <run>/svg/.
 *                 --no-render skips the SVGs altogether.
 *
 * Exit code: non-zero if any route/fault/tighten/snapshot/drc step failed or errored (the session is still closed and whatever
 * metrics exist are still written), if the run directory check fails, or on any fatal error.
 *
 * Flow: build bundle (esbuild, ESM, inline-worker:planning plugin replicated from config/esbuild-common.ts) ->
 *   copy the case to <run>/board -> `open --headless true` -> import the bundle once in the client (cached on globalThis)
 *   -> probe Worker support -> [snapshot unrouted, DRC] -> route (new TopologicalRouter().route(normalizeRouterOptions({})))
 *   -> snapshot + DRC -> [fault -> snapshot + DRC] -> tighten -> snapshot + DRC -> tighten2 -> snapshot + DRC -> `session close --destroy` (always).
 *   Long jobs run in the background inside the client and are polled (`start` / `status`), results over a chunk protocol.
 *
 * Artefacts in the run dir: bundle.mjs, board/, probe.json, doctor.json, route.json, tighten1.json, tighten2.json, fault.json,
 *   snapshot-{unrouted,routed,fault,tight1,tight2}.json, drc-{unrouted,routed,fault,tight1,tight2}.json, metrics.json,
 *   svg/ (the SVGs; copied to --depot too when given).
 *
 * METRICS (Node side, independent of src/): see bench/tighten-metrics.mjs. Canvas -> ctx mapping:
 *   nets: every canvas line/arc on copper layer 1/2 with a net (arc converted to centre form with the same formula as
 *         canvasArcToPrim), width = lineWidth. vias with a net -> ctx.vias; netless vias -> statics.vias.
 *   allPads/padShapes/drc: from the in-client dumpInput() (real pad shapes) + netless pads injected as cli-dump62.js does.
 *   statics.outline: layer-11 polyline(s) of the snapshot (or chained layer-11 lines/arcs). statics.fills / tracks:
 *         canvas fills and netless copper polylines/lines (this board has none). NOT covered: keepouts/regions,
 *         copper pours (fills only as plain polygons), silk/other layers, hole-to-hole, via-in-pad rules, plane layers.
 *   extra checks implemented here: duplicate/overlapping same-net same-layer copper (>1 mil centre-line overlap) and
 *         jointAudit (scripts/audit-lib.mjs) over canvas chains (prims linked by endpoints within 3 mil, oriented
 *         end-to-start, branch points break chains).
 */
import * as fs from 'node:fs';
import * as path from 'node:path';
import { spawnSync } from 'node:child_process';
import { cli, DATA_ROOT, ROOT } from '../scripts/lceda-cli.mjs';
import { jointAudit } from '../scripts/audit-lib.mjs';
import * as M from './tighten-metrics.mjs';
import { renderSnapshot, arcCentre } from './tighten-e2e-render.mjs';

// ───────────────────────── args ─────────────────────────
const argv = process.argv.slice(2);
if (argv.includes('--help') || argv.includes('-h')) {
	const src = fs.readFileSync(new URL(import.meta.url), 'utf8');
	console.log(src.slice(src.indexOf('/**'), src.indexOf('*/') + 2));
	process.exit(0);
}
const opt = (k, d) => { const i = argv.indexOf(k); return i >= 0 && argv[i + 1] !== undefined && !argv[i + 1].startsWith('--') ? argv[i + 1] : d; };
const flag = (k) => argv.includes(k);
const CASE = opt('--case', 'clock-pcb3');
const STEPS = new Set(opt('--steps', 'route,tighten,tighten2,drc').split(',').map(s => s.trim()).filter(Boolean));
const NO_WORKER = flag('--no-worker');
const FAULT_AT = Number(opt('--fault-at', '7'));
const KEEP = flag('--keep-session');
const REUSE = opt('--reuse', null);
const NO_RENDER = flag('--no-render');
const DEPOT = opt('--depot', null);   // no default: SVGs go to the depot only when --depot <dir> is passed
const FORCE = flag('--force');
const ROUTE_TIMEOUT = Number(opt('--route-timeout-min', '45')) * 60000;
const TIGHTEN_TIMEOUT = Number(opt('--tighten-timeout-min', '30')) * 60000;
const WORK_BASE = path.join(DATA_ROOT, 'tmp', 'testwork-ask003');
const RUN = opt('--run', 'run-' + new Date().toISOString().replace(/[-:]/g, '').replace(/\..*/, ''));
const RUN_DIR = REUSE ? path.resolve(REUSE) : path.join(WORK_BASE, RUN);

const sleep = (ms) => new Promise(r => setTimeout(r, ms));
const t00 = Date.now();
const stamp = () => `[${((Date.now() - t00) / 1000).toFixed(1).padStart(7)}s]`;
const log = (...a) => console.log(stamp(), ...a);
const wjson = (name, obj) => fs.writeFileSync(path.join(RUN_DIR, name), JSON.stringify(obj, null, 1), 'utf8');
const rjson = (name) => JSON.parse(fs.readFileSync(path.join(RUN_DIR, name), 'utf8'));
const has = (name) => fs.existsSync(path.join(RUN_DIR, name));
/** step failures (route/fault/tighten/snapshot/drc): collected, reported at the end, make the process exit non-zero */
const FAILURES = [];
const fail = (msg) => { FAILURES.push(msg); log('FAILURE: ' + msg.split('\n')[0].slice(0, 300)); };

// ───────────────────────── bundle ─────────────────────────
async function buildBundle(outFile) {
	const esbuild = await import('esbuild');
	// replicated from config/esbuild-common.ts (inline-worker:planning -> IIFE string of src/worker/planning-worker.ts)
	const inlinePlanningWorker = {
		name: 'inline-planning-worker',
		setup(build) {
			build.onResolve({ filter: /^inline-worker:planning$/ }, a => ({ path: a.path, namespace: 'inline-worker' }));
			build.onLoad({ filter: /.*/, namespace: 'inline-worker' }, async () => {
				const r = await esbuild.build({
					entryPoints: [path.join(ROOT, 'src/worker/planning-worker.ts')],
					bundle: true, write: false, format: 'iife', platform: 'browser', target: 'es2020',
					minify: false, sourcemap: false, logLevel: 'silent',
				});
				return { contents: r.outputFiles[0].text, loader: 'text' };
			});
		},
	};
	await esbuild.build({
		entryPoints: [path.join(ROOT, 'bench', 'tighten-e2e-entry.ts')],
		outfile: outFile, bundle: true, format: 'esm', platform: 'browser', target: 'es2022',
		legalComments: 'none', plugins: [inlinePlanningWorker], logLevel: 'warning',
	});
}

// ───────────────────────── client ─────────────────────────
const CODE_FILE = path.join(ROOT, 'bench', 'tighten-e2e-eda.js');
let SID = null;
function inv(args, timeoutMs = 120000) {
	const r = cli(['invoke', '--session', SID, '--ext-uuid', 'eda', '--code-file', CODE_FILE,
		'--args', JSON.stringify(args), '--timeout', String(timeoutMs)], timeoutMs + 30000);
	if (!r.json?.ok) throw new Error(`invoke ${args.cmd}${args.job ? ':' + args.job : ''} failed: ${(r.json ? JSON.stringify(r.json) : r.raw + ' ' + r.stderr).slice(0, 1500)}`);
	return r.json.value;
}

/** start a background job in the client and poll it to the end; returns {status, result, error, logs, progress, elapsedMs, meta} */
async function runJob(name, startArgs, { maxWaitMs, quiet = false, showLogs = false } = {}) {
	inv({ cmd: 'start', job: name, ...startArgs });
	const t0 = Date.now();
	const logs = [], progress = [];
	let lf = 0, pf = 0, lastBeat = 0, last;
	for (;;) {
		await sleep(name === 'route' ? 2000 : 800);
		last = inv({ cmd: 'status', job: name, logFrom: lf, progFrom: pf });
		if (!last.exists) throw new Error(`job ${name} vanished from the client (session state lost?)`);
		for (const l of last.logs) {
			logs.push(l);
			if (showLogs || l.level === 'warn' || l.level === 'error') if (!quiet) log(`  [${name}/${l.level}] ${l.message}`.slice(0, 300));
		}
		for (const p of last.progress) progress.push(p);
		lf += last.logs.length; pf += last.progress.length;
		if (Date.now() - lastBeat > 15000 && !quiet) {
			lastBeat = Date.now();
			const lp = progress[progress.length - 1];
			log(`  ${name}: ${last.status} ${(last.elapsedMs / 1000).toFixed(0)}s, ${last.progressCount} progress msgs, ${last.logTotal} logs${lp ? `, last: ${lp.stage} ${lp.percentage}% ${String(lp.message).slice(0, 90)}` : ''}`);
		}
		if (last.status !== 'running' && lf >= last.logTotal && pf >= last.progTotal) break;
		if (last.status !== 'running') continue;   // drain
		if (Date.now() - t0 > maxWaitMs) throw new Error(`job ${name} exceeded ${(maxWaitMs / 60000).toFixed(0)} min wall limit (still running in the client)`);
	}
	return { status: last.status, result: last.result, error: last.error, stack: last.stack, logs, progress, progressCount: last.progressCount, elapsedMs: last.elapsedMs, meta: last.meta };
}

async function takeSnapshot(label) {
	const r = await runJob('snapshot', {}, { maxWaitMs: 180000, quiet: true });
	if (r.status !== 'done') throw new Error(`snapshot ${label} failed: ${r.error}`);
	let json = '';
	for (let i = 0; i < r.result.chunkCount; i++) {
		const part = inv({ cmd: 'chunk', token: r.result.token, i }, 120000);
		json += part.data;
		if (part.done) break;
	}
	const snap = JSON.parse(json);
	snap.label = label;
	wjson(`snapshot-${label}.json`, snap);
	log(`snapshot ${label}: lines ${snap.lines.length}, arcs ${snap.arcs.length}, vias ${snap.vias.length}, pads ${snap.pads.length}, polylines ${snap.polylines.length}, fills ${snap.fills.length} (${(r.result.totalChars / 1e6).toFixed(2)} MB, ${r.result.wallMs} ms in client)`);
	return snap;
}

async function takeDrc(label) {
	const r = await runJob('drc', {}, { maxWaitMs: 600000, quiet: true });
	if (r.status !== 'done') { wjson(`drc-${label}.json`, { error: r.error }); fail(`DRC ${label} ${r.status}: ${r.error}`); return null; }
	wjson(`drc-${label}.json`, r.result);
	const det = r.result.detailed;
	log(`EDA DRC ${label}: check() total ${r.result.check?.total} (${(r.result.check?.categories ?? []).map(c => `${c.name}=${c.count}`).join(', ')}); checkDetailed totalCount ${det?.raw?.totalCount ?? det?.count} state=${det?.state}; ${(r.result.wallMs / 1000).toFixed(1)}s`);
	return r.result;
}

/** print the count and the first 5 write-back verification issues (net/layer/kind/x/y) */
function printIssues(tag, issues) {
	const list = Array.isArray(issues) ? issues : [];
	log(`  ${tag} issues: ${list.length}`);
	for (const i of list.slice(0, 5)) log(`    net ${i.net} layer ${i.layer} ${i.kind} @(${Number(i.x).toFixed(1)}, ${Number(i.y).toFixed(1)})`);
}

// ───────────────────────── analysis ─────────────────────────
const NUM = (v) => (typeof v === 'number' && Number.isFinite(v) ? v : 0);
const r1 = (v) => Math.round(v * 10) / 10;
const r2 = (v) => Math.round(v * 100) / 100;

function chainOutline(snap) {
	const poly = (snap.polylines ?? []).filter(p => (p.layer ?? p.layerId) === 11 && p.pts?.length >= 3);
	if (poly.length) {
		poly.sort((a, b) => b.pts.length - a.pts.length);
		return { pts: poly[0].pts.map(q => ({ x: q.x, y: q.y })), source: `layer-11 polyline (${poly.length} found, longest used, ${poly[0].pts.length} pts)` };
	}
	const segs = [];
	for (const l of snap.lines ?? []) if (l.layer === 11) segs.push([[l.startX, l.startY], [l.endX, l.endY]]);
	for (const a of snap.arcs ?? []) {
		if (a.layer !== 11) continue;
		const c = arcCentre(a);
		if (!c) continue;
		const n = Math.max(4, Math.ceil(Math.abs(c.sweep) / (Math.PI / 12)));
		const pts = [];
		for (let k = 0; k <= n; k++) { const t = c.startAngle + c.sweep * k / n; pts.push([c.cx + c.r * Math.cos(t), c.cy + c.r * Math.sin(t)]); }
		for (let k = 0; k < n; k++) segs.push([pts[k], pts[k + 1]]);
	}
	if (!segs.length) return { pts: null, source: 'none found' };
	const used = new Array(segs.length).fill(false);
	let cur = segs[0].slice(); used[0] = true;
	const d = (p, q) => Math.hypot(p[0] - q[0], p[1] - q[1]);
	for (let guard = 0; guard < segs.length; guard++) {
		const tail = cur[cur.length - 1];
		let found = false;
		for (let i = 0; i < segs.length; i++) {
			if (used[i]) continue;
			if (d(segs[i][0], tail) < 1) { cur.push(segs[i][1]); used[i] = true; found = true; break; }
			if (d(segs[i][1], tail) < 1) { cur.push(segs[i][0]); used[i] = true; found = true; break; }
		}
		if (!found) break;
	}
	return { pts: cur.map(p => ({ x: p[0], y: p[1] })), source: `chained layer-11 segments (${used.filter(Boolean).length}/${segs.length} chained)` };
}

/**
 * EDA reports ELLIPSE pad shapes with MIXED units (first value mm, second mil, e.g. ['ELLIPSE', 1.3, 51.2] = round 51.2 mil);
 * same normalisation as normalizeEllipse in src/router/tighten-core.ts (rubberband-engine.normalizeEllipseUnits).
 */
function normalizeEllipseUnits(shape) {
	if (!Array.isArray(shape) || String(shape[0] ?? '').toUpperCase() !== 'ELLIPSE') return shape;
	const a = shape[1], b = shape[2];
	if (!(a > 0) || !(b > 0)) return shape;
	if (Math.abs(a * 39.3701 - b) <= Math.max(0.5, b * 0.02)) return ['ELLIPSE', b, b];
	if (Math.abs(a - b) <= 0.01 && a < 1) return ['ELLIPSE', a * 39.3701, b * 39.3701];
	return shape;
}
function normalizePadShapes(ps) {
	const out = {};
	for (const [k, v] of Object.entries(ps ?? {})) out[k] = v && Array.isArray(v.pad) ? { ...v, pad: normalizeEllipseUnits(v.pad) } : v;
	return out;
}

function buildCtx(snap) {
	const nets = new Map();
	const prims = [];
	for (const l of snap.lines ?? []) {
		if ((l.layer !== 1 && l.layer !== 2) || !l.net) continue;
		prims.push({ prim: { kind: 'line', net: l.net, layer: l.layer, x1: l.startX, y1: l.startY, x2: l.endX, y2: l.endY }, width: NUM(l.lineWidth) || 10, id: l.primitiveId });
	}
	for (const a of snap.arcs ?? []) {
		if ((a.layer !== 1 && a.layer !== 2) || !a.net) continue;
		const c = arcCentre(a);
		if (!c) continue;
		prims.push({ prim: { kind: 'arc', net: a.net, layer: a.layer, cx: c.cx, cy: c.cy, r: c.r, startAngle: c.startAngle, sweep: c.sweep }, width: NUM(a.lineWidth) || 10, id: a.primitiveId });
	}
	for (const t of prims) {
		if (!nets.has(t.prim.net)) nets.set(t.prim.net, { name: t.prim.net, tracks: [] });
		nets.get(t.prim.net).tracks.push({ prim: t.prim, width: t.width, layer: t.prim.layer, id: t.id });
	}
	const vias = [], svias = [];
	for (const v of snap.vias ?? []) {
		const o = { net: v.net ?? '', x: v.x, y: v.y, dia: NUM(v.diameter) || 24, hole: NUM(v.holeDiameter) || 12 };
		(o.net ? vias : svias).push(o);
	}
	const o = chainOutline(snap);
	const sTracks = [];
	for (const l of snap.lines ?? []) {
		if ((l.layer !== 1 && l.layer !== 2) || l.net) continue;
		sTracks.push({ id: l.primitiveId, net: '', layer: l.layer, lw: NUM(l.lineWidth) || 10, pts: [{ x: l.startX, y: l.startY }, { x: l.endX, y: l.endY }] });
	}
	for (const p of snap.polylines ?? []) {
		const ly = p.layer ?? p.layerId;
		if ((ly !== 1 && ly !== 2) || !(p.pts?.length >= 2)) continue;
		sTracks.push({ id: p.primitiveId, net: p.net ?? '', layer: ly, lw: NUM(p.lineWidth) || 10, pts: p.pts });
	}
	const fills = [];
	for (const fl of snap.fills ?? []) {
		const ly = fl.layer ?? fl.layerId;
		if ((ly !== 1 && ly !== 2) || !(fl.pts?.length >= 3)) continue;
		fills.push({ net: fl.net ?? '', layer: ly, lw: NUM(fl.lineWidth), pts: fl.pts });
	}
	// REAL pads only: straight from the canvas pad primitives. The in-client dumpInput()/collectPads() allPads list is NOT used
	// because once tracks exist getAllPrimitivesByNet() returns lines/arcs that collectPads() turns into phantom "pads"
	// (observed: 637 allPads on the routed board vs 212 real pads) - that pollutes DRC/connectivity. Reported in info.
	const allPads = (snap.pads ?? []).map((p, index) => {
		const pd = Array.isArray(p.pad) ? normalizeEllipseUnits(p.pad) : [];
		const hole = Array.isArray(p.hole) && p.hole.length >= 2 ? (p.hole[1] ?? 0) : 0;
		const w = pd[1] ?? 0, h = pd[2] ?? 0;
		return {
			x: p.x, y: p.y, net: p.net ?? '', id: p.primitiveId ?? `pad_${index}`,
			layer: p.layer === 12 ? 12 : (p.layer === 2 ? 2 : 1), padType: p.layer === 12 ? 'th' : 'smd', holeDiameter: hole,
			radius: Math.max(w / 2, h / 2, hole / 2), bodyRadius: Math.max(Math.min(w, h) / 2, hole / 2), rotation: p.rotation ?? 0, index,
		};
	});
	const realKeys = new Set(allPads.map(p => `${p.x.toFixed(2)},${p.y.toFixed(2)}`));
	const phantom = (snap.allPads ?? []).filter(p => !realKeys.has(`${p.x.toFixed(2)},${p.y.toFixed(2)}`)).length;
	return {
		ctx: {
			drc: snap.drc, allPads, padShapes: normalizePadShapes(snap.padShapes), vias,
			statics: { outline: o.pts ?? [], tracks: sTracks, vias: svias, fills },
			nets: [...nets.values()],
		},
		info: { outline: o.source, realPads: allPads.length, inClientAllPads: (snap.allPads ?? []).length, phantomAllPads: phantom, staticTracks: sTracks.length, staticVias: svias.length, fills: fills.length, copperPrims: prims.length },
	};
}

const segDistPt = (px, py, x1, y1, x2, y2) => {
	const dx = x2 - x1, dy = y2 - y1, l2 = dx * dx + dy * dy;
	let t = l2 > 0 ? ((px - x1) * dx + (py - y1) * dy) / l2 : 0;
	t = t < 0 ? 0 : t > 1 ? 1 : t;
	return Math.hypot(px - x1 - t * dx, py - y1 - t * dy);
};
function distToPrim(px, py, p) {
	if (p.kind === 'line') return segDistPt(px, py, p.x1, p.y1, p.x2, p.y2);
	const ang = Math.atan2(py - p.cy, px - p.cx);
	let rel = (ang - p.startAngle) * (p.sweep >= 0 ? 1 : -1);
	rel = ((rel % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI);
	if (rel <= Math.abs(p.sweep)) return Math.abs(Math.hypot(px - p.cx, py - p.cy) - p.r);
	const e = M.primEnds(p);
	return Math.min(Math.hypot(px - e[0][0], py - e[0][1]), Math.hypot(px - e[1][0], py - e[1][1]));
}
function samplePrimPts(p, step, cb) {
	const L = M.primLength(p);
	const n = Math.max(1, Math.ceil(L / step));
	for (let i = 0; i < n; i++) {
		const t = (i + 0.5) / n;
		if (p.kind === 'line') cb(p.x1 + (p.x2 - p.x1) * t, p.y1 + (p.y2 - p.y1) * t, L / n);
		else { const a = p.startAngle + p.sweep * t; cb(p.cx + p.r * Math.cos(a), p.cy + p.r * Math.sin(a), L / n); }
	}
}

/** (a) duplicate / overlapping same-net same-layer copper: centre-lines coincide over > 1 mil */
function duplicateCopper(ctx) {
	const out = [];
	const groups = new Map();
	for (const n of ctx.nets) for (const t of n.tracks) { const k = n.name + '|' + t.layer; if (!groups.has(k)) groups.set(k, []); groups.get(k).push(t.prim); }
	const TOL = 0.3;
	for (const [k, ps] of groups) {
		for (let i = 0; i < ps.length; i++) {
			for (let j = i + 1; j < ps.length; j++) {
				const a = ps[i], b = ps[j];
				if (a.kind !== b.kind) continue;
				if (a.kind === 'line') {
					const la = Math.hypot(a.x2 - a.x1, a.y2 - a.y1), lb = Math.hypot(b.x2 - b.x1, b.y2 - b.y1);
					if (la < 1e-6 || lb < 1e-6) continue;
					const dx = (a.x2 - a.x1) / la, dy = (a.y2 - a.y1) / la;
					const perp = (x, y) => Math.abs((x - a.x1) * dy - (y - a.y1) * dx);
					if (perp(b.x1, b.y1) > TOL || perp(b.x2, b.y2) > TOL) continue;
					const pr = (x, y) => (x - a.x1) * dx + (y - a.y1) * dy;
					const s0 = Math.min(pr(b.x1, b.y1), pr(b.x2, b.y2)), s1 = Math.max(pr(b.x1, b.y1), pr(b.x2, b.y2));
					const ov = Math.min(la, s1) - Math.max(0, s0);
					if (ov > 1) out.push({ net: k.split('|')[0], layer: a.layer, kind: 'line', overlapMil: r2(ov), at: [r1(a.x1 + dx * Math.max(0, s0)), r1(a.y1 + dy * Math.max(0, s0))] });
				} else {
					if (Math.hypot(a.cx - b.cx, a.cy - b.cy) > TOL * 2 || Math.abs(a.r - b.r) > TOL * 2) continue;
					const span = (p) => { const s = p.sweep >= 0 ? p.startAngle : p.startAngle + p.sweep; return [((s % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI), Math.abs(p.sweep)]; };
					const [sa, wa] = span(a), [sb, wb] = span(b);
					let best = 0;
					for (const off of [-2 * Math.PI, 0, 2 * Math.PI]) best = Math.max(best, Math.min(sa + wa, sb + off + wb) - Math.max(sa, sb + off));
					if (best * a.r > 1) out.push({ net: k.split('|')[0], layer: a.layer, kind: 'arc', overlapMil: r2(best * a.r), at: [r1(a.cx), r1(a.cy)] });
				}
			}
		}
	}
	return { count: out.length, list: out.slice(0, 10) };
}

/** (b) jointAudit over canvas chains */
function canvasJointAudit(ctx, linkTol = 3) {
	const result = { nets: [] };
	const groups = new Map();
	for (const n of ctx.nets) for (const t of n.tracks) { const k = n.name + '@' + t.layer; if (!groups.has(k)) groups.set(k, []); groups.get(k).push(t.prim); }
	const orient = (p, flip) => {
		if (!flip) return p;
		if (p.kind === 'line') return { ...p, x1: p.x2, y1: p.y2, x2: p.x1, y2: p.y1 };
		return { ...p, startAngle: p.startAngle + p.sweep, sweep: -p.sweep };
	};
	let chains = 0;
	for (const [k, ps] of groups) {
		const ends = [];
		ps.forEach((p, i) => { const e = M.primEnds(p); ends.push({ i, e: 0, x: e[0][0], y: e[0][1] }, { i, e: 1, x: e[1][0], y: e[1][1] }); });
		// cluster: link only a pair of endpoints of DIFFERENT prims that are each other's single neighbour within linkTol
		const nb = ends.map(() => []);
		for (let a = 0; a < ends.length; a++) for (let b = a + 1; b < ends.length; b++) {
			if (ends[a].i === ends[b].i) continue;
			if (Math.hypot(ends[a].x - ends[b].x, ends[a].y - ends[b].y) <= linkTol) { nb[a].push(b); nb[b].push(a); }
		}
		const link = new Array(ends.length).fill(-1);
		for (let a = 0; a < ends.length; a++) if (nb[a].length === 1 && nb[nb[a][0]].length === 1) link[a] = nb[a][0];
		const idx = (i, e) => i * 2 + e;
		const seen = new Array(ps.length).fill(false);
		const walk = (startPrim, entryEnd) => {
			const chain = [];
			let i = startPrim, enter = entryEnd;
			while (i !== -1 && !seen[i]) {
				seen[i] = true;
				chain.push(orient(ps[i], enter === 1));
				const exitEnd = 1 - enter;
				const nxt = link[idx(i, exitEnd)];
				if (nxt === -1) break;
				i = ends[nxt].i; enter = ends[nxt].e;
			}
			return chain;
		};
		for (let i = 0; i < ps.length; i++) {
			if (seen[i]) continue;
			if (link[idx(i, 0)] === -1) result.nets.push({ name: k, prims: walk(i, 0) });
			else if (link[idx(i, 1)] === -1) result.nets.push({ name: k, prims: walk(i, 1) });
		}
		for (let i = 0; i < ps.length; i++) if (!seen[i]) result.nets.push({ name: k + '(cycle)', prims: walk(i, 0) });
	}
	chains = result.nets.length;
	const a = jointAudit(result);
	return { chains, joints: a.joints, gaps: a.gaps, sharp: a.sharp, bigArcs: a.bigArcs, first5: a.issues.slice(0, 5) };
}

/** geometry delta between two stages: length of A not covered by B (removed) and of B not covered by A (added) */
function geomDelta(ctxA, ctxB, step = 2, tol = 0.6) {
	const index = (ctx) => { const m = new Map(); for (const n of ctx.nets) for (const t of n.tracks) { const k = n.name + '|' + t.layer; if (!m.has(k)) m.set(k, []); m.get(k).push(t.prim); } return m; };
	const IA = index(ctxA), IB = index(ctxB);
	const cells = new Map(), changedNets = new Set();
	const CELL = 200;
	let removed = 0, added = 0;
	const one = (src, dst, sign) => {
		let tot = 0;
		for (const [k, ps] of src) {
			const others = dst.get(k) ?? [];
			for (const p of ps) samplePrimPts(p, step, (x, y, len) => {
				let ok = false;
				for (const q of others) { if (distToPrim(x, y, q) <= tol) { ok = true; break; } }
				if (!ok) {
					tot += len; changedNets.add(k.split('|')[0]);
					const ck = `${Math.floor(x / CELL)},${Math.floor(y / CELL)}`;
					const c = cells.get(ck) ?? { x: 0, y: 0, len: 0, n: 0 };
					c.x += x * len; c.y += y * len; c.len += len; c.n++;
					cells.set(ck, c);
				}
			});
		}
		return tot;
	};
	removed = one(IA, IB, -1); added = one(IB, IA, 1);
	return { removedMil: r1(removed), addedMil: r1(added), changedNets: [...changedNets].sort(), cells };
}

function edaDrcSummary(drcRes) {
	if (!drcRes) return null;
	const det = drcRes.detailed?.raw;
	const viol = det?.violations ?? [];
	const byType = {};
	for (const v of viol) { const k = `${v.errorType} | ${v.obj1Type}${v.obj2Type ? ' - ' + v.obj2Type : ''}`; byType[k] = (byType[k] ?? 0) + 1; }
	const nonConn = viol.filter(v => v.errorType !== 'No Connection');
	const pick = [...nonConn, ...viol.filter(v => v.errorType === 'No Connection')].slice(0, 20)
		.map(v => ({ errorType: v.errorType, obj1: v.obj1Type, obj2: v.obj2Type, pos: v.position ? [r1(v.position.x), r1(v.position.y)] : null, layer: v.layerIds, minDistanceMm: v.minDistance, clearanceMm: v.clearance }));
	return {
		checkTotal: drcRes.check?.total ?? null, checkCategories: drcRes.check?.categories ?? [],
		detailedTotal: det?.totalCount ?? null, detailedState: det?.state ?? null, failedItems: det?.failedItems ?? null,
		byType, nonConnectionCount: nonConn.length, noConnectionCount: viol.length - nonConn.length, first20: pick,
		checkError: drcRes.checkError, detailedError: drcRes.detailedError,
	};
}
/** EDA DRC violations of `after` with no same-type violation of `before` within 3 mil (ids are reassigned by EDA, so position based) */
function edaDrcNew(beforeRes, afterRes) {
	const a = afterRes?.detailed?.raw?.violations, b = beforeRes?.detailed?.raw?.violations;
	if (!a || !b) return null;
	const out = [];
	for (const v of a) {
		const hit = b.some(w => w.errorType === v.errorType && w.obj1Type === v.obj1Type && w.obj2Type === v.obj2Type &&
			Math.hypot((w.position?.x ?? 1e9) - (v.position?.x ?? -1e9), (w.position?.y ?? 1e9) - (v.position?.y ?? -1e9)) <= 3);
		if (!hit) out.push({ errorType: v.errorType, obj1: v.obj1Type, obj2: v.obj2Type, pos: [r1(v.position?.x), r1(v.position?.y)], minDistanceMm: v.minDistance, clearanceMm: v.clearance });
	}
	return { count: out.length, first10: out.slice(0, 10) };
}

function netTable(ctx) {
	const t = {};
	for (const n of ctx.nets) {
		let L = 0, ln = 0, ar = 0;
		for (const x of n.tracks) { L += M.primLength(x.prim); if (x.prim.kind === 'line') ln++; else ar++; }
		t[n.name] = { lines: ln, arcs: ar, lengthMil: r2(L) };
	}
	return t;
}

/** nearest OTHER same-net same-layer copper (a prim not ending at the corner): centre-line distance + that track's width. A free corner with
 *  distance < (w1+w2)/2 sits where two own-net runs overlap (the tighten job pins such crossings as 'junction' fixed points). */
function ownNetCopperNear(ctx, c) {
	let best = null;
	for (const n of ctx.nets) {
		if (n.name !== c.net) continue;
		for (const t of n.tracks) {
			if (t.layer !== c.layer) continue;
			const e = M.primEnds(t.prim);
			if (e.some(q => Math.hypot(q[0] - c.x, q[1] - c.y) <= 0.8)) continue;
			const d = distToPrim(c.x, c.y, t.prim);
			if (!best || d < best.dist) best = { dist: r2(d), width: t.width, kind: t.prim.kind };
		}
	}
	return best;
}

function analyzeStage(label, snap, prev, prevLabel, drcRes, prevDrc, tightenInfo) {
	const { ctx, info } = buildCtx(snap);
	const c = M.counts(ctx);
	const sc = M.sharpCorners(ctx);
	const sd = M.strictDrc(ctx);
	const conn = M.connectivity(ctx);
	const dang = M.danglingEnds(ctx);
	const m = {
		label, info,
		lines: c.lines, arcs: c.arcs, lengthMil: r1(c.lengthMil), nets: ctx.nets.length, vias: snap.vias.length,
		sharp: { count: sc.count, free: sc.free, fixed: sc.fixed, freeList: sc.list.filter(s => !s.atFixed).map(c => ({ net: c.net, layer: c.layer, x: r2(c.x), y: r2(c.y), angle: c.angle, type: c.type, atPad: c.atPad, atVia: c.atVia, inPad: c.inPad, ownNetCopper: ownNetCopperNear(ctx, c) })) },
		strictDrc: { count: sd.violations.length, first10: sd.violations.slice(0, 10).map(v => ({ item: v.item, netA: v.netA, netB: v.netB, kind: v.kind, gap: r2(v.gap), rule: r2(v.rule), x: r1(v.x), y: r1(v.y) })) },
		dangling: { count: dang.count, first5: dang.list.slice(0, 5).map(d => ({ net: d.net, layer: d.layer, x: r1(d.x), y: r1(d.y) })) },
		connectivity: {
			netsMultiPart: Object.entries(conn).filter(([, v]) => v.parts.length > 1).length,
			netsFloating: Object.entries(conn).filter(([, v]) => v.floating > 0).length,
			netsTotal: Object.keys(conn).length,
		},
		duplicates: duplicateCopper(ctx),
		jointAudit: canvasJointAudit(ctx),
		edaDrc: edaDrcSummary(drcRes),
		tighten: tightenInfo ?? null,
		netTable: netTable(ctx),
	};
	if (prev) {
		const pc = prev.ctx;
		const nv = M.newViolations(prev.sd, sd);
		m.vsPrev = {
			prev: prevLabel,
			newStrictViolations: { count: nv.length, first10: nv.slice(0, 10).map(v => ({ item: v.item, netA: v.netA, netB: v.netB, kind: v.kind, gap: r2(v.gap), rule: r2(v.rule), beforeGap: v.beforeGap === null ? null : r2(v.beforeGap), x: r1(v.x), y: r1(v.y) })) },
			connectivityDiff: M.connectivityDiff(prev.conn, conn),
			widthsPreserved: M.widthsPreserved(pc.nets, ctx.nets),
			edaDrcNew: edaDrcNew(prevDrc, drcRes),
			// per-prim comparison with tight tolerances (prims matched one-to-one; unmatched = changed): idempotence of a second pass
			primDiff: (() => { const o = {}; for (const tol of [0.01, 0.1, 0.5]) { const sg = M.sameGeometry(pc.nets, ctx.nets, tol); o['tol' + tol] = { netsDiffering: sg.diffs.length, unmatchedPrims: sg.diffs.reduce((a, d) => a + d.unmatched, 0), extraPrims: sg.diffs.reduce((a, d) => a + d.extra, 0), nets: sg.diffs.slice(0, 12).map(d => d.net) }; } return o; })(),
			geomDelta: (() => { const g = geomDelta(pc, ctx); return { removedMil: g.removedMil, addedMil: g.addedMil, changedNets: g.changedNets }; })(),
			lengthDeltaMil: r1(c.lengthMil - M.counts(pc).lengthMil),
			primDelta: { lines: c.lines - M.counts(pc).lines, arcs: c.arcs - M.counts(pc).arcs },
			viasBefore: prev.snap.vias.length, viasAfter: snap.vias.length,
		};
	}
	return { m, state: { ctx, sd, conn, snap } };
}

// ───────────────────────── zoom regions ─────────────────────────
function pickZoomBoxes(routedSnap, tightSnap, newViolPts) {
	const A = buildCtx(routedSnap).ctx, B = buildCtx(tightSnap).ctx;
	const g = geomDelta(A, B);
	const CELL = 200;
	const score = new Map();
	for (const [k, c] of g.cells) score.set(k, { x: c.x / c.len, y: c.y / c.len, s: c.len });
	for (const v of newViolPts) {
		const k = `${Math.floor(v[0] / CELL)},${Math.floor(v[1] / CELL)}`;
		const e = score.get(k) ?? { x: v[0], y: v[1], s: 0 };
		e.s += 200; score.set(k, e);
	}
	let list = [...score.values()].filter(e => e.s > 0).sort((a, b) => b.s - a.s);
	let basis = 'changed copper length (tight1 vs routed) + NEW strict violations';
	if (list.length < 3) {
		// nothing (or little) changed: fall back to where the routed board has free sharp corners / most copper
		const sc = M.sharpCorners(A).list.filter(s => !s.fixed);
		const m = new Map();
		for (const s of sc) { const k = `${Math.floor(s.x / CELL)},${Math.floor(s.y / CELL)}`; const e = m.get(k) ?? { x: 0, y: 0, s: 0, n: 0 }; e.x += s.x; e.y += s.y; e.n++; e.s += 1; m.set(k, e); }
		const extra = [...m.values()].map(e => ({ x: e.x / e.n, y: e.y / e.n, s: e.s })).sort((a, b) => b.s - a.s);
		list = [...list, ...extra];
		basis += ' / fallback: cells with most free sharp corners in the routed board';
	}
	const chosen = [];
	for (const e of list) {
		if (chosen.every(c => Math.hypot(c.x - e.x, c.y - e.y) >= 320)) chosen.push(e);
		if (chosen.length === 3) break;
	}
	return { basis, boxes: chosen.map(e => ({ centre: [r1(e.x), r1(e.y)], score: r1(e.s), box: [Math.round(e.x - 160), Math.round(e.y - 120), Math.round(e.x + 160), Math.round(e.y + 120)] })) };
}

// ───────────────────────── report ─────────────────────────
function printTable(stages) {
	const cols = stages.map(s => s.label);
	const rows = [];
	const row = (name, fn) => rows.push([name, ...stages.map(s => { try { const v = fn(s); return v === undefined || v === null ? '-' : String(v); } catch { return '?'; } })]);
	row('lines', s => s.lines); row('arcs', s => s.arcs); row('vias', s => s.vias); row('total length (mil)', s => s.lengthMil);
	row('sharp corners (count/free/fixed)', s => `${s.sharp.count}/${s.sharp.free}/${s.sharp.fixed}`);
	row('strictDrc violations', s => s.strictDrc.count);
	row('NEW strict viol vs prev', s => s.vsPrev?.newStrictViolations.count);
	row('connectivityDiff vs prev (n)', s => s.vsPrev?.connectivityDiff.length);
	row('dangling ends', s => s.dangling.count);
	row('nets multi-part / floating', s => `${s.connectivity.netsMultiPart}/${s.connectivity.netsFloating} of ${s.connectivity.netsTotal}`);
	row('widthsPreserved vs prev', s => s.vsPrev ? s.vsPrev.widthsPreserved.ok : undefined);
	row('EDA DRC total (detailed)', s => s.edaDrc?.detailedTotal);
	row('EDA DRC non-"No Connection"', s => s.edaDrc?.nonConnectionCount);
	row('EDA DRC "No Connection"', s => s.edaDrc?.noConnectionCount);
	row('EDA DRC NEW vs prev (pos)', s => s.vsPrev?.edaDrcNew?.count);
	row('duplicate copper pairs', s => s.duplicates.count);
	row('jointAudit gaps/sharp/bigArcs', s => `${s.jointAudit.gaps}/${s.jointAudit.sharp}/${s.jointAudit.bigArcs} (joints ${s.jointAudit.joints})`);
	row('geom removed/added mil vs prev', s => s.vsPrev ? `${s.vsPrev.geomDelta.removedMil}/${s.vsPrev.geomDelta.addedMil}` : undefined);
	row('length delta vs prev (mil)', s => s.vsPrev?.lengthDeltaMil);
	row('tighten adopted/failed/aborted/total', s => s.tighten ? `${s.tighten.changed}/${s.tighten.failed}/${s.tighten.aborted}/${s.tighten.total}` : undefined);
	row('tighten savedMil (reported)', s => s.tighten ? r1(s.tighten.savedMil) : undefined);
	const w0 = Math.max(...rows.map(r => r[0].length));
	const wc = cols.map((c, i) => Math.max(c.length, ...rows.map(r => r[i + 1].length)));
	console.log('\n' + 'metric'.padEnd(w0) + ' | ' + cols.map((c, i) => c.padEnd(wc[i])).join(' | '));
	console.log('-'.repeat(w0) + '-+-' + wc.map(w => '-'.repeat(w)).join('-+-'));
	for (const r of rows) console.log(r[0].padEnd(w0) + ' | ' + r.slice(1).map((v, i) => v.padEnd(wc[i])).join(' | '));
	console.log('');
	for (const s of stages) {
		const lists = [
			['FREE sharp corners (complete list)', s.sharp.freeList],
			['strictDrc first10', s.strictDrc.first10],
			['NEW strict viol first10 (vs ' + (s.vsPrev?.prev ?? '-') + ')', s.vsPrev?.newStrictViolations.first10],
			['connectivityDiff', s.vsPrev?.connectivityDiff],
			['dangling first5', s.dangling.first5],
			['duplicates first10', s.duplicates.list],
			['jointAudit first5', s.jointAudit.first5],
			['EDA DRC NEW first10', s.vsPrev?.edaDrcNew?.first10],
			['EDA DRC first20 (non-"No Connection" first)', s.edaDrc?.first20],
		];
		console.log(`--- ${s.label}`);
		for (const [name, l] of lists) if (l && l.length) console.log(`  ${name}:\n` + l.map(x => '    ' + JSON.stringify(x)).join('\n'));
		if (s.edaDrc) console.log('  EDA DRC by type: ' + JSON.stringify(s.edaDrc.byType));
	}
}

// ───────────────────────── main ─────────────────────────
async function main() {
	if (REUSE) {
		if (!fs.existsSync(RUN_DIR) || !fs.statSync(RUN_DIR).isDirectory()) throw new Error(`--reuse: run dir not found: ${RUN_DIR}`);
	} else if (fs.existsSync(RUN_DIR) && fs.readdirSync(RUN_DIR).length > 0) {
		if (!FORCE) throw new Error(`run dir ${RUN_DIR} already exists and is not empty (old snapshots would be mixed into the metrics). Use a new --run name, --force to clear the stale artefacts, or --reuse <runDir> to only recompute metrics.`);
		const stale = /^(snapshot-.*\.json|drc-.*\.json|route\.json|fault\.json|tighten\d*\.json|probe\.json|doctor\.json|metrics\.json)$/;
		let n = 0;
		for (const f of fs.readdirSync(RUN_DIR)) if (stale.test(f)) { fs.rmSync(path.join(RUN_DIR, f), { force: true }); n++; }
		fs.rmSync(path.join(RUN_DIR, 'svg'), { recursive: true, force: true });
		console.log(stamp(), `--force: removed ${n} stale artefact file(s) + svg/ from ${RUN_DIR}`);
	}
	fs.mkdirSync(RUN_DIR, { recursive: true });
	log(`run dir ${RUN_DIR}${REUSE ? ' (REUSE: no client)' : ''}; steps=${[...STEPS].join(',')}; case=${CASE}${NO_WORKER ? '; --no-worker' : ''}`);
	
	if (!REUSE) {
		const caseDir = path.join(DATA_ROOT, 'case', CASE);
		if (!fs.existsSync(caseDir)) throw new Error(`case dir not found: ${caseDir}`);
		const boardDir = path.join(RUN_DIR, 'board');
		fs.rmSync(boardDir, { recursive: true, force: true });
		fs.cpSync(caseDir, boardDir, { recursive: true });
		const eprj = fs.readdirSync(boardDir).find(f => f.endsWith('.eprj3'));
		if (!eprj) throw new Error('no .eprj3 in ' + boardDir);
		const bundle = path.join(RUN_DIR, 'bundle.mjs');
		await buildBundle(bundle);
		log(`bundle built: ${bundle} (${(fs.statSync(bundle).size / 1e6).toFixed(2)} MB)`);

		const st = cli(['start'], 200000);
		const doc = cli(['status'], 60000);
		wjson('doctor.json', { startStatus: st.status, doctor: doc.json?.value ?? null });
		log(`client: bridgeVersion=${doc.json?.value?.bridgeVersion} connected=${doc.json?.value?.connected}`);

		try {
			const open = cli(['open', '--path', path.join(boardDir, eprj), '--headless', 'true'], 150000);
			SID = open.json?.value?.sessionId;
			if (!SID) throw new Error('open failed: ' + (open.raw || open.stderr).slice(0, 400));
			log(`session ${SID} opened`);
			const load = inv({ cmd: 'load', bundlePath: bundle }, 150000);
			log(`bundle imported in client; exports ${load.exports}`);
			const probe = inv({ cmd: 'probe' });
			wjson('probe.json', probe);
			log('worker probe: ' + JSON.stringify(probe));

			await takeSnapshot('unrouted');
			if (STEPS.has('drc')) await takeDrc('unrouted');

			// ── route ──
			if (STEPS.has('route')) {
				log('route: TopologicalRouter.route(normalizeRouterOptions({})) ...');
				const t0 = Date.now();
				const r = await runJob('route', { options: {} }, { maxWaitMs: ROUTE_TIMEOUT });
				const wall = Date.now() - t0;
				wjson('route.json', { status: r.status, error: r.error, stack: r.stack, wallMsDriver: wall, result: r.result, options: r.meta?.options, logs: r.logs, progressCount: r.progressCount, progressSample: r.progress.filter((p, i) => i % 25 === 0 || i === r.progress.length - 1) });
				if (r.status !== 'done') {
					log(`ROUTE FAILED: ${r.error}\n${r.stack ?? ''}`);
					throw new Error('route() did not complete in the client: ' + r.status + ' ' + r.error);
				}
				log(`route done: wall ${(r.result.wallMs / 1000).toFixed(1)}s (driver ${(wall / 1000).toFixed(1)}s); summary ${JSON.stringify(r.result.summary)}; Worker constructions during route: ${r.result.workerConstructed}`);
				for (const l of r.logs.slice(-12)) log(`   route log[${l.level}] ${l.message}`.slice(0, 260));
				await takeSnapshot('routed');
				if (STEPS.has('drc')) await takeDrc('routed');
			}
			// ── fault injection: a tighten job whose N-th canvas create call throws (aborts at the first write failure, restores the run) ──
			if (STEPS.has('fault')) {
				log(`fault: tightenExistingRouting() with createFailAt=${FAULT_AT}${NO_WORKER ? ' [no-worker fallback]' : ''} ...`);
				const t0 = Date.now();
				const r = await runJob('tighten', { noWorker: NO_WORKER, faultAt: FAULT_AT }, { maxWaitMs: TIGHTEN_TIMEOUT, showLogs: true });
				const wall = Date.now() - t0;
				wjson('fault.json', { status: r.status, error: r.error, stack: r.stack, wallMsDriver: wall, faultAt: FAULT_AT, result: r.result, logs: r.logs, progressCount: r.progressCount, progress: r.progress });
				if (r.status === 'done') {
					const res = r.result.result;
					log(`fault done: aborted=${res.aborted} changed(adopted)=${res.changed} failed=${res.failed} verified=${res.verified} issues=${res.issues?.length ?? 0} savedMil=${r2(res.savedMil)} total=${res.total}`);
					log(`fault final message: ${JSON.stringify(res.message)}`);
					for (const l of r.logs) if (l.level === 'warn' || l.level === 'error') log(`  fault log[${l.level}] ${l.message}`);
					printIssues('fault', res.issues);
				} else fail(`fault job status ${r.status}: ${r.error}`);
				await takeSnapshot('fault');
				if (STEPS.has('drc')) await takeDrc('fault');
			}
			// ── tighten ──
			for (const [step, snapLabel, file] of [['tighten', 'tight1', 'tighten1.json'], ['tighten2', 'tight2', 'tighten2.json']]) {
				if (!STEPS.has(step)) continue;
				log(`${step}: TopologicalRouter.tightenExistingRouting()${NO_WORKER ? ' [no-worker fallback]' : ''} ...`);
				const t0 = Date.now();
				const r = await runJob('tighten', { noWorker: NO_WORKER }, { maxWaitMs: TIGHTEN_TIMEOUT, showLogs: true });
				const wall = Date.now() - t0;
				wjson(file, { status: r.status, error: r.error, stack: r.stack, wallMsDriver: wall, result: r.result, logs: r.logs, progressCount: r.progressCount, progress: r.progress });
				if (r.status !== 'done') {
					log(`TIGHTEN (${step}) FAILED: ${r.error}\n${r.stack ?? ''}`);
					fail(`tighten (${step}) ${r.status}: ${r.error}`);
					await takeSnapshot(snapLabel).catch(() => {});
					if (STEPS.has('drc')) await takeDrc(snapLabel).catch(() => {});
					break;
				}
				log(`${step} done: wall ${(r.result.wallMs / 1000).toFixed(1)}s; result ${JSON.stringify({ ...r.result.result, issues: r.result.result.issues?.length ?? 0 })}; Worker constructions: ${r.result.workerConstructed}`);
					printIssues(step, r.result.result.issues);
				await takeSnapshot(snapLabel);
				if (STEPS.has('drc')) await takeDrc(snapLabel);
			}
		} catch (e) {
			// route failure, snapshot failure, client/CLI error ...: remember it, still close the session and still write the metrics of what exists
			fail(String(e?.stack ?? e));
		} finally {
			if (SID && !KEEP) {
				try { cli(['session', 'close', '--session', SID, '--destroy'], 60000); log(`session ${SID} closed (destroyed)`); }
				catch (e) { log('session close failed: ' + e.message); }
			} else if (SID) log(`--keep-session: session ${SID} left open`);
		}
	}

	// ── analysis ──
	const order = [['routed', 'routed'], ['fault', 'fault'], ['tight1', 'tight1'], ['tight2', 'tight2']].filter(([, l]) => has(`snapshot-${l}.json`));
	if (!order.length) { log('no stage snapshots in run dir - nothing to analyse'); if (!REUSE) FAILURES.push('no stage snapshots produced'); return; }
	const tinfo = { fault: has('fault.json') ? rjson('fault.json') : null, tight1: has('tighten1.json') ? rjson('tighten1.json') : null, tight2: has('tighten2.json') ? rjson('tighten2.json') : null };
	const stages = [], snaps = {};
	let prev = null, prevLabel = null, prevDrc = null;
	for (const [, label] of order) {
		const snap = rjson(`snapshot-${label}.json`);
		snaps[label] = snap;
		const drcRes = has(`drc-${label}.json`) ? rjson(`drc-${label}.json`) : null;
		const ti = tinfo[label]?.result?.result ? { ...tinfo[label].result.result, issues: tinfo[label].result.result.issues ?? [], wallMs: tinfo[label].result.wallMs, workerConstructed: tinfo[label].result.workerConstructed, noWorker: tinfo[label].result.noWorker, status: tinfo[label].status } : (tinfo[label] ? { status: tinfo[label].status, error: tinfo[label].error } : null);
		log(`analysing ${label} ...`);
		const t0 = Date.now();
		const { m, state } = analyzeStage(label, snap, prev, prevLabel, drcRes, prevDrc, ti);
		log(`  ${label} analysed in ${((Date.now() - t0) / 1000).toFixed(1)}s`);
		stages.push(m);
		prev = state; prevLabel = label; prevDrc = drcRes;
	}
	const unroutedDrc = has('drc-unrouted.json') ? edaDrcSummary(rjson('drc-unrouted.json')) : null;
	const out = { runDir: RUN_DIR, case: CASE, steps: [...STEPS], noWorker: NO_WORKER, probe: has('probe.json') ? rjson('probe.json') : null, doctor: has('doctor.json') ? rjson('doctor.json') : null, unroutedEdaDrc: unroutedDrc, stages };
	// fault stage: does the canvas equal "routed + the runs adopted before the fault"?
	if (snaps.fault && snaps.routed && has('fault.json')) {
		const fj = rjson('fault.json');
		const prog = (fj.progress ?? []).filter(p => p.stage === '逐段拉紧' && typeof p.changed === 'number');
		const adoptedNets = [];
		let lastChanged = 0;
		for (const p of prog) {
			if (p.changed > lastChanged) { adoptedNets.push(String(p.message).split(' ')[0]); lastChanged = p.changed; }
		}
		const A = buildCtx(snaps.routed).ctx, Bc = buildCtx(snaps.fault).ctx;
		const sg = M.sameGeometry(A.nets, Bc.nets, 0.01);
		const sg1 = M.sameGeometry(A.nets, Bc.nets, 0.3);
		const diffNets = sg.diffs.map(d => d.net).sort();
		const adoptedSet = [...new Set(adoptedNets)].sort();
		const stage = stages.find(x => x.label === 'fault');
		out.faultCheck = {
			faultAt: fj.faultAt, progressChangedAtEnd: lastChanged, adoptedNetsFromProgress: adoptedSet,
			netsDifferingFromRouted_tol0_01: diffNets, netsDifferingFromRouted_tol0_3: sg1.diffs.map(d => d.net).sort(),
			differingNotAdopted: diffNets.filter(n => !adoptedSet.includes(n)),
			adoptedButIdentical: adoptedSet.filter(n => !diffNets.includes(n)),
			primCountDiffsOnNonAdoptedNets: sg.diffs.filter(d => !adoptedSet.includes(d.net)),
			duplicates: stage?.duplicates, connectivityDiffVsRouted: stage?.vsPrev?.connectivityDiff, edaDrcNewVsRouted: stage?.vsPrev?.edaDrcNew,
			strictNewVsRouted: stage?.vsPrev?.newStrictViolations, lengthDeltaMil: stage?.vsPrev?.lengthDeltaMil, primDelta: stage?.vsPrev?.primDelta,
		};
		log('fault check: ' + JSON.stringify(out.faultCheck));
	}
	// route/tighten raw facts for the report
	if (has('route.json')) { const r = rjson('route.json'); out.route = { status: r.status, error: r.error, wallMs: r.result?.wallMs, summary: r.result?.summary, workerConstructed: r.result?.workerConstructed, logTail: (r.logs ?? []).slice(-60), logTotal: (r.logs ?? []).length, progressCount: r.progressCount, workerLogLine: (r.logs ?? []).find(l => /Web Worker|不支持 Web Worker/.test(l.message))?.message }; }
	for (const k of ['fault', 'tight1', 'tight2']) {
		const f = k === 'fault' ? 'fault.json' : k === 'tight1' ? 'tighten1.json' : 'tighten2.json';
		if (has(f)) { const t = rjson(f); out[k] = { status: t.status, error: t.error, wallMs: t.result?.wallMs, result: t.result?.result, workerConstructed: t.result?.workerConstructed, logs: t.logs, progressCount: t.progressCount, progress: t.progress }; }
	}

	// zoom regions + renders
	if (!NO_RENDER && snaps.routed && snaps.tight1) {
		const newPts = (stages.find(x => x.label === 'tight1')?.vsPrev?.newStrictViolations.first10 ?? []).map(v => [v.x, v.y]);
		const zoom = pickZoomBoxes(snaps.routed, snaps.tight1, newPts);
		out.zoom = zoom;
		if (DEPOT) fs.mkdirSync(DEPOT, { recursive: true });
		fs.mkdirSync(path.join(RUN_DIR, 'svg'), { recursive: true });
		const write = (name, svg) => { if (DEPOT) fs.writeFileSync(path.join(DEPOT, name), svg, 'utf8'); fs.writeFileSync(path.join(RUN_DIR, 'svg', name), svg, 'utf8'); };
		const marks = newPts;
		write('ASK-003-clock-before.svg', renderSnapshot(snaps.routed, { title: 'ASK-003 clock-pcb3 BEFORE tighten (routed, canvas)' }));
		write('ASK-003-clock-after.svg', renderSnapshot(snaps.tight1, { title: 'ASK-003 clock-pcb3 AFTER tighten (tight1, canvas)', marks }));
		zoom.boxes.forEach((b, i) => {
			write(`ASK-003-clock-before-zoom${i + 1}.svg`, renderSnapshot(snaps.routed, { box: b.box, widthPx: 1200, title: `BEFORE zoom${i + 1} box ${b.box.join(',')}` }));
			write(`ASK-003-clock-after-zoom${i + 1}.svg`, renderSnapshot(snaps.tight1, { box: b.box, widthPx: 1200, title: `AFTER zoom${i + 1} box ${b.box.join(',')}`, marks }));
		});
		log(`SVGs written to ${path.join(RUN_DIR, 'svg')}${DEPOT ? ' and ' + DEPOT : ' (no --depot given: run dir only)'}: whole-board before/after + ${zoom.boxes.length} zoom pairs; boxes ${JSON.stringify(zoom.boxes.map(b => b.box))}; basis: ${zoom.basis}`);
	}

	wjson('metrics.json', out);
	printTable(stages);
	log(`metrics.json written to ${path.join(RUN_DIR, 'metrics.json')}`);
}

process.on('SIGINT', () => {
	if (SID && !KEEP) { try { cli(['session', 'close', '--session', SID, '--destroy'], 60000); } catch { /* ignore */ } }
	process.exit(130);
});
main()
	.catch(e => { console.error(stamp(), 'FATAL', e?.stack ?? e); process.exitCode = 1; })
	.finally(() => {
		if (FAILURES.length) {
			console.error(stamp(), `${FAILURES.length} step failure(s):`);
			for (const f of FAILURES) console.error('  - ' + f.split('\n')[0].slice(0, 300));
			process.exitCode = 1;
		}
	});
