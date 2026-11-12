#!/usr/bin/env node
/**
 * tighten-bench.mjs - offline benchmark of the "tighten existing routing" feature with STRICT metrics.
 *
 *   node bench/tighten-bench.mjs [--root DIR] [--world clean|prod] [--boards routed,bad,wide]
 *                                [--seed 1] [--nets 80] [--out file.json] [--route-root DIR]
 *                                [--budget 60000] [--timeout 300000] [--verbose]
 *
 *   --root DIR        source root (containing src/) whose src/worker/planning-worker.ts is bundled and driven
 *                     (default: this repo). Pass a pristine pre-fix snapshot to get baseline numbers.
 *   --world MODE      clean (default): baseWorld = static copper only (new contract)
 *                     prod: baseWorld additionally holds a copy of every existing track (old main thread)
 *   --boards LIST     routed  = synthetic routing board (seed/nets) routed by PlanningEngine.plan, then tightened.
 *                               The routed result is cached as JSON in <os tmpdir>/tighten-bench-cache so that
 *                               before/after comparisons tighten the SAME input. A missing cache is produced with
 *                               --route-root (default: env TIGHTEN_ROUTE_ROOT, else --root).
 *                     bad     = hand-built bad traces (90deg staircases, 45deg zig-zags, U detour, T net, via net, ...)
 *                     wide    = bad board with mixed ACTUAL widths 6/10/15/20/30 mil (clean with actual widths);
 *                               `width` is passed per run
 *   --out FILE        also write the JSON document there. Stdout always gets the JSON document, stderr the table.
 *
 * Per board: metrics BEFORE, tighten (real worker entry), apply messages like the main thread does, metrics AFTER,
 * then a SECOND tighten on the output (idempotence = the pass-2 prim sets equal pass-1's within 0.01 mil per net
 * after canonical matching, plus no new free sharp corner). Metrics come from bench/tighten-metrics.mjs (strict:
 * actual widths, real pad shapes incl. POLYGON, dense sampling). A violation is NEW per LOCATION and foreign item:
 * not excused by a before-violation of the same net/item within max(20 mil, 2w) that is at least as bad.
 * Exit code is non-zero (and a BENCH FAIL line is printed) when a pass times out or the worker never sends tightenDone.
 * sharp>=30 = degree-2 joints with turn >= 30 deg (free = not on an own-net pad centre / via).
 */
import * as fs from 'fs';
import * as os from 'os';
import * as path from 'path';
import * as crypto from 'crypto';
import { fileURLToPath } from 'url';
import * as M from './tighten-metrics.mjs';
import * as B from './tighten-boards.mjs';

const HERE = path.dirname(fileURLToPath(import.meta.url));
const REPO = path.resolve(HERE, '..');
const log = (...a) => console.error(...a);
console.log = (...a) => console.error(...a);
if (!process.argv.includes('--verbose')) { console.warn = () => {}; console.info = () => {}; console.debug = () => {}; }

const argv = process.argv.slice(2);
const argOf = (k, d) => { const i = argv.indexOf(k); return i >= 0 && i + 1 < argv.length ? argv[i + 1] : d; };
const opts = {
	root: path.resolve(argOf('--root', REPO)),
	routeRoot: argOf('--route-root', process.env.TIGHTEN_ROUTE_ROOT || null),
	world: String(argOf('--world', 'clean')),
	boards: String(argOf('--boards', 'routed,bad,wide')).split(',').map(s => s.trim()).filter(Boolean),
	seed: Number(argOf('--seed', 1)),
	nets: Math.max(2, Math.round(Number(argOf('--nets', 80)))),
	budgetMs: Number(argOf('--budget', 60000)),
	timeoutMs: Number(argOf('--timeout', 300000)),
	out: argOf('--out', null),
	verbose: argv.includes('--verbose'),
};
if (argv.includes('--help') || argv.includes('-h')) {
	const src = fs.readFileSync(fileURLToPath(import.meta.url), 'utf8');
	process.stdout.write(src.slice(src.indexOf('/**'), src.indexOf('*/') + 2) + '\n');
	process.exit(0);
}
if (!['clean', 'prod'].includes(opts.world)) { log('bad --world (clean|prod)'); process.exit(1); }
for (const b of opts.boards) if (!['routed', 'bad', 'wide'].includes(b)) { log(`bad board ${b}`); process.exit(1); }

const failures = [];   // timeout / missing tightenDone (make the bench exit non-zero)
const r1 = (v) => Math.round(v * 10) / 10;
const sha1 = (s) => crypto.createHash('sha1').update(s).digest('hex').slice(0, 12);

// ─────────────────────────────────────────────────────────────
// measuring one state
// ─────────────────────────────────────────────────────────────
function measure(mod, board, nets, vias) {
	const ctx = B.metricsCtx(mod, board, nets, vias);
	const c = M.counts(ctx);
	const s = M.sharpCorners(ctx, { minDeg: 30 });
	const d = M.strictDrc(ctx);
	return {
		ctx, lines: c.lines, arcs: c.arcs, lengthMil: c.lengthMil,
		sharp: s, drc: d, conn: M.connectivity(ctx), dangling: M.danglingEnds(ctx),
	};
}
const sharpSummary = (s) => ({ total: s.count, free: s.free, fixed: s.fixed });

async function evaluate(label, board, nets, vias, code, mod) {
	log(`\n[${label}] ${nets.length} nets, ${nets.reduce((a, n) => a + n.prims.length, 0)} prims, world=${opts.world}`);
	const before = measure(mod, board, nets, vias);
	log(`  before: len ${r1(before.lengthMil)}, sharp30 ${before.sharp.count} (free ${before.sharp.free}), arcs ${before.arcs}, strictDRC ${before.drc.violations.length}`);
	const p1 = await B.tightenPass({ code, board, nets, vias, world: opts.world, timeoutMs: opts.timeoutMs });
	if (p1.error) throw new Error(`worker error: ${p1.error}`);
	if (p1.failure) { failures.push(`[${label}] pass 1: ${p1.failure}`); log(`  FAIL: ${p1.failure}`); }
	const after = measure(mod, board, p1.nets, vias);
	const newV = M.newViolations(before.drc, after.drc);
	const connDiff = M.connectivityDiff(before.conn, after.conn);
	const widths = M.widthsPreserved(M.toNets(nets, (n, p) => p.width), M.toNets(p1.nets, (n, p) => p.width));
	log(`  pass1 ${p1.wallMs}ms: reported ${p1.runsReported}/${p1.runs.length} runs, changed ${p1.runsChanged}, len ${r1(after.lengthMil)}`);
	const p2 = await B.tightenPass({ code, board, nets: p1.nets, vias, world: opts.world, timeoutMs: opts.timeoutMs });
	if (p2.error) throw new Error(`worker error (pass 2): ${p2.error}`);
	if (p2.failure) { failures.push(`[${label}] pass 2: ${p2.failure}`); log(`  FAIL: ${p2.failure}`); }
	const after2 = measure(mod, board, p2.nets, vias);
	const new2 = M.newViolations(after.drc, after2.drc);
	const geo2 = M.sameGeometry(p1.nets, p2.nets, 0.01);
	// sharp corners of pass 2 that were not already there (position keyed)
	const key = (c) => `${c.net}|${c.layer}|${Math.round(c.x)}|${Math.round(c.y)}`;
	const had = new Set(after.sharp.list.map(key));
	const newSharp2 = after2.sharp.list.filter(c => !c.atFixed && !had.has(key(c))).length;
	const dangleWorse = after.dangling.count > before.dangling.count;
	const rep = {
		board: label,
		runs: p1.runs.length, nets: nets.length, uncoveredPrims: p1.uncovered,
		adopted: p1.runsReported, adoptedByWorker: p1.done?.adopted ?? null, runsChanged: p1.runsChanged, rewrittenNets: p1.rewrittenNets.length,
		tightenDone: p1.done ? { total: p1.done.total, adopted: p1.done.adopted, rounded: p1.done.rounded, solved: p1.done.solved, rewritten: p1.done.rewritten, saved: r1(p1.done.saved) } : null,
		length: { before: r1(before.lengthMil), after: r1(after.lengthMil), delta: r1(after.lengthMil - before.lengthMil), longerThanBefore: after.lengthMil > before.lengthMil + 0.5 },
		sharp30: { before: sharpSummary(before.sharp), after: sharpSummary(after.sharp) },
		arcs: { before: before.arcs, after: after.arcs },
		lines: { before: before.lines, after: after.lines },
		strictDrc: {
			before: before.drc.violations.length, after: after.drc.violations.length, new: newV.length,
			worstNew: newV.slice(0, 6).map(v => ({ netA: v.netA, netB: v.netB, item: v.item, kind: v.kind, gap: r1(v.gap), beforeGap: v.beforeGap === null ? null : r1(v.beforeGap), rule: v.rule, x: Math.round(v.x), y: Math.round(v.y) })),
			pairsAfter: after.drc.pairs.slice(0, 6),   // extra: per net-pair summary (not the acceptance number)
		},
		connectivity: { changedNets: connDiff.length, list: connDiff.slice(0, 6).map(x => ({ net: x.net, kind: x.kind })) },
		danglingEnds: { before: before.dangling.count, after: after.dangling.count, worse: dangleWorse },
		widthsPreserved: widths.ok, widthBad: widths.bad.slice(0, 5),
		wallMs: p1.wallMs,
		failures: [p1.failure, p2.failure].filter(Boolean),
		pass2: {
			wallMs: p2.wallMs, runsChanged: p2.runsChanged, deltaLengthMil: r1(after2.lengthMil - after.lengthMil),
			newFreeSharp: newSharp2, freeSharpAfter: after2.sharp.free, newStrictDrc: new2.length,
			// GEOMETRY: every net's prim set (direction / order independent) equal within 0.01 mil, no new free sharp corner
			geometryEqual: geo2.equal, geometryDiffs: geo2.diffs.slice(0, 5),
			idempotent: geo2.equal && newSharp2 === 0,
		},
	};
	log(`  after : len ${rep.length.after}, sharp30 ${after.sharp.count} (free ${after.sharp.free}), arcs ${after.arcs}, strictDRC ${rep.strictDrc.after} (new ${rep.strictDrc.new}), connChanged ${rep.connectivity.changedNets}, widthsOK ${rep.widthsPreserved}`);
	return rep;
}

// ─────────────────────────────────────────────────────────────
// boards
// ─────────────────────────────────────────────────────────────
async function routedInput(codeMain, mod) {
	const board = B.genRoutingBoard({ seed: opts.seed, nets: opts.nets, layers: 2 });
	const boardHash = sha1(JSON.stringify([board.allPads.map(p => [p.net, p.x, p.y, p.layer]), board.statics]));
	const dir = path.join(os.tmpdir(), 'tighten-bench-cache');
	const file = path.join(dir, `routed-s${opts.seed}-n${opts.nets}-l2.json`);
	let cached = null;
	try { cached = JSON.parse(fs.readFileSync(file, 'utf8')); } catch { /* none */ }
	if (cached && cached.boardHash === boardHash) {
		log(`[routed] using cached routing ${file} (routed with ${cached.routedWith})`);
		return { board, nets: cached.nets, vias: cached.vias, cache: file, routedWith: cached.routedWith };
	}
	const routeRoot = path.resolve(opts.routeRoot || opts.root);
	log(`[routed] routing ${board.networks.length} nets with ${routeRoot} (budget ${opts.budgetMs}ms) ...`);
	const code = routeRoot === opts.root ? codeMain : await B.buildBundle(routeRoot);
	void mod;
	const t0 = Date.now();
	const res = await B.routeBoard(code, board, { budgetMs: opts.budgetMs });
	log(`[routed] done in ${Date.now() - t0}ms, ${res.nets.length} routed nets, ${res.vias.length} vias`);
	fs.mkdirSync(dir, { recursive: true });
	fs.writeFileSync(file, JSON.stringify({ boardHash, routedWith: routeRoot, nets: res.nets, vias: res.vias }));
	return { board, nets: res.nets, vias: res.vias, cache: file, routedWith: routeRoot };
}
function badInput() {
	const board = B.genBadBoard({ seed: opts.seed });
	const nets = board.traces.map(t => ({ name: t.name, prims: t.prims.map(p => ({ ...p, width: board.drc.traceWidthMil })) }));
	return { board, nets, vias: board.vias };
}
function wideInput(mod) {
	const board = B.genBadBoard({ seed: opts.seed });
	const res = B.assignWideWidths(board, mod, M.strictDrc, board.vias);
	if (!res.clean) log('WARNING: wide board still has strict-DRC violations at the narrowest width');
	const nets = board.traces.map(t => ({ name: t.name, prims: t.prims.map(p => ({ ...p })) }));
	const dist = {};
	for (const n of nets) for (const p of n.prims) dist[p.width] = (dist[p.width] ?? 0) + 1;
	return { board, nets, vias: board.vias, widthDistribution: dist, cleanBefore: res.clean };
}

// ─────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────
const doc = { meta: { script: 'bench/tighten-bench.mjs', node: process.version, root: opts.root, world: opts.world, seed: opts.seed, nets: opts.nets, units: 'mil / ms' }, boards: {} };
let exitCode = 0;
try {
	globalThis.__RB_VERBOSE__ = opts.verbose;
	const code = await B.buildBundle(opts.root);
	const { mod } = await B.loadFresh(code);
	for (const name of opts.boards) {
		let inp;
		if (name === 'routed') inp = await routedInput(code, mod);
		else if (name === 'bad') inp = badInput();
		else inp = wideInput(mod);
		const rep = await evaluate(name, inp.board, inp.nets, inp.vias, code, mod);
		if (inp.widthDistribution) rep.widthDistribution = inp.widthDistribution;
		if (inp.routedWith) rep.routedWith = inp.routedWith;
		if (name !== 'routed' && rep.strictDrc.before) log(`  WARNING: ${name} board is not strict-DRC clean before tightening (${rep.strictDrc.before})`);
		doc.boards[name] = rep;
	}
} catch (e) {
	exitCode = 1;
	doc.error = String(e?.stack ?? e);
	log('ERROR:', doc.error);
}

// compact table
const rows = Object.values(doc.boards);
if (rows.length) {
	const cols = [
		['board', r => r.board], ['runs', r => r.runs], ['adopted', r => r.adopted], ['len before', r => r.length.before], ['len after', r => r.length.after],
		['sharp30 b(free)', r => `${r.sharp30.before.total}(${r.sharp30.before.free})`], ['sharp30 a(free)', r => `${r.sharp30.after.total}(${r.sharp30.after.free})`],
		['arcs b/a', r => `${r.arcs.before}/${r.arcs.after}`], ['DRC b/a/new', r => `${r.strictDrc.before}/${r.strictDrc.after}/${r.strictDrc.new}`],
		['conn chg', r => r.connectivity.changedNets], ['widths ok', r => r.widthsPreserved], ['ms', r => r.wallMs],
		['p2 dLen', r => r.pass2.deltaLengthMil], ['p2 chg', r => r.pass2.runsChanged], ['p2 newSharp', r => r.pass2.newFreeSharp], ['idem', r => r.pass2.idempotent],
	];
	const table = [cols.map(c => c[0]), ...rows.map(r => cols.map(c => String(c[1](r))))];
	const w = cols.map((_, i) => Math.max(...table.map(t => t[i].length)));
	log(`\n== tighten bench  root=${opts.root}  world=${opts.world} ==`);
	for (const t of table) log(t.map((v, i) => v.padEnd(w[i])).join('  '));
}
if (failures.length) {
	exitCode = 1;
	doc.failures = failures;
	for (const f of failures) log(`BENCH FAIL: ${f}`);
}
const text = JSON.stringify(doc, null, 2);
if (opts.out) fs.writeFileSync(opts.out, text + '\n', 'utf8');
process.stdout.write(text + '\n');
process.exit(exitCode);
