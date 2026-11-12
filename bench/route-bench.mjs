#!/usr/bin/env node
/**
 * route-bench.mjs — determinism + speed suite for PlanningEngine.plan() (offline, synthetic boards).
 *
 *   node bench/route-bench.mjs [--suite quick|full] [--runs N] [--out file.json]
 *                              [--allow-hash-change] [--case nets=80,budget=60000[,seed=1,layers=2]]
 *                              [--min-routed N] [--verbose]
 *
 *   quick (default)  nets 20                (budget 1800000)   ~8 s per run
 *   full             nets 20 and nets 40    (budget 1800000)   ~8 s + ~25 s per run
 *   --case K=V,...   one free-form ad-hoc case instead of a suite (keys: nets budget seed layers).
 *                    No expected hash/routed count unless the same key set is in EXPECTED below.
 *   --runs N         repetitions per case (default 1). Every run is a separate node process
 *                    (spawns `scripts/bench-offline.mjs --phases a --runs 1`), so process-level
 *                    state cannot leak between runs. Reported wall ms = median over runs.
 *
 * Exit code 1 when: any run has drcEdge violations > 0, routed nets < baseline (or < --min-routed),
 * bench-offline fails, or a result hash differs from EXPECTED (or between runs of one case) unless
 * --allow-hash-change is given. A changed hash is always printed loudly.
 *
 * Hash note: with a big budget (engine ends by itself) the hash is deterministic. A case whose wall
 * time reaches ~95% of its budget was truncated by the wall clock; it is non-deterministic by nature
 * and reported as hash=n/a(budget-bound) instead of being compared.
 *
 * The engine has wall-clock-based internal limits, so heavy CPU load can change the geometry too
 * (a loaded run gave 6d7388718ea8 for nets=20); a warning is printed when the load average is high.
 *
 * When an intended engine change alters the routing, re-measure and update EXPECTED below
 * (hash, routed) in the same commit.
 */
import { spawn } from 'child_process';
import * as fs from 'fs';
import * as os from 'os';
import * as path from 'path';
import { fileURLToPath } from 'url';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const BENCH = path.join(ROOT, 'scripts', 'bench-offline.mjs');

// Expected results on the reference machine (seed 1, 2 layers). Key = caseKey().
const EXPECTED = {
	// since the negotiated-grid completion phase (ASK-002 r2): 17/20 -> 20/20 and 32/40 -> 40/40;
	// with __RB_NO_NGRID__ set the hashes return to 1a80913285bd / 2c6d360eea55
	'nets=20,budget=1800000,seed=1,layers=2': { hash: 'e39bf0c71155', routed: 20, total: 20, approxWallS: 8 },
	// cd50895fe56d (32/40) was an artifact of the original slow code tripping the 60 s rip-up stagnation timer;
	// with the timer out of play (frozen clock or __RB_STAG_MS__ large) the original code also gives 170b1bec7074
	// since commit 869c38b (MST cross-layer penalty + alternate endpoints) the hashes are 1a80913285bd / 2c6d360eea55; with __RB_NO_XPEN__ and __RB_NO_ALT_ENDPOINT__ set they return to d66b7e731782 / 170b1bec7074
	'nets=40,budget=1800000,seed=1,layers=2': { hash: 'c2dc6b0034a1', routed: 40, total: 40, approxWallS: 25 },
};

const argv = process.argv.slice(2);
const argOf = (k, d) => { const i = argv.indexOf(k); return i >= 0 && i + 1 < argv.length ? argv[i + 1] : d; };
const flag = (k) => argv.includes(k);
if (flag('--help') || flag('-h')) {
	const src = fs.readFileSync(fileURLToPath(import.meta.url), 'utf8');
	process.stdout.write(src.slice(src.indexOf('/**'), src.indexOf('*/') + 2) + '\n');
	process.exit(0);
}

const runs = Math.max(1, Math.round(Number(argOf('--runs', 1))));
const suite = argOf('--suite', 'quick');
const allowHashChange = flag('--allow-hash-change');
const verbose = flag('--verbose');
const outFile = argOf('--out', null);
const minRoutedArg = argOf('--min-routed', null);

const caseKey = (c) => `nets=${c.nets},budget=${c.budget},seed=${c.seed},layers=${c.layers}`;
function parseCase(str) {
	const c = { nets: 20, budget: 1800000, seed: 1, layers: 2 };
	for (const kv of str.split(',').filter(Boolean)) {
		const [k, v] = kv.split('=');
		if (!(k in c) || !Number.isFinite(Number(v))) { console.error(`bad --case item "${kv}" (keys: nets budget seed layers)`); process.exit(2); }
		c[k] = Number(v);
	}
	return c;
}

let cases;
if (argOf('--case', null)) cases = [parseCase(argOf('--case', ''))];
else if (suite === 'quick') cases = [parseCase('nets=20')];
else if (suite === 'full') cases = [parseCase('nets=20'), parseCase('nets=40')];
else { console.error(`bad --suite "${suite}" (quick|full)`); process.exit(2); }

const median = (a) => { const s = [...a].sort((x, y) => x - y); const m = s.length >> 1; return s.length % 2 ? s[m] : (s[m - 1] + s[m]) / 2; };

/** one bench-offline process, phase A, one run; resolves to {ok, run?, error?} */
function runOnce(c, tag) {
	return new Promise((resolve) => {
		fs.mkdirSync(path.join(ROOT, 'tmp'), { recursive: true });
		const tmpOut = path.join(ROOT, 'tmp', `route-bench-${process.pid}-${tag}.json`);
		const args = [BENCH, '--phases', 'a', '--runs', '1', '--nets', String(c.nets), '--budget', String(c.budget),
			'--seed', String(c.seed), '--layers', String(c.layers), '--out', tmpOut];
		if (verbose) args.push('--verbose');
		const p = spawn(process.execPath, args, { cwd: ROOT, stdio: ['ignore', 'pipe', verbose ? 'inherit' : 'pipe'] });
		let stderrTail = '';
		p.stdout.resume();
		p.stderr?.on('data', (d) => { stderrTail = (stderrTail + d).slice(-2000); });
		p.on('exit', (code) => {
			let doc = null;
			try { doc = JSON.parse(fs.readFileSync(tmpOut, 'utf8')); fs.unlinkSync(tmpOut); } catch { /* handled below */ }
			const run = doc?.phaseA?.runs?.[0];
			if (code !== 0 || !run) resolve({ ok: false, error: doc?.error ?? `bench-offline exit ${code}\n${stderrTail}` });
			else resolve({ ok: true, run });
		});
	});
}

const phaseStr = (arr) => (arr ?? []).map(s => { const m = s.match(/^(.*?):\s*([\d.]+)s/); return m ? `${m[1]} ${m[2]}s` : s; }).join(' | ');

let failed = false;
// The engine has wall-clock-based internal limits, so under CPU load the geometry (hash) can differ
// even with a huge budget (observed: nets=20 -> 6d7388718ea8 under load vs d66b7e731782 on an idle machine).
const load1 = os.loadavg()[0];
if (load1 > os.cpus().length * 0.5) console.log(`WARNING: load average ${load1.toFixed(1)} on ${os.cpus().length} cpus — hash=CHANGED may be load noise; re-run on an idle machine before trusting it`);
const report = { meta: { node: process.version, suite: argOf('--case', null) ? 'case' : suite, runs, date: new Date().toISOString() }, cases: [] };

for (const c of cases) {
	const key = caseKey(c);
	const exp = EXPECTED[key];
	const results = [];
	for (let i = 1; i <= runs; i++) {
		console.error(`[${key}] run ${i}/${runs} ...`);
		results.push(await runOnce(c, `${i}`));
	}
	const bad = results.find(r => !r.ok);
	if (bad) {
		console.log(`FAIL ${key}: ${bad.error}`);
		failed = true;
		report.cases.push({ case: c, key, error: bad.error });
		continue;
	}
	const rs = results.map(r => r.run);
	const wallMs = Math.round(median(rs.map(r => r.wallMs)));
	const r0 = rs[0];
	const hashes = [...new Set(rs.map(r => r.resultHash))];
	const budgetBound = wallMs >= c.budget * 0.95;
	const problems = [];

	let hashTag;
	if (budgetBound) hashTag = `hash=n/a(budget-bound,${hashes[0]})`;
	else if (hashes.length > 1) { hashTag = `hash=UNSTABLE(${hashes.join(',')})`; problems.push('hash differs between runs'); }
	else if (!exp) hashTag = `hash=n/a(no baseline,${hashes[0]})`;
	else if (hashes[0] === exp.hash) hashTag = `hash=OK(${hashes[0]})`;
	else { hashTag = `hash=CHANGED(${hashes[0]}, expected ${exp.hash})`; problems.push(`hash changed ${exp.hash} -> ${hashes[0]}`); }

	const edgeMax = Math.max(...rs.map(r => r.drcEdge.violationCount));
	const routedMin = Math.min(...rs.map(r => r.nets.routed));
	const minRouted = minRoutedArg !== null ? Number(minRoutedArg) : exp?.routed;
	let fatal = false;
	if (edgeMax > 0) { problems.push(`drcEdge violations ${edgeMax} > 0`); fatal = true; }
	if (minRouted !== undefined && routedMin < minRouted) { problems.push(`routed nets ${routedMin} < baseline ${minRouted}`); fatal = true; }
	const hashProblem = problems.some(p => p.startsWith('hash'));
	if (hashProblem && !allowHashChange) fatal = true;
	if (fatal) failed = true;

	console.log(
		`${fatal ? 'FAIL' : hashProblem ? 'WARN' : 'ok  '} ${key}  wall=${wallMs}ms${runs > 1 ? ` (runs ${rs.map(r => r.wallMs).join('/')})` : ''}` +
		`  nets=${r0.nets.routed}/${r0.nets.total}  pads=${r0.pads.routed}/${r0.pads.total}  vias=${r0.vias}  len=${Math.round(r0.lengthMil)}mil` +
		`  drc=${r0.drc.violationCount}/edge=${r0.drcEdge.violationCount}  ${hashTag}`);
	console.log(`     phases: ${phaseStr(r0.enginePhaseSeconds)}`);
	if (hashProblem) console.log(`     !!! RESULT CHANGED: ${problems.filter(p => p.startsWith('hash')).join('; ')}${allowHashChange ? ' (allowed by --allow-hash-change)' : ''}`);
	for (const p of problems.filter(p => !p.startsWith('hash'))) console.log(`     !!! ${p}`);
	report.cases.push({
		case: c, key, expected: exp ?? null, wallMs, wallMsRuns: rs.map(r => r.wallMs), hashes, budgetBound, problems,
		routedNets: r0.nets, pads: r0.pads, vias: r0.vias, lengthMil: r0.lengthMil, drc: r0.drc.violationCount, drcEdge: r0.drcEdge.violationCount,
		enginePhaseSeconds: r0.enginePhaseSeconds,
	});
}

report.ok = !failed;
if (outFile) fs.writeFileSync(outFile, JSON.stringify(report, null, 2) + '\n', 'utf8');
console.log(failed ? 'route-bench: FAILED' : 'route-bench: ok');
process.exit(failed ? 1 : 0);
