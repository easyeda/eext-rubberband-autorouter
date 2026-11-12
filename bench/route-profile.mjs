#!/usr/bin/env node
/**
 * route-profile.mjs — V8 CPU profile of the router, summarised by function name.
 *
 *   node bench/route-profile.mjs [bench-offline args...] [--callers name]... [--top-self N] [--top-incl N]
 *   node bench/route-profile.mjs --dump tmp/dumps/<name>-dump.json [route-dump args...] [--callers name]...
 *   node bench/route-profile.mjs --analyze tmp/prof-<timestamp> [--callers name]...   (no run, just re-read)
 *
 *   Without --dump the extra args go to `scripts/bench-offline.mjs`. Defaults are injected when absent:
 *   --phases a --runs 1 --budget 1800000 (so `node bench/route-profile.mjs --nets 12` profiles one natural run).
 *   With --dump they go to `bench/route-dump.mjs <dump>` (e.g. --budget 120000 --ripup 3).
 *
 *   The child runs as `node --cpu-prof --cpu-prof-dir=tmp/prof-<timestamp>`; the .cpuprofile files stay
 *   there (tmp/ is git-ignored; open them in Chrome DevTools / speedscope for line detail).
 *
 *   Output (stdout): top-25 SELF time (time spent in the function body itself) and top-40 INCLUSIVE time
 *   (function + everything it called; recursion counted once per sample), aggregated by functionName
 *   (anonymous functions appear as "(anonymous)@<line>" = line in the esbuild bundle; a function name
 *   that exists in several files/classes is merged). --callers NAME (repeatable) prints, for NAME, its
 *   callers ranked by the inclusive time NAME spent under each caller (outermost recursive frames only).
 *   Percentages are of total sampled time (profile includes bundling/startup/audits, not only plan()).
 */
import { spawnSync } from 'child_process';
import * as fs from 'fs';
import * as path from 'path';
import { fileURLToPath } from 'url';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const argv = process.argv.slice(2);
if (argv.includes('--help') || argv.includes('-h')) {
	const src = fs.readFileSync(fileURLToPath(import.meta.url), 'utf8');
	process.stdout.write(src.slice(src.indexOf('/**'), src.indexOf('*/') + 2) + '\n');
	process.exit(0);
}

// ── split own flags from pass-through args ──
const own = { callers: [], topSelf: 25, topIncl: 40, dump: null, analyze: null };
const pass = [];
for (let i = 0; i < argv.length; i++) {
	const a = argv[i];
	if (a === '--callers') own.callers.push(argv[++i]);
	else if (a === '--top-self') own.topSelf = Number(argv[++i]);
	else if (a === '--top-incl') own.topIncl = Number(argv[++i]);
	else if (a === '--dump') own.dump = argv[++i];
	else if (a === '--analyze') own.analyze = argv[++i];
	else pass.push(a);
}

let profDir;
if (own.analyze) profDir = path.resolve(own.analyze);
else {
	const stamp = new Date().toISOString().replace(/[-:]/g, '').replace(/\..*/, '').replace('T', '-');
	profDir = path.join(ROOT, 'tmp', `prof-${stamp}`);
	fs.mkdirSync(profDir, { recursive: true });
	let script, args;
	if (own.dump) {
		script = path.join(ROOT, 'bench', 'route-dump.mjs');
		args = [path.resolve(own.dump), ...pass];
	} else {
		script = path.join(ROOT, 'scripts', 'bench-offline.mjs');
		args = [...pass];
		if (!args.includes('--phases')) args.push('--phases', 'a');
		if (!args.includes('--runs')) args.push('--runs', '1');
		if (!args.includes('--budget')) args.push('--budget', '1800000');
	}
	console.error(`profiling: node --cpu-prof ${path.relative(ROOT, script)} ${args.join(' ')}\n  -> ${path.relative(ROOT, profDir)}/`);
	const t0 = Date.now();
	const r = spawnSync(process.execPath, ['--cpu-prof', `--cpu-prof-dir=${profDir}`, script, ...args], {
		cwd: ROOT, stdio: ['ignore', 'ignore', 'inherit'], maxBuffer: 1 << 30,
	});
	console.error(`child exit ${r.status} after ${((Date.now() - t0) / 1000).toFixed(1)}s`);
	if (r.status !== 0) console.error('WARNING: child exited non-zero; analysing whatever profile was written');
}

const files = fs.existsSync(profDir) ? fs.readdirSync(profDir).filter(f => f.endsWith('.cpuprofile')) : [];
if (!files.length) { console.error(`no .cpuprofile in ${profDir}`); process.exit(1); }

// ── aggregate ──
const selfBy = new Map();     // name -> us
const inclBy = new Map();     // name -> us
const callerBy = new Map();   // callee name -> Map(caller name -> us)
let totalUs = 0;
const wantCallers = new Set(own.callers);

for (const f of files) {
	const prof = JSON.parse(fs.readFileSync(path.join(profDir, f), 'utf8'));
	const nodes = new Map(prof.nodes.map(n => [n.id, n]));
	const parent = new Map();
	for (const n of prof.nodes) for (const c of n.children ?? []) parent.set(c, n.id);
	const nameOf = (n) => {
		const fn = n.callFrame.functionName;
		return fn ? fn : `(anonymous)@${n.callFrame.lineNumber + 1}`;
	};
	const names = new Map(prof.nodes.map(n => [n.id, nameOf(n)]));
	// self time per node: sample i lasts until sample i+1
	const selfUs = new Map();
	const { samples, timeDeltas } = prof;
	for (let i = 0; i < samples.length; i++) {
		const dt = i + 1 < samples.length ? timeDeltas[i + 1] : (timeDeltas[i] ?? 0);
		selfUs.set(samples[i], (selfUs.get(samples[i]) ?? 0) + Math.max(0, dt));
		totalUs += Math.max(0, dt);
	}
	for (const [id, us] of selfUs) {
		const nm = names.get(id);
		if (nm === '(idle)') continue;
		selfBy.set(nm, (selfBy.get(nm) ?? 0) + us);
		// inclusive: each distinct name on the stack once
		const seen = new Set();
		for (let cur = id; cur !== undefined; cur = parent.get(cur)) {
			const n2 = names.get(cur);
			if (seen.has(n2)) continue;
			seen.add(n2);
			inclBy.set(n2, (inclBy.get(n2) ?? 0) + us);
		}
		// callers of the wanted functions: attribute this sample's self time to each outermost frame of X
		if (wantCallers.size) {
			const chain = [];
			for (let cur = id; cur !== undefined; cur = parent.get(cur)) chain.push(cur);   // leaf .. root
			for (const w of wantCallers) {
				// outermost frame of w in the stack = the last occurrence while walking leaf->root
				let outer = -1;
				for (let k = 0; k < chain.length; k++) if (names.get(chain[k]) === w) outer = k;
				if (outer < 0) continue;
				const callerId = chain[outer + 1];
				const cn = callerId === undefined ? '(root)' : names.get(callerId);
				let m = callerBy.get(w); if (!m) callerBy.set(w, m = new Map());
				m.set(cn, (m.get(cn) ?? 0) + us);
			}
		}
	}
	void nodes;
}

const fmtMs = (us) => (us / 1000).toFixed(0).padStart(8) + ' ms';
const pct = (us) => (100 * us / totalUs).toFixed(1).padStart(5) + '%';
const top = (m, n) => [...m].sort((a, b) => b[1] - a[1]).slice(0, n);

console.log(`profile: ${path.relative(ROOT, profDir)}  files=${files.length}  sampled=${(totalUs / 1e6).toFixed(1)}s`);
console.log(`\n== top ${own.topSelf} by SELF time ==`);
for (const [n, us] of top(selfBy, own.topSelf)) console.log(`${fmtMs(us)} ${pct(us)}  ${n}`);
console.log(`\n== top ${own.topIncl} by INCLUSIVE time ==`);
for (const [n, us] of top(inclBy, own.topIncl)) console.log(`${fmtMs(us)} ${pct(us)}  ${n}`);
for (const w of own.callers) {
	const m = callerBy.get(w);
	console.log(`\n== callers of ${w} (time spent in ${w} and below, by caller) ==`);
	if (!m) { console.log('  (function not found in profile — names are case-sensitive; see the lists above)'); continue; }
	const tot = [...m.values()].reduce((a, b) => a + b, 0);
	console.log(`  total ${(tot / 1000).toFixed(0)} ms`);
	for (const [n, us] of top(m, 15)) console.log(`${fmtMs(us)} ${pct(us)}  ${n}`);
}
