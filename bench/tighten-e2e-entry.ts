/**
 * tighten-e2e-entry.ts - esbuild entry (bundled to ESM by bench/tighten-e2e-run.mjs) that runs INSIDE the
 * real JLCEDA Pro client. It only wires the PRODUCTION classes (TopologicalRouter.route /
 * tightenExistingRouting, worker-source) to the canvas and exposes a tiny command surface
 * (`handle(args)`) that bench/tighten-e2e-eda.js calls through `invoke --code-file`.
 *
 * Nothing here changes routing logic. Test-only hooks:
 *   - a counting Worker subclass is installed around route()/tighten calls (counts constructions);
 *   - `noWorker` temporarily sets globalThis.Worker = undefined (forces the production main-thread fallback);
 *   - tighten job with `faultAt: n` sets globalThis.__RB_TIGHTEN_FAULT__ = { createFailAt: n } just before
 *     tightenExistingRouting() (n-th canvas create call throws) and ALWAYS deletes it afterwards (finally).
 *
 * Commands (args.cmd):
 *   probe                       worker environment facts (typeof Worker, isWorkerSupported, createPlanningWorker null/non-null)
 *   start {job, ...}            start a long job in the background (returns at once): route | tighten [noWorker, faultAt] | drc | snapshot
 *   status {job, logFrom, progFrom}   job state + new log/progress lines (+ result when done)
 *   chunk {token, i}            chunk i of a stored payload (snapshot result), 4 MB per chunk
 */
import { TopologicalRouter } from '../src/router/topological-router';
import { normalizeRouterOptions } from '../src/types/router-params';
import { createPlanningWorker, isWorkerSupported } from '../src/router/worker-source';
import { parsePolygonPath } from '../src/router/polygon-parse';
import { setup, ensurePcbTab, dumpInput } from '../scripts/eda-dump-driver';

export { setup, ensurePcbTab };

type Job = {
	name: string; status: 'running' | 'done' | 'error'; startedAt: number; endedAt?: number;
	logs: Array<{ t: number; level: string; message: string }>;
	progress: Array<Record<string, any>>;
	progressCount: number;
	result?: any; error?: string; stack?: string; meta: Record<string, any>;
};

const G = globalThis as any;
const JOBS: Record<string, Job> = (G.__RB_E2E_JOBS__ ??= {});
const STORE: Record<string, string[]> = (G.__RB_E2E_STORE__ ??= {});
const CHUNK = 4 * 1024 * 1024;

function eda(): any { return G.eda; }

function plain(o: any): any {
	const r: any = {};
	for (const k of Object.keys(o ?? {})) {
		const v = o[k];
		if (typeof v === 'function') continue;
		if (v && typeof v === 'object') { try { r[k] = JSON.parse(JSON.stringify(v)); } catch { /* skip */ } }
		else r[k] = v;
	}
	return r;
}

function store(payload: any): { token: string; chunkCount: number; totalChars: number } {
	const json = JSON.stringify(payload);
	const chunks: string[] = [];
	for (let i = 0; i < json.length; i += CHUNK) chunks.push(json.slice(i, i + CHUNK));
	if (!chunks.length) chunks.push('');
	const token = `e2e_${Date.now()}_${chunks.length}`;
	for (const k of Object.keys(STORE)) delete STORE[k];
	STORE[token] = chunks;
	return { token, chunkCount: chunks.length, totalChars: json.length };
}

// ── worker instrumentation ────────────────────────────────────────────
interface WorkerCounter { constructed: number; supported: boolean; typeofWorker: string }
async function withWorkerHook<T>(noWorker: boolean, fn: (c: WorkerCounter) => Promise<T>): Promise<T> {
	const orig = G.Worker;
	const counter: WorkerCounter = { constructed: 0, supported: false, typeofWorker: typeof orig };
	if (noWorker) {
		G.Worker = undefined;
	} else if (typeof orig === 'function') {
		const Orig = orig;
		G.Worker = class CountingWorker extends Orig {
			constructor(...a: any[]) { super(...a); counter.constructed++; }
		};
	}
	try {
		counter.supported = isWorkerSupported();
		return await fn(counter);
	} finally {
		G.Worker = orig;
	}
}

// ── DRC ──────────────────────────────────────────────────────────────
/** generic walk of the grouped tree: leaf = object without array children */
function summarizeDrcTree(tree: any): any {
	const out: any = { categories: [], total: 0, sample: [] };
	const isArr = Array.isArray(tree);
	out.topType = isArr ? 'array' : typeof tree;
	out.topLength = isArr ? tree.length : undefined;
	const childrenOf = (n: any): any[] | null => {
		if (!n || typeof n !== 'object') return null;
		for (const k of ['list', 'children', 'items', 'errors', 'data']) if (Array.isArray(n[k])) return n[k];
		return null;
	};
	const walk = (n: any, path: string[], depth: number): number => {
		const ch = childrenOf(n);
		if (!ch) {
			if (out.sample.length < 20) out.sample.push({ path: path.join(' / '), item: JSON.parse(JSON.stringify(n ?? null)) });
			return 1;
		}
		let c = 0;
		const nm = typeof n.name === 'string' ? n.name : (typeof n.title === 'string' ? n.title : String(path.length));
		for (const x of ch) c += walk(x, [...path, nm], depth + 1);
		if (depth === 1) out.categories.push({ name: nm, count: c, declaredCount: n.count ?? n.total ?? undefined });
		return c;
	};
	if (isArr) for (const t of tree) out.total += walk(t, [], 1);
	return out;
}

async function runDrc(): Promise<any> {
	const e = eda();
	const t0 = Date.now();
	const res: any = { wallMs: 0 };
	try {
		const tree = await e.pcb_Drc.check(true, false, true);
		res.check = summarizeDrcTree(tree);
		try { res.rawHead = JSON.stringify(tree).slice(0, 1500); } catch { /* ignore */ }
		res.rawChars = (() => { try { return JSON.stringify(tree).length; } catch { return -1; } })();
	} catch (err: any) {
		res.checkError = String(err?.message ?? err);
	}
	if (typeof e.pcb_Drc?.checkDetailed === 'function') {
		try {
			const d = await e.pcb_Drc.checkDetailed();
			let s = '';
			try { s = JSON.stringify(d); } catch { /* ignore */ }
			res.detailed = { keys: d && typeof d === 'object' ? Object.keys(d) : null, chars: s.length, head: s.slice(0, 2500) };
			const arr = Array.isArray(d) ? d : (d?.violations ?? d?.items ?? d?.list ?? null);
			if (Array.isArray(arr)) res.detailed.count = arr.length;
			if (d?.state !== undefined) res.detailed.state = d.state;
			// keep a bounded raw copy (positions) for the driver
			if (s.length < 3_000_000) res.detailed.raw = d;
		} catch (err: any) {
			res.detailedError = String(err?.message ?? err);
		}
	}
	res.wallMs = Date.now() - t0;
	return res;
}

// ── snapshot ─────────────────────────────────────────────────────────
async function snapshot(withPads: boolean): Promise<any> {
	const e = eda();
	const lines = ((await e.pcb_PrimitiveLine.getAll()) ?? []).map(plain);
	const arcs = ((await e.pcb_PrimitiveArc.getAll()) ?? []).map(plain);
	const vias = ((await e.pcb_PrimitiveVia.getAll()) ?? []).map(plain);
	const pads = ((await e.pcb_PrimitivePad.getAll()) ?? []).map(plain);
	const polylines = ((await e.pcb_PrimitivePolyline.getAll()) ?? []).map((p: any) => {
		const r = plain(p);
		try { r.pts = parsePolygonPath(p.polygon?.polygon ?? p.polygon ?? []); } catch { r.pts = []; }
		return r;
	});
	let fills: any[] = [];
	try {
		fills = ((await e.pcb_PrimitiveFill.getAll()) ?? []).map((p: any) => {
			const r = plain(p);
			try { r.pts = parsePolygonPath(p.complexPolygon?.polygon ?? p.complexPolygon ?? p.polygon?.polygon ?? p.polygon ?? []); } catch { r.pts = []; }
			return r;
		});
	} catch { /* none */ }
	const snap: any = { takenAt: new Date().toISOString(), lines, arcs, vias, pads, polylines, fills };
	if (withPads) {
		const res = await dumpInput({ ripupRounds: 3, budgetMs: 1, optimize: true, allowViaFallback: true, allowViaInPad: false, netOrder: 'auto' });
		const input = res.input as any;
		// netless pads (NC pins / mounting holes) - same injection as scripts/cli-dump62.js
		const known = new Set(input.allPads.map((p: any) => `${p.x.toFixed(2)},${p.y.toFixed(2)}`));
		let added = 0;
		for (const p of pads) {
			if (p.net) continue;
			const x = p.x ?? 0, y = p.y ?? 0;
			const key = `${Number(x).toFixed(2)},${Number(y).toFixed(2)}`;
			if (known.has(key)) continue;
			const pd = Array.isArray(p.pad) ? p.pad : [];
			const w = pd[1] ?? 0, h = pd[2] ?? 0;
			const hole = Array.isArray(p.hole) && p.hole.length >= 2 ? (p.hole[1] ?? 0) : 0;
			const rawLayer = p.layer ?? 1;
			input.allPads.push({
				x, y, net: '', id: p.primitiveId ?? `netless_${added}`,
				layer: rawLayer === 12 ? 12 : (rawLayer > 2 ? 1 : rawLayer),
				padType: rawLayer === 12 ? 'th' : 'smd', holeDiameter: hole,
				radius: Math.max(w / 2, h / 2, hole / 2), bodyRadius: Math.max(Math.min(w, h) / 2, hole / 2),
				rotation: p.rotation ?? 0, index: input.allPads.length,
			});
			added++;
		}
		snap.drc = input.drc;
		snap.allPads = input.allPads;
		snap.padShapes = input.padShapes;
		snap.copperLayers = input.copperLayers;
		snap.netlessPadsAdded = added;
	}
	return snap;
}

// ── job machinery ────────────────────────────────────────────────────
function newJob(name: string, meta: Record<string, any>): Job {
	const job: Job = { name, status: 'running', startedAt: Date.now(), logs: [], progress: [], progressCount: 0, meta };
	JOBS[name] = job;
	return job;
}
function hooks(job: Job) {
	const onProgress = (p: any) => {
		job.progressCount++;
		// keep every stage change + every 20th message + all with message text (bounded to 4000 entries)
		if (job.progress.length < 4000) job.progress.push({ t: Date.now() - job.startedAt, stage: p.stage, percentage: p.percentage, message: p.message, done: p.done, total: p.total, changed: p.changed, savedMil: p.savedMil });
	};
	const onLog = (message: string, level: string) => {
		if (job.logs.length < 20000) job.logs.push({ t: Date.now() - job.startedAt, level, message: String(message) });
	};
	let lastStats: any = null;
	const onStats = (s: any) => { lastStats = s; job.meta.lastStats = s; };
	return { onProgress, onLog, onStats, getStats: () => lastStats };
}

function launch(job: Job, fn: () => Promise<any>): void {
	void (async () => {
		try { job.result = await fn(); job.status = 'done'; }
		catch (err: any) { job.status = 'error'; job.error = String(err?.message ?? err); job.stack = String(err?.stack ?? '').slice(0, 2000); }
		finally { job.endedAt = Date.now(); }
	})();
}

/** jobs that modify the canvas (and swap the global Worker); read-only jobs (drc, snapshot) are not listed */
const MUTATING_JOBS = new Set(['route', 'tighten']);

export async function handle(args: any): Promise<any> {
	const cmd = args?.cmd;
	if (cmd === 'probe') {
		const out: any = {
			typeofWorker: typeof G.Worker, typeofBlob: typeof G.Blob, typeofCreateObjectURL: typeof G.URL?.createObjectURL,
			isWorkerSupported: isWorkerSupported(),
		};
		try {
			const w: any = createPlanningWorker();
			out.createPlanningWorker = w ? 'non-null' : 'null';
			if (w) { try { w.terminate(); out.terminated = true; } catch (err: any) { out.terminateError = String(err?.message ?? err); } }
		} catch (err: any) { out.createPlanningWorker = 'threw: ' + String(err?.message ?? err); }
		return out;
	}
	if (cmd === 'chunk') {
		const chunks = STORE[args.token];
		if (!chunks) throw new Error('chunk store missing / token mismatch');
		return { data: chunks[args.i], done: args.i >= chunks.length - 1 };
	}
	if (cmd === 'status') {
		const job = JOBS[args.job];
		if (!job) return { exists: false };
		const lf = args.logFrom ?? 0, pf = args.progFrom ?? 0;
		return {
			exists: true, status: job.status, elapsedMs: (job.endedAt ?? Date.now()) - job.startedAt,
			logTotal: job.logs.length, progTotal: job.progress.length, progressCount: job.progressCount,
			logs: job.logs.slice(lf, lf + 400), progress: job.progress.slice(pf, pf + 400),
			meta: job.meta,
			...(job.status === 'running' ? {} : { result: job.result, error: job.error, stack: job.stack }),
		};
	}
	if (cmd !== 'start') throw new Error('unknown cmd ' + cmd);

	const name: string = args.job;
	if (JOBS[name]?.status === 'running') throw new Error(`job ${name} already running`);
	// canvas-modifying jobs (route, tighten) share the global Worker hook and write to the same canvas: never run two at once
	if (MUTATING_JOBS.has(name)) {
		const busy = [...MUTATING_JOBS].find(n => JOBS[n]?.status === 'running');
		if (busy) throw new Error(`cannot start job ${name}: canvas-modifying job ${busy} is still running`);
	}
	const job = newJob(name, { args });
	const h = hooks(job);

	if (name === 'route') {
		launch(job, () => withWorkerHook(false, async (counter) => {
			const router = new TopologicalRouter(h.onProgress, h.onLog, h.onStats);
			const opts = normalizeRouterOptions({ ...(args.options ?? {}) });
			job.meta.options = opts;
			const t0 = Date.now();
			const summary = await router.route(opts);
			return { wallMs: Date.now() - t0, summary, stats: h.getStats(), workerConstructed: counter.constructed, workerCounter: counter };
		}));
	} else if (name === 'tighten') {
		launch(job, () => withWorkerHook(!!args.noWorker, async (counter) => {
			const router = new TopologicalRouter(h.onProgress, h.onLog, h.onStats);
			const t0 = Date.now();
			// test-only fault injection (see topological-router.ts faultCheck): the n-th canvas create call of THIS run throws; always removed afterwards
			if (args.faultAt) G.__RB_TIGHTEN_FAULT__ = { createFailAt: Number(args.faultAt) };
			let result;
			try { result = await router.tightenExistingRouting(); } finally { delete G.__RB_TIGHTEN_FAULT__; }
			return { wallMs: Date.now() - t0, result, workerConstructed: counter.constructed, workerCounter: counter, noWorker: !!args.noWorker, faultAt: args.faultAt ? Number(args.faultAt) : undefined };
		}));
	} else if (name === 'drc') {
		launch(job, () => runDrc());
	} else if (name === 'snapshot') {
		launch(job, async () => {
			const t0 = Date.now();
			const snap = await snapshot(args.withPads !== false);
			const meta = store(snap);
			return { ...meta, wallMs: Date.now() - t0, counts: { lines: snap.lines.length, arcs: snap.arcs.length, vias: snap.vias.length, pads: snap.pads.length, polylines: snap.polylines.length, fills: snap.fills.length } };
		});
	} else {
		delete JOBS[name];
		throw new Error('unknown job ' + name);
	}
	return { started: name };
}
