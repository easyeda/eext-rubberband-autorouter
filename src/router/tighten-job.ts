/**
 * tighten-job - 「拉紧已有布线」任务主循环（Worker 与主线程回退共用）
 *
 * 流程：
 *  1. 建精确世界（tighten-core）：焊盘真实形状 + 钻孔、过孔、只读走线（各自线宽）、
 *     静态障碍（baseWorld，丢弃被拉紧走线的旧副本）、全部 run（各自线宽）。
 *  2. 每条 run 记下固定点（端点 / 穿过的本网焊盘·过孔 / 其它本网 run 的 T 接点）与
 *     连通锚点，原始几何对各条目组的余量懒计算（「既有违规不恶化」口径）。
 *  3. 多轮（≤4）逐段：同伦保持拉紧 + 圆角（tightenChain），过精确校验 + 连通 +
 *     不变长 + 有改善 才采纳；采纳后替换世界里的条目，邻居下一轮再相互拉紧。
 *  4. 可选（需传入 PlanningEngine）：明显绕路的子段用可视图 A* 重解（会改拓扑），
 *     候选再走拉紧 + 圆角 + 精确校验，且必须短 ≥2mil 才采纳；整网 MST 重布同理，
 *     只在「整网同层同宽、无过孔/只读本网铜、旧走线已连通全部焊盘」时尝试。
 */

import type { DRCRules } from './drc-rules';
import type { RBObstacle } from './rubberband-engine';
import type { EnginePad, PadShape, PlannedVia, RenderedPrim, PlanningEngine } from './planning-engine';
import {
	TightenWorld, WorldItem, RunEnv, FixedPt, Shape, Pt,
	padItems, viaItems, trackItems, staticItems, slackOfPrimsVsItems, validatePrims,
	normalizeChain, mergeChain, chainLength, chainMaxGap, cornerStats, locateOnChain,
	chainShapeMin, splitAtFixed, tightenChain, tightenCandidate, primStart, primEnd,
	primProject, shapeDist, shapeCenter, chainBBox,
} from './tighten-core';

// ─────────────────────────────────────────────
// 消息契约
// ─────────────────────────────────────────────

export interface TightenJobRun {
	id: number;
	net: string;
	layer: number;
	/** 实际线宽（mil），缺省 drc.traceWidthMil */
	width?: number;
	a: { x: number; y: number };
	b: { x: number; y: number };
	prims: RenderedPrim[];
}

export interface TightenJobInput {
	type: 'tighten';
	drc: DRCRules;
	allPads: EnginePad[];
	padShapes: Record<string, PadShape>;
	/** 仅静态障碍（按 r = 铜半宽 + 间距 + traceWidth/2 + 余量 预膨胀） */
	baseWorld: RBObstacle[];
	/** 固定过孔（精确圆，全层） */
	vias: PlannedVia[];
	/** 不参与拉紧的只读铜皮（精确） */
	tracks?: Array<{ net: string; layer: number; width: number; prim: RenderedPrim }>;
	runs: TightenJobRun[];
}

export type TightenKind = 'taut' | 'smooth' | 'solve' | null;

export type TightenJobEvent =
	| { type: 'tightenRun'; id: number; net: string; prims: RenderedPrim[] | null; oldLen: number; newLen: number; kind: TightenKind }
	| { type: 'netRewrite'; net: string; prims: RenderedPrim[]; oldLen: number; newLen: number }
	| {
		type: 'tightenDone'; total: number; adopted: number; saved: number; rounded: number; solved: number;
		rewritten: number; solveAttempts: number; solveMs: number; passes: number;
	};

// ─────────────────────────────────────────────
// 参数
// ─────────────────────────────────────────────

const MAX_PASSES = 4;
/** 阶段 2（A* 重解）总预算 */
const SOLVE_BUDGET_MS = 25_000;
const SOLVE_MAX_ATTEMPTS = 300;
/** 阶段 2 / 整网重布至少缩短这么多才采纳 */
const SOLVE_MIN_GAIN = 2;
/** 阶段 2 触发：子段比直连至少长这么多 */
const SOLVE_MIN_DETOUR = 5;

/** isStatic：锚点来自本网静态铜（baseWorld 里的同网铺铜/填充/走线骨架），整网重布无法保证保住它 */
interface PadAnchor { shape: Shape; need: number; isStatic?: boolean }

interface JobRun {
	id: number;
	net: string;
	layer: number;
	width: number;
	a: Pt;
	b: Pt;
	inputLen: number;
	orig: RenderedPrim[];
	origLen: number;
	cur: RenderedPrim[];
	origItems: WorldItem[];
	fixed: FixedPt[];
	anchors: PadAnchor[];
	junctions: Pt[];
	slackCache: Map<string, number>;
	adopted: boolean;
	kind: TightenKind;
	reported: boolean;
	engineGroups: string[];
	loop: boolean;
	/** 已被整网改写取代（不再逐段上报） */
	rewritten: boolean;
}

const yieldNow = (): Promise<void> => new Promise(res => setTimeout(res, 0));

export async function runTightenJob(
	msg: TightenJobInput,
	emit: (m: TightenJobEvent) => void,
	isCancelled: () => boolean,
	engine?: PlanningEngine,
): Promise<void> {
	const drc = msg.drc;
	const tw = drc.traceWidthMil;
	const world = new TightenWorld();
	const pads = (msg.allPads ?? []).filter(p => p && Number.isFinite(p.x) && Number.isFinite(p.y));
	const vias = (msg.vias ?? []).filter(v => v && Number.isFinite(v.x) && Number.isFinite(v.y));
	const tracks = (msg.tracks ?? []).filter(t => t && t.prim && Number.isFinite(t.width));
	padItems(world, pads, msg.padShapes ?? {}, drc);
	viaItems(world, vias, drc);
	tracks.forEach((t, i) => trackItems(world, [t.prim], t.width, t.net, t.layer, `trk:${i}`, drc));

	// ── runs ──
	const runs: JobRun[] = [];
	for (const r of msg.runs ?? []) {
		const width = Number.isFinite(r.width) && (r.width ?? 0) > 0 ? (r.width as number) : tw;
		const prims = (r.prims ?? []).filter(Boolean);
		const inputLen = chainLength(prims);
		const loop = Math.hypot(r.a.x - r.b.x, r.a.y - r.b.y) < 1e-3;
		const ordered = prims.length ? normalizeChain(prims, r.a, loop ? null : r.b, r.net, r.layer) : [];
		const items = ordered.length ? trackItems(world, ordered, width, r.net, r.layer, `run:${r.id}`, drc) : [];
		runs.push({
			id: r.id, net: r.net, layer: r.layer, width, a: { ...r.a }, b: { ...r.b },
			inputLen, orig: ordered, origLen: chainLength(ordered), cur: ordered, origItems: items,
			fixed: [], anchors: [], junctions: [], slackCache: new Map(),
			adopted: false, kind: null, reported: false, engineGroups: [], loop, rewritten: false,
		});
	}
	const runById = new Map(runs.map(r => [r.id, r]));
	const runsByNet = new Map<string, JobRun[]>();
	for (const r of runs) {
		const arr = runsByNet.get(r.net);
		if (arr) arr.push(r); else runsByNet.set(r.net, [r]);
	}

	// ── 静态世界：丢弃被拉紧/只读走线在 baseWorld 里的旧副本 ──
	const copperByNet = new Map<string, Array<{ layer: number; prim: RenderedPrim }>>();
	for (const r of runs) for (const p of r.orig) {
		const arr = copperByNet.get(r.net) ?? [];
		arr.push({ layer: r.layer, prim: p });
		copperByNet.set(r.net, arr);
	}
	for (const t of tracks) {
		const arr = copperByNet.get(t.net) ?? [];
		arr.push({ layer: t.layer, prim: t.prim });
		copperByNet.set(t.net, arr);
	}
	const copyGroups = new Set<string>();
	const isCopy = (group: string, obs: RBObstacle[]): boolean => {
		const prefix = group.split(':')[0];
		let copy = false;
		if (prefix === 'via') {
			copy = obs.every(o => vias.some(v => Math.hypot(v.x - o.x, v.y - o.y) < 0.5));
		} else if (['line', 'arc', 'poly', 'trk'].includes(prefix)) {
			const net = obs[0]?.net;
			const cu = net ? copperByNet.get(net) : undefined;
			if (cu && obs.every(o => o.net === net)) {
				copy = obs.filter(o => !o.blockerOnly).every(o => cu.some(c =>
					(o.layer === undefined || c.layer === o.layer) && primProject(c.prim, o.x, o.y).d < 1.0));
			}
		}
		if (copy) copyGroups.add(group);
		return copy;
	};
	staticItems(world, msg.baseWorld ?? [], drc, isCopy);

	// ── 固定点 / 连通锚点（按原始几何） ──
	for (const r of runs) computeFixed(r, world, runsByNet, tracks, drc);

	// ── 原始余量（懒） ──
	const origSlack = (r: JobRun, group: string): number => {
		const hit = r.slackCache.get(group);
		if (hit !== undefined) return hit;
		let items: WorldItem[];
		if (group.startsWith('run:')) {
			const o = runById.get(Number(group.slice(4)));
			items = o ? o.origItems : [];
		} else if (group.startsWith('net:')) {
			items = (runsByNet.get(group.slice(4)) ?? []).flatMap(q => q.origItems);
		} else {
			items = world.groupItems(group);
		}
		const v = slackOfPrimsVsItems(r.orig, r.width, items);
		r.slackCache.set(group, v);
		return v;
	};
	const envOf = (r: JobRun): RunEnv => ({
		world, net: r.net, layer: r.layer, width: r.width,
		selfGroups: new Set([`run:${r.id}`]),
		origSlack: g => origSlack(r, g),
	});

	// ── 可选：A* 引擎世界（旧口径，只用来提候选；采纳一律走精确校验） ──
	if (engine) {
		try {
			engine.initTightenWorld(drc, pads, (msg.baseWorld ?? []).filter(o => !copyGroups.has(o.group ?? '')), msg.padShapes ?? {});
			for (const r of runs) r.engineGroups = r.cur.length ? engine.addRunToWorld(r.cur) : [];
			for (const v of vias) engine.addViaToWorld(v);
			for (const t of tracks) engine.addRunToWorld([{ ...t.prim, net: t.net, layer: t.layer }]);
		} catch {
			engine = undefined;
		}
	}

	const adoptPrims = (r: JobRun, prims: RenderedPrim[]): void => {
		world.removeGroup(`run:${r.id}`);
		trackItems(world, prims, r.width, r.net, r.layer, `run:${r.id}`, drc);
		r.cur = prims;
		if (engine) {
			try {
				engine.removeWorldGroups(r.engineGroups);
				r.engineGroups = engine.addRunToWorld(prims);
			} catch { /* 引擎世界只影响候选质量 */ }
		}
	};

	/** 采纳检查：校验 + 连通 + 长度 + 尖角 */
	const acceptable = (r: JobRun, env: RunEnv, prims: RenderedPrim[], minGain: number): boolean => {
		if (!prims.length) return false;
		if (prims.some(p => p.layer !== r.layer || !finitePrim(p))) return false;
		if (chainMaxGap(prims) > 0.01) return false;
		const s = primStart(prims[0]), e = primEnd(prims[prims.length - 1]);
		if (Math.hypot(s.x - r.a.x, s.y - r.a.y) > 0.001) return false;
		if (!r.loop && Math.hypot(e.x - r.b.x, e.y - r.b.y) > 0.001) return false;
		const newLen = chainLength(prims), curLen = chainLength(r.cur);
		if (newLen > curLen + 0.005 || newLen > r.origLen + 0.008 || newLen > r.inputLen + 0.008) return false;
		const cs = cornerStats(r.cur, statFixed(r, r.cur)), ns = cornerStats(prims, statFixed(r, prims));
		if (ns.sharp > cs.sharp) return false;
		const improved = minGain > 0
			? newLen < curLen - minGain
			: newLen < curLen - 0.01 || ns.sharp < cs.sharp || ns.kinks < cs.kinks;
		if (!improved) return false;
		for (const j of r.junctions) if (locateOnChain(prims, j.x, j.y).d > 0.01) return false;
		for (const an of r.anchors) if (chainShapeMin(prims, an.shape) > an.need + 1e-6) return false;
		return validatePrims(env, prims).ok;
	};

	// ── 主循环 ──
	const order = [...runs].sort((p, q) =>
		(q.origLen - Math.hypot(q.b.x - q.a.x, q.b.y - q.a.y)) - (p.origLen - Math.hypot(p.b.x - p.a.x, p.b.y - p.a.y)));
	let dirty = new Set(runs.map(r => r.id));
	let passes = 0;
	let solveCount = 0, solveSpent = 0;
	let lastYield = Date.now();
	const maybeYield = async (): Promise<void> => {
		if (Date.now() - lastYield > 25) { await yieldNow(); lastYield = Date.now(); }
	};
	const report = (r: JobRun, prims: RenderedPrim[] | null): void => {
		r.reported = true;
		emit({
			type: 'tightenRun', id: r.id, net: r.net, prims, oldLen: r.inputLen,
			newLen: prims ? chainLength(prims) : chainLength(r.cur), kind: prims ? r.kind : null,
		});
	};

	/** 阶段 1（拉紧 + 圆角，同伦不变）；采纳后迭代到不动点 */
	const stage1 = (r: JobRun): RenderedPrim[] | null => {
		if (!r.cur.length) return null;
		const env = envOf(r);
		const fixed = fixedFor(r);
		let changed: RenderedPrim[] | null = null;
		try {
			// 阶梯：拉紧 + 圆角 → 原样 + 圆角
			const t = tightenChain(env, r.cur, fixed, { taut: !r.loop });
			if (t && acceptable(r, env, t.prims, 0)) {
				changed = t.prims;
				r.kind = t.taut ? 'taut' : 'smooth';
			} else if (t && t.taut) {
				const s = tightenChain(env, r.cur, fixed, { taut: false });
				if (s && acceptable(r, env, s.prims, 0)) { changed = s.prims; r.kind = 'smooth'; }
			}
			if (changed) {
				adoptPrims(r, changed);
				// 迭代到不动点：穿过焊盘的固定点是「链上离焊盘中心最近处」，圆角后会
				// 移动一点，再拉一次还能再短一点——在本任务内收敛，避免下次拉紧时蠕动
				for (let it = 0; it < 8; it++) {
					const t2 = tightenChain(env, r.cur, fixed, { taut: !r.loop });
					if (!t2 || !acceptable(r, env, t2.prims, 0)) break;
					adoptPrims(r, t2.prims);
					changed = t2.prims;
					if (t2.taut) r.kind = 'taut';
				}
			}
		} catch (err) {
			// 单段几何异常：保持当前几何（不影响其它段）
			if ((globalThis as any).__RB_TIGHTEN_DIAG__) console.warn('[tighten] run', r.id, err);
		}
		return changed ? r.cur : null;
	};
	/** 采纳后：自己与包围盒附近的段下一轮重拉（邻居让出空间后能再紧） */
	const markAround = (r: JobRun, prims: RenderedPrim[], set: Set<number>): void => {
		r.adopted = true;
		const bb = chainBBox(prims);
		for (const q of runs) {
			if (q.id === r.id || !q.cur.length) continue;
			const qb = chainBBox(q.cur);
			if (qb.x0 > bb.x1 + 60 || qb.x1 < bb.x0 - 60 || qb.y0 > bb.y1 + 60 || qb.y1 < bb.y0 - 60) continue;
			set.add(q.id);
		}
		set.add(r.id);
	};
	const runPasses = async (dirtyInit: Set<number>, maxPasses: number, firstReport: boolean): Promise<void> => {
		let dirty = dirtyInit;
		for (let pass = 1; pass <= maxPasses && dirty.size && !isCancelled(); pass++) {
			passes++;
			const next = new Set<number>();
			for (const r of order) {
				if (isCancelled()) break;
				if (!dirty.has(r.id)) continue;
				const changed = stage1(r);
				if (changed) markAround(r, changed, next);
				if ((firstReport && pass === 1) || changed) report(r, changed);
				await maybeYield();
			}
			dirty = next;
		}
	};

	await runPasses(dirty, MAX_PASSES, true);
	// 取消时补报未处理的段（进度计数）
	for (const r of runs) if (!r.reported) report(r, null);

	// ── 阶段 2：明显绕路的子段 A* 重解（在阶段 1 收敛、邻居都已拉紧之后做） ──
	// 重解采纳后邻居会让出新空间，重复「重解 → 拉紧」直到重解不再采纳（≤3 轮）
	for (let round = 0; round < 3 && engine && !isCancelled(); round++) {
		const dirty2 = new Set<number>();
		// 余量（当前长 − 直连）大的先试：预算花在最可能缩短的段上
		const order2 = [...runs].sort((p, q) =>
			(chainLength(q.cur) - Math.hypot(q.b.x - q.a.x, q.b.y - q.a.y)) - (chainLength(p.cur) - Math.hypot(p.b.x - p.a.x, p.b.y - p.a.y)));
		for (const r of order2) {
			if (isCancelled() || solveCount >= SOLVE_MAX_ATTEMPTS || solveSpent >= SOLVE_BUDGET_MS) break;
			if (r.loop || !r.cur.length) continue;
			try {
				const res = await solveDetours(r, envOf(r), fixedFor(r));
				if (res) {
					adoptPrims(r, res);
					r.kind = 'solve';
					markAround(r, res, dirty2);
					report(r, res);
				}
			} catch { /* 重解失败：保持当前几何 */ }
			await maybeYield();
		}
		if (!dirty2.size) break;
		await runPasses(dirty2, 3, false);
	}

	// ── 整网 MST 重布 ──
	let rewritten = 0;
	const rewriteSaved = new Map<string, number>();
	if (engine && !isCancelled()) {
		for (const [net, rs] of runsByNet) {
			if (isCancelled()) break;
			if (solveCount >= SOLVE_MAX_ATTEMPTS || solveSpent >= SOLVE_BUDGET_MS) break;
			try {
				const res = await rewriteNet(net, rs);
				if (res) {
					rewritten++;
					rewriteSaved.set(net, rs.reduce((sum, q) => sum + q.inputLen, 0) - res.newLen);
					emit({ type: 'netRewrite', net, prims: res.prims, oldLen: res.oldLen, newLen: res.newLen });
				}
			} catch { /* 整网改写失败：保持逐段结果 */ }
			await maybeYield();
		}
	}

	let adopted = 0, saved = 0, rounded = 0, solved = 0;
	for (const r of runs) {
		if (!r.adopted) continue;
		adopted++;
		if (r.rewritten) continue;
		saved += Math.max(0, r.inputLen - chainLength(r.cur));
		if (r.kind === 'smooth') rounded++;
		if (r.kind === 'solve') solved++;
	}
	for (const v of rewriteSaved.values()) saved += Math.max(0, v);
	emit({
		type: 'tightenDone', total: runs.length, adopted, saved, rounded, solved, rewritten,
		solveAttempts: solveCount, solveMs: Math.round(solveSpent), passes,
	});

	// ─────────────────────────────────────────────
	// 内部函数
	// ─────────────────────────────────────────────

	function fixedFor(r: JobRun): FixedPt[] {
		return [{ x: r.a.x, y: r.a.y, kind: 'end' }, { x: r.b.x, y: r.b.y, kind: 'end' }, ...r.fixed];
	}

	/** 尖角统计口径的固定点：端点 + T 接点 + 焊盘点（投影到被统计的链上） */
	function statFixed(r: JobRun, prims: RenderedPrim[]): FixedPt[] {
		const out: FixedPt[] = [{ x: r.a.x, y: r.a.y, kind: 'end' }, { x: r.b.x, y: r.b.y, kind: 'end' }];
		for (const f of r.fixed) {
			if (f.kind !== 'pad') { out.push(f); continue; }
			const l = locateOnChain(prims, f.x, f.y);
			const p = prims[l.i];
			if (!p) continue;
			out.push({ ...f, ...pointAtS(p, l.s) });
		}
		return out;
	}

	/** 阶段 2：子段（固定点之间）明显绕路时用 A* 重解，候选走 B + C + 精确校验 */
	async function solveDetours(r: JobRun, env: RunEnv, fixed: FixedPt[]): Promise<RenderedPrim[] | null> {
		if (!engine) return null;
		const { subs } = splitAtFixed(r.cur, fixed);
		let changedAny = false;
		const out: RenderedPrim[][] = [];
		for (const sub of subs) {
			out.push(sub);
			if (!sub.length) continue;
			const s = primStart(sub[0]), e = primEnd(sub[sub.length - 1]);
			const straight = Math.hypot(e.x - s.x, e.y - s.y);
			const len = chainLength(sub);
			// 只要「比直连长出可观的量」就值得一试（拉紧后仍绕的段，换另一侧可能更短）
			if (straight <= 5 || len - straight < Math.max(SOLVE_MIN_DETOUR, straight * 0.02)) continue;
			if (solveCount >= SOLVE_MAX_ATTEMPTS || solveSpent >= SOLVE_BUDGET_MS || isCancelled()) break;
			const t0 = Date.now();
			let sc: RenderedPrim[] | null = null;
			try { sc = await engine.solveRunSegment(s, e, r.layer, r.net); } catch { sc = null; }
			solveSpent += Date.now() - t0;
			solveCount++;
			if (!sc || !sc.length) continue;
			const norm = mergeChain(normalizeChain(sc.map(p => ({ ...p, net: r.net, layer: r.layer })), s, e, r.net, r.layer));
			if (!norm.length || chainMaxGap(norm) > 0.01) continue;
			const ns = primStart(norm[0]), ne = primEnd(norm[norm.length - 1]);
			if (Math.hypot(ns.x - s.x, ns.y - s.y) > 0.001 || Math.hypot(ne.x - e.x, ne.y - e.y) > 0.001) continue;
			const cand = tightenCandidate(env, norm, []);
			if ((globalThis as any).__RB_TIGHTEN_DIAG__) {
				console.warn('[tighten] solve', r.id, 'sub', len.toFixed(2), 'A*', chainLength(norm).toFixed(2), 'cand', cand ? chainLength(cand).toFixed(2) : null,
					'rawValid', validatePrims(env, norm).ok);
			}
			if (!cand || chainLength(cand) >= len - SOLVE_MIN_GAIN) continue;
			out[out.length - 1] = cand;
			changedAny = true;
		}
		if (!changedAny) return null;
		const joined = mergeChain(out.flat());
		// 子段接缝（焊盘点）再圆角
		const t = tightenChain(env, joined, fixed, { taut: false });
		const best = t && acceptable(r, env, t.prims, SOLVE_MIN_GAIN) ? t.prims : joined;
		return acceptable(r, env, best, SOLVE_MIN_GAIN) ? best : null;
	}

	/**
	 * 整网 MST 重布：只在安全条件下尝试（同层同宽、无本网过孔/只读铜、旧走线连通
	 * 全部焊盘），候选逐链 B + C + 精确校验，整网短 max(10, 8%) 才采纳。
	 */
	async function rewriteNet(net: string, rs: JobRun[]): Promise<{ prims: RenderedPrim[]; oldLen: number; newLen: number } | null> {
		if (!engine || !rs.length) return null;
		const layer = rs[0].layer, width = rs[0].width;
		if (rs.some(q => q.layer !== layer || Math.abs(q.width - width) > 1e-6 || q.loop)) return null;
		if (vias.some(v => v.net === net) || tracks.some(t => t.net === net)) return null;
		// 旧走线接触/压着同网静态铜（铺铜、填充）：它们是连通锚点，MST 重布不保证保住 → 不重布
		if (rs.some(q => q.anchors.some(a => a.isStatic))) return null;
		const netPads = world.groupItemsByNet(net).filter(it => it.kind === 'pad');
		if (netPads.length < 2) return null;
		if (netPads.some(it => it.layer !== undefined && it.layer !== layer)) return null;
		// 旧走线必须已连通全部焊盘（不改变连通关系），且没有悬空端（悬空的走线头
		// 可能是有意保留的测试点/天线，整网改写会把它删掉）
		if (!netConnected(rs, netPads, width)) return null;
		for (const q of rs) {
			for (const e of [q.a, q.b]) {
				const onPad = netPads.some(it => shapeDist(it.shape, e.x, e.y) < width / 2);
				const onRun = rs.some(o => o !== q && (
					Math.hypot(o.a.x - e.x, o.a.y - e.y) < 0.8 || Math.hypot(o.b.x - e.x, o.b.y - e.y) < 0.8 ||
					locateOnChain(o.cur, e.x, e.y).d < width / 2));
				if (!onPad && !onRun) return null;
			}
		}
		const oldLen = rs.reduce((s, q) => s + chainLength(q.cur), 0);
		if (oldLen <= 0) return null;
		const t0 = Date.now();
		let cand: RenderedPrim[] | null = null;
		try { cand = await engine.rerouteNetMST(net, layer); } catch { cand = null; }
		solveSpent += Date.now() - t0;
		solveCount++;
		if (!cand || !cand.length) return null;
		const quick = chainLength(cand);
		if (quick > oldLen - Math.max(10, oldLen * 0.08) + 50) return null;
		const env: RunEnv = {
			world, net, layer, width,
			selfGroups: new Set(rs.map(q => `run:${q.id}`)),
			origSlack: g => Math.min(...rs.map(q => origSlack(q, g))),
		};
		const chains = splitChains(cand.map(p => ({ ...p, net, layer })));
		const outPrims: RenderedPrim[] = [];
		for (const ch of chains) {
			const a = primStart(ch[0]), b = primEnd(ch[ch.length - 1]);
			const norm = mergeChain(normalizeChain(ch, a, b, net, layer));
			const c = tightenCandidate(env, norm, []);
			if (!c) return null;
			outPrims.push(...c);
		}
		const newLen = chainLength(outPrims);
		if (newLen > oldLen - Math.max(10, oldLen * 0.08)) return null;
		if (!validatePrims(env, outPrims).ok) return null;
		// 每个焊盘都必须被新走线接触；每条链两端都落在本网焊盘铜皮内
		for (const it of netPads) if (chainShapeMin(outPrims, it.shape) > Math.min(0, width / 2 - 0.25)) return null;
		const outChains = splitChains(outPrims);
		for (const ch of outChains) {
			for (const p of [primStart(ch[0]), primEnd(ch[ch.length - 1])]) {
				if (!netPads.some(it => shapeDist(it.shape, p.x, p.y) <= 0)) return null;
			}
		}
		// 真正的连通性复核：新铜皮必须把本网全部焊盘连成一个连通块（不能只是「每盘被碰到」）
		if (!padsConnectedByCopper(outChains, netPads.map(it => it.shape), width)) return null;
		// 采纳：整网 run 条目替换为新链
		for (const q of rs) {
			world.removeGroup(`run:${q.id}`);
			try { engine.removeWorldGroups(q.engineGroups); } catch { /* ignore */ }
			q.engineGroups = [];
		}
		trackItems(world, outPrims, width, net, layer, `net:${net}`, drc);
		try { engine.addRunToWorld(outPrims); } catch { /* ignore */ }
		for (const q of rs) { q.cur = []; q.adopted = true; q.kind = 'solve'; q.rewritten = true; }
		return { prims: outPrims, oldLen, newLen };
	}
}

// ─────────────────────────────────────────────
// 辅助
// ─────────────────────────────────────────────

function finitePrim(p: RenderedPrim): boolean {
	const v = p.kind === 'line' ? [p.x1, p.y1, p.x2, p.y2] : [p.cx, p.cy, p.r, p.startAngle, p.sweep];
	return v.every(x => typeof x === 'number' && Number.isFinite(x));
}

function pointAtS(p: RenderedPrim, s: number): Pt {
	if (p.kind === 'line') {
		const L = Math.hypot((p.x2 ?? 0) - (p.x1 ?? 0), (p.y2 ?? 0) - (p.y1 ?? 0));
		const t = L > 1e-12 ? s / L : 0;
		return { x: (p.x1 ?? 0) + ((p.x2 ?? 0) - (p.x1 ?? 0)) * t, y: (p.y1 ?? 0) + ((p.y2 ?? 0) - (p.y1 ?? 0)) * t };
	}
	const r = p.r ?? 0;
	const a = (p.startAngle ?? 0) + (r > 1e-12 ? Math.sign(p.sweep ?? 0) * s / r : 0);
	return { x: (p.cx ?? 0) + r * Math.cos(a), y: (p.cy ?? 0) + r * Math.sin(a) };
}

/** 按接缝断开（>0.01）拆成多条连续链 */
function splitChains(prims: RenderedPrim[]): RenderedPrim[][] {
	const out: RenderedPrim[][] = [];
	let cur: RenderedPrim[] = [];
	for (const p of prims) {
		if (cur.length) {
			const e = primEnd(cur[cur.length - 1]), s = primStart(p);
			if (Math.hypot(e.x - s.x, e.y - s.y) > 0.01) { out.push(cur); cur = []; }
		}
		cur.push(p);
	}
	if (cur.length) out.push(cur);
	return out;
}

/** 链上采样点（步长 step） */
function sampleChain(prims: RenderedPrim[], step: number): Pt[] {
	const out: Pt[] = [];
	for (const p of prims) {
		const L = p.kind === 'line'
			? Math.hypot((p.x2 ?? 0) - (p.x1 ?? 0), (p.y2 ?? 0) - (p.y1 ?? 0))
			: Math.abs(p.sweep ?? 0) * (p.r ?? 0);
		const n = Math.max(1, Math.ceil(L / step));
		for (let k = 0; k <= n; k++) out.push(pointAtS(p, (L * k) / n));
	}
	return out;
}

/**
 * 固定点（按原始几何）：
 *  - 穿过/接触的本网焊盘、过孔（不在端点处接触者）：链上离焊盘中心最近处；
 *  - 其它本网 run / 只读本网走线的端点落在本链中段：T 接点；
 *  - 与其它本网铜皮中段相交（无节点的交叉）：交叉处最近点。
 */
function computeFixed(
	r: JobRun, world: TightenWorld, runsByNet: Map<string, JobRun[]>,
	tracks: Array<{ net: string; layer: number; width: number; prim: RenderedPrim }>,
	drc: DRCRules,
): void {
	if (!r.orig.length) return;
	const w = r.width;
	const bb = chainBBox(r.orig);
	const nearEnd = (p: Pt, tol: number) =>
		Math.hypot(p.x - r.a.x, p.y - r.a.y) <= tol || Math.hypot(p.x - r.b.x, p.y - r.b.y) <= tol;
	const addFixed = (f: FixedPt): void => {
		if (r.fixed.some(q => Math.hypot(q.x - f.x, q.y - f.y) < 0.05)) return;
		if (nearEnd(f, 0.05)) return;
		r.fixed.push(f);
	};
	const pointOnChain = (x: number, y: number): Pt => {
		const l = locateOnChain(r.orig, x, y);
		return pointAtS(r.orig[l.i], l.s);
	};
	// 本网焊盘 / 过孔
	world.query({ x0: bb.x0 - w, y0: bb.y0 - w, x1: bb.x1 + w, y1: bb.y1 + w }, it => {
		if (it.net !== r.net || (it.kind !== 'pad' && it.kind !== 'via')) return;
		if (it.layer !== undefined && it.layer !== r.layer) return;
		const dmin = chainShapeMin(r.orig, it.shape);
		const touch = w / 2 - 0.05;
		if (dmin >= touch) return;
		// 连通锚点：新走线中心线到铜皮的有符号距离必须 ≤ need。原走线穿过铜皮（dmin < 0）时，
		// 中心线须保持至少 w/2 的深度（整个线宽都压在铜上，真正的搭接）；原来更浅者不得比原来更浅。
		// 原来只擦着边（dmin ≥ 0）：不得比原来更远。圆角切点（tighten-core filletLL/LA 的 padNeed）用同一口径。
		r.anchors.push({ shape: it.shape, need: dmin >= 0 ? dmin : Math.max(dmin, -w / 2) });
		if (shapeDist(it.shape, r.a.x, r.a.y) < touch || shapeDist(it.shape, r.b.x, r.b.y) < touch) return;
		const c = shapeCenter(it.shape);
		let p = pointOnChain(c.x, c.y);
		if (shapeDist(it.shape, p.x, p.y) >= touch) {
			let best = Infinity;
			for (const q of sampleChain(r.orig, 0.25)) {
				const d = shapeDist(it.shape, q.x, q.y);
				if (d < best) { best = d; p = q; }
			}
			p = pointOnChain(p.x, p.y);
		}
		// 原走线正好穿过焊盘中心：中心是硬途经点（不倒角，尖角落在本网焊盘中心属允许）；
		// 偏心穿过：可在焊盘铜皮内倒角
		if (Math.hypot(p.x - c.x, p.y - c.y) < 0.05) addFixed({ x: c.x, y: c.y, kind: 'junction' });
		else addFixed({ x: p.x, y: p.y, kind: 'pad', anchor: it.shape });
	});
	// 本网静态铜（同网铺铜/填充/未拆段的走线，预膨胀骨架）：旧走线贴着/压着它，
	// 新走线就不许离得更远（锚点 + 固定点，不允许滑开断连）
	const minRule = Math.min(drc.trackToTrackMil, drc.trackToFillMil, drc.trackToPadMil);
	world.query({ x0: bb.x0 - w - 50, y0: bb.y0 - w - 50, x1: bb.x1 + w + 50, y1: bb.y1 + w + 50 }, it => {
		if (it.net !== r.net || it.kind !== 'static') return;
		if (it.layer !== undefined && it.layer !== r.layer) return;
		const dmin = chainShapeMin(r.orig, it.shape);
		if (dmin >= it.R0 - minRule + w / 2 - 0.05) return;
		r.anchors.push({ shape: it.shape, need: dmin + 0.01, isStatic: true });
		if (shapeDist(it.shape, r.a.x, r.a.y) <= dmin + 0.01 || shapeDist(it.shape, r.b.x, r.b.y) <= dmin + 0.01) return;
		let p = r.a, best = Infinity;
		for (const q of sampleChain(r.orig, 0.25)) {
			const d = shapeDist(it.shape, q.x, q.y);
			if (d < best) { best = d; p = q; }
		}
		p = pointOnChain(p.x, p.y);
		addFixed({ x: p.x, y: p.y, kind: 'pad', anchor: it.shape });
	});
	// 其它本网铜皮
	const others: Array<{ prims: RenderedPrim[]; ends: Pt[]; width: number }> = [];
	for (const q of runsByNet.get(r.net) ?? []) {
		if (q.id === r.id || q.layer !== r.layer || !q.orig.length) continue;
		others.push({ prims: q.orig, ends: [q.a, q.b], width: q.width });
	}
	for (const t of tracks) {
		if (t.net !== r.net || t.layer !== r.layer) continue;
		others.push({ prims: [t.prim], ends: [primStart(t.prim), primEnd(t.prim)], width: t.width });
	}
	const samples = sampleChain(r.orig, 0.5);
	for (const o of others) {
		const ob = chainBBox(o.prims);
		const tol = (w + o.width) / 2 - 0.05;
		if (ob.x0 > bb.x1 + tol || ob.x1 < bb.x0 - tol || ob.y0 > bb.y1 + tol || ob.y1 < bb.y0 - tol) continue;
		// 对方端点落在本链中段：T 接点
		for (const e of o.ends) {
			if (nearEnd(e, tol)) continue;
			const l = locateOnChain(r.orig, e.x, e.y);
			if (l.d >= tol) continue;
			const p = pointAtS(r.orig[l.i], l.s);
			addFixed({ x: p.x, y: p.y, kind: 'junction' });
			const j = { x: p.x, y: p.y };
			if (!r.junctions.some(q => Math.hypot(q.x - j.x, q.y - j.y) < 0.05)) r.junctions.push(j);
		}
		// 中段交叉（无节点）
		let bestIdx = -1, bestD = Infinity;
		const flush = (): void => {
			if (bestIdx < 0) return;
			const p0 = samples[bestIdx];
			const farFromEnds = !nearEnd(p0, tol) && !o.ends.some(e => Math.hypot(e.x - p0.x, e.y - p0.y) <= tol);
			if (farFromEnds) {
				const p = pointOnChain(p0.x, p0.y);
				addFixed({ x: p.x, y: p.y, kind: 'junction' });
				if (!r.junctions.some(q => Math.hypot(q.x - p.x, q.y - p.y) < 0.05)) r.junctions.push(p);
			}
			bestIdx = -1; bestD = Infinity;
		};
		for (let k = 0; k < samples.length; k++) {
			const s = samples[k];
			if (s.x < ob.x0 - tol || s.x > ob.x1 + tol || s.y < ob.y0 - tol || s.y > ob.y1 + tol) { flush(); continue; }
			let d = Infinity;
			for (const p of o.prims) d = Math.min(d, primProject(p, s.x, s.y).d);
			if (d < tol) {
				if (d < bestD) { bestD = d; bestIdx = k; }
			} else {
				flush();
			}
		}
		flush();
	}
}

/**
 * 一组铜皮链是否把全部焊盘连成一个连通块（纯函数，便于单测）。
 * 链内部视为连通；链与链：端点相接（< 0.8mil）或一条链的端点落在另一条链中心线 width/2 以内（T 接）；
 * 链与焊盘：链中心线到焊盘铜皮的最小距离 < width/2 − 0.05（含「直穿焊盘中段」）。
 * 并查集：全部焊盘落在同一分量才返回 true。
 */
export function padsConnectedByCopper(chains: RenderedPrim[][], padShapes: Shape[], width: number): boolean {
	const live = chains.filter(c => c.length);
	const n = live.length + padShapes.length;
	const parent = Array.from({ length: n }, (_, i) => i);
	const find = (i: number): number => { while (parent[i] !== i) { parent[i] = parent[parent[i]]; i = parent[i]; } return i; };
	const unite = (i: number, j: number): void => { parent[find(i)] = find(j); };
	const ends = live.map(c => [primStart(c[0]), primEnd(c[c.length - 1])]);
	for (let i = 0; i < live.length; i++) {
		for (let j = i + 1; j < live.length; j++) {
			let touch = ends[i].some(e => ends[j].some(f => Math.hypot(e.x - f.x, e.y - f.y) < 0.8));
			if (!touch) touch = ends[i].some(e => locateOnChain(live[j], e.x, e.y).d < width / 2);
			if (!touch) touch = ends[j].some(e => locateOnChain(live[i], e.x, e.y).d < width / 2);
			if (touch) unite(i, j);
		}
		for (let k = 0; k < padShapes.length; k++) {
			if (chainShapeMin(live[i], padShapes[k]) < width / 2 - 0.05) unite(i, live.length + k);
		}
	}
	if (!padShapes.length) return true;
	const root = find(live.length);
	for (let k = 1; k < padShapes.length; k++) if (find(live.length + k) !== root) return false;
	return true;
}

/** 旧走线是否把本网全部焊盘连成一个连通块（run 端点 0.8mil 相接 / 接触同一焊盘 / 端点落在对方链上） */
function netConnected(rs: JobRun[], netPads: WorldItem[], width: number): boolean {
	return padsConnectedByCopper(rs.map(q => q.cur), netPads.map(it => it.shape), width);
}
