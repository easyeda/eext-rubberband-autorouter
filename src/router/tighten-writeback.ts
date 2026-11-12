/**
 * 拉紧结果回写（纯逻辑，不引用全局 `eda`，可离线单测）
 *
 * 真机实测（lceda-pro 4.1.99）推翻了旧的「先建后删 + 按 id 删旧」方案：
 *  - EDA 在 create 时会就地合并/拆分同网铜：共线首尾相接/重叠合并成一根（新 id）、
 *    子段被吞、近共线（转角 < 1°）两线丢掉公共顶点、交叉/T 接/穿过孔处拆开（全部换新 id）、
 *    重复弧新弧被吞；
 *  - delete(ids) 永远返回 true（未知/已删 id 也一样），返回值不含任何信息；
 *  - 弧读回时端点按 0.1mil、扫角按 0.1° 取整（存储是精确的）。
 * 所以「先建新、再按存下的 id 删旧」会：新图元与旧铜重合时被吞/合并，删旧时连带删掉 → 洞；
 * 邻段 create 拆分/合并后存下的 id 失效 → 删旧静默无效 → 新旧并存。
 *
 * 新方案（不变式）：
 *  A. 永不信任存下的图元 id：每次动一段之前，现读该网的线/弧，按几何（同层、采样点都落在
 *     模型路径 TOL_MIL 内）找出当前承载这段铜的画布图元；回写器只记每段「当前模型几何」。
 *  B. 先删旧、再建新；任何创建失败/校验失败 → 按几何删掉新几何、重建旧几何、再校验并汇报
 *     （恢复失败明确报网络与坐标）。第一次回写失败即中止整个拉紧。
 *  C. 防合并：写出的相邻两线转角 < MERGE_GUARD_DEG 时，偏差 ≤ PREMERGE_DEV_MIL 就自己先合并，
 *     否则在顶点插一段很短的圆弧隔开（实测：两线之间夹弧，EDA 不再合并）。
 *  D. 写后校验：现读该网，新模型每个图元都被同网同层同线宽画布铜覆盖、旧路径上不再有
 *     「只属于旧路径」的铜；不符即视为写失败（走 B）。
 *  E. 弧精度：弧端点吸附 0.1mil 网格、扫角取 0.1° 整倍数（|扫角| < 0.1° 改写成线），相邻线
 *     端点精确落在吸附后的弧端点上 → 读回 == 写入；< 0.1mil 的线不创建（EDA 会静默丢弃）。
 * 每个写出图元相对求解器图元的位置误差 ≤ 0.1mil。
 *
 * 画布 API 由调用方注入（WritebackApi），本模块只做几何、编排与记账。
 */

import type { RenderedPrim } from './planning-engine';
import { WRITER_ERR } from './tighten-core';

export { WRITER_ERR };

export interface Pt { x: number; y: number }

/**
 * 注入的画布接口。
 * - create*：成功返回新图元 id；失败抛异常或返回空串（id 只用来判断成功，从不存下来删）；
 * - delete*：返回值被忽略（真机永远返回 true），删没删掉由之后的现读判定；
 * - list*(net)：现读该网全部线/弧（原始 EDA 对象，字段见 canvasLineToRendered / canvasArcToRendered）。
 */
export interface WritebackApi {
	createLine(net: string, layer: number, sx: number, sy: number, ex: number, ey: number, width: number): Promise<string>;
	createArc(net: string, layer: number, sx: number, sy: number, ex: number, ey: number, degrees: number, width: number): Promise<string>;
	deleteLines(ids: string[]): Promise<unknown>;
	deleteArcs(ids: string[]): Promise<unknown>;
	listLines(net: string): Promise<any[]>;
	listArcs(net: string): Promise<any[]>;
}

/** 几何匹配容差（mil）：覆盖弧读回 0.1mil / 0.1° 取整带来的偏差 */
export const TOL_MIL = 0.3;
/** 弧端点网格（mil）与扫角步长（度）：与 EDA 读回取整一致 */
export const GRID_MIL = 0.1;
export const SWEEP_STEP_DEG = 0.1;
/** 短于此值的线不创建（真机 0.04mil 线返回 id 但不落盘） */
export const MIN_LINE_MIL = 0.1;
/** 相邻两线转角低于此值（度）视为「EDA 会丢顶点」（实测阈值 1°，留余量） */
export const MERGE_GUARD_DEG = 1.5;
/** 被丢顶点偏离弦线不超过此值（mil）时自己先合并 */
export const PREMERGE_DEV_MIL = 0.02;
/** 防合并隔离弧的切线长（mil，上限） */
export const FILLET_T_MIL = 2;
/** 「新几何与当前几何相同」的容差（mil）：相同则不碰画布 */
export const NOOP_TOL_MIL = 0.01;
/** 链内相邻图元端点视为同一顶点的距离（mil；求解器链内间隙 ≤ 0.01） */
export const JOIN_TOL_MIL = 0.02;
/** 判定「线宽相同」的容差（mil） */
export const WIDTH_EPS = 0.01;
/** 读回扫角绝对值低于此值（度）= EDA 读回为 0：按直线处理 */
export const ZERO_SWEEP_DEG = 0.05;
/** 连续覆盖检查的最小细分长度（mil）：比它更短的区间不再细分（漏判的超差 ≤ 其一半） */
const COVER_EPS_MIL = 0.01;
/** emitError 上界的精度（mil）：返回值 ≥ 真实最大误差，且至多比它大这么多 */
const EMIT_ERR_EPS_MIL = 0.002;

/**
 * 画布线宽的统一口径：读到的 lineWidth 不是正的有限数（0 / 缺失 / NaN）时取 fallback（调用方给 DRC 默认线宽）。
 * 拉紧读侧建模型（topological-router）与回写现读（readNetCopper）必须用同一口径，否则同一根铜会被判成「线宽不同 = 缺铜」。
 */
export function widthOrDefault(raw: unknown, fallback: number): number {
	const w = Number(raw);
	return Number.isFinite(w) && w > 0 ? w : fallback;
}

// ─── 基础几何 ───────────────────────────────────────────────────

const finite = (...v: number[]): boolean => v.every(n => Number.isFinite(n));
const dist = (a: Pt, b: Pt): number => Math.hypot(a.x - b.x, a.y - b.y);
const TAU = Math.PI * 2;
/** 吸附到 0.1mil 网格（用 *10/10 而不是 *0.1，避免 100.30000000000001） */
export const snapGrid = (v: number): number => Math.round(v * 10) / 10;
const snapPt = (p: Pt): Pt => ({ x: snapGrid(p.x), y: snapGrid(p.y) });
const onGrid = (p: Pt): boolean => Math.abs(p.x - snapGrid(p.x)) < 1e-9 && Math.abs(p.y - snapGrid(p.y)) < 1e-9;
const quantDeg = (d: number): number => Math.round(d * 10) / 10;

/** 图元两端点（弧由 cx/cy/r/startAngle/sweep 反算） */
export function primEnds(p: RenderedPrim): [Pt, Pt] {
	if (p.kind === 'line') return [{ x: p.x1!, y: p.y1! }, { x: p.x2!, y: p.y2! }];
	const sw = p.sweep ?? 0;
	return [
		{ x: p.cx! + p.r! * Math.cos(p.startAngle!), y: p.cy! + p.r! * Math.sin(p.startAngle!) },
		{ x: p.cx! + p.r! * Math.cos(p.startAngle! + sw), y: p.cy! + p.r! * Math.sin(p.startAngle! + sw) },
	];
}

export function primLength(p: RenderedPrim): number {
	if (p.kind === 'line') return Math.hypot(p.x2! - p.x1!, p.y2! - p.y1!);
	return Math.abs((p.r ?? 0) * (p.sweep ?? 0));
}

function primFinite(p: RenderedPrim): boolean {
	return p.kind === 'line'
		? finite(p.x1!, p.y1!, p.x2!, p.y2!)
		: finite(p.cx!, p.cy!, p.r!, p.startAngle!, p.sweep!) && p.r! > 0;
}

function pointAt(p: RenderedPrim, t: number): Pt {
	if (p.kind === 'line') return { x: p.x1! + (p.x2! - p.x1!) * t, y: p.y1! + (p.y2! - p.y1!) * t };
	const a = p.startAngle! + p.sweep! * t;
	return { x: p.cx! + p.r! * Math.cos(a), y: p.cy! + p.r! * Math.sin(a) };
}

function segDist(q: Pt, a: Pt, b: Pt): number {
	const dx = b.x - a.x, dy = b.y - a.y;
	const L2 = dx * dx + dy * dy;
	if (L2 < 1e-18) return dist(q, a);
	const t = Math.max(0, Math.min(1, ((q.x - a.x) * dx + (q.y - a.y) * dy) / L2));
	return Math.hypot(q.x - a.x - dx * t, q.y - a.y - dy * t);
}

/** 点到图元（线段 / 圆弧）的距离 */
export function distToPrim(p: RenderedPrim, q: Pt): number {
	if (p.kind === 'line') return segDist(q, { x: p.x1!, y: p.y1! }, { x: p.x2!, y: p.y2! });
	const phi = Math.atan2(q.y - p.cy!, q.x - p.cx!);
	if (inArcSpan(p, phi)) return Math.abs(Math.hypot(q.x - p.cx!, q.y - p.cy!) - p.r!);
	const [s, e] = primEnds(p);
	return Math.min(dist(q, s), dist(q, e));
}

function inArcSpan(p: RenderedPrim, phi: number): boolean {
	const s = p.sweep!;
	let rel = phi - p.startAngle!;
	if (s >= 0) {
		rel = ((rel % TAU) + TAU) % TAU;
		return rel <= s + 1e-12;
	}
	rel = -((((-rel) % TAU) + TAU) % TAU);
	return rel >= s - 1e-12;
}

interface BBox { x0: number; y0: number; x1: number; y1: number }

function bboxOf(p: RenderedPrim): BBox {
	const [s, e] = primEnds(p);
	const b: BBox = { x0: Math.min(s.x, e.x), y0: Math.min(s.y, e.y), x1: Math.max(s.x, e.x), y1: Math.max(s.y, e.y) };
	if (p.kind === 'arc') {
		for (let k = -4; k <= 4; k++) {
			const a = k * Math.PI / 2;
			if (!inArcSpan(p, a)) continue;
			const x = p.cx! + p.r! * Math.cos(a), y = p.cy! + p.r! * Math.sin(a);
			b.x0 = Math.min(b.x0, x); b.x1 = Math.max(b.x1, x); b.y0 = Math.min(b.y0, y); b.y1 = Math.max(b.y1, y);
		}
	}
	return b;
}

const bbNear = (a: BBox, b: BBox, tol: number): boolean =>
	!(a.x0 > b.x1 + tol || b.x0 > a.x1 + tol || a.y0 > b.y1 + tol || b.y0 > a.y1 + tol);
const bbHas = (b: BBox, q: Pt, tol: number): boolean =>
	q.x >= b.x0 - tol && q.x <= b.x1 + tol && q.y >= b.y0 - tol && q.y <= b.y1 + tol;

/** 沿图元均匀采样（含两端），步长 ≤ step，最多 maxN 段 */
function samplePrim(p: RenderedPrim, step = 1, maxN = 400): Pt[] {
	const n = Math.max(2, Math.min(maxN, Math.ceil(primLength(p) / step)));
	const out: Pt[] = [];
	for (let k = 0; k <= n; k++) out.push(pointAt(p, k / n));
	return out;
}

/*
 * 连续检查（替代点采样）：g 是「点到某个集合的距离」（可在某个 > tol 的值处截断），对位置 1-Lipschitz；
 * pointAt 按弧长匀速（线、弧都是），所以 h(s) = g(pointAt(s/L)) 对弧长 s 也是 1-Lipschitz：
 * 区间 [a,b] 上 max h ≤ (h(a) + h(b) + (b − a)) / 2。据此二分细分，不会漏掉两个采样点之间的洞。
 */

/**
 * 单个图元 c 到 m 的子段 [ta, tb]（参数 0..1）上各点距离的精确上界；给不出时返回 Infinity。用来少做细分：
 *  - 都是线：点到线段的距离沿直线是凸函数 ⇒ 子段上的最大值在两端；
 *  - 都是弧且同心（圆心差 ≤ 1e-6）、子弧整个落在 c 的角度范围内：距离 ≤ |Δr| + 圆心差。
 * 其它组合给不出（交给 Lipschitz 细分）。不看层（调用方自己筛）。
 */
function subBound(c: RenderedPrim, m: RenderedPrim, ta: number, tb: number): number {
	if (m.kind === 'line' && c.kind === 'line') return Math.max(distToPrim(c, pointAt(m, ta)), distToPrim(c, pointAt(m, tb)));
	if (m.kind !== 'arc' || c.kind !== 'arc') return Infinity;
	const dcen = Math.hypot(c.cx! - m.cx!, c.cy! - m.cy!);
	if (dcen > 1e-6) return Infinity;
	const delta = m.sweep! * (tb - ta);
	const s = c.sweep! >= 0 ? 1 : -1;
	// 子弧按 c 的方向走：起点取同向的那一端
	const th0 = Math.sign(delta) === s || delta === 0 ? m.startAngle! + m.sweep! * ta : m.startAngle! + m.sweep! * tb;
	const rel = (((s * (th0 - c.startAngle!)) % TAU) + TAU) % TAU;
	if (rel + Math.abs(delta) > Math.abs(c.sweep!) + 1e-9) return Infinity;
	return dcen + Math.abs(c.r! - m.r!);
}

/**
 * 沿图元找第一个 g > tol 的点；整条都 ≤ tol（细分到 COVER_EPS_MIL 为止）→ null。
 * bound(ta, tb)：可选的精确上界（参数 0..1），≤ tol 即该子段已确定被覆盖。
 */
function firstOver(p: RenderedPrim, g: (q: Pt) => number, tol: number, bound?: (ta: number, tb: number) => number): Pt | null {
	const L = primLength(p);
	const at = (s: number): Pt => pointAt(p, L > 0 ? s / L : 0);
	const ga = g(at(0));
	if (ga > tol) return at(0);
	if (L <= 0) return null;
	const gb = g(at(L));
	if (gb > tol) return at(L);
	const stack: Array<[number, number, number, number]> = [[0, L, ga, gb]];
	while (stack.length) {
		const [a, b, fa, fb] = stack.pop()!;
		if ((fa + fb + (b - a)) / 2 <= tol || b - a <= COVER_EPS_MIL) continue;
		if (bound && bound(a / L, b / L) <= tol) continue;
		const m = (a + b) / 2;
		const q = at(m);
		const fm = g(q);
		if (fm > tol) return q;
		// 先压右半、后压左半：先查靠近起点的一侧
		stack.push([m, b, fm, fb], [a, m, fa, fm]);
	}
	return null;
}

/** 沿图元求 g 的最大值的上界（≥ 真值，至多大 eps；g 不截断）；bound 同 firstOver */
function maxAlong(p: RenderedPrim, g: (q: Pt) => number, eps: number, bound?: (ta: number, tb: number) => number): number {
	const L = primLength(p);
	const at = (s: number): Pt => pointAt(p, L > 0 ? s / L : 0);
	const ga = g(at(0));
	if (L <= 0) return ga;
	const gb = g(at(L));
	let best = Math.max(ga, gb);
	let ub = best;
	const stack: Array<[number, number, number, number]> = [[0, L, ga, gb]];
	while (stack.length) {
		const [a, b, fa, fb] = stack.pop()!;
		let bd = (fa + fb + (b - a)) / 2;
		if (bd > best + eps && bound) bd = Math.min(bd, bound(a / L, b / L));
		if (bd <= best + eps) { ub = Math.max(ub, bd); continue; }
		const m = (a + b) / 2;
		const fm = g(at(m));
		best = Math.max(best, fm);
		stack.push([a, m, fa, fm], [m, b, fm, fb]);
	}
	return Math.max(ub, best);
}

function reversePrim(p: RenderedPrim): RenderedPrim {
	if (p.kind === 'line') return { ...p, x1: p.x2, y1: p.y2, x2: p.x1, y2: p.y1 };
	return { ...p, startAngle: p.startAngle! + p.sweep!, sweep: -p.sweep! };
}

/** 有向转角（弧度，逆时针为正）：P→V 与 V→Q */
function turnAngle(P: Pt, V: Pt, Q: Pt): number {
	const ax = V.x - P.x, ay = V.y - P.y, bx = Q.x - V.x, by = Q.y - V.y;
	return Math.atan2(ax * by - ay * bx, ax * bx + ay * by);
}

// ─── 画布图元 ↔ 引擎图元 ────────────────────────────────────────

/**
 * 两点式弧（EDA 口径：起点/终点/arcAngle°，正角 = 圆心在弦左侧）→ 引擎弧（cx/cy/r/startAngle/sweep）。
 * 与 EDA 弧的换算已用区域命中实测核对（正负扫角都对）。
 */
export function arcFromChord(
	net: string, layer: number, sx: number, sy: number, ex: number, ey: number, deg: number,
): RenderedPrim | null {
	if (!finite(sx, sy, ex, ey, deg)) return null;
	const sweep = deg * Math.PI / 180;
	const L = Math.hypot(ex - sx, ey - sy);
	if (!sweep || Math.abs(Math.sin(sweep / 2)) < 1e-6 || L < 1e-6) return null;
	const R = L / (2 * Math.abs(Math.sin(sweep / 2)));
	const sgn = sweep > 0 ? 1 : -1;
	const h = sgn * R * Math.cos(sweep / 2);
	const nx = -(ey - sy) / L, ny = (ex - sx) / L;
	const cx = (sx + ex) / 2 + h * nx, cy = (sy + ey) / 2 + h * ny;
	const a0 = Math.atan2(sy - cy, sx - cx);
	return { kind: 'arc', net, layer, cx, cy, r: R, startAngle: a0, sweep };
}

/**
 * 画布弧对象 → 引擎图元。读回扫角为 0（真机 |arcAngle| < 0.05° 读回即 0）时按端点间直线处理
 *（而不是丢掉——丢掉会让该段在 run 链上断开）。端点重合/字段无效 → null。
 */
export function canvasArcToRendered(a: any, net: string): RenderedPrim | null {
	const layer: number = a?.layer ?? a?.layerId ?? 1;
	const sx = a?.startX, sy = a?.startY, ex = a?.endX, ey = a?.endY;
	if (typeof sx !== 'number' || typeof sy !== 'number' || typeof ex !== 'number' || typeof ey !== 'number') return null;
	const deg = Number(a.arcAngle ?? a.sweep ?? 0);
	if (!finite(sx, sy, ex, ey, deg) || Math.hypot(ex - sx, ey - sy) < 1e-6) return null;
	if (Math.abs(deg) < ZERO_SWEEP_DEG) return { kind: 'line', net, layer, x1: sx, y1: sy, x2: ex, y2: ey };
	return arcFromChord(net, layer, sx, sy, ex, ey, deg);
}

/** 画布线对象 → 引擎线；坐标无效 → null */
export function canvasLineToRendered(l: any, net: string): RenderedPrim | null {
	if (!lineEndpointsUsable(l)) return null;
	const layer: number = l.layer ?? l.layerId ?? 1;
	return { kind: 'line', net, layer, x1: l.startX ?? l.x, y1: l.startY ?? l.y, x2: l.endX, y2: l.endY };
}

// ─── 写出规划（C + E）────────────────────────────────────────────

export type CreateCall =
	| { kind: 'line'; layer: number; sx: number; sy: number; ex: number; ey: number }
	| { kind: 'arc'; layer: number; sx: number; sy: number; ex: number; ey: number; deg: number };

/** 创建调用 → 它在画布上读回的模型几何（弧端点/扫角已在网格上，读回 == 写入） */
export function callToPrim(c: CreateCall, net: string): RenderedPrim | null {
	if (c.kind === 'line') return { kind: 'line', net, layer: c.layer, x1: c.sx, y1: c.sy, x2: c.ex, y2: c.ey };
	return arcFromChord(net, c.layer, c.sx, c.sy, c.ex, c.ey, c.deg);
}

/** 模型图元 → 创建调用（不做防合并处理；弧端点/扫角吸附到读回网格） */
export function primToCall(p: RenderedPrim, layer: number): CreateCall | null {
	if (!primFinite(p)) return null;
	const [s, e] = primEnds(p);
	if (p.kind === 'line') {
		if (dist(s, e) < MIN_LINE_MIL) return null;
		return { kind: 'line', layer, sx: s.x, sy: s.y, ex: e.x, ey: e.y };
	}
	const deg = quantDeg(p.sweep! * 180 / Math.PI);
	const S = snapPt(s), E = snapPt(e);
	if (dist(S, E) < MIN_LINE_MIL) return null;
	if (Math.abs(deg) < SWEEP_STEP_DEG - 1e-9) return { kind: 'line', layer, sx: S.x, sy: S.y, ex: E.x, ey: E.y };
	return { kind: 'arc', layer, sx: S.x, sy: S.y, ex: E.x, ey: E.y, deg };
}

interface Chain { prims: RenderedPrim[]; closed: boolean }

/**
 * 把一批图元（任意顺序/方向，可含多条链）按端点连成有向链。
 * 只穿过「恰有两个本批端点、且附近没有其它同网铜（blockers）」的顶点——
 * 那正是 EDA 可能合并的顶点；T 接点/挨着别的铜的顶点是链的边界。
 */
function buildChains(prims: RenderedPrim[], blockers: RenderedPrim[]): Chain[] {
	// 微图元（两端落进同一顶点簇）不成边：否则它把顶点度数变成 3、链在此断开，防合并处理不到这个顶点
	const ps = prims.filter(p => { const [s, e] = primEnds(p); return primLength(p) > 1e-9 && dist(s, e) > JOIN_TOL_MIL; });
	const nodes: Pt[] = [];
	const nodeOf = (q: Pt): number => {
		for (let i = 0; i < nodes.length; i++) if (dist(nodes[i], q) <= JOIN_TOL_MIL) return i;
		nodes.push(q);
		return nodes.length - 1;
	};
	const ends = ps.map(p => { const [s, e] = primEnds(p); return [nodeOf(s), nodeOf(e)] as [number, number]; });
	const inc = new Map<number, number[]>();
	ends.forEach(([a, b], i) => {
		inc.set(a, [...(inc.get(a) ?? []), i]);
		if (b !== a) inc.set(b, [...(inc.get(b) ?? []), i]);
	});
	const bbs = blockers.map(bk => ({ p: bk, bb: bboxOf(bk) }));
	const blocked = (n: number): boolean => bbs.some(({ p, bb }) => bbHas(bb, nodes[n], TOL_MIL) && distToPrim(p, nodes[n]) <= TOL_MIL);
	const pass = new Map<number, boolean>();
	const passThrough = (n: number): boolean => {
		let v = pass.get(n);
		if (v === undefined) { v = (inc.get(n)?.length ?? 0) === 2 && !blocked(n); pass.set(n, v); }
		return v;
	};
	const used = new Set<number>();
	const chains: Chain[] = [];
	const walk = (startNode: number, first: number): Chain => {
		const out: RenderedPrim[] = [];
		let cur = startNode, i = first;
		for (;;) {
			used.add(i);
			const fwd = ends[i][0] === cur;
			out.push(fwd ? ps[i] : reversePrim(ps[i]));
			cur = fwd ? ends[i][1] : ends[i][0];
			if (cur === startNode && out.length > 1 && passThrough(cur)) return { prims: out, closed: true };
			if (!passThrough(cur)) break;
			const nx = (inc.get(cur) ?? []).find(j => !used.has(j));
			if (nx === undefined) break;
			i = nx;
		}
		return { prims: out, closed: false };
	};
	// 先从边界顶点出发（开链），再处理剩下的环
	for (let n = 0; n < nodes.length; n++) {
		if (passThrough(n)) continue;
		for (const i of inc.get(n) ?? []) if (!used.has(i)) chains.push(walk(n, i));
	}
	for (let i = 0; i < ps.length; i++) if (!used.has(i)) chains.push(walk(ends[i][0], i));
	return chains;
}

/** 链上工作元素：顶点 verts[i] → verts[i+1] 之间的线或弧 */
type WEl = { kind: 'line'; absorbed: Pt[] } | { kind: 'arc'; cx: number; cy: number; r: number; sweep: number; fillet?: boolean };
type OutEl = { kind: 'line'; s: Pt; e: Pt } | { kind: 'arc'; s: Pt; e: Pt; deg: number };

const GUARD_RAD = MERGE_GUARD_DEG * Math.PI / 180;

/** 链 → 工作表示（顶点优先取线的精确坐标） */
function chainToWork(prims: RenderedPrim[], closed: boolean): { verts: Pt[]; els: WEl[] } {
	const m = prims.length;
	const verts: Pt[] = [];
	for (let i = 0; i <= m; i++) {
		if (i === 0) verts.push(primEnds(prims[0])[0]);
		else if (i === m) verts.push(primEnds(prims[m - 1])[1]);
		else verts.push(prims[i].kind === 'line' ? primEnds(prims[i])[0] : primEnds(prims[i - 1])[1]);
	}
	if (closed && m) verts[m] = { ...verts[0] };
	const els: WEl[] = prims.map(p => (p.kind === 'line'
		? { kind: 'line', absorbed: [] }
		: { kind: 'arc', cx: p.cx!, cy: p.cy!, r: p.r!, sweep: p.sweep! }));
	return { verts, els };
}

/** 顶点 j（els[j-1] 与 els[j] 之间）两侧都是线时的有向转角，否则 null */
function lineTurnAt(verts: Pt[], els: WEl[], j: number): number | null {
	if (j <= 0 || j >= els.length) return null;
	if (els[j - 1].kind !== 'line' || els[j].kind !== 'line') return null;
	return turnAngle(verts[j - 1], verts[j], verts[j + 1]);
}

/** C-1：近共线且被丢顶点偏差 ≤ PREMERGE_DEV_MIL → 自己先合并（零形变地抢在 EDA 前面） */
function preMerge(verts: Pt[], els: WEl[]): void {
	for (let changed = true; changed;) {
		changed = false;
		for (let j = 1; j < els.length; j++) {
			const th = lineTurnAt(verts, els, j);
			if (th === null || Math.abs(th) >= GUARD_RAD) continue;
			const P = verts[j - 1], Q = verts[j + 1];
			const A = els[j - 1] as Extract<WEl, { kind: 'line' }>, B = els[j] as Extract<WEl, { kind: 'line' }>;
			const pts = [verts[j], ...A.absorbed, ...B.absorbed];
			if (pts.some(q => segDist(q, P, Q) > PREMERGE_DEV_MIL)) continue;
			A.absorbed = pts;
			verts.splice(j, 1);
			els.splice(j, 1);
			changed = true;
			break;
		}
	}
}

/** C-2：其余近共线顶点插入短隔离弧（EDA 不合并「线-弧-线」） */
function insertFillets(verts: Pt[], els: WEl[]): void {
	for (let j = 1; j < els.length; j++) {
		const th = lineTurnAt(verts, els, j);
		if (th === null || th === 0 || Math.abs(th) >= GUARD_RAD) continue;
		const P = verts[j - 1], V = verts[j], Q = verts[j + 1];
		const la = dist(P, V), lb = dist(V, Q);
		const t = Math.min(FILLET_T_MIL, la / 3, lb / 3);
		if (t < 0.15) continue;   // 两侧线太短，隔不开：交给写后校验（D）
		const ua = { x: (V.x - P.x) / la, y: (V.y - P.y) / la }, ub = { x: (Q.x - V.x) / lb, y: (Q.y - V.y) / lb };
		const S = { x: V.x - ua.x * t, y: V.y - ua.y * t }, E = { x: V.x + ub.x * t, y: V.y + ub.y * t };
		const arc = arcFromChord('', 0, S.x, S.y, E.x, E.y, th * 180 / Math.PI);
		if (!arc) continue;
		verts.splice(j, 1, S, E);
		els.splice(j, 0, { kind: 'arc', cx: arc.cx!, cy: arc.cy!, r: arc.r!, sweep: arc.sweep!, fillet: true });
		j++;
	}
}

/** 点到原链的最近距离 */
function distToChain(orig: Array<{ p: RenderedPrim; bb: BBox }>, q: Pt): number {
	let best = Infinity;
	for (const o of orig) {
		if (best < Infinity && !bbHas(o.bb, q, best)) continue;
		best = Math.min(best, distToPrim(o.p, q));
	}
	return best;
}

/** 一个写出弧相对原链的最大位置误差 */
function arcError(orig: Array<{ p: RenderedPrim; bb: BBox }>, s: Pt, e: Pt, deg: number): number {
	const a = arcFromChord('', 0, s.x, s.y, e.x, e.y, deg);
	if (!a) return Infinity;
	let m = 0;
	for (const q of samplePrim(a, 0.5, 64)) m = Math.max(m, distToChain(orig, q));
	return m;
}

/**
 * E：弧端点吸附网格、扫角取 0.1° 整倍数；与弧相邻的线端点精确落在吸附后的弧端点上；
 * < 0.1mil 的线/弦不创建（下一图元从当前游标接上）。
 * 弧终点两种取法：(i) 终点就近吸附（扫角量化引入的弦高误差 ≈ 弦长/8·Δθ）；
 * (ii) 终点沿原圆滑到量化扫角处再吸附（大弧更准，只用于非末元素——末端是固定点）。
 */
function quantize(verts: Pt[], els: WEl[], closed: boolean, orig: Array<{ p: RenderedPrim; bb: BBox }>): OutEl[] {
	const n = els.length;
	const out: OutEl[] = [];
	if (!n) return out;
	// 极小弧（弦 < 1mil 且弦高 ≤ 0.02mil）按线处理：吸附网格反而会把它挪出 0.1mil 以上
	for (let i = 0; i < n; i++) {
		const el = els[i];
		if (el.kind !== 'arc' || el.fillet) continue;
		const ch = dist(verts[i], verts[i + 1]);
		// 已是可精确读回的弧（端点在网格上、扫角为 0.1° 整倍数）保持原样：保证重复写出是 no-op
		const dg = el.sweep * 180 / Math.PI;
		if (onGrid(verts[i]) && onGrid(verts[i + 1]) && Math.abs(dg * 10 - Math.round(dg * 10)) < 1e-6) continue;
		if (ch < 1 && ch / 2 * Math.tan(Math.abs(el.sweep) / 4) <= PREMERGE_DEV_MIL) els[i] = { kind: 'line', absorbed: [] };
	}
	let cur: Pt = els[0].kind === 'arc' ? snapPt(verts[0]) : { ...verts[0] };
	const pushLine = (end: Pt): void => {
		if (dist(cur, end) < MIN_LINE_MIL) return;
		out.push({ kind: 'line', s: { ...cur }, e: end });
		cur = end;
	};
	const ensureGrid = (): void => {
		for (let guard = 0; !onGrid(cur) && guard < 8; guard++) {
			const g = snapPt(cur);
			const last = out[out.length - 1];
			if (last && last.kind === 'line') {
				last.e = g;
				if (dist(last.s, last.e) < MIN_LINE_MIL) { out.pop(); cur = last.s; continue; }
			}
			cur = g;
		}
		if (!onGrid(cur)) cur = snapPt(cur);
	};
	for (let i = 0; i < n; i++) {
		const el = els[i];
		const isLast = i === n - 1;
		const nextArc = isLast ? (closed && els[0].kind === 'arc') : els[i + 1].kind === 'arc';
		const endRaw = verts[i + 1];
		const endFor = (): Pt => (nextArc ? snapPt(endRaw) : { ...endRaw });
		if (el.kind === 'line') { pushLine(endFor()); continue; }
		ensureGrid();
		const a0 = Math.atan2(cur.y - el.cy, cur.x - el.cx);
		const a1 = Math.atan2(endRaw.y - el.cy, endRaw.x - el.cx);
		let d = a1 - a0;
		d += TAU * Math.round((el.sweep - d) / TAU);
		let deg = quantDeg(d * 180 / Math.PI);
		if (el.fillet && Math.abs(deg) < SWEEP_STEP_DEG) deg = (el.sweep >= 0 ? 1 : -1) * SWEEP_STEP_DEG;
		if (Math.abs(deg) < SWEEP_STEP_DEG - 1e-9) { pushLine(endFor()); continue; }
		const E1 = snapPt(endRaw);
		let best = { e: E1, err: arcError(orig, cur, E1, deg) };
		if (best.err > 0.05 && !isLast) {
			const ar = a0 + deg * Math.PI / 180;
			const E2 = snapPt({ x: el.cx + el.r * Math.cos(ar), y: el.cy + el.r * Math.sin(ar) });
			const err2 = arcError(orig, cur, E2, deg);
			if (err2 < best.err) best = { e: E2, err: err2 };
		}
		// (iii) 终点固定（末元素）时改滑起点：前一元素是线（切线方向）才行，线的终点随之移到新起点
		const prev = out[out.length - 1];
		if (best.err > 0.05 && prev && prev.kind === 'line') {
			const ae = Math.atan2(E1.y - el.cy, E1.x - el.cx) - deg * Math.PI / 180;
			const S3 = snapPt({ x: el.cx + el.r * Math.cos(ae), y: el.cy + el.r * Math.sin(ae) });
			if (dist(prev.s, S3) >= MIN_LINE_MIL) {
				const ln: RenderedPrim = { kind: 'line', net: '', layer: 0, x1: prev.s.x, y1: prev.s.y, x2: S3.x, y2: S3.y };
				const err3 = Math.max(arcError(orig, S3, E1, deg), ...samplePrim(ln, 0.5, 64).map(q => distToChain(orig, q)));
				if (err3 < best.err) { prev.e = S3; cur = S3; best = { e: E1, err: err3 }; }
			}
		}
		if (dist(cur, best.e) < MIN_LINE_MIL) continue;
		out.push({ kind: 'arc', s: { ...cur }, e: best.e, deg });
		cur = best.e;
	}
	// 末尾元素因过短被丢：把最后一根线的终点拉到链端点（链端点是固定点，不能悬空 < 0.1mil）
	const endPt = closed && els[0].kind === 'arc' ? snapPt(verts[n]) : verts[n];
	const last = out[out.length - 1];
	if (last && last.kind === 'line' && dist(cur, endPt) > 1e-9 && dist(last.s, endPt) >= MIN_LINE_MIL) last.e = { ...endPt };
	return out;
}

/** 写出序列里是否还有「两线相接、转角 < 防合并阈值」的顶点（含闭链首尾） */
function hasMergeRisk(out: OutEl[], closed: boolean): boolean {
	const pairs: Array<[OutEl, OutEl]> = [];
	for (let i = 1; i < out.length; i++) pairs.push([out[i - 1], out[i]]);
	if (closed && out.length > 2) pairs.push([out[out.length - 1], out[0]]);
	return pairs.some(([a, b]) => a.kind === 'line' && b.kind === 'line' && dist(a.e, b.s) < 1e-9 &&
		Math.abs(turnAngle(a.s, a.e, b.e)) < GUARD_RAD);
}

function outToWork(out: OutEl[]): { verts: Pt[]; els: WEl[] } {
	const verts: Pt[] = out.length ? [{ ...out[0].s }] : [];
	const els: WEl[] = [];
	for (const o of out) {
		verts.push({ ...o.e });
		if (o.kind === 'line') els.push({ kind: 'line', absorbed: [] });
		else {
			const a = arcFromChord('', 0, o.s.x, o.s.y, o.e.x, o.e.y, o.deg)!;
			els.push({ kind: 'arc', cx: a.cx!, cy: a.cy!, r: a.r!, sweep: a.sweep! });
		}
	}
	return { verts, els };
}

/** 闭链：把起点转到一个「安全」顶点（挨着弧，或两线转角 ≥ 阈值），首尾接缝处就不会被 EDA 合并 */
function rotateClosed(verts: Pt[], els: WEl[]): void {
	const n = els.length;
	if (n < 3) return;
	for (let k = 0; k < n; k++) {
		const prev = els[(k - 1 + n) % n], next = els[k];
		const safe = prev.kind === 'arc' || next.kind === 'arc' ||
			Math.abs(turnAngle(verts[(k - 1 + n) % n], verts[k], verts[k + 1])) >= GUARD_RAD;
		if (!safe) continue;
		if (k === 0) return;
		const vs = verts.slice(0, n);   // 去掉重复的末顶点
		const rv = [...vs.slice(k), ...vs.slice(0, k)];
		rv.push({ ...rv[0] });
		const re = [...els.slice(k), ...els.slice(0, k)];
		verts.splice(0, verts.length, ...rv);
		els.splice(0, els.length, ...re);
		return;
	}
}

function emitChain(chain: Chain): OutEl[] {
	const orig = chain.prims.map(p => ({ p, bb: bboxOf(p) }));
	let { verts, els } = chainToWork(chain.prims, chain.closed);
	if (chain.closed) rotateClosed(verts, els);
	let out: OutEl[] = [];
	for (let iter = 0; iter < 4; iter++) {
		preMerge(verts, els);
		insertFillets(verts, els);
		if (chain.closed) rotateClosed(verts, els);
		out = quantize(verts, els, chain.closed, orig);
		if (!hasMergeRisk(out, chain.closed)) break;
		({ verts, els } = outToWork(out));
	}
	return out;
}

export interface EmitPlan {
	calls: CreateCall[];
	/** 每个调用写出后在画布上读回的几何（与 calls 一一对应） */
	model: RenderedPrim[];
}

/**
 * 求解器图元 → 画布创建调用序列（含防合并 C 与精度处理 E）。
 * blockers：同网同层、本批之外的现存铜（判断哪些顶点 EDA 不会合并）。
 * 任何图元非有限数 / 层不一致 → null；全部退化（< 0.1mil）→ 空计划。
 */
export function planEmit(prims: RenderedPrim[], net: string, layer: number, blockers: RenderedPrim[] = []): EmitPlan | null {
	for (const p of prims) {
		if ((p.layer ?? layer) !== layer) return null;
		if (p.kind !== 'line' && p.kind !== 'arc') return null;
		if (!primFinite(p)) return null;
	}
	const norm = prims.map(p => ({ ...p, net, layer }));
	const calls: CreateCall[] = [];
	const model: RenderedPrim[] = [];
	for (const ch of buildChains(norm, blockers.filter(b => b.layer === layer))) {
		for (const o of emitChain(ch)) {
			const c: CreateCall = o.kind === 'line'
				? { kind: 'line', layer, sx: o.s.x, sy: o.s.y, ex: o.e.x, ey: o.e.y }
				: { kind: 'arc', layer, sx: o.s.x, sy: o.s.y, ex: o.e.x, ey: o.e.y, deg: o.deg };
			const m = callToPrim(c, net);
			if (!m) continue;
			calls.push(c);
			model.push(m);
		}
	}
	return { calls, model };
}

/**
 * 写出几何相对原图元的最大位置误差（mil；双向 Hausdorff：写出点到原链 + 原链点到写出）。
 * 连续求上界（不是点采样）：返回值 ≥ 真实误差，至多大 EMIT_ERR_EPS_MIL——回写器据此对照 WRITER_ERR 预算。
 */
export function emitError(orig: RenderedPrim[], emitted: RenderedPrim[]): number {
	const o = orig.map(p => ({ p, bb: bboxOf(p) }));
	const e = emitted.map(p => ({ p, bb: bboxOf(p) }));
	if (!o.length || !e.length) return o.length === e.length ? 0 : Infinity;
	let m = 0;
	const along = (x: RenderedPrim, to: Array<{ p: RenderedPrim; bb: BBox }>): number => maxAlong(
		x, q => distToChain(to, q), EMIT_ERR_EPS_MIL,
		(ta, tb) => { let b = Infinity; for (const t of to) b = Math.min(b, subBound(t.p, x, ta, tb)); return b; },
	);
	for (const x of emitted) m = Math.max(m, along(x, o));
	for (const x of orig) m = Math.max(m, along(x, e));
	return m;
}

// ─── 按线宽拆段 ─────────────────────────────────────────────────

export interface WidthRun {
	prims: RenderedPrim[];
	a: Pt;
	b: Pt;
	layer: number;
}

/**
 * 一个连续导线段按线宽变化处拆成多段：每段内线宽一致，回写时线宽才能原样保留。
 * widthOf 由调用方给出（读 __width，缺省取默认线宽）。
 * 拆分点 = 线宽变化处前一图元的末端；各段 a/b 取沿 a→b 方向的端点
 * （首段 a / 末段 b 沿用原 a/b）。线宽一致时原样返回单段。
 */
export function splitRunByWidth(
	seg: WidthRun, widthOf: (p: RenderedPrim) => number,
): Array<WidthRun & { width: number }> {
	if (!seg.prims.length) return [];
	const groups: Array<{ prims: RenderedPrim[]; width: number }> = [];
	for (const p of seg.prims) {
		const w = widthOf(p);
		const g = groups[groups.length - 1];
		if (g && Math.abs(g.width - w) <= WIDTH_EPS) g.prims.push(p);
		else groups.push({ prims: [p], width: w });
	}
	if (groups.length === 1) return [{ ...seg, width: groups[0].width }];

	// 沿链走一遍，得到每个图元「按行进方向」的终点
	const ends: Pt[] = [];
	let cur = seg.a;
	for (const p of seg.prims) {
		const [s, e] = primEnds(p);
		const fwd = Math.hypot(s.x - cur.x, s.y - cur.y) <= Math.hypot(e.x - cur.x, e.y - cur.y);
		const next = fwd ? e : s;
		ends.push(next);
		cur = next;
	}
	const out: Array<WidthRun & { width: number }> = [];
	let idx = 0;
	let a = seg.a;
	groups.forEach((g, gi) => {
		idx += g.prims.length;
		const b = gi === groups.length - 1 ? seg.b : ends[idx - 1];
		out.push({ prims: g.prims, a, b, layer: seg.layer, width: g.width });
		a = b;
	});
	return out;
}

// ─── 现读画布 + 几何匹配 ────────────────────────────────────────

/** 现读到的一个画布图元（id 只在本次读到之后立刻用来删除，从不存下来） */
export interface CanvasItem { cls: 'line' | 'arc'; id: string; layer: number; width: number; prim: RenderedPrim; bb: BBox }
/** 模型图元（带线宽） */
export interface ModelItem { prim: RenderedPrim; width: number }

/**
 * 现读一个网的线 + 弧（转换失败的画布图元不返回 → 永远不会被删）。
 * 线宽按 widthOrDefault(lineWidth, defaultWidth) 取——与读侧建模型同一口径（lineWidth 读成 0/缺失时取默认线宽）。
 * net 过滤：getAll(net) 按网过滤是真机 4.1.99 上「观察到」的行为（e2e 按网读、返回的就是该网，≈14ms），
 * 不是文档保证；这里再按 net 字段过滤一遍兜底（客户端若忽略参数返回全板，结果仍正确，只是慢）。
 */
export async function readNetCopper(api: WritebackApi, net: string, defaultWidth = 0): Promise<CanvasItem[]> {
	const out: CanvasItem[] = [];
	for (const l of (await api.listLines(net)) ?? []) {
		if (String(l?.net ?? '') !== net) continue;
		const p = canvasLineToRendered(l, net);
		const id = String(l.primitiveId ?? '');
		if (!p || !id || primLength(p) < 1e-9) continue;
		out.push({ cls: 'line', id, layer: p.layer, width: widthOrDefault(l.lineWidth, defaultWidth), prim: p, bb: bboxOf(p) });
	}
	for (const a of (await api.listArcs(net)) ?? []) {
		if (String(a?.net ?? '') !== net) continue;
		const p = canvasArcToRendered(a, net);
		const id = String(a.primitiveId ?? '');
		if (!p || !id) continue;
		out.push({ cls: 'arc', id, layer: p.layer, width: widthOrDefault(a.lineWidth, defaultWidth), prim: p, bb: bboxOf(p) });
	}
	return out;
}

/** 模型集合（网格哈希加速）：点是否落在某个同层模型图元 tol 内；图元是否整条（连续）落在集合 tol 内 */
class PathSet {
	private static CELL = 32;
	private cells = new Map<string, Array<{ p: RenderedPrim; bb: BBox }>>();
	/** 登记外扩：≥ 2·TOL_MIL，distTo 在此值处截断仍严格大于 tol（连续检查要求） */
	private pad: number;
	constructor(items: ModelItem[], pad = 2 * TOL_MIL) {
		this.pad = pad;
		const C = PathSet.CELL;
		for (const m of items) {
			const it = { p: m.prim, bb: bboxOf(m.prim) };
			const x0 = Math.floor((it.bb.x0 - pad) / C), x1 = Math.floor((it.bb.x1 + pad) / C);
			const y0 = Math.floor((it.bb.y0 - pad) / C), y1 = Math.floor((it.bb.y1 + pad) / C);
			for (let i = x0; i <= x1; i++) {
				for (let j = y0; j <= y1; j++) {
					const k = i + ',' + j;
					const arr = this.cells.get(k);
					if (arr) arr.push(it); else this.cells.set(k, [it]);
				}
			}
		}
	}
	has(q: Pt, layer: number, tol = TOL_MIL): boolean {
		const C = PathSet.CELL;
		const arr = this.cells.get(Math.floor(q.x / C) + ',' + Math.floor(q.y / C));
		if (!arr) return false;
		for (const it of arr) {
			if (it.p.layer !== layer || !bbHas(it.bb, q, tol)) continue;
			if (distToPrim(it.p, q) <= tol) return true;
		}
		return false;
	}
	/** 点到同层集合的距离，在 pad 处截断（截断值对位置仍是 1-Lipschitz） */
	private distTo(q: Pt, layer: number): number {
		const C = PathSet.CELL;
		const arr = this.cells.get(Math.floor(q.x / C) + ',' + Math.floor(q.y / C));
		let best = this.pad;
		if (!arr) return best;
		for (const it of arr) {
			if (it.p.layer !== layer || !bbHas(it.bb, q, best)) continue;
			best = Math.min(best, distToPrim(it.p, q));
		}
		return best;
	}
	/** 图元上第一个离集合 > tol 的点（连续检查）；整条都在 tol 内 → null */
	outside(p: RenderedPrim, layer: number, tol = TOL_MIL): Pt | null {
		if (tol >= this.pad) throw new Error('PathSet.outside: tol 必须小于登记外扩');
		const C = PathSet.CELL;
		const bound = (ta: number, tb: number): number => {
			const q = pointAt(p, ta);
			let best = Infinity;
			for (const it of this.cells.get(Math.floor(q.x / C) + ',' + Math.floor(q.y / C)) ?? []) {
				if (it.p.layer !== layer) continue;
				best = Math.min(best, subBound(it.p, p, ta, tb));
				if (best <= tol) break;
			}
			return best;
		};
		return firstOver(p, q => this.distTo(q, layer), tol, bound);
	}
}

/**
 * 模型图元是否被画布铜（同层、同线宽）完整覆盖；返回第一个没被覆盖的点。
 * 连续覆盖：沿模型图元弧长，「到候选画布铜的最近距离」处处 ≤ tol（见 firstOver），
 * 不再按固定点数采样——长图元上两个采样点之间的洞也会被发现。
 */
function uncoveredPoint(m: ModelItem, canvas: CanvasItem[], tol = TOL_MIL): Pt | null {
	const bb = bboxOf(m.prim);
	const cap = 2 * tol;
	const cands = canvas.filter(c => c.layer === m.prim.layer && Math.abs(c.width - m.width) <= WIDTH_EPS && bbNear(c.bb, bb, cap));
	const g = (q: Pt): number => {
		let best = cap;
		for (const c of cands) {
			if (!bbHas(c.bb, q, best)) continue;
			best = Math.min(best, distToPrim(c.prim, q));
		}
		return best;
	};
	const bound = (ta: number, tb: number): number => {
		let best = Infinity;
		for (const c of cands) { best = Math.min(best, subBound(c.prim, m.prim, ta, tb)); if (best <= tol) break; }
		return best;
	};
	return firstOver(m.prim, g, tol, bound);
}

/** 两组模型几何是否相同（互相覆盖，同层同线宽，容差 tol） */
function sameGeometry(a: ModelItem[], b: ModelItem[], tol: number): boolean {
	const asCanvas = (xs: ModelItem[]): CanvasItem[] =>
		xs.map(x => ({ cls: x.prim.kind, id: '', layer: x.prim.layer, width: x.width, prim: x.prim, bb: bboxOf(x.prim) }));
	const ca = asCanvas(a), cb = asCanvas(b);
	return a.every(x => !uncoveredPoint(x, cb, tol)) && b.every(x => !uncoveredPoint(x, ca, tol));
}

/** 恢复目标：要建的调用（null = 太短不建）+ 它在画布上应读回的模型几何 */
interface RestoreTarget { call: CreateCall | null; item: ModelItem; /** 画布上读回相同的图元也不认领：删掉重建（端点被改到线端点，读回取整看不出区别） */ force?: boolean }

/** 精确相同的容差：线端点 1e-4mil；弧按读回取整口径（端点 0.1mil、扫角 0.1°，各留余量） */
const EXACT_LINE_TOL = 1e-4;
const EXACT_ARC_TOL = 0.1;
const EXACT_ARC_DEG = 0.06;
/** 弧读回取整最大偏差（0.05 × √2 ≈ 0.07，留余量）：弧端点离旧线端点不超过此值视为同一接点 */
const ARC_JOIN_MIL = 0.08;

/** 画布图元是否与模型图元「原样相同」（同层同线宽同种类，端点按正/反方向对得上；弧再比扫角） */
function sameAsModel(c: CanvasItem, m: ModelItem): boolean {
	const p = m.prim;
	if (c.prim.kind !== p.kind || c.layer !== p.layer || Math.abs(c.width - m.width) > WIDTH_EPS) return false;
	const [cs, ce] = primEnds(c.prim), [ms, me] = primEnds(p);
	const tol = p.kind === 'line' ? EXACT_LINE_TOL : EXACT_ARC_TOL;
	const fwd = dist(cs, ms) <= tol && dist(ce, me) <= tol;
	const rev = dist(cs, me) <= tol && dist(ce, ms) <= tol;
	if (!fwd && !rev) return false;
	if (p.kind === 'line') return true;
	// 带符号比扫角：正向对上 → 扫角相同；反向对上 → 扫角相反（同端点、反向鼓起的镜像弧不算相同）
	const near = (want: number): boolean => Math.abs(c.prim.sweep! - want) * 180 / Math.PI <= EXACT_ARC_DEG;
	return (fwd && near(p.sweep!)) || (rev && near(-p.sweep!));
}

/** 画布图元的采样点（分类用，粗一些） */
const canvasSamples = (c: CanvasItem): Pt[] => samplePrim(c.prim, 1, 100);

/** 一处铜的差异：缺铜（模型有、画布没有）/ 多余铜（画布有、不该有） */
export interface CopperIssue { net: string; layer: number; kind: 'missing' | 'extra'; x: number; y: number }

export function formatIssues(list: CopperIssue[], max = 6): string {
	const s = list.slice(0, max).map(i =>
		`网络 ${i.net} 层${i.layer} ${i.kind === 'missing' ? '缺铜' : '多余铜'}@(${i.x.toFixed(1)}, ${i.y.toFixed(1)})`).join('、');
	return list.length > max ? `${s} 等 ${list.length} 处` : s;
}

// ─── 串行队列 ───────────────────────────────────────────────────

/** 严格按到达顺序串行执行异步任务；单个任务异常不中断后续任务 */
export class SerialQueue {
	private tail: Promise<void> = Promise.resolve();

	push(fn: () => Promise<void>): Promise<void> {
		const p = this.tail.then(fn, fn).catch(() => { /* 单任务失败不影响后续 */ });
		this.tail = p;
		return p;
	}

	/** 等到当前已入队的全部任务完成 */
	idle(): Promise<void> {
		return this.tail;
	}
}

// ─── 回写状态机 ─────────────────────────────────────────────────

export interface WbRunState {
	id: number;
	net: string;
	layer: number;
	width: number;
	/** 该段当前的模型几何（初始 = 从画布读到的；写入后 = 实际写出的） */
	prims: RenderedPrim[];
	/** 原始长度（mil） */
	origLen: number;
}

/** 本网不属于任何 run 的现存线/弧（只读；重写本网时必须保住它们） */
export interface StaticCopper { net: string; layer: number; width: number; prim: RenderedPrim }

export interface RunMsg { id: number; prims: RenderedPrim[] | null; oldLen?: number; newLen?: number }
export interface NetRewriteMsg { net: string; prims: RenderedPrim[] | null; oldLen?: number; newLen?: number }

export interface ApplyOutcome {
	applied: boolean;
	/** 未采纳原因（applied=false 时） */
	reason?: string;
	/** 写失败（已尝试恢复原走线） */
	rolledBack?: boolean;
	/** 因先前回写失败已中止，本消息被直接忽略（未碰画布、未计入已处理） */
	ignored?: boolean;
	/** 写出几何超出回写误差预算（emitError > WRITER_ERR）：未写入、未碰画布；不算失败、不中止 */
	overBudget?: boolean;
}

/** 回写汇总（全部按「段」计，保证 adopted ≤ total） */
export interface WbSummary {
	/** 总段数 */
	total: number;
	/** 已处理段数（含无改动 / 失败；中止后被忽略的不计） */
	done: number;
	/** 采纳（已写入画布）的段数；整网重写 = 该网全部段 */
	adopted: number;
	/** 回写失败的段数（整网重写失败 = 该网全部段） */
	failed: number;
	/** 回写失败次数 */
	rollbacks: number;
	/** 写后现读校验通过的段数 */
	verified: number;
	/** 失败后成功恢复原走线的次数 */
	restored: number;
	/** 失败后未能恢复的差异（缺铜 / 多余铜，各带一个坐标） */
	unrestored: CopperIssue[];
	/** 结束时对写过的网做的画布 vs 模型核对结果（null = 尚未核对） */
	audit: CopperIssue[] | null;
	/** 参与结束核对的网数 */
	auditedNets: number;
	/** 中止后被忽略的求解事件数 */
	ignored: number;
	/** 写出几何超出回写误差预算而未采纳的次数（未碰画布，不算失败） */
	overBudget: number;
	/** 尚未处理的段数（total - done） */
	pending: number;
	halted: boolean;
	haltReason: string;
	savedMil: number;
}

interface WriteResult {
	ok: boolean;
	/** 成功：写出后的模型几何 */
	model: RenderedPrim[];
	/** 成功且经过写后现读校验（几何未变的 no-op 不计） */
	verified: boolean;
	error?: string;
	/** 失败时：画布是否已恢复为原走线（含「根本没动画布」） */
	restored?: boolean;
	/** 失败且未恢复：剩余差异 */
	issues?: CopperIssue[];
	/** 未尝试写入（写出几何超出误差预算）：不算失败，ok=false 但不走失败/中止 */
	overBudget?: boolean;
}

type WbRun = WbRunState & { curLen: number; superseded: boolean };

/**
 * 回写状态机：持有每段当前模型几何与长度，逐条应用求解消息。
 * - 每次回写都按几何现读匹配旧铜（不依赖 id），先删旧、再建新、再现读校验；
 * - 同一 run 可被多次回写；一个 run 无论改写几次只计一次「采纳」；savedMil = Σ(原长 − 最新长)；
 * - 整网重写替换该网全部 run（该网每一段都计为采纳），之后该网的 tightenRun 消息被忽略；
 * - 第一次回写失败（创建失败 / 校验不符）即恢复原走线并「中止」：此后所有消息直接忽略、不再碰画布。
 *   原因：求解器是基于「已采纳的新几何」继续往下解的，失败后板上仍是旧铜，
 *   后续候选可能与旧铜相交，只能整体停下并如实汇报。
 * 调用方须保证按到达顺序串行调用（SerialQueue）。
 */
export class TightenWriteback {
	private runs = new Map<number, WbRun>();
	private byNet = new Map<string, number[]>();
	private statics = new Map<string, ModelItem[]>();
	private seen = new Set<number>();
	private adoptedKeys = new Set<number>();
	private failedKeys = new Set<number>();
	private verifiedKeys = new Set<number>();
	private touched = new Set<string>();
	private unrestoredIssues: CopperIssue[] = [];
	private auditIssues: CopperIssue[] | null = null;
	private ignoredCount = 0;
	private overBudgetCount = 0;
	private restoredCount = 0;
	private haltedFlag = false;
	private haltMsg = '';
	/** 回写失败次数 */
	public rollbacks = 0;

	constructor(
		private api: WritebackApi,
		runs: WbRunState[],
		private log?: (msg: string, level: string) => void,
		/** 第一次回写失败、进入中止态时回调一次（调用方据此终止 Worker / 求解） */
		private onHalt?: (reason: string) => void,
		statics: StaticCopper[] = [],
		/** 画布 lineWidth 读成 0/缺失时取的线宽（= 读侧建模型的回退线宽，通常 drc.traceWidthMil；见 widthOrDefault） */
		private defaultWidth = 0,
	) {
		for (const r of runs) {
			this.runs.set(r.id, { ...r, prims: [...r.prims], curLen: r.origLen, superseded: false });
			const arr = this.byNet.get(r.net);
			if (arr) arr.push(r.id); else this.byNet.set(r.net, [r.id]);
		}
		for (const s of statics) {
			const arr = this.statics.get(s.net) ?? [];
			arr.push({ prim: { ...s.prim, layer: s.layer }, width: s.width });
			this.statics.set(s.net, arr);
		}
	}

	get total(): number { return this.runs.size; }
	/** 已收到过结果的不同 run 数（含 null 结果） */
	get done(): number { return Math.min(this.seen.size, this.runs.size); }
	get adopted(): number { return this.adoptedKeys.size; }
	get halted(): boolean { return this.haltedFlag; }
	get savedMil(): number {
		let s = 0;
		for (const r of this.runs.values()) s += r.origLen - r.curLen;
		return Math.max(0, s);
	}
	/** 某段当前的模型几何（测试/诊断用） */
	primsOf(id: number): RenderedPrim[] { return [...(this.runs.get(id)?.prims ?? [])]; }
	/** 写过（或尝试写过）画布的网 */
	touchedNets(): string[] { return [...this.touched]; }

	summary(): WbSummary {
		const done = this.done;
		return {
			total: this.total, done, adopted: this.adopted, failed: this.failedKeys.size,
			rollbacks: this.rollbacks, verified: this.verifiedKeys.size, restored: this.restoredCount,
			unrestored: [...this.unrestoredIssues], audit: this.auditIssues ? [...this.auditIssues] : null,
			auditedNets: this.auditIssues ? this.touched.size : 0,
			ignored: this.ignoredCount, overBudget: this.overBudgetCount, pending: Math.max(0, this.total - done),
			halted: this.haltedFlag, haltReason: this.haltMsg, savedMil: this.savedMil,
		};
	}

	/** 某网的完整模型（全部 run 当前几何 + 只读静态铜），可排除若干 run */
	private netModel(net: string, exclude: Set<number> = new Set()): ModelItem[] {
		const out: ModelItem[] = [];
		for (const id of this.byNet.get(net) ?? []) {
			if (exclude.has(id)) continue;
			const r = this.runs.get(id)!;
			for (const p of r.prims) out.push({ prim: p, width: r.width });
		}
		out.push(...(this.statics.get(net) ?? []));
		return out;
	}

	private ignore(): ApplyOutcome {
		this.ignoredCount++;
		return { applied: false, reason: '回写失败已中止', ignored: true };
	}

	/** 一次回写失败：记账、告警（含未恢复的坐标）、进入中止态 */
	private onWriteFailed(label: string, runIds: number[], res: WriteResult): ApplyOutcome {
		this.rollbacks++;
		for (const i of runIds) { this.failedKeys.add(i); this.seen.add(i); }
		if (res.restored) {
			this.restoredCount++;
			this.log?.(`${label}回写失败，已恢复原走线：${res.error}`, 'warn');
		} else {
			const issues = res.issues ?? [];
			this.unrestoredIssues.push(...issues);
			// error 级：宿主会弹提示，用户必须看到——这些位置要手动检查
			this.log?.(
				`${label}回写失败且未能恢复原走线（需手动检查）：${res.error}` + (issues.length ? `；${formatIssues(issues, 20)}` : ''),
				'error',
			);
		}
		if (!this.haltedFlag) {
			this.haltedFlag = true;
			this.haltMsg = `${label}回写失败：${res.error}`;
			this.log?.('回写失败：已中止拉紧，后续求解结果不再写入画布', 'warn');
			this.onHalt?.(this.haltMsg);
		}
		return { applied: false, reason: res.error, rolledBack: true };
	}

	/** 写出几何超预算：不写、不算失败、不中止；如实记账并告警 */
	private onOverBudget(label: string, res: WriteResult): ApplyOutcome {
		this.overBudgetCount++;
		this.log?.(`${label}未采纳：${res.error}`, 'warn');
		return { applied: false, reason: res.error, overBudget: true };
	}

	async applyRun(msg: RunMsg): Promise<ApplyOutcome> {
		if (this.haltedFlag) return this.ignore();
		const r = this.runs.get(msg.id);
		if (!r) return { applied: false, reason: '未知 run' };
		this.seen.add(msg.id);
		if (!msg.prims || !msg.prims.length) return { applied: false, reason: '无改动' };
		if (r.superseded) return { applied: false, reason: '该网已整网重写' };
		const res = await this.rewrite(r.net, r.layer, r.width, [r], msg.prims);
		if (res.overBudget) return this.onOverBudget(`网络 ${r.net} 段 ${r.id}`, res);
		if (!res.ok) return this.onWriteFailed(`网络 ${r.net} 段 ${r.id}`, [r.id], res);
		r.prims = res.model;
		if (Number.isFinite(msg.newLen)) r.curLen = msg.newLen!;
		this.adoptedKeys.add(r.id);
		if (res.verified) this.verifiedKeys.add(r.id);
		return { applied: true };
	}

	async applyNetRewrite(msg: NetRewriteMsg): Promise<ApplyOutcome> {
		if (this.haltedFlag) return this.ignore();
		const ids = this.byNet.get(msg.net);
		if (!ids || !ids.length) return { applied: false, reason: '未知网络' };
		if (!msg.prims || !msg.prims.length) return { applied: false, reason: '无新图元' };
		const rs = ids.map(i => this.runs.get(i)!);
		const carrier = rs[0];
		// 整网重写用同一线宽/层写入；各段不一致时放弃，避免改线宽
		if (rs.some(q => Math.abs(q.width - carrier.width) > WIDTH_EPS || q.layer !== carrier.layer)) {
			return { applied: false, reason: '该网各段线宽/层不一致，跳过整网重写' };
		}
		const res = await this.rewrite(msg.net, carrier.layer, carrier.width, rs, msg.prims);
		if (res.overBudget) return this.onOverBudget(`网络 ${msg.net}（整网重写）`, res);
		if (!res.ok) return this.onWriteFailed(`网络 ${msg.net}（整网重写）`, ids, res);
		const prevLen = rs.reduce((t, q) => t + q.curLen, 0);
		for (const q of rs) {
			q.prims = [];
			q.curLen = 0;
			q.superseded = true;
			// 整网重写 = 该网每一段都被采纳（与 total 同按「段」计，保证 adopted ≤ total）
			this.adoptedKeys.add(q.id);
			this.seen.add(q.id);
			if (res.verified) this.verifiedKeys.add(q.id);
		}
		carrier.prims = res.model;
		carrier.curLen = Number.isFinite(msg.newLen) ? msg.newLen! : prevLen;
		return { applied: true };
	}

	// ── 画布操作 ──

	private async create(net: string, c: CreateCall, width: number): Promise<void> {
		const id = c.kind === 'line'
			? await this.api.createLine(net, c.layer, c.sx, c.sy, c.ex, c.ey, width)
			: await this.api.createArc(net, c.layer, c.sx, c.sy, c.ex, c.ey, c.deg, width);
		if (!id) throw new Error('创建未返回图元 id');
	}

	/** 删除现读到的图元（返回值无信息，忽略；删没删掉由之后的现读判定） */
	private async deleteItems(items: CanvasItem[]): Promise<void> {
		const lines = items.filter(c => c.cls === 'line').map(c => c.id);
		const arcs = items.filter(c => c.cls === 'arc').map(c => c.id);
		try { if (lines.length) await this.api.deleteLines(lines); } catch { /* 由校验发现 */ }
		try { if (arcs.length) await this.api.deleteArcs(arcs); } catch { /* 由校验发现 */ }
	}

	/**
	 * 现读核对：keep 必须被画布完整覆盖；画布上不能有「落在 gone 上、又不在 keep ∪ shape 上」的铜。
	 * shape：只参与「解释画布铜」、不要求被覆盖的模型铜（写前就与画布不一致的其它模型铜，见 rewrite）。
	 */
	private async check(
		net: string, keep: ModelItem[], gone: ModelItem[], shape: ModelItem[] = [],
	): Promise<{ canvas: CanvasItem[]; missing: Array<{ m: ModelItem; at: Pt }>; extra: CopperIssue[] }> {
		const canvas = await readNetCopper(this.api, net, this.defaultWidth);
		const missing: Array<{ m: ModelItem; at: Pt }> = [];
		for (const m of keep) {
			const at = uncoveredPoint(m, canvas);
			if (at) missing.push({ m, at });
		}
		const keepSet = new PathSet([...keep, ...shape]), goneSet = new PathSet(gone);
		const explSet = new PathSet([...gone, ...keep, ...shape]);
		const extra: CopperIssue[] = [];
		for (const c of canvas) {
			// 只把「能被 gone ∪ keep 完全解释」（连续检查）的图元算作残留旧铜；模型外的铜（例如读完之后用户画的）
			// 不在这里判（也绝不会被删），由结束核对 audit() 报为多余铜
			if (explSet.outside(c.prim, c.layer)) continue;
			const q = canvasSamples(c).find(pt => goneSet.has(pt, c.layer) && !keepSet.has(pt, c.layer));
			if (q) extra.push({ net, layer: c.layer, kind: 'extra', x: q.x, y: q.y });
		}
		return { canvas, missing, extra };
	}

	/**
	 * 补回缺失的「其它」模型铜（被删掉的跨段合并图元的另一部分），然后再核对一次。
	 * preBad：写前就没被画布覆盖的其它模型铜——不要求覆盖、绝不补建（补建 = 凭空加铜），留给结束核对如实上报。
	 */
	private async checkAndRepair(net: string, mine: ModelItem[], others: ModelItem[], gone: ModelItem[], preBad: Set<ModelItem>) {
		const ok = others.filter(o => !preBad.has(o));
		const bad = others.filter(o => preBad.has(o));
		const keep = [...mine, ...ok];
		let chk = await this.check(net, keep, gone, bad);
		const otherSet = new Set(ok);
		const repair = chk.missing.filter(x => otherSet.has(x.m));
		if (repair.length) {
			for (const { m } of repair) {
				const c = primToCall(m.prim, m.prim.layer);
				if (!c) continue;
				try { await this.create(net, c, m.width); } catch { /* 由下一次核对发现 */ }
			}
			chk = await this.check(net, keep, gone, bad);
		}
		return chk;
	}

	/**
	 * 把 rs 这几段（同网同层同线宽）的当前几何换成 newPrims：
	 * 误差预算检查 → 现读（记下写前就不一致的其它模型铜）→ 按几何删旧 → 建新 → 现读校验；失败则恢复原走线。
	 */
	private async rewrite(net: string, layer: number, width: number, rs: WbRun[], newPrims: RenderedPrim[]): Promise<WriteResult> {
		const exclude = new Set(rs.map(q => q.id));
		const oldItems: ModelItem[] = rs.flatMap(q => q.prims.map(p => ({ prim: p, width: q.width })));
		const others = this.netModel(net, exclude);
		const blockers = others.map(o => o.prim);
		const plan = planEmit(newPrims, net, layer, blockers);
		if (!plan) return { ok: false, model: [], verified: false, error: '图元几何无效（未改动画布）', restored: true };
		if (!plan.calls.length) return { ok: false, model: [], verified: false, error: '无新图元（未改动画布）', restored: true };
		// 求解器的余量按「写出几何偏离求解几何 ≤ WRITER_ERR」算：超出就不能写（可能压到间距底线以内）
		const err = emitError(newPrims.map(p => ({ ...p, net, layer })), plan.model);
		if (!(err <= WRITER_ERR)) {
			return {
				ok: false, model: [], verified: false, overBudget: true,
				error: `写出几何偏离求解结果 ${Number.isFinite(err) ? err.toFixed(3) : String(err)}mil，超出回写误差预算 ${WRITER_ERR}mil（未改动画布）`,
			};
		}
		const newItems: ModelItem[] = plan.model.map(p => ({ prim: p, width }));
		// 新几何与当前几何相同：不碰画布
		if (sameGeometry(newItems, oldItems, NOOP_TOL_MIL)) {
			return { ok: true, model: rs.flatMap(q => q.prims), verified: false };
		}

		this.touched.add(net);
		let error = '';
		let preBad = new Set<ModelItem>();
		try {
			// 1. 现读，按几何找出承载旧路径的画布图元：有采样点「只在旧路径上」、且整条（连续检查）都能被
			//    旧路径/本网其它模型解释（解释不了的铜不是我们的，绝不删）
			const canvas = await readNetCopper(this.api, net, this.defaultWidth);
			// 写前基线：其它模型铜里写前就没被画布（同层同线宽）覆盖的——不是本次写入造成的，不计入本次校验、不补建
			preBad = new Set(others.filter(o => uncoveredPoint(o, canvas)));
			if (preBad.size) {
				this.log?.(`网络 ${net} 有 ${preBad.size} 个模型图元写前就与画布不一致（不计入本次校验，结束核对如实上报）`, 'warn');
			}
			const oldSet = new PathSet(oldItems), othSet = new PathSet(others);
			const explSet = new PathSet([...oldItems, ...others]);
			const del = canvas.filter(c =>
				canvasSamples(c).some(q => oldSet.has(q, c.layer) && !othSet.has(q, c.layer)) && !explSet.outside(c.prim, c.layer));
			// 2. 先删旧
			await this.deleteItems(del);
			// 3. 再建新
			for (const c of plan.calls) await this.create(net, c, width);
			// 4. 现读校验（被删的跨段合并图元的「其它段」部分在这里补回）
			const chk = await this.checkAndRepair(net, newItems, others, oldItems, preBad);
			if (!chk.missing.length && !chk.extra.length) return { ok: true, model: plan.model, verified: true };
			const issues = [
				...chk.missing.map(x => ({ net, layer: x.m.prim.layer, kind: 'missing' as const, x: x.at.x, y: x.at.y })),
				...chk.extra,
			];
			error = '写后校验不符：' + formatIssues(issues);
		} catch (err) {
			error = '创建/读取失败：' + (err instanceof Error ? err.message : String(err));
		}
		const rb = await this.restore(net, layer, width, oldItems, newItems, others, preBad);
		return { ok: false, model: [], verified: false, error, restored: rb.restored, issues: rb.issues };
	}

	/**
	 * 恢复原走线。
	 *
	 * 旧模型图元（oldItems）的来源只有两种：(1) 一开始从画布现读到的（从未被我们写过）；(2) 之前某次成功回写
	 * 的 plan.model（已经过防合并/网格处理，本来就是「写出口径」）。两种都应「原样」建回，不能再过一遍
	 * planEmit——planEmit 会把线端点吸附到弧端点网格、插隔离弧、合并近共线线，结果和原图元不再一致。
	 * （真机 LCD3 实测的碎线：线端 1593.6279、对着读回取整为 1593.6 的弧起点；弧按取整值建回时起点落在线内部，
	 * EDA 把线在 1593.6 拆开，多出 0.028mil 的一根——见下面 pullToLine 与 force。）
	 * 所以主路径 = 逐图元原样重建（verbatim）：
	 *  - 画布上「与旧图元精确相同」（线端点 1e-4mil；弧按读回取整口径）的图元原样保留，不碰；
	 *  - 其余落在 新 ∪ 旧 路径上、又能被 新 ∪ 旧 ∪ 其它模型铜解释的图元（新建的近似副本、EDA 拆出的碎片、
	 *    与旧线部分重叠的新线……）全部先删，再按旧图元原样重建；这样不会把旧线建在新线上面（EDA 会把重叠部分
	 *    当成「已存在」，只补出一小段碎线）；
	 *  - 仍有缺铜（EDA 把原样重建的图元合并/变形了：浅折线、近共线）才退到第二遍：走 planEmit 的防合并口径重建。
	 * 适用口径：绝大多数情况是第一种（原样）；planEmit 口径只在原样重建被 EDA 改形时才用。
	 * preBad（写前就不一致的其它模型铜）不要求覆盖、不补建。
	 */
	private async restore(
		net: string, layer: number, width: number, oldItems: ModelItem[], newItems: ModelItem[], others: ModelItem[],
		preBad: Set<ModelItem>,
	): Promise<{ restored: boolean; issues: CopperIssue[] }> {
		try {
			// 弧端点读回时被取整（最多 ~0.07mil），而与它相接的线端点是精确值：弧按取整后的端点原样建回，
			// 这个端点会落在线的内部（差 0.028mil），EDA 会把线在该点拆成两根（多出一根碎线，真机 LCD3 实测）。
			// 所以弧的端点若与某旧线端点相差 ≤ 读回取整误差，就用那个线端点建弧（线保持原样，弧在读回取整内）。
			const lineEnds: Pt[] = oldItems.filter(m => m.prim.kind === 'line').flatMap(m => primEnds(m.prim));
			const pullToLine = (q: Pt): Pt => {
				let best: Pt | null = null, bd = Infinity;
				for (const e of lineEnds) { const d = dist(e, q); if (d < bd) { bd = d; best = e; } }
				return best && bd > 1e-9 && bd <= ARC_JOIN_MIL ? { ...best } : q;
			};
			const verbatim: RestoreTarget[] = oldItems.map(m => {
				let c = primToCall(m.prim, layer);
				let force = false;
				if (c && c.kind === 'arc') {
					const s0 = pullToLine({ x: c.sx, y: c.sy }), e0 = pullToLine({ x: c.ex, y: c.ey });
					force = s0.x !== c.sx || s0.y !== c.sy || e0.x !== c.ex || e0.y !== c.ey;
					if (force && dist(s0, e0) >= MIN_LINE_MIL) c = { ...c, sx: s0.x, sy: s0.y, ex: e0.x, ey: e0.y };
					else force = false;
				}
				return { call: c, item: c ? { prim: callToPrim(c, net) ?? m.prim, width: m.width } : m, force };
			});
			let r = await this.restorePass(net, width, verbatim, oldItems, newItems, others, preBad);
			if (r.issues.length) {
				// 原样重建被 EDA 改形（缺铜）：退到防合并口径
				const plan = planEmit(oldItems.map(m => m.prim), net, layer, others.map(o => o.prim));
				if (plan) {
					const guarded: RestoreTarget[] = plan.calls.map((c, i) => ({ call: c, item: { prim: plan.model[i], width } }));
					r = await this.restorePass(net, width, guarded, oldItems, newItems, others, preBad);
				}
			}
			return { restored: !r.issues.length, issues: r.issues };
		} catch {
			// 连现读都失败：无法确认画布状态，按「未恢复」上报（坐标取旧路径起点）
			const p0 = oldItems[0] ? primEnds(oldItems[0].prim)[0] : { x: NaN, y: NaN };
			return { restored: false, issues: [{ net, layer, kind: 'missing', x: p0.x, y: p0.y }] };
		}
	}

	/** 一遍恢复：保留与目标精确相同的画布图元，删掉其余能解释的新/旧近似铜，补建缺的目标，再核对 */
	private async restorePass(
		net: string, width: number, targets: RestoreTarget[], oldItems: ModelItem[], newItems: ModelItem[], others: ModelItem[],
		preBad: Set<ModelItem>,
	): Promise<{ issues: CopperIssue[] }> {
		const canvas = await readNetCopper(this.api, net, this.defaultWidth);
		// 精确相同的画布图元：每个目标/其它模型图元最多认领一个（重复的副本不认领 → 会被删）
		const kept = new Set<CanvasItem>();
		const claim = (m: ModelItem): CanvasItem | undefined => {
			const c = canvas.find(x => !kept.has(x) && sameAsModel(x, m));
			if (c) kept.add(c);
			return c;
		};
		const have = targets.map(t => !t.force && !!claim(t.item));
		for (const o of others) claim(o);
		const explModel = [...newItems, ...oldItems, ...targets.map(t => t.item)];
		const expl = new PathSet(explModel);
		const othSet = new PathSet(others);
		const allSet = new PathSet([...explModel, ...others]);
		const del = canvas.filter(c => {
			if (kept.has(c)) return false;
			return canvasSamples(c).some(q => expl.has(q, c.layer) && !othSet.has(q, c.layer)) && !allSet.outside(c.prim, c.layer);
		});
		await this.deleteItems(del);
		const delSet = new Set(del);
		const remain = canvas.filter(c => !delSet.has(c));
		let createFailed = 0;
		for (let i = 0; i < targets.length; i++) {
			const t = targets[i];
			if (have[i] || !t.call) continue;
			// 剩下的画布铜（其它模型铜、不认识的铜）已经盖住它：不重复建
			if (!uncoveredPoint(t.item, remain)) continue;
			try { await this.create(net, t.call, width); } catch { createFailed++; }
		}
		const chk = await this.checkAndRepair(net, oldItems, others, newItems, preBad);
		const issues: CopperIssue[] = [
			...chk.missing.map(x => ({ net, layer: x.m.prim.layer, kind: 'missing' as const, x: x.at.x, y: x.at.y })),
			...chk.extra,
		];
		if (createFailed) this.log?.(`恢复原走线时有 ${createFailed} 个图元创建失败`, 'warn');
		return { issues };
	}

	/**
	 * 结束核对：对每个写过的网现读一次，与完整模型（全部 run 当前几何 + 静态铜）比对。
	 * 缺铜 = 模型图元没被同层同线宽画布铜覆盖（连续检查）；多余铜 = 画布图元有部分不在任何模型图元上（连续检查）。
	 * 写前就不一致的模型铜也在这里如实上报（回写过程中不计入校验、也不补建）。
	 */
	async audit(): Promise<CopperIssue[]> {
		const out: CopperIssue[] = [];
		for (const net of this.touched) {
			const model = this.netModel(net);
			const layers = new Set(model.map(m => m.prim.layer));
			let canvas: CanvasItem[];
			try { canvas = await readNetCopper(this.api, net, this.defaultWidth); } catch (err) {
				// 终检读不到画布 ≠ 没问题：按「缺铜」上报（坐标取模型第一个图元的起点），并告警
				const p0 = model[0] ? primEnds(model[0].prim)[0] : { x: NaN, y: NaN };
				out.push({ net, layer: model[0]?.prim.layer ?? 1, kind: 'missing', x: p0.x, y: p0.y });
				this.log?.(`网络 ${net} 终检时无法重新读取画布，无法确认铜与模型一致（需手动检查）：${err instanceof Error ? err.message : String(err)}`, 'warn');
				continue;
			}
			for (const m of model) {
				const at = uncoveredPoint(m, canvas);
				if (at) out.push({ net, layer: m.prim.layer, kind: 'missing', x: at.x, y: at.y });
			}
			const set = new PathSet(model);
			for (const c of canvas) {
				if (!layers.has(c.layer)) continue;
				const q = set.outside(c.prim, c.layer);
				if (q) out.push({ net, layer: c.layer, kind: 'extra', x: q.x, y: q.y });
			}
		}
		this.auditIssues = out;
		return out;
	}
}

/** 两处差异是否是「同一处」：同网同层同类、坐标相距 ≤ 1mil */
const sameSpot = (a: CopperIssue, b: CopperIssue): boolean =>
	a.net === b.net && a.layer === b.layer && a.kind === b.kind &&
	(Math.hypot(a.x - b.x, a.y - b.y) <= 1 || (!Number.isFinite(a.x) && !Number.isFinite(b.x)));

/** 合并两组差异，去掉重复的同一处（保留 first 中的条目） */
export function mergeIssues(first: CopperIssue[], second: CopperIssue[]): CopperIssue[] {
	const out = [...first];
	for (const i of second) if (!out.some(o => sameSpot(o, i))) out.push(i);
	return out;
}

// ─── 汇报（纯函数，便于单测） ───────────────────────────────────

/** 终态进度/弹窗文案：中止 / 取消 / 完成三种口径都按「段」如实汇报 */
export function formatTightenReport(
	s: WbSummary, cancelled: boolean,
): { stage: string; message: string; level: 'success' | 'warn' | 'error' } {
	// 中止时未恢复的差异已在正文列出：核对里同一处不重复，其它位置（含其它网）照常列出
	const audit = (s.audit ?? []).filter(a => !s.unrestored.some(u => sameSpot(u, a)));
	const auditTail = audit.length ? `；画布与模型不一致（需手动检查）：${formatIssues(audit)}` : '';
	const budgetTail = s.overBudget ? `；${s.overBudget} 次结果超出回写误差预算未写入` : '';
	const verifiedTail = s.verified ? `，写后校验通过 ${s.verified} 段` : '';
	if (s.halted) {
		const restoreText = s.unrestored.length
			? `失败段未能恢复原走线（需手动检查）：${formatIssues(s.unrestored)}`
			: '失败段已恢复原走线';
		return {
			stage: '已中止',
			level: 'error',
			message: `拉紧已中止（回写失败）：已写入 ${s.adopted} 段、失败 ${s.failed} 段、未处理 ${s.pending}/${s.total} 段，` +
				`共省 ${s.savedMil.toFixed(0)}mil；已写入的保留，${restoreText}${budgetTail}${auditTail}`,
		};
	}
	if (cancelled) {
		return {
			stage: '已停止', level: 'warn',
			message: `拉紧已停止：已处理 ${s.done}/${s.total} 段，采纳 ${s.adopted} 段${verifiedTail}${budgetTail}${auditTail}`,
		};
	}
	return {
		stage: '完成', level: audit.length ? 'warn' : 'success',
		message: `拉紧完成：${s.adopted}/${s.total} 段有改善，共省 ${s.savedMil.toFixed(0)}mil${verifiedTail}${budgetTail}${auditTail}`,
	};
}

// ─── Worker 会话（纯逻辑，Worker 以接口注入，可离线单测） ────────

/** Worker 的最小接口 */
export interface WorkerLike {
	onmessage: ((ev: { data: any }) => void) | null;
	onerror: ((e: any) => void) | null;
	postMessage(m: unknown): void;
	terminate(): void;
}

export interface WorkerSession {
	/** 正常完成 / 取消 / 中止时 resolve；Worker 出错时 reject。两种都先等 queue 排空 */
	done: Promise<void>;
	/** 用户取消：终止 Worker，等已入队回写落盘后 resolve */
	cancel(): void;
	/** 回写失败中止：终止 Worker、忽略之后到达的消息，等已入队任务落盘后 resolve */
	halt(): void;
}

/**
 * 在 Worker 上跑一次拉紧任务。
 * - 每条非 error 消息交给 onEvent（由调用方入串行队列）；`tightenDone` 额外排一个收尾任务，
 *   排在所有已到达消息之后——写入全部落盘才算完成；
 * - error 消息 / onerror：立即终止 Worker、不再接收消息，但 **等写入队列排空后** 才 reject，
 *   避免调用方在队列仍在写画布时就当作已结束（解除「拉紧中」标志）。
 */
export function startWorkerSession(
	w: WorkerLike, queue: SerialQueue, job: unknown, onEvent: (m: any) => void,
	hooks: { onTerminate?: () => void } = {},
): WorkerSession {
	let settled = false;
	let stopped = false;   // 之后到达的消息一律忽略
	let resolveFn!: () => void;
	let rejectFn!: (e: Error) => void;
	const done = new Promise<void>((res, rej) => { resolveFn = res; rejectFn = rej; });
	const terminate = (): void => {
		stopped = true;
		hooks.onTerminate?.();
		try { w.terminate(); } catch { /* ignore */ }
	};
	const settleAfterIdle = (fn: () => void): void => {
		void queue.idle().then(() => { if (!settled) { settled = true; fn(); } });
	};
	const fail = (err: Error): void => {
		if (stopped) return;
		terminate();
		settleAfterIdle(() => rejectFn(err));
	};
	w.onmessage = (ev) => {
		const m = ev?.data;
		if (!m || stopped) return;
		if (m.type === 'error') { fail(new Error(String(m.message ?? 'worker error'))); return; }
		onEvent(m);
		if (m.type === 'tightenDone') {
			void queue.push(async () => {
				if (stopped) return;
				terminate();
				if (!settled) { settled = true; resolveFn(); }
			});
		}
	};
	w.onerror = (e) => { fail(new Error(String(e?.message ?? 'worker error'))); };
	w.postMessage(job);
	return {
		done,
		cancel: () => { terminate(); settleAfterIdle(resolveFn); },
		halt: () => { terminate(); settleAfterIdle(resolveFn); },
	};
}

// ─── 静态世界排除（拉紧：已精确表示的图元不再进 baseWorld） ──────

export type CopperPrimKind = 'line' | 'arc' | 'polyline';

/**
 * 「已在拉紧世界里被精确表示」的画布图元 id 集合（按种类分）。
 * 只有真的转换成 run / track 发给 Worker 的图元才登记；转换失败的
 * （弧参数无效、线坐标非有限数、折线不足两点……）不登记，留在 baseWorld 当障碍。
 */
export class RepresentedSet {
	private sets: Record<CopperPrimKind, Set<string>> = { line: new Set(), arc: new Set(), polyline: new Set() };
	add(kind: CopperPrimKind, id: unknown): void {
		const s = String(id ?? '');
		if (s) this.sets[kind].add(s);
	}
	has(kind: CopperPrimKind, id: unknown): boolean {
		const s = String(id ?? '');
		return !!s && this.sets[kind].has(s);
	}
}

/**
 * 静态世界收集时：这个图元要不要跳过（因为它已经以 run/track 精确表示）？
 * 仅当拉紧口径（represented 非 null）且是带网络的铜层(1/2)图元、且 id 已登记时才跳过；
 * 其余一律保留——id 缺失的也保留（宁可重复挡路也不能让铜凭空消失）。
 * represented 为 null（布线口径）永远返回 false，行为与旧版完全一致。
 */
export function isRepresentedExactly(
	represented: RepresentedSet | null, kind: CopperPrimKind,
	net: string | undefined, layer: number, id: unknown,
): boolean {
	if (!represented || !net) return false;
	if (layer !== 1 && layer !== 2) return false;
	return represented.has(kind, id);
}

/**
 * pcb_Net.getAllPrimitivesByNet 的一个条目是不是焊盘（真机实测）：
 * 焊盘没有 pcbItemPrimitiveType、type = 1(SMD)/2(TH)；走线 {pcbItemPrimitiveType:'Track', type: undefined}、
 * 弧 'ArcTrack'、过孔 {'Via', type: 0}。凡带非焊盘 pcbItemPrimitiveType 的一律不是焊盘；
 * 其余沿用旧口径（type 缺省或为 1/2 才收）。
 */
export function isNetPadEntry(p: any): boolean {
	if (!p || typeof p !== 'object') return false;
	const kind = p.pcbItemPrimitiveType;
	if (kind !== undefined && kind !== null && kind !== '' && !/^pad$/i.test(String(kind))) return false;
	return p.type === undefined || p.type === 1 || p.type === 2;
}

/** 画布线图元的坐标是否可用（四个端点都是有限数） */
export function lineEndpointsUsable(l: any): boolean {
	const sx = l?.startX ?? l?.x, sy = l?.startY ?? l?.y;
	return [sx, sy, l?.endX, l?.endY].every(v => typeof v === 'number' && Number.isFinite(v));
}
