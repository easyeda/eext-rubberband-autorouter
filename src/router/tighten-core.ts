/**
 * tighten-core - 「拉紧已有布线」的纯几何内核（不依赖 eda.* / PlanningEngine）
 *
 * 三块：
 *  A. 精确世界：外网铜皮（焊盘真实形状/钻孔/过孔/走线（各自线宽）/静态障碍）建成
 *     「有符号距离函数 + 包围盒 + 包络圆」条目，均匀网格索引。候选图元的校验按
 *     中心线对条目的距离场做球面步进（sphere tracing），线/弧对圆盘/胶囊走闭式解，
 *     与圆弧半径、角度采样无关，误差 ≤ 0.002mil；按「条目组」报告最小间距，
 *     支持「既有违规不恶化」口径。
 *  B. 同伦保持的橡皮筋拉紧：把待拉紧链在固定点（端点/穿过的本网焊盘/T 接点）
 *     处切段，每段压平成折线，在「膨胀障碍边界采样点」上做三角形松弛：
 *     顶点 K 的三角形 AKB 内无障碍点 → 删 K；否则用 {A,B}∪三角内点 的 K 侧凸包
 *     链替换 K。只扫过不含障碍点的区域 → 同伦不变（不会跳过外网焊盘/走线）。
 *     收敛后的拐点映射回其来源包络圆 + 绕行方向，按精确切线 + 圆弧渲染。
 *  C. 圆角：剩余线-线 / 线-弧拐角（含偏心穿过焊盘处，切点须留在铜皮内）按「能过
 *     校验 A 的最大切线圆弧」倒角，切掉的角区不得含外网条目（否则同伦改变）。
 *     端点、T 接点、正穿焊盘中心的途经点不倒角（验收口径允许这些固定点上的尖角）。
 *
 * 单位 mil；弧 startAngle/sweep 为弧度，正 sweep = 数学正方向（x 右 y 上）逆时针。
 */

import type { RenderedPrim, EnginePad, PadShape, PlannedVia } from './planning-engine';
import type { RBObstacle } from './rubberband-engine';
import type { DRCRules } from './drc-rules';
import {
	pointSegDist, segSegDist, getTangents, polygonToRect, slotCoverHalfExtents,
	pointInPolygonPx, capsuleToCircles,
} from './rubberband-engine';

// ─────────────────────────────────────────────
// 常量
// ─────────────────────────────────────────────

const TAU = Math.PI * 2;
/**
 * 回写误差预算（mil）：回写器写出的每个图元相对求解器图元最多偏 0.1mil
 * （弧端点吸附 0.1mil 网格、扫角取 0.1° 整倍数、短隔离弧，见 tighten-writeback.ts 头注 E）。
 */
export const WRITER_ERR = 0.1;
/**
 * 校验底线：新采纳的几何离障碍至少 规则 + MIN_MARGIN（原几何本来就更近者例外，见 validatePrims）。
 * = 回写误差 0.1 + EDA 自身算术/取整余量 0.05。0.05 的依据（真机 run-fix1，旧口径渲染余量 0.05、校验底线规则−0.008）：
 * EDA DRC 实测 Track–Pad/Via 间距 5.915…5.946mil（规则 5.9843），即比求解器的位置偏 0.058…0.119mil（≥ 0.1 的那部分
 * 约 0.02 来自 EDA 自身口径）；0.15 − CHECK_TOL(0.008) = 0.142 ≥ 0.119 并留 0.02 余量。
 */
export const MIN_MARGIN = WRITER_ERR + 0.05;
/**
 * 渲染外扩余量：拉紧结果离障碍比规则多留的量。比校验底线再多 0.05，
 * 这 0.05 就是圆角/拟合允许「切进」的余量（与旧口径 0.05 一致），所以圆角容量不变。
 */
export const RENDER_MARGIN = MIN_MARGIN + 0.05;
/**
 * 步进器精度：primShapeMin 在「cap 以下」返回的最小值 m 满足 真值 ≤ m ≤ 真值 + TRACE_TAU。
 * 证明：步长 = v_k − m_k + 2·TRACE_TAU（v_k 当前采样值，m_k 已采样最小值）。距离函数沿弧长 1-Lipschitz，
 * 区间 [s_k, s_k+step] 内 f(t) ≥ max(v_k − (t−s_k), v_{k+1} − (s_k+step−t))，区间最小值 ≥ (v_k + v_{k+1} − step)/2
 * = (v_{k+1} + m_k)/2 − TRACE_TAU ≥ min(v_{k+1}, m_k) − TRACE_TAU。
 * （只用单侧 Lipschitz 会得到 2·TRACE_TAU；两侧夹逼后误差上界是 1·TRACE_TAU，因此 validatePrims 只减一个。）
 */
const TRACE_TAU = 0.002;
/** 校验容差：允许比「规则/既有间距」近这么多（验收口径 0.01，留 0.002 余量） */
const CHECK_TOL = 0.008;
/** 拓扑点相对起始路径的内缩量（起始路径必须严格在拓扑点围成的区域之外） */
const TOPO_SHRINK = 0.006;
/** 起始路径压平的弦高（拓扑内缩量必须大于它） */
const FLATTEN_SAG = 0.002;
/** 拓扑点采样弦高 */
const TOPO_SAG = 0.02;
/** 尖角口径（验收标准 1） */
export const SHARP_DEG = 30;
/** 非切向接缝口径 */
const KINK_RAD = (2 * Math.PI) / 180;

// ─────────────────────────────────────────────
// 基础几何
// ─────────────────────────────────────────────

export interface Pt { x: number; y: number }
export interface BBox { x0: number; y0: number; x1: number; y1: number }

const hyp = Math.hypot;
function mod2pi(a: number): number {
	const r = a % TAU;
	return r < 0 ? r + TAU : r;
}
function cross(ax: number, ay: number, bx: number, by: number): number { return ax * by - ay * bx; }
function orient(a: Pt, b: Pt, c: Pt): number { return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x); }

export function primStart(p: RenderedPrim): Pt {
	if (p.kind === 'line') return { x: p.x1 ?? 0, y: p.y1 ?? 0 };
	return { x: (p.cx ?? 0) + (p.r ?? 0) * Math.cos(p.startAngle ?? 0), y: (p.cy ?? 0) + (p.r ?? 0) * Math.sin(p.startAngle ?? 0) };
}
export function primEnd(p: RenderedPrim): Pt {
	if (p.kind === 'line') return { x: p.x2 ?? 0, y: p.y2 ?? 0 };
	const a = (p.startAngle ?? 0) + (p.sweep ?? 0);
	return { x: (p.cx ?? 0) + (p.r ?? 0) * Math.cos(a), y: (p.cy ?? 0) + (p.r ?? 0) * Math.sin(a) };
}
export function primLen(p: RenderedPrim): number {
	if (p.kind === 'line') return hyp((p.x2 ?? 0) - (p.x1 ?? 0), (p.y2 ?? 0) - (p.y1 ?? 0));
	return Math.abs(p.sweep ?? 0) * (p.r ?? 0);
}
export function chainLength(prims: RenderedPrim[]): number {
	let s = 0;
	for (const p of prims) s += primLen(p);
	return s;
}
/** 起点处的单位行进方向 */
export function primDirStart(p: RenderedPrim): Pt {
	if (p.kind === 'line') {
		const dx = (p.x2 ?? 0) - (p.x1 ?? 0), dy = (p.y2 ?? 0) - (p.y1 ?? 0);
		const l = hyp(dx, dy) || 1;
		return { x: dx / l, y: dy / l };
	}
	const a = p.startAngle ?? 0;
	return (p.sweep ?? 0) >= 0 ? { x: -Math.sin(a), y: Math.cos(a) } : { x: Math.sin(a), y: -Math.cos(a) };
}
/** 终点处的单位行进方向 */
export function primDirEnd(p: RenderedPrim): Pt {
	if (p.kind === 'line') return primDirStart(p);
	const a = (p.startAngle ?? 0) + (p.sweep ?? 0);
	return (p.sweep ?? 0) >= 0 ? { x: -Math.sin(a), y: Math.cos(a) } : { x: Math.sin(a), y: -Math.cos(a) };
}
/** 沿图元弧长 s 处的点 */
function primAt(p: RenderedPrim, s: number): Pt {
	if (p.kind === 'line') {
		const L = primLen(p);
		const t = L > 1e-12 ? s / L : 0;
		return { x: (p.x1 ?? 0) + ((p.x2 ?? 0) - (p.x1 ?? 0)) * t, y: (p.y1 ?? 0) + ((p.y2 ?? 0) - (p.y1 ?? 0)) * t };
	}
	const r = p.r ?? 0;
	const a = (p.startAngle ?? 0) + (r > 1e-12 ? Math.sign(p.sweep ?? 0) * s / r : 0);
	return { x: (p.cx ?? 0) + r * Math.cos(a), y: (p.cy ?? 0) + r * Math.sin(a) };
}
export function reversePrim(p: RenderedPrim): RenderedPrim {
	if (p.kind === 'line') return { ...p, x1: p.x2, y1: p.y2, x2: p.x1, y2: p.y1 };
	return { ...p, startAngle: (p.startAngle ?? 0) + (p.sweep ?? 0), sweep: -(p.sweep ?? 0) };
}
/** 图元在弧长 s 处切成两段 */
function splitPrim(p: RenderedPrim, s: number): [RenderedPrim, RenderedPrim] {
	const m = primAt(p, s);
	if (p.kind === 'line') {
		return [{ ...p, x2: m.x, y2: m.y }, { ...p, x1: m.x, y1: m.y }];
	}
	const r = p.r ?? 1;
	const ds = Math.sign(p.sweep ?? 0) * s / r;
	return [{ ...p, sweep: ds }, { ...p, startAngle: (p.startAngle ?? 0) + ds, sweep: (p.sweep ?? 0) - ds }];
}
function lineP(net: string, layer: number, a: Pt, b: Pt): RenderedPrim {
	return { kind: 'line', net, layer, x1: a.x, y1: a.y, x2: b.x, y2: b.y };
}

/** 点到圆弧（中心线）距离 */
function pointArcDist(x: number, y: number, cx: number, cy: number, r: number, a0: number, sw: number): number {
	const ang = Math.atan2(y - cy, x - cx);
	const d = sw >= 0 ? mod2pi(ang - a0) : mod2pi(a0 - ang);
	if (d <= Math.abs(sw)) return Math.abs(hyp(x - cx, y - cy) - r);
	const a1 = a0 + sw;
	return Math.min(
		hyp(x - cx - r * Math.cos(a0), y - cy - r * Math.sin(a0)),
		hyp(x - cx - r * Math.cos(a1), y - cy - r * Math.sin(a1)),
	);
}
/** 点到图元中心线距离 + 最近点弧长参数 */
export function primProject(p: RenderedPrim, x: number, y: number): { d: number; s: number } {
	if (p.kind === 'line') {
		const x1 = p.x1 ?? 0, y1 = p.y1 ?? 0, dx = (p.x2 ?? 0) - x1, dy = (p.y2 ?? 0) - y1;
		const L2 = dx * dx + dy * dy;
		let t = L2 > 1e-18 ? ((x - x1) * dx + (y - y1) * dy) / L2 : 0;
		t = Math.max(0, Math.min(1, t));
		return { d: hyp(x - x1 - dx * t, y - y1 - dy * t), s: t * Math.sqrt(L2) };
	}
	const cx = p.cx ?? 0, cy = p.cy ?? 0, r = p.r ?? 0, a0 = p.startAngle ?? 0, sw = p.sweep ?? 0;
	const ang = Math.atan2(y - cy, x - cx);
	const d = sw >= 0 ? mod2pi(ang - a0) : mod2pi(a0 - ang);
	if (d <= Math.abs(sw)) return { d: Math.abs(hyp(x - cx, y - cy) - r), s: d * r };
	const ps = primStart(p), pe = primEnd(p);
	const ds = hyp(x - ps.x, y - ps.y), de = hyp(x - pe.x, y - pe.y);
	return ds <= de ? { d: ds, s: 0 } : { d: de, s: primLen(p) };
}
function rectDist(lx: number, ly: number, hx: number, hy: number): number {
	const qx = Math.abs(lx) - hx, qy = Math.abs(ly) - hy;
	if (qx <= 0 && qy <= 0) return Math.max(qx, qy);
	return hyp(Math.max(qx, 0), Math.max(qy, 0));
}

export function primBBox(p: RenderedPrim): BBox {
	if (p.kind === 'line') {
		return {
			x0: Math.min(p.x1 ?? 0, p.x2 ?? 0), y0: Math.min(p.y1 ?? 0, p.y2 ?? 0),
			x1: Math.max(p.x1 ?? 0, p.x2 ?? 0), y1: Math.max(p.y1 ?? 0, p.y2 ?? 0),
		};
	}
	return arcBBox(p.cx ?? 0, p.cy ?? 0, p.r ?? 0, p.startAngle ?? 0, p.sweep ?? 0);
}
function arcBBox(cx: number, cy: number, r: number, a0: number, sw: number): BBox {
	const s = { x: cx + r * Math.cos(a0), y: cy + r * Math.sin(a0) };
	const e = { x: cx + r * Math.cos(a0 + sw), y: cy + r * Math.sin(a0 + sw) };
	const b: BBox = { x0: Math.min(s.x, e.x), y0: Math.min(s.y, e.y), x1: Math.max(s.x, e.x), y1: Math.max(s.y, e.y) };
	for (let k = 0; k < 4; k++) {
		const ang = (k * Math.PI) / 2;
		const d = sw >= 0 ? mod2pi(ang - a0) : mod2pi(a0 - ang);
		if (d <= Math.abs(sw)) {
			const x = cx + r * Math.cos(ang), y = cy + r * Math.sin(ang);
			b.x0 = Math.min(b.x0, x); b.x1 = Math.max(b.x1, x);
			b.y0 = Math.min(b.y0, y); b.y1 = Math.max(b.y1, y);
		}
	}
	return b;
}
function bboxUnion(a: BBox, b: BBox): BBox {
	return { x0: Math.min(a.x0, b.x0), y0: Math.min(a.y0, b.y0), x1: Math.max(a.x1, b.x1), y1: Math.max(a.y1, b.y1) };
}
function bboxGrow(a: BBox, g: number): BBox { return { x0: a.x0 - g, y0: a.y0 - g, x1: a.x1 + g, y1: a.y1 + g }; }
function bboxGap(a: BBox, b: BBox): number {
	const dx = Math.max(0, a.x0 - b.x1, b.x0 - a.x1);
	const dy = Math.max(0, a.y0 - b.y1, b.y0 - a.y1);
	return hyp(dx, dy);
}
export function chainBBox(prims: RenderedPrim[]): BBox {
	let b: BBox | null = null;
	for (const p of prims) b = b ? bboxUnion(b, primBBox(p)) : primBBox(p);
	return b ?? { x0: 0, y0: 0, x1: 0, y1: 0 };
}

// ─────────────────────────────────────────────
// A. 精确世界
// ─────────────────────────────────────────────

/**
 * 铜皮形状（rho = 铜皮半宽；静态障碍以「骨架 + 预膨胀半径」表示，rho = 0）。
 * dist() 为点到铜皮的有符号距离（内部为负）。
 */
export type Shape =
	| { t: 'disc'; x: number; y: number; rho: number }
	| { t: 'seg'; x1: number; y1: number; x2: number; y2: number; rho: number }
	| { t: 'arc'; cx: number; cy: number; r: number; a0: number; sw: number; rho: number }
	| { t: 'box'; x: number; y: number; hx: number; hy: number; c: number; s: number }
	| { t: 'poly'; pts: Pt[] };

export function shapeDist(sh: Shape, x: number, y: number): number {
	switch (sh.t) {
		case 'disc': return hyp(x - sh.x, y - sh.y) - sh.rho;
		case 'seg': return pointSegDist(x, y, sh.x1, sh.y1, sh.x2, sh.y2) - sh.rho;
		case 'arc': return pointArcDist(x, y, sh.cx, sh.cy, sh.r, sh.a0, sh.sw) - sh.rho;
		case 'box': {
			const dx = x - sh.x, dy = y - sh.y;
			return rectDist(dx * sh.c + dy * sh.s, -dx * sh.s + dy * sh.c, sh.hx, sh.hy);
		}
		case 'poly': {
			let d = Infinity;
			const n = sh.pts.length;
			for (let i = 0; i < n; i++) {
				const a = sh.pts[i], b = sh.pts[(i + 1) % n];
				d = Math.min(d, pointSegDist(x, y, a.x, a.y, b.x, b.y));
			}
			return pointInPolygonPx(x, y, sh.pts) ? -d : d;
		}
	}
}
export function shapeBBox(sh: Shape): BBox {
	switch (sh.t) {
		case 'disc': return { x0: sh.x - sh.rho, y0: sh.y - sh.rho, x1: sh.x + sh.rho, y1: sh.y + sh.rho };
		case 'seg': return bboxGrow({ x0: Math.min(sh.x1, sh.x2), y0: Math.min(sh.y1, sh.y2), x1: Math.max(sh.x1, sh.x2), y1: Math.max(sh.y1, sh.y2) }, sh.rho);
		case 'arc': return bboxGrow(arcBBox(sh.cx, sh.cy, sh.r, sh.a0, sh.sw), sh.rho);
		case 'box': {
			const ex = Math.abs(sh.hx * sh.c) + Math.abs(sh.hy * sh.s);
			const ey = Math.abs(sh.hx * sh.s) + Math.abs(sh.hy * sh.c);
			return { x0: sh.x - ex, y0: sh.y - ey, x1: sh.x + ex, y1: sh.y + ey };
		}
		case 'poly': {
			const b: BBox = { x0: Infinity, y0: Infinity, x1: -Infinity, y1: -Infinity };
			for (const p of sh.pts) { b.x0 = Math.min(b.x0, p.x); b.y0 = Math.min(b.y0, p.y); b.x1 = Math.max(b.x1, p.x); b.y1 = Math.max(b.y1, p.y); }
			return b;
		}
	}
}
/**
 * 形状上的一组探测点（判断「条目是否（部分）落在切角区内」用）：骨架/轮廓的特征点
 * （圆心、线段端点+中点、弧端点+中点、矩形四角+中心、多边形顶点），铜半宽 rho>0 时再加四向极值点。
 */
function shapeProbePts(sh: Shape): Pt[] {
	const out: Pt[] = [];
	const withRho = (c: Pt, rho: number): void => {
		out.push(c);
		if (rho > 0) {
			out.push({ x: c.x + rho, y: c.y }, { x: c.x - rho, y: c.y }, { x: c.x, y: c.y + rho }, { x: c.x, y: c.y - rho });
		}
	};
	switch (sh.t) {
		case 'disc': withRho({ x: sh.x, y: sh.y }, sh.rho); break;
		case 'seg':
			withRho({ x: sh.x1, y: sh.y1 }, sh.rho);
			withRho({ x: sh.x2, y: sh.y2 }, sh.rho);
			withRho({ x: (sh.x1 + sh.x2) / 2, y: (sh.y1 + sh.y2) / 2 }, sh.rho);
			break;
		case 'arc': {
			for (const f of [0, 0.25, 0.5, 0.75, 1]) {
				const a = sh.a0 + sh.sw * f;
				withRho({ x: sh.cx + sh.r * Math.cos(a), y: sh.cy + sh.r * Math.sin(a) }, sh.rho);
			}
			break;
		}
		case 'box': {
			out.push({ x: sh.x, y: sh.y });
			for (const [sx, sy] of [[1, 1], [-1, 1], [-1, -1], [1, -1]]) {
				const lx = sx * sh.hx, ly = sy * sh.hy;
				out.push({ x: sh.x + lx * sh.c - ly * sh.s, y: sh.y + lx * sh.s + ly * sh.c });
			}
			break;
		}
		case 'poly':
			for (const p of sh.pts) out.push(p);
			break;
	}
	return out;
}
/** 形状中心（固定点投影参考） */
export function shapeCenter(sh: Shape): Pt {
	switch (sh.t) {
		case 'disc': case 'box': return { x: sh.x, y: sh.y };
		case 'seg': return { x: (sh.x1 + sh.x2) / 2, y: (sh.y1 + sh.y2) / 2 };
		case 'arc': return { x: sh.cx + sh.r * Math.cos(sh.a0 + sh.sw / 2), y: sh.cy + sh.r * Math.sin(sh.a0 + sh.sw / 2) };
		case 'poly': return sh.pts[0];
	}
}

/** 世界条目：中心线到 shape 的距离必须 ≥ R0 + w/2（w = 被校验走线的实际线宽） */
export interface WorldItem {
	id: number;
	/** 条目组：同一物理对象（一条 run 的全部图元 / 一个焊盘）共享，最小间距按组报告 */
	group: string;
	net?: string;
	layer?: number;
	kind: 'track' | 'pad' | 'via' | 'hole' | 'static';
	R0: number;
	shape: Shape;
	bbox: BBox;
}

/** 均匀网格空间索引 */
export class TightenWorld {
	items: Array<WorldItem | undefined> = [];
	maxR0 = 0;
	private cells = new Map<number, number[]>();
	private big: number[] = [];
	private mark: number[] = [];
	private stamp = 1;
	private readonly cell = 40;
	private readonly groups = new Map<string, Set<number>>();

	private key(ix: number, iy: number): number { return ix * 1000003 + iy; }
	private range(b: BBox): [number, number, number, number] {
		const c = this.cell;
		return [Math.floor(b.x0 / c), Math.floor(b.y0 / c), Math.floor(b.x1 / c), Math.floor(b.y1 / c)];
	}

	add(it: Omit<WorldItem, 'id' | 'bbox'> & { bbox?: BBox }): WorldItem {
		const bbox = it.bbox ?? shapeBBox(it.shape);
		if (![bbox.x0, bbox.y0, bbox.x1, bbox.y1, it.R0].every(Number.isFinite)) {
			throw new Error('tighten world: non-finite item');
		}
		const item: WorldItem = { ...it, bbox, id: this.items.length };
		this.items.push(item);
		this.mark.push(0);
		this.maxR0 = Math.max(this.maxR0, item.R0);
		const [ix0, iy0, ix1, iy1] = this.range(bbox);
		if ((ix1 - ix0 + 1) * (iy1 - iy0 + 1) > 4000) {
			this.big.push(item.id);
		} else {
			for (let ix = ix0; ix <= ix1; ix++) {
				for (let iy = iy0; iy <= iy1; iy++) {
					const k = this.key(ix, iy);
					const arr = this.cells.get(k);
					if (arr) arr.push(item.id); else this.cells.set(k, [item.id]);
				}
			}
		}
		let g = this.groups.get(item.group);
		if (!g) { g = new Set(); this.groups.set(item.group, g); }
		g.add(item.id);
		return item;
	}

	remove(id: number): void {
		const it = this.items[id];
		if (!it) return;
		this.items[id] = undefined;
		this.groups.get(it.group)?.delete(id);
		const [ix0, iy0, ix1, iy1] = this.range(it.bbox);
		if ((ix1 - ix0 + 1) * (iy1 - iy0 + 1) > 4000) {
			this.big = this.big.filter(q => q !== id);
			return;
		}
		for (let ix = ix0; ix <= ix1; ix++) {
			for (let iy = iy0; iy <= iy1; iy++) {
				const k = this.key(ix, iy);
				const arr = this.cells.get(k);
				if (!arr) continue;
				const j = arr.indexOf(id);
				if (j >= 0) arr.splice(j, 1);
			}
		}
	}

	removeGroup(group: string): void {
		const g = this.groups.get(group);
		if (!g) return;
		for (const id of [...g]) this.remove(id);
	}

	groupItems(group: string): WorldItem[] {
		const g = this.groups.get(group);
		if (!g) return [];
		const out: WorldItem[] = [];
		for (const id of g) { const it = this.items[id]; if (it) out.push(it); }
		return out;
	}

	groupItemsByNet(net: string): WorldItem[] {
		const out: WorldItem[] = [];
		for (const it of this.items) if (it && it.net === net) out.push(it);
		return out;
	}

	/** 包围盒相交的条目（去重） */
	query(b: BBox, cb: (it: WorldItem) => void): void {
		const st = ++this.stamp;
		const [ix0, iy0, ix1, iy1] = this.range(b);
		const visit = (id: number) => {
			if (this.mark[id] === st) return;
			this.mark[id] = st;
			const it = this.items[id];
			if (!it) return;
			if (it.bbox.x0 > b.x1 || it.bbox.x1 < b.x0 || it.bbox.y0 > b.y1 || it.bbox.y1 < b.y0) return;
			cb(it);
		};
		if ((ix1 - ix0 + 1) * (iy1 - iy0 + 1) > 20000) {
			for (const it of this.items) if (it) visit(it.id);
			return;
		}
		for (let ix = ix0; ix <= ix1; ix++) {
			for (let iy = iy0; iy <= iy1; iy++) {
				const arr = this.cells.get(this.key(ix, iy));
				if (arr) for (const id of arr) visit(id);
			}
		}
		for (const id of this.big) visit(id);
	}
}

/**
 * 线段到矩形的最小有符号距离：到凸集的有符号距离是凸函数，沿线段做三分搜索即得真最小值
 * （比球面步进快得多：贴着/逼近矩形的长线段用步进器要走 L/0.004 步）。返回值是某个采样点的函数值，不会低于真值。
 */
function lineBoxMin(p: RenderedPrim, sh: Extract<Shape, { t: 'box' }>): number {
	const x1 = p.x1 ?? 0, y1 = p.y1 ?? 0, dx = (p.x2 ?? 0) - x1, dy = (p.y2 ?? 0) - y1;
	const f = (t: number): number => shapeDist(sh, x1 + dx * t, y1 + dy * t);
	let lo = 0, hi = 1;
	let best = Math.min(f(0), f(1));
	for (let i = 0; i < 70; i++) {
		const a = lo + (hi - lo) / 3, b = hi - (hi - lo) / 3;
		const fa = f(a), fb = f(b);
		if (fa < best) best = fa;
		if (fb < best) best = fb;
		if (fa < fb) hi = b; else lo = a;
	}
	return best;
}

/**
 * 图元中心线到形状的最小有符号距离（内部为负）。
 *
 * cap = 精度阈值：中心线上距离函数 ≥ cap 的部分只被大步跨过（球面步进，距离函数 1-Lipschitz）。
 *  - 真最小值 < cap：返回 m，真值 ∈ [m − TRACE_TAU, m]（线/弧对圆盘/胶囊走闭式解，精确）；
 *  - 真最小值 ≥ cap：返回值 ≥ cap（具体值不精确，只保证「不低于 cap」）。
 * 因此：
 *  - cap = +Infinity → 全精度，求真最小值（chainShapeMin 等「求值」场合必须这么用）；
 *  - 只想判断「是否低于阈值 T」时传 T（越小越快）；
 *  - 绝不能传很小的负数（如 −1e9）：步长 v − cap 会变成 v + 1e9，一步跳到末端，只采样了两个端点。
 * 迭代守卫（2,000,000 步）耗尽时返回保守下界 min(m, v_last − 剩余弧长)，不会高估。
 */
export function primShapeMin(p: RenderedPrim, sh: Shape, cap: number): number {
	if (p.kind === 'line') {
		if (sh.t === 'disc') return pointSegDist(sh.x, sh.y, p.x1 ?? 0, p.y1 ?? 0, p.x2 ?? 0, p.y2 ?? 0) - sh.rho;
		if (sh.t === 'seg') return segSegDist(p.x1 ?? 0, p.y1 ?? 0, p.x2 ?? 0, p.y2 ?? 0, sh.x1, sh.y1, sh.x2, sh.y2) - sh.rho;
		if (sh.t === 'box') return lineBoxMin(p, sh);
	} else if (sh.t === 'disc') {
		return pointArcDist(sh.x, sh.y, p.cx ?? 0, p.cy ?? 0, p.r ?? 0, p.startAngle ?? 0, p.sweep ?? 0) - sh.rho;
	}
	const L = primLen(p);
	let s = 0, m = Infinity, v = Infinity;
	for (let guard = 0; guard < 2_000_000; guard++) {
		const q = primAt(p, s);
		v = shapeDist(sh, q.x, q.y);
		if (v < m) m = v;
		if (s >= L) return m;
		const step = Math.max(v - cap, v - m + 2 * TRACE_TAU);
		s = Math.min(L, s + step);
	}
	// 守卫耗尽：[s, L] 还没扫过，1-Lipschitz 给出该段下界 v_last − (L − s_last)
	return Math.min(m, v - (L - s));
}

// ─── 条目构建 ───

/** EDA ELLIPSE 单位归一化（与 rubberband-engine.normalizeEllipseUnits 同口径） */
function normalizeEllipse(shape: any[]): any[] {
	if (!Array.isArray(shape) || String(shape[0] ?? '').toUpperCase() !== 'ELLIPSE') return shape;
	const a = shape[1], b = shape[2];
	if (!(a > 0) || !(b > 0)) return shape;
	if (Math.abs(a * 39.3701 - b) <= Math.max(0.5, b * 0.02)) return ['ELLIPSE', b, b];
	if (Math.abs(a - b) <= 0.01 && a < 1) return ['ELLIPSE', a * 39.3701, b * 39.3701];
	return shape;
}

/**
 * 焊盘铜皮 → Shape（与 padShapeDistance 同一套解析；ROUNDRECT 按 RECT 处理——
 * 圆角半径未知，按直角算铜皮只会更保守；非等轴 ELLIPSE 用外包胶囊）。
 */
export function padToShape(pad: EnginePad, ps: PadShape | undefined): Shape {
	const fallback: Shape = { t: 'disc', x: pad.x, y: pad.y, rho: Math.max(0, pad.radius || 0) };
	const raw = ps?.pad;
	if (!Array.isArray(raw)) return fallback;
	const slot = slotCoverHalfExtents({ pad: raw, hole: ps?.hole ?? null, holeRotation: ps?.holeRotation ?? 0 });
	if (slot) {
		const a = (slot.rotDeg * Math.PI) / 180;
		return { t: 'box', x: pad.x, y: pad.y, hx: slot.hx, hy: slot.hy, c: Math.cos(a), s: Math.sin(a) };
	}
	let shape = normalizeEllipse(raw);
	let rotDeg = ps?.rotation ?? pad.rotation ?? 0;
	if (String(shape[0] ?? '').toUpperCase() === 'POLYGON') {
		const rect = polygonToRect(raw);
		if (!rect) return fallback;
		shape = ['RECT', rect.w, rect.h];
		rotDeg = rect.rotationDeg;
	}
	const w = Number(shape[1]), h = Number(shape[2]);
	if (!(w > 0) || !(h > 0)) {
		return { t: 'disc', x: pad.x, y: pad.y, rho: 0 };
	}
	const name = String(shape[0] ?? 'ELLIPSE').toUpperCase();
	const rot = (rotDeg * Math.PI) / 180;
	const c = Math.cos(rot), s = Math.sin(rot);
	if (name === 'RECT' || name === 'ROUNDRECT') return { t: 'box', x: pad.x, y: pad.y, hx: w / 2, hy: h / 2, c, s };
	if (Math.abs(w - h) < 0.01) return { t: 'disc', x: pad.x, y: pad.y, rho: w / 2 };
	// OVAL / 非等轴 ELLIPSE：胶囊（长轴方向的两端帽圆心）
	const r = Math.min(w, h) / 2;
	const half = Math.max(w, h) / 2 - r;
	const ax = h >= w ? { x: 0, y: half } : { x: half, y: 0 };
	const gx = ax.x * c - ax.y * s, gy = ax.x * s + ax.y * c;
	return { t: 'seg', x1: pad.x - gx, y1: pad.y - gy, x2: pad.x + gx, y2: pad.y + gy, rho: r };
}

export function primToShape(p: RenderedPrim, rho: number): Shape {
	if (p.kind === 'line') return { t: 'seg', x1: p.x1 ?? 0, y1: p.y1 ?? 0, x2: p.x2 ?? 0, y2: p.y2 ?? 0, rho };
	return { t: 'arc', cx: p.cx ?? 0, cy: p.cy ?? 0, r: p.r ?? 0, a0: p.startAngle ?? 0, sw: p.sweep ?? 0, rho };
}

/** 焊盘铜皮 + 钻孔条目 */
export function padItems(world: TightenWorld, pads: EnginePad[], padShapes: Record<string, PadShape>, drc: DRCRules): void {
	pads.forEach((p, i) => {
		if (!Number.isFinite(p.x) || !Number.isFinite(p.y)) return;
		const ps = padShapes[`${p.x.toFixed(2)},${p.y.toFixed(2)}`];
		const allLayers = p.layer === 12 || p.padType === 'th';
		world.add({
			group: `pad:${i}`, net: p.net || undefined, layer: allLayers ? undefined : p.layer,
			kind: 'pad', R0: drc.trackToPadMil, shape: padToShape(p, ps),
		});
		const hd = Math.max(0, Number(p.holeDiameter) || 0);
		if (hd > 0 && allLayers) {
			world.add({
				group: `hole:${i}`, net: p.net || undefined, layer: undefined,
				kind: 'hole', R0: drc.trackToPadMil, shape: { t: 'disc', x: p.x, y: p.y, rho: hd / 2 },
			});
		}
	});
}

export function viaItems(world: TightenWorld, vias: PlannedVia[], drc: DRCRules): void {
	vias.forEach((v, i) => {
		if (!Number.isFinite(v.x) || !Number.isFinite(v.y)) return;
		world.add({
			group: `via:${i}`, net: v.net || undefined, layer: undefined,
			kind: 'via', R0: drc.trackToPadMil, shape: { t: 'disc', x: v.x, y: v.y, rho: Math.max(0, v.dia / 2) },
		});
	});
}

export function trackItems(
	world: TightenWorld, prims: RenderedPrim[], width: number, net: string, layer: number,
	group: string, drc: DRCRules,
): WorldItem[] {
	const out: WorldItem[] = [];
	for (const p of prims) {
		if (primLen(p) < 1e-9 && p.kind === 'arc') continue;
		out.push(world.add({
			group, net: net || undefined, layer, kind: 'track', R0: drc.trackToTrackMil,
			shape: primToShape(p, width / 2),
		}));
	}
	return out;
}

/**
 * 静态障碍（baseWorld，预膨胀圆群）→ 精确条目。
 *
 * baseWorld 的折线/走线障碍是「顶点圆 + 沿线阻挡圆（间距 1.8r）」：两圆之间腰部只有
 * 0.44r 厚——按圆群校验会把真实胶囊中段漏掉。这里按组还原骨架：相邻顶点圆若沿线
 * 阻挡圆齐全（capsuleToCircles 逐个命中）就还原为一条骨架线段（胶囊）；fill 组还原为
 * 闭合多边形（含内部）。其余圆原样保留。
 * isCopy(group, obs) 为真的组（被拉紧走线在 baseWorld 里的旧副本）整体丢弃。
 */
export function staticItems(
	world: TightenWorld, baseWorld: RBObstacle[], drc: DRCRules,
	isCopy: (group: string, obs: RBObstacle[]) => boolean,
): void {
	const tw = drc.traceWidthMil;
	const groups = new Map<string, RBObstacle[]>();
	baseWorld.forEach((o, i) => {
		if (!o || o.soft) return;
		if (![o.x, o.y, o.r].every(Number.isFinite)) return;
		const g = o.group ?? `__${i}`;
		const arr = groups.get(g);
		if (arr) arr.push(o); else groups.set(g, [o]);
	});
	let gi = 0;
	for (const [g, obs] of groups) {
		gi++;
		if (isCopy(g, obs)) continue;
		const add = (shape: Shape, r: number, o: RBObstacle) => world.add({
			group: `st:${gi}`, net: o.net || undefined, layer: o.layer, kind: 'static',
			R0: r - tw / 2, shape,
		});
		const prefix = g.split(':')[0];
		const polyish = ['line', 'arc', 'poly', 'trk', 'board', 'fill'].includes(prefix);
		const caps = obs.filter(o => o.kind === 'capsule');
		for (const o of caps) {
			add({ t: 'seg', x1: o.x1 ?? o.x, y1: o.y1 ?? o.y, x2: o.x2 ?? o.x, y2: o.y2 ?? o.y, rho: 0 }, o.r, o);
		}
		const circles = obs.filter(o => o.kind !== 'capsule');
		if (!polyish) {
			for (const o of circles) add({ t: 'disc', x: o.x, y: o.y, rho: 0 }, o.r, o);
			continue;
		}
		const verts = circles.filter(o => !o.blockerOnly);
		const blockers = circles.filter(o => o.blockerOnly);
		const bkey = (x: number, y: number, r: number) => `${x.toFixed(2)},${y.toFixed(2)},${r.toFixed(2)}`;
		const bset = new Set(blockers.map(o => bkey(o.x, o.y, o.r)));
		const edgeOk = (a: RBObstacle, b: RBObstacle): boolean => {
			if (Math.abs(a.r - b.r) > 1e-6) return false;
			for (const c of capsuleToCircles(a.x, a.y, b.x, b.y, a.r)) {
				if (!bset.has(bkey(c.x, c.y, a.r))) return false;
			}
			return true;
		};
		// 圆弧副本：弦→弧外凸补偿（弦高）
		let sag = 0;
		if (prefix === 'arc' && verts.length >= 3) {
			const p0 = verts[0], p1 = verts[Math.floor(verts.length / 2)], p2 = verts[verts.length - 1];
			const a = hyp(p1.x - p0.x, p1.y - p0.y), b = hyp(p2.x - p1.x, p2.y - p1.y), c = hyp(p2.x - p0.x, p2.y - p0.y);
			const area2 = Math.abs(orient(p0, p1, p2));
			if (area2 > 1e-9) {
				const R = (a * b * c) / (2 * area2);
				const chord = hyp(verts[1].x - verts[0].x, verts[1].y - verts[0].y);
				sag = R - Math.sqrt(Math.max(0, R * R - chord * chord / 4));
			}
		}
		const edges: Array<[RBObstacle, RBObstacle]> = [];
		for (let k = 0; k + 1 < verts.length; k++) if (edgeOk(verts[k], verts[k + 1])) edges.push([verts[k], verts[k + 1]]);
		if (prefix === 'fill' && verts.length >= 3) {
			// 闭合多边形（含内部）：区域 {d < r} = 多边形 ∪ 边胶囊
			const pts = verts.map(o => ({ x: o.x, y: o.y }));
			add({ t: 'poly', pts }, verts[0].r, verts[0]);
			for (const o of blockers) {
				if (pointInPolygonPx(o.x, o.y, pts)) continue;
				let covered = false;
				for (let k = 0; k < pts.length && !covered; k++) {
					const a = pts[k], b = pts[(k + 1) % pts.length];
					if (pointSegDist(o.x, o.y, a.x, a.y, b.x, b.y) < 1e-3 && o.r <= verts[0].r + 1e-6) covered = true;
				}
				if (!covered) add({ t: 'disc', x: o.x, y: o.y, rho: 0 }, o.r, o);
			}
			continue;
		}
		const used = new Set<RBObstacle>();
		for (const [a, b] of edges) {
			add({ t: 'seg', x1: a.x, y1: a.y, x2: b.x, y2: b.y, rho: 0 }, a.r + sag, a);
			used.add(a); used.add(b);
		}
		for (const o of verts) if (!used.has(o)) add({ t: 'disc', x: o.x, y: o.y, rho: 0 }, o.r, o);
		for (const o of blockers) {
			let covered = false;
			for (const [a, b] of edges) {
				if (o.r <= a.r + sag + 1e-6 && pointSegDist(o.x, o.y, a.x, a.y, b.x, b.y) < 1e-3) { covered = true; break; }
			}
			if (!covered) add({ t: 'disc', x: o.x, y: o.y, rho: 0 }, o.r, o);
		}
	}
}

// ─────────────────────────────────────────────
// 校验环境
// ─────────────────────────────────────────────

/** 一条待拉紧走线的校验上下文 */
export interface RunEnv {
	world: TightenWorld;
	net: string;
	layer: number;
	width: number;
	/** 自身条目组（不参与校验）；可多个（整网改写时整网排除） */
	selfGroups: Set<string>;
	/** 原始几何（任务开始时）对条目组的最小间距余量（slack = 距离 − R），懒计算 */
	origSlack: (group: string) => number;
}

function relevant(env: RunEnv, it: WorldItem): boolean {
	if (it.net !== undefined && it.net === env.net) return false;
	if (env.selfGroups.has(it.group)) return false;
	if (it.layer !== undefined && it.layer !== env.layer) return false;
	return true;
}

/** 对 items 的余量（按组取最小），只精确计算 < cap 的部分 */
export function slackByGroup(
	world: TightenWorld, prims: RenderedPrim[], width: number, filter: (it: WorldItem) => boolean,
	cap: number, out = new Map<string, number>(),
): Map<string, number> {
	const grow = width / 2 + world.maxR0 + cap + 0.5;
	for (const p of prims) {
		const pb = primBBox(p);
		world.query(bboxGrow(pb, grow), it => {
			if (!filter(it)) return;
			const R = it.R0 + width / 2;
			const lb = bboxGap(pb, it.bbox) - R;
			const prev = out.get(it.group) ?? Infinity;
			if (lb >= Math.min(prev, cap)) {
				if (!out.has(it.group)) out.set(it.group, Math.max(lb, cap));
				return;
			}
			const m = primShapeMin(p, it.shape, R + Math.min(prev, cap)) - R;
			if (m < prev) out.set(it.group, m);
		});
	}
	return out;
}

/** 原始几何对任意 items 组的余量（job 用于 origSlack 的懒计算） */
export function slackOfPrimsVsItems(prims: RenderedPrim[], width: number, items: WorldItem[]): number {
	let best = Infinity;
	for (const p of prims) {
		const pb = primBBox(p);
		for (const it of items) {
			const R = it.R0 + width / 2;
			const lb = bboxGap(pb, it.bbox) - R;
			if (lb >= best || lb > 2) continue;
			const m = primShapeMin(p, it.shape, R + Math.min(best, 2)) - R;
			if (m < best) best = m;
		}
	}
	return Number.isFinite(best) ? best : 1e9;
}

/**
 * 条目额外余量：对方是另一条（也可能被回写、也带 0.1mil 误差）的走线时，两侧误差叠加，再加一份回写误差。
 * 焊盘/过孔/孔/静态铜不会被拉紧回写，不加。
 */
export function itemExtraMargin(it: WorldItem): number {
	return it.kind === 'track' ? WRITER_ERR : 0;
}

/**
 * 校验 A：候选图元对全部外网条目
 *   原余量 so ≥ 0（原几何合规）：余量 ≥ MIN_MARGIN（+ 走线额外 WRITER_ERR） − CHECK_TOL，
 *     即写出后（≤ 0.1mil 误差 + EDA 口径）仍不低于规则；
 *   原余量 so < 0（原几何已违规）：余量 ≥ so − CHECK_TOL（不比原来更糟；回写误差无法再补偿）。
 * 返回失败的条目组。
 */
export function validatePrims(env: RunEnv, prims: RenderedPrim[], early = false): { ok: boolean; bad: Set<string> } {
	const bad = new Set<string>();
	const w = env.width;
	const grow = w / 2 + env.world.maxR0 + MIN_MARGIN + WRITER_ERR + 0.1;
	for (const p of prims) {
		if (p.layer !== undefined && p.layer !== env.layer) { bad.add('layer'); continue; }
		const pb = primBBox(p);
		env.world.query(bboxGrow(pb, grow), it => {
			if (early && bad.size) return;
			if (!relevant(env, it) || bad.has(it.group)) return;
			const R = it.R0 + w / 2;
			const T0 = R + MIN_MARGIN + itemExtraMargin(it) - CHECK_TOL;
			if (bboxGap(pb, it.bbox) >= T0 + 0.01) return;
			const m = primShapeMin(p, it.shape, T0) - TRACE_TAU;
			if (m >= T0) return;
			const so = env.origSlack(it.group);
			// 原几何已违规：不比原来更糟即可；合规：必须达到 T0
			if (so < 0 && m >= R + so - CHECK_TOL) return;
			bad.add(it.group);
		});
		if (early && bad.size) break;
	}
	return { ok: bad.size === 0, bad };
}

// ─────────────────────────────────────────────
// B. 拓扑点 / 包络圆
// ─────────────────────────────────────────────

/** 拓扑点：膨胀障碍边界上的采样点；wid = 来源包络圆序号（-1 = 凹边，不可绕） */
interface TPt { x: number; y: number; item: number; wid: number }

export /** 单段圆弧采样步数的安全上限（只防病态输入；正常板子半径 ≪ 1e5mil 时远低于它） */
const MAX_ARC_STEPS = 200_000;
/** 半径 s、扫角 span 的圆弧按弦高 ≤ sag 采样所需的段数（弦高界对任意半径都成立，不封顶到 720） */
export function arcSteps(s: number, span: number, sag: number): number {
	if (s <= sag) return Math.max(1, Math.ceil(span / (Math.PI / 2)));
	const d = 2 * Math.acos(1 - sag / s);
	return Math.min(MAX_ARC_STEPS, Math.max(1, Math.ceil(span / d)));
}
/** 圆 (cx,cy,s) 上从 a0 逆时针到 a0+span 的采样点（含两端） */
function pushArcPts(out: TPt[], cx: number, cy: number, s: number, a0: number, span: number, sag: number, item: number, wid: number): void {
	if (s <= 1e-9) { out.push({ x: cx, y: cy, item, wid }); return; }
	const n = arcSteps(s, span, sag);
	for (let k = 0; k <= n; k++) {
		const a = a0 + (span * k) / n;
		out.push({ x: cx + s * Math.cos(a), y: cy + s * Math.sin(a), item, wid });
	}
}

/**
 * 条目在总外扩 W（离铜皮）下的拓扑点。点都落在 {d < W} 的边界上，
 * 凸片段取内接多边形（多边形 ⊂ 真区域），凹边（弧内侧）加密到弦高 ≤ FLATTEN_SAG。
 */
function itemTopoPoints(it: WorldItem, W: number, out: TPt[]): void {
	const sh = it.shape;
	const id = it.id;
	switch (sh.t) {
		case 'disc': {
			const s = sh.rho + W;
			if (s <= 1e-9) { out.push({ x: sh.x, y: sh.y, item: id, wid: 0 }); return; }
			const n = arcSteps(s, TAU, TOPO_SAG);
			for (let k = 0; k < n; k++) {
				const a = (TAU * k) / n;
				out.push({ x: sh.x + s * Math.cos(a), y: sh.y + s * Math.sin(a), item: id, wid: 0 });
			}
			return;
		}
		case 'seg': {
			const s = Math.max(0, sh.rho + W);
			const L = hyp(sh.x2 - sh.x1, sh.y2 - sh.y1);
			if (L < 1e-9) { pushArcPts(out, sh.x1, sh.y1, s, 0, TAU, TOPO_SAG, id, 0); return; }
			const th = Math.atan2(sh.y2 - sh.y1, sh.x2 - sh.x1);
			pushArcPts(out, sh.x1, sh.y1, s, th + Math.PI / 2, Math.PI, TOPO_SAG, id, 0);
			pushArcPts(out, sh.x2, sh.y2, s, th - Math.PI / 2, Math.PI, TOPO_SAG, id, 1);
			return;
		}
		case 'box': {
			const s = Math.max(0, W);
			const hx = Math.max(0, sh.hx + Math.min(0, W)), hy = Math.max(0, sh.hy + Math.min(0, W));
			const corners = [[1, 1], [-1, 1], [-1, -1], [1, -1]];
			corners.forEach(([sx, sy], k) => {
				const lx = sx * hx, ly = sy * hy;
				const gx = sh.x + lx * sh.c - ly * sh.s, gy = sh.y + lx * sh.s + ly * sh.c;
				const rot = Math.atan2(sh.s, sh.c);
				pushArcPts(out, gx, gy, s, rot + (k * Math.PI) / 2, Math.PI / 2, TOPO_SAG, id, k);
			});
			return;
		}
		case 'arc': {
			const s = Math.max(0, sh.rho + W);
			// 统一成逆时针：b0 → b0+span
			const span = Math.abs(sh.sw);
			const b0 = sh.sw >= 0 ? sh.a0 : sh.a0 + sh.sw;
			const b1 = b0 + span;
			// 端点包络圆序号：wid 0 = 原弧起点、1 = 原弧终点
			const w0 = sh.sw >= 0 ? 0 : 1, w1 = 1 - w0;
			const E0 = { x: sh.cx + sh.r * Math.cos(b0), y: sh.cy + sh.r * Math.sin(b0) };
			const E1 = { x: sh.cx + sh.r * Math.cos(b1), y: sh.cy + sh.r * Math.sin(b1) };
			if (s <= 1e-9) {
				pushArcPts(out, sh.cx, sh.cy, sh.r, b0, span, FLATTEN_SAG, id, -1);
				return;
			}
			pushArcPts(out, sh.cx, sh.cy, sh.r + s, b0, span, TOPO_SAG, id, 2);
			pushArcPts(out, E1.x, E1.y, s, b1, Math.PI, TOPO_SAG, id, w1);
			if (sh.r - s > 0.3) {
				const n = arcSteps(sh.r - s, span, FLATTEN_SAG);
				for (let k = 0; k <= n; k++) {
					const a = b1 - (span * k) / n;
					out.push({ x: sh.cx + (sh.r - s) * Math.cos(a), y: sh.cy + (sh.r - s) * Math.sin(a), item: id, wid: -1 });
				}
			} else {
				out.push({ x: sh.cx, y: sh.cy, item: id, wid: -1 });
			}
			pushArcPts(out, E0.x, E0.y, s, b0 + Math.PI, Math.PI, TOPO_SAG, id, w0);
			return;
		}
		case 'poly': {
			const s = Math.max(0, W);
			const n = sh.pts.length;
			for (let k = 0; k < n; k++) {
				const a = sh.pts[k], b = sh.pts[(k + 1) % n];
				const L = hyp(b.x - a.x, b.y - a.y);
				if (L < 1e-9) continue;
				const th = Math.atan2(b.y - a.y, b.x - a.x);
				pushArcPts(out, a.x, a.y, s, th + Math.PI / 2, Math.PI, TOPO_SAG, id, k);
				pushArcPts(out, b.x, b.y, s, th - Math.PI / 2, Math.PI, TOPO_SAG, id, (k + 1) % n);
			}
			return;
		}
	}
}

/** 包络圆（渲染用）：总外扩 W 下的圆心/半径 */
function wrapCircle(it: WorldItem, wid: number, W: number): { x: number; y: number; r: number } | null {
	const sh = it.shape;
	switch (sh.t) {
		case 'disc': return { x: sh.x, y: sh.y, r: Math.max(0, sh.rho + W) };
		case 'seg': return wid === 0 ? { x: sh.x1, y: sh.y1, r: Math.max(0, sh.rho + W) } : { x: sh.x2, y: sh.y2, r: Math.max(0, sh.rho + W) };
		case 'box': {
			const corners = [[1, 1], [-1, 1], [-1, -1], [1, -1]];
			const cc = corners[wid];
			if (!cc) return null;
			const hx = Math.max(0, sh.hx + Math.min(0, W)), hy = Math.max(0, sh.hy + Math.min(0, W));
			const lx = cc[0] * hx, ly = cc[1] * hy;
			return { x: sh.x + lx * sh.c - ly * sh.s, y: sh.y + lx * sh.s + ly * sh.c, r: Math.max(0, W) };
		}
		case 'arc': {
			if (wid === 2) return { x: sh.cx, y: sh.cy, r: sh.r + Math.max(0, sh.rho + W) };
			const a = wid === 0 ? sh.a0 : sh.a0 + sh.sw;
			return { x: sh.cx + sh.r * Math.cos(a), y: sh.cy + sh.r * Math.sin(a), r: Math.max(0, sh.rho + W) };
		}
		case 'poly': {
			const p = sh.pts[wid];
			return p ? { x: p.x, y: p.y, r: Math.max(0, W) } : null;
		}
	}
}

/** 拓扑点网格 */
class PtGrid {
	private cells = new Map<number, number[]>();
	constructor(readonly pts: TPt[], private readonly cell: number) {
		pts.forEach((p, i) => {
			const k = this.key(Math.floor(p.x / cell), Math.floor(p.y / cell));
			const arr = this.cells.get(k);
			if (arr) arr.push(i); else this.cells.set(k, [i]);
		});
	}
	private key(ix: number, iy: number): number { return ix * 1000003 + iy; }
	forBox(x0: number, y0: number, x1: number, y1: number, cb: (i: number) => void): void {
		const c = this.cell;
		const ix0 = Math.floor(x0 / c), iy0 = Math.floor(y0 / c), ix1 = Math.floor(x1 / c), iy1 = Math.floor(y1 / c);
		if ((ix1 - ix0 + 1) * (iy1 - iy0 + 1) > this.cells.size * 2) {
			for (const arr of this.cells.values()) for (const i of arr) cb(i);
			return;
		}
		for (let ix = ix0; ix <= ix1; ix++) {
			for (let iy = iy0; iy <= iy1; iy++) {
				const arr = this.cells.get(this.key(ix, iy));
				if (arr) for (const i of arr) cb(i);
			}
		}
	}
}

/** 路径顶点：pi ≥ 0 拓扑点；-1 自由顶点；-2 固定点 */
interface PV { x: number; y: number; pi: number }

/** 闭三角形 AKB 内的拓扑点（不含与 A/B 重合者） */
function triPoints(A: PV, K: PV, B: PV, grid: PtGrid, exclude: number): number[] {
	const o = orient(A, K, B);
	const out: number[] = [];
	if (Math.abs(o) < 1e-12) return out;
	const sg = o > 0 ? 1 : -1;
	const x0 = Math.min(A.x, K.x, B.x), x1 = Math.max(A.x, K.x, B.x);
	const y0 = Math.min(A.y, K.y, B.y), y1 = Math.max(A.y, K.y, B.y);
	const e = -1e-9;
	grid.forBox(x0, y0, x1, y1, i => {
		const p = grid.pts[i];
		if (p.x < x0 || p.x > x1 || p.y < y0 || p.y > y1) return;
		if (hyp(p.x - A.x, p.y - A.y) < 1e-7 || hyp(p.x - B.x, p.y - B.y) < 1e-7) return;
		if (exclude >= 0 && (i === exclude || hyp(p.x - K.x, p.y - K.y) < 1e-7)) return;
		if (orient(A, K, p) * sg < e) return;
		if (orient(K, B, p) * sg < e) return;
		if (orient(B, A, p) * sg < e) return;
		out.push(i);
	});
	return out;
}

/** {A,B} ∪ 点集 的凸包上、朝 K 一侧、从 A 到 B 的链（不含 A、B），返回拓扑点下标 */
function hullChain(A: PV, B: PV, K: PV, idxs: number[], pts: TPt[]): number[] {
	const L = hyp(B.x - A.x, B.y - A.y);
	const ux = (B.x - A.x) / L, uy = (B.y - A.y) / L;
	const side = orient(A, B, K) >= 0 ? 1 : -1;
	type Q = { x: number; y: number; i: number };
	const qs: Q[] = [{ x: 0, y: 0, i: -10 }, { x: L, y: 0, i: -11 }];
	for (const i of idxs) {
		const dx = pts[i].x - A.x, dy = pts[i].y - A.y;
		qs.push({ x: dx * ux + dy * uy, y: Math.max(0, side * (ux * dy - uy * dx)), i });
	}
	qs.sort((p, q) => p.x - q.x || p.y - q.y);
	const cr = (o: Q, a: Q, b: Q) => (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
	const lower: Q[] = [];
	for (const q of qs) {
		while (lower.length >= 2 && cr(lower[lower.length - 2], lower[lower.length - 1], q) <= 1e-12) lower.pop();
		lower.push(q);
	}
	const upper: Q[] = [];
	for (let k = qs.length - 1; k >= 0; k--) {
		const q = qs[k];
		while (upper.length >= 2 && cr(upper[upper.length - 2], upper[upper.length - 1], q) <= 1e-12) upper.pop();
		upper.push(q);
	}
	lower.pop(); upper.pop();
	const hull = [...lower, ...upper]; // 逆时针
	const ia = hull.findIndex(q => q.i === -10), ib = hull.findIndex(q => q.i === -11);
	if (ia < 0 || ib < 0) return [];
	const chain: number[] = [];
	for (let k = (ib + 1) % hull.length; k !== ia; k = (k + 1) % hull.length) {
		if (hull[k].i >= 0) chain.push(hull[k].i);
	}
	chain.reverse();
	return chain;
}

/**
 * 三角形松弛：同伦保持的最短路（相对拓扑点集）。
 * inner(pi)：拓扑点所属障碍的「内侧参考点」（包络圆心 / 形状中心）。路径若在某
 * 拓扑点处朝外侧拐（障碍在拐角外侧），该点不应钉住路径——从三角形点集里剔除它，
 * 允许松开（否则会被错误地永久挂住）。
 */
function relaxPath(path: PV[], grid: PtGrid, inner: (pi: number) => Pt | null): PV[] {
	const pts = grid.pts;
	const maxOps = 40 * path.length + 20000;
	let ops = 0;
	let i = 1;
	while (i < path.length - 1 && ops < maxOps) {
		const K = path[i];
		if (K.pi === -2) { i++; continue; }
		const A = path[i - 1], B = path[i + 1];
		if (hyp(A.x - B.x, A.y - B.y) < 1e-9) {
			// A-K-A 尖刺（零面积）：直接删
			path.splice(i, 2);
			ops++;
			i = Math.max(1, i - 1);
			continue;
		}
		let exclude = -1;
		if (K.pi >= 0) {
			const c = inner(K.pi);
			const t = cross(K.x - A.x, K.y - A.y, B.x - K.x, B.y - K.y);
			const s1 = c ? cross(K.x - A.x, K.y - A.y, c.x - K.x, c.y - K.y) : 0;
			const s2 = c ? cross(B.x - K.x, B.y - K.y, c.x - K.x, c.y - K.y) : 0;
			if (!c || !(s1 * t > 0 || s2 * t > 0)) exclude = K.pi;
		}
		const inside = triPoints(A, K, B, grid, exclude);
		const chain = inside.length ? hullChain(A, B, K, inside, pts) : [];
		if (chain.length === 1 && chain[0] === K.pi) { i++; continue; }
		if (!chain.length) {
			// 三角形内无点（或点全在弦 AB 上）：删 K
			path.splice(i, 1);
			ops++;
			i = Math.max(1, i - 1);
			continue;
		}
		let newLen = 0;
		let prev: Pt = A;
		for (const c of chain) { newLen += hyp(pts[c].x - prev.x, pts[c].y - prev.y); prev = pts[c]; }
		newLen += hyp(B.x - prev.x, B.y - prev.y);
		const oldLen = hyp(K.x - A.x, K.y - A.y) + hyp(B.x - K.x, B.y - K.y);
		if (newLen > oldLen - 1e-9) { i++; continue; }
		path.splice(i, 1, ...chain.map(c => ({ x: pts[c].x, y: pts[c].y, pi: c })));
		ops++;
		i = Math.max(1, i - 1);
	}
	return path;
}

/** ccw = 逆时针绕行（左拐）；getTangents 的侧向标志在数学坐标系下恰为 ccw */
interface RNode { x: number; y: number; r: number; ccw: boolean; turn: number }

/** 切线 + 圆弧渲染（带一致性检查：切线存在、弧扫角与折线转角相符） */
function renderNodes(nodes: RNode[], net: string, layer: number): RenderedPrim[] | null {
	const n = nodes.length;
	if (n < 2) return null;
	const tans: Array<[number, number, number, number]> = [];
	for (let i = 0; i + 1 < n; i++) {
		const a = nodes[i], b = nodes[i + 1];
		const d = hyp(b.x - a.x, b.y - a.y);
		if (a.r > 0 && b.r > 0) {
			if (a.ccw === b.ccw ? d <= Math.abs(a.r - b.r) + 1e-9 : d <= a.r + b.r + 1e-9) return null;
		} else if (d <= Math.max(a.r, b.r) + 1e-9) {
			return null;
		}
		tans.push(getTangents(a.x, a.y, a.r, a.ccw, b.x, b.y, b.r, b.ccw));
	}
	const out: RenderedPrim[] = [];
	for (let i = 0; i + 1 < n; i++) {
		const t = tans[i];
		if (i > 0) {
			const nd = nodes[i];
			if (nd.r > 0) {
				const prevT = tans[i - 1];
				const a1 = Math.atan2(prevT[3] - nd.y, prevT[2] - nd.x);
				const a2 = Math.atan2(t[1] - nd.y, t[0] - nd.x);
				let sw = a2 - a1;
				if (!nd.ccw) { while (sw > 1e-12) sw -= TAU; while (sw <= -TAU) sw += TAU; } else { while (sw < -1e-12) sw += TAU; while (sw >= TAU) sw -= TAU; }
				if (Math.abs(sw) > TAU - 1e-6) sw = 0;
				if (Math.abs(sw - nd.turn) > 0.6) return null;
				if (Math.abs(sw) * nd.r > 1e-7) {
					out.push({ kind: 'arc', net, layer, cx: nd.x, cy: nd.y, r: nd.r, startAngle: a1, sweep: sw });
				}
			}
		}
		if (hyp(t[2] - t[0], t[3] - t[1]) > 1e-7) out.push({ kind: 'line', net, layer, x1: t[0], y1: t[1], x2: t[2], y2: t[3] });
	}
	return out;
}

// ─────────────────────────────────────────────
// 链工具
// ─────────────────────────────────────────────

/** 压平为折线（弧上顶点在弧上，弦高 ≤ sag） */
function flatten(prims: RenderedPrim[], sag: number): Pt[] {
	const out: Pt[] = [];
	const push = (p: Pt) => {
		const l = out[out.length - 1];
		if (!l || hyp(l.x - p.x, l.y - p.y) > 1e-9) out.push(p);
	};
	for (const p of prims) {
		push(primStart(p));
		if (p.kind === 'arc') {
			const r = p.r ?? 0, sw = p.sweep ?? 0;
			const n = arcSteps(r, Math.abs(sw), sag);
			for (let k = 1; k < n; k++) {
				const a = (p.startAngle ?? 0) + (sw * k) / n;
				push({ x: (p.cx ?? 0) + r * Math.cos(a), y: (p.cy ?? 0) + r * Math.sin(a) });
			}
		}
		push(primEnd(p));
	}
	return out;
}

/**
 * 把一段图元链按连通性从 a 排序、逐根定向；接缝空隙（≤0.8）用短线桥接，
 * 保证输出首尾相接（不移动任何原图元）。
 */
export function normalizeChain(prims: RenderedPrim[], a: Pt, b: Pt | null, net: string, layer: number): RenderedPrim[] {
	const rest = prims.filter(p => primLen(p) > 1e-9 || p.kind === 'line');
	const out: RenderedPrim[] = [];
	let cur: Pt = a;
	const used = new Set<number>();
	while (used.size < rest.length) {
		let pick = -1, rev = false, best = 0.8;
		for (let i = 0; i < rest.length; i++) {
			if (used.has(i)) continue;
			const s = primStart(rest[i]), e = primEnd(rest[i]);
			const ds = hyp(s.x - cur.x, s.y - cur.y), de = hyp(e.x - cur.x, e.y - cur.y);
			if (ds < best) { best = ds; pick = i; rev = false; }
			if (de < best) { best = de; pick = i; rev = true; }
		}
		if (pick < 0) break;
		used.add(pick);
		const q = rev ? reversePrim(rest[pick]) : rest[pick];
		const qs = primStart(q);
		if (hyp(qs.x - cur.x, qs.y - cur.y) > 1e-9) out.push(lineP(net, layer, cur, qs));
		out.push({ ...q, net, layer });
		cur = primEnd(q);
	}
	// 断开的残余：按原序追加（桥接）——不丢任何图元
	for (let i = 0; i < rest.length; i++) {
		if (used.has(i)) continue;
		const q = rest[i];
		const qs = primStart(q);
		if (hyp(qs.x - cur.x, qs.y - cur.y) > 1e-9) out.push(lineP(net, layer, cur, qs));
		out.push({ ...q, net, layer });
		cur = primEnd(q);
	}
	if (b && hyp(b.x - cur.x, b.y - cur.y) > 1e-9) out.push(lineP(net, layer, cur, b));
	return out;
}

/** 合并共线相邻直线 / 同圆同向相邻弧，丢弃零长图元（不改变几何） */
export function mergeChain(prims: RenderedPrim[]): RenderedPrim[] {
	const out: RenderedPrim[] = [];
	for (const p of prims) {
		if (primLen(p) < 1e-7) {
			// 零长：若它造成空隙则用于桥接（几乎不会）
			continue;
		}
		const last = out[out.length - 1];
		if (last && last.kind === 'line' && p.kind === 'line') {
			const e = primEnd(last), s = primStart(p);
			if (hyp(e.x - s.x, e.y - s.y) < 1e-7) {
				const u = primDirStart(last), v = primDirStart(p);
				if (Math.abs(cross(u.x, u.y, v.x, v.y)) < 1e-9 && u.x * v.x + u.y * v.y > 0) {
					out[out.length - 1] = { ...last, x2: p.x2, y2: p.y2 };
					continue;
				}
			}
		}
		if (last && last.kind === 'arc' && p.kind === 'arc' &&
			Math.abs((last.cx ?? 0) - (p.cx ?? 0)) < 1e-7 && Math.abs((last.cy ?? 0) - (p.cy ?? 0)) < 1e-7 &&
			Math.abs((last.r ?? 0) - (p.r ?? 0)) < 1e-7 && Math.sign(last.sweep ?? 0) === Math.sign(p.sweep ?? 0)) {
			const e = primEnd(last), s = primStart(p);
			if (hyp(e.x - s.x, e.y - s.y) < 1e-7 && Math.abs((last.sweep ?? 0) + (p.sweep ?? 0)) < TAU - 1e-6) {
				out[out.length - 1] = { ...last, sweep: (last.sweep ?? 0) + (p.sweep ?? 0) };
				continue;
			}
		}
		out.push(p);
	}
	return out;
}

/** 链的最大接缝空隙 */
export function chainMaxGap(prims: RenderedPrim[]): number {
	let g = 0;
	for (let i = 1; i < prims.length; i++) {
		const e = primEnd(prims[i - 1]), s = primStart(prims[i]);
		g = Math.max(g, hyp(e.x - s.x, e.y - s.y));
	}
	return g;
}

/**
 * 固定点。pad 类：anchor = 焊盘（或本网静态铜）形状，该处允许圆角，但两个切点都须
 * 在铜皮内（≥0.5mil）或不比接缝处更远——走线仍穿过它。
 * （任务内对每段迭代到不动点，反复拉紧不会让走线一点点滑出焊盘。）
 */
export interface FixedPt { x: number; y: number; kind: 'end' | 'junction' | 'pad'; anchor?: Shape }

function nearFixed(fixed: FixedPt[], p: Pt, tol: number): FixedPt | null {
	let best: FixedPt | null = null, bd = tol;
	for (const f of fixed) {
		const d = hyp(f.x - p.x, f.y - p.y);
		if (d <= bd) { bd = d; best = f; }
	}
	return best;
}

/** 接缝统计：sharp = 转角 ≥30° 的非固定点接缝；kinks = 转角 ≥2° 的非固定点接缝 */
export function cornerStats(prims: RenderedPrim[], fixed: FixedPt[]): { sharp: number; kinks: number } {
	let sharp = 0, kinks = 0;
	for (let i = 1; i < prims.length; i++) {
		const J = primStart(prims[i]);
		if (nearFixed(fixed, J, 0.05)) continue;
		const u = primDirEnd(prims[i - 1]), v = primDirStart(prims[i]);
		const t = Math.abs(Math.atan2(cross(u.x, u.y, v.x, v.y), u.x * v.x + u.y * v.y));
		if (t >= (SHARP_DEG * Math.PI) / 180 - 1e-9) sharp++;
		if (t >= KINK_RAD) kinks++;
	}
	return { sharp, kinks };
}

/** 链上离点最近的位置 */
export function locateOnChain(prims: RenderedPrim[], x: number, y: number): { i: number; s: number; d: number } {
	let best = { i: 0, s: 0, d: Infinity };
	prims.forEach((p, i) => {
		const pr = primProject(p, x, y);
		if (pr.d < best.d - 1e-12) best = { i, s: pr.s, d: pr.d };
	});
	return best;
}

/** 链中心线到形状的最小有符号距离（全精度，真最小值；内部为负） */
export function chainShapeMin(prims: RenderedPrim[], sh: Shape): number {
	let m = Infinity;
	for (const p of prims) m = Math.min(m, primShapeMin(p, sh, Infinity));
	return m;
}

/**
 * 在固定点处切链：返回各子链（首尾即固定点位置）以及切点（joints.length = subs.length - 1）。
 * 不在链上的固定点（焊盘点经上轮圆角后偏离）投影到链上最近处。
 */
export function splitAtFixed(prims: RenderedPrim[], fixed: FixedPt[]): { subs: RenderedPrim[][]; joints: FixedPt[] } {
	const locs: Array<{ i: number; s: number; f: FixedPt }> = [];
	for (const f of fixed) {
		if (f.kind === 'end') continue;
		const l = locateOnChain(prims, f.x, f.y);
		if (!Number.isFinite(l.d)) continue;
		locs.push({ i: l.i, s: l.s, f });
	}
	locs.sort((p, q) => p.i - q.i || p.s - q.s);
	const subs: RenderedPrim[][] = [];
	const joints: FixedPt[] = [];
	let curSub: RenderedPrim[] = [];
	const cut = (f: FixedPt, at: Pt): void => {
		if (!curSub.length) return; // 与链首/上一切点重合
		subs.push(curSub);
		curSub = [];
		joints.push({ ...f, x: at.x, y: at.y });
	};
	let li = 0;
	for (let i = 0; i < prims.length; i++) {
		let p: RenderedPrim | null = prims[i];
		let off = 0;
		while (li < locs.length && locs[li].i === i) {
			const { s, f } = locs[li++];
			if (!p) continue;
			const sl = s - off;
			const L = primLen(p);
			if (sl <= 1e-6) { cut(f, primStart(p)); continue; }
			if (sl >= L - 1e-6) {
				curSub.push(p);
				cut(f, primEnd(p));
				p = null;
				continue;
			}
			const [h, t] = splitPrim(p, sl);
			curSub.push(h);
			cut(f, primEnd(h));
			p = t;
			off = s;
		}
		if (p && (primLen(p) > 1e-9 || !curSub.length)) curSub.push(p);
	}
	if (curSub.length) subs.push(curSub);
	else if (subs.length) joints.pop(); // 最后一个切点就是链尾
	return { subs, joints };
}

// ─────────────────────────────────────────────
// B. 拉紧（单段）
// ─────────────────────────────────────────────

interface TopoCtx {
	grid: PtGrid;
	/** 条目 id → 渲染外扩（离铜皮总量 W） */
	renderW: Map<number, number>;
	items: Map<number, WorldItem>;
}

/**
 * 为一条 run 建拓扑上下文：start = 起始几何（拓扑点要严格在它之外）。
 * mRender：渲染余量（组 → 余量），默认 RENDER_MARGIN，被起始路径更近者/原违规者下调。
 */
function buildTopo(env: RunEnv, start: RenderedPrim[], region: BBox, override?: Map<string, number>): TopoCtx {
	const w = env.width;
	const startSlack = slackByGroup(env.world, start, w, it => relevant(env, it), RENDER_MARGIN + WRITER_ERR + 1);
	const pts: TPt[] = [];
	const renderW = new Map<number, number>();
	const items = new Map<number, WorldItem>();
	const reg = bboxGrow(region, 0.5);
	env.world.query(bboxGrow(reg, w / 2 + env.world.maxR0 + 1), it => {
		if (!relevant(env, it)) return;
		const R = it.R0 + w / 2;
		const ss = startSlack.get(it.group) ?? 1e9;
		const extra = itemExtraMargin(it);
		// 原几何合规（so ≥ 0）：一律按 RENDER_MARGIN 渲染——哪怕起始路径本身更近甚至违规（阶段 2 候选用 A* 的全局线宽口径，
		// 按实际线宽可能违规）：把路径推开到回写误差之外，推不开则校验不过、该段保持原样。
		// 原几何已违规（so < 0）：不比起始路径/原违规深度更近（回写误差无法再补偿，只能不恶化）。
		const render = RENDER_MARGIN + extra;
		let mR = render;
		if (ss < render) {
			const so = env.origSlack(it.group);
			if (so < 0) mR = Math.max(Math.min(render, ss), so);
		}
		const ov = override?.get(it.group);
		// 被挤的窄缝：渲染余量降到校验底线（不再往下）
		if (ov !== undefined) mR = Math.min(mR, ov + extra);
		if (mR < 0) mR = Math.max(mR, Math.min(0, env.origSlack(it.group)));
		const topo = Math.min(mR, ss - TOPO_SHRINK);
		if (bboxGap(reg, it.bbox) > R + mR + 0.1) return;
		renderW.set(it.id, R + mR);
		items.set(it.id, it);
		const before = pts.length;
		itemTopoPoints(it, R + topo, pts);
		// 只保留区域内的点（三角形都在起始路径凸包内）
		let k = before;
		for (let j = before; j < pts.length; j++) {
			const p = pts[j];
			if (p.x >= reg.x0 && p.x <= reg.x1 && p.y >= reg.y0 && p.y <= reg.y1) pts[k++] = p;
		}
		pts.length = k;
	});
	const span = Math.max(region.x1 - region.x0, region.y1 - region.y0, 1);
	const cell = Math.max(2, Math.min(50, span / 64));
	return { grid: new PtGrid(pts, cell), renderW, items };
}

/** 单段拉紧：sub 为起止固定的子链；返回新子链或 null（无解/退化） */
function tautSub(env: RunEnv, topo: TopoCtx, sub: RenderedPrim[]): RenderedPrim[] | null {
	if (!sub.length) return null;
	const A = primStart(sub[0]), B = primEnd(sub[sub.length - 1]);
	if (hyp(A.x - B.x, A.y - B.y) < 1e-3) return null; // 闭环：不拉（会塌成点）
	const poly = flatten(sub, FLATTEN_SAG);
	const path: PV[] = poly.map((p, i) => ({ x: p.x, y: p.y, pi: i === 0 || i === poly.length - 1 ? -2 : -1 }));
	if (path.length < 2) return null;
	path[0].x = A.x; path[0].y = A.y;
	path[path.length - 1].x = B.x; path[path.length - 1].y = B.y;
	const pts = topo.grid.pts;
	const inner = (pi: number): Pt | null => {
		const tp = pts[pi];
		const it = topo.items.get(tp.item);
		if (!it) return null;
		if (tp.wid < 0) {
			const sh = it.shape;
			if (sh.t !== 'arc') return shapeCenter(sh);
			const d = hyp(tp.x - sh.cx, tp.y - sh.cy);
			if (d < 1e-9) return null;
			return { x: sh.cx + (tp.x - sh.cx) * sh.r / d, y: sh.cy + (tp.y - sh.cy) * sh.r / d };
		}
		const c = wrapCircle(it, tp.wid, topo.renderW.get(tp.item) ?? 0);
		if (c && hyp(c.x - tp.x, c.y - tp.y) > 1e-6) return c;
		return shapeCenter(it.shape);
	};
	const relaxed = relaxPath(path, topo.grid, inner);
	// 拐点 → 节点（同一包络圆、同向的连续拐点合并）
	const nodes: Array<RNode & { key: string }> = [{ x: A.x, y: A.y, r: 0, ccw: false, turn: 0, key: 'A' }];
	for (let k = 1; k < relaxed.length - 1; k++) {
		const P = relaxed[k - 1], K = relaxed[k], N = relaxed[k + 1];
		const ux = K.x - P.x, uy = K.y - P.y, vx = N.x - K.x, vy = N.y - K.y;
		const turn = Math.atan2(cross(ux, uy, vx, vy), ux * vx + uy * vy);
		if (Math.abs(turn) < 1e-9) continue;
		const tp = K.pi >= 0 ? pts[K.pi] : null;
		const ccw = turn > 0;
		if (!tp || tp.wid < 0) {
			nodes.push({ x: K.x, y: K.y, r: 0, ccw, turn: 0, key: `p${k}` });
			continue;
		}
		const key = `${tp.item}:${tp.wid}:${ccw ? 1 : 0}`;
		const last = nodes[nodes.length - 1];
		if (last.key === key) { last.turn += turn; continue; }
		const it = topo.items.get(tp.item);
		const W = topo.renderW.get(tp.item);
		const c = it && W !== undefined ? wrapCircle(it, tp.wid, W) : null;
		if (!c) return null;
		// 同向且互相包含的包络圆（重复焊盘 / 同心外包络）：只保留大的那个
		if (nodes.length > 1 && last.ccw === ccw && last.r > 0) {
			const d = hyp(c.x - last.x, c.y - last.y);
			if (d + c.r <= last.r + 1e-6) { last.turn += turn; continue; }
			if (d + last.r <= c.r + 1e-6) { Object.assign(last, { x: c.x, y: c.y, r: c.r, key }); last.turn += turn; continue; }
		}
		nodes.push({ x: c.x, y: c.y, r: c.r, ccw, turn, key });
	}
	nodes.push({ x: B.x, y: B.y, r: 0, ccw: false, turn: 0, key: 'B' });
	// 节点半径为 0 的绕行（被压扁的条目）保持尖角
	const out = renderNodes(nodes, env.net, env.layer);
	if (!out) return null;
	if (!out.length) return [lineP(env.net, env.layer, A, B)];
	// 端点精确
	const s0 = primStart(out[0]), e0 = primEnd(out[out.length - 1]);
	if (hyp(s0.x - A.x, s0.y - A.y) > 1e-6 || hyp(e0.x - B.x, e0.y - B.y) > 1e-6) return null;
	if (chainMaxGap(out) > 1e-6) return null;
	return out;
}

// ─────────────────────────────────────────────
// C. 圆角
// ─────────────────────────────────────────────

/** 切角区（多边形）内是否含有外网条目的任何部分（按形状的特征点 / 极值点判定，不只看一个代表点） */
function regionSwallows(env: RunEnv, poly: Pt[]): boolean {
	let b: BBox = { x0: Infinity, y0: Infinity, x1: -Infinity, y1: -Infinity };
	for (const p of poly) b = { x0: Math.min(b.x0, p.x), y0: Math.min(b.y0, p.y), x1: Math.max(b.x1, p.x), y1: Math.max(b.y1, p.y) };
	let hit = false;
	env.world.query(b, it => {
		if (hit || !relevant(env, it)) return;
		for (const rp of shapeProbePts(it.shape)) {
			if (pointInPolygonPx(rp.x, rp.y, poly)) { hit = true; return; }
		}
	});
	return hit;
}

function arcPolyPts(cx: number, cy: number, r: number, a0: number, sw: number): Pt[] {
	const n = Math.max(2, Math.ceil(Math.abs(sw) / 0.05));
	const out: Pt[] = [];
	for (let k = 0; k <= n; k++) {
		const a = a0 + (sw * k) / n;
		out.push({ x: cx + r * Math.cos(a), y: cy + r * Math.sin(a) });
	}
	return out;
}

function radiusLadder(rMax: number): number[] {
	const out: number[] = [];
	for (let r = Math.min(rMax, 400); r >= 0.25 && out.length < 40; r *= 0.8) out.push(r);
	return out;
}

/** 线-线圆角 */
function filletLL(
	env: RunEnv, P: RenderedPrim, Q: RenderedPrim, availP: number, availQ: number,
	pad: Shape | null,
): RenderedPrim[] | null {
	const padAnchor = pad;
	const J = primEnd(P);
	const u = primDirEnd(P), v = primDirStart(Q);
	const phi = Math.acos(Math.max(-1, Math.min(1, u.x * v.x + u.y * v.y)));
	if (phi < KINK_RAD || phi > Math.PI - 0.02) return null;
	const tanH = Math.tan(phi / 2);
	const tMax = Math.min(availP, availQ);
	if (tMax <= 1e-4) return null;
	const sgn = cross(u.x, u.y, v.x, v.y) >= 0 ? 1 : -1;
	const padNeed = padAnchor ? Math.max(shapeDist(padAnchor, J.x, J.y), -env.width / 2) : 0;
	for (const r of radiusLadder(tMax / tanH)) {
		const t = r * tanH;
		const T1 = { x: J.x - u.x * t, y: J.y - u.y * t };
		const T2 = { x: J.x + v.x * t, y: J.y + v.y * t };
		if (padAnchor && (shapeDist(padAnchor, T1.x, T1.y) > padNeed || shapeDist(padAnchor, T2.x, T2.y) > padNeed)) continue;
		const n1 = { x: -sgn * u.y, y: sgn * u.x };
		const C = { x: T1.x + n1.x * r, y: T1.y + n1.y * r };
		const a0 = Math.atan2(T1.y - C.y, T1.x - C.x);
		const arc: RenderedPrim = { kind: 'arc', net: env.net, layer: env.layer, cx: C.x, cy: C.y, r, startAngle: a0, sweep: sgn * phi };
		const ae = primEnd(arc);
		if (hyp(ae.x - T2.x, ae.y - T2.y) > 1e-6) continue;
		if (!validatePrims(env, [arc], true).ok) continue;
		const region = [T1, J, T2, ...arcPolyPts(C.x, C.y, r, a0 + sgn * phi, -sgn * phi)];
		if (regionSwallows(env, region)) continue;
		const out: RenderedPrim[] = [];
		const Ps = primStart(P), Qe = primEnd(Q);
		if (hyp(T1.x - Ps.x, T1.y - Ps.y) > 1e-7) out.push({ ...P, x2: T1.x, y2: T1.y });
		else { arc.startAngle = Math.atan2(Ps.y - C.y, Ps.x - C.x); }
		out.push(arc);
		if (hyp(Qe.x - T2.x, Qe.y - T2.y) > 1e-7) out.push({ ...Q, x1: T2.x, y1: T2.y });
		return out;
	}
	return null;
}

/** 线-弧圆角（线在前、弧在后） */
function filletLA(
	env: RunEnv, L: RenderedPrim, C: RenderedPrim, availL: number, availArc: number,
	pad: Shape | null,
): RenderedPrim[] | null {
	const padAnchor = pad;
	const P0 = primStart(L), J = primEnd(L);
	const Ll = primLen(L);
	const u = primDirEnd(L), v = primDirStart(C);
	const turn = Math.atan2(cross(u.x, u.y, v.x, v.y), u.x * v.x + u.y * v.y);
	if (Math.abs(turn) < KINK_RAD || Math.abs(turn) > Math.PI - 0.02) return null;
	const s = turn > 0 ? 1 : -1;
	const O = { x: C.cx ?? 0, y: C.cy ?? 0 }, Ra = C.r ?? 0, al0 = C.startAngle ?? 0, swA = C.sweep ?? 0;
	const sa = swA >= 0 ? 1 : -1;
	const nL = { x: -u.y, y: u.x };
	const padNeed = padAnchor ? Math.max(shapeDist(padAnchor, J.x, J.y), -env.width / 2) : 0;
	const rCap = Math.min(400, availL / Math.tan(Math.min(Math.abs(turn), 3) / 2) + 1, s === sa ? Ra * 0.98 : 400);
	for (const r of radiusLadder(rCap)) {
		const D = s === sa ? Ra - r : Ra + r;
		if (D <= 1e-6) continue;
		const Wx = P0.x + s * nL.x * r - O.x, Wy = P0.y + s * nL.y * r - O.y;
		const b = u.x * Wx + u.y * Wy;
		const c = Wx * Wx + Wy * Wy - D * D;
		const disc = b * b - c;
		if (disc < 0) continue;
		const sq = Math.sqrt(disc);
		const lams = [-b + sq, -b - sq].filter(l => l <= Ll + 1e-7 && l >= Ll - availL - 1e-7).sort((x, y) => y - x);
		for (const lam of lams) {
			const T1 = { x: P0.x + u.x * lam, y: P0.y + u.y * lam };
			const Cc = { x: T1.x + s * nL.x * r, y: T1.y + s * nL.y * r };
			const dOC = hyp(Cc.x - O.x, Cc.y - O.y);
			if (dOC < 1e-9) continue;
			const T2 = { x: O.x + (Cc.x - O.x) * Ra / dOC, y: O.y + (Cc.y - O.y) * Ra / dOC };
			const beta = Math.atan2(T2.y - O.y, T2.x - O.x);
			const delta = sa > 0 ? mod2pi(beta - al0) : mod2pi(al0 - beta);
			if (delta > availArc + 1e-9 || delta > Math.PI) continue;
			const f1 = Math.atan2(T1.y - Cc.y, T1.x - Cc.x), f2 = Math.atan2(T2.y - Cc.y, T2.x - Cc.x);
			const fsw = s > 0 ? mod2pi(f2 - f1) : -mod2pi(f1 - f2);
			// 一致性：线方向 + 圆角扫角 = 弧在 T2 处切向
			const tv = sa > 0 ? { x: -Math.sin(beta), y: Math.cos(beta) } : { x: Math.sin(beta), y: -Math.cos(beta) };
			const ca = Math.cos(fsw), sn = Math.sin(fsw);
			const rx = u.x * ca - u.y * sn, ry = u.x * sn + u.y * ca;
			if (hyp(rx - tv.x, ry - tv.y) > 1e-4) continue;
			if (padAnchor && (shapeDist(padAnchor, T1.x, T1.y) > padNeed || shapeDist(padAnchor, T2.x, T2.y) > padNeed)) continue;
			const fil: RenderedPrim = { kind: 'arc', net: env.net, layer: env.layer, cx: Cc.x, cy: Cc.y, r, startAngle: f1, sweep: fsw };
			if (!validatePrims(env, [fil], true).ok) continue;
			const region = [T1, J, ...arcPolyPts(O.x, O.y, Ra, al0, sa * delta), ...arcPolyPts(Cc.x, Cc.y, r, f1 + fsw, -fsw)];
			if (regionSwallows(env, region)) continue;
			const out: RenderedPrim[] = [];
			if (lam > 1e-7) out.push({ ...L, x2: T1.x, y2: T1.y });
			else fil.startAngle = Math.atan2(P0.y - Cc.y, P0.x - Cc.x);
			out.push(fil);
			const rest = swA - sa * delta;
			if (Math.abs(rest) * Ra > 1e-7) out.push({ ...C, startAngle: beta, sweep: rest });
			return out;
		}
	}
	return null;
}

function jointTurn(P: RenderedPrim, Q: RenderedPrim): number {
	const u = primDirEnd(P), v = primDirStart(Q);
	return Math.abs(Math.atan2(cross(u.x, u.y, v.x, v.y), u.x * v.x + u.y * v.y));
}

/** C：剩余拐角倒圆（固定端点/T 接点不动；穿过焊盘处切点须留在焊盘铜皮内） */
export function roundCorners(env: RunEnv, prims: RenderedPrim[], fixed: FixedPt[]): RenderedPrim[] {
	const out = [...prims];
	const needsRound = (k: number): boolean => {
		if (k < 0 || k + 1 >= out.length) return false;
		const J = primEnd(out[k]);
		const f = nearFixed(fixed, J, 0.02);
		if (f && f.kind !== 'pad') return false;
		return jointTurn(out[k], out[k + 1]) >= KINK_RAD;
	};
	for (let k = 0; k + 1 < out.length; k++) {
		if (!needsRound(k)) continue;
		const P = out[k], Q = out[k + 1];
		const J = primEnd(P);
		const f = nearFixed(fixed, J, 0.02);
		const anchor = f && f.kind === 'pad' ? f.anchor ?? null : null;
		if (f && f.kind === 'pad' && !anchor) continue;
		let res: RenderedPrim[] | null = null;
		const nextRound = needsRound(k + 1);
		if (P.kind === 'line' && Q.kind === 'line') {
			res = filletLL(env, P, Q, primLen(P), primLen(Q) * (nextRound ? 0.5 : 1), anchor);
		} else if (P.kind === 'line' && Q.kind === 'arc') {
			res = filletLA(env, P, Q, primLen(P), Math.abs(Q.sweep ?? 0) * (nextRound ? 0.5 : 1), anchor);
		} else if (P.kind === 'arc' && Q.kind === 'line') {
			// 反向：线在前、弧在后。P 的前一接缝已处理过，可用全部扫角
			const r = filletLA(env, reversePrim(Q), reversePrim(P), primLen(Q) * (nextRound ? 0.5 : 1), Math.abs(P.sweep ?? 0), anchor);
			if (r) res = r.map(reversePrim).reverse();
		}
		if (!res) continue;
		out.splice(k, 2, ...res);
		k += res.length - 2;
	}
	return out;
}

// ─────────────────────────────────────────────
// 整段：拉紧 + 圆角
// ─────────────────────────────────────────────

export interface TightenResult {
	prims: RenderedPrim[];
	/** 拉紧阶段是否改变了几何（否则只是圆角） */
	taut: boolean;
}

/**
 * 对一条 run 做 B + C。cur 必须是从 a 到 b 的有序连续链。
 * 返回 null 表示没有可用的新几何（调用方保持原样）。
 */
export function tightenChain(env: RunEnv, cur: RenderedPrim[], fixed: FixedPt[], opts: { taut?: boolean; candidate?: boolean } = {}): TightenResult | null {
	if (!cur.length) return null;
	const a = primStart(cur[0]), b = primEnd(cur[cur.length - 1]);
	const doTaut = opts.taut !== false && hyp(a.x - b.x, a.y - b.y) > 1e-3;
	const { subs, joints } = splitAtFixed(cur, fixed);
	const jointFixed: FixedPt[] = [
		{ x: a.x, y: a.y, kind: 'end' }, { x: b.x, y: b.y, kind: 'end' },
		...joints,
	];
	const newSubs: RenderedPrim[][] = subs.map(s => s);
	let tautChanged = false;
	if (doTaut) {
		const region = chainBBox(cur);
		let override: Map<string, number> | undefined;
		for (let attempt = 0; attempt < 2; attempt++) {
			const topo = buildTopo(env, cur, region, override);
			const bad = new Set<string>();
			let pending = false;
			subs.forEach((sub, k) => {
				if (newSubs[k] !== sub) return; // 已采纳
				const cand = tautSub(env, topo, sub);
				if (!cand) return;
				// 阶段 1：起点是当前（合法）几何，结果不得更长；候选模式起点可能不合法（A* 用的是
				// 全局线宽口径），按实际线宽拉开后变长是正常的——由调用方和当前几何比
				if (!opts.candidate && chainLength(cand) > chainLength(sub) + 1e-6) return;
				const v = validatePrims(env, cand);
				if (v.ok) { newSubs[k] = mergeChain(cand); tautChanged = true; return; }
				for (const g of v.bad) bad.add(g);
				pending = true;
			});
			if (!pending) break;
			// 第二次：失败组的渲染余量降到校验底线 MIN_MARGIN（被挤的窄缝）
			override = new Map([...(override ?? new Map())]);
			for (const g of bad) override.set(g, MIN_MARGIN);
		}
	}
	let chain: RenderedPrim[] = [];
	for (const s of newSubs) chain.push(...s);
	chain = mergeChain(chain);
	const rounded = mergeChain(roundCorners(env, chain, jointFixed));
	if (!rounded.length) return null;
	return { prims: rounded, taut: tautChanged };
}

/** 直接对一条（可能换了拓扑的）候选链做 B + C（阶段 2 / 整网重布用） */
export function tightenCandidate(env: RunEnv, cand: RenderedPrim[], fixed: FixedPt[]): RenderedPrim[] | null {
	const t = tightenChain(env, cand, fixed, { candidate: true });
	const best = t ? t.prims : cand;
	if (!validatePrims(env, best).ok) {
		if (t && validatePrims(env, cand).ok) return cand;
		return null;
	}
	return best;
}
