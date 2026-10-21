/**
 * EdgeRouter - 焊盘到焊盘的「最小距离」走线
 *
 * 旧实现的问题（用户反馈：绕路多、出线不正常）：
 *  - 每个连接都走「焊盘中心 → 逃逸点（≈91mil）→ 寻路 → 逃逸点 → 焊盘中心」，
 *    逃逸点由「净空最大」启发式选出，与目标方向无关 ⇒ 无条件多出 2×91mil，
 *    且出线方向经常背对目标（"不正常的出线"）。
 *  - 直连（焊盘中心到焊盘中心）只在逃逸失败后才尝试，任意角度最短路径几乎用不上。
 *
 * 新模型：
 *  - 出线方向不再由启发式决定，而是作为「候选集合」交给最短路求解器：
 *    每个候选点位于焊盘铜皮轮廓在某个方向上的边界（+ 极小余量），
 *    候选代价 = 焊盘中心到候选点的引出段长度。
 *  - 多源 Dijkstra 同时以全部源候选为起点、全部目标候选为终点，
 *    取「引出段 + 路径 + 引入段」总长最小者。
 *  - 因此：两点之间直线可连时，总长恰等于两焊盘直线距离（理论下界）；
 *    直线被挡时才绕行，且绕行量由可见图精确求解。
 *  - 焊盘自身的铜皮轮廓作为「本体禁行区」（blockerOnly + forceBlock）：
 *    走线必须从焊盘向外离开，不能压过自己的焊盘；
 *    但引出段在焊盘内部，因此引出段不参与该判定。
 */
import {
	RBObstacle, RBPrimitive, pointSegDist, fanoutFillet,
} from './rubberband-engine';
import { routeBetweenEndpoints, validatePrimsAgainst } from './visibility-router';

/** 焊盘铜皮轮廓圆（相对焊盘中心） */
export interface PadOutline {
	x: number;
	y: number;
	r: number;
}

/** 焊盘终端（布线的端点） */
export interface PadTerminal {
	x: number;
	y: number;
	net: string;
	/** 焊盘铜皮轮廓（相对焊盘中心偏移的圆群） */
	outline: PadOutline[];
	/** 长焊盘长轴单位向量（可选）：常规出线方向 */
	axisX?: number;
	axisY?: number;
}

/** 出线候选点 */
export interface ExitCandidate {
	x: number;
	y: number;
	/** 出线方向（单位向量） */
	dirX: number;
	dirY: number;
	/** 引出段长度（焊盘中心 → 候选点） */
	length: number;
}

export interface PadEdgeOptions {
	/** 走廊放宽级别 */
	expands?: number[];
	/** 单次求解时间预算（毫秒） */
	budgetMs?: number;
	/** 拐角圆角半径（mil），0 = 直角 */
	filletRadiusMil?: number;
	/** 非轴向出线方向的额外代价（mil）：优化阶段用于偏好正交走线 */
	dirPenaltyMil?: number;
	/** 候选点相对焊盘边界的外扩余量（mil） */
	marginMil?: number;
}

export interface PadEdgeResult {
	ok: boolean;
	/** 完整图元（含首尾引出段） */
	prims: RBPrimitive[];
	/** 总长（含引出/引入段） */
	length: number;
	src?: ExitCandidate;
	dst?: ExitCandidate;
}

/** 均匀出线方向数（45° 步进） */
const UNIFORM_DIRS = 8;
/** 候选点外扩余量：让首段切线不被「本体禁行区」判为侵入 */
const DEFAULT_MARGIN_MIL = 0.05;

/**
 * 射线在圆群（焊盘铜皮轮廓）上的出射距离。
 * 圆群并集的凸包沿方向 u 的支撑距离 = max_i(o_i·u + r_i)；
 * 这里求的是「射线离开圆群的最远交点」，用同样的 max 语义（圆群从中心出发连续覆盖）。
 */
function rayExitDistance(outline: PadOutline[], ux: number, uy: number): number {
	let best = 0;
	for (const c of outline) {
		const along = c.x * ux + c.y * uy;
		const perp = c.x * uy - c.y * ux;
		const d2 = c.r * c.r - perp * perp;
		if (d2 < 0) continue;         // 射线与该圆不相交
		const t = along + Math.sqrt(d2);
		if (t > best) best = t;
	}
	return best;
}

/**
 * 生成出线候选点：
 *  - 8 个均匀方向（45° 步进）——覆盖「先横向让出再转弯」等必要绕行
 *  - 焊盘长轴方向及其反向——长焊盘的常规出线
 *  - 正对目标的方向——保证直线可连时总长恰为两焊盘直线距离
 */
export function buildExitCandidates(
	pad: PadTerminal,
	toward: { x: number; y: number },
	marginMil = DEFAULT_MARGIN_MIL,
): ExitCandidate[] {
	const dirs: Array<[number, number]> = [];
	for (let k = 0; k < UNIFORM_DIRS; k++) {
		const a = (k * Math.PI) / (UNIFORM_DIRS / 2);
		dirs.push([Math.cos(a), Math.sin(a)]);
	}
	if (pad.axisX !== undefined && pad.axisY !== undefined) {
		dirs.push([pad.axisX, pad.axisY], [-pad.axisX, -pad.axisY]);
	}
	const dx = toward.x - pad.x, dy = toward.y - pad.y;
	const dl = Math.hypot(dx, dy);
	if (dl > 1e-6) dirs.push([dx / dl, dy / dl]);

	const out: ExitCandidate[] = [];
	const seen = new Set<string>();
	for (const [ux, uy] of dirs) {
		const key = `${Math.round(ux * 1000)},${Math.round(uy * 1000)}`;
		if (seen.has(key)) continue;
		seen.add(key);
		const t = rayExitDistance(pad.outline, ux, uy);
		if (t <= 1e-6) continue;
		const len = t + marginMil;
		out.push({ x: pad.x + ux * len, y: pad.y + uy * len, dirX: ux, dirY: uy, length: len });
	}
	return out;
}

/**
 * 焊盘本体禁行区：走线不得压过自己的焊盘。
 * blockerOnly：只参与碰撞判定，不生成切点/边（否则会沿焊盘边缘绕圈）。
 * forceBlock：即使与当前网络同名也阻挡（同名障碍默认被跳过）。
 */
export function buildPadKeepout(pad: PadTerminal, layer?: number): RBObstacle[] {
	return pad.outline.map((c, i) => ({
		id: `ko${i}:${pad.x.toFixed(1)},${pad.y.toFixed(1)}`,
		kind: 'circle' as const,
		x: pad.x + c.x,
		y: pad.y + c.y,
		r: c.r,
		net: pad.net,
		layer,
		group: `ko:${pad.net}:${pad.x.toFixed(1)},${pad.y.toFixed(1)}`,
		blockerOnly: true,
		forceBlock: true,
		noPrune: true,
	}));
}

/** 引出段是否可行：不侵入其它网络的障碍（本体禁行区按网络跳过） */
function stubFeasible(pad: PadTerminal, cand: ExitCandidate, obstacles: RBObstacle[], net: string): boolean {
	for (const o of obstacles) {
		if (o.net === net) continue;
		if (o.forceBlock) continue;          // 本体禁行区（自己的焊盘）
		if (pointSegDist(o.x, o.y, pad.x, pad.y, cand.x, cand.y) < o.r - 0.02) return false;
	}
	return true;
}

/** 两个图元之间插入圆角（仅处理 直线→直线） */
function filletJoin(a: RBPrimitive, b: RBPrimitive, radius: number): RBPrimitive[] | null {
	if (radius <= 0) return null;
	if (a.kind !== 'line' || b.kind !== 'line') return null;
	const f = fanoutFillet(
		{ x1: a.x1, y1: a.y1, x2: a.x2, y2: a.y2 },
		{ x1: b.x1, y1: b.y1, x2: b.x2, y2: b.y2 },
		radius,
	);
	return f ? f.prims : null;
}

/** 图元总长 */
export function primsLength(prims: RBPrimitive[]): number {
	let total = 0;
	for (const p of prims) {
		if (p.kind === 'line') total += Math.hypot(p.x2 - p.x1, p.y2 - p.y1);
		else total += Math.abs(p.sweep) * p.r;
	}
	return total;
}

/**
 * 焊盘 → 焊盘的最小距离走线。
 *
 * @param obstacles 当前层上的障碍（不含两个端点焊盘的本体禁行区，本函数内部追加）
 */
export async function routePadEdge(
	src: PadTerminal,
	dst: PadTerminal,
	obstacles: RBObstacle[],
	net: string,
	opts: PadEdgeOptions = {},
): Promise<PadEdgeResult> {
	const expands = opts.expands ?? [150, 500];
	const margin = opts.marginMil ?? DEFAULT_MARGIN_MIL;
	const penalty = opts.dirPenaltyMil ?? 0;

	const allSrc = buildExitCandidates(src, dst, margin);
	const allDst = buildExitCandidates(dst, src, margin);
	const srcCands = allSrc.filter(c => stubFeasible(src, c, obstacles, net));
	const dstCands = allDst.filter(c => stubFeasible(dst, c, obstacles, net));
	if (!srcCands.length || !dstCands.length) return { ok: false, prims: [], length: 0 };

	// 出线方向偏好：非轴向（45° 倍数之外）加一点惩罚，仅用于优化阶段
	const costOf = (c: ExitCandidate): number => {
		if (penalty <= 0) return c.length;
		const axis = Math.abs(c.dirX) < 1e-6 || Math.abs(c.dirY) < 1e-6;
		return c.length + (axis ? 0 : penalty);
	};

	const keepouts = [...buildPadKeepout(src), ...buildPadKeepout(dst)];
	const routeObs = [...obstacles, ...keepouts];

	const res = await routeBetweenEndpoints(
		srcCands.map(c => ({ x: c.x, y: c.y, cost: costOf(c) })),
		dstCands.map(c => ({ x: c.x, y: c.y, cost: costOf(c) })),
		routeObs, net, expands, opts.budgetMs,
		{ x: src.x, y: src.y }, { x: dst.x, y: dst.y },
	);
	if (!res.ok || !res.src || !res.dst) return { ok: false, prims: [], length: 0 };

	const srcCand = srcCands.find(c => c.x === res.src!.x && c.y === res.src!.y) ?? srcCands[0];
	const dstCand = dstCands.find(c => c.x === res.dst!.x && c.y === res.dst!.y) ?? dstCands[0];

	const stubIn: RBPrimitive = { kind: 'line', x1: src.x, y1: src.y, x2: srcCand.x, y2: srcCand.y };
	const stubOut: RBPrimitive = { kind: 'line', x1: dstCand.x, y1: dstCand.y, x2: dst.x, y2: dst.y };
	const path = res.prims.slice();

	// 拐角圆角：引出段与路径首/末段之间
	const plain: RBPrimitive[] = [stubIn, ...path, stubOut];
	const prims: RBPrimitive[] = [];
	const R = opts.filletRadiusMil ?? 0;
	if (path.length) {
		const head = filletJoin(stubIn, path[0], R);
		if (head) {
			prims.push(head[0], head[1]);
			path[0] = head[2];
		} else {
			prims.push(stubIn);
		}
		const last = path[path.length - 1];
		const tail = filletJoin(last, stubOut, R);
		if (tail) {
			prims.push(...path.slice(0, -1), tail[0], tail[1], tail[2]);
		} else {
			prims.push(...path, stubOut);
		}
	} else {
		prims.push(stubIn, stubOut);
	}

	// 圆角会切掉拐角、可能贴近障碍物：复验不通过则退回直角版本
	let finalPrims = prims;
	if (R > 0 && prims.length !== plain.length) {
		const legal = await validatePrimsAgainst(prims, obstacles, net);
		if (!legal) finalPrims = plain;
	}

	return { ok: true, prims: finalPrims, length: primsLength(finalPrims), src: srcCand, dst: dstCand };
}
