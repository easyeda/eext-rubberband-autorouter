/**
 * EdgeRouter - 焊盘出线几何
 *
 * 布线主模型是「焊盘中心 → 可视图最短路 → 焊盘中心」：本网络焊盘不作为障碍，
 * 出线方向就是最短路的起始方向，走线天然切线连续、没有人为折角。
 *
 * 但在密集焊盘排里，邻焊盘的外扩间距圈可能吞没本焊盘中心（间距 < 两焊盘半径+
 * 外扩之和），中心起点的任何线段都会被判侵入 —— 此时退回到「出线候选」模型：
 * 从焊盘轮廓边界出线（引出段），引出段与路径之间用切线圆角过渡。
 *
 * 因此本文件提供：
 *  - buildExitCandidates：出线候选点（中心直连失败的同层退避、过孔兜底共用）
 *  - routePadEdge：出线候选 + 引出段 + 切线圆角的同层退避走线
 *  - primsLength：图元求长工具
 */
import {
	RBObstacle, RBPrimitive, fanoutFillet,
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
	/** 候选点相对焊盘边界的外扩余量（mil） */
	marginMil?: number;
}

export interface PadEdgeResult {
	ok: boolean;
	/** 完整图元（含首尾引出段） */
	prims: RBPrimitive[];
	/** 总长（含引出/引入段） */
	length: number;
}

/** 均匀出线方向数（45° 步进） */
const UNIFORM_DIRS = 8;
/** 候选点外扩余量：给引出段留出一点裕度 */
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
 * 生成出线候选点（同层退避 / 过孔兜底的引出段方向）：
 *  - 8 个均匀方向（45° 步进）——覆盖「先横向让出再转弯」等必要绕行
 *  - 焊盘长轴方向及其反向——长焊盘的常规出线
 *  - 正对目标的方向——保证直线可连时引出段最短
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
 * 焊盘本体禁行区：走线不得压过自己的焊盘（退避模型下从边界出线）。
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

/**
 * 引出段是否可行（从焊盘中心到指定点的直线段）：
 * 不得侵入其它网络的障碍 —— 但完全落在本焊盘铜皮之内的侵入段除外。
 *
 * 依据：走线压自家焊盘铜皮不构成任何间距冲突；而密集焊盘排里邻焊盘的
 * 外扩间距圈会吞没本焊盘中心（相邻间距 < 两焊盘半径+外扩之和），此时
 * 从中心出发的任何线段开头一段必然「侵入」——这是焊盘排本身的既有状态，
 * 不是走线造成的，不能因此判死所有出线方向。
 */
export function stubFeasible(
	pad: PadTerminal, x2: number, y2: number, obstacles: RBObstacle[], net: string,
): boolean {
	const dx = x2 - pad.x, dy = y2 - pad.y;
	const len = Math.hypot(dx, dy);
	if (len < 1e-9) return true;
	const ux = dx / len, uy = dy / len;
	// 本焊盘铜皮沿出线方向的出射距离（超出它之后必须真正清障）
	const exit = rayExitDistance(pad.outline, ux, uy) - 0.5;
	for (const o of obstacles) {
		if (o.net === net) continue;
		if (o.forceBlock) continue;          // 本体禁行区（自己的焊盘）
		const r = o.r - 0.02;
		const wx = o.x - pad.x, wy = o.y - pad.y;
		const b = wx * ux + wy * uy;
		const c = wx * wx + wy * wy - r * r;
		let t0: number, t1: number;
		if (c < 0) {
			t0 = 0; t1 = b + Math.sqrt(b * b - c);   // 焊盘中心已在侵入圈内
		} else {
			const disc = b * b - c;
			if (disc < 0) continue;
			const sq = Math.sqrt(disc);
			t0 = b - sq; t1 = b + sq;
		}
		if (t1 < 0 || t0 > len) continue;
		t0 = Math.max(0, t0);
		t1 = Math.min(len, t1);
		if (t1 - t0 < 0.01) continue;
		if (t1 <= exit) continue;            // 侵入段完全在自家铜皮内 → 可接受
		return false;
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
 * 焊盘 → 焊盘的「出线候选」走线（同层退避，仅在中心直连失败时使用）。
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

	const allSrc = buildExitCandidates(src, dst, margin);
	const allDst = buildExitCandidates(dst, src, margin);
	const srcCands = allSrc.filter(c => stubFeasible(src, c.x, c.y, obstacles, net));
	const dstCands = allDst.filter(c => stubFeasible(dst, c.x, c.y, obstacles, net));
	if (!srcCands.length || !dstCands.length) return { ok: false, prims: [], length: 0 };

	const keepouts = [...buildPadKeepout(src), ...buildPadKeepout(dst)];
	const routeObs = [...obstacles, ...keepouts];

	const res = await routeBetweenEndpoints(
		srcCands.map(c => ({ x: c.x, y: c.y, cost: c.length })),
		dstCands.map(c => ({ x: c.x, y: c.y, cost: c.length })),
		routeObs, net, expands, opts.budgetMs,
		{ x: src.x, y: src.y }, { x: dst.x, y: dst.y },
	);
	if (!res.ok || !res.src || !res.dst) return { ok: false, prims: [], length: 0 };

	const srcCand = srcCands.find(c => c.x === res.src!.x && c.y === res.src!.y) ?? srcCands[0];
	const dstCand = dstCands.find(c => c.x === res.dst!.x && c.y === res.dst!.y) ?? dstCands[0];

	const stubIn: RBPrimitive = { kind: 'line', x1: src.x, y1: src.y, x2: srcCand.x, y2: srcCand.y };
	const stubOut: RBPrimitive = { kind: 'line', x1: dstCand.x, y1: dstCand.y, x2: dst.x, y2: dst.y };
	const path = res.prims.slice();

	// 拐角圆角：引出段与路径首/末段之间（切线圆弧过渡，G1 连续）
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

	return { ok: true, prims: finalPrims, length: primsLength(finalPrims) };
}
