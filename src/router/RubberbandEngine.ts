/**
 * RubberbandEngine - RBR（拓扑橡皮筋）核心几何引擎
 *
 * 实现论文/参考代码 (Stefan Salewski, Ruby-PCB-Router, Tal Dayan 博士论文) 的核心效果：
 *  1. 绳子拉紧 (string tightening)：路径表示为节点圆序列，
 *     相邻节点的切线段被障碍物挡住时插入该障碍物节点，迭代至收敛，
 *     得到绕障碍凸包的最短路径（凸包拉紧效果）。
 *  2. 切线 + 圆弧渲染 (get_tangents / draw_routes)：
 *     相邻圆之间画外公/内公切线，障碍圆上画绕行圆弧。
 *
 * 本模块为纯几何计算，不依赖 EDA API，可在 Node 中离线测试。
 * 所有坐标/长度单位统一为 mil。
 */

// ─────────────────────────────────────────────
// 类型
// ─────────────────────────────────────────────

/** 障碍物：圆或胶囊（线段 + 半径）。r 均为已膨胀半径（含线宽一半 + 间距） */
export interface RBObstacle {
	id: string;
	kind: 'circle' | 'capsule';
	/** circle 圆心 */
	x: number;
	y: number;
	r: number;
	/** capsule 端点（kind === 'capsule' 时有效） */
	x1?: number;
	y1?: number;
	x2?: number;
	y2?: number;
	net?: string;
	/** 所在层；undefined 表示阻挡所有层（如板框） */
	layer?: number;
	/** 组号：同一物理障碍拆成的圆群共享组号，组内互不阻挡 */
	group?: string;
	/** 实心障碍（填充区/板框）：不参与走廊剪枝，防止路径从内部穿过 */
	noPrune?: boolean;
	/** 仅阻挡不建图（填充内部网格圆）：路径不能绕它们转，只需被挡住 */
	blockerOnly?: boolean;
}

/** 路径节点：障碍物圆或终端（焊盘，r=0）。rgt = 绳子从该圆的右侧（顺时针侧）绕过 */
export interface RBNode {
	x: number;
	y: number;
	r: number;
	rgt: boolean;
	/** 关联的障碍物（终端节点为 null） */
	obs: RBObstacle | null;
	/** 障碍物组号（终端为 undefined） */
	group?: string;
}

/** 渲染图元 */
export interface RBLine {
	kind: 'line';
	x1: number;
	y1: number;
	x2: number;
	y2: number;
}
export interface RBArc {
	kind: 'arc';
	/** 圆心、半径、起始角、扫掠角（弧度，正值逆时针） */
	cx: number;
	cy: number;
	r: number;
	startAngle: number;
	sweep: number;
}
export type RBPrimitive = RBLine | RBArc;

export interface TightenResult {
	/** 收敛成功 */
	ok: boolean;
	nodes: RBNode[];
	/** 迭代次数（调试用） */
	iterations: number;
}

export interface TightenOptions {
	/** 排除的网络（本网络的障碍不阻挡） */
	net: string;
	/** 布线层；layer 为 undefined 的障碍（板框）始终阻挡 */
	layer: number;
	/** 最大插入迭代次数（防振荡） */
	maxIterations?: number;
	/** 最大节点数 */
	maxNodes?: number;
}

// ─────────────────────────────────────────────
// 基础几何
// ─────────────────────────────────────────────

export function dist(ax: number, ay: number, bx: number, by: number): number {
	return Math.hypot(ax - bx, ay - by);
}

/** 点到线段的最短距离 */
export function pointSegDist(
	px: number, py: number,
	x1: number, y1: number, x2: number, y2: number,
): number {
	const dx = x2 - x1, dy = y2 - y1;
	const len2 = dx * dx + dy * dy;
	if (len2 < 1e-12) return Math.hypot(px - x1, py - y1);
	let t = ((px - x1) * dx + (py - y1) * dy) / len2;
	t = Math.max(0, Math.min(1, t));
	return Math.hypot(px - (x1 + t * dx), py - (y1 + t * dy));
}

function segsIntersectProper(
	x1: number, y1: number, x2: number, y2: number,
	x3: number, y3: number, x4: number, y4: number,
): boolean {
	const ccw = (ax: number, ay: number, bx: number, by: number, cx: number, cy: number) =>
		(cy - ay) * (bx - ax) - (by - ay) * (cx - ax);
	const d1 = ccw(x3, y3, x4, y4, x1, y1);
	const d2 = ccw(x3, y3, x4, y4, x2, y2);
	const d3r = ccw(x1, y1, x2, y2, x3, y3);
	const d4 = ccw(x1, y1, x2, y2, x4, y4);
	if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3r > 0 && d4 < 0) || (d3r < 0 && d4 > 0))) return true;
	const on = (ax: number, ay: number, bx: number, by: number, cx: number, cy: number) =>
		Math.min(ax, cx) <= bx && bx <= Math.max(ax, cx) &&
		Math.min(ay, cy) <= by && by <= Math.max(ay, cy);
	if (d1 === 0 && on(x3, y3, x1, y1, x4, y4)) return true;
	if (d2 === 0 && on(x3, y3, x2, y2, x4, y4)) return true;
	if (d3r === 0 && on(x1, y1, x3, y3, x2, y2)) return true;
	if (d4 === 0 && on(x1, y1, x4, y4, x2, y2)) return true;
	return false;
}

/** 线段到线段的最短距离 */
export function segSegDist(
	x1: number, y1: number, x2: number, y2: number,
	x3: number, y3: number, x4: number, y4: number,
): number {
	if (segsIntersectProper(x1, y1, x2, y2, x3, y3, x4, y4)) return 0;
	return Math.min(
		pointSegDist(x1, y1, x3, y3, x4, y4),
		pointSegDist(x2, y2, x3, y3, x4, y4),
		pointSegDist(x3, y3, x1, y1, x2, y2),
		pointSegDist(x4, y4, x1, y1, x2, y2),
	);
}

/**
 * 两圆之间的切线（移植自 RBR get_tangents）
 * l1/l2 = 绳子从该圆右侧绕过（rgt）。
 * 返回 [x1', y1', x2', y2']：圆1 上的切点 与 圆2 上的切点。
 * 半径为 0 时退化为到圆心的直线。
 */
export function getTangents(
	x1: number, y1: number, r1: number, l1: boolean,
	x2: number, y2: number, r2: number, l2: boolean,
): [number, number, number, number] {
	const d = Math.hypot(x1 - x2, y1 - y2);
	if (d < 1e-9) return [x1, y1, x2, y2];
	const vx = (x2 - x1) / d;
	const vy = (y2 - y1) / d;
	const adjR2 = r2 * (l1 === l2 ? 1 : -1);
	const c = (r1 - adjR2) / d;
	let h = 1 - c * c;
	if (h < 0) h = 0; // 圆重叠时钳制：切线穿过圆心方向
	h = Math.sqrt(h) * (l1 ? -1 : 1);
	const nx = vx * c - h * vy;
	const ny = vy * c + h * vx;
	return [x1 + r1 * nx, y1 + r1 * ny, x2 + adjR2 * nx, y2 + adjR2 * ny];
}

// ─────────────────────────────────────────────
// 障碍物阻挡检测
// ─────────────────────────────────────────────

function obstacleBlocks(
	obs: RBObstacle,
	layer: number,
	net: string,
	segX1: number, segY1: number, segX2: number, segY2: number,
	skipGroups: Set<string>,
): boolean {
	if (obs.group && skipGroups.has(obs.group)) return false;
	if (obs.net && obs.net === net) return false;
	if (obs.layer !== undefined && obs.layer !== layer) return false;
	return obstacleBlocksRaw(obs, segX1, segY1, segX2, segY2);
}

/** 不带 skip 的纯几何阻挡判定 */
function obstacleBlocksRaw(
	obs: RBObstacle,
	segX1: number, segY1: number, segX2: number, segY2: number,
): boolean {

	if (obs.kind === 'circle') {
		return pointSegDist(obs.x, obs.y, segX1, segY1, segX2, segY2) < obs.r;
	}
	// capsule
	return segSegDist(segX1, segY1, segX2, segY2, obs.x1!, obs.y1!, obs.x2!, obs.y2!) < obs.r;
}

/**
 * 找到阻挡切线段 a→b 的第一个障碍物（按沿段距离最近者）
 */
function firstBlocker(
	a: RBNode, b: RBNode,
	obstacles: RBObstacle[],
	net: string, layer: number,
): { obs: RBObstacle; t: number } | null {
	const [tx1, ty1, tx2, ty2] = getTangents(a.x, a.y, a.r, a.rgt, b.x, b.y, b.r, b.rgt);
	const skipGroups = new Set<string>();
	if (a.group) skipGroups.add(a.group);
	if (b.group) skipGroups.add(b.group);

	const segLen = Math.hypot(tx2 - tx1, ty2 - ty1);
	let best: { obs: RBObstacle; t: number } | null = null;

	for (const obs of obstacles) {
		if (!obstacleBlocks(obs, layer, net, tx1, ty1, tx2, ty2, skipGroups)) continue;
		// 沿段位置（用障碍物中心/近端投影）
		let t: number;
		if (obs.kind === 'circle') {
			t = ((obs.x - tx1) * (tx2 - tx1) + (obs.y - ty1) * (ty2 - ty1)) / (segLen * segLen || 1);
		} else {
			const mx = (obs.x1! + obs.x2!) / 2, my = (obs.y1! + obs.y2!) / 2;
			t = ((mx - tx1) * (tx2 - tx1) + (my - ty1) * (ty2 - ty1)) / (segLen * segLen || 1);
		}
		if (!best || t < best.t) best = { obs, t };
	}
	return best;
}

/**
 * 计算插入节点 C（侧面 s）时 A→C→B 的路径长度
 */
function detourLength(a: RBNode, b: RBNode, c: { x: number; y: number; r: number }, s: boolean): number {
	const t1 = getTangents(a.x, a.y, a.r, a.rgt, c.x, c.y, c.r, s);
	const t2 = getTangents(c.x, c.y, c.r, s, b.x, b.y, b.r, b.rgt);
	return Math.hypot(t1[2] - t1[0], t1[3] - t1[1]) + Math.hypot(t2[2] - t2[0], t2[3] - t2[1]);
}


// ─────────────────────────────────────────────
// 渲染：切线 + 圆弧（RBR draw_routes 风格）
// ─────────────────────────────────────────────

/**
 * 将路径节点序列转为直线 + 圆弧图元。
 * 相邻圆之间画切线，障碍圆处画绕行圆弧（方向由 rgt 决定）。
 * terminals（r=0）处不画弧。
 */
export function renderTrack(nodes: RBNode[]): RBPrimitive[] {
	const prims: RBPrimitive[] = [];
	if (nodes.length < 2) return prims;

	for (let i = 0; i < nodes.length - 1; i++) {
		const a = nodes[i], b = nodes[i + 1];
		const [tx1, ty1, tx2, ty2] = getTangents(a.x, a.y, a.r, a.rgt, b.x, b.y, b.r, b.rgt);
		if (Math.hypot(tx2 - tx1, ty2 - ty1) > 1e-6) {
			prims.push({ kind: 'line', x1: tx1, y1: ty1, x2: tx2, y2: ty2 });
		}

		// 在 b 处画圆弧：连接本段切线的终点 (tx2,ty2) 与下一段切线的起点
		if (b.r > 0 && i + 2 < nodes.length) {
			const c = nodes[i + 2];
			const [nx1, ny1] = getTangents(b.x, b.y, b.r, b.rgt, c.x, c.y, c.r, c.rgt);
			const a1 = Math.atan2(ty2 - b.y, tx2 - b.x);
			const a2 = Math.atan2(ny1 - b.y, nx1 - b.x);
			let sweep = a2 - a1;
			// 方向由 rgt 决定（与可视图弧边一致）：
			// rgt=true（右侧绕行）→ 顺时针（负扫掠角）；false → 逆时针（正扫掠角）
			if (b.rgt) {
				while (sweep >= 0) sweep -= 2 * Math.PI;
			} else {
				while (sweep <= 0) sweep += 2 * Math.PI;
			}
			if (Math.abs(sweep) > 1e-3) {
				prims.push({ kind: 'arc', cx: b.x, cy: b.y, r: b.r, startAngle: a1, sweep });
			}
		}
	}
	return prims;
}

// ─────────────────────────────────────────────
// 障碍物几何：胶囊 → 圆群、焊盘轮廓 → 圆群
// ─────────────────────────────────────────────

/**
 * 胶囊（线段+半径）转为圆群：圆心沿线段均匀分布，间距 ≤ 1.5r，
 * 保证圆群凸包与胶囊凸包一致（等半径相邻圆的切线平行于中心线，无缝隙）。
 */
export function capsuleToCircles(
	x1: number, y1: number, x2: number, y2: number, r: number,
): Array<{ x: number; y: number; r: number }> {
	const len = Math.hypot(x2 - x1, y2 - y1);
	if (len < 1e-9) return [{ x: x1, y: y1, r }];
	const spacing = Math.max(1.8 * r, 0.5);
	const n = Math.max(1, Math.ceil(len / spacing));
	const out: Array<{ x: number; y: number; r: number }> = [];
	for (let i = 0; i <= n; i++) {
		const t = i / n;
		out.push({ x: x1 + (x2 - x1) * t, y: y1 + (y2 - y1) * t, r });
	}
	return out;
}

/** 焊盘物理轮廓 → 圆群（未膨胀），unit: mil */
export function padShapeToCircles(pad: { pad: any[]; hole?: any[] | null }): Array<{ x: number; y: number; r: number }> {
	const shape = Array.isArray(pad.pad) ? pad.pad : [];
	const w = shape[1] ?? 0;
	const h = shape[2] ?? 0;
	const shapeName = String(shape[0] ?? 'ELLIPSE').toUpperCase();

	// 通孔焊盘：圆形，半径取焊盘外径一半
	const hole = pad.hole;
	if (Array.isArray(hole) && hole.length >= 2 && hole[1] > 0) {
		return [{ x: 0, y: 0, r: Math.max(w, h) / 2 }];
	}

	if (h >= w * 1.05) {
		// 竖长焊盘（OVAL/RECT）：短边一半为半径，沿长轴三个圆（两端 + 中间）
		const r = w / 2;
		const off = h / 2 - r;
		return [
			{ x: 0, y: off, r },
			{ x: 0, y: 0, r },
			{ x: 0, y: -off, r },
		];
	}
	if (w > h * 1.05) {
		// 横长焊盘：沿 x 轴三个圆
		const r = h / 2;
		const off = w / 2 - r;
		return [
			{ x: off, y: 0, r },
			{ x: 0, y: 0, r },
			{ x: -off, y: 0, r },
		];
	}
	// 近方形：单圆（半径 = 半宽，不遮挡侧向间隙）
	return [{ x: 0, y: 0, r: Math.max(w, h) / 2 }];
}

// ─────────────────────────────────────────────
// 障碍物构建辅助：凸包角点（可绕行）+ 网格（阻挡）
// ─────────────────────────────────────────────

/** 生成 id 的简易计数器（各调用方共享自增，仅保证唯一性） */
let __obsSeq = 0;
export function nextObsId(prefix: string): string {
	return `${prefix}${__obsSeq++}`;
}

/**
 * 胶囊障碍（走线段/线条）：端点圆可绕行，沿线密集圆仅阻挡。
 * 凸包拉紧效果由端点圆之间的公切线自然产生。
 */
export function buildCapsuleObstacle(
	x1: number, y1: number, x2: number, y2: number, r: number,
	net: string | undefined, layer: number | undefined, group: string,
	noPrune = false,
): RBObstacle[] {
	const out: RBObstacle[] = [
		{ id: nextObsId('w'), kind: 'circle', x: x1, y: y1, r, net, layer, group, noPrune },
		{ id: nextObsId('w'), kind: 'circle', x: x2, y: y2, r, net, layer, group, noPrune },
	];
	for (const c of capsuleToCircles(x1, y1, x2, y2, r)) {
		out.push({ id: nextObsId('b'), kind: 'circle', x: c.x, y: c.y, r, net, layer, group, noPrune, blockerOnly: true });
	}
	return out;
}

/**
 * 多边形障碍（填充区/板框）：顶点圆可绕行（凸包切线沿边自然产生），
 * 边线 + 内部网格圆仅阻挡（防止路径从内部/缺口穿过）。
 */
export function buildPolygonObstacle(
	pts: Array<{ x: number; y: number }>, r: number,
	net: string | undefined, layer: number | undefined, group: string,
	noPrune = false,
): RBObstacle[] {
	const out: RBObstacle[] = [];
	// 顶点圆（可绕行；重复顶点去重由调用方保证）
	for (let i = 0; i < pts.length - 1; i++) {
		out.push({ id: nextObsId('w'), kind: 'circle', x: pts[i].x, y: pts[i].y, r, net, layer, group, noPrune });
	}
	// 边线阻挡圆
	for (let i = 0; i < pts.length - 1; i++) {
		for (const c of capsuleToCircles(pts[i].x, pts[i].y, pts[i + 1].x, pts[i + 1].y, r)) {
			out.push({ id: nextObsId('b'), kind: 'circle', x: c.x, y: c.y, r, net, layer, group, noPrune, blockerOnly: true });
		}
	}
	// 内部网格
	if (pts.length >= 3) {
		const xs = pts.map(p => p.x), ys = pts.map(p => p.y);
		const mnX = Math.min(...xs), mxX = Math.max(...xs), mnY = Math.min(...ys), mxY = Math.max(...ys);
		const step = r * 1.2;
		const nx = Math.max(1, Math.ceil((mxX - mnX) / step));
		const ny = Math.max(1, Math.ceil((mxY - mnY) / step));
		for (let gx = 0; gx <= nx; gx++) {
			for (let gy = 0; gy <= ny; gy++) {
				const px = mnX + ((mxX - mnX) * gx) / nx;
				const py = mnY + ((mxY - mnY) * gy) / ny;
				if (pointInPolygonPx(px, py, pts)) {
					out.push({ id: nextObsId('b'), kind: 'circle', x: px, y: py, r, net, layer, group, noPrune, blockerOnly: true });
				}
			}
		}
	}
	return out;
}

/** 射线法：点是否在多边形内 */
export function pointInPolygonPx(px: number, py: number, poly: Array<{ x: number; y: number }>): boolean {
	let inside = false;
	for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
		const xi = poly[i].x, yi = poly[i].y;
		const xj = poly[j].x, yj = poly[j].y;
		if (((yi > py) !== (yj > py)) && (px < ((xj - xi) * (py - yi)) / (yj - yi) + xi)) {
			inside = !inside;
		}
	}
	return inside;
}

/**
 * 开口折线障碍（预布走线/线条）：拐点圆可绕行，沿线密集圆仅阻挡。
 */
export function buildPolylineObstacle(
	pts: Array<{ x: number; y: number }>, r: number,
	net: string | undefined, layer: number | undefined, group: string,
	noPrune = false,
): RBObstacle[] {
	const out: RBObstacle[] = [];
	for (const p of pts) {
		out.push({ id: nextObsId('w'), kind: 'circle', x: p.x, y: p.y, r, net, layer, group, noPrune });
	}
	for (let i = 0; i < pts.length - 1; i++) {
		for (const c of capsuleToCircles(pts[i].x, pts[i].y, pts[i + 1].x, pts[i + 1].y, r)) {
			out.push({ id: nextObsId('b'), kind: 'circle', x: c.x, y: c.y, r, net, layer, group, noPrune, blockerOnly: true });
		}
	}
	return out;
}

/**
 * 计算焊盘的逃逸点：在 16 个候选方向中选「净空最大」的方向，
 * 距离 = 焊盘半径 + minClear。返回 null 表示各方向都被堵死。
 * 这使成排焊盘的走线先垂直离开焊盘行再转弯（论文中的扇出效果），
 * 避免先布走线的群封死后续焊盘的出线通道。
 */
export function computeEscapePoint(
	padX: number,
	padY: number,
	padR: number,
	obstacles: RBObstacle[],
	net: string,
	layer: number,
	minClear: number,
	toward?: { x: number; y: number },
): { x: number; y: number } | null {
	const scored = scoreEscapeDirections(padX, padY, padR, obstacles, net, layer, minClear, toward);
	if (!scored.length || scored[0].score <= 0.5) return null; // 各方向都被堵死
	return { x: scored[0].x, y: scored[0].y };
}

/**
 * 对 16 个候选逃逸方向评分（净空 + 朝向目标奖励 + 轴向平手奖励），降序返回
 */
function scoreEscapeDirections(
	padX: number,
	padY: number,
	padR: number,
	obstacles: RBObstacle[],
	net: string,
	layer: number,
	minClear: number,
	toward?: { x: number; y: number },
): Array<{ score: number; x: number; y: number }> {
	const d = (padR || 0) + minClear;
	const scored: Array<{ score: number; x: number; y: number }> = [];
	for (let k = 0; k < 16; k++) {
		const a = (k * Math.PI) / 8;
		const ex = padX + d * Math.cos(a);
		const ey = padY + d * Math.sin(a);
		// 评分：逃逸点到所有异网络障碍圆边界的最小距离
		let score = Infinity;
		for (const o of obstacles) {
			if (o.net && o.net === net) continue;
			if (o.layer !== undefined && o.layer !== layer) continue;
			const dd = Math.hypot(ex - o.x, ey - o.y) - o.r;
			if (dd < score) score = dd;
		}
		if (score === Infinity) { score = 1000; }
		// 朝向目标的方向给予奖励（走线应朝目标一侧出线）
		let towardBonus = 0;
		if (toward) {
			const tdx = toward.x - padX, tdy = toward.y - padY;
			const tl = Math.hypot(tdx, tdy);
			if (tl > 1) {
				towardBonus = ((Math.cos(a) * tdx + Math.sin(a) * tdy) / tl) * minClear * 0.8;
			}
		}
		// 平手时优先水平/垂直方向（k 为 4 的倍数是轴向）
		const axisBonus = k % 4 === 0 ? 0.5 : 0;
		scored.push({ score: score + towardBonus + axisBonus, x: ex, y: ey });
	}
	scored.sort((a, b) => b.score - a.score);
	return scored;
}

/**
 * 计算焊盘的前 N 个逃逸点备选（评分降序），用于首选方向被堵时的重试
 */
export function computeEscapeAlternatives(
	padX: number,
	padY: number,
	padR: number,
	obstacles: RBObstacle[],
	net: string,
	layer: number,
	minClear: number,
	toward: { x: number; y: number },
	n: number,
): Array<{ x: number; y: number }> {
	const scored = scoreEscapeDirections(padX, padY, padR, obstacles, net, layer, minClear, toward);
	return scored.slice(0, n).filter(s2 => s2.score > 0.5).map(s2 => ({ x: s2.x, y: s2.y }));
}
