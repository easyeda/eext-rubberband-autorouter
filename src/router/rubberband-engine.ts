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
	/** 只建图、不阻挡：可相切/骑行的辅助圆（弧的同心外包络），不参与碰撞判定 */
	soft?: boolean;
	/** 弦→弧外凸补偿量：阻挡圆 r = 基准半径 + bulge。可视图相切判定按 r-bulge
	 * （基准半径）执行，否则这点外扩会让「贴着已布走线」的切线全部判侵入 */
	bulge?: number;
	/** 即使与当前网络同名也要阻挡（焊盘自身的外扩轮廓：走线不得压过本网络焊盘） */
	forceBlock?: boolean;
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

/**
 * POLYGON 焊盘形状 → 等效 RECT（w×h + 旋转角，度）。
 *
 * 部分封装库（如 STM32F407 的 LQFP144）把矩形焊盘存成绝对坐标多边形：
 * `["POLYGON", [x0,y0,"L",x1,y1,...]]`，顶点质心恰为焊盘中心。
 * 旧解析把 shape[1]（顶点数组）当宽度、shape[2]（undefined）当高度，
 * padShapeToCircles 产出 NaN 障碍圆、padShapeDistance 返回 NaN ——
 * 这些焊盘在障碍模型里整体「隐形」，QFP 引脚排成排失守。
 *
 * 归一化：解析顶点（跳过 "L"/"A" 指令标记）→ 质心回零 → 按最长边方向
 * 旋转到主轴 → 轴对齐包围盒。旋转矩形焊盘（4 顶点）结果精确；
 * 一般多边形为保守近似。返回 null 表示不可解析。
 */
export function polygonToRect(shape: any[]): { w: number; h: number; rotationDeg: number } | null {
	const raw = Array.isArray(shape[1]) ? shape[1] : null;
	if (!raw) return null;
	const pts: Array<{ x: number; y: number }> = [];
	for (let i = 0; i + 1 < raw.length; i++) {
		if (typeof raw[i] === 'number' && typeof raw[i + 1] === 'number') {
			pts.push({ x: raw[i], y: raw[i + 1] });
			i++;
		}
	}
	// 闭合多边形的首尾重复顶点只算一次
	if (pts.length >= 2) {
		const f = pts[0], l = pts[pts.length - 1];
		if (Math.abs(f.x - l.x) < 0.01 && Math.abs(f.y - l.y) < 0.01) pts.pop();
	}
	if (pts.length < 3) return null;
	let cx = 0, cy = 0;
	for (const p of pts) { cx += p.x; cy += p.y; }
	cx /= pts.length; cy /= pts.length;
	// 主轴 = 最长边方向（矩形焊盘的长边即长轴）
	let bestLen2 = -1, theta = 0;
	for (let i = 0; i < pts.length; i++) {
		const q = pts[(i + 1) % pts.length];
		const ex = q.x - pts[i].x, ey = q.y - pts[i].y;
		const l2 = ex * ex + ey * ey;
		if (l2 > bestLen2) { bestLen2 = l2; theta = Math.atan2(ey, ex); }
	}
	const cosT = Math.cos(-theta), sinT = Math.sin(-theta);
	let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
	for (const p of pts) {
		const px = p.x - cx, py = p.y - cy;
		const rx = px * cosT - py * sinT;
		const ry = px * sinT + py * cosT;
		if (rx < minX) minX = rx;
		if (rx > maxX) maxX = rx;
		if (ry < minY) minY = ry;
		if (ry > maxY) maxY = ry;
	}
	return { w: maxX - minX, h: maxY - minY, rotationDeg: (theta * 180) / Math.PI };
}

/**
 * EDA API 的焊盘形状单位陷阱：ELLIPSE 两维单位可能不一致 ——
 * 实测全量 101 工程 6493 个 ELLIPSE：5421 例满足「第二维 ≈ 第一维×39.37」
 * （第一维毫米、第二维 mil，物理上是同一圆形焊盘，如 [1.7,66.9]），
 * 1070 例两维完全相等（同单位；其中 1001 例经焊盘半径交叉验证为 mil，
 * 仅 2 例为亚毫米级的 mm）。
 *
 * 旧规则「第一维一律当毫米」会把 [66.9,66.9]（mil）放大 39 倍成 2634mil
 * 巨型障碍，直接堵死整板（rubberband-router 基准板 14 网全灭的根因）。
 * 因此只在「混合单位特征成立」时归一化；两维相等/无关时原样返回。
 * 亚毫米圆形（a==b 且 <1mil）按毫米换算（物理上不存在 0.5mil 焊盘）。
 */
function normalizeEllipseUnits(shape: any[]): any[] {
	if (!Array.isArray(shape) || String(shape[0] ?? '').toUpperCase() !== 'ELLIPSE') return shape;
	const a = shape[1], b = shape[2];
	if (!(a > 0) || !(b > 0)) return shape;
	// 混合单位特征：第一维毫米、第二维 mil，且物理上是圆
	if (Math.abs(a * 39.3701 - b) <= Math.max(0.5, b * 0.02)) return ['ELLIPSE', b, b];
	// 两维相等：同单位。亚毫米级视为 mm（0.5mil 焊盘物理不存在）
	if (Math.abs(a - b) <= 0.01 && a < 1) return ['ELLIPSE', a * 39.3701, b * 39.3701];
	return shape;
}

/**
 * SLOT 孔焊盘的保守铜皮覆盖（按孔轴对齐的半 extent）。
 *
 * case12 USB 外壳焊盘实测：pad ["OVAL",2.2,39.4] + hole ["SLOT",19.685,51.181]
 * + rotation 90。常规解读（OVAL 尺寸=全 extent、按 pad rotation 旋转）把铜皮画成
 * 2.2×39.4mil 的细条且转到 x 轴上——但 D+ 底层走线距盘心 38.65mil 处被 EDA 判
 * d=0 触碰：真实铜皮沿孔长轴（holeRotation=0 → 全局 y）延伸 ±39.4mil，即该记录
 * 的 OVAL 第二维是**半长**。编码无文档可依，这里按孔轴做保守覆盖：
 *   半extent = max(pad 维（OVAL 按半extent、其它按全extent折半）, 孔半extent + 2.2mil 环宽)
 * 宁可多盖：外壳盘附近少布一根线，好过 TH Pad to Track d=0（过盖范围受邻盘
 * 间距天然限制，实测不与相邻 0.5mm 排焊盘冲突）。非 SLOT 孔返回 null（维持原逻辑）。
 */
export function slotCoverHalfExtents(pad: { pad: any[]; hole?: any[] | null; holeRotation?: number }): { hx: number; hy: number; rotDeg: number } | null {
	const hole = pad.hole;
	if (!Array.isArray(hole) || String(hole[0] ?? '').toUpperCase() !== 'SLOT') return null;
	const hw = Number(hole[1]) || 0, hh = Number(hole[2]) || 0;
	if (!(hw > 0) || !(hh > 0)) return null;
	const shape = Array.isArray(pad.pad) ? pad.pad : [];
	const name = String(shape[0] ?? '').toUpperCase();
	const d1 = Number(shape[1]) || 0, d2 = Number(shape[2]) || 0;
	const p1 = name === 'OVAL' ? d1 : d1 / 2;
	const p2 = name === 'OVAL' ? d2 : d2 / 2;
	const RING = 2.2;
	return {
		hx: Math.max(p1, hw / 2 + RING),
		hy: Math.max(p2, hh / 2 + RING),
		rotDeg: Number(pad.holeRotation) || 0,
	};
}

/**
 * 焊盘物理轮廓 → 圆群（未膨胀），unit: mil。
 *
 * 长焊盘（OVAL/RECT 胶囊）用「两端帽圆 + 密集侧向填充圆」表示：
 * 稀疏三圆在斜 45° 方向上会把焊盘边界低估约 (长轴/2 − 短轴/2)·sin45°
 * （23.4×77.5 焊盘低估 19mil），导致出线候选点、过孔落点跑进焊盘铜皮里。
 * 端帽圆是凸包极点（参与切点生成），中间填充圆只做碰撞判定。
 * 所有圆都按 rotation 旋转（EDA 角度约定：逆时针为正）。
 */
export function padShapeToCircles(
	pad: { pad: any[]; hole?: any[] | null; rotation?: number; holeRotation?: number },
): Array<{ x: number; y: number; r: number; blockerOnly?: boolean }> {
	const rawShape = Array.isArray(pad.pad) ? pad.pad : [];
	// POLYGON（绝对坐标顶点）先归一化成 RECT：顶点已含旋转，pad.rotation 不再叠加
	// ELLIPSE 先做单位归一化（第一维毫米陷阱）
	let shape = normalizeEllipseUnits(rawShape);
	let rotationDeg = pad.rotation ?? 0;
	if (String(shape[0] ?? '').toUpperCase() === 'POLYGON') {
		const rect = polygonToRect(rawShape);
		if (!rect) return [];
		shape = ['RECT', rect.w, rect.h];
		rotationDeg = rect.rotationDeg;
	}
	const w = shape[1] ?? 0;
	const h = shape[2] ?? 0;
	const shapeName = String(shape[0] ?? 'ELLIPSE').toUpperCase();
	// NaN 防御：非有限尺寸一律按无铜皮处理（返回孔圆），不让 NaN 流进障碍集
	const rot = ((rotationDeg ?? 0) * Math.PI) / 180;
	const cosR = Math.cos(rot), sinR = Math.sin(rot);
	const rotAll = (arr: Array<{ x: number; y: number; r: number; blockerOnly?: boolean }>) =>
		arr.map(c => {
			const o: { x: number; y: number; r: number; blockerOnly?: boolean } = {
				x: c.x * cosR - c.y * sinR,
				y: c.x * sinR + c.y * cosR,
				r: c.r,
			};
			if (c.blockerOnly) o.blockerOnly = true;
			return o;
		});

	// 通孔焊盘的铜皮外形仍按 pad 形状建模：孔只影响钻孔，不应缩小铜皮障碍。
	// （旧逻辑把带孔焊盘一律当圆，RECT 角部会漏出 ~1mil，造成贴边 DRC）
	const hole = pad.hole;
	// SLOT 孔焊盘：按孔轴对齐的保守覆盖（编码歧义，见 slotCoverHalfExtents 注释）
	const slot = slotCoverHalfExtents(pad);
	if (slot) {
		const out = capsuleOutline(slot.hx * 2, slot.hy * 2);
		const srot = (slot.rotDeg * Math.PI) / 180;
		const cs = Math.cos(srot), sn = Math.sin(srot);
		return out.map(c => {
			const o: { x: number; y: number; r: number; blockerOnly?: boolean } = {
				x: c.x * cs - c.y * sn,
				y: c.x * sn + c.y * cs,
				r: c.r,
			};
			if (c.blockerOnly) o.blockerOnly = true;
			return o;
		});
	}
	// !(w > 0) 同时拦住 0 / NaN（NaN 与任何比较均为 false，=== 0 拦不住）
	if (!(w > 0) || !(h > 0)) {
		const r = Array.isArray(hole) && hole.length >= 2 ? Math.max(0, hole[1] ?? 0) / 2 : 0;
		return r > 0 ? [{ x: 0, y: 0, r }] : [];
	}

	if (shapeName === 'RECT' || shapeName === 'ROUNDRECT') {
		if (Math.abs(w - h) <= Math.min(w, h) * 0.3) {
			// 近方形矩形焊盘：3 圆体覆盖主体 + 4 角圆覆盖角部
			// （外接圆会封死相邻焊盘间隙；3 圆体的角部外露会造成 TH Pad 违规）
			const rBody = w / 2;
			const out = [
				{ x: 0, y: h / 2 - rBody, r: rBody },
				{ x: 0, y: 0, r: rBody },
				{ x: 0, y: -(h / 2 - rBody), r: rBody },
			];
			const k = 0.65; // 角圆中心内缩系数
			for (const sx of [-1, 1]) {
				for (const sy of [-1, 1]) {
					const cxp = (sx * w) / 2 * k;
					const cyp = (sy * h) / 2 * k;
					const rc = (1 - k) * Math.hypot(w / 2, h / 2);
					out.push({ x: cxp, y: cyp, r: rc });
				}
			}
			return rotAll(out);
		}
		// 长条矩形/圆角矩形（FPC、连接器焊盘常见）：主体按胶囊 + 两端各一个角圆。
		// 胶囊会漏掉矩形角部（23.4×77.5 焊盘角外露 ~17.8mil，走线贴角造成
		// SMD Pad↔Track 间距违规）。角圆取端帽圆心、半径 = 短半边·√2：边界
		// 恰好过 4 个角点并完整盖住短边；横向比焊盘宽 ~0.21×短边（保守方向，
		// 把本就过不了 DRC 的窄缝正确堵死）。blockerOnly：只做碰撞判定，
		// 不产生切点（切线可能切进焊盘中段）。
		const out = capsuleOutline(w, h);
		const rCorner = (Math.min(w, h) / 2) * Math.SQRT2;
		const halfC = Math.max(w, h) / 2 - Math.min(w, h) / 2;
		const vertical = h >= w;
		for (const s of [-1, 1]) {
			out.push(vertical
				? { x: 0, y: s * halfC, r: rCorner, blockerOnly: true }
				: { x: s * halfC, y: 0, r: rCorner, blockerOnly: true });
		}
		return rotAll(out);
	}
	return rotAll(capsuleOutline(w, h));
}

/**
 * 点到焊盘铜皮的有符号距离（内部为负），unit: mil。
 * 用于过孔落点校验：过孔铜环（半径 viaR）必须完全落在自家焊盘铜皮之外，
 * 只按「沿出线方向步进 viaR」是不够的——斜向出线时距离增长率只有 cosθ，
 * 45° 方向会少走出去 ~30%，过孔铜环仍会压进焊盘。
 */
export function padShapeDistance(
	pad: { pad: any[]; hole?: any[] | null; rotation?: number; holeRotation?: number },
	dx: number, dy: number,
): number {
	const rawShape = Array.isArray(pad.pad) ? pad.pad : [];
	// SLOT 孔焊盘：与 padShapeToCircles 同一保守覆盖（矩形距离，角部略偏严=安全方向）
	const slot = slotCoverHalfExtents(pad);
	if (slot) {
		const srot = (slot.rotDeg * Math.PI) / 180;
		const cs = Math.cos(srot), sn = Math.sin(srot);
		const lx = dx * cs + dy * sn;
		const ly = -dx * sn + dy * cs;
		const qx = Math.abs(lx) - slot.hx, qy = Math.abs(ly) - slot.hy;
		if (qx <= 0 && qy <= 0) return Math.max(qx, qy);
		return Math.hypot(Math.max(qx, 0), Math.max(qy, 0));
	}
	// POLYGON（绝对坐标顶点）先归一化成 RECT：顶点已含旋转，pad.rotation 不再叠加
	// ELLIPSE 先做单位归一化（第一维毫米陷阱）
	let shape = normalizeEllipseUnits(rawShape);
	let rotationDeg = pad.rotation ?? 0;
	if (String(shape[0] ?? '').toUpperCase() === 'POLYGON') {
		const rect = polygonToRect(rawShape);
		if (!rect) return Math.hypot(dx, dy);
		shape = ['RECT', rect.w, rect.h];
		rotationDeg = rect.rotationDeg;
	}
	const w = shape[1] ?? 0, h = shape[2] ?? 0;
	const shapeName = String(shape[0] ?? 'ELLIPSE').toUpperCase();
	// !(w > 0) 同时拦住 0 / NaN：NaN 尺寸按无铜皮处理（点到焊盘距离=欧氏距离）
	if (!(w > 0) || !(h > 0)) return Math.hypot(dx, dy);
	const rot = ((rotationDeg ?? 0) * Math.PI) / 180;
	const cosR = Math.cos(rot), sinR = Math.sin(rot);
	const lx = dx * cosR + dy * sinR;
	const ly = -dx * sinR + dy * cosR;
	const a = w / 2, b = h / 2;
	const rectDist = (x: number, y: number, ra: number, rb: number) => {
		const qx = Math.abs(x) - ra, qy = Math.abs(y) - rb;
		if (qx <= 0 && qy <= 0) return Math.max(qx, qy);
		return Math.hypot(Math.max(qx, 0), Math.max(qy, 0));
	};
	if (shapeName === 'RECT') return rectDist(lx, ly, a, b);
	if (shapeName === 'OVAL' || shapeName === 'ROUNDRECT') {
		const r = Math.min(a, b);
		const half = Math.max(a, b) - r;
		const along = b >= a ? ly : lx;
		const perp = b >= a ? lx : ly;
		return Math.hypot(perp, Math.max(Math.abs(along) - half, 0)) - r;
	}
	// ELLIPSE（含近圆）：按椭圆径向缩放估算，偏保守（低估 → 过孔只会走得更远）
	if (Math.abs(a - b) < 0.01) return Math.hypot(lx, ly) - a;
	const k = Math.hypot(lx / a, ly / b);
	return (k - 1) * Math.min(a, b);
}

/**
 * 胶囊（OVAL / 长 RECT）轮廓圆群：端帽圆 + 侧向密集填充圆。
 * 填充圆间距 ≤ 0.4·r，半径外扩半个弦高，保证圆群并集覆盖胶囊且不超过 +0.25mil。
 */
function capsuleOutline(w: number, h: number): Array<{ x: number; y: number; r: number; blockerOnly?: boolean }> {
	const r = Math.min(w, h) / 2;
	const half = Math.max(w, h) / 2 - r; // 端帽圆心到中心距离
	const vertical = h >= w;
	const P = (t: number, rr: number, blockerOnly?: boolean) => ({
		x: vertical ? 0 : t,
		y: vertical ? t : 0,
		r: rr,
		...(blockerOnly ? { blockerOnly: true } : {}),
	});
	if (half <= 1e-6) return [P(0, r)];
	const out = [P(half, r), P(-half, r)];
	const spacing = Math.max(0.5, r * 0.4);
	const n = Math.max(2, Math.ceil((2 * half) / spacing));
	const d = (2 * half) / n;
	const sag = r - Math.sqrt(Math.max(0, r * r - (d / 2) * (d / 2)));
	for (let i = 1; i < n; i++) out.push(P(-half + d * i, r + sag, true));
	return out;
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

// ─────────────────────────────────────────────
// 角部圆角（fillet）：两条相交线段之间插入切线圆弧，实现 G1 光滑过渡
// ─────────────────────────────────────────────

export interface FilletResult {
	/** 入线段截短后的终点 */
	trimIn: { x: number; y: number };
	/** 切线圆弧 */
	arc: RBArc;
	/** 出线段截短后的起点 */
	trimOut: { x: number; y: number };
}

/**
 * 在拐点 P 处计算圆角：入方向 d1（指向 P）、出方向 d2（离开 P）、圆���半径 R。
 * A = P − d1·t，B = P + d2·t，t = R·tan(φ/2)（φ = 转向角）。
 * 返回截短点与圆弧；无法圆角时返回 null。
 */
export function cornerFillet(
	px: number, py: number,
	dx1: number, dy1: number,
	dx2: number, dy2: number,
	radius: number,
): FilletResult | null {
	const l1 = Math.hypot(dx1, dy1), l2 = Math.hypot(dx2, dy2);
	if (l1 < 1e-9 || l2 < 1e-9) return null;
	const ux1 = dx1 / l1, uy1 = dy1 / l1;
	const ux2 = dx2 / l2, uy2 = dy2 / l2;
	const dot = Math.max(-1, Math.min(1, ux1 * ux2 + uy1 * uy2));
	const phi = Math.acos(dot); // 转向角
	if (phi < 0.06) return null;       // 近似直行：无需圆角
	if (phi > Math.PI - 0.06) return null; // 掉头：无法圆角
	const half = phi / 2;
	const tanHalf = Math.tan(half);
	if (tanHalf < 1e-6) return null;
	const t = radius * tanHalf;
	// 转向侧：cross(d1,d2) < 0 → 右转（顺时针），> 0 → 左转（逆时针）
	const cross = ux1 * uy2 - uy1 * ux2;
	const sgn = cross >= 0 ? 1 : -1;
	// 切点
	const ax = px - ux1 * t, ay = py - uy1 * t;
	const bx = px + ux2 * t, by = py + uy2 * t;
	// 圆心 = A + R×法线（转向侧法线：d1 旋转 sgn×90°）
	const n1x = -sgn * uy1, n1y = sgn * ux1;
	const cx = ax + radius * n1x;
	const cy = ay + radius * n1y;
	// 校验 B 在圆上（数值容差）
	const rErr = Math.abs(Math.hypot(bx - cx, by - cy) - radius);
	if (rErr > 0.5) return null;
	const startAngle = Math.atan2(ay - cy, ax - cx);
	const endAngle = Math.atan2(by - cy, bx - cx);
	// 扫掠：转向侧决定方向，幅值 = φ
	const sweep = sgn > 0 ? phi : -phi;
	return {
		trimIn: { x: ax, y: ay },
		arc: { kind: 'arc', cx, cy, r: radius, startAngle, sweep },
		trimOut: { x: bx, y: by },
	};
}

/**
 * 对「出线段→路径第一段」的拐点做圆角。
 * stub 指向拐点（stub.x2/y2 = 拐点），first 从拐点出发（first.x1/y1 = 拐点）。
 */
export function fanoutFillet(
	stub: { x1: number; y1: number; x2: number; y2: number },
	first: { x1: number; y1: number; x2: number; y2: number },
	radius: number,
): { prims: RBPrimitive[] } | null {
	const d1x = stub.x2 - stub.x1, d1y = stub.y2 - stub.y1;
	const d2x = first.x2 - first.x1, d2y = first.y2 - first.y1;
	const stubLen = Math.hypot(d1x, d1y);
	const firstLen = Math.hypot(d2x, d2y);
	if (stubLen < 2 || firstLen < 2) return null;
	const dot = (d1x * d2x + d1y * d2y) / (stubLen * firstLen);
	const phi = Math.acos(Math.max(-1, Math.min(1, dot)));
	if (phi < 0.06 || phi > Math.PI - 0.06) return null;
	const tanHalf = Math.tan(phi / 2);
	// R 受两段长度限制（切点距拐点 t = R·tan(φ/2) 需 ≤ 段长 − 余量）
	const R = Math.min(radius, (stubLen - 1) / tanHalf, (firstLen - 1) / tanHalf);
	if (R < 1) return null;
	const f = cornerFillet(stub.x2, stub.y2, d1x, d1y, d2x, d2y, R);
	if (!f) return null;
	return {
		prims: [
			{ kind: 'line', x1: stub.x1, y1: stub.y1, x2: f.trimIn.x, y2: f.trimIn.y },
			f.arc,
			{ kind: 'line', x1: f.trimOut.x, y1: f.trimOut.y, x2: first.x2, y2: first.y2 },
		],
	};
}

// ─────────────────────────────────────────────
// 焊盘出线模型：SMD 长焊盘从端帽沿长轴出线
// ─────────────────────────────────────────────

export interface PadExitModel {
	/** 端帽圆心（绝对坐标）：走线从此处沿轴向离开 */
	capX: number;
	capY: number;
	/** 轴向单位向量（指向出线侧） */
	axisX: number;
	axisY: number;
	/** 短轴半径（端帽圆半径，未膨胀） */
	r: number;
}

/**
 * 计算焊盘的出线模型：长圆形/矩形焊盘从端帽沿长轴出线（考虑焊盘长宽与旋转角），
 * 近圆形焊盘返回 null（走 scored 方向出线）。
 * sideHint: 期望出线侧（目标方向），用于选择沿轴的正/负方向。
 */
export function padExitModel(
	padX: number,
	padY: number,
	pad: { pad: any[]; hole?: any[] | null; rotation?: number },
	sideHint: { x: number; y: number },
): PadExitModel | null {
	const shape = normalizeEllipseUnits(Array.isArray(pad.pad) ? pad.pad : []);
	let w = shape[1] ?? 0;
	let h = shape[2] ?? 0;
	let rotation = pad.rotation ?? 0;
	const shapeName = String(shape[0] ?? 'ELLIPSE').toUpperCase();
	if (shapeName === 'POLYGON') {
		// 绝对坐标顶点归一化成 RECT：铜皮轮廓才是出线依据，封装里常见的
		// 微小钻孔标记（如 LQFP144 的 8.5mil 孔）不否定轴向
		const rect = polygonToRect(shape);
		if (!rect) return null;
		w = rect.w;
		h = rect.h;
		rotation = rect.rotationDeg;
	} else if (Array.isArray(pad.hole) && pad.hole.length >= 2 && pad.hole[1] > 0) {
		// 通孔圆焊盘：无轴向
		return null;
	}
	// 长宽比不足：无明确轴向（!isFinite 拦住 NaN，NaN 与任何比较均为 false）
	const longAx = Math.max(w, h), shortAx = Math.min(w, h);
	if (!Number.isFinite(longAx) || longAx < shortAx * 1.4 || shortAx < 1e-6) return null;

	// 长轴方向：本地 +Y 旋转 rotation 度（EDA 角度约定：逆时针为正）
	const rot = (rotation * Math.PI) / 180;
	// 本地长轴：h >= w 时沿 Y，否则沿 X
	let axLocal: number, ayLocal: number;
	if (h >= w) { axLocal = 0; ayLocal = 1; } else { axLocal = 1; ayLocal = 0; }
	// 旋转（屏幕坐标 y 向上，逆时针为正）
	const cosR = Math.cos(rot), sinR = Math.sin(rot);
	let axisX = axLocal * cosR - ayLocal * sinR;
	let axisY = axLocal * sinR + ayLocal * cosR;
	const al = Math.hypot(axisX, axisY);
	if (al < 1e-9) return null;
	axisX /= al; axisY /= al;

	// 轴向选择：与 sideHint 同侧
	if (axisX * sideHint.x + axisY * sideHint.y < 0) { axisX = -axisX; axisY = -axisY; }

	// 端帽圆心：焊盘中心沿轴偏移 (longAx/2 − shortAx/2)
	const off = longAx / 2 - shortAx / 2;
	const r = shortAx / 2;
	return {
		capX: padX + axisX * off,
		capY: padY + axisY * off,
		axisX, axisY, r,
	};
}
