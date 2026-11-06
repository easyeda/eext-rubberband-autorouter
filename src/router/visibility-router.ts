/**
 * VisibilityRouter - 可视图 + Dijkstra：绕圆障碍的最短路径（精确解）
 *
 * 原理：绕一组圆盘的最短路径由「圆之间的公切线段 + 沿圆的圆弧」组成
 * （即橡皮筋凸包拉紧效果，见 Tal Dayan 博士论文 / Ruby-PCB-Router）。
 * 把每条有效公切线段和每段绕行圆弧作为图的边，Dijkstra 求出的就是
 * 全局最短的橡皮筋路径，无贪心震荡问题。
 *
 * 输出：精确图元序列（直线 + 圆弧），几何与 Dijkstra 选择的边完全一致。
 *
 * 多端点（multi-endpoint）：源/目标各给一组候选出线点，Dijkstra 以全部
 * 源候选为起点（初始代价 = 引出段长度）、全部目标候选为终点（再叠加
 * 引入段长度），取全局最小者。这样「从焊盘哪一侧出线」不再由启发式决定，
 * 而是由路径总长决定 —— 直线可连时总长即等于两焊盘直线距离。
 *
 * 性能：
 *  - 走廊剪枝（只保留直连线附近的圆，实心障碍 noPrune 除外）
 *  - blockerOnly 圆（填充内部网格、焊盘自身轮廓）只参与阻挡判定，不生成切点/边
 *  - 均匀网格 + DDA 加速线段/圆弧的阻挡查询
 */
import {
	RBObstacle, RBPrimitive, getTangents, capsuleToCircles, pointSegDist, dist,
} from './rubberband-engine';

export interface RouteConnectionResult {
	ok: boolean;
	/** 精确图元序列（直线 + 圆弧） */
	prims: RBPrimitive[];
}

/** 求解累计统计（常驻、零 IO；测试台读它定位热点，不必开 debug 日志） */
export const vrStats = {
	calls: 0,
	failCalls: 0,
	solveMs: 0,
	pairMs: 0,
	termMs: 0,
	arcMs: 0,
	dijMs: 0,
	maxN: 0,
	sumN: 0,
	pairsVisited: 0,
	tubeTries: 0,
	tubePaths: 0,
	tubeSolveOk: 0,
};

export const vrStatsReset = (): void => {
	vrStats.calls = 0; vrStats.failCalls = 0; vrStats.solveMs = 0;
	vrStats.pairMs = 0; vrStats.termMs = 0; vrStats.arcMs = 0; vrStats.dijMs = 0;
	vrStats.maxN = 0; vrStats.sumN = 0; vrStats.pairsVisited = 0;
	vrStats.tubeTries = 0; vrStats.tubePaths = 0; vrStats.tubeSolveOk = 0;
};

/** 端点候选：位置 + 附加代价（引出/引入段长度，参与最短路比较） */
export interface RouteEndpoint {
	x: number;
	y: number;
	cost: number;
}

/** 多端点寻路结果 */
export interface CandidateRouteResult {
	ok: boolean;
	/** 中间路径图元（不含首尾引出/引入段） */
	prims: RBPrimitive[];
	/** 实际选中的源/目标候选 */
	src?: RouteEndpoint;
	dst?: RouteEndpoint;
}

/** 视图节点：圆上的切点或终端 */
interface VGNode {
	/** 圆下标；-1 表示终端 */
	circle: number;
	x: number;
	y: number;
	angle: number;
	adj: Array<VGEdge>;
}

/** 图边：切线段或圆弧，自带精确几何 */
interface VGEdge {
	to: number;
	cost: number;
	isArc: boolean;
	/** 切线段端点（isArc=false） */
	x1?: number; y1?: number; x2?: number; y2?: number;
	/** 圆弧（isArc=true）：圆心/半径/起始角/扫掠角 */
	cx?: number; cy?: number; r?: number; startAngle?: number; sweep?: number;
}

const EPS_TOUCH = 0.02;   // 相切容差（浮点误差）
const GRID_CELL = 100;    // 阻挡查询网格（≥ 最大障碍半径即可）
/**
 * 链相切容差（mil）：同一物理铜链（已布走线拆成的重叠圆链）在拐弯处，
 * 相邻圆的外公切线/贴边弧会切入相邻圆的膨胀盘 0.1~1.2mil —— 这是「弦 vs 弧」
 * 的矢高误差，不是真实间距违规（障碍半径本就含 w/2+间距+w/2+0.1 的余量）。
 * 若不容忍，可视图里「贴着已布走线绕弯」的切线全部判侵入，路径被迫绕大圈，
 * 并在绕行圆上产生 180° 折返尖刺（V 形折角）。
 * 仅当阻挡圆与被切圆同网、同层且圆盘相交（= 同一条铜链的相邻圆）时放行；
 * 焊盘/静态障碍不在此列（它们不与走线圆构成铜链）。
 */
const CHAIN_TOL = 0.05;

/** 阻挡圆 k 对被切圆 t 的链容差：同一铜链（同网同层且圆盘相交）时放行 CHAIN_TOL */
function chainTolFor(k: RBObstacle, t: RBObstacle | undefined): number {
	if (!t || !k.net || k.net !== t.net) return 0;
	if ((k.layer ?? -1) !== (t.layer ?? -1)) return 0;
	// 圆盘不相交 → 不是同一条铜链上的相邻圆，不给容差
	if (Math.hypot(k.x - t.x, k.y - t.y) >= k.r + t.r) return 0;
	return CHAIN_TOL;
}

/**
 * 均匀网格：按圆心插入；查询与线段/点可能相交的圆下标。
 * 用扁平数组 + 戳记法避免每查询分配 Set / Map 哈希。
 */
class CircleGrid {
	// 稠密局部网格：桶坐标以所有圆的包围盒为原点重新编号。
	// 旧实现用 (gx+2048)*4096+(gy+2048) 的稀疏键，buckets.length 恒为
	// ~8.4M，每个求解器都要 new Int32Array(8.4M)（~33MB）——case12 这种
	// 大板 200+ 次求解时是纯 GC/清零开销。稠密网格的格子数只有
	// (板宽/100)×(板高/100)（~25×22），分配和查表都便宜一个数量级。
	private buckets: number[][] = [];
	private stamp: Int32Array;
	private curStamp = 0;
	private circles: RBObstacle[];
	private minGx = 0;
	private minGy = 0;
	private w = 1;
	private h = 1;
	/** 异常大坐标范围时退回旧的稀疏桶（避免 w*h 爆内存） */
	private legacy = false;

	private key(gx: number, gy: number): number {
		const lx = gx - this.minGx, ly = gy - this.minGy;
		if (this.legacy) return (gx + 2048) * 4096 + (gy + 2048);
		if (lx < 0 || ly < 0 || lx >= this.w || ly >= this.h) return -1;
		return ly * this.w + lx;
	}

	/** 访问一个格子（去重）；cb 返回 true 时终止 */
	private visit(key: number, stampId: number, cb: (idx: number) => boolean | void): boolean {
		if (key < 0) return false;
		if (this.legacy) {
			// 稀疏退回路径用 Set 去重（极少触发）
			const seen = (this as any).__seen as Set<number>;
			if (seen.has(key)) return false;
			seen.add(key);
		} else {
			if (this.stamp[key] === stampId) return false;
			this.stamp[key] = stampId;
		}
		const arr = this.buckets[key];
		if (arr) for (let k = 0; k < arr.length; k++) if (cb(arr[k]) === true) return true;
		return false;
	}

	constructor(circles: RBObstacle[]) {
		this.circles = circles;
		if (!circles.length) { this.stamp = new Int32Array(0); return; }
		// 圆的 bbox 覆盖范围（插入半径上限 400mil，防异常数据撑爆）
		const ranges: Array<[number, number, number, number]> = new Array(circles.length);
		let minGx = Infinity, maxGx = -Infinity, minGy = Infinity, maxGy = -Infinity;
		for (let i = 0; i < circles.length; i++) {
			const c = circles[i];
			const r = Math.min(Math.max(c.r, 0), 400);
			const i0 = Math.floor((c.x - r) / GRID_CELL), i1 = Math.floor((c.x + r) / GRID_CELL);
			const j0 = Math.floor((c.y - r) / GRID_CELL), j1 = Math.floor((c.y + r) / GRID_CELL);
			ranges[i] = [i0, i1, j0, j1];
			if (i0 < minGx) minGx = i0;
			if (i1 > maxGx) maxGx = i1;
			if (j0 < minGy) minGy = j0;
			if (j1 > maxGy) maxGy = j1;
		}
		this.minGx = minGx;
		this.minGy = minGy;
		this.w = maxGx - minGx + 1;
		this.h = maxGy - minGy + 1;
		const total = this.w * this.h;
		if (total > 2_000_000) {
			// 兜底：坐标跨度异常（通常是脏数据里的离群点），保持旧行为
			this.legacy = true;
			this.buckets = [];
			for (let i = 0; i < circles.length; i++) {
				const c = circles[i];
				const k = (Math.floor(c.x / GRID_CELL) + 2048) * 4096 + (Math.floor(c.y / GRID_CELL) + 2048);
				let arr = this.buckets[k];
				if (!arr) { arr = []; this.buckets[k] = arr; }
				arr.push(i);
			}
			this.stamp = new Int32Array(0);
			return;
		}
		this.buckets = new Array(total);
		// 多格插入：把圆放进它 bbox 覆盖的每个格子。这样射线只查经过的
		// 格子，不需要每个采样点再查 3×3 邻域（旧实现 9 倍桶查找量）。
		for (let i = 0; i < circles.length; i++) {
			const [i0, i1, j0, j1] = ranges[i];
			for (let gx = i0; gx <= i1; gx++) {
				const lx = gx - minGx;
				for (let gy = j0; gy <= j1; gy++) {
					const key = lx + (gy - minGy) * this.w;
					let arr = this.buckets[key];
					if (!arr) { arr = []; this.buckets[key] = arr; }
					arr.push(i);
				}
			}
		}
		this.stamp = new Int32Array(total);
	}

	/** 遍历线段经过的格子里的圆（多格插入 + 去重，不会重复） */
	nearSegment(x1: number, y1: number, x2: number, y2: number, cb: (idx: number) => boolean | void): void {
		const stampId = ++this.curStamp;
		if (this.legacy) {
			// 旧稀疏路径：按采样点查 3×3
			const seen = new Set<number>();
			(this as any).__seen = seen;
			const d = Math.hypot(x2 - x1, y2 - y1);
			const steps = Math.max(1, Math.ceil(d / (GRID_CELL * 0.8)));
			for (let s = 0; s <= steps; s++) {
				const t = s / steps;
				const px = x1 + (x2 - x1) * t, py = y1 + (y2 - y1) * t;
				const gx = Math.floor(px / GRID_CELL), gy = Math.floor(py / GRID_CELL);
				for (let dx = -1; dx <= 1; dx++) for (let dy = -1; dy <= 1; dy++) {
					if (this.visit(this.key(gx + dx, gy + dy), stampId, cb)) return;
				}
			}
			(this as any).__seen = null;
			return;
		}
		// 网格 DDA（Amanatides-Woo）：只走线段真正穿过的格子
		let gx = Math.floor(x1 / GRID_CELL), gy = Math.floor(y1 / GRID_CELL);
		const gxEnd = Math.floor(x2 / GRID_CELL), gyEnd = Math.floor(y2 / GRID_CELL);
		const dx = x2 - x1, dy = y2 - y1;
		const stepX = dx > 0 ? 1 : dx < 0 ? -1 : 0;
		const stepY = dy > 0 ? 1 : dy < 0 ? -1 : 0;
		let tMaxX = stepX === 0 ? Infinity : ((stepX > 0 ? (gx + 1) * GRID_CELL : gx * GRID_CELL) - x1) / dx;
		let tMaxY = stepY === 0 ? Infinity : ((stepY > 0 ? (gy + 1) * GRID_CELL : gy * GRID_CELL) - y1) / dy;
		const tDeltaX = stepX === 0 ? Infinity : Math.abs(GRID_CELL / dx);
		const tDeltaY = stepY === 0 ? Infinity : Math.abs(GRID_CELL / dy);
		if (this.visit(this.key(gx, gy), stampId, cb)) return;
		let guard = 0;
		while ((gx !== gxEnd || gy !== gyEnd) && guard++ < 20000) {
			if (tMaxX < tMaxY) {
				gx += stepX; tMaxX += tDeltaX;
			} else if (tMaxY < tMaxX) {
				gy += stepY; tMaxY += tDeltaY;
			} else {
				// 正好穿过格点：两个方向依次走，避免漏格
				gx += stepX; tMaxX += tDeltaX;
				if (this.visit(this.key(gx, gy), stampId, cb)) return;
				gy += stepY; tMaxY += tDeltaY;
			}
			if (this.visit(this.key(gx, gy), stampId, cb)) return;
		}
	}

	/** 遍历点所在格子里的圆（多格插入保证覆盖） */
	nearPoint(px: number, py: number, cb: (idx: number) => void): void {
		const key = this.key(Math.floor(px / GRID_CELL), Math.floor(py / GRID_CELL));
		if (key < 0) return;
		const arr = this.buckets[key];
		if (arr) for (let k = 0; k < arr.length; k++) cb(arr[k]);
	}
}

/**
 * 线段是否不侵入任何圆（快速网格查询；touchedA/B 为允许相切的圆下标）
 */
function makeSegmentChecker(circles: RBObstacle[], grid: CircleGrid, net = '') {
	// 弦→弧外凸补偿：带 bulge 的阻挡圆按基准半径（r - bulge）判相切，
	// 让切线能贴着已布走线的圆链走（否则外扩量会封死所有贴边切线）
	const effR = (c: RBObstacle) => c.r - (c.bulge ?? 0);
	return (x1: number, y1: number, x2: number, y2: number, touchedA: number, touchedB: number): boolean => {
		let blocked = false;
		// 链容差：阻挡圆与被切圆（切线两端所在圆）属于同一铜链时，允许
		// 弦vs弧矢高量级（≤CHAIN_TOL）的切入——那是建模误差而非真违规
		const tA = touchedA >= 0 ? circles[touchedA] : undefined;
		const tB = touchedB >= 0 ? circles[touchedB] : undefined;
		grid.nearSegment(x1, y1, x2, y2, (k) => {
			if (k === touchedA || k === touchedB) return;
			const c = circles[k];
			if (c.soft) return;
			if (net && c.net === net && !c.forceBlock) return;
			const tol = Math.max(chainTolFor(c, tA), chainTolFor(c, tB));
			if (pointSegDist(c.x, c.y, x1, y1, x2, y2) < effR(c) - EPS_TOUCH - tol) { blocked = true; return true; }   // 挡死：终止遍历
		});
		return !blocked;
	};
}

/**
 * 圆弧（从 a1 扫过 sweep）是否不侵入其他圆
 */
function makeArcChecker(circles: RBObstacle[], grid: CircleGrid, net = '') {
	return (cx: number, cy: number, r: number, a1: number, sweep: number, selfIdx: number): boolean => {
		const steps = Math.max(4, Math.ceil(Math.abs(sweep) * r / (GRID_CELL * 0.5)));
		let blocked = false;
		// 链容差：贴边绕行弧与弧自身所在圆同链的圆，允许矢高量级切入
		const self = selfIdx >= 0 ? circles[selfIdx] : undefined;
		for (let s = 0; s <= steps && !blocked; s++) {
			const a = a1 + (sweep * s) / steps;
			const px = cx + r * Math.cos(a), py = cy + r * Math.sin(a);
			grid.nearPoint(px, py, (k) => {
				if (blocked || k === selfIdx) return;
				const o = circles[k];
				if (o.soft) return;
				if (net && o.net === net && !o.forceBlock) return;
				const tol = chainTolFor(o, self);
				if (Math.hypot(px - o.x, py - o.y) < o.r - (o.bulge ?? 0) - EPS_TOUCH - tol) blocked = true;
			});
		}
		return !blocked;
	};
}

/**
 * 让出主线程（协作式调度）。
 * 可视图求解是重 CPU 的同步计算，若不主动让出，EDA 主线程会被独占，
 * 表现为「整个 EDA 卡死 + console 无报错」（阻塞而非异常）。
 *
 * 注意：不能用 requestAnimationFrame——EDA 窗口在后台/无头时 rAF 会被节流到
 * 约 1fps，会让总耗时爆炸。MessageChannel 是没有任何节流的宏任务让出方式。
 */
let __yieldChannel: MessageChannel | null = null;
let __yieldResolvers: Array<() => void> = [];
function ensureYieldChannel(): MessageChannel | null {
	if (__yieldChannel) return __yieldChannel;
	if (typeof MessageChannel !== 'function') return null;
	const ch = new MessageChannel();
	ch.port1.onmessage = () => {
		const rs = __yieldResolvers;
		__yieldResolvers = [];
		for (const r of rs) r();
	};
	__yieldChannel = ch;
	return ch;
}

function yieldToHost(): Promise<void> {
	// 注意：不要用 scheduler.yield——EDA 的 Electron 版本里它是假让出
	// （立即 resolve，实测 0.03ms，不把控制权交还事件循环，UI 仍会冻结）。
	// MessageChannel 是唯一既无节流又真正让出的原语；退路用 setTimeout。
	const ch = ensureYieldChannel();
	if (!ch) return new Promise<void>(resolve => setTimeout(resolve, 0));
	return new Promise<void>(resolve => {
		__yieldResolvers.push(resolve);
		ch.port2.postMessage(0);
	});
}

/** 单次求解的时间预算（毫秒）。超出则放弃该走廊级别，避免长时间独占主线程 */
const SOLVE_BUDGET_MS = 3000;
/** 协作让出的最小间隔（毫秒） */
const YIELD_INTERVAL_MS = 16;

/** 单端点寻路（兼容旧接口）：src→dst 最短路径 */
export async function routeConnection(
	src: { x: number; y: number },
	dst: { x: number; y: number },
	obstacles: RBObstacle[],
	net = '',
	expands: number[] = [150, 500],
	budgetMs: number = SOLVE_BUDGET_MS,
	isCancelled?: () => boolean,
	directOnly = false,
): Promise<RouteConnectionResult> {
	const res = await routeBetweenEndpoints(
		[{ x: src.x, y: src.y, cost: 0 }],
		[{ x: dst.x, y: dst.y, cost: 0 }],
		obstacles, net, expands, budgetMs,
		undefined, undefined, isCancelled, directOnly,
	);
	return { ok: res.ok, prims: res.prims };
}

/**
 * 粗网格走廊探测（管道引导用，coarse-to-fine 的粗级）。
 * 16mil 格、八连通 A* + 层间过孔边，只光栅化 src/dst 包围盒附近障碍。
 * 返回走廊折线点列；失败返回 null（调用方退回直连走廊剪枝）。
 * 这是「引导」不是「求解」：不做净距终检/过孔复核 —— 管道级求解结果
 * 仍要过 validatePrimsAgainst 全量复验。粗路径只需大体可行。
 */
function coarseTube(
	all: RBObstacle[], net: string,
	src: { x: number; y: number }, dst: { x: number; y: number },
): Array<{ x: number; y: number }> | null {
	if (all.length < 8) return null;
	const CELL = Number((globalThis as any).__RB_TUBE_CELL__) || 16;
	const span = Math.hypot(dst.x - src.x, dst.y - src.y);
	if (span < 1) return null;
	const margin = Math.min(span * 0.4 + 150, 600);
	const x0 = Math.floor(Math.min(src.x, dst.x) - margin);
	const x1 = Math.ceil(Math.max(src.x, dst.x) + margin);
	const y0 = Math.floor(Math.min(src.y, dst.y) - margin);
	const y1 = Math.ceil(Math.max(src.y, dst.y) + margin);
	const nx = Math.ceil((x1 - x0) / CELL) + 1;
	const ny = Math.ceil((y1 - y0) / CELL) + 1;
	if (nx * ny > 1_500_000) return null;
	const hasL2 = all.some(o => o.layer === 2);
	const nl = hasL2 ? 2 : 1;
	const stride = nx * ny;
	const total = stride * nl;
	const blocked = new Uint8Array(total);
	const INFL = CELL * 0.5;
	for (const o of all) {
		if (o.soft) continue;
		if (o.net && o.net === net) continue;
		const lis = o.layer === undefined ? [0, 1] : (o.layer === 1 ? [0] : o.layer === 2 ? [1] : []);
		if (!lis.length) continue;
		const rr = o.r + INFL;
		const i0 = Math.max(0, Math.floor((o.x - rr - x0) / CELL));
		const i1 = Math.min(nx - 1, Math.ceil((o.x + rr - x0) / CELL));
		const j0 = Math.max(0, Math.floor((o.y - rr - y0) / CELL));
		const j1 = Math.min(ny - 1, Math.ceil((o.y + rr - y0) / CELL));
		const rr2 = rr * rr;
		for (const li of lis) {
			const base = li * stride;
			for (let i = i0; i <= i1; i++) {
				const dx = x0 + i * CELL - o.x, dx2 = dx * dx;
				if (dx2 > rr2) continue;
				for (let j = j0; j <= j1; j++) {
					const dy = y0 + j * CELL - o.y;
					if (dx2 + dy * dy <= rr2) blocked[base + j * nx + i] = 1;
				}
			}
		}
	}
	// 端点松绑：端点候选被邻盘净距圈覆盖时仍可从焊盘内部出发
	//（覆盖半径取「压住端点的最大障碍圆的伸出量」，封顶 120）
	const release = (px: number, py: number): void => {
		let rr = 24;
		for (const o of all) {
			if (o.soft || (o.net && o.net === net)) continue;
			const d = Math.hypot(o.x - px, o.y - py);
			if (d < o.r + 2) rr = Math.max(rr, Math.min(120, o.r - d + CELL + 4));
		}
		const i0 = Math.max(0, Math.floor((px - rr - x0) / CELL));
		const i1 = Math.min(nx - 1, Math.ceil((px + rr - x0) / CELL));
		const j0 = Math.max(0, Math.floor((py - rr - y0) / CELL));
		const j1 = Math.min(ny - 1, Math.ceil((py + rr - y0) / CELL));
		const rr2 = rr * rr;
		for (let li = 0; li < nl; li++) {
			const base = li * stride;
			for (let i = i0; i <= i1; i++) {
				const dx = x0 + i * CELL - px, dx2 = dx * dx;
				for (let j = j0; j <= j1; j++) {
					const dy = y0 + j * CELL - py;
					if (dx2 + dy * dy <= rr2) blocked[base + j * nx + i] = 0;
				}
			}
		}
	};
	release(src.x, src.y);
	release(dst.x, dst.y);

	const toCell = (p: { x: number; y: number }): { i: number; j: number } => ({
		i: Math.max(0, Math.min(nx - 1, Math.round((p.x - x0) / CELL))),
		j: Math.max(0, Math.min(ny - 1, Math.round((p.y - y0) / CELL))),
	});
	const s = toCell(src), d = toCell(dst);
	const D8: Array<[number, number, number]> = [
		[1, 0, 1], [-1, 0, 1], [0, 1, 1], [0, -1, 1],
		[1, 1, Math.SQRT2], [1, -1, Math.SQRT2], [-1, 1, Math.SQRT2], [-1, -1, Math.SQRT2],
	];
	const idxOf = (i: number, j: number, li: number) => li * stride + j * nx + i;
	const gScore = new Float64Array(total).fill(Infinity);
	const cameFrom = new Int32Array(total).fill(-1);
	const closed = new Uint8Array(total);
	// 小根堆（内联，避免依赖）
	const heapK: number[] = [], heapV: number[] = [];
	const push = (k: number, v: number) => {
		heapK.push(k); heapV.push(v);
		let i = heapK.length - 1;
		while (i > 0) {
			const p = (i - 1) >> 1;
			if (heapK[p] <= heapK[i]) break;
			[heapK[p], heapK[i]] = [heapK[i], heapK[p]];
			[heapV[p], heapV[i]] = [heapV[i], heapV[p]];
			i = p;
		}
	};
	const pop = (): number => {
		const top = heapV[0];
		const lk = heapK.pop()!, lv = heapV.pop()!;
		if (heapK.length) {
			heapK[0] = lk; heapV[0] = lv;
			let i = 0;
			for (;;) {
				const l = 2 * i + 1, r = l + 1;
				let m = i;
				if (l < heapK.length && heapK[l] < heapK[m]) m = l;
				if (r < heapK.length && heapK[r] < heapK[m]) m = r;
				if (m === i) break;
				[heapK[m], heapK[i]] = [heapK[i], heapK[m]];
				[heapV[m], heapV[i]] = [heapV[i], heapV[m]];
				i = m;
			}
		}
		return top;
	};
	const viaCost = 200;
	for (let li = 0; li < nl; li++) {
		const id = idxOf(s.i, s.j, li);
		if (blocked[id]) continue;
		gScore[id] = 0;
		push(Math.hypot(s.i - d.i, s.j - d.j) * CELL, id);
	}
	let best = -1;
	let pops = 0;
	while (heapK.length) {
		if ((++pops & 1023) === 0 && pops > 400_000) break;   // 探测预算：只找「大体可行」
		const cur = pop();
		if (closed[cur]) continue;
		closed[cur] = 1;
		const li = Math.floor(cur / stride);
		const rem = cur - li * stride;
		const j = Math.floor(rem / nx);
		const i = rem - j * nx;
		if (i === d.i && j === d.j) { best = cur; break; }
		const g = gScore[cur];
		for (const [dx, dy, w] of D8) {
			const ni = i + dx, nj = j + dy;
			if (ni < 0 || nj < 0 || ni >= nx || nj >= ny) continue;
			const nid = idxOf(ni, nj, li);
			if (blocked[nid] || closed[nid]) continue;
			const ng = g + w * CELL;
			if (ng < gScore[nid] - 1e-9) {
				gScore[nid] = ng;
				cameFrom[nid] = cur;
				push(ng + Math.hypot(ni - d.i, nj - d.j) * CELL, nid);
			}
		}
		if (nl > 1) {
			const nid = idxOf(i, j, 1 - li);
			if (!blocked[nid] && !closed[nid] && g + viaCost < gScore[nid] - 1e-9) {
				gScore[nid] = g + viaCost;
				cameFrom[nid] = cur;
				push(gScore[nid] + Math.hypot(i - d.i, j - d.j) * CELL, nid);
			}
		}
	}
	if (best < 0) return null;
	const pts: Array<{ x: number; y: number }> = [];
	for (let cur = best; cur !== -1; cur = cameFrom[cur]) {
		const li = Math.floor(cur / stride);
		const rem = cur - li * stride;
		const j = Math.floor(rem / nx);
		const i = rem - j * nx;
		pts.push({ x: x0 + i * CELL, y: y0 + j * CELL });
	}
	pts.reverse();
	// 共线取直（压缩折线长度，管道距离检查更快）
	const simp: Array<{ x: number; y: number }> = [pts[0]];
	for (let k = 1; k < pts.length - 1; k++) {
		const a = simp[simp.length - 1], b = pts[k], c = pts[k + 1];
		const cross = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
		const dot = (b.x - a.x) * (c.x - b.x) + (b.y - a.y) * (c.y - b.y);
		if (Math.abs(cross) < 1e-6 && dot > 0) continue;
		simp.push(b);
	}
	simp.push(pts[pts.length - 1]);
	return simp;
}

/**
 * 多端点寻路：源/目标各一组候选点，求「引出段 + 路径 + 引入段」总长最小的走线。
 *
 * refA/refB：走廊剪枝用的参考线段（通常是两焊盘中心）；缺省用首个候选点。
 * isCancelled：外部取消探针 —— 在每个协作让出点轮询（≤16ms 粒度），
 * 取消后立即放弃求解。没有它，「停止布线」要等当前边整条兜底链
 * （直连×2 + 边界候选 + 底层 + 过孔候选，预算 3s×N）烧完才能停。
 */
export async function routeBetweenEndpoints(
	srcCands: RouteEndpoint[],
	dstCands: RouteEndpoint[],
	obstacles: RBObstacle[],
	net = '',
	expands: number[] = [150, 500],
	budgetMs: number = SOLVE_BUDGET_MS,
	refA?: { x: number; y: number },
	refB?: { x: number; y: number },
	isCancelled?: () => boolean,
	directOnly = false,
): Promise<CandidateRouteResult> {
	if (!srcCands.length || !dstCands.length) return { ok: false, prims: [] };
	// 圆障碍列表（胶囊兜底转为圆群）
	const all: RBObstacle[] = [];
	for (const o of obstacles) {
		if (o.kind === 'circle') {
			all.push(o);
		} else {
			for (const c of capsuleToCircles(o.x1!, o.y1!, o.x2!, o.y2!, o.r)) {
				all.push({ ...o, kind: 'circle', x: c.x, y: c.y, r: c.r });
			}
		}
	}
	// 规范化排序：与障碍收集顺序无关的确定性 tie-break
	// （EDA 与离线的收集顺序不同，等长路径的选择会随顺序漂移）
	all.sort((a, b) => a.x - b.x || a.y - b.y || a.r - b.r || (a.net || '').localeCompare(b.net || ''));

	// ── 直连快判（directOnly 模式）──
	// 快扫专用：只判「直线能不能过」，不建任何图。非平凡边全部留给
	// 完整链求解 —— 否则快扫建一次走廊图、收尾轮再建一次，同一成本付两遍
	//（实测占大板 40% 预算）。
	if (directOnly) {
		if (srcCands.length === 1 && dstCands.length === 1) {
			const grid = new CircleGrid(all);
			const segOK = makeSegmentChecker(all, grid, net);
			const a = srcCands[0], b = dstCands[0];
			if (segOK(a.x, a.y, b.x, b.y, -1, -1)) {
				return { ok: true, prims: [{ kind: 'line', x1: a.x, y1: a.y, x2: b.x, y2: b.y }] };
			}
		}
		return { ok: false, prims: [] };
	}

	const ax = refA ?? { x: srcCands[0].x, y: srcCands[0].y };
	const bx = refB ?? { x: dstCands[0].x, y: dstCands[0].y };

	// ── 管道引导（coarse-to-fine）──
	// 先用粗网格 A*（16mil 格）找一条可行走廊折线（毫秒级），只保留折线
	// 管道近旁的障碍建切点图 —— 图规模从走廊全量 ~150 圆降到 ~40 圆，
	// O(n²) 建图成本近似平方下降（实测单次求解 0.5s → 0.05~0.1s）。
	// 粗路径失败 = 走廊内数学无路 → 跳过管道级直接走直连走廊退避；
	// 管道级求解失败/复验失败 → 照旧退避 150/500，正确性不受影响。
	const tubePathsBefore = vrStats.tubePaths;
	if (!(globalThis as any).__RB_NO_TUBE__) {
		vrStats.tubeTries++;
		const tube = coarseTube(all, net, ax, bx);
		if (tube) {
			vrStats.tubePaths++;
			const kept: RBObstacle[] = [];
			const picked: RBObstacle[] = [];
			for (const c of all) {
				if (c.noPrune) { kept.push(c); continue; }
				let best = Infinity;
				for (let k = 1; k < tube.length; k++) {
					const d = pointSegDist(c.x, c.y, tube[k - 1].x, tube[k - 1].y, tube[k].x, tube[k].y);
					if (d < best) best = d;
				}
				if (best < c.r + 80) picked.push(c);
			}
			const subset = kept.concat(picked);
			if ((globalThis as any).__RB_DEBUG__) {
				console.log(`[VR][dbg] 管道级: 折线${tube.length}点 子集=${subset.length}`);
			}
			const deadline = Date.now() + budgetMs;
			(globalThis as any).__RB_PHASE__ = 'vr:solve';
			const res = await solveMulti(srcCands, dstCands, subset, net, deadline, isCancelled);
			(globalThis as any).__RB_PHASE__ = 'vr:validate';
			if (res.ok && await validatePrimsAgainst(res.prims, all, net)) {
				vrStats.tubeSolveOk++;
				return res;
			}
		}
	}

	// 走廊剪枝：先只保留直连线附近窄带内的圆（大幅提速），
	// 求解失败再逐级放宽范围重试。
	// noPrune（填充/板框等实心障碍）始终保留——否则窄走廊会把内部网格圆剪掉，
	// 导致"穿过填充"的假合法路径。
	// 管道级已覆盖 150 级的子集：管道存在但失败时跳过 150，直接退避放宽级。
	const tubeExisted = vrStats.tubePaths > tubePathsBefore;
	const fallbackExpands = tubeExisted ? expands.filter(e => e !== 150) : expands;
	for (let ei = 0; ei < fallbackExpands.length; ei++) {
		const expand = fallbackExpands[ei];
		let subset = all;
		if (expand !== Infinity) {
			const kept: RBObstacle[] = [];
			const candidates: Array<{ o: RBObstacle; d: number }> = [];
			for (const c of all) {
				if (c.noPrune) { kept.push(c); continue; }
				const dSeg = pointSegDist(c.x, c.y, ax.x, ax.y, bx.x, bx.y);
				if (dSeg < c.r + expand || dist(c.x, c.y, ax.x, ax.y) < c.r + expand || dist(c.x, c.y, bx.x, bx.y) < c.r + expand) {
					candidates.push({ o: c, d: dSeg });
				}
			}
			// 规模上限：只保留离直连线最近的障碍，防止走廊放宽后图爆炸
			const MAX_SUBSET = 1400;
			if (candidates.length > MAX_SUBSET) {
				candidates.sort((a, b) => a.d - b.d);
				candidates.length = MAX_SUBSET;
			}
			subset = kept.concat(candidates.map(c => c.o));
		}
		// 每一级之间主动让出，保证 EDA 主线程始终可响应
		if (ei > 0) await yieldToHost();

		if ((globalThis as any).__RB_DEBUG__) {
			const wr = subset.filter(o => !o.blockerOnly).length;
			const np = subset.filter(o => o.noPrune).length;
			console.log(`[VR][dbg] 走廊级别${ei} expand=${expand}: 子集=${subset.length} 可绕=${wr} noPrune=${np}`);
		}

		const deadline = Date.now() + budgetMs;
		(globalThis as any).__RB_PHASE__ = 'vr:solve';
		const res = await solveMulti(srcCands, dstCands, subset, net, deadline, isCancelled);
		(globalThis as any).__RB_PHASE__ = 'vr:validate';
		if (res.ok && await validatePrimsAgainst(res.prims, all, net)) return res;
	}
	return { ok: false, prims: [] };
}

/** 找到表面经过点 p 的圆下标（|dist - r| ≤ eps），供链容差判定用；找不到返回 -1 */
function findSurfaceCircle(all: RBObstacle[], px: number, py: number, eps = 0.05): number {
	for (let i = 0; i < all.length; i++) {
		const c = all[i];
		if (Math.abs(Math.hypot(px - c.x, py - c.y) - c.r) <= eps) return i;
	}
	return -1;
}

/** 用全量障碍复验路径图元（防止走廊剪枝造成的假合法路径） */
export async function validatePrimsAgainst(prims: RBPrimitive[], all: RBObstacle[], net = ''): Promise<boolean> {
	const grid = new CircleGrid(all);
	const segOK = makeSegmentChecker(all, grid, net);
	const arcOK = makeArcChecker(all, grid, net);
	let vLastYield = Date.now();
	for (let i = 0; i < prims.length; i++) {
		if ((i & 15) === 0) {
			const now = Date.now();
			if (now - vLastYield >= YIELD_INTERVAL_MS) { vLastYield = now; await yieldToHost(); }
		}
		const pr = prims[i];
		if (pr.kind === 'line') {
			// 端点所在的圆视为「被切圆」：切线合法地与它们相切，
			// 且链容差（同一铜链相邻圆的矢高切入）需要据此放行
			const tA = findSurfaceCircle(all, pr.x1, pr.y1);
			const tB = findSurfaceCircle(all, pr.x2, pr.y2);
			if (!segOK(pr.x1, pr.y1, pr.x2, pr.y2, tA, tB)) {
				if ((globalThis as any).__RB_VERBOSE__) {
					let worst: RBObstacle | null = null;
					let wd = Infinity;
					for (const o of all) {
						const d = pointSegDist(o.x, o.y, pr.x1, pr.y1, pr.x2, pr.y2) - o.r;
						if (d < wd) { wd = d; worst = o; }
					}
					console.log(`[VR] 线段被拒 #${i}: (${pr.x1.toFixed(0)},${pr.y1.toFixed(0)})→(${pr.x2.toFixed(0)},${pr.y2.toFixed(0)}) 侵入 ${worst?.group} @(${worst?.x.toFixed(0)},${worst?.y.toFixed(0)}) r=${worst?.r.toFixed(1)} 越界 ${(-wd).toFixed(2)}`);
				}
				return false;
			}
		} else {
			const startPt = { x: pr.cx! + pr.r! * Math.cos(pr.startAngle!), y: pr.cy! + pr.r! * Math.sin(pr.startAngle!) };
			const selfIdx = findSurfaceCircle(all, startPt.x, startPt.y);
			if (!arcOK(pr.cx, pr.cy, pr.r, pr.startAngle, pr.sweep, selfIdx)) {
				if ((globalThis as any).__RB_VERBOSE__) console.log(`[VR] 弧被拒 #${i}: c=(${pr.cx.toFixed(0)},${pr.cy.toFixed(0)}) r=${pr.r.toFixed(1)}`);
				return false;
			}
		}
	}
	return true;
}

/**
 * 多端点求解：全部源候选为起点，全部目标候选为终点，求总代价最小者。
 * 节点布局：0..S-1 = 源候选，S..S+D-1 = 目标候选，其后是圆上的切点。
 */
async function solveMulti(
	srcCands: RouteEndpoint[],
	dstCands: RouteEndpoint[],
	obstacles: RBObstacle[],
	net = '',
	deadline: number = Date.now() + SOLVE_BUDGET_MS,
	isCancelled?: () => boolean,
): Promise<CandidateRouteResult> {
	// 取消探针：每个协作让出点轮询一次（异常安全）
	const cancelled = (): boolean => {
		try { return isCancelled?.() === true; } catch { return false; }
	};
	// 分相计时（__RB_DEBUG__ 时输出）：定位单次求解的耗时大头
	const __t0 = Date.now();
	let __mark = __t0, __tPair = 0, __tTerm = 0, __tArc = 0, __tDij = 0;
	const __dbg = !!(globalThis as any).__RB_DEBUG__;
	// 拆分：可绕行圆（建图）与纯阻挡圆（填充内部网格 / 焊盘自身轮廓，只参与碰撞检查）
	const circles: RBObstacle[] = [];
	const blockers: RBObstacle[] = [];
	for (const o of obstacles) {
		if (o.blockerOnly) blockers.push(o);
		else circles.push(o);
	}
	const everything = [...circles, ...blockers];
	const n = circles.length;

	const grid = new CircleGrid(everything);
	const segOK = makeSegmentChecker(everything, grid, net);
	const arcOK = makeArcChecker(everything, grid, net);

	const nodes: VGNode[] = [];
	// 0 .. S-1 源候选；S .. S+D-1 目标候选
	for (const c of srcCands) nodes.push({ circle: -1, x: c.x, y: c.y, angle: 0, adj: [] });
	for (const c of dstCands) nodes.push({ circle: -1, x: c.x, y: c.y, angle: 0, adj: [] });
	const S = srcCands.length, D = dstCands.length;

	// ── 直连快判（在建图之前）──
	// 单候选↔单候选且直线段合法时，直连就是全局最优（端点代价固定，
	// 任何折线路径都 ≥ 直线距离），直接构造单段路径返回，省掉 O(n²)
	// 公切线建图。快扫的大多数边属于「一布就通」，实测建图占求解 85%
	// —— 这一步把它们的单次成本从 ~0.7s 降到微秒级。
	if (S === 1 && D === 1) {
		const a = nodes[0], b = nodes[1];
		if (segOK(a.x, a.y, b.x, b.y, -1, -1)) {
			return {
				ok: true,
				prims: [{ kind: 'line', x1: a.x, y1: a.y, x2: b.x, y2: b.y }],
			};
		}
	}

	// 切点去重：键 = 圆下标 + 角度量化
	const ptIndex = new Map<string, number>();
	// 按圆分桶（避免弧边循环里对全部切点做字符串扫描：那是 O(n×切点数)）
	const ptsByCircle: number[][] = Array.from({ length: n }, () => []);
	const getOrCreatePt = (ci: number, x: number, y: number): number => {
		const angle = Math.atan2(y - circles[ci].y, x - circles[ci].x);
		const key = `${ci}:${Math.round(angle * 1000)}`;
		const existing = ptIndex.get(key);
		if (existing !== undefined) return existing;
		const idx = nodes.length;
		nodes.push({
			circle: ci,
			x: circles[ci].x + circles[ci].r * Math.cos(angle),
			y: circles[ci].y + circles[ci].r * Math.sin(angle),
			angle, adj: [],
		});
		ptIndex.set(key, idx);
		ptsByCircle[ci].push(idx);
		return idx;
	};

	// 2. 终端（出线候选）↔ 圆切线 —— 提前到建图之前：本步便宜
	//（O(n·(S+D))），且「终端零切线 ∧ 无直连」意味着数学上无路，
	// 可以跳过整个 O(n²) 公切线建图（失败求解占快扫的大头）
	const connectTerminal = (termIdx: number) => {
		const term = nodes[termIdx];
		for (let i = 0; i < n; i++) {
			const ci = circles[i];
			if (net && ci.net === net && !ci.forceBlock) continue;
			if (dist(term.x, term.y, ci.x, ci.y) < ci.r + EPS_TOUCH) continue; // 终端在圆内/圆上：切线退化
			for (const l of [true, false] as const) {
				const [tx1, ty1, tx2, ty2] = getTangents(term.x, term.y, 0, false, ci.x, ci.y, ci.r, l);
				if (!segOK(tx1, ty1, tx2, ty2, -1, i)) continue;
				const len = Math.hypot(tx2 - tx1, ty2 - ty1);
				if (len < 1e-6) continue;
				const pi = getOrCreatePt(i, tx2, ty2);
				nodes[termIdx].adj.push({ to: pi, cost: len, isArc: false, x1: tx1, y1: ty1, x2: tx2, y2: ty2 });
				nodes[pi].adj.push({ to: termIdx, cost: len, isArc: false, x1: tx2, y1: ty2, x2: tx1, y2: ty1 });
			}
		}
	};
	for (let k = 0; k < S + D; k++) connectTerminal(k);

	// 3. 源候选 ↔ 目标候选 直连（无障碍阻挡时）—— 最短路径的常见情形
	for (let i = 0; i < S; i++) {
		const a = nodes[i];
		for (let j = 0; j < D; j++) {
			const b = nodes[S + j];
			if (!segOK(a.x, a.y, b.x, b.y, -1, -1)) continue;
			const len = Math.hypot(b.x - a.x, b.y - a.y);
			if (len < 1e-6) continue;
			a.adj.push({ to: S + j, cost: len, isArc: false, x1: a.x, y1: a.y, x2: b.x, y2: b.y });
			nodes[S + j].adj.push({ to: i, cost: len, isArc: false, x1: b.x, y1: b.y, x2: a.x, y2: a.y });
		}
	}

	// 封死快败：任一侧终端完全无出边（切线 0 + 直连无）⇒ 图内无路
	if (!nodes.slice(0, S).some(t => t.adj.length > 0) ||
		!nodes.slice(S, S + D).some(t => t.adj.length > 0)) {
		vrStats.calls++;
		vrStats.failCalls++;
		vrStats.solveMs += Date.now() - __t0;
		vrStats.termMs += Date.now() - __t0;
		vrStats.maxN = Math.max(vrStats.maxN, n);
		vrStats.sumN += n;
		return { ok: false, prims: [] };
	}

	__tTerm = Date.now() - __mark; __mark = Date.now();
	// 1. 圆-圆公切线（4 种侧别组合）
	//    配对数是 O(n²)，实测占单次求解 70~90%（Dijkstra≈0）：x 序扫描 +
	//    距离截断 —— 相距超过 r1+r2+K 的两圆之间不存在有价值的公切线
	//    路由边（长直切线要么被中间障碍判死，要么退化成「横穿整片空地」
	//    的大斜线，正是布线效果变差的来源），直接跳过
	let lastYield = Date.now();
	let timedOut = false;
	const __xOrder = circles.map((_, idx) => idx).sort((a, b) => circles[a].x - circles[b].x);
	let __maxR = 0;
	for (const c of circles) if (c.r > __maxR) __maxR = c.r;
	let __pairsVisited = 0;
	const __pairsAll = (n * (n - 1)) / 2;
	for (let oi = 0; oi < n && !timedOut; oi++) {
		const i = __xOrder[oi];
		const ci = circles[i];
		if (net && ci.net === net && !ci.forceBlock) continue;
		{
			const now = Date.now();
			if (now > deadline) { timedOut = true; break; }
			if (now - lastYield >= YIELD_INTERVAL_MS) {
				lastYield = now;
				await yieldToHost();
			}
			if (cancelled()) { timedOut = true; break; }
		}
		const reach = ci.r + __maxR + (Number((globalThis as any).__RB_VR_REACH__) || 800);
		for (let oj = oi + 1; oj < n; oj++) {
			const j = __xOrder[oj];
			if (circles[j].x - ci.x > reach) break;
			if ((++__pairsVisited & 31) === 0) {
				const now = Date.now();
				if (now > deadline) { timedOut = true; break; }
				if (now - lastYield >= YIELD_INTERVAL_MS) {
					lastYield = now;
					await yieldToHost();
				}
				if (cancelled()) { timedOut = true; break; }
			}
			const cj = circles[j];
			// y 预筛：x 序扫描里 |Δy| 超界的圆对占大半，切线 4 组组合 + segOK
			// 网格遍历全浪费。只 continue 不改变扫描顺序 → 输出逐位一致（无
			// tie-break 混沌回归），实测建图占求解 85~90%，这刀纯赚。
			const dy = cj.y - ci.y;
			if (dy > reach || dy < -reach) continue;
			if (net && cj.net === net && !cj.forceBlock) continue;
			const d = dist(ci.x, ci.y, cj.x, cj.y);
			// 切线解算内联（与 getTangents 逐位一致）：单位向量 vx/vy 对每
			// 圆对只算一次（原来 4 组组合各做一次 hypot+除法）；d≈0 时
			// cVal=±Infinity 被 |cVal|>1 提前拦下，与 getTangents 的 d<1e-9
			// 分支在此处不可达，行为等价。
			const vx = (cj.x - ci.x) / d;
			const vy = dy / d;
			// 四种内外切线组合（外切 l1===l2 / 内切相异）手工展开为位循环，
			// 避免每对分配 [[..],[..]] 数组（建图热路径 GC 大头之一）
			for (let lv = 0; lv < 4; lv++) {
				const l1 = lv < 2;
				const l2 = (lv & 1) === 0;
				// 内公切线（l1!==l2）在绕障布线里极少成为有效边，却同样要过
				// 4 组 segOK；实验开关用于量化它的成本/收益。
				// 内公切线（l1!==l2，切线穿过两障碍之间）在绕障布线里几乎不成为
				// 最终边。实测默认砍掉后，整板布通数与几何逐位一致
				//（126589mil / 429 线段 / 238 弧 / 83 过孔），建图 24.0s→19.8s、
				// 整板 48.2s→42.2s。需要旧口径做 A/B 时设 __RB_VR_INNER__=1。
				if (!(globalThis as any).__RB_VR_INNER__ && l1 !== l2) continue;
				const adjR2 = cj.r * (l1 === l2 ? 1 : -1);
				const cVal = (ci.r - adjR2) / d;
				if (Math.abs(cVal) > 1) continue; // 该类型切线不存在
				let hs = Math.sqrt(1 - cVal * cVal);
				if (l1) hs = -hs; // getTangents: h = sqrt(1-c²)·(l1 ? -1 : 1)
				const nx = vx * cVal - hs * vy;
				const ny = vy * cVal + hs * vx;
				const tx1 = ci.x + ci.r * nx;
				const ty1 = ci.y + ci.r * ny;
				const tx2 = cj.x + adjR2 * nx;
				const ty2 = cj.y + adjR2 * ny;
				if (!segOK(tx1, ty1, tx2, ty2, i, j)) continue;
				const len = Math.hypot(tx2 - tx1, ty2 - ty1);
				if (len < 1e-6) continue;
				const pi = getOrCreatePt(i, tx1, ty1);
				const pj = getOrCreatePt(j, tx2, ty2);
				nodes[pi].adj.push({ to: pj, cost: len, isArc: false, x1: tx1, y1: ty1, x2: tx2, y2: ty2 });
				nodes[pj].adj.push({ to: pi, cost: len, isArc: false, x1: tx2, y1: ty2, x2: tx1, y2: ty1 });
			}
		}
	}
	if (timedOut) {
		// 建图对循环超时：时间几乎全烧在圆对扫描上，计入统计后放弃
		vrStats.calls++;
		vrStats.failCalls++;
		vrStats.solveMs += Date.now() - __t0;
		vrStats.pairMs += Date.now() - __t0;
		vrStats.maxN = Math.max(vrStats.maxN, n);
		vrStats.sumN += n;
		vrStats.pairsVisited += __pairsVisited;
		return { ok: false, prims: [] };
	}

	__tPair = Date.now() - __mark; __mark = Date.now();
	// 4. 同圆弧边：切点按角度排序，相邻切点之间双向连弧（弧几何合法即建边）
	for (let i = 0; i < n; i++) {
		{
			const now = Date.now();
			if (now > deadline) return { ok: false, prims: [] };
			if (now - lastYield >= YIELD_INTERVAL_MS) {
				lastYield = now;
				await yieldToHost();
			}
			if (cancelled()) return { ok: false, prims: [] };
		}
		const pts = ptsByCircle[i];
		if (pts.length < 2) continue;
		pts.sort((a, b) => nodes[a].angle - nodes[b].angle);
		const m = pts.length;
		for (let k = 0; k < m; k++) {
			const a = pts[k], b = pts[(k + 1) % m];
			if (a === b) continue;
			let sweepCCW = nodes[b].angle - nodes[a].angle;
			while (sweepCCW <= 0) sweepCCW += 2 * Math.PI;
			if (sweepCCW < 1e-9) continue;
			// 逆时针 a→b
			if (arcOK(circles[i].x, circles[i].y, circles[i].r, nodes[a].angle, sweepCCW, i)) {
				nodes[a].adj.push({
					to: b, cost: sweepCCW * circles[i].r, isArc: true,
					cx: circles[i].x, cy: circles[i].y, r: circles[i].r,
					startAngle: nodes[a].angle, sweep: sweepCCW,
				});
			}
			// 顺时针 b→a
			if (arcOK(circles[i].x, circles[i].y, circles[i].r, nodes[b].angle, -sweepCCW, i)) {
				nodes[b].adj.push({
					to: a, cost: sweepCCW * circles[i].r, isArc: true,
					cx: circles[i].x, cy: circles[i].y, r: circles[i].r,
					startAngle: nodes[b].angle, sweep: -sweepCCW,
				});
			}
		}
	}

	// 图规模上限：节点过多时该走廊级别已失去意义（Dijkstra/重建都会过重），
	// 直接放弃，让调用方尝试更窄走廊或降级
	const MAX_GRAPH_NODES = 250000;
	if (nodes.length > MAX_GRAPH_NODES) return { ok: false, prims: [] };

	// 5. Dijkstra（二叉堆，O(E log V)）：多源（全部源候选），多目标。
	//    状态 = (节点, 到达旋转侧)：切点图里「顺时针弧到达切点 P 后沿反向
	//    切线离开」是合法图路径但几何上是 180° 尖点（导线折返叠线，实测
	//    TXD/RESET 网）。到达旋转侧约束下一跳方向：CCW 到达只许沿 CCW
	//    切向继续（切线同向或 CCW 弧）。状态编码 = 节点*3 + r
	//   （0=CCW 到达，1=CW 到达，2=无约束 —— 终端/起点）。
	const total3 = nodes.length * 3;
	const distArr = new Array<number>(total3).fill(Infinity);
	const prevNode = new Array<number>(total3).fill(-1);
	const prevEdge = new Array<VGEdge | null>(total3).fill(null);
	const settled = new Array<boolean>(total3).fill(false);
	const heap: Array<[number, number]> = [];
	const heapPush = (d: number, v: number) => {
		heap.push([d, v]);
		let i = heap.length - 1;
		while (i > 0) {
			const p = (i - 1) >> 1;
			if (heap[p][0] <= heap[i][0]) break;
			[heap[p], heap[i]] = [heap[i], heap[p]];
			i = p;
		}
	};
	const heapPop = (): [number, number] | null => {
		if (heap.length === 0) return null;
		const top = heap[0];
		const last = heap.pop()!;
		if (heap.length > 0) {
			heap[0] = last;
			let i = 0;
			for (;;) {
				const l = 2 * i + 1, r = 2 * i + 2;
				let s = i;
				if (l < heap.length && heap[l][0] < heap[s][0]) s = l;
				if (r < heap.length && heap[r][0] < heap[s][0]) s = r;
				if (s === i) break;
				[heap[s], heap[i]] = [heap[i], heap[s]];
				i = s;
			}
		}
		return top;
	};
	for (let i = 0; i < S; i++) {
		distArr[i * 3 + 2] = srcCands[i].cost;
		heapPush(srcCands[i].cost, i * 3 + 2);
	}
	let dijkstraPops = 0;
	let bestDst = -1;
	let bestTotal = Infinity;
	for (;;) {
		// Dijkstra 也是 O(E log V) 的重计算，必须同样让出主线程并受预算约束
		if ((++dijkstraPops & 511) === 0) {
			const now = Date.now();
			if (now > deadline) return { ok: false, prims: [] };
			if (now - lastYield >= YIELD_INTERVAL_MS) {
				lastYield = now;
				await yieldToHost();
			}
			if (cancelled()) return { ok: false, prims: [] };
		}
		const top = heapPop();
		if (!top) break;
		const [du, s] = top;
		if (settled[s]) continue;
		settled[s] = true;
		const u = Math.floor(s / 3);
		const arrR = s % 3;   // 0=CCW 1=CW 2=无约束
		// 剩余路径代价不可能小于当前弹出距离 → 无更优解
		if (du >= bestTotal) break;
		if (u >= S && u < S + D) {
			const cand = du + dstCands[u - S].cost;
			if (cand < bestTotal) { bestTotal = cand; bestDst = s; }
		}
		// 到达旋转侧的行进切向（圆节点才有约束）
		const nu = nodes[u];
		let tx = 0, ty = 0;
		if (arrR !== 2 && nu.circle >= 0) {
			const au = nu.angle;
			if (arrR === 0) { tx = -Math.sin(au); ty = Math.cos(au); }
			else { tx = Math.sin(au); ty = -Math.cos(au); }
		}
		for (const e of nodes[u].adj) {
			if (settled[e.to * 3] && settled[e.to * 3 + 1] && settled[e.to * 3 + 2]) continue;
			// 旋转连续性：本边的出发方向必须与到达切向同向（点积 > 0.5）
			// __RB_NO_ROT__：A/B 诊断开关（关掉=旧行为）
			if (arrR !== 2 && nu.circle >= 0 && !(globalThis as any).__RB_NO_ROT__) {
				let dx: number, dy: number;
				if (e.isArc) {
					const t = e.sweep! >= 0 ? { x: -Math.sin(e.startAngle!), y: Math.cos(e.startAngle!) } : { x: Math.sin(e.startAngle!), y: -Math.cos(e.startAngle!) };
					dx = t.x; dy = t.y;
				} else {
					dx = e.x2! - e.x1!; dy = e.y2! - e.y1!;
				}
				const dl = Math.hypot(dx, dy) || 1;
				if ((dx * tx + dy * ty) / dl < 0.5) continue;   // 尖点：禁止折返
			}
			// 新状态的到达旋转侧：弧 = 自身扫向；线 = 到达方向与目标切点切向的匹配
			let nr = 2;
			const nw = nodes[e.to];
			if (nw.circle >= 0) {
				if (e.isArc) nr = e.sweep! >= 0 ? 0 : 1;
				else {
					const aw = nw.angle;
					const ccwx = -Math.sin(aw), ccwy = Math.cos(aw);
					const dx = e.x2! - e.x1!, dy = e.y2! - e.y1!;
					const dl = Math.hypot(dx, dy) || 1;
					const dCCW = (dx * ccwx + dy * ccwy) / dl;
					nr = dCCW >= 0 ? 0 : 1;
				}
			}
			const ns = e.to * 3 + nr;
			const nd = du + e.cost;
			if (nd < distArr[ns] - 1e-9) {
				distArr[ns] = nd;
				prevNode[ns] = s;
				prevEdge[ns] = e;
				heapPush(nd, ns);
			}
		}
	}

	__tArc = Date.now() - __mark; __mark = Date.now();
	__tDij = __mark - __t0 - __tPair - __tTerm - __tArc;
	// 常驻累计：测试台读 vrStats 定位热点（不开 debug 日志，零 IO 失真）
	vrStats.calls++;
	vrStats.solveMs += __mark - __t0;
	vrStats.pairMs += __tPair;
	vrStats.termMs += __tTerm;
	vrStats.arcMs += __tArc;
	vrStats.dijMs += __tDij;
	vrStats.maxN = Math.max(vrStats.maxN, n);
	vrStats.sumN += n;
	vrStats.pairsVisited += __pairsVisited;
	if (__dbg) {
		console.warn(`[VR-time] n=${n} S=${S} D=${D} 圆对=${__tPair}ms 终端=${__tTerm}ms 弧边=${__tArc}ms Dijkstra=${__tDij}ms 总=${__mark - __t0}ms 对=${__pairsVisited}/${__pairsAll}`);
	}

	if (bestDst < 0) {
		vrStats.failCalls++;
		const total = nodes.length;
		if ((globalThis as any).__RB_VERBOSE__) {
			console.log(`[VR] 无路径: wrappable=${n} blockers=${blockers.length} nodes=${total} src=${S} dst=${D}`);
		}
		if ((globalThis as any).__RB_DEBUG__) {
			// 诊断 1：src 终端的合法切线数（0 = 图内即被封死）
			for (let i = 0; i < S; i++) console.log(`[VR][dbg] src#${i}(${nodes[i].x.toFixed(0)},${nodes[i].y.toFixed(0)}) 邻接=${nodes[i].adj.length}`);
			// 诊断 2：从 src 出发的 BFS 可达分量范围
			const seen = new Array<boolean>(total).fill(false);
			const q: number[] = [];
			for (let i = 0; i < S; i++) { seen[i] = true; q.push(i); }
			for (let h = 0; h < q.length; h++) for (const e of nodes[q[h]].adj) if (!seen[e.to]) { seen[e.to] = true; q.push(e.to); }
			let mnx = Infinity, mxx = -Infinity, mny = Infinity, mxy = -Infinity, withCircle = 0;
			for (const v of q) {
				if (nodes[v].circle < 0) continue;
				withCircle++;
				mnx = Math.min(mnx, nodes[v].x); mxx = Math.max(mxx, nodes[v].x);
				mny = Math.min(mny, nodes[v].y); mxy = Math.max(mxy, nodes[v].y);
			}
			console.log(`[VR][dbg] 可达=${q.length}/${total} 切点圆=${withCircle}/${n} bbox=(${mnx.toFixed(0)},${mny.toFixed(0)})→(${mxx.toFixed(0)},${mxy.toFixed(0)})`);
			// 诊断 3：直连线上压着的圆（按距离排序 top8，看是谁封的路）
			const onLine = everything
				.map((c, k) => ({ c, d: pointSegDist(c.x, c.y, nodes[0].x, nodes[0].y, nodes[S].x, nodes[S].y) }))
				.filter(e => e.d < e.c.r + 2)
				.sort((a, b) => a.d - b.d)
				.slice(0, 8);
			for (const e of onLine) {
				console.log(`[VR][dbg] 挡线: (${e.c.x.toFixed(0)},${e.c.y.toFixed(0)})r${e.c.r.toFixed(1)} net=${e.c.net ?? '-'} L${e.c.layer ?? '*'} block=${!!e.c.blockerOnly} noPrune=${!!e.c.noPrune} 距=${e.d.toFixed(1)}`);
			}
		}
		return { ok: false, prims: [] };
	}

	// 6. 回溯路径 → 精确图元（边自带几何，无需重算；bestDst 为状态 id）
	const prims: RBPrimitive[] = [];
	let srcNode = Math.floor(bestDst / 3);
	{
		const revEdges: VGEdge[] = [];
		for (let cur = bestDst; prevNode[cur] !== -1; cur = prevNode[cur]) {
			const e = prevEdge[cur];
			if (!e) break;
			revEdges.push(e);
			srcNode = Math.floor(prevNode[cur] / 3);
		}
		for (let k = revEdges.length - 1; k >= 0; k--) {
			const e = revEdges[k];
			if (e.isArc) {
				prims.push({
					kind: 'arc',
					cx: e.cx!, cy: e.cy!, r: e.r!,
					startAngle: e.startAngle!, sweep: e.sweep!,
				});
			} else {
				prims.push({ kind: 'line', x1: e.x1!, y1: e.y1!, x2: e.x2!, y2: e.y2! });
			}
		}
	}

	return {
		ok: true,
		prims,
		src: srcCands[srcNode],
		dst: dstCands[Math.floor(bestDst / 3) - S],
	};
}
