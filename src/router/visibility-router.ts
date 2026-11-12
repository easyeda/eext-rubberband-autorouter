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
	RBObstacle, RBPrimitive, capsuleToCircles, pointSegDist, dist,
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

/**
 * Math.hypot(a, b) 的两参数等价实现（V8 内置 MathHypot 的算法：取 max 缩放 + Kahan 求和，
 * 两项时 Kahan 补偿恒为 0，退化为 sqrt(n1² + n2²) * max）。内置版每次调用约 50ns（变长参数 +
 * 临时数组），热循环里是显著开销；此版约 10ns，且逐位一致。
 * 逐位一致只对 V8 的该算法成立，所以模块加载时用确定性样本（随机量级 + 特殊值）与 Math.hypot
 * 对拍一遍，任何不一致就整体退回 Math.hypot —— 换了引擎也不会改变结果。
 */
function hypotFast(a: number, b: number): number {
	a = Math.abs(a); b = Math.abs(b);
	let m = 0;
	if (a === a && a > m) m = a;
	if (b === b && b > m) m = b;
	if (m === Infinity) return Infinity;
	if (a !== a || b !== b) return NaN;
	if (m === 0) return 0;
	const n1 = a / m, n2 = b / m;
	return Math.sqrt(n1 * n1 + n2 * n2) * m;
}
function hypotSelfCheck(): boolean {
	const special = [0, -0, 1, -1, 1e-310, 1e300, -1e300, 5e-324, Infinity, -Infinity, NaN, 0.1, 3, 4, 1234.5678];
	for (const a of special) for (const b of special) {
		if (!Object.is(Math.hypot(a, b), hypotFast(a, b))) return false;
	}
	let st = 0x9E3779B9;
	const rnd = (): number => {   // xorshift32，确定性
		st ^= st << 13; st >>>= 0; st ^= st >>> 17; st ^= st << 5; st >>>= 0;
		return st / 4294967296;
	};
	for (let i = 0; i < 20000; i++) {
		const sc = Math.pow(10, Math.floor(rnd() * 9) - 3);
		const a = (rnd() - 0.5) * sc * 1000, b = (rnd() - 0.5) * sc * 1000;
		if (Math.hypot(a, b) !== hypotFast(a, b)) return false;
	}
	return true;
}
const HYPOT_FAST_OK = hypotSelfCheck();
/** 与 Math.hypot(a, b) 逐位相同（见 hypotFast） */
function hyp2(a: number, b: number): number {
	return HYPOT_FAST_OK ? hypotFast(a, b) : Math.hypot(a, b);
}

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
 *
 * 扁平化实现（纯加速，判定布尔值与旧实现逐位一致）：
 *  - 桶为 CSR（cellStart/cellItems，Int32Array），不再是 number[][]；
 *  - 每圆的 x/y/阈值/网络号/层号放进 Float64Array/Int32Array，查询内联，无回调闭包；
 *  - 「永远挡不住」的圆（soft / 同网非 forceBlock / 阈值≤0）不进桶：它们在旧谓词里
 *    本来就只会被跳过，所以剪掉不改变任何答案（网络由构造参数 net 给定）；
 *  - 阈值 = r - bulge - EPS_TOUCH，与旧谓词同样的运算顺序（先减 bulge 再减 EPS 再减 tol）；
 *    先用平方距离 + 1e-9 相对余量做保守预拒（hypot 误差 ~1e-16），只有可能挡住时才算
 *    精确的 Math.hypot 与链容差 —— 布尔结果不变；
 *  - 「最近挡路者优先」：记住最近 8 个挡住过查询的圆，下次先用同一谓词测它们（~92% 的查询在此得出 blocked）。
 *    只缓存插桶半径未被 400 截断且阻挡半径不超出插桶半径的圆（保证网格本来也会访问到它）。
 */
class CircleGrid {
	// 稠密局部网格：桶坐标以所有圆的包围盒为原点重新编号。
	// 旧实现用 (gx+2048)*4096+(gy+2048) 的稀疏键，buckets.length 恒为
	// ~8.4M，每个求解器都要 new Int32Array(8.4M)（~33MB）——case12 这种
	// 大板 200+ 次求解时是纯 GC/清零开销。稠密网格的格子数只有
	// (板宽/100)×(板高/100)（~25×22），分配和查表都便宜一个数量级。
	private cellStart: Int32Array = new Int32Array(2);
	private cellItems: Int32Array = new Int32Array(0);
	private stamp: Int32Array;
	private curStamp = 0;
	private minGx = 0;
	private minGy = 0;
	private w = 1;
	private h = 1;
	/** 异常大坐标范围时退回旧的稀疏桶（避免 w*h 爆内存） */
	private legacy = false;
	private legacyBuckets: number[][] = [];
	// 每圆扁平数据（下标 = 构造时 circles 的下标）
	private px: Float64Array;
	private py: Float64Array;
	private pr: Float64Array;       // 原始半径（链容差用）
	private thr: Float64Array;      // r - bulge - EPS_TOUCH（不可能挡住 = -Infinity）
	private thrSq: Float64Array;    // 预拒用：(thr*(1+1e-9))²；不可能挡住 = -1
	private deepSq: Float64Array;   // 必挡用：((thr-CHAIN_TOL)*(1-1e-9))²（thr-CHAIN_TOL ≤ 1e-3 时 = -1，不启用）
	private layer: Float64Array;    // layer ?? -1
	private netId: Int32Array;      // 0 = 无网络
	private cacheable: Uint8Array;
	private blk = new Int32Array(8).fill(-1);   // 最近挡路者环形缓存（8 槽）
	private blkPos = 0;

	private key(gx: number, gy: number): number {
		const lx = gx - this.minGx, ly = gy - this.minGy;
		if (lx < 0 || ly < 0 || lx >= this.w || ly >= this.h) return -1;
		return ly * this.w + lx;
	}

	constructor(circles: RBObstacle[], net = '') {
		const n = circles.length;
		this.px = new Float64Array(n);
		this.py = new Float64Array(n);
		this.pr = new Float64Array(n);
		this.thr = new Float64Array(n);
		this.thrSq = new Float64Array(n);
		this.deepSq = new Float64Array(n).fill(-1);
		this.layer = new Float64Array(n);
		this.netId = new Int32Array(n);
		this.cacheable = new Uint8Array(n);
		const netIds = new Map<string, number>();
		const canBlock = new Uint8Array(n);
		for (let i = 0; i < n; i++) {
			const c = circles[i];
			this.px[i] = c.x; this.py[i] = c.y; this.pr[i] = c.r;
			this.layer[i] = c.layer ?? -1;
			if (c.net) {
				let id = netIds.get(c.net);
				if (id === undefined) { id = netIds.size + 1; netIds.set(c.net, id); }
				this.netId[i] = id;
			}
			const th = c.r - (c.bulge ?? 0) - EPS_TOUCH;
			const ok = !c.soft && !(net && c.net === net && !c.forceBlock) && th > 0;
			if (ok) {
				canBlock[i] = 1;
				this.thr[i] = th;
				const s = th * (1 + 1e-9);
				this.thrSq[i] = s * s;
				if (th - CHAIN_TOL > 1e-3) { const dp = (th - CHAIN_TOL) * (1 - 1e-9); this.deepSq[i] = dp * dp; }
				this.cacheable[i] = (c.r <= 400 && th <= c.r) ? 1 : 0;
			} else {
				this.thr[i] = -Infinity;
				this.thrSq[i] = -1;
			}
		}
		if (!n) { this.stamp = new Int32Array(0); return; }
		// 圆的 bbox 覆盖范围（插入半径上限 400mil，防异常数据撑爆）
		const ranges = new Float64Array(n * 4);
		let minGx = Infinity, maxGx = -Infinity, minGy = Infinity, maxGy = -Infinity;
		for (let i = 0; i < n; i++) {
			const c = circles[i];
			const r = Math.min(Math.max(c.r, 0), 400);
			const i0 = Math.floor((c.x - r) / GRID_CELL), i1 = Math.floor((c.x + r) / GRID_CELL);
			const j0 = Math.floor((c.y - r) / GRID_CELL), j1 = Math.floor((c.y + r) / GRID_CELL);
			ranges[i * 4] = i0; ranges[i * 4 + 1] = i1; ranges[i * 4 + 2] = j0; ranges[i * 4 + 3] = j1;
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
			for (let i = 0; i < n; i++) {
				const c = circles[i];
				const k = (Math.floor(c.x / GRID_CELL) + 2048) * 4096 + (Math.floor(c.y / GRID_CELL) + 2048);
				let arr = this.legacyBuckets[k];
				if (!arr) { arr = []; this.legacyBuckets[k] = arr; }
				arr.push(i);
			}
			this.stamp = new Int32Array(0);
			return;
		}
		// 多格插入：把圆放进它 bbox 覆盖的每个格子。这样射线只查经过的
		// 格子，不需要每个采样点再查 3×3 邻域（旧实现 9 倍桶查找量）。
		// CSR 两遍：先数后填。
		const start = new Int32Array(total + 1);
		const w = this.w;
		for (let i = 0; i < n; i++) {
			if (!canBlock[i]) continue;
			const i0 = ranges[i * 4], i1 = ranges[i * 4 + 1], j0 = ranges[i * 4 + 2], j1 = ranges[i * 4 + 3];
			for (let gx = i0; gx <= i1; gx++) {
				const lx = gx - minGx;
				for (let gy = j0; gy <= j1; gy++) start[lx + (gy - minGy) * w + 1]++;
			}
		}
		for (let k = 0; k < total; k++) start[k + 1] += start[k];
		const items = new Int32Array(start[total]);
		const fill = start.slice(0, total);
		for (let i = 0; i < n; i++) {
			if (!canBlock[i]) continue;
			const i0 = ranges[i * 4], i1 = ranges[i * 4 + 1], j0 = ranges[i * 4 + 2], j1 = ranges[i * 4 + 3];
			for (let gx = i0; gx <= i1; gx++) {
				const lx = gx - minGx;
				for (let gy = j0; gy <= j1; gy++) items[fill[lx + (gy - minGy) * w]++] = i;
			}
		}
		this.cellStart = start;
		this.cellItems = items;
		this.stamp = new Int32Array(total);
	}

	/** 阻挡圆 k 对被切圆 t 的链容差（与 chainTolFor 同义：同网同层且圆盘相交 → CHAIN_TOL） */
	private chainTol(k: number, t: number): number {
		if (t < 0) return 0;
		const nk = this.netId[k];
		if (nk === 0 || nk !== this.netId[t]) return 0;
		if (this.layer[k] !== this.layer[t]) return 0;
		if (hyp2(this.px[k] - this.px[t], this.py[k] - this.py[t]) >= this.pr[k] + this.pr[t]) return 0;
		return CHAIN_TOL;
	}

	/** 圆 k（已确认不是 touched）是否挡住线段；与 pointSegDist(...) < thr - tol 同值 */
	private segHit(
		k: number, x1: number, y1: number, dx: number, dy: number, len2: number, tA: number, tB: number,
	): boolean {
		const cx = this.px[k], cy = this.py[k];
		let qx: number, qy: number;
		if (len2 < 1e-12) {
			qx = cx - x1; qy = cy - y1;
		} else {
			let t = ((cx - x1) * dx + (cy - y1) * dy) / len2;
			t = Math.max(0, Math.min(1, t));
			qx = cx - (x1 + t * dx); qy = cy - (y1 + t * dy);
		}
		const q2 = qx * qx + qy * qy;
		if (!(q2 <= this.thrSq[k])) return false;
		// 深度侵入：d ≤ sqrt(q2)(1+ε) < thr-CHAIN_TOL ≤ thr-tol（tol ∈ {0, CHAIN_TOL}），无需 hypot / 链容差
		if (q2 < this.deepSq[k]) return true;
		const th = this.thr[k];
		const d = hyp2(qx, qy);
		if (!(d < th)) return false;
		const tol = Math.max(this.chainTol(k, tA), this.chainTol(k, tB));
		return d < th - tol;
	}

	/** 扫一个格子里的圆（桶内去重靠格子戳记）；有圆挡住返回 true */
	private scanCell(
		key: number, x1: number, y1: number, dx: number, dy: number, len2: number, tA: number, tB: number,
	): boolean {
		if (key < 0) return false;
		if (this.stamp[key] === this.curStamp) return false;
		this.stamp[key] = this.curStamp;
		const items = this.cellItems;
		for (let p = this.cellStart[key], e = this.cellStart[key + 1]; p < e; p++) {
			const k = items[p];
			if (k === tA || k === tB) continue;
			if (this.segHit(k, x1, y1, dx, dy, len2, tA, tB)) {
				if (this.cacheable[k]) { this.blkPos = (this.blkPos + 1) & 7; this.blk[this.blkPos] = k; }
				return true;
			}
		}
		return false;
	}

	/**
	 * 线段是否被任一圆挡住（touchedA/B 为允许相切的圆下标，-1 = 无）。
	 * 等价于旧 nearSegment + 回调谓词的「是否 blocked」。
	 */
	segBlocked(x1: number, y1: number, x2: number, y2: number, tA: number, tB: number): boolean {
		// 全 NaN 线段（重合等半径圆的退化切线）：旧实现 DDA 在 NaN 格上空转 20000 步、
		// 从不命中任何圆（返回 false）；这里直接短路，结果相同
		if (x1 !== x1 && y1 !== y1 && x2 !== x2 && y2 !== y2) return false;
		const dx = x2 - x1, dy = y2 - y1;
		const len2 = dx * dx + dy * dy;
		if (this.legacy) return this.segBlockedLegacy(x1, y1, x2, y2, dx, dy, len2, tA, tB);
		// 最近挡路者优先（同一谓词，结果只会更早得出 true）
		const blk = this.blk;
		for (let s = 0; s < 8; s++) {
			const slot = (this.blkPos - s) & 7;
			const lb = blk[slot];
			if (lb >= 0 && lb !== tA && lb !== tB && this.segHit(lb, x1, y1, dx, dy, len2, tA, tB)) {
				if (s > 0) { const fs = this.blkPos & 7; blk[slot] = blk[fs]; blk[fs] = lb; }
				return true;
			}
		}
		++this.curStamp;
		// 网格 DDA（Amanatides-Woo）：只走线段真正穿过的格子
		let gx = Math.floor(x1 / GRID_CELL), gy = Math.floor(y1 / GRID_CELL);
		const gxEnd = Math.floor(x2 / GRID_CELL), gyEnd = Math.floor(y2 / GRID_CELL);
		const stepX = dx > 0 ? 1 : dx < 0 ? -1 : 0;
		const stepY = dy > 0 ? 1 : dy < 0 ? -1 : 0;
		let tMaxX = stepX === 0 ? Infinity : ((stepX > 0 ? (gx + 1) * GRID_CELL : gx * GRID_CELL) - x1) / dx;
		let tMaxY = stepY === 0 ? Infinity : ((stepY > 0 ? (gy + 1) * GRID_CELL : gy * GRID_CELL) - y1) / dy;
		const tDeltaX = stepX === 0 ? Infinity : Math.abs(GRID_CELL / dx);
		const tDeltaY = stepY === 0 ? Infinity : Math.abs(GRID_CELL / dy);
		if (this.scanCell(this.key(gx, gy), x1, y1, dx, dy, len2, tA, tB)) return true;
		let guard = 0;
		while ((gx !== gxEnd || gy !== gyEnd) && guard++ < 20000) {
			if (tMaxX < tMaxY) {
				gx += stepX; tMaxX += tDeltaX;
			} else if (tMaxY < tMaxX) {
				gy += stepY; tMaxY += tDeltaY;
			} else {
				// 正好穿过格点：两个方向依次走，避免漏格
				gx += stepX; tMaxX += tDeltaX;
				if (this.scanCell(this.key(gx, gy), x1, y1, dx, dy, len2, tA, tB)) return true;
				gy += stepY; tMaxY += tDeltaY;
			}
			if (this.scanCell(this.key(gx, gy), x1, y1, dx, dy, len2, tA, tB)) return true;
		}
		return false;
	}

	/** 旧稀疏路径（极少触发）：按采样点查 3×3，Set 按稀疏键去重；不用缓存 */
	private segBlockedLegacy(
		x1: number, y1: number, x2: number, y2: number, dx: number, dy: number, len2: number, tA: number, tB: number,
	): boolean {
		const seen = new Set<number>();
		const d = Math.hypot(x2 - x1, y2 - y1);
		const steps = Math.max(1, Math.ceil(d / (GRID_CELL * 0.8)));
		for (let s = 0; s <= steps; s++) {
			const t = s / steps;
			const px = x1 + (x2 - x1) * t, py = y1 + (y2 - y1) * t;
			const gx = Math.floor(px / GRID_CELL), gy = Math.floor(py / GRID_CELL);
			for (let ox = -1; ox <= 1; ox++) for (let oy = -1; oy <= 1; oy++) {
				const key = (gx + ox + 2048) * 4096 + (gy + oy + 2048);
				if (key < 0 || seen.has(key)) continue;
				seen.add(key);
				const arr = this.legacyBuckets[key];
				if (!arr) continue;
				for (let q = 0; q < arr.length; q++) {
					const k = arr[q];
					if (k === tA || k === tB) continue;
					if (this.segHit(k, x1, y1, dx, dy, len2, tA, tB)) return true;
				}
			}
		}
		return false;
	}

	/**
	 * 表面经过点 p 的第一个圆下标（|dist - r| ≤ eps），找不到 -1；与 findSurfaceCircle 线性扫描同值
	 * （下标 = 构造时 circles 的下标，取最小者）。先用 |Δx|、|Δy| ≤ r+eps（+1e-7 盖住 hypot 舍入）
	 * 在 Float64Array 上预拒，只对幸存者算 Math.hypot。
	 */
	findSurface(x: number, y: number, eps = 0.05): number {
		const px = this.px, py = this.py, pr = this.pr;
		for (let i = 0, n = pr.length; i < n; i++) {
			const m = pr[i] + eps + 1e-7;
			const dx = x - px[i];
			if (dx > m || dx < -m) continue;
			const dy = y - py[i];
			if (dy > m || dy < -m) continue;
			if (Math.abs(hyp2(dx, dy) - pr[i]) <= eps) return i;
		}
		return -1;
	}

	/** 点所在格子里是否有圆挡住该点（selfIdx 为圆弧自身所在圆，-1 = 无）；多格插入保证覆盖 */
	pointBlocked(px: number, py: number, selfIdx: number): boolean {
		const gx = Math.floor(px / GRID_CELL), gy = Math.floor(py / GRID_CELL);
		let arr: ArrayLike<number>, p: number, e: number;
		if (this.legacy) {
			const key = (gx + 2048) * 4096 + (gy + 2048);
			if (key < 0) return false;
			const b = this.legacyBuckets[key];
			if (!b) return false;
			arr = b; p = 0; e = b.length;
		} else {
			const key = this.key(gx, gy);
			if (key < 0) return false;
			arr = this.cellItems; p = this.cellStart[key]; e = this.cellStart[key + 1];
		}
		for (; p < e; p++) {
			const k = arr[p];
			if (k === selfIdx) continue;
			const qx = px - this.px[k], qy = py - this.py[k];
			const q2 = qx * qx + qy * qy;
			if (!(q2 <= this.thrSq[k])) continue;
			if (q2 < this.deepSq[k]) return true;
			const th = this.thr[k];
			const d = hyp2(qx, qy);
			if (!(d < th)) continue;
			if (d < th - this.chainTol(k, selfIdx)) return true;
		}
		return false;
	}
}

/**
 * 线段是否不侵入任何圆（快速网格查询；touchedA/B 为允许相切的圆下标）
 *
 * 阻挡判据（见 CircleGrid）：圆非 soft；同网且非 forceBlock 的圆放行；
 * pointSegDist(圆心, 线段) < (r - bulge) - EPS_TOUCH - 链容差 即挡死。
 * 弦→弧外凸补偿：带 bulge 的阻挡圆按基准半径（r - bulge）判相切，
 * 让切线能贴着已布走线的圆链走（否则外扩量会封死所有贴边切线）。
 * 链容差：阻挡圆与被切圆（切线两端所在圆）属于同一铜链时，允许
 * 弦vs弧矢高量级（≤CHAIN_TOL）的切入——那是建模误差而非真违规。
 * grid 必须用同一个 net 构造。
 */
function makeSegmentChecker(grid: CircleGrid) {
	return (x1: number, y1: number, x2: number, y2: number, touchedA: number, touchedB: number): boolean =>
		!grid.segBlocked(x1, y1, x2, y2, touchedA, touchedB);
}

/**
 * 圆弧（从 a1 扫过 sweep）是否不侵入其他圆（链容差：贴边绕行弧与弧自身所在圆同链的圆，允许矢高量级切入）
 */
function makeArcChecker(grid: CircleGrid) {
	return (cx: number, cy: number, r: number, a1: number, sweep: number, selfIdx: number): boolean => {
		const steps = Math.max(4, Math.ceil(Math.abs(sweep) * r / (GRID_CELL * 0.5)));
		for (let s = 0; s <= steps; s++) {
			const a = a1 + (sweep * s) / steps;
			const px = cx + r * Math.cos(a), py = cy + r * Math.sin(a);
			if (grid.pointBlocked(px, py, selfIdx)) return false;
		}
		return true;
	};
}

/** 仅供测试：暴露内部实现以便与暴力扫描对拍 */
export const __vrTest = { CircleGrid, makeSegmentChecker, makeArcChecker, chainTolFor, EPS_TOUCH, canonicalSort, canonicalCmpOld, findSurfaceCircle, pruneByTube, corridorSubset, orderByX, coarseTube, hypotFast, HYPOT_FAST_OK };

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
		// 队列清空后释放引用，避免 Node 进程被空闲通道挂住
		if (__yieldResolvers.length === 0) (ch.port1 as unknown as { unref?: () => void }).unref?.();
	};
	// Node 下否则进程不会自行退出；浏览器/Worker 无 unref/ref，不受影响（让出期间由 yieldToHost 临时 ref）
	(ch.port1 as unknown as { unref?: () => void }).unref?.();
	(ch.port2 as unknown as { unref?: () => void }).unref?.();
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
		(ch.port1 as unknown as { ref?: () => void }).ref?.();
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
	const xMax = x0 + (nx - 1) * CELL, yMax = y0 + (ny - 1) * CELL;   // 格心矩形
	for (const o of all) {
		if (o.soft) continue;
		if (o.net && o.net === net) continue;
		const lis = o.layer === undefined ? [0, 1] : (o.layer === 1 ? [0] : o.layer === 2 ? [1] : []);
		if (!lis.length) continue;
		const rr = o.r + INFL;
		// 圆盘够不到窗口的格心矩形 → 下面逐格的 dx²>rr² / dy 判据必然全拒（+1e-6 余量盖住浮点误差）
		if (o.x + rr < x0 - 1e-6 || o.x - rr > xMax + 1e-6 || o.y + rr < y0 - 1e-6 || o.y - rr > yMax + 1e-6) continue;
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
			// d < o.r + 2 要求 |Δx|、|Δy| < o.r + 2（+1e-6 余量盖住 hypot 舍入）：先拒，省 hypot
			const lim = o.r + 2 + 1e-6;
			const ex = o.x - px, ey = o.y - py;
			if (ex > lim || ex < -lim || ey > lim || ey < -lim) continue;
			const d = hyp2(o.x - px, o.y - py);
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
	// 八邻域（顺序：先四连通 [+x,-x,+y,-y]，再四个对角 [++,+-,-+,--]；对角权 √2）
	const D8X = [1, -1, 0, 0, 1, 1, -1, -1], D8Y = [0, 0, 1, -1, 1, -1, 1, -1];
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
			const tk = heapK[p]; heapK[p] = heapK[i]; heapK[i] = tk;
			const tv = heapV[p]; heapV[p] = heapV[i]; heapV[i] = tv;
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
				const tk = heapK[m]; heapK[m] = heapK[i]; heapK[i] = tk;
				const tv = heapV[m]; heapV[m] = heapV[i]; heapV[i] = tv;
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
		push(hyp2(s.i - d.i, s.j - d.j) * CELL, id);
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
		for (let q = 0; q < 8; q++) {
			const ni = i + D8X[q], nj = j + D8Y[q], w = q < 4 ? 1 : Math.SQRT2;
			if (ni < 0 || nj < 0 || ni >= nx || nj >= ny) continue;
			const nid = idxOf(ni, nj, li);
			if (blocked[nid] || closed[nid]) continue;
			const ng = g + w * CELL;
			if (ng < gScore[nid] - 1e-9) {
				gScore[nid] = ng;
				cameFrom[nid] = cur;
				push(ng + hyp2(ni - d.i, nj - d.j) * CELL, nid);
			}
		}
		if (nl > 1) {
			const nid = idxOf(i, j, 1 - li);
			if (!blocked[nid] && !closed[nid] && g + viaCost < gScore[nid] - 1e-9) {
				gScore[nid] = g + viaCost;
				cameFrom[nid] = cur;
				push(gScore[nid] + hyp2(i - d.i, j - d.j) * CELL, nid);
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

/** 规范排序的旧比较器（含 NaN 语义；非有限键时回退到它，保证与旧实现逐位一致） */
function canonicalCmpOld(a: RBObstacle, b: RBObstacle): number {
	return a.x - b.x || a.y - b.y || a.r - b.r || (a.net || '').localeCompare(b.net || '');
}

/**
 * 规范排序（x, y, r, net(localeCompare)）：结果与 `all.sort(canonicalCmpOld)` 逐位一致。
 * 全部键有限时旧比较器是一致的全序预序，稳定排序的结果唯一 —— 这里对下标排序，
 * 比较走 Float64Array 键（不碰对象属性），整数键相等才比网络串（相同串直接视为相等），
 * 最后用下标兜底（= 稳定）。任一键非有限（NaN/Infinity）时旧比较器不自洽，退回原比较器原地排序。
 * 返回排序后的数组（可能是新数组）。
 */
function canonicalSort(all: RBObstacle[]): RBObstacle[] {
	const n = all.length;
	if (n < 2) return all;
	const kx = new Float64Array(n), ky = new Float64Array(n), kr = new Float64Array(n);
	for (let i = 0; i < n; i++) {
		const o = all[i];
		const x = o.x, y = o.y, r = o.r;
		kx[i] = x; ky[i] = y; kr[i] = r;
		const vx = kx[i], vy = ky[i], vr = kr[i];
		if (vx - vx !== 0 || vy - vy !== 0 || vr - vr !== 0) return all.sort(canonicalCmpOld);
	}
	const idx = new Array<number>(n);
	for (let i = 0; i < n; i++) idx[i] = i;
	idx.sort((a, b) => {
		const dx = kx[a] - kx[b];
		if (dx !== 0) return dx;
		const dy = ky[a] - ky[b];
		if (dy !== 0) return dy;
		const dr = kr[a] - kr[b];
		if (dr !== 0) return dr;
		const na = all[a].net || '', nb = all[b].net || '';
		if (na !== nb) {
			const c = na.localeCompare(nb);
			if (c !== 0) return c;
		}
		return a - b;
	});
	const out = new Array<RBObstacle>(n);
	for (let i = 0; i < n; i++) out[i] = all[idx[i]];
	return out;
}

/**
 * 多端点寻路：源/目标各一组候选点，求「引出段 + 路径 + 引入段」总长最小的走线。
 *
 * refA/refB：走廊剪枝用的参考线段（通常是两焊盘中心）；缺省用首个候选点。
 * isCancelled：外部取消探针 —— 在每个协作让出点轮询（≤16ms 粒度），
 * 取消后立即放弃求解。没有它，「停止布线」要等当前边整条兜底链
 * （直连×2 + 边界候选 + 底层 + 过孔候选，预算 3s×N）烧完才能停。
 */
export function routeBetweenEndpoints(
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
	// 差分校验钩子（opt-in，仅测试/基准用）：globalThis.__RB_VR_VERIFY__(args, impl) 接管调用，
	// 用来把本实现的结果与旧实现逐位对拍；未设置时零开销直通
	const vf = (globalThis as any).__RB_VR_VERIFY__;
	if (vf) return vf([srcCands, dstCands, obstacles, net, expands, budgetMs, refA, refB, isCancelled, directOnly], routeBetweenEndpointsImpl);
	return routeBetweenEndpointsImpl(srcCands, dstCands, obstacles, net, expands, budgetMs, refA, refB, isCancelled, directOnly);
}

async function routeBetweenEndpointsImpl(
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
	let all: RBObstacle[] = [];
	for (const o of obstacles) {
		if (o.kind === 'circle') {
			all.push(o);
		} else {
			for (const c of capsuleToCircles(o.x1!, o.y1!, o.x2!, o.y2!, o.r)) {
				all.push({ ...o, kind: 'circle', x: c.x, y: c.y, r: c.r });
			}
		}
	}

	// ── 直连快判（directOnly 模式）──
	// 快扫专用：只判「直线能不能过」，不建任何图。非平凡边全部留给
	// 完整链求解 —— 否则快扫建一次走廊图、收尾轮再建一次，同一成本付两遍
	//（实测占大板 40% 预算）。
	if (directOnly) {
		if (srcCands.length === 1 && dstCands.length === 1) {
			const grid = new CircleGrid(all, net);
			const segOK = makeSegmentChecker(grid);
			const a = srcCands[0], b = dstCands[0];
			if (segOK(a.x, a.y, b.x, b.y, -1, -1)) {
				return { ok: true, prims: [{ kind: 'line', x1: a.x, y1: a.y, x2: b.x, y2: b.y }] };
			}
		}
		return { ok: false, prims: [] };
	}

	// 规范化排序（directOnly 的单段判定与顺序无关，故排在其后）：与障碍收集顺序无关的确定性 tie-break
	// （EDA 与离线的收集顺序不同，等长路径的选择会随顺序漂移）
	all = canonicalSort(all);

	const ax = refA ?? { x: srcCands[0].x, y: srcCands[0].y };
	const bx = refB ?? { x: dstCands[0].x, y: dstCands[0].y };

	// ── 管道引导（coarse-to-fine）──
	// 先用粗网格 A*（16mil 格）找一条可行走廊折线（毫秒级），只保留折线
	// 管道近旁的障碍建切点图 —— 图规模从走廊全量 ~150 圆降到 ~40 圆，
	// O(n²) 建图成本近似平方下降（实测单次求解 0.5s → 0.05~0.1s）。
	// 粗路径失败 = 走廊内数学无路 → 跳过管道级直接走直连走廊退避；
	// 管道级求解失败/复验失败 → 照旧退避 150/500，正确性不受影响。
	let vgrid: CircleGrid | null = null;   // 复验网格：各走廊级别共用（同一 all / net）
	const tubePathsBefore = vrStats.tubePaths;
	if (!(globalThis as any).__RB_NO_TUBE__) {
		vrStats.tubeTries++;
		const tube = coarseTube(all, net, ax, bx);
		if (tube) {
			vrStats.tubePaths++;
			const kept: RBObstacle[] = [];
			const picked: RBObstacle[] = [];
			pruneByTube(all, tube, kept, picked);
			const subset = kept.concat(picked);
			if ((globalThis as any).__RB_DEBUG__) {
				console.log(`[VR][dbg] 管道级: 折线${tube.length}点 子集=${subset.length}`);
			}
			const deadline = Date.now() + budgetMs;
			(globalThis as any).__RB_PHASE__ = 'vr:solve';
			const res = await solveMulti(srcCands, dstCands, subset, net, deadline, isCancelled);
			(globalThis as any).__RB_PHASE__ = 'vr:validate';
			if (res.ok && await validatePrimsWithGrid(res.prims, all, (vgrid ??= new CircleGrid(all, net)))) {
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
			subset = corridorSubset(all, ax, bx, expand);
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
		if (res.ok && await validatePrimsWithGrid(res.prims, all, (vgrid ??= new CircleGrid(all, net)))) return res;
	}
	return { ok: false, prims: [] };
}

/**
 * 走廊剪枝子集：noPrune 全部保留，其余圆到直连线（或两端点）距离 < r+expand 的入选；
 * 候选超过 MAX_SUBSET 时只留离直连线最近的（规模上限，防止走廊放宽后图爆炸）。
 */
function corridorSubset(
	all: RBObstacle[], ax: { x: number; y: number }, bx: { x: number; y: number }, expand: number,
): RBObstacle[] {
	const kept: RBObstacle[] = [];
	const candidates: Array<{ o: RBObstacle; d: number }> = [];
	// 直连线 bbox 预拒：圆心离 bbox 超过 r+expand（+1e-6 余量盖住浮点误差）时，
	// 三个距离（到线段、到两端点）都必然 ≥ r+expand，原判据全为假，跳过
	const bx0 = Math.min(ax.x, bx.x), bx1 = Math.max(ax.x, bx.x);
	const by0 = Math.min(ax.y, bx.y), by1 = Math.max(ax.y, bx.y);
	for (const c of all) {
		if (c.noPrune) { kept.push(c); continue; }
		const tm = c.r + expand + 1e-6;
		if (c.x < bx0 - tm || c.x > bx1 + tm || c.y < by0 - tm || c.y > by1 + tm) continue;
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
	return kept.concat(candidates.map(c => c.o));
}

/**
 * 管道剪枝：noPrune 全部进 kept；其余圆离管道折线任一段 < r+80 的进 picked（保持 all 内顺序）。
 * 等价于旧的「逐段取最小距离再比较」：min < T ⇔ 存在某段 d < T（NaN 距离两边都不入选）。
 * 每段先做 bbox 预拒（圆心离段 bbox 超过 T+1e-6 则该段距离必然 ≥ T，不必算 hypot），
 * 再对命中者用与 pointSegDist 同一表达式精确判定。
 */
function pruneByTube(
	all: RBObstacle[], tube: Array<{ x: number; y: number }>,
	kept: RBObstacle[], picked: RBObstacle[],
): void {
	const ns = tube.length - 1;
	const sx1 = new Float64Array(Math.max(ns, 0)), sy1 = new Float64Array(Math.max(ns, 0));
	const sx2 = new Float64Array(Math.max(ns, 0)), sy2 = new Float64Array(Math.max(ns, 0));
	const mnx = new Float64Array(Math.max(ns, 0)), mxx = new Float64Array(Math.max(ns, 0));
	const mny = new Float64Array(Math.max(ns, 0)), mxy = new Float64Array(Math.max(ns, 0));
	let gx0 = Infinity, gx1 = -Infinity, gy0 = Infinity, gy1 = -Infinity;
	for (let k = 0; k < ns; k++) {
		const a = tube[k], b = tube[k + 1];
		sx1[k] = a.x; sy1[k] = a.y; sx2[k] = b.x; sy2[k] = b.y;
		const lx = Math.min(a.x, b.x), hx = Math.max(a.x, b.x), ly = Math.min(a.y, b.y), hy = Math.max(a.y, b.y);
		mnx[k] = lx; mxx[k] = hx; mny[k] = ly; mxy[k] = hy;
		if (lx < gx0) gx0 = lx;
		if (hx > gx1) gx1 = hx;
		if (ly < gy0) gy0 = ly;
		if (hy > gy1) gy1 = hy;
	}
	for (const c of all) {
		if (c.noPrune) { kept.push(c); continue; }
		const px = c.x, py = c.y;
		const T = c.r + 80, tm = T + 1e-6;
		if (px < gx0 - tm || px > gx1 + tm || py < gy0 - tm || py > gy1 + tm) continue;
		for (let k = 0; k < ns; k++) {
			if (px < mnx[k] - tm || px > mxx[k] + tm || py < mny[k] - tm || py > mxy[k] + tm) continue;
			if (pointSegDist(px, py, sx1[k], sy1[k], sx2[k], sy2[k]) < T) { picked.push(c); break; }
		}
	}
}

/**
 * 按 x 升序的稳定下标序（等价于 `idx.sort((a, b) => xs[a] - xs[b])`）。
 * 已有序时直接返回恒等排列（稳定排序对有序输入不动）；有 NaN/Infinity 键时旧比较器不自洽，
 * 退回原比较器；否则比较 Float64Array 键并以下标兜底（= 稳定），结果唯一。
 */
function orderByX(xs: Float64Array): number[] {
	const n = xs.length;
	const idx = new Array<number>(n);
	for (let i = 0; i < n; i++) idx[i] = i;
	let sorted = true, finite = true;
	for (let i = 0; i < n; i++) {
		const x = xs[i];
		if (x - x !== 0) { finite = false; break; }
		if (i > 0 && xs[i - 1] > x) sorted = false;
	}
	if (!finite) return idx.sort((a, b) => xs[a] - xs[b]);
	if (sorted) return idx;
	return idx.sort((a, b) => {
		const d = xs[a] - xs[b];
		return d !== 0 ? d : a - b;
	});
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
	return validatePrimsWithGrid(prims, all, new CircleGrid(all, net));
}

/** validatePrimsAgainst 的实现：grid 必须由同一 (all, net) 构造（同一次求解的各走廊级别可共用一个） */
async function validatePrimsWithGrid(prims: RBPrimitive[], all: RBObstacle[], grid: CircleGrid): Promise<boolean> {
	const segOK = makeSegmentChecker(grid);
	const arcOK = makeArcChecker(grid);
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
			const tA = grid.findSurface(pr.x1, pr.y1);
			const tB = grid.findSurface(pr.x2, pr.y2);
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
			const selfIdx = grid.findSurface(startPt.x, startPt.y);
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
	// 每圆扁平数据（建图热循环读它们，不碰对象属性）：坐标/半径 + 「同网非 forceBlock」跳过标记
	const cX = new Float64Array(n), cY = new Float64Array(n), cR = new Float64Array(n);
	const cSkip = new Uint8Array(n);
	for (let i = 0; i < n; i++) {
		const c = circles[i];
		cX[i] = c.x; cY[i] = c.y; cR[i] = c.r;
		if (net && c.net === net && !c.forceBlock) cSkip[i] = 1;
	}

	const grid = new CircleGrid(everything, net);
	const segOK = makeSegmentChecker(grid);
	const arcOK = makeArcChecker(grid);

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
	// 数值键（圆下标*8192 + 量化角度+4096）：角度∈[-π,π]，量化值 ∈[-3142,3142]，键与
	// 字符串键 `${ci}:${round}` 一一对应（避免每个切点拼字符串）；NaN 角度（重合等半径圆
	// 的退化切线）按圆单独记一个槽，等价于旧的 `${ci}:NaN`
	const ptIndex = new Map<number, number>();
	const nanPt = new Map<number, number>();
	// 按圆分桶（避免弧边循环里对全部切点做字符串扫描：那是 O(n×切点数)）
	const ptsByCircle: number[][] = Array.from({ length: n }, () => []);
	const getOrCreatePt = (ci: number, x: number, y: number): number => {
		const angle = Math.atan2(y - cY[ci], x - cX[ci]);
		const q = Math.round(angle * 1000);
		const isNaNq = q !== q;
		const key = isNaNq ? ci : ci * 8192 + (q + 4096);
		const idxMap = isNaNq ? nanPt : ptIndex;
		const existing = idxMap.get(key);
		if (existing !== undefined) return existing;
		const idx = nodes.length;
		nodes.push({
			circle: ci,
			x: cX[ci] + cR[ci] * Math.cos(angle),
			y: cY[ci] + cR[ci] * Math.sin(angle),
			angle, adj: [],
		});
		idxMap.set(key, idx);
		ptsByCircle[ci].push(idx);
		return idx;
	};

	// 2. 终端（出线候选）↔ 圆切线 —— 提前到建图之前：本步便宜
	//（O(n·(S+D))），且「终端零切线 ∧ 无直连」意味着数学上无路，
	// 可以跳过整个 O(n²) 公切线建图（失败求解占快扫的大头）
	const connectTerminal = (termIdx: number) => {
		const term = nodes[termIdx];
		const tmx = term.x, tmy = term.y;
		for (let i = 0; i < n; i++) {
			if (cSkip[i]) continue;
			const cix = cX[i], ciy = cY[i], cir = cR[i];
			const dTerm = hyp2(tmx - cix, tmy - ciy);
			if (dTerm < cir + EPS_TOUCH) continue; // 终端在圆内/圆上：切线退化
			// getTangents(term, r=0, l1=false, 圆, l2=l) 内联：d 与 dist 同式；单位向量对两个 l 共用
			const vx = (cix - tmx) / dTerm;
			const vy = (ciy - tmy) / dTerm;
			for (let li = 0; li < 2; li++) {
				const l = li === 0;   // 先 true 后 false
				let tx1: number, ty1: number, tx2: number, ty2: number;
				if (dTerm < 1e-9) {
					tx1 = tmx; ty1 = tmy; tx2 = cix; ty2 = ciy;
				} else {
					const adjR2 = cir * (l ? -1 : 1);   // l1(false) === l2(l) ? 1 : -1
					const c = (0 - adjR2) / dTerm;
					let h = 1 - c * c;
					if (h < 0) h = 0;
					h = Math.sqrt(h) * 1;   // l1 = false
					const nx = vx * c - h * vy;
					const ny = vy * c + h * vx;
					tx1 = tmx + 0 * nx; ty1 = tmy + 0 * ny; tx2 = cix + adjR2 * nx; ty2 = ciy + adjR2 * ny;
				}
				if (!segOK(tx1, ty1, tx2, ty2, -1, i)) continue;
				const len = hyp2(tx2 - tx1, ty2 - ty1);
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
			const len = hyp2(b.x - a.x, b.y - a.y);
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
	const __xOrder = orderByX(cX);
	const __xs = new Float64Array(n);   // 按 x 序排列的 x（内层 break 判据读它）
	for (let k = 0; k < n; k++) __xs[k] = cX[__xOrder[k]];
	let __maxR = 0;
	for (const c of circles) if (c.r > __maxR) __maxR = c.r;
	let __pairsVisited = 0;
	const __pairsAll = (n * (n - 1)) / 2;
	for (let oi = 0; oi < n && !timedOut; oi++) {
		const i = __xOrder[oi];
		if (cSkip[i]) continue;
		{
			const now = Date.now();
			if (now > deadline) { timedOut = true; break; }
			if (now - lastYield >= YIELD_INTERVAL_MS) {
				lastYield = now;
				await yieldToHost();
			}
			if (cancelled()) { timedOut = true; break; }
		}
		const cix = cX[i], ciy = cY[i], cir = cR[i];
		const reach = cir + __maxR + (Number((globalThis as any).__RB_VR_REACH__) || 800);
		const inner = !!(globalThis as any).__RB_VR_INNER__;
		for (let oj = oi + 1; oj < n; oj++) {
			if (__xs[oj] - cix > reach) break;
			if ((++__pairsVisited & 31) === 0) {
				const now = Date.now();
				if (now > deadline) { timedOut = true; break; }
				if (now - lastYield >= YIELD_INTERVAL_MS) {
					lastYield = now;
					await yieldToHost();
				}
				if (cancelled()) { timedOut = true; break; }
			}
			const j = __xOrder[oj];
			// y 预筛：x 序扫描里 |Δy| 超界的圆对占大半，切线 4 组组合 + segOK
			// 网格遍历全浪费。只 continue 不改变扫描顺序 → 输出逐位一致（无
			// tie-break 混沌回归），实测建图占求解 85~90%，这刀纯赚。
			const cjx = cX[j], cjy = cY[j], cjr = cR[j];
			const dy = cjy - ciy;
			if (dy > reach || dy < -reach) continue;
			if (cSkip[j]) continue;
			const d = hyp2(cix - cjx, ciy - cjy);
			// 切线解算内联（与 getTangents 逐位一致）：单位向量 vx/vy 对每
			// 圆对只算一次（原来 4 组组合各做一次 hypot+除法）；d≈0 时
			// cVal=±Infinity 被 |cVal|>1 提前拦下，与 getTangents 的 d<1e-9
			// 分支在此处不可达，行为等价。
			const vx = (cjx - cix) / d;
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
				if (!inner && l1 !== l2) continue;
				const adjR2 = cjr * (l1 === l2 ? 1 : -1);
				const cVal = (cir - adjR2) / d;
				if (Math.abs(cVal) > 1) continue; // 该类型切线不存在
				let hs = Math.sqrt(1 - cVal * cVal);
				if (l1) hs = -hs; // getTangents: h = sqrt(1-c²)·(l1 ? -1 : 1)
				const nx = vx * cVal - hs * vy;
				const ny = vy * cVal + hs * vx;
				const tx1 = cix + cir * nx;
				const ty1 = ciy + cir * ny;
				const tx2 = cjx + adjR2 * nx;
				const ty2 = cjy + adjR2 * ny;
				if (grid.segBlocked(tx1, ty1, tx2, ty2, i, j)) continue;
				const len = hyp2(tx2 - tx1, ty2 - ty1);
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
	// 连通性快败：把「同一圆上的全部切点」视为互通（弧边的超集），从源候选沿切线边做 BFS：
	//  - 没有任何目标候选可达 ⇒ 真实图（弧边是其子集、Dijkstra 还有旋转约束）里必然无路，
	//    与跑完弧边 + Dijkstra 得到的 {ok:false} 相同，省掉弧边几何复核与 Dijkstra；
	//  - 不在源分量里的圆，其弧边 Dijkstra 永远走不到（弧只连同圆切点、切线边只连同分量），
	//    不必做弧几何复核（该圆的邻接表保持无弧边，也不会被读到）。
	// 开了 debug/verbose 诊断时走完整路径（含全部弧边），保持日志不变。
	let circleSeen: Uint8Array | null = null;
	if (!(globalThis as any).__RB_DEBUG__ && !(globalThis as any).__RB_VERBOSE__) {
		const seen = new Uint8Array(nodes.length);
		circleSeen = new Uint8Array(n);
		const queue = new Int32Array(nodes.length);
		let qh = 0, qt = 0;
		for (let i = 0; i < S; i++) { seen[i] = 1; queue[qt++] = i; }
		while (qh < qt) {
			const u = queue[qh++];
			const nu = nodes[u];
			const adj = nu.adj;
			for (let k = 0; k < adj.length; k++) {
				const v = adj[k].to;
				if (seen[v]) continue;
				seen[v] = 1; queue[qt++] = v;
			}
			const ci = nu.circle;
			if (ci >= 0 && !circleSeen[ci]) {
				circleSeen[ci] = 1;
				const pts = ptsByCircle[ci];
				for (let k = 0; k < pts.length; k++) {
					const v = pts[k];
					if (seen[v]) continue;
					seen[v] = 1; queue[qt++] = v;
				}
			}
		}
		let dstReached = false;
		for (let j = 0; j < D; j++) if (seen[S + j]) { dstReached = true; break; }
		if (!dstReached) {
			vrStats.calls++;
			vrStats.solveMs += Date.now() - __t0;
			vrStats.pairMs += __tPair;
			vrStats.termMs += __tTerm;
			vrStats.maxN = Math.max(vrStats.maxN, n);
			vrStats.sumN += n;
			vrStats.pairsVisited += __pairsVisited;
			vrStats.failCalls++;
			return { ok: false, prims: [] };
		}
	}
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
		if (circleSeen && !circleSeen[i]) continue;   // 不在源分量里：弧边永远走不到
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
	const distArr = new Float64Array(total3).fill(Infinity);
	const prevNode = new Int32Array(total3).fill(-1);
	const prevEdge = new Array<VGEdge | null>(total3).fill(null);
	const settled = new Uint8Array(total3);
	// 二叉堆：键/值分两个数组（不再每次入堆分配 [d, v] 元组）；上浮/下沉比较与交换顺序同旧实现
	const heapD: number[] = [], heapV: number[] = [];
	const heapPush = (d: number, v: number) => {
		heapD.push(d); heapV.push(v);
		let i = heapD.length - 1;
		while (i > 0) {
			const p = (i - 1) >> 1;
			if (heapD[p] <= heapD[i]) break;
			const td = heapD[p]; heapD[p] = heapD[i]; heapD[i] = td;
			const tv = heapV[p]; heapV[p] = heapV[i]; heapV[i] = tv;
			i = p;
		}
	};
	let popD = 0, popV = 0;
	const heapPop = (): boolean => {
		if (heapD.length === 0) return false;
		popD = heapD[0]; popV = heapV[0];
		const lastD = heapD.pop()!, lastV = heapV.pop()!;
		const len = heapD.length;
		if (len > 0) {
			heapD[0] = lastD; heapV[0] = lastV;
			let i = 0;
			for (;;) {
				const l = 2 * i + 1, r = 2 * i + 2;
				let s = i;
				if (l < len && heapD[l] < heapD[s]) s = l;
				if (r < len && heapD[r] < heapD[s]) s = r;
				if (s === i) break;
				const td = heapD[s]; heapD[s] = heapD[i]; heapD[i] = td;
				const tv = heapV[s]; heapV[s] = heapV[i]; heapV[i] = tv;
				i = s;
			}
		}
		return true;
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
		if (!heapPop()) break;
		const du = popD, s = popV;
		if (settled[s]) continue;
		settled[s] = 1;
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
					if (e.sweep! >= 0) { dx = -Math.sin(e.startAngle!); dy = Math.cos(e.startAngle!); }
					else { dx = Math.sin(e.startAngle!); dy = -Math.cos(e.startAngle!); }
				} else {
					dx = e.x2! - e.x1!; dy = e.y2! - e.y1!;
				}
				const dl = hyp2(dx, dy) || 1;
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
					const dl = hyp2(dx, dy) || 1;
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
