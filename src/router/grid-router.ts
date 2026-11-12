/**
 * GridRouter —— 网格 A* 兜底求解器（自研实现）
 *
 * 定位：可视图求解器（visibility-router）在两类场景下会「无路径」：
 *  1. 端点焊盘被邻焊盘的净距圈覆盖（0.5mm 间距引脚排）——可视图只从
 *     单一端点出发，端点一旦落在别人的障碍圈内就无解；
 *  2. 路径必须从两个障碍之间的任意点穿过（切点图中没有对应节点）。
 *
 * 本模块把障碍（已含间距膨胀）光栅化成「每层一张占据图」，在其上跑
 * 八连通 A*（含层间过孔边），再把网格路径化简成折线/过孔。代价：
 *  - 只作为**兜底**调用（单条边），不进主路径，避免拖慢常规求解；
 *  - 网格尺寸自适应：单元过大则精度下降，过小则内存/时间不可控，
 *    用 maxCells 上限自动放大单元。
 *
 * 与可视图的关系：互补。可视图给出连续最优解（切线+圆弧，形态好），
 * 网格给出**可达性最优**（能不能过去），只在可视图失败时接管。
 */

import { RBObstacle, RBPrimitive, pointSegDist } from './rubberband-engine';

export interface GridRouteRequest {
	src: { x: number; y: number };
	dst: { x: number; y: number };
	/** 起点/终点焊盘（用于在自身铜皮内解除阻挡：出线豁免） */
	srcPad?: { x: number; y: number; r: number; viaKeepoutR?: number };
	dstPad?: { x: number; y: number; r: number; viaKeepoutR?: number };
	/** 焊盘铜皮覆盖圆（绝对坐标，半径已含少量余量）：长焊盘必须按整条
	 * 铜皮解禁，只用焊盘中心的小圆会把起点困在邻盘净距圈的「口袋」里
	 *（case12 0.5mm 排：起点四周全是邻盘间距圈，A* 只能弹出 8 个格子）。 */
	srcReleases?: Array<{ x: number; y: number; r: number }>;
	dstReleases?: Array<{ x: number; y: number; r: number }>;
	/** 出线候选点（调用方已按真实焊盘轮廓验证过引出段）：迷宫从这些点
	 * 起步，绕开被邻盘净距圈封死的焊盘中心。引出段图元由调用方拼接。 */
	srcStubs?: Array<{ x: number; y: number }>;
	dstStubs?: Array<{ x: number; y: number }>;
	/** 起点/终点焊盘允许起步的层（顶层 SMD 只能 [1]，TH/多层 [1,2]）。
	 * 缺省 = 全部层（旧行为）。⚠ 绝不能对顶层 SMD 缺省：A* 会直接从
	 * 顶层焊盘中心起步走底层，产出一条没有过孔的底层走线（EDA 判定断网）。 */
	srcPadLayers?: number[];
	dstPadLayers?: number[];
	/** 障碍（半径已含线宽一半 + 间距）；soft 忽略 */
	obstacles: RBObstacle[];
	net: string;
	/** 参与布线的层（如 [1, 2]） */
	layers: number[];
	/** 初始单元尺寸（mil），默认 8 */
	cell?: number;
	/** 单次过孔的等效代价（mil），默认 200 */
	viaCostMil?: number;
	/** 网格单元上限（每层），默认 400k */
	maxCells?: number;
	/** 过孔相对走线额外占用的半径（mil）：过孔铜环 + 焊盘间距比走线宽。
	 * 层间跳变只允许落在「对障碍多留这么多」的格点，避免 A* 在过孔放不下
	 * 的窄缝里换层 → 整条路径因 viaPadClear 被否决。0 = 不额外约束。 */
	viaExtraMil?: number;
	/** 焊盘类障碍的过孔额外净空（焊盘障碍本身已含 pad 间距，口径不同） */
	viaExtraPadMil?: number;
	/** 是否允许过孔落在焊盘上（默认 false：同网焊盘也要挡层间跳变） */
	viaInPad?: boolean;
	/** 协商拥挤：可穿越但计罚分的障碍（异网已布铜皮）。不进占据图，
	 * 按网累加到罚分图；A* 求「穿越代价最小」的路径，调用方据此决定撕谁。 */
	penaltyObstacles?: RBObstacle[];
	/** 每网单格罚分（mil 当量），缺省 60 */
	penaltyOf?: (net: string) => number;
	budgetMs?: number;
	isCancelled?: () => boolean;
}

export interface GridVia {
	x: number;
	y: number;
}

export type GridPrim = RBPrimitive & { layer: number };

export interface GridRouteResult {
	ok: boolean;
	/** 折线图元（layer 字段有效） */
	prims: GridPrim[];
	vias: GridVia[];
	/** 实际使用的单元尺寸（调试） */
	cell: number;
	/** 失败原因（调试） */
	debug?: string;
	/** 最终路径穿越的罚分障碍所属网络（协商拥挤：调用方撕掉它们才能提交） */
	crossed?: string[];
}

interface Cell {
	x: number;
	y: number;
	layer: number;
}

/** Math.hypot 的整数格差查表缓存（惰性填充，值就是 Math.hypot(|a|,|b|) 本身，
 * 逐位一致；Math.hypot 在 V8 里是热点 builtin，A* 每个邻格要调 2 次）。 */
const HYP_N = 1024;
let hypTab: Float64Array | null = null;
function hyp(a: number, b: number): number {
	if (a < 0) a = -a;
	if (b < 0) b = -b;
	if (a < HYP_N && b < HYP_N) {
		if (hypTab === null) hypTab = new Float64Array(HYP_N * HYP_N).fill(NaN);
		const k = a * HYP_N + b;
		let v = hypTab[k];
		if (v !== v) { v = Math.hypot(a, b); hypTab[k] = v; }
		return v;
	}
	return Math.hypot(a, b);
}

const D8DX = new Int8Array([1, -1, 0, 0, 1, 1, -1, -1]);
const D8DY = new Int8Array([0, 0, 1, -1, 1, -1, 1, -1]);
/** LOS 清距跳采样：清距 c（格）→ 可安全跳过的后续样本数 m（0.4m 取整位移 ≤ c-1，留 1e-6 余量） */
const LOS_JUMP = new Int32Array(256);
for (let c = 1; c < 256; c++) LOS_JUMP[c] = Math.max(0, Math.floor(2.5 * (c - 1) - 1e-6));
const D8W = new Float64Array([1, 1, 1, 1, Math.SQRT2, Math.SQRT2, Math.SQRT2, Math.SQRT2]);

/** 最小堆（二叉堆，按 f 值）。typed array + 空穴下沉：比较序列与逐次交换版
 * 完全一致（同键入堆顺序 → 同弹出顺序），只省掉 push/pop 与交换的开销。 */
class MinHeap {
	private keys = new Float64Array(4096);
	private vals = new Int32Array(4096);
	private n = 0;

	get size(): number { return this.n; }

	clear(): void { this.n = 0; }

	push(key: number, val: number): void {
		let n = this.n;
		if (n === this.keys.length) {
			const nk = new Float64Array(n * 2); nk.set(this.keys); this.keys = nk;
			const nv = new Int32Array(n * 2); nv.set(this.vals); this.vals = nv;
		}
		const keys = this.keys, vals = this.vals;
		this.n = n + 1;
		while (n > 0) {
			const p = (n - 1) >> 1;
			if (keys[p] <= key) break;
			keys[n] = keys[p];
			vals[n] = vals[p];
			n = p;
		}
		keys[n] = key;
		vals[n] = val;
	}

	pop(): number {
		const keys = this.keys, vals = this.vals;
		const top = vals[0];
		const n = --this.n;
		if (n > 0) {
			const key = keys[n], val = vals[n];
			let i = 0;
			for (;;) {
				const l = i * 2 + 1;
				if (l >= n) break;
				const r = l + 1;
				let m = i, mk = key;
				if (keys[l] < mk) { m = l; mk = keys[l]; }
				if (r < n && keys[r] < mk) { m = r; mk = keys[r]; }
				if (m === i) break;
				keys[i] = keys[m];
				vals[i] = vals[m];
				i = m;
			}
			keys[i] = key;
			vals[i] = val;
		}
		return top;
	}
}

/**
 * 每次调用都要的大块临时数组（占据图、g/父链/闭集、Lazy Theta 缓存……最多
 * 每层 1.2M 格）。模块级池复用，避免每次分配 + 填充 + GC。
 * 不变量：搜索类数组（gScore/cameFrom/closed/lazyParent/lazyG/goalMark）在
 * 两次调用之间始终是「初始值」——搜索期间记录被写过的格（touched），调用结束
 * 只还原这些格；占据类数组（blocked/viaBlocked/penalty/stamp）每次按 total
 * 清零。异常退出 → dirty，下次整体重填。
 */
class GridScratch {
	cap = 0;
	busy = false;
	dirty = false;
	searching = false;
	nt = 0;
	goalIds: number[] = [];
	blocked = new Uint8Array(0);
	viaBlocked = new Uint8Array(0);
	penalty = new Float32Array(0);
	stamp = new Int32Array(0);
	gScore = new Float64Array(0);
	lazyG = new Float64Array(0);
	cameFrom = new Int32Array(0);
	lazyParent = new Int32Array(0);
	closed = new Uint8Array(0);
	goalMark = new Uint8Array(0);
	touched = new Int32Array(0);
	clr = new Uint8Array(0);
	vis = new Uint8Array(0);
	qa = new Int32Array(0);
	qb = new Int32Array(0);
	heap = new MinHeap();

	ensure(total: number): void {
		if (total <= this.cap && !this.dirty) return;
		if (total > this.cap) {
			const cap = Math.max(total, Math.ceil(this.cap * 1.25));
			this.cap = cap;
			this.blocked = new Uint8Array(cap);
			this.viaBlocked = new Uint8Array(0);
			this.penalty = new Float32Array(0);
			this.stamp = new Int32Array(0);
			this.gScore = new Float64Array(cap);
			this.lazyG = new Float64Array(cap);
			this.cameFrom = new Int32Array(cap);
			this.lazyParent = new Int32Array(cap);
			this.closed = new Uint8Array(cap);
			this.goalMark = new Uint8Array(cap);
			this.touched = new Int32Array(cap);
			this.clr = new Uint8Array(0);
			this.vis = new Uint8Array(0);
			this.qa = new Int32Array(0);
			this.qb = new Int32Array(0);
		}
		this.gScore.fill(Infinity); this.lazyG.fill(Infinity);
		this.cameFrom.fill(-1); this.lazyParent.fill(-1);
		this.closed.fill(0); this.goalMark.fill(0);
		this.dirty = false;
		this.searching = false;
	}

	/** 取清零后的 total 长度占据类数组（按需分配） */
	zeroed<T extends Uint8Array | Float32Array | Int32Array>(
		key: 'viaBlocked' | 'penalty' | 'stamp', total: number, ctor: new (n: number) => T,
	): T {
		let arr = this[key] as unknown as T;
		if (arr.length < total) { arr = new ctor(this.cap); (this as unknown as Record<string, unknown>)[key] = arr; }
		else arr.fill(0, 0, total);
		return arr;
	}

	/** LOS 清距用：按 total 取（内容由调用方逐层完整覆写，无需清零） */
	clearance(total: number): Uint8Array {
		if (this.clr.length < total) this.clr = new Uint8Array(this.cap);
		return this.clr;
	}

	/** 可达性 BFS 用：vis 清零，队列备好 */
	bfsBuffers(total: number): void {
		if (this.vis.length < total) { this.vis = new Uint8Array(this.cap); this.qa = new Int32Array(this.cap); this.qb = new Int32Array(this.cap); }
		else this.vis.fill(0, 0, total);
	}

	/** 调用结束：只还原搜索期间写过的格 */
	finish(): void {
		if (this.searching) {
			const t = this.touched, n = this.nt;
			if (n * 4 > this.cap) {
				this.gScore.fill(Infinity); this.lazyG.fill(Infinity);
				this.cameFrom.fill(-1); this.lazyParent.fill(-1); this.closed.fill(0);
			} else {
				const g = this.gScore, lg = this.lazyG, cf = this.cameFrom, lp = this.lazyParent, cl = this.closed;
				for (let k = 0; k < n; k++) {
					const id = t[k];
					g[id] = Infinity; lg[id] = Infinity; cf[id] = -1; lp[id] = -1; cl[id] = 0;
				}
			}
			this.searching = false;
		}
		for (const id of this.goalIds) this.goalMark[id] = 0;
		this.goalIds.length = 0;
	}
}
const POOL = new GridScratch();

/**
 * 网格求解统计（离线基准 / 现场日志共用）：
 * calls=调用次数，ok=成功，ms=总耗时，cells=累计格数（nx·ny·层），
 * maxCellsSeen=单次最大格数，capped=因越过 maxCells 被迫放大 cell 的次数。
 */
export const gridStats = {
	calls: 0, ok: 0, ms: 0, cells: 0, maxCellsSeen: 0, capped: 0,
	rasterMs: 0, astarMs: 0,
	byCell: {} as Record<number, number>,
};
export function gridStatsReset(): void {
	gridStats.calls = 0; gridStats.ok = 0; gridStats.ms = 0; gridStats.cells = 0;
	gridStats.maxCellsSeen = 0; gridStats.capped = 0; gridStats.rasterMs = 0; gridStats.astarMs = 0;
	gridStats.byCell = {};
}

/**
 * 在给定层集合上求解 src→dst 的网格最短路。
 * 失败返回 { ok: false }（调用方回退到其它策略）。
 */
export async function gridRoute(req: GridRouteRequest): Promise<GridRouteResult> {
	const __t0 = Date.now();
	try {
		const res = await gridRouteInner(req);
		gridStats.calls++;
		if (res.ok) gridStats.ok++;
		return res;
	} finally {
		gridStats.ms += Date.now() - __t0;
	}
}

async function gridRouteInner(req: GridRouteRequest): Promise<GridRouteResult> {
	// 求解体内无 await（同步跑完），池不会被交错使用；busy 仅为防御重入
	const sc = POOL.busy ? new GridScratch() : POOL;
	sc.busy = true;
	try {
		return gridRouteCore(req, sc);
	} catch (e) {
		sc.dirty = true;
		throw e;
	} finally {
		sc.finish();
		sc.busy = false;
	}
}

function gridRouteCore(req: GridRouteRequest, sc: GridScratch): GridRouteResult {
	const fail: GridRouteResult = { ok: false, prims: [], vias: [], cell: req.cell ?? 8 };
	if (!req.layers.length) return fail;
	const { src, dst } = req;
	const span = Math.hypot(dst.x - src.x, dst.y - src.y);
	// 走廊外扩：留出绕行余量（最短直线距离的 60% + 150mil）
	const margin = span * 0.6 + 150;
	const x0 = Math.floor(Math.min(src.x, dst.x) - margin);
	const x1 = Math.ceil(Math.max(src.x, dst.x) + margin);
	const y0 = Math.floor(Math.min(src.y, dst.y) - margin);
	const y1 = Math.ceil(Math.max(src.y, dst.y) + margin);
	// 单元尺寸自适应：保证每层网格 ≤ maxCells。
	// 400k 上限会把长边（跨板连接器网）逼到 cell≈7.3 —— 0.5mm 排的
	// 15.98mil 逃逸车道在粗网格里无法表示（case9 LCD 墙的直接死因）。
	// 1.2M 单元 ≈ 3.2MB 瞬态内存，cell=4 可覆盖 ~1500mil 跨度。
	const maxCells = req.maxCells ?? 1_200_000;
	// 单元尺寸自适应：保证每层网格 ≤ maxCells
	let cell = Math.max(4, req.cell ?? 8);
	let grew = false;
	for (;;) {
		const nx = Math.ceil((x1 - x0) / cell) + 1;
		const ny = Math.ceil((y1 - y0) / cell) + 1;
		if (nx * ny <= maxCells) break;
		cell *= 1.35;
		grew = true;
		if (cell > 60) return fail;
	}
	const nx = Math.ceil((x1 - x0) / cell) + 1;
	const ny = Math.ceil((y1 - y0) / cell) + 1;
	const nLayers = req.layers.length;
	const stride = nx * ny;
	const total = stride * nLayers;
	gridStats.cells += total;
	if (total > gridStats.maxCellsSeen) gridStats.maxCellsSeen = total;
	gridStats.byCell[cell] = (gridStats.byCell[cell] ?? 0) + 1;
	if (grew) gridStats.capped++;
	const deadline = Date.now() + (req.budgetMs ?? 4000);
	const cancelled = (): boolean => {
		try { return req.isCancelled?.() === true; } catch { return false; }
	};

	// ── 光栅化 ──
	const __tRaster = Date.now();
	// 半单元外扩：保证相邻自由格中心之间的线段不切进障碍圆
	const inflate = cell * 0.5;
	sc.ensure(total);
	sc.blocked.fill(0, 0, total);
	const blocked = sc.blocked;
	// 过孔落点网格：比走线多要求 viaExtraMil 的净空。只有层间跳变的格点
	// 需要它（见 A* 的 via 边），普通走线不看。
	const viaExtra = Math.max(0, req.viaExtraMil ?? 0);
	const viaBlocked = viaExtra > 0 ? sc.zeroed('viaBlocked', total, Uint8Array) : null;
	const layerIndex = new Map<number, number>();
	req.layers.forEach((l, i) => layerIndex.set(l, i));

	const markCircle = (lx: number, ly: number, r: number, li: number, extraOverride?: number, viaOnly = false): void => {
		const rr = r + inflate;
		const i0 = Math.max(0, Math.floor((lx - rr - x0) / cell));
		const i1 = Math.min(nx - 1, Math.ceil((lx + rr - x0) / cell));
		const j0 = Math.max(0, Math.floor((ly - rr - y0) / cell));
		const j1 = Math.min(ny - 1, Math.ceil((ly + rr - y0) / cell));
		const base = li * stride;
		if (!viaOnly) {
			const rr2 = rr * rr;
			for (let i = i0; i <= i1; i++) {
				const px = x0 + i * cell;
				const dx = px - lx, dx2 = dx * dx;
				for (let j = j0; j <= j1; j++) {
					const py = y0 + j * cell;
					const dy = py - ly;
					if (dx2 + dy * dy <= rr2) blocked[base + j * nx + i] = 1;
				}
			}
		}
		if (viaBlocked) {
			const ex = extraOverride ?? viaExtra;
			if (ex <= 0) return;
			// 注意：这里**不加** inflate。半单元外扩是为「相邻格心之间的线段」
			// 留的余量，而过孔是格心上的点；用e.r+ex 才是与 viaPadClear /
			// viaObstacleClear 同口径的净距，否则 0.5mm 排外的合法槽位会被
			// 多判 4~6mil 而全灭（实测网格抢救完成数从 20+ 掉到个位数）。
			const vr = r + ex;
			const vr2 = vr * vr;
			const vi0 = Math.max(0, Math.floor((lx - vr - x0) / cell));
			const vi1 = Math.min(nx - 1, Math.ceil((lx + vr - x0) / cell));
			const vj0 = Math.max(0, Math.floor((ly - vr - y0) / cell));
			const vj1 = Math.min(ny - 1, Math.ceil((ly + vr - y0) / cell));
			for (let i = vi0; i <= vi1; i++) {
				const px = x0 + i * cell;
				const dx = px - lx, dx2 = dx * dx;
				for (let j = vj0; j <= vj1; j++) {
					const py = y0 + j * cell;
					const dy = py - ly;
					if (dx2 + dy * dy <= vr2) viaBlocked[base + j * nx + i] = 1;
				}
			}
		}
	};

	// 窗口外障碍的纯跳过：圆/胶囊的包围盒（外扩到最大影响半径 + 1 格 + 1mil 余量）
	// 与网格范围不相交时，markCircle 的任何一个采样圆都不可能标到格子（格心
	// 都在窗口内），整段逐格循环可省；`marked` 计数口径不变（仍按过滤后的障碍计）。
	const gx1 = x0 + (nx - 1) * cell, gy1 = y0 + (ny - 1) * cell;
	const reachExtra = inflate + Math.max(0, viaExtra, req.viaExtraPadMil ?? 0) + cell + 1;
	const missesWindow = (o: RBObstacle): boolean => {
		const R = o.r + reachExtra;
		let ax = o.x, bx = o.x, ay = o.y, by = o.y;
		if (o.kind !== 'circle') {
			ax = Math.min(o.x, o.x2!); bx = Math.max(o.x, o.x2!);
			ay = Math.min(o.y, o.y2!); by = Math.max(o.y, o.y2!);
		}
		return bx + R < x0 || ax - R > gx1 || by + R < y0 || ay - R > gy1;
	};
	const nLay = req.layers.length;

	let marked = 0;
	for (const o of req.obstacles) {
		if (o.soft) continue;
		const sameNet = !!(o.net && o.net === req.net);
		// 同网走线不挡路；同网焊盘在 via-in-pad 关闭时必须挡「层间跳变」
		//（过孔不能落在自家焊盘铜皮上，否则 viaPadClear 判死整条边）。
		const sameNetPad = sameNet && !!o.group?.startsWith('pad:');
		if (sameNet && !(sameNetPad && req.viaInPad !== true)) continue;
		const viaOnly = sameNetPad;
		let liLo = 0, liHi = nLay - 1;
		if (o.layer !== undefined) {
			const li = layerIndex.get(o.layer);
			if (li === undefined) continue;
			liLo = liHi = li;
		}
		if (!missesWindow(o)) {
			const ex = o.group?.startsWith('pad:') ? req.viaExtraPadMil : undefined;
			for (let li = liLo; li <= liHi; li++) {
				if (o.kind === 'circle') {
					markCircle(o.x, o.y, o.r, li, ex, viaOnly);
				} else {
					// 胶囊：沿线采样成圆
					const len = Math.hypot(o.x2! - o.x, o.y2! - o.y);
					const steps = Math.max(1, Math.ceil(len / (cell * 0.75)));
					for (let s = 0; s <= steps; s++) {
						const t = s / steps;
						markCircle(o.x + (o.x2! - o.x) * t, o.y + (o.y2! - o.y) * t, o.r, li, ex, viaOnly);
					}
				}
			}
		}
		if (++marked > 20000) break;   // 障碍过多保护：兜底求解不值得无限光栅化
	}

	// ── 罚分图（协商拥挤）──
	// 异网已布铜皮不进占据图，按「每网每格计一次」累加罚分：A* 可以穿过，
	// 但要付代价 —— 求的是「撕掉谁最便宜」，而不是「能不能过」。
	const penObs = req.penaltyObstacles ?? [];
	const penalty = penObs.length ? sc.zeroed('penalty', total, Float32Array) : null;
	if (penalty) {
		const stamp = sc.zeroed('stamp', total, Int32Array);   // 同网多圆重叠只计一次
		const netIds = new Map<string, number>();
		const penOf = req.penaltyOf ?? (() => 60);
		const markPen = (lx: number, ly: number, r: number, li: number, id: number, val: number): void => {
			const rr = r + inflate;
			const i0 = Math.max(0, Math.floor((lx - rr - x0) / cell));
			const i1 = Math.min(nx - 1, Math.ceil((lx + rr - x0) / cell));
			const j0 = Math.max(0, Math.floor((ly - rr - y0) / cell));
			const j1 = Math.min(ny - 1, Math.ceil((ly + rr - y0) / cell));
			if (i0 > i1 || j0 > j1) return;
			const base = li * stride;
			const rr2 = rr * rr;
			for (let i = i0; i <= i1; i++) {
				const dx = x0 + i * cell - lx, dx2 = dx * dx;
				for (let j = j0; j <= j1; j++) {
					const dy = y0 + j * cell - ly;
					if (dx2 + dy * dy > rr2) continue;
					const k = base + j * nx + i;
					if (stamp[k] === id) continue;
					stamp[k] = id;
					penalty[k] += val;
				}
			}
		};
		for (const o of penObs) {
			if (o.soft || !o.net || o.net === req.net) continue;
			let id = netIds.get(o.net);
			if (id === undefined) { id = netIds.size + 1; netIds.set(o.net, id); }
			const val = penOf(o.net);
			const lis: number[] = [];
			if (o.layer === undefined) lis.push(...req.layers.map((_, i) => i));
			else {
				const li = layerIndex.get(o.layer);
				if (li !== undefined) lis.push(li);
			}
			for (const li of lis) {
				if (o.kind === 'circle') {
					markPen(o.x, o.y, o.r, li, id, val);
				} else {
					const len = Math.hypot(o.x2! - o.x, o.y2! - o.y);
					const steps = Math.max(1, Math.ceil(len / (cell * 0.75)));
					for (let s = 0; s <= steps; s++) {
						const t = s / steps;
						markPen(o.x + (o.x2! - o.x) * t, o.y + (o.y2! - o.y) * t, o.r, li, id, val);
					}
				}
			}
		}
	}

	// ── 端点松绑：起点/终点焊盘自身铜皮内一律视为可通行 ──
	// 这是网格兜底相对可视图的关键优势：邻盘净距圈覆盖焊盘中心时，
	// 可视图从中心出发无切点可用，网格仍能从焊盘内部任意格起步。
	const releaseDisc = (px: number, py: number, rad: number, layerIds: number[]): void => {
		for (const layerId of layerIds) {
			const li = layerIndex.get(layerId);
			if (li === undefined) continue;
			const base = li * stride;
			const i0 = Math.max(0, Math.floor((px - rad - x0) / cell));
			const i1 = Math.min(nx - 1, Math.ceil((px + rad - x0) / cell));
			const j0 = Math.max(0, Math.floor((py - rad - y0) / cell));
			const j1 = Math.min(ny - 1, Math.ceil((py + rad - y0) / cell));
			for (let i = i0; i <= i1; i++) {
				const dx = x0 + i * cell - px;
				for (let j = j0; j <= j1; j++) {
					const dy = y0 + j * cell - py;
					if (dx * dx + dy * dy <= rad * rad) blocked[base + j * nx + i] = 0;
				}
			}
		}
	};
	const release = (pad: { x: number; y: number; r: number; viaKeepoutR?: number } | undefined, layerIds: number[], hasShapeReleases = false): void => {
		if (!pad) return;
		// 有焊盘形状解禁圆时不再用 bodyRadius+cell 的大圆：长条盘会多解禁
		// 侧向 ~8mil，A* 会走合法铜皮外的斜线，最后被 segClear 判死。
		if (!hasShapeReleases) releaseDisc(pad.x, pad.y, pad.r + cell, layerIds);
		// 焊盘禁孔圈：via-in-pad 关闭时，层间跳变不能落在焊盘铜皮附近
		//（A* 默认会在起点格直接换层 → 过孔压在本焊盘上，viaPadClear 判死整条边）
		if (viaBlocked && pad.viaKeepoutR && pad.viaKeepoutR > 0) {
			const vk = pad.viaKeepoutR;
			for (let li = 0; li < nLayers; li++) {
				const base = li * stride;
				const i0 = Math.max(0, Math.floor((pad.x - vk - x0) / cell));
				const i1 = Math.min(nx - 1, Math.ceil((pad.x + vk - x0) / cell));
				const j0 = Math.max(0, Math.floor((pad.y - vk - y0) / cell));
				const j1 = Math.min(ny - 1, Math.ceil((pad.y + vk - y0) / cell));
				for (let i = i0; i <= i1; i++) {
					const dx = x0 + i * cell - pad.x;
					for (let j = j0; j <= j1; j++) {
						const dy = y0 + j * cell - pad.y;
						if (dx * dx + dy * dy <= vk * vk) viaBlocked[base + j * nx + i] = 1;
					}
				}
			}
		}
	};
	const srcLayers = (req.srcPadLayers?.length ? req.srcPadLayers : req.layers)
		.filter(l => layerIndex.has(l));
	const dstLayers = (req.dstPadLayers?.length ? req.dstPadLayers : req.layers)
		.filter(l => layerIndex.has(l));
	release(req.srcPad, srcLayers, (req.srcReleases?.length ?? 0) > 0);
	release(req.dstPad, dstLayers, (req.dstReleases?.length ?? 0) > 0);
	// 长焊盘按整条铜皮解禁（否则起点被困在邻盘间距圈的口袋里）
	for (const p of req.srcReleases ?? []) releaseDisc(p.x, p.y, p.r, srcLayers);
	for (const p of req.dstReleases ?? []) releaseDisc(p.x, p.y, p.r, dstLayers);

	// 出线候选点松绑：候选点周围一格视为可通行（引出段已由调用方验证）
	const releasePoint = (p: { x: number; y: number }, layerIds: number[]): void => {
		const r = cell;
		for (const layerId of layerIds) {
			const li = layerIndex.get(layerId);
			if (li === undefined) continue;
			const base = li * stride;
			const i0 = Math.max(0, Math.floor((p.x - r - x0) / cell));
			const i1 = Math.min(nx - 1, Math.ceil((p.x + r - x0) / cell));
			const j0 = Math.max(0, Math.floor((p.y - r - y0) / cell));
			const j1 = Math.min(ny - 1, Math.ceil((p.y + r - y0) / cell));
			for (let i = i0; i <= i1; i++) {
				const dx = x0 + i * cell - p.x;
				for (let j = j0; j <= j1; j++) {
					const dy = y0 + j * cell - p.y;
					if (dx * dx + dy * dy <= r * r) blocked[base + j * nx + i] = 0;
				}
			}
		}
	};
	const srcStubPts = req.srcStubs ?? [];
	const dstStubPts = req.dstStubs ?? [];
	for (const p of srcStubPts) releasePoint(p, srcLayers);
	for (const p of dstStubPts) releasePoint(p, dstLayers);

	const toCell = (p: { x: number; y: number }): { i: number; j: number } => ({
		i: Math.max(0, Math.min(nx - 1, Math.round((p.x - x0) / cell))),
		j: Math.max(0, Math.min(ny - 1, Math.round((p.y - y0) / cell))),
	});
	const s = toCell(src), d = toCell(dst);
	gridStats.rasterMs += Date.now() - __tRaster;
	const __tAstar = Date.now();

	// ── A* ──
	const gScore = sc.gScore;
	const cameFrom = sc.cameFrom;
	const closed = sc.closed;
	const heap = sc.heap;
	heap.clear();
	const viaCost = req.viaCostMil ?? 200;
	const seedStubByCell = new Map<number, { x: number; y: number } | null>();
	const goalStubByCell = new Map<number, { x: number; y: number }>();

	const idxOf = (i: number, j: number, li: number) => li * stride + j * nx + i;
	const hOf = (i: number, j: number) => hyp(i - d.i, j - d.j) * cell;

	// ── Lazy Theta*（任意角捷径，吸收自 Topola thetastar.rs）──
	// 生成邻格时额外尝试「祖父格→邻格」直线（光栅 LOS，采样步长 0.4 cell），
	// 代价 = g(祖父) + 欧氏距离；弹出时才做 LOS 校验（lazy），失败退回常规
	// A* 父链。最终图元仍逐段过 segClear 真几何校验，合法性口径不变——
	// 收益 = 拐点更少（楼梯在搜索期就消失）、扩展节点更少、拉直后路径更短。
	// __RB_THETA_GRID__ = 0 可关闭（A/B 对照）。
	const anyAngle = (globalThis as unknown as { __RB_THETA_GRID__?: number }).__RB_THETA_GRID__ !== 0;
	const lazyParent = sc.lazyParent;
	const lazyG = sc.lazyG;
	// rasterLos 的「清距跳采样」加速（结果与逐点采样逐位一致）：
	// clr[格] = 该格到同层最近阻挡格的切比雪夫距离（格数，封顶 255；网格外不算阻挡），
	// 即以该格为心、半径 clr-1 的方形内全部空闲。A* 开始后 blocked 不再被改写。
	// 采样序列不变：t = 0..steps，f = t/steps，px = ax + ddx*f，ci = round((px-x0)/cell)。
	// 相邻采样在格坐标上最多移动 0.4 格/轴（steps ≥ dist/(0.4 cell)），故样本 t+m
	// 的取整格与样本 t 的相差 ≤ ceil(0.4m + ε)（floor(x+d)-floor(x) ≤ ceil(d)，
	// ε 为浮点误差，~1e-11 格，LOS_JUMP 里留了 1e-6 余量）。clr=c 时 m 取
	// 使 ceil(0.4m+ε) ≤ c-1 的最大值，t+1..t+m 全落在空闲格，可直接跳过。
	// 被跳过的样本都在两端点格的包围盒内（取整单调、端点是整数格），不会越界，
	// 所以「越界 → false」只需在实际求值的样本上检查，与原循环等价。
	// __RB_GRID_LOSFAST__ = 0：旧的逐点采样；= 2：自检（同时算快慢两版，不一致即抛，
	// 并累加 __RB_GRID_LOSFAST_STATS__ {cmp, bad}），且立即建清距表。
	const losMode = (globalThis as unknown as { __RB_GRID_LOSFAST__?: number }).__RB_GRID_LOSFAST__;
	const losFastOn = losMode !== 0;
	const losCheck = losMode === 2;
	const clrReady = new Uint8Array(nLayers);
	let clr: Uint8Array | null = null;
	let losWork = 0;
	const buildClr = (li: number): void => {
		if (!clr) clr = sc.clearance(total);
		const c = clr;
		const base = li * stride;
		// 前向：左、上、左上、右上；后向：右、下、右下、左下（切比雪夫 8 邻两遍扫描）
		for (let j = 0; j < ny; j++) {
			const row = base + j * nx;
			for (let i = 0; i < nx; i++) {
				const id = row + i;
				if (blocked[id]) { c[id] = 0; continue; }
				let v = 255;
				if (i > 0 && c[id - 1] + 1 < v) v = c[id - 1] + 1;
				if (j > 0) {
					const u = id - nx;
					if (c[u] + 1 < v) v = c[u] + 1;
					if (i > 0 && c[u - 1] + 1 < v) v = c[u - 1] + 1;
					if (i < nx - 1 && c[u + 1] + 1 < v) v = c[u + 1] + 1;
				}
				c[id] = v;
			}
		}
		for (let j = ny - 1; j >= 0; j--) {
			const row = base + j * nx;
			for (let i = nx - 1; i >= 0; i--) {
				const id = row + i;
				let v = c[id];
				if (v === 0) continue;
				if (i < nx - 1 && c[id + 1] + 1 < v) v = c[id + 1] + 1;
				if (j < ny - 1) {
					const u = id + nx;
					if (c[u] + 1 < v) v = c[u] + 1;
					if (i < nx - 1 && c[u + 1] + 1 < v) v = c[u + 1] + 1;
					if (i > 0 && c[u - 1] + 1 < v) v = c[u - 1] + 1;
				}
				c[id] = v;
			}
		}
		clrReady[li] = 1;
	};
	const losPlain = (base: number, ax: number, ay: number, ddx: number, ddy: number, steps: number): boolean => {
		for (let t = 0; t <= steps; t++) {
			const f = t / steps;
			const px = ax + ddx * f, py = ay + ddy * f;
			const ci = Math.round((px - x0) / cell);
			const cj = Math.round((py - y0) / cell);
			if (ci < 0 || cj < 0 || ci >= nx || cj >= ny) return false;
			if (blocked[base + cj * nx + ci]) return false;
		}
		return true;
	};
	const losSkip = (c: Uint8Array, base: number, ax: number, ay: number, ddx: number, ddy: number, steps: number): boolean => {
		let t = 0;
		while (t <= steps) {
			const f = t / steps;
			const px = ax + ddx * f, py = ay + ddy * f;
			const ci = Math.round((px - x0) / cell);
			const cj = Math.round((py - y0) / cell);
			if (ci < 0 || cj < 0 || ci >= nx || cj >= ny) return false;
			const cc = c[base + cj * nx + ci];
			if (cc === 0) return false;
			t += LOS_JUMP[cc] + 1;
		}
		return true;
	};
	const rasterLos = (a: number, b: number, li: number): boolean => {
		const base = li * stride;
		const remA = a - base;
		const remB = b - base;
		if (remA < 0 || remA >= stride || remB < 0 || remB >= stride) return false;
		const aj = Math.floor(remA / nx), ai = remA - aj * nx;
		const bj = Math.floor(remB / nx), bi = remB - bj * nx;
		const ax = x0 + ai * cell, ay = y0 + aj * cell;
		const bx = x0 + bi * cell, by = y0 + bj * cell;
		const ddx = bx - ax, ddy = by - ay;
		const dist = Math.hypot(ddx, ddy);
		const steps = Math.max(1, Math.ceil(dist / (cell * 0.4)));
		if (losFastOn) {
			if (!clrReady[li]) {
				// 清距表建一层约等于 ~stride 次采样的开销：累计采样量够了再建
				losWork += steps;
				if (losCheck || losWork * 2 >= stride) buildClr(li);
			}
			if (clrReady[li]) {
				const fast = losSkip(clr!, base, ax, ay, ddx, ddy, steps);
				if (losCheck) {
					const st = (globalThis as unknown as { __RB_GRID_LOSFAST_STATS__?: { cmp: number; bad: number } }).__RB_GRID_LOSFAST_STATS__;
					const slow = losPlain(base, ax, ay, ddx, ddy, steps);
					if (st) { st.cmp++; if (slow !== fast) st.bad++; }
					if (slow !== fast) throw new Error(`LOSFAST mismatch a=${a} b=${b} li=${li} fast=${fast} slow=${slow}`);
					return slow;
				}
				return fast;
			}
		}
		return losPlain(base, ax, ay, ddx, ddy, steps);
	};

	// 起点：焊盘中心格（cost 0）+ 各出线候选点格（cost = 中心到候选点距离，
	// 引出段长度计入路径代价）。多层（TH 焊盘）各层都可起步。
	interface Seed { i: number; j: number; li: number; stub: { x: number; y: number } | null; cost: number }
	const seeds: Seed[] = [];
	const seedBest = new Map<number, Seed>();
	const addSeed = (i: number, j: number, li: number, stub: { x: number; y: number } | null, cost: number): void => {
		const id = idxOf(i, j, li);
		if (blocked[id]) return;
		const prev = seedBest.get(id);
		if (prev && prev.cost <= cost) return;
		seedBest.set(id, { i, j, li, stub, cost });
	};
	for (const layerId of srcLayers) {
		const li = layerIndex.get(layerId)!;
		addSeed(s.i, s.j, li, null, 0);
	}
	for (const p of srcStubPts) {
		const c = toCell(p);
		for (const layerId of srcLayers) {
			const li = layerIndex.get(layerId)!;
			addSeed(c.i, c.j, li, p, Math.hypot(p.x - src.x, p.y - src.y));
		}
	}
	// 终点：目标格 + 各 dst 候选点格（到点后接引入段）
	const goalCost = new Map<number, number>();   // cellIdx → 引入段长
	const isGoal = new Set<number>();
	const addGoal = (i: number, j: number, stub: { x: number; y: number } | null): void => {
		for (const layerId of dstLayers) {
			const li = layerIndex.get(layerId)!;
			const id = idxOf(i, j, li);
			isGoal.add(id);
			sc.goalMark[id] = 1;
			sc.goalIds.push(id);
			goalCost.set(id, stub ? Math.hypot(stub.x - dst.x, stub.y - dst.y) : 0);
			if (stub) goalStubByCell.set(id, stub);
		}
	};
	addGoal(d.i, d.j, null);
	for (const p of dstStubPts) {
		const c = toCell(p);
		addGoal(c.i, c.j, p);
	}
	const touched = sc.touched;
	let nt = 0;
	if (!seedBest.size) {
		const sId = idxOf(s.i, s.j, 0), dId = idxOf(d.i, d.j, 0);
		return { ...fail, cell, debug: `no-seeds startBlocked=${blocked[sId]} goalBlocked=${blocked[dId]} srcLayers=${srcLayers} dstLayers=${dstLayers}` };
	}

	// ── 可达性预检（双向 BFS，与 A* 交错推进）──
	// 目标不可达时 A* 要把整个可达连通域用堆 + Lazy Theta LOS 洪泛一遍才返回
	// 同一个失败；这里用与 A* 邻接规则逐条一致的 BFS 判连通：
	//   同层八邻（目标格未阻挡；斜走不切角）+ 同格跨层（目标格未阻挡，
	//   viaBlocked 在两端都为空）。移动关系对称，所以「起点域含目标」⇔
	//   「目标域含起点」，两端轮流各扩一格，先耗尽的一端即可证明不可达。
	// 只在能证明「不可达」时直接返回失败（A* 自己跑完也是同一个 no-path）；
	// 相遇 → 可达，BFS 停用，A* 照常。为避免给「很快就成功」的搜索白加开销，
	// BFS 在 A* 弹出 BFS_AFTER_POPS 个节点后才启动，之后每个 A* 弹出配 BFS_RATIO
	// 步（被 deadline / 取消打断时 A* 自己的检查会收手，行为不变）。
	// __RB_GRID_PRECHECK__ = 0 可关闭（A/B 对照）。
	// __RB_GRID_PRECHECK__ = 2：测试用「急切」模式（第 1 个弹出起每弹出配 1 步），
	// 让小场景也走完整的 BFS 路径。
	const preMode = (globalThis as unknown as { __RB_GRID_PRECHECK__?: number }).__RB_GRID_PRECHECK__;
	const BFS_AFTER_POPS = preMode === 2 ? 1 : 1500, BFS_RATIO = preMode === 2 ? 1 : 3;
	let bfsLive = preMode !== 0;
	let vis = sc.vis, qa = sc.qa, qb = sc.qb;
	let ah = 0, at = 0, bh = 0, bt = 0;
	// expand 返回新队尾；-1 = 与对端相遇
	const expand = (cur: number, q: Int32Array, tail: number, mine: number, other: number): number => {
		const li = (cur / stride) | 0;
		const rem = cur - li * stride;
		const j = (rem / nx) | 0;
		const i = rem - j * nx;
		const base = li * stride;
		for (let k = 0; k < 8; k++) {
			const dx = D8DX[k], dy = D8DY[k];
			const ni = i + dx, nj = j + dy;
			if (ni < 0 || nj < 0 || ni >= nx || nj >= ny) continue;
			const nid = base + nj * nx + ni;
			if (blocked[nid]) continue;
			if (dx !== 0 && dy !== 0) {
				if (blocked[base + j * nx + ni] || blocked[base + nj * nx + i]) continue;
			}
			const v = vis[nid];
			if (v & mine) continue;
			if (v & other) return -1;
			vis[nid] = v | mine;
			q[tail++] = nid;
		}
		for (let l2 = 0; l2 < nLayers; l2++) {
			if (l2 === li) continue;
			const nid = l2 * stride + rem;
			if (blocked[nid]) continue;
			if (viaBlocked && (viaBlocked[nid] || viaBlocked[cur])) continue;
			const v = vis[nid];
			if (v & mine) continue;
			if (v & other) return -1;
			vis[nid] = v | mine;
			q[tail++] = nid;
		}
		return tail;
	};
	/** BFS 初始化；返回 true = 起点即命中目标（可达，无需再跑） */
	const bfsInit = (): boolean => {
		sc.bfsBuffers(total);
		vis = sc.vis; qa = sc.qa; qb = sc.qb;
		for (const sd of seedBest.values()) {
			const id = idxOf(sd.i, sd.j, sd.li);
			if (!(vis[id] & 1)) { vis[id] |= 1; qa[at++] = id; }
		}
		for (const id of isGoal) {
			if (blocked[id]) continue;
			if (vis[id] & 1) return true;
			if (!(vis[id] & 2)) { vis[id] |= 2; qb[bt++] = id; }
		}
		return false;
	};
	/** 推进至多 steps 轮（每轮两端各扩一格）。-1 = 证明不可达，1 = 相遇，0 = 未决 */
	const bfsAdvance = (steps: number): number => {
		for (let n = 0; n < steps; n++) {
			if (ah >= at || bh >= bt) return -1;
			at = expand(qa[ah++], qa, at, 1, 2);
			if (at < 0) return 1;
			bt = expand(qb[bh++], qb, bt, 2, 1);
			if (bt < 0) return 1;
		}
		return 0;
	};

	// 起步：种子入堆（搜索数组自此被改写 → 登记 touched，结束时还原）
	sc.searching = true;
	for (const sd of seedBest.values()) {
		const id = idxOf(sd.i, sd.j, sd.li);
		touched[nt++] = id;
		gScore[id] = sd.cost;
		seedStubByCell.set(id, sd.stub);
		heap.push(sd.cost + hOf(sd.i, sd.j), id);
	}

	let best = -1;
	let bestCost = Infinity;
	let pops = 0;
	const goalMark = sc.goalMark;
	const dcx = d.i, dcy = d.j;
	while (heap.size) {
		if ((++pops & 255) === 0) {
			if (Date.now() > deadline || cancelled()) break;
		}
		if (bfsLive && pops >= BFS_AFTER_POPS) {
			let r = pops === BFS_AFTER_POPS && bfsInit() ? 1 : 0;
			if (r === 0) r = bfsAdvance(BFS_RATIO);
			if (r < 0) {
				// 一端耗尽：证明不可达，与 A* 耗尽堆后的失败等价
				sc.nt = nt;
				const sId = idxOf(s.i, s.j, 0), dId = idxOf(d.i, d.j, 0);
				return { ...fail, cell, debug: `no-path(unreachable) nx=${nx} ny=${ny} pops=${pops} startBlocked=${blocked[sId]} goalBlocked=${blocked[dId]}` };
			}
			if (r > 0) bfsLive = false;
		}
		const cur = heap.pop();
		if (closed[cur]) continue;
		closed[cur] = 1;
		const li = (cur / stride) | 0;
		const rem = cur - li * stride;
		const j = (rem / nx) | 0;
		const i = rem - j * nx;
		// Lazy Theta*：弹出时才校验捷径 LOS；成立则采纳祖父直连（更少拐点），
		// 失败保持常规 A* 父链。只裁决一次。
		if (anyAngle && lazyParent[cur] >= 0 && lazyG[cur] < gScore[cur] - 1e-9) {
			const p = lazyParent[cur];
			if (((p / stride) | 0) === li && rasterLos(p, cur, li)) {
				gScore[cur] = lazyG[cur];
				cameFrom[cur] = p;
			}
			lazyParent[cur] = -1;
		}
		const g = gScore[cur];
		if (goalMark[cur] === 1) {
			const total = g + (goalCost.get(cur) ?? 0);
			if (total < bestCost) { bestCost = total; best = cur; }
			// 目标格命中：八字邻接的最优性足够，直接收手
			break;
		}
		const base = li * stride;
		// Lazy Theta* 的祖父格在本格的邻格循环内不变（cameFrom[cur] 只在弹出时改；
		// 祖父已在闭集，其 gScore 在本轮不会被写）→ 提到循环外
		const p = anyAngle ? cameFrom[cur] : -1;
		const pRem = p >= 0 ? p - base : -1;
		const hasP = anyAngle && pRem >= 0 && pRem < stride;
		let pj = 0, pi = 0, gp = 0;
		if (hasP) { pj = (pRem / nx) | 0; pi = pRem - pj * nx; gp = gScore[p]; }
		// 同层八邻
		for (let k = 0; k < 8; k++) {
			const dx = D8DX[k], dy = D8DY[k];
			const ni = i + dx, nj = j + dy;
			if (ni < 0 || nj < 0 || ni >= nx || nj >= ny) continue;
			const nid = base + nj * nx + ni;
			if (blocked[nid] || closed[nid]) continue;
			// 斜走禁止切角：两个正交邻格必须都空
			if (dx !== 0 && dy !== 0) {
				if (blocked[base + j * nx + ni] || blocked[base + nj * nx + i]) continue;
			}
			const ng = g + D8W[k] * cell + (penalty ? penalty[nid] : 0);
			if (ng < gScore[nid] - 1e-9) {
				if (gScore[nid] === Infinity && lazyG[nid] === Infinity) touched[nt++] = nid;
				gScore[nid] = ng;
				cameFrom[nid] = cur;
				heap.push(ng + hyp(ni - dcx, nj - dcy) * cell, nid);
			}
			// Lazy Theta* 捷径候选：祖父格直连邻格（同层），弹出时验 LOS
			if (hasP) {
				const cand = gp + hyp(ni - pi, nj - pj) * cell;
				if (cand < gScore[nid] - 1e-9 && cand < lazyG[nid] - 1e-9) {
					if (gScore[nid] === Infinity && lazyG[nid] === Infinity) touched[nt++] = nid;
					lazyG[nid] = cand;
					lazyParent[nid] = p;
					heap.push(cand + hyp(ni - dcx, nj - dcy) * cell, nid);
				}
			}
		}
		// 层间过孔边（同格跨层）
		for (let l2 = 0; l2 < nLayers; l2++) {
			if (l2 === li) continue;
			const nid = l2 * stride + rem;
			if (blocked[nid] || closed[nid]) continue;
			// 过孔比走线粗：两侧层都必须在「过孔净距」网格上为空
			if (viaBlocked && (viaBlocked[nid] || viaBlocked[cur])) continue;
			const ng = g + viaCost + (penalty ? penalty[nid] : 0);
			if (ng < gScore[nid] - 1e-9) {
				if (gScore[nid] === Infinity && lazyG[nid] === Infinity) touched[nt++] = nid;
				gScore[nid] = ng;
				cameFrom[nid] = cur;
				heap.push(ng + hOf(i, j), nid);
			}
		}
	}
	sc.nt = nt;
	if (best < 0) {
		const sId = idxOf(s.i, s.j, 0), dId = idxOf(d.i, d.j, 0);
		return { ...fail, cell, debug: `no-path nx=${nx} ny=${ny} pops=${pops} startBlocked=${blocked[sId]} goalBlocked=${blocked[dId]}` };
	}

	// ── 回溯 ──
	const cells: Cell[] = [];
	let cur = best;
	let seedCell = -1;
	while (cur !== -1) {
		const li = Math.floor(cur / stride);
		const rem = cur - li * stride;
		const j = Math.floor(rem / nx);
		const i = rem - j * nx;
		cells.push({ x: x0 + i * cell, y: y0 + j * cell, layer: req.layers[li] });
		seedCell = cur;
		cur = cameFrom[cur];
	}
	cells.reverse();

	// 出线候选点接入：路径首/尾若是候选点格，把端点吸附到候选点上——
	// 引出段（焊盘中心→候选点）已由调用方按真实焊盘轮廓验证，最后拼接
	const startStub = seedCell >= 0 ? (seedStubByCell.get(seedCell) ?? null) : null;
	const goalStub = goalStubByCell.get(best) ?? null;
	if (startStub) {
		cells[0] = { x: startStub.x, y: startStub.y, layer: cells[0].layer };
	}
	if (goalStub && cells.length) {
		cells[cells.length - 1] = { x: goalStub.x, y: goalStub.y, layer: cells[cells.length - 1].layer };
	}

	// ── 化简：去共线、合并层间跳变 ──
	const pts: Cell[] = [];
	for (const c of cells) {
		const last = pts[pts.length - 1];
		if (last && last.x === c.x && last.y === c.y && last.layer === c.layer) continue;
		pts.push(c);
	}
	const simplified: Cell[] = [pts[0]];
	for (let k = 1; k < pts.length - 1; k++) {
		const a = simplified[simplified.length - 1], b = pts[k], c = pts[k + 1];
		const sameLayer = a.layer === b.layer && b.layer === c.layer;
		if (sameLayer) {
			const abx = b.x - a.x, aby = b.y - a.y;
			const bcx = c.x - b.x, bcy = c.y - b.y;
			const cross = abx * bcy - aby * bcx;
			const dot = abx * bcx + aby * bcy;
			if (Math.abs(cross) < 1e-6 && dot > 0) continue;   // 共线取直
		}
		simplified.push(b);
	}
	simplified.push(pts[pts.length - 1]);

	// ── 视线拉直（string pull）──
	// 8 连通 A* 的楼梯折线是接缝/锐角的头号来源。同层段内做「最长可视
	// 跳跃」：只要直线段到障碍保持净距（r + 0.4），就一步跳过去。
	// via 点（层切换处）保持不动，只拉直层内段。
	const segClear = (ax: number, ay: number, bx: number, by: number, layer: number): boolean => {
		for (const o of req.obstacles) {
			if (o.soft) continue;
			if (o.net && o.net === req.net) continue;
			if (o.layer !== undefined && o.layer !== layer) continue;
			// 阈值 = 障碍半径（已含线宽+间距）+ 0.4mil 余量
			const rr = o.r + 0.4;
			if (o.kind === 'circle') {
				if (pointSegDist(o.x, o.y, ax, ay, bx, by) < rr) return false;
				continue;
			}
			// 胶囊：沿轴采样（上限 96 点，长胶囊按等距覆盖）
			const ox1 = o.x1 ?? o.x, oy1 = o.y1 ?? o.y;
			const ox2 = o.x2 ?? ox1, oy2 = o.y2 ?? oy1;
			const len = Math.hypot(ox2 - ox1, oy2 - oy1);
			const steps = Math.min(96, Math.max(1, Math.ceil(len / (cell * 0.75))));
			for (let s = 0; s <= steps; s++) {
				const t = s / steps;
				if (pointSegDist(ox1 + (ox2 - ox1) * t, oy1 + (oy2 - oy1) * t, ax, ay, bx, by) < rr) return false;
			}
		}
		return true;
	};
	const pulled: Cell[] = [];
	{
		let i = 0;
		while (i < simplified.length) {
			pulled.push(simplified[i]);
			let run = i + 1;
			while (run < simplified.length && simplified[run].layer === simplified[i].layer) run++;
			// [i, run) 同层：从段尾向回找最长可视跳跃
			let j = run - 1;
			while (j > i + 1 && !segClear(simplified[i].x, simplified[i].y, simplified[j].x, simplified[j].y, simplified[i].layer)) j--;
			i = Math.max(j, i + 1);
		}
	}

	// ── 生成图元与过孔 ──
	const prims: GridPrim[] = [];
	const vias: GridVia[] = [];
	for (let k = 1; k < pulled.length; k++) {
		const a = pulled[k - 1], b = pulled[k];
		if (a.layer === b.layer) {
			if (a.x !== b.x || a.y !== b.y) {
				prims.push({ kind: 'line', layer: a.layer, x1: a.x, y1: a.y, x2: b.x, y2: b.y });
			}
		} else {
			vias.push({ x: a.x, y: a.y });
		}
	}
	// 首尾并入端点：非候选点端做「连接线净距修剪」——从焊盘中心逐格前探
	// 找可直连的格（邻盘净距圈压进本盘释放区时，格心合法 ≠ 线段合法），
	// 找不到整条边放弃；候选点端由已验证的引出段接管，跳过本逻辑。
	if (pulled.length) {
		if (!startStub) {
			const firstLayer = pulled[0].layer;
			let startIdx = -1;
			for (let k = 0; k < pulled.length; k++) {
				if (pulled[k].layer !== firstLayer) break;
				if (segClear(src.x, src.y, pulled[k].x, pulled[k].y, firstLayer)) { startIdx = k; break; }
			}
			if (startIdx < 0) return { ...fail, cell, debug: 'start-unreachable(segClear from pad)' };
			pulled.splice(0, startIdx);
		}
		if (!goalStub) {
			const lastLayer = pulled[pulled.length - 1].layer;
			let endIdx = -1;
			for (let k = pulled.length - 1; k >= 0; k--) {
				if (pulled[k].layer !== lastLayer) break;
				if (segClear(dst.x, dst.y, pulled[k].x, pulled[k].y, lastLayer)) { endIdx = k; break; }
			}
			if (endIdx < 0) return { ...fail, cell, debug: 'end-unreachable(segClear to pad)' };
			pulled.splice(endIdx + 1);
		}

		// 重新生成图元/过孔（起止格被修剪过）
		prims.length = 0;
		vias.length = 0;
		for (let k = 1; k < pulled.length; k++) {
			const a = pulled[k - 1], b = pulled[k];
			if (a.layer === b.layer) {
				if (a.x !== b.x || a.y !== b.y) {
					prims.push({ kind: 'line', layer: a.layer, x1: a.x, y1: a.y, x2: b.x, y2: b.y });
				}
			} else {
				vias.push({ x: a.x, y: a.y });
			}
		}
		const first = pulled[0];
		if (!startStub && Math.hypot(first.x - src.x, first.y - src.y) > 0.5) {
			prims.unshift({ kind: 'line', layer: first.layer, x1: src.x, y1: src.y, x2: first.x, y2: first.y });
		}
		const last = pulled[pulled.length - 1];
		if (!goalStub && Math.hypot(last.x - dst.x, last.y - dst.y) > 0.5) {
			prims.push({ kind: 'line', layer: last.layer, x1: last.x, y1: last.y, x2: dst.x, y2: dst.y });
		}
	}

	// ── 终检：每段直线逐一过净距（含焊盘内段落——邻盘净距圈压进本盘
	// 释放区时，格心合法 ≠ 线段合法）。任何一段不合法，整条边放弃。
	// 出线候选点引出段不在此列（调用方已按真实焊盘轮廓验证）。
	for (const p of prims) {
		if (p.kind !== 'line') continue;
		if (!segClear(p.x1!, p.y1!, p.x2!, p.y2!, p.layer)) return { ...fail, cell, debug: 'validate-line' };
	}
	// 引出段最后拼接（免检：stubFeasible 用真实轮廓 + 区间判定验证过）
	if (startStub) {
		prims.unshift({ kind: 'line', layer: pulled[0]?.layer ?? req.layers[0], x1: src.x, y1: src.y, x2: startStub.x, y2: startStub.y });
	}
	if (goalStub) {
		prims.push({ kind: 'line', layer: pulled[pulled.length - 1]?.layer ?? req.layers[0], x1: goalStub.x, y1: goalStub.y, x2: dst.x, y2: dst.y });
	}

	// 协商拥挤：按**最终图元**统计穿越的罚分网（端点修剪/拉直后才是要提交的路径）
	let crossed: string[] | undefined;
	if (penObs.length) {
		const cs = new Set<string>();
		for (const p of prims) {
			if (p.kind !== 'line') continue;
			const ax = p.x1!, ay = p.y1!, bx = p.x2!, by = p.y2!;
			const minx = Math.min(ax, bx), maxx = Math.max(ax, bx);
			const miny = Math.min(ay, by), maxy = Math.max(ay, by);
			for (const o of penObs) {
				if (o.soft || !o.net || o.net === req.net || cs.has(o.net)) continue;
				if (o.layer !== undefined && o.layer !== p.layer) continue;
				if (o.x + o.r < minx || o.x - o.r > maxx || o.y + o.r < miny || o.y - o.r > maxy) continue;
				if (o.kind === 'circle' && pointSegDist(o.x, o.y, ax, ay, bx, by) < o.r) cs.add(o.net);
			}
		}
		for (const v of vias) {
			for (const o of penObs) {
				if (o.soft || !o.net || o.net === req.net || cs.has(o.net)) continue;
				if (o.kind === 'circle' && Math.hypot(o.x - v.x, o.y - v.y) < o.r + viaExtra) cs.add(o.net);
			}
		}
		crossed = [...cs];
	}
	gridStats.astarMs += Date.now() - __tAstar;
	return { ok: prims.length > 0, prims, vias, cell, crossed };
}

/**
 * 校验网格路径：逐段检查是否侵入障碍（按各层）。
 * 网格本身就是按膨胀障碍生成的，这里做一次最终确认，防止半单元外扩
 * 在斜走时仍留有矢高误差。
 */
export function validateGridPath(
	path: Array<{ x: number; y: number; layer: number }>,
	obstacles: RBObstacle[],
	net: string,
	tol = 0.6,
): boolean {
	for (let k = 1; k < path.length; k++) {
		const a = path[k - 1], b = path[k];
		if (a.layer !== b.layer) continue;
		for (const o of obstacles) {
			if (o.soft) continue;
			if (o.net && o.net === net) continue;
			if (o.layer !== undefined && o.layer !== a.layer) continue;
			const d = o.kind === 'circle'
				? pointSegDist(o.x, o.y, a.x, a.y, b.x, b.y) - o.r
				: pointSegDist(o.x, o.y, a.x, a.y, b.x, b.y) - o.r;
			if (d < -tol) return false;
		}
	}
	return true;
}
