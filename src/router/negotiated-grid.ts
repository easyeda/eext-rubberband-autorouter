/**
 * negotiated-grid.ts —— 全局协商网格收尾（整板 PathFinder 式协商布线）
 *
 * 定位：顺序撕毁重布 / 逐边网格兜底在「引脚被邻网铜皮封死」的局面下
 * 收敛不了 —— 每次只挪一个网，局部动作永远拿不到「几个网同时让位」的解。
 * 这里换成同时求解：
 *
 *  - 整板统一网格（默认 4mil），每铜层一张节点平面，全部用 typed array；
 *  - 参与者 = 调用方给的未布通网（整网重布，残段已撕）。其余已布网是「固定网」：
 *    真实几何（线/过孔）光栅化成禁区戳，参与者可以付罚分穿越；被穿越的固定网
 *    升级为参与者（调用方撕掉它的铜皮，在网格上一并重布），未被碰的网几何不动；
 *  - 参与者之间 PathFinder 协商：节点代价 = 步长 ×（1+历史）×（1+当前拥挤×现时因子），
 *    每轮只重布有冲突的网，现时因子与历史逐轮抬高，直到无冲突或到轮数上限；
 *  - 每个网按 Prim 顺序长成一棵树：多源 A*（源 = 已连通的树节点 + 已连焊盘铜皮内节点）
 *    → 下一个焊盘的铜皮节点；TH 焊盘两层都可起落，SMD 只在本层；
 *  - 净距按「节点中心欧氏距离」构造保证：异网走线↔走线 / 走线↔焊盘 / 走线↔钻孔 /
 *    走线↔过孔 / 过孔↔过孔（含同网孔壁）/ 过孔↔焊盘 / 板框静态障碍。每条规则的
 *    数值由调用方按引擎提交检查（互检 / viaExactBad / viaObstacleClear / viaPadClear）
 *    的口径换算后传入，本模块只额外加 slack 吸收网格折线的「弦垂」（相邻节点间
 *    线段对点状物体的最近距离比端点小 ≤ 步长²/(8D)，4mil 网格 < 0.4mil）。
 *
 * 决策只依赖迭代/扩展计数，不看墙钟（确定性）；isCancelled 只用于中止。
 * 纯模块：不依赖引擎，输入输出均为普通对象。
 */

export interface NGPoint { x: number; y: number }

export interface NGPad {
	x: number;
	y: number;
	net: string;
	/** 铜皮所在层（SMD 单层，TH/多层 = [1,2]） */
	layers: number[];
	/** 铜皮（含钻孔）外接半径上界 */
	extent: number;
	/** 钻孔直径（0 = 无孔） */
	holeD: number;
	/** 点 → 焊盘铜皮的有符号距离（内部为负） */
	dist: (x: number, y: number) => number;
}

export interface NGStatic {
	kind: 'circle' | 'capsule';
	x: number;
	y: number;
	/** 障碍半径（已含线宽一半 + 间距，与引擎障碍同口径） */
	r: number;
	x1?: number; y1?: number; x2?: number; y2?: number;
	/** undefined = 全层 */
	layer?: number;
	net?: string;
	/** 板框：用于连通域判定（板外节点全部封死） */
	board?: boolean;
}

export interface NGSeg { layer: number; x1: number; y1: number; x2: number; y2: number }

export interface NGFixed {
	name: string;
	segs: NGSeg[];
	vias: NGPoint[];
	/** false：不可升级（不在网络表里的网），按静态硬障碍处理 */
	escalatable: boolean;
}

/**
 * 净距规则（中心距 / 中心到铜皮距离，mil）。「阻挡」= 距离 < 规则值 + slack。
 */
export interface NGRules {
	/** 异网走线中心线 ↔ 走线中心线 */
	trkTrk: number;
	/** 走线中心线 ↔ 异网焊盘铜皮 */
	trkPad: number;
	/** 走线中心线 ↔ 异网钻孔中心：holeD/2 + trkHoleExtra */
	trkHoleExtra: number;
	/** 走线中心线 ↔ 异网过孔中心（两方向取大） */
	trkVia: number;
	/** 异网过孔中心距 */
	viaVia: number;
	/** 同网过孔中心距（孔壁间距） */
	viaViaSame: number;
	/** 过孔中心 ↔ 异网焊盘铜皮 */
	viaPad: number;
	/** 过孔中心 ↔ 同网焊盘铜皮（via-in-pad 关闭时） */
	viaPadSame: number;
	/** 过孔中心 ↔ 异网钻孔中心：holeD/2 + viaHoleExtra */
	viaHoleExtra: number;
	/** 走线中心 ↔ 静态障碍：r + staticTrkExtra */
	staticTrkExtra: number;
	/** 过孔中心 ↔ 静态障碍：r + staticViaExtra */
	staticViaExtra: number;
	/** 网格弦垂余量 */
	slack: number;
}

export interface NGRequest {
	cell: number;
	layerCount: number;
	pads: NGPad[];
	/** 全部网络（pads = NGRequest.pads 下标） */
	nets: Array<{ name: string; pads: number[] }>;
	/** 初始参与者（网名） */
	participants: string[];
	fixed: NGFixed[];
	statics: NGStatic[];
	rules: NGRules;
	viaInPad: boolean;
	/** 过孔等效代价（mil），默认 150 */
	viaCost?: number;
	/** 协商轮数上限，默认 50 */
	maxIter?: number;
	/** 单次 A* 弹出上限，默认 1.5M */
	maxExpand?: number;
	/** 全阶段弹出总上限，默认 60M */
	maxTotalExpand?: number;
	/** 穿越固定网禁区的单格罚分（mil），默认 150 */
	fixedPenalty?: number;
	/** 最多升级多少个固定网，默认 24 */
	maxEscalate?: number;
	/** 每层节点上限，默认 1.2M：超出时格距按 0.5mil 递增直到 maxCell，仍超出则放弃 */
	maxNodes?: number;
	/** 自适应格距上限（mil），默认 6 */
	maxCell?: number;
	isCancelled?: () => boolean;
	log?: (msg: string) => void;
	/** 诊断输出（残余冲突点） */
	debug?: boolean;
}

export interface NGChain { layer: number; pts: NGPoint[] }

export interface NGNetResult {
	name: string;
	/** 已排序的同层折线链（相邻链尽量不相接，见 orderChains） */
	chains: NGChain[];
	vias: NGPoint[];
}

export interface NGResult {
	ok: boolean;
	reason?: string;
	routed: NGNetResult[];
	failed: string[];
	/** 被升级且交出了新几何的固定网（调用方撕掉原铜皮）。升级后又失败/被剔的网
	 * 已退回固定网（原铜皮重新占位、其它结果都与它无冲突），不在此列也不在 failed 里 */
	escalated: string[];
	iterations: number;
	converged: boolean;
	expansions: number;
}

const DX = [1, -1, 0, 0, 1, 1, -1, -1];
const DY = [0, 0, 1, -1, 1, -1, 1, -1];

function segDist(px: number, py: number, ax: number, ay: number, bx: number, by: number): number {
	const dx = bx - ax, dy = by - ay;
	const l2 = dx * dx + dy * dy;
	let t = l2 > 0 ? ((px - ax) * dx + (py - ay) * dy) / l2 : 0;
	if (t < 0) t = 0; else if (t > 1) t = 1;
	return Math.hypot(px - (ax + dx * t), py - (ay + dy * t));
}

function segSegDist(ax: number, ay: number, bx: number, by: number, cx: number, cy: number, dx: number, dy: number): number {
	// 相交 → 0；否则四个端点到对边距离的最小值
	const o = (px: number, py: number, qx: number, qy: number, rx: number, ry: number) => (qx - px) * (ry - py) - (qy - py) * (rx - px);
	const d1 = o(ax, ay, bx, by, cx, cy), d2 = o(ax, ay, bx, by, dx, dy);
	const d3 = o(cx, cy, dx, dy, ax, ay), d4 = o(cx, cy, dx, dy, bx, by);
	if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) return 0;
	return Math.min(segDist(ax, ay, cx, cy, dx, dy), segDist(bx, by, cx, cy, dx, dy), segDist(cx, cy, ax, ay, bx, by), segDist(dx, dy, ax, ay, bx, by));
}

/**
 * 线段到焊盘铜皮的最小（有符号）距离。只看离焊盘中心 ≤ extent+reach 的那段
 * （二次方程裁剪）；先 0.5mil 粗采样找最小样本，再在相邻两样本区间内做黄金
 * 分割细化 —— 焊盘铜皮是凸形（RECT/OVAL/SLOT 覆盖矩形/POLYGON 归一矩形），
 * 点到凸集的距离沿线段是凸函数，细化后即精确最小值（粗采样在矩形角附近
 * 可漏 ~0.25mil，细化补上）。
 */
export function padSegMin(pd: NGPad, ax: number, ay: number, bx: number, by: number, reach: number): number {
	const dx = bx - ax, dy = by - ay;
	const a = dx * dx + dy * dy;
	let t0 = 0, t1 = 1;
	if (a > 1e-12) {
		const Rr = pd.extent + reach + 1;
		const fx = ax - pd.x, fy = ay - pd.y;
		const b = 2 * (fx * dx + fy * dy), c = fx * fx + fy * fy - Rr * Rr;
		const disc = b * b - 4 * a * c;
		if (disc < 0) return Infinity;
		const sq = Math.sqrt(disc);
		t0 = Math.max(0, (-b - sq) / (2 * a)); t1 = Math.min(1, (-b + sq) / (2 * a));
		if (t0 > t1) return Infinity;
	} else {
		return pd.dist(ax, ay);
	}
	const f = (t: number) => pd.dist(ax + dx * t, ay + dy * t);
	const n = Math.max(1, Math.ceil((Math.sqrt(a) * (t1 - t0)) / 0.5));
	let best = Infinity, bk = 0;
	for (let k = 0; k <= n; k++) {
		const d = f(t0 + ((t1 - t0) * k) / n);
		if (d < best) { best = d; bk = k; }
	}
	let lo = t0 + ((t1 - t0) * Math.max(0, bk - 1)) / n, hi = t0 + ((t1 - t0) * Math.min(n, bk + 1)) / n;
	const G = (Math.sqrt(5) - 1) / 2;
	let c1 = hi - G * (hi - lo), c2 = lo + G * (hi - lo);
	let f1 = f(c1), f2 = f(c2);
	for (let it = 0; it < 40 && hi - lo > 1e-6; it++) {
		if (f1 <= f2) { hi = c2; c2 = c1; f2 = f1; c1 = hi - G * (hi - lo); f1 = f(c1); }
		else { lo = c1; c1 = c2; f1 = f2; c2 = lo + G * (hi - lo); f2 = f(c2); }
	}
	return Math.min(best, f1, f2);
}

/** 二叉最小堆（同键按入堆顺序弹出 → 确定性） */
class Heap {
	private keys = new Float64Array(1 << 14);
	private vals = new Int32Array(1 << 14);
	private seq = new Float64Array(1 << 14);
	private n = 0;
	private counter = 0;
	get size(): number { return this.n; }
	clear(): void { this.n = 0; this.counter = 0; }
	private less(i: number, j: number): boolean {
		const a = this.keys[i], b = this.keys[j];
		return a < b || (a === b && this.seq[i] < this.seq[j]);
	}
	push(key: number, val: number): void {
		if (this.n === this.keys.length) {
			const m = this.n * 2;
			const k = new Float64Array(m); k.set(this.keys); this.keys = k;
			const v = new Int32Array(m); v.set(this.vals); this.vals = v;
			const s = new Float64Array(m); s.set(this.seq); this.seq = s;
		}
		let i = this.n++;
		this.keys[i] = key; this.vals[i] = val; this.seq[i] = this.counter++;
		while (i > 0) {
			const p = (i - 1) >> 1;
			if (!this.less(i, p)) break;
			this.swap(i, p);
			i = p;
		}
	}
	popVal(): number {
		const top = this.vals[0];
		this.n--;
		if (this.n > 0) {
			this.keys[0] = this.keys[this.n]; this.vals[0] = this.vals[this.n]; this.seq[0] = this.seq[this.n];
			let i = 0;
			for (;;) {
				const l = 2 * i + 1, r = l + 1;
				let m = i;
				if (l < this.n && this.less(l, m)) m = l;
				if (r < this.n && this.less(r, m)) m = r;
				if (m === i) break;
				this.swap(i, m);
				i = m;
			}
		}
		return top;
	}
	private swap(i: number, j: number): void {
		const k = this.keys[i]; this.keys[i] = this.keys[j]; this.keys[j] = k;
		const v = this.vals[i]; this.vals[i] = this.vals[j]; this.vals[j] = v;
		const s = this.seq[i]; this.seq[i] = this.seq[j]; this.seq[j] = s;
	}
}

interface Stamp { trk: Int32Array; via: Int32Array }

interface Route {
	chains: NGChain[];
	vias: NGPoint[];
	/** 本网铜皮占据的走线节点（编码 layerIdx*N + i）：冲突判定用 */
	trkNodes: Int32Array;
	/** 本网过孔所在列 */
	viaCols: Int32Array;
	stamp: Stamp;
}

export function negotiatedGridRoute(req: NGRequest): NGResult {
	let cell = req.cell;
	const L = Math.max(1, Math.min(2, Math.round(req.layerCount)));
	const R = req.rules;
	let slack = R.slack;
	const log = req.log ?? (() => { /* 静默 */ });
	const cancelled = () => !!req.isCancelled?.();
	const maxIter = req.maxIter ?? 50;
	const maxExpand = req.maxExpand ?? 1_500_000;
	const maxTotal = req.maxTotalExpand ?? 60_000_000;
	const viaCost = req.viaCost ?? 150;
	let fixPen = req.fixedPenalty ?? 150;
	const maxEscalate = req.maxEscalate ?? 24;
	const fail = (reason: string): NGResult => ({
		ok: false, reason, routed: [], failed: [...req.participants], escalated: [], iterations: 0, converged: false, expansions: 0,
	});

	// ── 网络编号 ──
	const netId = new Map<string, number>();
	const names: string[] = [];
	const idOf = (name: string): number => {
		let v = netId.get(name);
		if (v === undefined) { v = names.length; netId.set(name, v); names.push(name); }
		return v;
	};
	for (const n of req.nets) idOf(n.name);
	const netByName = new Map(req.nets.map(n => [n.name, n] as const));

	// ── 包围盒：有板框取板框圈心包络，否则焊盘 + 静态障碍外扩 ──
	let minx = Infinity, miny = Infinity, maxx = -Infinity, maxy = -Infinity;
	const grow = (x: number, y: number) => {
		if (x < minx) minx = x; if (x > maxx) maxx = x;
		if (y < miny) miny = y; if (y > maxy) maxy = y;
	};
	const boards = req.statics.filter(o => o.board);
	if (boards.length) {
		for (const o of boards) {
			grow(o.x, o.y);
			if (o.kind === 'capsule') { grow(o.x1 ?? o.x, o.y1 ?? o.y); grow(o.x2 ?? o.x, o.y2 ?? o.y); }
		}
	} else {
		for (const o of req.statics) { grow(o.x - o.r, o.y - o.r); grow(o.x + o.r, o.y + o.r); }
	}
	for (const p of req.pads) { grow(p.x - p.extent - 60, p.y - p.extent - 60); grow(p.x + p.extent + 60, p.y + p.extent + 60); }
	if (boards.length) {
		// 焊盘外扩不越过板框圈心
		let bx0 = Infinity, by0 = Infinity, bx1 = -Infinity, by1 = -Infinity;
		for (const o of boards) {
			for (const [x, y] of [[o.x, o.y], [o.x1 ?? o.x, o.y1 ?? o.y], [o.x2 ?? o.x, o.y2 ?? o.y]]) {
				bx0 = Math.min(bx0, x); by0 = Math.min(by0, y); bx1 = Math.max(bx1, x); by1 = Math.max(by1, y);
			}
		}
		minx = Math.max(minx, bx0); miny = Math.max(miny, by0); maxx = Math.min(maxx, bx1); maxy = Math.min(maxy, by1);
	}
	if (!(maxx > minx && maxy > miny)) return fail('empty bbox');
	// 大板：格距自适应放大（内存 ≈ 60 字节 × 节点 × 层），放大到上限仍超就放弃
	const maxNodes = req.maxNodes ?? 1_200_000;
	while ((Math.floor((maxx - minx) / cell) + 2) * (Math.floor((maxy - miny) / cell) + 2) > maxNodes && cell < (req.maxCell ?? 6)) cell += 0.5;
	const x0 = Math.floor(minx / cell) * cell, y0 = Math.floor(miny / cell) * cell;
	const W = Math.floor((maxx - x0) / cell) + 1, H = Math.floor((maxy - y0) / cell) + 1;
	const N = W * H;
	if (N > maxNodes) return fail(`grid too large ${W}x${H}@${cell}mil`);
	if (cell !== req.cell) log(`[ngrid] 板面较大，格距放大到 ${cell}mil（${W}×${H}）`);
	// 弦垂余量随格距缩放：相邻节点（最长 = 对角 cell·√2）连线对「点状」物体（焊盘角/
	// 线段端点/过孔中心）的最近距离比两端点小 D - √(D² - L²/4)，D 取最小的阻挡规则
	// （最坏情况）。4mil 网格 ≈ 0.36（调用方给的 0.4 已覆盖），6mil ≈ 0.80。
	{
		const dMin = Math.min(R.trkPad, R.trkTrk, R.trkVia, R.trkHoleExtra);
		const half = (cell * Math.SQRT2) / 2;
		const sag = dMin > half ? dMin - Math.sqrt(dMin * dMin - half * half) : half;
		if (sag + 0.01 > slack) slack = sag + 0.01;
	}
	const LN = L * N;
	const X = (i: number) => x0 + (i % W) * cell;
	const Y = (i: number) => y0 + Math.floor(i / W) * cell;
	const forBox = (ax: number, ay: number, bx: number, by: number, fn: (i: number, x: number, y: number) => void) => {
		const ix0 = Math.max(0, Math.ceil((ax - x0) / cell)), ix1 = Math.min(W - 1, Math.floor((bx - x0) / cell));
		const iy0 = Math.max(0, Math.ceil((ay - y0) / cell)), iy1 = Math.min(H - 1, Math.floor((by - y0) / cell));
		for (let iy = iy0; iy <= iy1; iy++) {
			const y = y0 + iy * cell, row = iy * W;
			for (let ix = ix0; ix <= ix1; ix++) fn(row + ix, x0 + ix * cell, y);
		}
	};
	const nodeAt = (x: number, y: number): number => {
		const ix = Math.round((x - x0) / cell), iy = Math.round((y - y0) / cell);
		if (ix < 0 || iy < 0 || ix >= W || iy >= H) return -1;
		return iy * W + ix;
	};

	// ── 静态图：0 通行 / -1 全挡 / k>0 只放行网 k-1 ──
	const sTrk = new Int32Array(LN);
	const sVia = new Int32Array(N);
	const merge = (arr: Int32Array, k: number, owner: number) => {
		const cur = arr[k];
		if (cur === -1) return;
		if (owner === -1) { arr[k] = -1; return; }
		if (cur === 0) arr[k] = owner; else if (cur !== owner) arr[k] = -1;
	};
	for (const pd of req.pads) {
		const own = idOf(pd.net) + 1;
		const tT = R.trkPad + slack, tV = R.viaPad + slack, tVs = R.viaPadSame + slack;
		const hT = pd.holeD > 0 ? pd.holeD / 2 + R.trkHoleExtra + slack : -1;
		const hV = pd.holeD > 0 ? pd.holeD / 2 + R.viaHoleExtra + slack : -1;
		const reach = pd.extent + Math.max(tT, tV, tVs, hT, hV) + cell;
		const lays = pd.layers.filter(l => l >= 1 && l <= L);
		forBox(pd.x - reach, pd.y - reach, pd.x + reach, pd.y + reach, (i, x, y) => {
			const d = pd.dist(x, y);
			if (d < tT) for (const l of lays) merge(sTrk, (l - 1) * N + i, own);
			if (d < tV) merge(sVia, i, own);
			if (!req.viaInPad && d < tVs) merge(sVia, i, -1);
			if (hT > 0) {
				const dc = Math.hypot(x - pd.x, y - pd.y);
				if (dc < hT) for (let l = 0; l < L; l++) merge(sTrk, l * N + i, own);
				if (dc < hV) merge(sVia, i, own);
			}
		});
	}
	const boardMask = boards.length ? new Uint8Array(N) : null;
	for (const o of req.statics) {
		const own = o.net ? idOf(o.net) + 1 : -1;
		const tT = o.r + R.staticTrkExtra + slack, tV = o.r + R.staticViaExtra + slack;
		const m = Math.max(tT, tV);
		const ax = o.kind === 'capsule' ? Math.min(o.x1 ?? o.x, o.x2 ?? o.x) : o.x;
		const bx = o.kind === 'capsule' ? Math.max(o.x1 ?? o.x, o.x2 ?? o.x) : o.x;
		const ay = o.kind === 'capsule' ? Math.min(o.y1 ?? o.y, o.y2 ?? o.y) : o.y;
		const by = o.kind === 'capsule' ? Math.max(o.y1 ?? o.y, o.y2 ?? o.y) : o.y;
		forBox(ax - m, ay - m, bx + m, by + m, (i, x, y) => {
			const d = o.kind === 'capsule'
				? segDist(x, y, o.x1 ?? o.x, o.y1 ?? o.y, o.x2 ?? o.x, o.y2 ?? o.y)
				: Math.hypot(x - o.x, y - o.y);
			if (d < tT) {
				if (o.layer === undefined) { for (let l = 0; l < L; l++) merge(sTrk, l * N + i, own); }
				else if (o.layer >= 1 && o.layer <= L) merge(sTrk, (o.layer - 1) * N + i, own);
				if (o.board && boardMask) boardMask[i] = 1;
			}
			if (d < tV) merge(sVia, i, own);
		});
	}
	// 板外封死：从焊盘中心出发、不穿板框带的 4 连通域之外全部不可用
	if (boardMask) {
		const reach = new Uint8Array(N);
		const q = new Int32Array(N);
		let qh = 0, qt = 0;
		for (const pd of req.pads) {
			const i = nodeAt(pd.x, pd.y);
			if (i >= 0 && !reach[i] && !boardMask[i]) { reach[i] = 1; q[qt++] = i; }
		}
		while (qh < qt) {
			const i = q[qh++];
			const ix = i % W, iy = (i - ix) / W;
			if (ix > 0 && !reach[i - 1] && !boardMask[i - 1]) { reach[i - 1] = 1; q[qt++] = i - 1; }
			if (ix < W - 1 && !reach[i + 1] && !boardMask[i + 1]) { reach[i + 1] = 1; q[qt++] = i + 1; }
			if (iy > 0 && !reach[i - W] && !boardMask[i - W]) { reach[i - W] = 1; q[qt++] = i - W; }
			if (iy < H - 1 && !reach[i + W] && !boardMask[i + W]) { reach[i + W] = 1; q[qt++] = i + W; }
		}
		// 板框带本身也可能有合法节点（带内 = 静态已挡）；只封「既不可达也不在带内」的板外区
		let outside = 0;
		for (let i = 0; i < N; i++) {
			if (reach[i] || boardMask[i]) continue;
			outside++;
			sVia[i] = -1;
			for (let l = 0; l < L; l++) sTrk[l * N + i] = -1;
		}
		if (outside) log(`[ngrid] 板外节点 ${outside} 个封死`);
	}

	// ── 戳（禁区节点集）计算 ──
	const markT = new Uint32Array(LN), markV = new Uint32Array(N);
	let mEpoch = 0;
	const tTT = R.trkTrk + slack, tTV = R.trkVia + slack, tVV = R.viaVia + slack;
	const computeStamp = (segs: NGSeg[], vias: NGPoint[]): Stamp => {
		mEpoch++;
		const trk: number[] = [], via: number[] = [];
		const m = Math.max(tTT, tTV);
		for (const s of segs) {
			if (s.layer < 1 || s.layer > L) continue;
			const base = (s.layer - 1) * N;
			forBox(Math.min(s.x1, s.x2) - m, Math.min(s.y1, s.y2) - m, Math.max(s.x1, s.x2) + m, Math.max(s.y1, s.y2) + m, (i, x, y) => {
				const d = segDist(x, y, s.x1, s.y1, s.x2, s.y2);
				if (d < tTT && markT[base + i] !== mEpoch) { markT[base + i] = mEpoch; trk.push(base + i); }
				if (d < tTV && markV[i] !== mEpoch) { markV[i] = mEpoch; via.push(i); }
			});
		}
		for (const v of vias) {
			const mm = Math.max(tTV, tVV);
			forBox(v.x - mm, v.y - mm, v.x + mm, v.y + mm, (i, x, y) => {
				const d = Math.hypot(x - v.x, y - v.y);
				if (d < tTV) {
					for (let l = 0; l < L; l++) {
						const k = l * N + i;
						if (markT[k] !== mEpoch) { markT[k] = mEpoch; trk.push(k); }
					}
				}
				if (d < tVV && markV[i] !== mEpoch) { markV[i] = mEpoch; via.push(i); }
			});
		}
		return { trk: Int32Array.from(trk), via: Int32Array.from(via) };
	};

	// ── 固定网 ──
	const fTrk = new Uint16Array(LN), fVia = new Uint16Array(N);
	const fOwnT = new Int32Array(LN), fOwnV = new Int32Array(N);
	const fixed = req.fixed;
	const fStamps: Array<Stamp | null> = new Array(fixed.length).fill(null);
	const fSorted: Array<Stamp | null> = new Array(fixed.length).fill(null);
	const fActive = new Uint8Array(fixed.length);
	const own1 = (cur: number, k: number) => (cur === 0 ? k : cur === k ? k : -1);
	for (let f = 0; f < fixed.length; f++) {
		const fx = fixed[f];
		const st = computeStamp(fx.segs, fx.vias);
		if (!fx.escalatable || !netByName.has(fx.name)) {
			// 不可升级：并入静态图（只放行自己网）
			const own = idOf(fx.name) + 1;
			for (const k of st.trk) merge(sTrk, k, own);
			for (const k of st.via) merge(sVia, k, own);
			continue;
		}
		fStamps[f] = st;
		fSorted[f] = { trk: Int32Array.from(st.trk).sort(), via: Int32Array.from(st.via).sort() };
		fActive[f] = 1;
		for (const k of st.trk) { fTrk[k]++; fOwnT[k] = own1(fOwnT[k], f + 1); }
		for (const k of st.via) { fVia[k]++; fOwnV[k] = own1(fOwnV[k], f + 1); }
	}
	const removeFixed = (f: number) => {
		const st = fStamps[f];
		if (!st || !fActive[f]) return;
		fActive[f] = 0;
		for (const k of st.trk) { fTrk[k]--; if (fOwnT[k] === f + 1) fOwnT[k] = 0; }
		for (const k of st.via) { fVia[k]--; if (fOwnV[k] === f + 1) fOwnV[k] = 0; }
	};
	/** 升级失败的网退回固定网：原铜皮重新计入固定占据（只用于收尾，不再升级） */
	const restoreFixed = (f: number) => {
		const st = fStamps[f];
		if (!st || fActive[f]) return;
		fActive[f] = 1;
		for (const k of st.trk) { fTrk[k]++; fOwnT[k] = own1(fOwnT[k], f + 1); }
		for (const k of st.via) { fVia[k]++; fOwnV[k] = own1(fOwnV[k], f + 1); }
	};
	const fixedOfNet = new Map<number, number>();
	for (let f = 0; f < fixed.length; f++) if (fStamps[f]) fixedOfNet.set(idOf(fixed[f].name), f);
	const bsearch = (a: Int32Array, v: number): boolean => {
		let lo = 0, hi = a.length - 1;
		while (lo <= hi) {
			const m = (lo + hi) >> 1;
			if (a[m] === v) return true;
			if (a[m] < v) lo = m + 1; else hi = m - 1;
		}
		return false;
	};
	const fixedOwners = (k: number, isVia: boolean, out: Set<number>) => {
		const o = isVia ? fOwnV[k] : fOwnT[k];
		if (o > 0) { if (fActive[o - 1]) out.add(o - 1); return; }
		for (let f = 0; f < fixed.length; f++) {
			if (!fActive[f]) continue;
			const s = fSorted[f]!;
			if (bsearch(isVia ? s.via : s.trk, k)) out.add(f);
		}
	};

	// ── 参与者状态 ──
	const oTrk = new Uint16Array(LN), oVia = new Uint16Array(N);
	const hTrk = new Float32Array(LN), hVia = new Float32Array(N);
	const routes = new Map<number, Route | null>();
	const applyStamp = (st: Stamp, sgn: number) => {
		for (const k of st.trk) oTrk[k] += sgn;
		for (const k of st.via) oVia[k] += sgn;
	};

	// 接线段（焊盘中心 → 起落节点）对静态物（异网焊盘铜皮/钻孔、静态障碍）的精确
	// 净距：网格只检查了端点节点，放宽阈值的节点离铜皮最多 4mil，中间段要单独查。
	// 对固定网/参与者铜皮的净距由接线段格点的冲突判定近似覆盖，最终由 pullTaut
	// 的逐段精确复核兜底（不过复核的网判失败，不交出去）。
	const joinStaticClear = (pd: NGPad, layer: number, x: number, y: number): boolean => {
		if (Math.hypot(x - pd.x, y - pd.y) < 1e-9) return true;
		const minX = Math.min(pd.x, x), maxX = Math.max(pd.x, x), minY = Math.min(pd.y, y), maxY = Math.max(pd.y, y);
		for (const q of req.pads) {
			if (q.net === pd.net) continue;
			const reach = q.extent + Math.max(R.trkPad, q.holeD > 0 ? q.holeD / 2 + R.trkHoleExtra : 0) + 1;
			if (q.x + reach < minX || q.x - reach > maxX || q.y + reach < minY || q.y - reach > maxY) continue;
			if (q.layers.includes(layer) && padSegMin(q, pd.x, pd.y, x, y, R.trkPad) < R.trkPad) return false;
			if (q.holeD > 0 && segDist(q.x, q.y, pd.x, pd.y, x, y) < q.holeD / 2 + R.trkHoleExtra) return false;
		}
		for (const o of req.statics) {
			if (o.net && o.net === pd.net) continue;
			if (o.layer !== undefined && o.layer !== layer) continue;
			const need = o.r + R.staticTrkExtra;
			const ox0 = o.kind === 'capsule' ? Math.min(o.x1 ?? o.x, o.x2 ?? o.x) : o.x, ox1 = o.kind === 'capsule' ? Math.max(o.x1 ?? o.x, o.x2 ?? o.x) : o.x;
			const oy0 = o.kind === 'capsule' ? Math.min(o.y1 ?? o.y, o.y2 ?? o.y) : o.y, oy1 = o.kind === 'capsule' ? Math.max(o.y1 ?? o.y, o.y2 ?? o.y) : o.y;
			if (ox1 + need < minX || ox0 - need > maxX || oy1 + need < minY || oy0 - need > maxY) continue;
			const d = o.kind === 'capsule'
				? segSegDist(pd.x, pd.y, x, y, o.x1 ?? o.x, o.y1 ?? o.y, o.x2 ?? o.x, o.y2 ?? o.y)
				: segDist(o.x, o.y, pd.x, pd.y, x, y);
			if (d < need) return false;
		}
		return true;
	};

	// ── 焊盘铜皮节点（起落点集合） ──
	const padNodeCache = new Map<number, Int32Array>();
	const padNodes = (p: number): Int32Array => {
		const hit = padNodeCache.get(p);
		if (hit) return hit;
		const pd = req.pads[p];
		const own = idOf(pd.net) + 1;
		const lays = pd.layers.filter(l => l >= 1 && l <= L);
		let out: Array<{ k: number; d: number }> = [];
		for (const thr of [0, 2, 4]) {
			out = [];
			forBox(pd.x - pd.extent - thr - cell, pd.y - pd.extent - thr - cell, pd.x + pd.extent + thr + cell, pd.y + pd.extent + thr + cell, (i, x, y) => {
				if (pd.dist(x, y) > thr) return;
				const dc = Math.hypot(x - pd.x, y - pd.y);
				// 焊盘铜皮外的放宽点：离中心太近的接线段会被 mergePrims 当微段丢掉
				if (thr > 0 && pd.dist(x, y) > 0 && dc < 2.5) return;
				for (const l of lays) {
					const k = (l - 1) * N + i;
					const s = sTrk[k];
					if ((s === 0 || s === own) && joinStaticClear(pd, l, x, y)) out.push({ k, d: dc });
				}
			});
			if (out.length) break;
		}
		out.sort((a, b) => a.d - b.d || a.k - b.k);
		const arr = Int32Array.from(out.map(o => o.k));
		padNodeCache.set(p, arr);
		return arr;
	};

	// ── A* 工作区 ──
	const g = new Float64Array(LN);
	const par = new Int32Array(LN);
	const dirOf = new Int8Array(LN);
	const seen = new Uint32Array(LN), closed = new Uint32Array(LN), goal = new Uint32Array(LN);
	let sEpoch = 0;
	const heap = new Heap();
	let totalExpand = 0;
	let pf = 0.5;
	// 转向罚分表（方向 a → b）：同向 0 / 45° / 90° / 135° / 180°
	const turnTab = new Float64Array(64);
	for (let a = 0; a < 8; a++) {
		for (let b = 0; b < 8; b++) {
			const c = (DX[a] * DX[b] + DY[a] * DY[b]) / (Math.hypot(DX[a], DY[a]) * Math.hypot(DX[b], DY[b]));
			turnTab[a * 8 + b] = c > 0.99 ? 0 : c > 0.6 ? 0.15 * cell : c > -0.1 ? 0.4 * cell : c > -0.9 ? 1.0 * cell : 2.0 * cell;
		}
	}
	const STEP = [cell, cell, cell, cell, cell * Math.SQRT2, cell * Math.SQRT2, cell * Math.SQRT2, cell * Math.SQRT2];
	// 启发式权重 > 1：放弃严格最优换扩展量（结果仍确定）
	const HW = 1.25;

	/** 最近一次 search 失败原因：unreachable（无路）/ expand（单次上限）/ budget（总量或取消） */
	let lastFail: 'unreachable' | 'expand' | 'budget' = 'unreachable';
	const search = (own: number, sources: number[], goals: Int32Array, tx: number, ty: number, treach: number, ownVias: number[]): number[] | null => {
		sEpoch++;
		heap.clear();
		lastFail = 'unreachable';
		const e = sEpoch;
		for (const k of goals) goal[k] = e;
		const hOf = (x: number, y: number): number => {
			const dx = x - tx, dy = y - ty;
			const d = Math.sqrt(dx * dx + dy * dy) - treach;
			return d > 0 ? d * HW : 0;
		};
		for (const s of sources) {
			if (seen[s] === e) continue;
			seen[s] = e; g[s] = 0; par[s] = -1; dirOf[s] = -1;
			const i = s % N;
			heap.push(hOf(X(i), Y(i)), s);
		}
		const vSame2 = (R.viaViaSame / cell) * (R.viaViaSame / cell);
		let pops = 0;
		while (heap.size) {
			const u = heap.popVal();
			if (closed[u] === e) continue;
			closed[u] = e;
			if (goal[u] === e) {
				const path: number[] = [];
				for (let c = u; c !== -1; c = par[c]) path.push(c);
				path.reverse();
				totalExpand += pops;
				return path;
			}
			if (++pops > maxExpand) { lastFail = 'expand'; break; }
			// 全阶段总量上限也在搜索内部检查：否则超限后本轮剩余的每条连接仍各能吃满 maxExpand
			if (totalExpand + pops > maxTotal) { lastFail = 'budget'; break; }
			if ((pops & 0xffff) === 0 && cancelled()) { lastFail = 'budget'; break; }
			const lyr = u >= N ? 1 : 0;
			const base = lyr * N;
			const i = u - base;
			const iy = (i / W) | 0, ix = i - iy * W;
			const gu = g[u];
			const du = dirOf[u];
			for (let d = 0; d < 8; d++) {
				const nx = ix + DX[d], ny = iy + DY[d];
				if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
				const v = base + ny * W + nx;
				if (closed[v] === e) continue;
				const st = sTrk[v];
				if (st === -1 || (st !== 0 && st !== own)) continue;
				const len = STEP[d];
				let c = len * (1 + hTrk[v]) * (1 + pf * oTrk[v]);
				if (fTrk[v] > 0) c += fixPen * (len / cell);
				if (du >= 0) c += turnTab[du * 8 + d];
				const ng = gu + c;
				if (seen[v] !== e || ng < g[v]) {
					seen[v] = e; g[v] = ng; par[v] = u; dirOf[v] = d;
					heap.push(ng + hOf(x0 + nx * cell, y0 + ny * cell), v);
				}
			}
			if (L === 2) {
				const v = (1 - lyr) * N + i;
				if (closed[v] === e) continue;
				const sv = sVia[i];
				if (sv === -1 || (sv !== 0 && sv !== own)) continue;
				const st = sTrk[v];
				if (st === -1 || (st !== 0 && st !== own)) continue;
				let okSpacing = true;
				for (const j of ownVias) {
					if (j === i) continue;   // 同一列 = 同一个过孔（复用已有过孔）
					const jy = (j / W) | 0, jx = j - jy * W;
					const ddx = jx - ix, ddy = jy - iy;
					if (ddx * ddx + ddy * ddy < vSame2) { okSpacing = false; break; }
				}
				if (!okSpacing) continue;
				let c = viaCost * (1 + hVia[i]) * (1 + pf * oVia[i]) + cell * (1 + hTrk[v]) * (1 + pf * oTrk[v]);
				if (fVia[i] > 0) c += fixPen * 4;
				if (fTrk[v] > 0) c += fixPen;
				const ng = gu + c;
				if (seen[v] !== e || ng < g[v]) {
					seen[v] = e; g[v] = ng; par[v] = u; dirOf[v] = -1;
					heap.push(ng + hOf(x0 + ix * cell, y0 + iy * cell), v);
				}
			}
		}
		totalExpand += pops;
		return null;
	};

	// ── 几何：路径节点图 → 同层折线链 ──
	const P = req.pads.length;
	const padVertex = (p: number, layerIdx: number) => LN + p * 2 + layerIdx;
	const vX = (v: number) => (v < LN ? X(v % N) : req.pads[(v - LN) >> 1].x);
	const vY = (v: number) => (v < LN ? Y(v % N) : req.pads[(v - LN) >> 1].y);
	const vLayer = (v: number) => (v < LN ? Math.floor(v / N) + 1 : ((v - LN) & 1) + 1);
	const buildGeometry = (paths: number[][], joins: Array<{ pad: number; node: number }>) => {
		const adj = new Map<number, number[]>();
		const edgeKey = (a: number, b: number) => (a < b ? a * (LN + 2 * P + 2) + b : b * (LN + 2 * P + 2) + a);
		const edges = new Set<number>();
		const viaCols: number[] = [];
		const viaColSet = new Set<number>();
		const addEdge = (a: number, b: number) => {
			const key = edgeKey(a, b);
			if (edges.has(key)) return;
			edges.add(key);
			if (!adj.has(a)) adj.set(a, []);
			if (!adj.has(b)) adj.set(b, []);
			adj.get(a)!.push(b);
			adj.get(b)!.push(a);
		};
		const nodeSet = new Set<number>();
		for (const path of paths) {
			for (let k = 0; k < path.length; k++) {
				nodeSet.add(path[k]);
				if (k === 0) continue;
				const a = path[k - 1], b = path[k];
				if (a % N === b % N) {
					if (!viaColSet.has(a % N)) { viaColSet.add(a % N); viaCols.push(a % N); }
				} else addEdge(a, b);
			}
		}
		for (const j of joins) {
			nodeSet.add(j.node);
			const pd = req.pads[j.pad];
			const lidx = Math.floor(j.node / N);
			if (Math.hypot(X(j.node % N) - pd.x, Y(j.node % N) - pd.y) < 1e-6) continue;
			addEdge(padVertex(j.pad, lidx), j.node);
		}
		const isBreak = (v: number) => {
			if (v >= LN) return true;
			if (viaColSet.has(v % N)) return true;
			return (adj.get(v)?.length ?? 0) !== 2;
		};
		const used = new Set<number>();
		const chains: number[][] = [];
		const verts = [...adj.keys()].sort((a, b) => a - b);
		const walk = (start: number, next: number): number[] => {
			const out = [start];
			let prev = start, cur = next;
			used.add(edgeKey(prev, cur));
			for (;;) {
				out.push(cur);
				if (isBreak(cur)) break;
				const nb = adj.get(cur)!;
				const nxt = nb[0] === prev ? nb[1] : nb[0];
				if (used.has(edgeKey(cur, nxt))) break;
				used.add(edgeKey(cur, nxt));
				prev = cur; cur = nxt;
			}
			return out;
		};
		for (const v of verts) {
			if (!isBreak(v)) continue;
			for (const nb of adj.get(v)!) {
				if (used.has(edgeKey(v, nb))) continue;
				chains.push(walk(v, nb));
			}
		}
		for (const v of verts) {
			// 纯环（无断点）：罕见，任取一点断开
			for (const nb of adj.get(v)!) {
				if (used.has(edgeKey(v, nb))) continue;
				chains.push(walk(v, nb));
			}
		}
		// 共线内点化简
		const outChains: NGChain[] = [];
		for (const ch of chains) {
			const pts: NGPoint[] = [];
			for (let k = 0; k < ch.length; k++) {
				const p = { x: vX(ch[k]), y: vY(ch[k]) };
				if (pts.length >= 2) {
					const a = pts[pts.length - 2], b = pts[pts.length - 1];
					const ux = b.x - a.x, uy = b.y - a.y, wx = p.x - b.x, wy = p.y - b.y;
					if (Math.abs(ux * wy - uy * wx) < 1e-6 && ux * wx + uy * wy > 0) { pts[pts.length - 1] = p; continue; }
				}
				pts.push(p);
			}
			outChains.push({ layer: vLayer(ch[0] < LN ? ch[0] : ch[1]), pts });
		}
		// 接线段经过的格点也计入本网占据（冲突判定），其余为路径节点
		const trkNodes: number[] = [...nodeSet];
		for (const j of joins) {
			const pd = req.pads[j.pad];
			const lidx = Math.floor(j.node / N);
			const ax = pd.x, ay = pd.y, bx = X(j.node % N), by = Y(j.node % N);
			forBox(Math.min(ax, bx) - cell, Math.min(ay, by) - cell, Math.max(ax, bx) + cell, Math.max(ay, by) + cell, (i, x, y) => {
				if (segDist(x, y, ax, ay, bx, by) < cell * 0.5) trkNodes.push(lidx * N + i);
			});
		}
		const uniqT = [...new Set(trkNodes)].sort((a, b) => a - b);
		return {
			chains: orderChains(outChains),
			vias: viaCols.map(i => ({ x: X(i), y: Y(i) })),
			trkNodes: Int32Array.from(uniqT),
			viaCols: Int32Array.from(viaCols),
		};
	};

	// ── 单网布线：Prim 顺序 + 多源 A* ──
	const treeMark = new Uint32Array(LN), pathMark = new Uint32Array(LN);
	let tEpoch = 0;
	const routeNet = (nid: number): Route | null => {
		const name = names[nid];
		const net = netByName.get(name)!;
		const own = nid + 1;
		const pads = net.pads;
		if (pads.length < 2) return { chains: [], vias: [], trkNodes: new Int32Array(0), viaCols: new Int32Array(0), stamp: { trk: new Int32Array(0), via: new Int32Array(0) } };
		for (const p of pads) if (!padNodes(p).length) { log(`[ngrid] ${name}: 焊盘 (${req.pads[p].x.toFixed(0)},${req.pads[p].y.toFixed(0)}) 无可用起落节点`); return null; }
		tEpoch++;
		const te = tEpoch;
		const tree: number[] = [];
		const padOf = new Map<number, number>();
		const treePts: NGPoint[] = [];
		const paths: number[][] = [];
		const joins: Array<{ pad: number; node: number }> = [];
		const ownVias: number[] = [];
		const connected = new Set<number>();
		const addPad = (p: number) => {
			connected.add(p);
			treePts.push({ x: req.pads[p].x, y: req.pads[p].y });
			for (const k of padNodes(p)) {
				if (!padOf.has(k)) padOf.set(k, p);
				if (treeMark[k] !== te) { treeMark[k] = te; tree.push(k); }
			}
		};
		addPad(pads[0]);
		while (connected.size < pads.length) {
			let t = -1, best = Infinity;
			for (const p of pads) {
				if (connected.has(p)) continue;
				const pd = req.pads[p];
				let d = Infinity;
				for (const q of treePts) { const dd = Math.hypot(q.x - pd.x, q.y - pd.y); if (dd < d) d = dd; }
				if (d < best) { best = d; t = p; }
			}
			const tn = padNodes(t);
			let hit = -1;
			for (const k of tn) if (treeMark[k] === te) { hit = k; break; }
			if (hit >= 0) {
				if (pathMark[hit] !== te && padOf.has(hit)) joins.push({ pad: padOf.get(hit)!, node: hit });
				joins.push({ pad: t, node: hit });
				addPad(t);
				continue;
			}
			const pd = req.pads[t];
			if (totalExpand > maxTotal || cancelled()) { lastFail = 'budget'; return null; }
			// 同一条 A* 路径内的两次换层互相看不见（ownVias 只含已完成分支的过孔）：
			// 重建后复核孔壁间距，不合格就把这条路径的过孔当作「已有过孔」重搜（≤3 次）
			let path: number[] | null = null;
			const extra: number[] = [];
			for (let attempt = 0; attempt < 3; attempt++) {
				path = search(own, tree, tn, pd.x, pd.y, pd.extent, extra.length ? [...ownVias, ...extra] : ownVias);
				if (!path) break;
				const cols: number[] = [];
				for (let k = 1; k < path.length; k++) if (path[k - 1] % N === path[k] % N) cols.push(path[k] % N);
				const tooClose = (a: number, b: number) => a !== b && Math.hypot(X(a) - X(b), Y(a) - Y(b)) < R.viaViaSame;
				let bad = false;
				for (let a = 0; a < cols.length && !bad; a++) {
					for (let b = a + 1; b < cols.length && !bad; b++) bad = tooClose(cols[a], cols[b]);
					for (const o of ownVias) if (!bad) bad = tooClose(cols[a], o);
				}
				if (!bad) break;
				extra.push(...cols);
				path = null;
				lastFail = 'unreachable';
			}
			if (!path) { log(`[ngrid] ${name}: 连接 (${pd.x.toFixed(0)},${pd.y.toFixed(0)}) 无路径（${lastFail}）`); return null; }
			const s = path[0];
			if (pathMark[s] !== te && padOf.has(s)) joins.push({ pad: padOf.get(s)!, node: s });
			joins.push({ pad: t, node: path[path.length - 1] });
			paths.push(path);
			for (let k = 0; k < path.length; k++) {
				const v = path[k];
				if (k > 0 && path[k - 1] % N === v % N) ownVias.push(v % N);
				pathMark[v] = te;
				if (treeMark[v] !== te) { treeMark[v] = te; tree.push(v); }
				if ((k & 7) === 0) treePts.push({ x: X(v % N), y: Y(v % N) });
			}
			addPad(t);
		}
		// 去重接线
		const seenJ = new Set<string>();
		const uj = joins.filter(j => { const k = `${j.pad}|${j.node}`; if (seenJ.has(k)) return false; seenJ.add(k); return true; });
		const geo = buildGeometry(paths, uj);
		const segs: NGSeg[] = [];
		for (const ch of geo.chains) {
			for (let k = 1; k < ch.pts.length; k++) segs.push({ layer: ch.layer, x1: ch.pts[k - 1].x, y1: ch.pts[k - 1].y, x2: ch.pts[k].x, y2: ch.pts[k].y });
		}
		return { ...geo, stamp: computeStamp(segs, geo.vias) };
	};

	// ── 精确拉紧（网格楼梯 → 任意角折线）──
	// 收敛后按真实几何逐网做 string-pulling：链端点（焊盘中心/过孔/分叉点）不动，
	// 内点能隔点直连就删。判定是精确距离（不走网格），对象 = 其它参与者的最终
	// 折线/过孔 + 固定网几何 + 焊盘铜皮/钻孔 + 静态障碍，阈值与网格同一套规则。
	// 逐网顺序处理并即时更新索引：每次判定都是对「当前」他网几何的成对净距，
	// 所以结果两两合法与处理顺序无关。
	const pullTaut = (ids0: number[], onReject: (id: number) => number): Map<number, NGChain[]> => {
		const out = new Map<number, NGChain[]>();
		let ids = [...ids0];
		/** 拉紧采纳新线段时的浮点余量；复核原网格几何时取 0（规则值本身已含引擎余量） */
		const EPS_PULL = 0.05;
		type Obj = { t: 0; net: number; layer: number; x1: number; y1: number; x2: number; y2: number; dead: boolean }
			| { t: 1; net: number; x: number; y: number; dead: boolean }
			| { t: 2; net: number; pad: number; dead: boolean }
			| { t: 3; net: number; o: NGStatic; dead: boolean };
		const objs: Obj[] = [];
		const BC = 48;
		const BW = Math.ceil((W * cell) / BC) + 2;
		const buckets = new Map<number, number[]>();
		const reachMax = Math.max(R.trkTrk, R.trkVia, R.trkPad) + 2;
		const insert = (o: Obj, ax: number, ay: number, bx: number, by: number) => {
			const id = objs.length;
			objs.push(o);
			const i0 = Math.floor((ax - x0 - reachMax) / BC), i1 = Math.floor((bx - x0 + reachMax) / BC);
			const j0 = Math.floor((ay - y0 - reachMax) / BC), j1 = Math.floor((by - y0 + reachMax) / BC);
			for (let j = j0; j <= j1; j++) {
				for (let i = i0; i <= i1; i++) {
					const k = j * BW + i;
					const b = buckets.get(k);
					if (b) b.push(id); else buckets.set(k, [id]);
				}
			}
			return id;
		};
		const addSeg = (net: number, sg: NGSeg) => insert({ t: 0, net, layer: sg.layer, x1: sg.x1, y1: sg.y1, x2: sg.x2, y2: sg.y2, dead: false },
			Math.min(sg.x1, sg.x2), Math.min(sg.y1, sg.y2), Math.max(sg.x1, sg.x2), Math.max(sg.y1, sg.y2));
		const addFixed = (f: number) => {
			const nid = idOf(fixed[f].name);
			for (const sg of fixed[f].segs) addSeg(nid, sg);
			for (const v of fixed[f].vias) insert({ t: 1, net: nid, x: v.x, y: v.y, dead: false }, v.x, v.y, v.x, v.y);
		};
		for (let f = 0; f < fixed.length; f++) {
			if (fStamps[f] && !fActive[f]) continue;   // 已升级：几何已撕
			addFixed(f);
		}
		for (let p = 0; p < req.pads.length; p++) {
			const pd = req.pads[p];
			const ext = pd.extent + Math.max(0, pd.holeD / 2 + R.trkHoleExtra - pd.extent);
			insert({ t: 2, net: idOf(pd.net), pad: p, dead: false }, pd.x - ext, pd.y - ext, pd.x + ext, pd.y + ext);
		}
		for (const o of req.statics) {
			const ax = o.kind === 'capsule' ? Math.min(o.x1 ?? o.x, o.x2 ?? o.x) : o.x;
			const bx = o.kind === 'capsule' ? Math.max(o.x1 ?? o.x, o.x2 ?? o.x) : o.x;
			const ay = o.kind === 'capsule' ? Math.min(o.y1 ?? o.y, o.y2 ?? o.y) : o.y;
			const by = o.kind === 'capsule' ? Math.max(o.y1 ?? o.y, o.y2 ?? o.y) : o.y;
			insert({ t: 3, net: o.net ? idOf(o.net) : -1, o, dead: false }, ax - o.r, ay - o.r, bx + o.r, by + o.r);
		}
		const segIds = new Map<number, number[]>();
		const viaIds: Array<[number, number]> = [];
		for (const id of ids) {
			const r = routes.get(id)!;
			const list: number[] = [];
			for (const ch of r.chains) {
				for (let k = 1; k < ch.pts.length; k++) list.push(addSeg(id, { layer: ch.layer, x1: ch.pts[k - 1].x, y1: ch.pts[k - 1].y, x2: ch.pts[k].x, y2: ch.pts[k].y }));
			}
			for (const v of r.vias) viaIds.push([id, insert({ t: 1, net: id, x: v.x, y: v.y, dead: false }, v.x, v.y, v.x, v.y)]);
			segIds.set(id, list);
		}
		const qMark = new Uint32Array(1 << 20);
		let qEpoch = 0;
		const clear = (net: number, layer: number, ax: number, ay: number, bx: number, by: number, EPS: number): boolean => {
			qEpoch++;
			if (qEpoch === 0xffffffff) { qMark.fill(0); qEpoch = 1; }
			const i0 = Math.floor((Math.min(ax, bx) - x0) / BC), i1 = Math.floor((Math.max(ax, bx) - x0) / BC);
			const j0 = Math.floor((Math.min(ay, by) - y0) / BC), j1 = Math.floor((Math.max(ay, by) - y0) / BC);
			for (let j = j0; j <= j1; j++) {
				for (let i = i0; i <= i1; i++) {
					const b = buckets.get(j * BW + i);
					if (!b) continue;
					for (const oid of b) {
						if (oid < qMark.length) { if (qMark[oid] === qEpoch) continue; qMark[oid] = qEpoch; }
						const o = objs[oid];
						if (o.dead || o.net === net) continue;
						if (o.t === 0) {
							if (o.layer !== layer) continue;
							if (segSegDist(ax, ay, bx, by, o.x1, o.y1, o.x2, o.y2) < R.trkTrk + EPS) return false;
						} else if (o.t === 1) {
							if (segDist(o.x, o.y, ax, ay, bx, by) < R.trkVia + EPS) return false;
						} else if (o.t === 2) {
							const pd = req.pads[o.pad];
							if (pd.layers.includes(layer) && padSegMin(pd, ax, ay, bx, by, R.trkPad) < R.trkPad + EPS) return false;
							if (pd.holeD > 0 && segDist(pd.x, pd.y, ax, ay, bx, by) < pd.holeD / 2 + R.trkHoleExtra + EPS) return false;
						} else {
							const st = o.o;
							if (st.layer !== undefined && st.layer !== layer) continue;
							const d = st.kind === 'capsule'
								? segSegDist(ax, ay, bx, by, st.x1 ?? st.x, st.y1 ?? st.y, st.x2 ?? st.x, st.y2 ?? st.y)
								: segDist(st.x, st.y, ax, ay, bx, by);
							if (d < st.r + R.staticTrkExtra + EPS) return false;
						}
					}
				}
			}
			return true;
		};
		// 复核：协商结果（网格折线 + 接线段）逐段按精确距离过一遍。网格的弦垂余量、
		// 接线段只查了端点节点等近似，都在这里兜住 —— 不过的网判失败（升级网退回
		// 固定网，原几何重新入索引，再复核一轮直到稳定），不把违规交给引擎。
		// 过孔精确复核（过孔在网格节点上，网格判定本身无弦垂；这里主要防「退回固定网
		// 的原铜皮重新入索引」后与幸存网过孔的冲突）
		const QV = Math.max(R.viaVia, R.trkVia, R.viaPad, R.viaPadSame) + 2;
		const viaClear = (net: number, x: number, y: number): boolean => {
			qEpoch++;
			if (qEpoch === 0xffffffff) { qMark.fill(0); qEpoch = 1; }
			const i0 = Math.floor((x - QV - x0) / BC), i1 = Math.floor((x + QV - x0) / BC);
			const j0 = Math.floor((y - QV - y0) / BC), j1 = Math.floor((y + QV - y0) / BC);
			for (let j = j0; j <= j1; j++) {
				for (let i = i0; i <= i1; i++) {
					const b = buckets.get(j * BW + i);
					if (!b) continue;
					for (const oid of b) {
						if (oid < qMark.length) { if (qMark[oid] === qEpoch) continue; qMark[oid] = qEpoch; }
						const o = objs[oid];
						if (o.dead) continue;
						if (o.t === 2) {
							const pd = req.pads[o.pad];
							const d = pd.dist(x, y);
							if (o.net === net) { if (!req.viaInPad && d < R.viaPadSame) return false; continue; }
							if (d < R.viaPad) return false;
							if (pd.holeD > 0 && Math.hypot(pd.x - x, pd.y - y) < pd.holeD / 2 + R.viaHoleExtra) return false;
							continue;
						}
						if (o.net === net) continue;
						if (o.t === 0) {
							if (segDist(x, y, o.x1, o.y1, o.x2, o.y2) < R.trkVia) return false;
						} else if (o.t === 1) {
							if (Math.hypot(o.x - x, o.y - y) < R.viaVia) return false;
						} else {
							const st = o.o;
							const d = st.kind === 'capsule'
								? segDist(x, y, st.x1 ?? st.x, st.y1 ?? st.y, st.x2 ?? st.x, st.y2 ?? st.y)
								: Math.hypot(st.x - x, st.y - y);
							if (d < st.r + R.staticViaExtra) return false;
						}
					}
				}
			}
			return true;
		};
		for (let changed = true; changed;) {
			changed = false;
			for (const id of ids) {
				const r = routes.get(id)!;
				let ok = true;
				for (const ch of r.chains) {
					for (let k = 1; k < ch.pts.length && ok; k++) ok = clear(id, ch.layer, ch.pts[k - 1].x, ch.pts[k - 1].y, ch.pts[k].x, ch.pts[k].y, 0);
					if (!ok) break;
				}
				for (const v of r.vias) if (ok) ok = viaClear(id, v.x, v.y);
				if (ok) continue;
				log(`[ngrid] ${names[id]}: 精确复核未过，判失败`);
				for (const sid of segIds.get(id) ?? []) objs[sid].dead = true;
				for (const [vid, oid] of viaIds) if (vid === id) objs[oid].dead = true;
				const f = onReject(id);
				if (f >= 0) addFixed(f);
				ids = ids.filter(x => x !== id);
				changed = true;
				break;
			}
		}
		for (const id of ids) {
			const r = routes.get(id)!;
			const chains: NGChain[] = [];
			for (const sid of segIds.get(id) ?? []) objs[sid].dead = true;
			const fresh: number[] = [];
			for (const ch of r.chains) {
				const pts = ch.pts;
				const np: NGPoint[] = [pts[0]];
				let i = 0;
				while (i < pts.length - 1) {
					let j = pts.length - 1;
					while (j > i + 1 && !clear(id, ch.layer, pts[i].x, pts[i].y, pts[j].x, pts[j].y, EPS_PULL)) j--;
					np.push(pts[j]);
					i = j;
				}
				chains.push({ layer: ch.layer, pts: np });
				for (let k = 1; k < np.length; k++) fresh.push(addSeg(id, { layer: ch.layer, x1: np[k - 1].x, y1: np[k - 1].y, x2: np[k].x, y2: np[k].y }));
			}
			segIds.set(id, fresh);
			out.set(id, chains);
		}
		return out;
	};

	// ── 协商主循环 ──
	const nameOf = new Map<number, string>(names.map((n, i) => [i, n] as const));
	const part: number[] = [];
	const partSet = new Set<number>();
	const escalated = new Set<number>();
	const deadNets = new Set<number>();
	/** 升级后又失败/被剔的网：原铜皮重新计入固定占据，不交出、不算失败 */
	const restored = new Set<number>();
	for (const n of req.participants) {
		const net = netByName.get(n);
		if (!net || net.pads.length < 2) continue;
		const id = netId.get(n)!;
		if (!partSet.has(id)) { partSet.add(id); part.push(id); }
	}
	const markOwnT = new Uint32Array(LN), markOwnV = new Uint32Array(N);
	let oEpoch = 0;
	const evaluate = (bumpHist: boolean) => {
		const conflicted = new Map<number, number>();
		const cross = new Map<number, Set<number>>();
		for (const id of part) {
			const r = routes.get(id);
			if (!r) continue;
			oEpoch++;
			const e = oEpoch;
			for (const k of r.stamp.trk) markOwnT[k] = e;
			for (const k of r.stamp.via) markOwnV[k] = e;
			let cnt = 0;
			const owners = new Set<number>();
			for (const k of r.trkNodes) {
				const others = oTrk[k] - (markOwnT[k] === e ? 1 : 0);
				if (others > 0) { cnt++; if (bumpHist) hTrk[k] += 0.5; }
				if (fTrk[k] > 0) fixedOwners(k, false, owners);
			}
			for (const i of r.viaCols) {
				const others = oVia[i] - (markOwnV[i] === e ? 1 : 0);
				if (others > 0) { cnt++; if (bumpHist) hVia[i] += 0.5; }
				if (fVia[i] > 0) fixedOwners(i, true, owners);
			}
			if (owners.size) cnt += owners.size;
			if (cnt) conflicted.set(id, cnt);
			if (owners.size) cross.set(id, owners);
		}
		return { conflicted, cross };
	};

	let it = 0;
	let converged = false;
	let toRoute: number[] = [...part];
	// 最好的一轮（冲突网最少）：未收敛时退回它再做贪心剔除
	let bestConf = Infinity;
	let bestRoutes: Map<number, Route | null> | null = null;
	let stagnant = 0;
	// 连续求解失败计数：一次失败（例如单次扩展上限）保留旧路由、下轮重试；
	// 连续 3 次且手上没有路由才判死。判死的升级网退回固定网（原铜皮重新占位）
	const failCount = new Map<number, number>();
	const retire = (id: number) => {
		deadNets.add(id);
		const r = routes.get(id);
		if (r) applyStamp(r.stamp, -1);
		routes.set(id, null);
		if (escalated.has(id)) {
			escalated.delete(id);
			restored.add(id);
			restoreFixed(fixedOfNet.get(id)!);
		}
	};
	let budgetOut = false;
	for (; it < maxIter; it++) {
		if (cancelled()) break;
		if (totalExpand > maxTotal) { log(`[ngrid] 扩展总量超限 ${totalExpand}`); budgetOut = true; break; }
		for (const id of toRoute) {
			if (cancelled() || totalExpand > maxTotal) { budgetOut = true; break; }
			const old = routes.get(id);
			if (old) applyStamp(old.stamp, -1);
			routes.set(id, null);
			const r = routeNet(id);
			if (!r) {
				// 失败：旧路由放回（冲突照算，下轮再试）；预算耗尽则本轮到此为止
				if (old) { routes.set(id, old); applyStamp(old.stamp, 1); }
				if ((lastFail as string) === 'budget') { budgetOut = true; break; }
				const n = (failCount.get(id) ?? 0) + 1;
				failCount.set(id, n);
				if (n >= 3 && !old) retire(id);
				continue;
			}
			failCount.set(id, 0);
			routes.set(id, r);
			applyStamp(r.stamp, 1);
		}
		if (budgetOut) { log(`[ngrid] 扩展总量超限/取消 ${totalExpand}`); break; }
		const { conflicted, cross } = evaluate(true);
		// 升级被穿越的固定网（退回过的不再升级）
		const newly: number[] = [];
		const crossedF = new Set<number>();
		for (const id of part) for (const f of cross.get(id) ?? []) crossedF.add(f);
		let blocked = false;
		for (const f of [...crossedF].sort((a, b) => a - b)) {
			const nid = idOf(fixed[f].name);
			if (restored.has(nid) || escalated.size >= maxEscalate) { blocked = true; continue; }
			removeFixed(f);
			escalated.add(nid);
			if (!partSet.has(nid)) { partSet.add(nid); part.push(nid); newly.push(nid); }
		}
		if (blocked) fixPen *= 1.5;
		const unrouted = part.filter(id => !routes.get(id) && !deadNets.has(id));
		const confCount = conflicted.size;
		const fullRound = toRoute.length === part.length;
		log(`[ngrid] 第 ${it + 1} 轮：重布 ${toRoute.length}${fullRound ? '（全量）' : ''}，冲突网 ${confCount}，升级 ${newly.length}（参与 ${part.length}，待重试 ${unrouted.length}，失败 ${deadNets.size}）pf=${pf.toFixed(2)} 扩展累计 ${totalExpand}`);
		if (!confCount && !newly.length && !unrouted.length) { converged = true; it++; break; }
		if (newly.length) {
			// 参与者集合变了：旧快照缺新网的路由，作废
			bestConf = Infinity; bestRoutes = null; stagnant = 0;
		} else if (confCount + unrouted.length < bestConf) {
			bestConf = confCount + unrouted.length; bestRoutes = new Map(routes); stagnant = 0;
		} else stagnant++;
		pf = Math.min(pf * 1.4, 1000);
		// 只重布冲突网会把「不冲突但挡路」的网冻住（P0x 总线实测两网互穿 40 轮）：
		// 连续两轮没有改善就全量重布一轮，让历史罚分推动整体挪位
		let full = false;
		if (stagnant >= 2) { full = true; stagnant = 0; }
		const next = new Set<number>([...newly, ...unrouted]);
		for (const id of conflicted.keys()) next.add(id);
		toRoute = part.filter(id => (full || next.has(id)) && !deadNets.has(id));
	}
	if (!converged && bestRoutes) {
		// 快照之后判死/退回的网不再带回来（它们的占据会误伤别的网）
		for (const id of part) { const r = routes.get(id); if (r) applyStamp(r.stamp, -1); }
		for (const id of part) {
			const r = deadNets.has(id) ? null : bestRoutes.get(id) ?? null;
			routes.set(id, r);
			if (r) applyStamp(r.stamp, 1);
		}
		log(`[ngrid] 未收敛：退回最佳轮（冲突/未布 ${bestConf}）`);
	}

	if (!converged && req.debug) {
		// 诊断：列出残余冲突点（节点坐标 + 冲突对象）
		for (const id of part) {
			const r = routes.get(id);
			if (!r) continue;
			oEpoch++;
			const e = oEpoch;
			for (const k of r.stamp.trk) markOwnT[k] = e;
			for (const k of r.stamp.via) markOwnV[k] = e;
			const msgs: string[] = [];
			for (const k of r.trkNodes) {
				const others = oTrk[k] - (markOwnT[k] === e ? 1 : 0);
				const i = k % N;
				if (others > 0 || fTrk[k] > 0) {
					const who: string[] = [];
					for (const o of part) {
						if (o === id) continue;
						const ro = routes.get(o);
						if (ro && ro.stamp.trk.includes(k)) who.push(names[o]);
					}
					const fo = new Set<number>();
					if (fTrk[k] > 0) fixedOwners(k, false, fo);
					msgs.push(`T L${Math.floor(k / N) + 1}(${X(i).toFixed(0)},${Y(i).toFixed(0)}) ${who.join('/')}${fo.size ? ' fixed:' + [...fo].map(f => fixed[f].name).join('/') : ''}`);
				}
			}
			for (const i of r.viaCols) {
				const others = oVia[i] - (markOwnV[i] === e ? 1 : 0);
				if (others > 0 || fVia[i] > 0) msgs.push(`V(${X(i).toFixed(0)},${Y(i).toFixed(0)}) occ=${others} fix=${fVia[i]}`);
			}
			if (msgs.length) log(`[ngrid-debug] ${names[id]}: ${msgs.length} 冲突点 ${msgs.slice(0, 6).join(' | ')}`);
		}
	}
	// ── 未收敛：贪心剔除冲突网，剩下的必须两两无冲突且不压固定网 ──
	// 先剔原失败网（它们本来就没布通）；剔掉的升级网退回固定网，原铜皮重新占位，
	// 下一次 evaluate 会把压在它旧走廊上的幸存网也标成冲突继续剔 —— 保证交出去的
	// 结果里，退回的网原样放回不会撞任何人（引擎快照恢复必过）。
	const dropOne = (victim: number) => {
		const r = routes.get(victim);
		if (r) applyStamp(r.stamp, -1);
		routes.set(victim, null);
		dropped.add(victim);
		if (escalated.has(victim)) {
			escalated.delete(victim);
			restored.add(victim);
			restoreFixed(fixedOfNet.get(victim)!);
		}
	};
	const dropped = new Set<number>(deadNets);
	if (!converged) {
		for (const id of part) if (!routes.get(id) && !dropped.has(id)) dropOne(id);
		for (let guard = 0; guard < part.length + 1; guard++) {
			const { conflicted } = evaluate(false);
			if (!conflicted.size) break;
			let victim = -1, vk: [number, number] = [2, 0];
			for (const id of part) {
				const c = conflicted.get(id);
				if (!c) continue;
				const key: [number, number] = [escalated.has(id) ? 1 : 0, -c];
				if (victim < 0 || key[0] < vk[0] || (key[0] === vk[0] && key[1] < vk[1])) { victim = id; vk = key; }
			}
			dropOne(victim);
		}
	}

	const routed: NGNetResult[] = [];
	const failed: string[] = [];
	let finalIds: number[] = [];
	for (const id of part) {
		if (restored.has(id)) continue;   // 原样保留：既不算失败也不交出新几何
		const r = routes.get(id);
		if (!r || dropped.has(id)) { failed.push(nameOf.get(id)!); continue; }
		finalIds.push(id);
	}
	const pt = pullTaut(finalIds, (id) => {
		// 精确复核不过（网格弦垂/接线段等近似漏网）：判失败；升级网退回固定网
		if (!escalated.has(id)) { failed.push(nameOf.get(id)!); return -1; }
		{
			escalated.delete(id);
			restored.add(id);
			const f = fixedOfNet.get(id)!;
			restoreFixed(f);
			return f;
		}
	});
	finalIds = finalIds.filter(id => pt.has(id));
	for (const id of finalIds) {
		const r = routes.get(id)!;
		routed.push({ name: nameOf.get(id)!, chains: pt.get(id)!, vias: r.vias });
	}
	if (restored.size) log(`[ngrid] 退回固定网（原铜皮保留）：${[...restored].map(id => names[id]).join(', ')}`);
	return {
		ok: true,
		routed,
		failed,
		escalated: [...escalated].map(id => nameOf.get(id)!),
		iterations: it,
		converged,
		expansions: totalExpand,
	};
}

/**
 * 链排序：相邻两条链尽量不首尾相接（相接 ≤15mil 会被 mergePrims 合并共线段、
 * 被 smoothCorners 在分叉点圆角化 → 第三条分支断连；joint 审计也按相邻图元计）。
 * 贪心：按构造序挑「起点离当前末端 >15mil」的下一条（可反向）；挑不到才顺延。
 */
export function orderChains(chains: NGChain[]): NGChain[] {
	if (chains.length <= 1) return chains;
	const left = chains.map((c, i) => ({ c, i }));
	const out: NGChain[] = [];
	const far = (a: NGPoint, b: NGPoint) => Math.hypot(a.x - b.x, a.y - b.y) > 15;
	let cur = left.shift()!.c;
	out.push(cur);
	while (left.length) {
		const end = cur.pts[cur.pts.length - 1];
		let pick = -1, rev = false;
		for (let k = 0; k < left.length; k++) {
			const c = left[k].c;
			const s = c.pts[0], e = c.pts[c.pts.length - 1];
			if (far(s, end) && far(e, end)) { pick = k; rev = false; break; }
		}
		if (pick < 0) {
			for (let k = 0; k < left.length; k++) {
				const c = left[k].c;
				if (far(c.pts[0], end)) { pick = k; rev = false; break; }
				if (far(c.pts[c.pts.length - 1], end)) { pick = k; rev = true; break; }
			}
		}
		if (pick < 0) pick = 0;
		const c = left.splice(pick, 1)[0].c;
		cur = rev ? { layer: c.layer, pts: [...c.pts].reverse() } : c;
		out.push(cur);
	}
	return out;
}
