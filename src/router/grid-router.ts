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
}

interface Cell {
	x: number;
	y: number;
	layer: number;
}

const D8: Array<[number, number, number]> = [
	[1, 0, 1], [-1, 0, 1], [0, 1, 1], [0, -1, 1],
	[1, 1, Math.SQRT2], [1, -1, Math.SQRT2], [-1, 1, Math.SQRT2], [-1, -1, Math.SQRT2],
];

/** 最小堆（二叉堆，按 f 值） */
class MinHeap {
	private keys: number[] = [];
	private vals: number[] = [];

	get size(): number { return this.keys.length; }

	push(key: number, val: number): void {
		this.keys.push(key);
		this.vals.push(val);
		let i = this.keys.length - 1;
		while (i > 0) {
			const p = (i - 1) >> 1;
			if (this.keys[p] <= this.keys[i]) break;
			this.swap(p, i);
			i = p;
		}
	}

	pop(): number {
		const top = this.vals[0];
		const lastKey = this.keys.pop()!;
		const lastVal = this.vals.pop()!;
		if (this.keys.length) {
			this.keys[0] = lastKey;
			this.vals[0] = lastVal;
			let i = 0;
			for (;;) {
				const l = i * 2 + 1, r = l + 1;
				let m = i;
				if (l < this.keys.length && this.keys[l] < this.keys[m]) m = l;
				if (r < this.keys.length && this.keys[r] < this.keys[m]) m = r;
				if (m === i) break;
				this.swap(m, i);
				i = m;
			}
		}
		return top;
	}

	private swap(a: number, b: number): void {
		const k = this.keys[a]; this.keys[a] = this.keys[b]; this.keys[b] = k;
		const v = this.vals[a]; this.vals[a] = this.vals[b]; this.vals[b] = v;
	}
}

/**
 * 在给定层集合上求解 src→dst 的网格最短路。
 * 失败返回 { ok: false }（调用方回退到其它策略）。
 */
export async function gridRoute(req: GridRouteRequest): Promise<GridRouteResult> {
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
	const maxCells = req.maxCells ?? 400_000;
	// 单元尺寸自适应：保证每层网格 ≤ maxCells
	let cell = Math.max(4, req.cell ?? 8);
	for (;;) {
		const nx = Math.ceil((x1 - x0) / cell) + 1;
		const ny = Math.ceil((y1 - y0) / cell) + 1;
		if (nx * ny <= maxCells) break;
		cell *= 1.35;
		if (cell > 60) return fail;
	}
	const nx = Math.ceil((x1 - x0) / cell) + 1;
	const ny = Math.ceil((y1 - y0) / cell) + 1;
	const nLayers = req.layers.length;
	const stride = nx * ny;
	const total = stride * nLayers;
	const deadline = Date.now() + (req.budgetMs ?? 4000);
	const cancelled = (): boolean => {
		try { return req.isCancelled?.() === true; } catch { return false; }
	};

	// ── 光栅化 ──
	// 半单元外扩：保证相邻自由格中心之间的线段不切进障碍圆
	const inflate = cell * 0.5;
	const blocked = new Uint8Array(total);
	// 过孔落点网格：比走线多要求 viaExtraMil 的净空。只有层间跳变的格点
	// 需要它（见 A* 的 via 边），普通走线不看。
	const viaExtra = Math.max(0, req.viaExtraMil ?? 0);
	const viaBlocked = viaExtra > 0 ? new Uint8Array(total) : null;
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

	let marked = 0;
	for (const o of req.obstacles) {
		if (o.soft) continue;
		const sameNet = !!(o.net && o.net === req.net);
		// 同网走线不挡路；同网焊盘在 via-in-pad 关闭时必须挡「层间跳变」
		//（过孔不能落在自家焊盘铜皮上，否则 viaPadClear 判死整条边）。
		const sameNetPad = sameNet && !!o.group?.startsWith('pad:');
		if (sameNet && !(sameNetPad && req.viaInPad !== true)) continue;
		const viaOnly = sameNetPad;
		const lis: number[] = [];
		if (o.layer === undefined) lis.push(...req.layers.map((_, i) => i));
		else {
			const li = layerIndex.get(o.layer);
			if (li !== undefined) lis.push(li);
		}
		if (!lis.length) continue;
		for (const li of lis) {
			const ex = o.group?.startsWith('pad:') ? req.viaExtraPadMil : undefined;
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
		if (++marked > 20000) break;   // 障碍过多保护：兜底求解不值得无限光栅化
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

	// ── A* ──
	const gScore = new Float64Array(total).fill(Infinity);
	const cameFrom = new Int32Array(total).fill(-1);
	const closed = new Uint8Array(total);
	const heap = new MinHeap();
	const viaCost = req.viaCostMil ?? 200;
	const seedStubByCell = new Map<number, { x: number; y: number } | null>();
	const goalStubByCell = new Map<number, { x: number; y: number }>();

	const idxOf = (i: number, j: number, li: number) => li * stride + j * nx + i;
	const hOf = (i: number, j: number) => Math.hypot(i - d.i, j - d.j) * cell;

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
			goalCost.set(id, stub ? Math.hypot(stub.x - dst.x, stub.y - dst.y) : 0);
			if (stub) goalStubByCell.set(id, stub);
		}
	};
	addGoal(d.i, d.j, null);
	for (const p of dstStubPts) {
		const c = toCell(p);
		addGoal(c.i, c.j, p);
	}
	let started = 0;
	for (const sd of seedBest.values()) {
		const id = idxOf(sd.i, sd.j, sd.li);
		gScore[id] = sd.cost;
		seedStubByCell.set(id, sd.stub);
		heap.push(sd.cost + hOf(sd.i, sd.j), id);
		started++;
	}
	if (!started) {
		const sId = idxOf(s.i, s.j, 0), dId = idxOf(d.i, d.j, 0);
		return { ...fail, cell, debug: `no-seeds startBlocked=${blocked[sId]} goalBlocked=${blocked[dId]} srcLayers=${srcLayers} dstLayers=${dstLayers}` };
	}

	let best = -1;
	let bestCost = Infinity;
	let pops = 0;
	while (heap.size) {
		if ((++pops & 255) === 0) {
			if (Date.now() > deadline || cancelled()) break;
		}
		const cur = heap.pop();
		if (closed[cur]) continue;
		closed[cur] = 1;
		const li = Math.floor(cur / stride);
		const rem = cur - li * stride;
		const j = Math.floor(rem / nx);
		const i = rem - j * nx;
		const g = gScore[cur];
		if (isGoal.has(cur)) {
			const total = g + (goalCost.get(cur) ?? 0);
			if (total < bestCost) { bestCost = total; best = cur; }
			// 目标格命中：八字邻接的最优性足够，直接收手
			break;
		}
		// 同层八邻
		for (const [dx, dy, w] of D8) {
			const ni = i + dx, nj = j + dy;
			if (ni < 0 || nj < 0 || ni >= nx || nj >= ny) continue;
			const nid = idxOf(ni, nj, li);
			if (blocked[nid] || closed[nid]) continue;
			// 斜走禁止切角：两个正交邻格必须都空
			if (dx !== 0 && dy !== 0) {
				if (blocked[idxOf(i + dx, j, li)] || blocked[idxOf(i, j + dy, li)]) continue;
			}
			const ng = g + w * cell;
			if (ng < gScore[nid] - 1e-9) {
				gScore[nid] = ng;
				cameFrom[nid] = cur;
				heap.push(ng + hOf(ni, nj), nid);
			}
		}
		// 层间过孔边（同格跨层）
		const here = idxOf(i, j, li);
		for (let l2 = 0; l2 < nLayers; l2++) {
			if (l2 === li) continue;
			const nid = idxOf(i, j, l2);
			if (blocked[nid] || closed[nid]) continue;
			// 过孔比走线粗：两侧层都必须在「过孔净距」网格上为空
			if (viaBlocked && (viaBlocked[nid] || viaBlocked[here])) continue;
			const ng = g + viaCost;
			if (ng < gScore[nid] - 1e-9) {
				gScore[nid] = ng;
				cameFrom[nid] = here;
				heap.push(ng + hOf(i, j), nid);
			}
		}
	}
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

	return { ok: prims.length > 0, prims, vias, cell };
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
