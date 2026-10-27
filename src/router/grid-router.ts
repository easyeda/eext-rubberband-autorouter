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
	srcPad?: { x: number; y: number; r: number };
	dstPad?: { x: number; y: number; r: number };
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
	const layerIndex = new Map<number, number>();
	req.layers.forEach((l, i) => layerIndex.set(l, i));

	const markCircle = (lx: number, ly: number, r: number, li: number): void => {
		const rr = r + inflate;
		const i0 = Math.max(0, Math.floor((lx - rr - x0) / cell));
		const i1 = Math.min(nx - 1, Math.ceil((lx + rr - x0) / cell));
		const j0 = Math.max(0, Math.floor((ly - rr - y0) / cell));
		const j1 = Math.min(ny - 1, Math.ceil((ly + rr - y0) / cell));
		const base = li * stride;
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
	};

	let marked = 0;
	for (const o of req.obstacles) {
		if (o.soft) continue;
		if (o.net && o.net === req.net) continue;
		const lis: number[] = [];
		if (o.layer === undefined) lis.push(...req.layers.map((_, i) => i));
		else {
			const li = layerIndex.get(o.layer);
			if (li !== undefined) lis.push(li);
		}
		if (!lis.length) continue;
		for (const li of lis) {
			if (o.kind === 'circle') {
				markCircle(o.x, o.y, o.r, li);
			} else {
				// 胶囊：沿线采样成圆
				const len = Math.hypot(o.x2! - o.x, o.y2! - o.y);
				const steps = Math.max(1, Math.ceil(len / (cell * 0.75)));
				for (let s = 0; s <= steps; s++) {
					const t = s / steps;
					markCircle(o.x + (o.x2! - o.x) * t, o.y + (o.y2! - o.y) * t, o.r, li);
				}
			}
		}
		if (++marked > 20000) break;   // 障碍过多保护：兜底求解不值得无限光栅化
	}

	// ── 端点松绑：起点/终点焊盘自身铜皮内一律视为可通行 ──
	// 这是网格兜底相对可视图的关键优势：邻盘净距圈覆盖焊盘中心时，
	// 可视图从中心出发无切点可用，网格仍能从焊盘内部任意格起步。
	const release = (pad: { x: number; y: number; r: number } | undefined, layerIds: number[]): void => {
		if (!pad) return;
		const r = pad.r + cell;
		for (const layerId of layerIds) {
			const li = layerIndex.get(layerId);
			if (li === undefined) continue;
			const base = li * stride;
			const i0 = Math.max(0, Math.floor((pad.x - r - x0) / cell));
			const i1 = Math.min(nx - 1, Math.ceil((pad.x + r - x0) / cell));
			const j0 = Math.max(0, Math.floor((pad.y - r - y0) / cell));
			const j1 = Math.min(ny - 1, Math.ceil((pad.y + r - y0) / cell));
			for (let i = i0; i <= i1; i++) {
				const dx = x0 + i * cell - pad.x;
				for (let j = j0; j <= j1; j++) {
					const dy = y0 + j * cell - pad.y;
					if (dx * dx + dy * dy <= r * r) blocked[base + j * nx + i] = 0;
				}
			}
		}
	};
	release(req.srcPad, req.layers);
	release(req.dstPad, req.layers);

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

	const idxOf = (i: number, j: number, li: number) => li * stride + j * nx + i;
	const hOf = (i: number, j: number) => Math.hypot(i - d.i, j - d.j) * cell;

	// 起点：允许多层（TH 焊盘）——取所有层中未被挡的起点格
	let started = 0;
	for (let li = 0; li < nLayers; li++) {
		const id = idxOf(s.i, s.j, li);
		if (blocked[id]) continue;
		gScore[id] = 0;
		heap.push(hOf(s.i, s.j), id);
		started++;
	}
	if (!started) return { ...fail, cell };

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
		if (i === d.i && j === d.j) {
			if (g < bestCost) { bestCost = g; best = cur; }
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
			const ng = g + viaCost;
			if (ng < gScore[nid] - 1e-9) {
				gScore[nid] = ng;
				cameFrom[nid] = here;
				heap.push(ng + hOf(i, j), nid);
			}
		}
	}
	if (best < 0) return { ...fail, cell };

	// ── 回溯 ──
	const cells: Cell[] = [];
	let cur = best;
	while (cur !== -1) {
		const li = Math.floor(cur / stride);
		const rem = cur - li * stride;
		const j = Math.floor(rem / nx);
		const i = rem - j * nx;
		cells.push({ x: x0 + i * cell, y: y0 + j * cell, layer: req.layers[li] });
		cur = cameFrom[cur];
	}
	cells.reverse();

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

	// ── 生成图元与过孔 ──
	const prims: GridPrim[] = [];
	const vias: GridVia[] = [];
	for (let k = 1; k < simplified.length; k++) {
		const a = simplified[k - 1], b = simplified[k];
		if (a.layer === b.layer) {
			if (a.x !== b.x || a.y !== b.y) {
				prims.push({ kind: 'line', layer: a.layer, x1: a.x, y1: a.y, x2: b.x, y2: b.y });
			}
		} else {
			vias.push({ x: a.x, y: a.y });
		}
	}
	// 首尾并入端点焊盘中心（避免网格量化在焊盘外留断口）
	if (simplified.length) {
		const first = simplified[0];
		if (Math.hypot(first.x - src.x, first.y - src.y) > cell) {
			prims.unshift({ kind: 'line', layer: first.layer, x1: src.x, y1: src.y, x2: first.x, y2: first.y });
		}
		const last = simplified[simplified.length - 1];
		if (Math.hypot(last.x - dst.x, last.y - dst.y) > cell) {
			prims.push({ kind: 'line', layer: last.layer, x1: last.x, y1: last.y, x2: dst.x, y2: dst.y });
		}
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
