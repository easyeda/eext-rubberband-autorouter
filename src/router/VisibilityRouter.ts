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
 * 性能：
 *  - 走廊剪枝（只保留直连线附近的圆，实心障碍 noPrune 除外）
 *  - blockerOnly 圆（填充内部网格）只参与阻挡判定，不生成切点/边
 *  - 均匀网格 + DDA 加速线段/圆弧的阻挡查询
 */
import {
	RBObstacle, RBPrimitive, getTangents, capsuleToCircles, pointSegDist, dist,
} from './RubberbandEngine';

export interface RouteConnectionResult {
	ok: boolean;
	/** 精确图元序列（直线 + 圆弧） */
	prims: RBPrimitive[];
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
 * 均匀网格：按圆心插入；查询与线段/点可能相交的圆下标。
 * 用扁平数组 + 戳记法避免每查询分配 Set / Map 哈希。
 */
class CircleGrid {
	private buckets: number[][] = [];
	private stamp: Int32Array;
	private curStamp = 0;
	private circles: RBObstacle[];
	private key(gx: number, gy: number): number { return (gx + 2048) * 4096 + (gy + 2048); }

	constructor(circles: RBObstacle[]) {
		this.circles = circles;
		for (let i = 0; i < circles.length; i++) {
			const c = circles[i];
			const key = this.key(Math.floor(c.x / GRID_CELL), Math.floor(c.y / GRID_CELL));
			let arr = this.buckets[key];
			if (!arr) { arr = []; this.buckets[key] = arr; }
			arr.push(i);
		}
		this.stamp = new Int32Array(this.buckets.length);
	}

	/** 遍历线段附近（距离 < 半径可达）的所有圆下标（可能重复） */
	nearSegment(x1: number, y1: number, x2: number, y2: number, cb: (idx: number) => void): void {
		const stampId = ++this.curStamp;
		const dist = Math.hypot(x2 - x1, y2 - y1);
		const steps = Math.max(1, Math.ceil(dist / (GRID_CELL * 0.8)));
		for (let s = 0; s <= steps; s++) {
			const t = s / steps;
			const px = x1 + (x2 - x1) * t, py = y1 + (y2 - y1) * t;
			const gx = Math.floor(px / GRID_CELL), gy = Math.floor(py / GRID_CELL);
			for (let dx = -1; dx <= 1; dx++) for (let dy = -1; dy <= 1; dy++) {
				const key = this.key(gx + dx, gy + dy);
				if (this.stamp[key] === stampId) continue;
				this.stamp[key] = stampId;
				const arr = this.buckets[key];
				if (arr) for (let k = 0; k < arr.length; k++) cb(arr[k]);
			}
		}
	}

	/** 遍历点附近的所有圆下标 */
	nearPoint(px: number, py: number, cb: (idx: number) => void): void {
		const gx = Math.floor(px / GRID_CELL), gy = Math.floor(py / GRID_CELL);
		for (let dx = -1; dx <= 1; dx++) for (let dy = -1; dy <= 1; dy++) {
			const arr = this.buckets[this.key(gx + dx, gy + dy)];
			if (arr) for (let k = 0; k < arr.length; k++) cb(arr[k]);
		}
	}
}

/**
 * 线段是否不侵入任何圆（快速网格查询；touchedA/B 为允许相切的圆下标）
 */
function makeSegmentChecker(circles: RBObstacle[], grid: CircleGrid, net = '') {
	return (x1: number, y1: number, x2: number, y2: number, touchedA: number, touchedB: number): boolean => {
		let blocked = false;
		grid.nearSegment(x1, y1, x2, y2, (k) => {
			if (blocked || k === touchedA || k === touchedB) return;
			const c = circles[k];
			if (net && c.net === net) return;
			if (pointSegDist(c.x, c.y, x1, y1, x2, y2) < c.r - EPS_TOUCH) blocked = true;
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
		for (let s = 0; s <= steps && !blocked; s++) {
			const a = a1 + (sweep * s) / steps;
			const px = cx + r * Math.cos(a), py = cy + r * Math.sin(a);
			grid.nearPoint(px, py, (k) => {
				if (blocked || k === selfIdx) return;
				const o = circles[k];
				if (net && o.net === net) return;
				if (Math.hypot(px - o.x, py - o.y) < o.r - EPS_TOUCH) blocked = true;
			});
		}
		return !blocked;
	};
}

/**
 * 计算单条连接（src→dst）的橡皮筋路径（可视图 Dijkstra）
 * obstacles: 障碍（圆或胶囊；胶囊转为圆群）。网络/层过滤由调用方完成。
 */
export function routeConnection(
	src: { x: number; y: number },
	dst: { x: number; y: number },
	obstacles: RBObstacle[],
	net = '',
	expands: number[] = [150, 500, Infinity],
): RouteConnectionResult {
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

	// 走廊剪枝：先只保留直连线附近窄带内的圆（大幅提速），
	// 求解失败再逐级放宽范围重试。
	// noPrune（填充/板框等实心障碍）始终保留——否则窄走廊会把内部网格圆剪掉，
	// 导致"穿过填充"的假合法路径。
	for (const expand of expands) {
		let subset = all;
		if (expand !== Infinity) {
			subset = all.filter(c =>
				c.noPrune ||
				pointSegDist(c.x, c.y, src.x, src.y, dst.x, dst.y) < c.r + expand ||
				dist(c.x, c.y, src.x, src.y) < c.r + expand ||
				dist(c.x, c.y, dst.x, dst.y) < c.r + expand);
		}
		const res = solve(src, dst, subset, net);
		if (res.ok && validatePrimsAgainst(res.prims, all, net)) return res;
	}
	return { ok: false, prims: [] };
}

/** 用全量障碍复验路径图元（防止走廊剪枝造成的假合法路径） */
function validatePrimsAgainst(prims: RBPrimitive[], all: RBObstacle[], net = ''): boolean {
	const grid = new CircleGrid(all);
	const segOK = makeSegmentChecker(all, grid, net);
	const arcOK = makeArcChecker(all, grid, net);
	for (let i = 0; i < prims.length; i++) {
		const pr = prims[i];
		if (pr.kind === 'line') {
			if (!segOK(pr.x1, pr.y1, pr.x2, pr.y2, -1, -1)) {
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
			if (!arcOK(pr.cx, pr.cy, pr.r, pr.startAngle, pr.sweep, -1)) {
				if ((globalThis as any).__RB_VERBOSE__) console.log(`[VR] 弧被拒 #${i}: c=(${pr.cx.toFixed(0)},${pr.cy.toFixed(0)}) r=${pr.r.toFixed(1)}`);
				return false;
			}
		}
	}
	return true;
}

/** 对给定圆集合求解橡皮筋路径 */
function solve(
	src: { x: number; y: number },
	dst: { x: number; y: number },
	obstacles: RBObstacle[],
	net = '',
): RouteConnectionResult {
	// 拆分：可绕行圆（建图）与纯阻挡圆（填充内部网格，只参与碰撞检查）
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
	// 圆下标在 everything 中的偏移：circles 下标一致，blockers 偏移 n
	// （下面的 touched 下标均相对 everything）

	const nodes: VGNode[] = [];
	// node 0 = src, node 1 = dst
	nodes.push({ circle: -1, x: src.x, y: src.y, angle: 0, adj: [] });
	nodes.push({ circle: -1, x: dst.x, y: dst.y, angle: 0, adj: [] });

	// 切点去重：键 = 圆下标 + 角度量化
	const ptIndex = new Map<string, number>();
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
		return idx;
	};

	// 1. 圆-圆公切线（4 种侧别组合）
	for (let i = 0; i < n; i++) {
		const ci = circles[i];
		if (net && ci.net === net) continue;
		for (let j = i + 1; j < n; j++) {
			const cj = circles[j];
			const d = dist(ci.x, ci.y, cj.x, cj.y);
			for (const [l1, l2] of [[true, true], [true, false], [false, true], [false, false]] as const) {
				const adjR2 = cj.r * (l1 === l2 ? 1 : -1);
				const cVal = (ci.r - adjR2) / d;
				if (Math.abs(cVal) > 1) continue; // 该类型切线不存在
				const [tx1, ty1, tx2, ty2] = getTangents(ci.x, ci.y, ci.r, l1, cj.x, cj.y, cj.r, l2);
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

	// 2. 终端 ↔ 圆切线
	const connectTerminal = (termIdx: number) => {
		const term = nodes[termIdx];
		for (let i = 0; i < n; i++) {
			const ci = circles[i];
			if (net && ci.net === net) continue;
			if (dist(term.x, term.y, ci.x, ci.y) < ci.r - EPS_TOUCH) continue; // 终端在圆内
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
	connectTerminal(0);
	connectTerminal(1);

	// 3. 直连（无障碍阻挡时）
	if (segOK(src.x, src.y, dst.x, dst.y, -1, -1)) {
		nodes[0].adj.push({ to: 1, cost: dist(src.x, src.y, dst.x, dst.y), isArc: false, x1: src.x, y1: src.y, x2: dst.x, y2: dst.y });
	}

	// 4. 同圆弧边：切点按角度排序，相邻切点之间双向连弧（弧几何合法即建边）
	for (let i = 0; i < n; i++) {
		const pts: number[] = [];
		for (const [key, idx] of ptIndex) {
			if (key.startsWith(`${i}:`)) pts.push(idx);
		}
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

	// 5. Dijkstra（二叉堆，O(E log V)）
	const total = nodes.length;
	const distArr = new Array<number>(total).fill(Infinity);
	const prevNode = new Array<number>(total).fill(-1);
	const prevEdge = new Array<VGEdge | null>(total).fill(null);
	const settled = new Array<boolean>(total).fill(false);
	distArr[0] = 0;
	const heap: Array<[number, number]> = [[0, 0]];
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
	for (;;) {
		const top = heapPop();
		if (!top) break;
		const [du, u] = top;
		if (settled[u]) continue;
		settled[u] = true;
		if (u === 1) break;
		for (const e of nodes[u].adj) {
			if (settled[e.to]) continue;
			const nd = du + e.cost;
			if (nd < distArr[e.to] - 1e-9) {
				distArr[e.to] = nd;
				prevNode[e.to] = u;
				prevEdge[e.to] = e;
				heapPush(nd, e.to);
			}
		}
	}

	if (!isFinite(distArr[1])) {
		if ((globalThis as any).__RB_VERBOSE__) {
			const seen = new Array<boolean>(total).fill(false);
			const stack = [0];
			seen[0] = true;
			while (stack.length) {
				const u = stack.pop()!;
				for (const e of nodes[u].adj) if (!seen[e.to]) { seen[e.to] = true; stack.push(e.to); }
			}
			// 双向可达集 + 边界包围盒：定位堵点
			const seen2 = new Array<boolean>(total).fill(false);
			const stack2 = [1];
			seen2[1] = true;
			while (stack2.length) {
				const u = stack2.pop()!;
				for (const e of nodes[u].adj) if (!seen2[e.to]) { seen2[e.to] = true; stack2.push(e.to); }
			}
			const bbox = (pred: (k: number) => boolean) => {
				let mnx = Infinity, mny = Infinity, mxx = -Infinity, mxy = -Infinity, cnt = 0;
				for (let k = 0; k < total; k++) {
					if (!pred(k) || nodes[k].circle < 0) continue;
					cnt++;
					mnx = Math.min(mnx, nodes[k].x); mxx = Math.max(mxx, nodes[k].x);
					mny = Math.min(mny, nodes[k].y); mxy = Math.max(mxy, nodes[k].y);
				}
				return cnt ? `n=${cnt} x[${mnx.toFixed(0)},${mxx.toFixed(0)}] y[${mny.toFixed(0)},${mxy.toFixed(0)}]` : '空';
			};
			console.log(`[VR] 无路径: wrappable=${n} blockers=${blockers.length} nodes=${total} src出边=${nodes[0].adj.length} dst出边=${nodes[1].adj.length}`);
			console.log(`[VR]   src分量: ${bbox(k => seen[k])}`);
			console.log(`[VR]   dst分量: ${bbox(k => seen2[k])}`);
			// 两分量之间的"缝隙"障碍：与 src 分量的节点相切距离近、且自身不在任一分量的切点中
			const srcSet = new Set<number>(), dstSet = new Set<number>();
			for (let k = 0; k < total; k++) { if (seen[k]) srcSet.add(k); if (seen2[k]) dstSet.add(k); }
			const seam = new Map<string, number>();
			for (let k = 0; k < total; k++) {
				if (!seen[k] && !seen2[k]) continue;
				const nd = nodes[k];
				if (nd.circle < 0) continue;
				for (const e of nd.adj) {
					const other = nodes[e.to];
					if (seen[k] !== seen2[e.to] && seen[k] !== seen[e.to]) {
						const c = circles[nd.circle];
						const key = `${c.group ?? c.id}`;
						seam.set(key, (seam.get(key) ?? 0) + 1);
					}
				}
			}
			console.log(`[VR]   缝隙障碍: ${[...seam.entries()].slice(0, 8).map(([k, v]) => k + 'x' + v).join(', ')}`);
		}
		return { ok: false, prims: [] };
	}

	// 6. 回溯路径 → 精确图元（边自带几何，无需重算）
	if ((globalThis as any).__RB_VERBOSE__ && prevNode[1] === -1) {
		console.log(`[VR] dist[1]=${distArr[1]} 但 prevNode[1]=-1（异常）`);
	}
	const prims: RBPrimitive[] = [];
	{
		const revEdges: VGEdge[] = [];
		for (let cur = 1; cur !== 0; cur = prevNode[cur]) {
			const e = prevEdge[cur];
			if (!e) break;
			revEdges.push(e);
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

	return { ok: true, prims };
}
