/**
 * 可视图求解器：加速后的线段/圆弧阻挡判定 vs 暴力线性扫描（原始谓词）对拍
 * 运行方式：npx tsx test/routing-vis-test.ts
 *
 * 覆盖：多尺寸圆、多网络/多层、soft / forceBlock / bulge、同网重叠圆链（链容差边界）、
 * touchedA/B、零长/全 NaN 线段、圆弧（含 selfIdx）、稀疏退回（legacy）网格。
 */
import { __vrTest } from '../src/router/visibility-router';
import { pointSegDist, type RBObstacle } from '../src/router/rubberband-engine';

const {
	CircleGrid, makeSegmentChecker, makeArcChecker, chainTolFor, EPS_TOUCH,
	canonicalSort, canonicalCmpOld, findSurfaceCircle, pruneByTube, corridorSubset, orderByX, coarseTube, hypotFast, HYPOT_FAST_OK,
} = __vrTest;

let passed = 0;
let failed = 0;
function assert(condition: boolean, message: string): void {
	if (condition) { console.log(`  ok  ${message}`); passed++; }
	else { console.error(`  FAIL ${message}`); failed++; }
}

// ─── 确定性随机数（mulberry32）───
function rng(seed: number) {
	let a = seed >>> 0;
	return () => {
		a = (a + 0x6D2B79F5) >>> 0;
		let t = a;
		t = Math.imul(t ^ (t >>> 15), t | 1);
		t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
		return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
	};
}

// ─── 原始谓词（暴力版，逐字对应旧 makeSegmentChecker / makeArcChecker 的回调体）───
const effR = (c: RBObstacle) => c.r - (c.bulge ?? 0);

function refSegOK(
	circles: RBObstacle[], net: string, noChain: boolean,
	x1: number, y1: number, x2: number, y2: number, touchedA: number, touchedB: number,
): boolean {
	const tA = touchedA >= 0 ? circles[touchedA] : undefined;
	const tB = touchedB >= 0 ? circles[touchedB] : undefined;
	for (let k = 0; k < circles.length; k++) {
		if (k === touchedA || k === touchedB) continue;
		const c = circles[k];
		if (c.soft) continue;
		if (net && c.net === net && !c.forceBlock) continue;
		const tol = noChain ? 0 : Math.max(chainTolFor(c, tA), chainTolFor(c, tB));
		if (pointSegDist(c.x, c.y, x1, y1, x2, y2) < effR(c) - EPS_TOUCH - tol) return false;
	}
	return true;
}

const GRID_CELL_TEST = 100;
function refArcOK(
	circles: RBObstacle[], net: string,
	cx: number, cy: number, r: number, a1: number, sweep: number, selfIdx: number,
	legacyCellOnly = false,
): boolean {
	const steps = Math.max(4, Math.ceil(Math.abs(sweep) * r / (GRID_CELL_TEST * 0.5)));
	const self = selfIdx >= 0 ? circles[selfIdx] : undefined;
	for (let s = 0; s <= steps; s++) {
		const a = a1 + (sweep * s) / steps;
		const px = cx + r * Math.cos(a), py = cy + r * Math.sin(a);
		for (let k = 0; k < circles.length; k++) {
			if (k === selfIdx) continue;
			const o = circles[k];
			// 旧 legacy 稀疏桶按圆心入桶、点查询只看自己所在格：圆心不在同格的圆不参与
			if (legacyCellOnly && (Math.floor(o.x / GRID_CELL_TEST) !== Math.floor(px / GRID_CELL_TEST) ||
				Math.floor(o.y / GRID_CELL_TEST) !== Math.floor(py / GRID_CELL_TEST))) continue;
			if (o.soft) continue;
			if (net && o.net === net && !o.forceBlock) continue;
			const tol = chainTolFor(o, self);
			if (Math.hypot(px - o.x, py - o.y) < o.r - (o.bulge ?? 0) - EPS_TOUCH - tol) return false;
		}
	}
	return true;
}

// ─── 数据生成 ───
interface Board { circles: RBObstacle[]; chains: number[][]; }

function genBoard(seed: number, nCircles: number, W: number, H: number, rMax: number): Board {
	const R = rng(seed);
	const circles: RBObstacle[] = [];
	const chains: number[][] = [];
	let id = 0;
	const NETS = 14;
	// 1) 单圆：焊盘 / 过孔 / 填充网格圆 / soft 辅助圆
	const nSingles = Math.floor(nCircles * 0.6);
	for (let i = 0; i < nSingles; i++) {
		const o: RBObstacle = {
			id: `s${id++}`, kind: 'circle',
			x: R() * W, y: R() * H,
			r: R() < 0.7 ? 3 + R() * 20 : 3 + R() * rMax,
		};
		const u = R();
		if (u < 0.75) o.net = `N${Math.floor(R() * NETS)}`;
		const lu = R();
		if (lu < 0.4) o.layer = 0; else if (lu < 0.8) o.layer = 1;   // 其余 undefined = 全层
		if (R() < 0.06) o.soft = true;
		if (R() < 0.12) o.forceBlock = true;
		if (R() < 0.2) o.bulge = R() * 0.4;
		if (R() < 0.03) o.bulge = o.r + 1;   // 阈值 ≤ 0：永远挡不住
		circles.push(o);
	}
	// 2) 重叠同网圆链（已布走线拆成的圆链），含 bulge
	while (circles.length < nCircles) {
		const net = `N${Math.floor(R() * NETS)}`;
		const layer = R() < 0.5 ? 0 : 1;
		const r = 6 + R() * 8;
		let x = R() * W, y = R() * H, ang = R() * Math.PI * 2;
		const len = 8 + Math.floor(R() * 14);
		const ids: number[] = [];
		for (let k = 0; k < len && circles.length < nCircles; k++) {
			circles.push({
				id: `c${id++}`, kind: 'circle', x, y, r, net, layer,
				bulge: R() < 0.7 ? 0.05 + R() * 0.3 : undefined,
				forceBlock: R() < 0.05 ? true : undefined,
			});
			ids.push(circles.length - 1);
			ang += (R() - 0.5) * 0.9;
			x += Math.cos(ang) * r * (0.5 + R() * 0.4);
			y += Math.sin(ang) * r * (0.5 + R() * 0.4);
		}
		chains.push(ids);
	}
	// 3) 几对重合圆（同位置同半径，不同网）
	for (let k = 0; k < 6; k++) {
		const c = circles[Math.floor(R() * circles.length)];
		circles.push({ ...c, id: `d${id++}`, net: R() < 0.5 ? undefined : `N${Math.floor(R() * NETS)}` });
	}
	return { circles, chains };
}

function pointOn(c: RBObstacle, a: number) { return { x: c.x + c.r * Math.cos(a), y: c.y + c.r * Math.sin(a) }; }

interface Q { x1: number; y1: number; x2: number; y2: number; tA: number; tB: number; }

function genSegments(board: Board, seed: number, count: number, W: number, H: number): Q[] {
	const R = rng(seed);
	const { circles, chains } = board;
	const qs: Q[] = [];
	const pickTouched = () => (R() < 0.6 ? -1 : Math.floor(R() * circles.length));
	for (let i = 0; i < count; i++) {
		const u = R();
		if (u < 0.4) {
			// 随机线段：短段居多
			const x1 = R() * W, y1 = R() * H;
			const L = R() < 0.7 ? 5 + R() * 120 : 100 + R() * 700;
			const a = R() * Math.PI * 2;
			qs.push({ x1, y1, x2: x1 + L * Math.cos(a), y2: y1 + L * Math.sin(a), tA: pickTouched(), tB: pickTouched() });
		} else if (u < 0.6) {
			// 圆表面点 → 圆表面点（切线形态），端点所在圆即 touched
			const a = Math.floor(R() * circles.length);
			const j = R() < 0.6 ? Math.floor(R() * circles.length) : (a + 1 + Math.floor(R() * 5)) % circles.length;
			const p = pointOn(circles[a], R() * Math.PI * 2), q = pointOn(circles[j], R() * Math.PI * 2);
			qs.push({ x1: p.x, y1: p.y, x2: q.x, y2: q.y, tA: a, tB: j });
		} else if (u < 0.92) {
			// 链容差边界：短线段擦过某链圆 k 的外侧，距离 ≈ 阈值 ± 0.08；touched = 与 k 同链的邻圆
			const ch = chains[Math.floor(R() * chains.length)];
			const ki = Math.floor(R() * ch.length);
			const k = circles[ch[ki]];
			const t = ch[Math.max(0, Math.min(ch.length - 1, ki + (R() < 0.5 ? -1 : 1)))];
			const pa = circles[ch[Math.max(0, ki - 1)]], pb = circles[ch[Math.min(ch.length - 1, ki + 1)]];
			let phi: number;
			if (R() < 0.7) {
				// 链的法向（两侧随机）+ 小抖动：此处邻圆够不着，只有 k 自身边界起作用
				phi = Math.atan2(pb.y - pa.y, pb.x - pa.x) + (R() < 0.5 ? 1 : -1) * Math.PI / 2 + (R() - 0.5) * 0.6;
			} else phi = R() * Math.PI * 2;
			const thr = effR(k) - EPS_TOUCH;
			const delta = -0.08 + R() * 0.12;
			const nx = Math.cos(phi), ny = Math.sin(phi);
			const px = k.x + (thr + delta) * nx, py = k.y + (thr + delta) * ny;
			const L1 = 1 + R() * 8, L2 = 1 + R() * 8;
			qs.push({
				x1: px - ny * L1, y1: py + nx * L1, x2: px + ny * L2, y2: py - nx * L2,
				tA: t, tB: R() < 0.5 ? -1 : ch[Math.floor(R() * ch.length)],
			});
		} else if (u < 0.96) {
			// 零长线段
			const x = R() * W, y = R() * H;
			qs.push({ x1: x, y1: y, x2: x, y2: y, tA: -1, tB: -1 });
		} else if (u < 0.98) {
			qs.push({ x1: NaN, y1: NaN, x2: NaN, y2: NaN, tA: -1, tB: -1 });   // 重合等半径圆的退化切线
		} else {
			// 水平/竖直线段（DDA 恰穿格点的退化情形）
			const gx = Math.floor(R() * (W / 100)) * 100, gy = Math.floor(R() * (H / 100)) * 100;
			if (R() < 0.5) qs.push({ x1: gx, y1: gy + R() * 100, x2: gx + 300 * R(), y2: gy + R() * 100, tA: -1, tB: -1 });
			else qs.push({ x1: gx, y1: gy, x2: gx + 200 * (R() < 0.5 ? 1 : -1), y2: gy + 200 * (R() < 0.5 ? 1 : -1), tA: -1, tB: -1 });
		}
	}
	return qs;
}

function runSegments(label: string, board: Board, net: string, qs: Q[]): void {
	const grid = new CircleGrid(board.circles, net);
	const segOK = makeSegmentChecker(grid);
	let bad = 0, blocked = 0, tolMattered = 0, firstBad = '';
	for (let i = 0; i < qs.length; i++) {
		const q = qs[i];
		const want = refSegOK(board.circles, net, false, q.x1, q.y1, q.x2, q.y2, q.tA, q.tB);
		const got = segOK(q.x1, q.y1, q.x2, q.y2, q.tA, q.tB);
		if (!want) blocked++;
		if (want !== got) { bad++; if (!firstBad) firstBad = JSON.stringify(q); }
		else if (want && !refSegOK(board.circles, net, true, q.x1, q.y1, q.x2, q.y2, q.tA, q.tB)) tolMattered++;
	}
	assert(bad === 0, `${label}: 线段 ${qs.length} 条与暴力扫描一致（挡死 ${blocked}，链容差起作用 ${tolMattered}）${firstBad ? ' 首个不符 ' + firstBad : ''}`);
	assert(blocked > qs.length * 0.05 && blocked < qs.length * 0.95, `${label}: 样本里挡死/放行两类都有`);
}

function genArcs(board: Board, seed: number, count: number, W: number, H: number) {
	const R = rng(seed);
	const { circles } = board;
	const arcs: Array<{ cx: number; cy: number; r: number; a1: number; sweep: number; self: number }> = [];
	for (let i = 0; i < count; i++) {
		if (R() < 0.7) {
			// 绕某个圆的贴边弧（selfIdx = 该圆），半径取该圆半径
			const s = Math.floor(R() * circles.length);
			const c = circles[s];
			arcs.push({ cx: c.x, cy: c.y, r: c.r, a1: R() * 6.4 - 3.2, sweep: (R() < 0.5 ? -1 : 1) * (0.1 + R() * 3), self: s });
		} else {
			arcs.push({
				cx: R() * W, cy: R() * H, r: 3 + R() * 60, a1: R() * 6.4 - 3.2,
				sweep: (R() - 0.5) * 6, self: R() < 0.5 ? -1 : Math.floor(R() * circles.length),
			});
		}
	}
	return arcs;
}

function runArcs(label: string, board: Board, net: string, W: number, H: number, legacy = false): void {
	const grid = new CircleGrid(board.circles, net);
	const arcOK = makeArcChecker(grid);
	const arcs = genArcs(board, 777 + net.length, 600, W, H);
	let bad = 0, blocked = 0;
	for (const a of arcs) {
		const want = refArcOK(board.circles, net, a.cx, a.cy, a.r, a.a1, a.sweep, a.self, legacy);
		const got = arcOK(a.cx, a.cy, a.r, a.a1, a.sweep, a.self);
		if (!want) blocked++;
		if (want !== got) bad++;
	}
	assert(bad === 0, `${label}: 圆弧 ${arcs.length} 条与暴力扫描一致（挡死 ${blocked}）`);
	assert(blocked > 20 && blocked < arcs.length - 20, `${label}: 圆弧样本里挡死/放行两类都有`);
}

const t0 = Date.now();
{
	console.log('\n主网格（非 legacy）：2200 圆');
	const W = 4000, H = 2600;
	const board = genBoard(12345, 2200, W, H, 90);
	const qs = genSegments(board, 99, 1800, W, H);
	for (const net of ['', 'N3', 'N11']) {
		runSegments(`net='${net}'`, board, net, qs);
		runArcs(`net='${net}'`, board, net, W, H);
	}
	// 同一个网格实例上连续多次查询（最近挡路者缓存）也必须一致：反复重放同一批并打乱顺序
	const grid = new CircleGrid(board.circles, 'N3');
	const segOK = makeSegmentChecker(grid);
	const R = rng(5);
	let bad = 0;
	for (let rep = 0; rep < 2; rep++) {
		for (let i = 0; i < 1500; i++) {
			const q = qs[Math.floor(R() * qs.length)];
			if (segOK(q.x1, q.y1, q.x2, q.y2, q.tA, q.tB) !== refSegOK(board.circles, 'N3', false, q.x1, q.y1, q.x2, q.y2, q.tA, q.tB)) bad++;
		}
	}
	assert(bad === 0, '乱序重放同一网格实例（缓存不得改变答案）');
}
{
	console.log('\n小场景（稠密，多重叠）：400 圆');
	const W = 500, H = 400;
	const board = genBoard(2024, 400, W, H, 60);
	const qs = genSegments(board, 7, 1200, W, H);
	runSegments("net='N5'", board, 'N5', qs);
	runArcs("net='N5'", board, 'N5', W, H);
}
{
	console.log('\nlegacy 稀疏退回（含离群点，坐标跨度巨大）');
	const W = 1500, H = 1000;
	const board = genBoard(31337, 500, W, H, 45);
	board.circles.push({ id: 'outlier', kind: 'circle', x: 5e5, y: 5e5, r: 10, net: 'N1' });
	const qs = genSegments(board, 11, 800, W, H);
	runSegments("legacy net='N1'", board, 'N1', qs);
	runArcs("legacy net='N1'", board, 'N1', W, H, true);
}
{
	console.log('\n边界');
	const grid0 = new CircleGrid([], 'X');
	assert(makeSegmentChecker(grid0)(0, 0, 100, 100, -1, -1) === true, '空网格线段放行');
	assert(makeArcChecker(grid0)(0, 0, 10, 0, 1, -1) === true, '空网格圆弧放行');
	const c: RBObstacle[] = [{ id: 'a', kind: 'circle', x: 50, y: 50, r: 10 }];
	const g = new CircleGrid(c, '');
	const ok = makeSegmentChecker(g);
	assert(ok(0, 50, 100, 50, -1, -1) === false, '穿过圆心被挡');
	assert(ok(0, 50, 100, 50, 0, -1) === true, 'touchedA 放行');
	assert(ok(0, 60, 100, 60, -1, -1) === true, '距离=r 的相切线放行（EPS_TOUCH）');
	assert(ok(0, 59, 100, 59, -1, -1) === false, '侵入 1mil 被挡');
}

// ─────────────────────────────────────────────────────────────
// 以下：routeBetweenEndpoints 逐调用开销优化的等价性（旧实现逐字复制为参照）
// ─────────────────────────────────────────────────────────────

/** 旧 coarseTube（逐字复制，仅改名）：粗网格走廊探测参照 */
function refCoarseTube(
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


function sameRef<T>(a: T[], b: T[]): boolean {
	if (a.length !== b.length) return false;
	for (let i = 0; i < a.length; i++) if (a[i] !== b[i]) return false;
	return true;
}

{
	console.log('\n规范排序 canonicalSort ≡ 旧比较器稳定排序（含重复坐标/同坐标不同网/NaN）');
	const NETN = ['', 'N1', 'N10', 'N2', 'n2', 'ä', 'a', 'B', 'GND', 'VCC', '网络一', '网络二'];
	const mk = (R: () => number, n: number, grid: number, nanRate: number): RBObstacle[] => {
		const out: RBObstacle[] = [];
		for (let i = 0; i < n; i++) {
			const o: RBObstacle = {
				id: `o${i}`, kind: 'circle',
				x: Math.floor(R() * grid) * 2.5, y: Math.floor(R() * grid) * 2.5, r: [1, 2, 2, 3.5, 3.5, 3.5, 8][Math.floor(R() * 7)],
			};
			const nn = NETN[Math.floor(R() * NETN.length)];
			if (nn) o.net = nn;
			if (R() < nanRate) { const f = Math.floor(R() * 3); if (f === 0) o.x = NaN; else if (f === 1) o.y = NaN; else o.r = NaN; }
			out.push(o);
		}
		return out;
	};
	const R = rng(4242);
	let bad = 0, cases = 0, tiesSeen = 0;
	for (const [n, grid, nan] of [[0, 3, 0], [1, 3, 0], [2, 2, 0], [60, 3, 0], [400, 6, 0], [1500, 12, 0], [1500, 600, 0], [300, 5, 0.01], [300, 5, 0.3]] as Array<[number, number, number]>) {
		for (let rep = 0; rep < 6; rep++) {
			const list = mk(R, n, grid, nan);
			const want = list.slice().sort(canonicalCmpOld);
			const got = canonicalSort(list.slice());
			cases++;
			if (!sameRef(want, got)) bad++;
			for (let i = 1; i < want.length; i++) if (want[i].x === want[i - 1].x && want[i].y === want[i - 1].y && want[i].r === want[i - 1].r) tiesSeen++;
		}
	}
	// ±0、负坐标、大数量级
	const odd: RBObstacle[] = [
		{ id: 'a', kind: 'circle', x: 0, y: 0, r: 1 }, { id: 'b', kind: 'circle', x: -0, y: 0, r: 1, net: 'N1' },
		{ id: 'c', kind: 'circle', x: 0, y: -0, r: 1, net: 'N1' }, { id: 'd', kind: 'circle', x: -5, y: 1e12, r: 1 },
		{ id: 'e', kind: 'circle', x: 1e-300, y: -1e300, r: 1 }, { id: 'f', kind: 'circle', x: Infinity, y: 0, r: 1 },
		{ id: 'g', kind: 'circle', x: -Infinity, y: 0, r: 1 }, { id: 'h', kind: 'circle', x: 0, y: 0, r: 1, net: '' },
	];
	for (let rep = 0; rep < 20; rep++) {
		const list = odd.slice(0, rep < 10 ? 5 : 8).sort(() => R() - 0.5);
		cases++;
		if (!sameRef(list.slice().sort(canonicalCmpOld), canonicalSort(list.slice()))) bad++;
	}
	assert(bad === 0, `canonicalSort 与旧排序结果逐元素一致（${cases} 组，其中同 (x,y,r) 的相邻对 ${tiesSeen}）`);
	assert(tiesSeen > 200, 'canonicalSort 样本里确有大量 (x,y,r) 全相同的并列元素');
}

{
	console.log('\norderByX ≡ idx.sort(旧比较器)');
	const R = rng(99);
	let bad = 0, cases = 0;
	for (const [n, grid, sortedIn, nanRate] of [[0, 5, false, 0], [1, 5, false, 0], [50, 4, false, 0], [374, 30, false, 0], [374, 30, true, 0], [374, 3, true, 0], [200, 9, false, 0.05]] as Array<[number, number, boolean, number]>) {
		for (let rep = 0; rep < 8; rep++) {
			const xs = new Float64Array(n);
			for (let i = 0; i < n; i++) xs[i] = R() < nanRate ? NaN : Math.floor(R() * grid) * 1.5;
			if (sortedIn) xs.sort();
			const want = Array.from({ length: n }, (_, i) => i).sort((a, b) => xs[a] - xs[b]);
			const got = orderByX(xs);
			cases++;
			if (!sameRef(want, got)) bad++;
		}
	}
	assert(bad === 0, `orderByX 与旧排序一致（${cases} 组，含有序输入/大量重复/NaN）`);
}

{
	console.log('\nfindSurface ≡ findSurfaceCircle 线性扫描（最小下标）');
	const R = rng(777);
	let bad = 0, queries = 0, hits = 0, multi = 0;
	for (let rep = 0; rep < 12; rep++) {
		const n = 5 + Math.floor(R() * 600);
		const cs: RBObstacle[] = [];
		for (let i = 0; i < n; i++) {
			const c: RBObstacle = { id: `s${i}`, kind: 'circle', x: Math.floor(R() * 40) * 10, y: Math.floor(R() * 30) * 10, r: 1 + Math.floor(R() * 12) * 2.5 };
			if (R() < 0.02) c.r = -3;
			if (R() < 0.02) c.r = NaN;
			if (R() < 0.02) c.x = NaN;
			cs.push(c);
		}
		const grid = new CircleGrid(cs, rep % 2 ? 'N1' : '');
		for (let q = 0; q < 1500; q++) {
			let x: number, y: number;
			const u = R();
			if (u < 0.5) {   // 恰在某圆表面（含 ±eps 抖动），重合圆会有多个解
				const c = cs[Math.floor(R() * n)];
				const a = R() * 6.3, jit = (R() - 0.5) * 0.14;
				x = c.x + (c.r + jit) * Math.cos(a); y = c.y + (c.r + jit) * Math.sin(a);
			} else if (u < 0.9) { x = R() * 420; y = R() * 320; }
			else { x = [NaN, Infinity, 0, -0, 1e9][Math.floor(R() * 5)]; y = R() * 100; }
			const want = findSurfaceCircle(cs, x, y);
			const got = grid.findSurface(x, y);
			queries++;
			if (want >= 0) hits++;
			if (want > 0) multi++;
			if (want !== got) bad++;
		}
	}
	assert(bad === 0, `findSurface 与线性扫描一致（${queries} 次查询，命中 ${hits}，下标>0 的命中 ${multi}）`);
	assert(hits > 1500 && hits < queries - 1500, 'findSurface 样本里命中/未命中两类都有');
}

{
	console.log('\n管道/走廊剪枝 ≡ 旧逐圆循环');
	const R = rng(2468);
	let bad = 0, cases = 0, nonTrivial = 0;
	for (let rep = 0; rep < 40; rep++) {
		const n = 20 + Math.floor(R() * 900);
		const cs: RBObstacle[] = [];
		for (let i = 0; i < n; i++) {
			const c: RBObstacle = { id: `p${i}`, kind: 'circle', x: R() * 3000, y: R() * 2000, r: 2 + R() * (R() < 0.1 ? 120 : 12) };
			if (R() < 0.25) c.noPrune = true;
			cs.push(c);
		}
		// 管道折线：网格点上的折线（含零长段、共线点），长短不一
		const m = 2 + Math.floor(R() * 25);
		const tube: Array<{ x: number; y: number }> = [];
		let tx = Math.floor(R() * 180) * 16, ty = Math.floor(R() * 120) * 16;
		for (let k = 0; k < m; k++) {
			tube.push({ x: tx, y: ty });
			if (R() > 0.1) { tx += (Math.floor(R() * 5) - 2) * 16 * (1 + Math.floor(R() * 6)); ty += (Math.floor(R() * 5) - 2) * 16 * (1 + Math.floor(R() * 6)); }
		}
		const wantKept: RBObstacle[] = [], wantPicked: RBObstacle[] = [];
		for (const c of cs) {
			if (c.noPrune) { wantKept.push(c); continue; }
			let best = Infinity;
			for (let k = 1; k < tube.length; k++) {
				const d = pointSegDist(c.x, c.y, tube[k - 1].x, tube[k - 1].y, tube[k].x, tube[k].y);
				if (d < best) best = d;
			}
			if (best < c.r + 80) wantPicked.push(c);
		}
		const gotKept: RBObstacle[] = [], gotPicked: RBObstacle[] = [];
		pruneByTube(cs, tube, gotKept, gotPicked);
		cases++;
		if (wantPicked.length > 0 && wantPicked.length < n * 0.8) nonTrivial++;
		if (!sameRef(wantKept, gotKept) || !sameRef(wantPicked, gotPicked)) bad++;
		// 走廊剪枝：旧循环（含 MAX_SUBSET 规模上限）
		const ax = { x: R() * 3000, y: R() * 2000 }, bx = R() < 0.1 ? { x: ax.x, y: ax.y } : { x: R() * 3000, y: R() * 2000 };
		for (const expand of [150, 500, 0, 3000]) {
			const kept: RBObstacle[] = [];
			const cand: Array<{ o: RBObstacle; d: number }> = [];
			for (const c of cs) {
				if (c.noPrune) { kept.push(c); continue; }
				const dSeg = pointSegDist(c.x, c.y, ax.x, ax.y, bx.x, bx.y);
				if (dSeg < c.r + expand || Math.hypot(c.x - ax.x, c.y - ax.y) < c.r + expand || Math.hypot(c.x - bx.x, c.y - bx.y) < c.r + expand) cand.push({ o: c, d: dSeg });
			}
			if (cand.length > 1400) { cand.sort((a, b) => a.d - b.d); cand.length = 1400; }
			const want = kept.concat(cand.map(c => c.o));
			const got = corridorSubset(cs, ax, bx, expand);
			cases++;
			if (!sameRef(want, got)) bad++;
		}
	}
	// 超过 MAX_SUBSET 的走廊
	{
		const cs: RBObstacle[] = [];
		for (let i = 0; i < 2600; i++) cs.push({ id: `m${i}`, kind: 'circle', x: R() * 800, y: R() * 600, r: 3 + R() * 5, noPrune: i % 9 === 0 });
		const ax = { x: 100, y: 100 }, bx = { x: 700, y: 500 };
		const kept: RBObstacle[] = [], cand: Array<{ o: RBObstacle; d: number }> = [];
		for (const c of cs) {
			if (c.noPrune) { kept.push(c); continue; }
			const dSeg = pointSegDist(c.x, c.y, ax.x, ax.y, bx.x, bx.y);
			if (dSeg < c.r + 500 || Math.hypot(c.x - ax.x, c.y - ax.y) < c.r + 500 || Math.hypot(c.x - bx.x, c.y - bx.y) < c.r + 500) cand.push({ o: c, d: dSeg });
		}
		assert(cand.length > 1400, '规模上限样本确实超过 MAX_SUBSET');
		cand.sort((a, b) => a.d - b.d); cand.length = 1400;
		const want = kept.concat(cand.map(c => c.o));
		cases++;
		if (!sameRef(want, corridorSubset(cs, ax, bx, 500))) bad++;
	}
	assert(bad === 0, `pruneByTube / corridorSubset 与旧循环一致（${cases} 组）`);
	assert(nonTrivial >= 20, `管道剪枝样本里「部分入选」的非平凡情形足够多（${nonTrivial}）`);
}

{
	console.log('\nhypotFast ≡ Math.hypot（两参数）');
	const R = rng(31);
	let bad = 0;
	for (let i = 0; i < 300000; i++) {
		const sc = Math.pow(10, Math.floor(R() * 9) - 3);
		const a = (R() - 0.5) * sc * 1000, b = (R() - 0.5) * sc * 1000;
		if (Math.hypot(a, b) !== hypotFast(a, b)) bad++;
	}
	for (const a of [0, -0, 1, 5e-324, 1e308, Infinity, -Infinity, NaN]) for (const b of [0, -0, 3, 1e-320, 1e308, Infinity, NaN]) {
		if (!Object.is(Math.hypot(a, b), hypotFast(a, b))) bad++;
	}
	assert(bad === 0, 'hypotFast 与 Math.hypot 逐位一致（30 万随机 + 特殊值）');
	assert(HYPOT_FAST_OK === true, '本运行时的加载期自检通过（启用 hypotFast）');
}

{
	console.log('\ncoarseTube 新 ≡ 旧（逐字复制的参照实现）');
	const R = rng(1357);
	let bad = 0, cases = 0, nulls = 0, paths = 0, rel = 0;
	for (let rep = 0; rep < 160; rep++) {
		const W = 600 + R() * 2400, H = 500 + R() * 1800;
		const n = Math.floor(R() * 700);   // 含 < 8 的小表
		const cs: RBObstacle[] = [];
		const net = R() < 0.6 ? 'N1' : '';
		for (let i = 0; i < n; i++) {
			const c: RBObstacle = { id: `t${i}`, kind: 'circle', x: R() * W, y: R() * H, r: 2 + R() * (R() < 0.08 ? 60 : 10) };
			const u = R();
			if (u < 0.3) c.net = 'N1'; else if (u < 0.45) c.net = 'N2';
			const lu = R();
			if (lu < 0.3) c.layer = 1; else if (lu < 0.55) c.layer = 2;   // 其余 undefined = 全层；偶有 3（不落任何层）
			if (R() < 0.02) c.layer = 3;
			if (R() < 0.05) c.soft = true;
			cs.push(c);
		}
		if (R() < 0.3) cs.push({ id: 'big', kind: 'circle', x: R() * W, y: R() * H, r: 150 + R() * 300 });
		for (let q = 0; q < 6; q++) {
			const src = { x: R() * W, y: R() * H };
			let dst = { x: R() * W, y: R() * H };
			if (q === 0) dst = { x: src.x + R() * 0.5, y: src.y };         // 过短 → null
			if (q === 1 && cs.length) dst = { x: cs[0].x + 1, y: cs[0].y };  // 端点压在障碍上（release 路径）
			if (q === 2) dst = { x: src.x + 60 + R() * 100, y: src.y + R() * 40 };   // 短连接
			// 窗口外也有障碍（板大于窗口）
			const want = refCoarseTube(cs, net, src, dst);
			const got = coarseTube(cs, net, src, dst);
			cases++;
			if (want === null) nulls++; else paths++;
			if (q === 1) rel++;
			const same = want === null ? got === null
				: got !== null && want.length === got.length && want.every((p, i) => Object.is(p.x, got[i].x) && Object.is(p.y, got[i].y));
			if (!same) bad++;
		}
	}
	assert(bad === 0, `coarseTube 与旧实现一致（${cases} 次：找到路径 ${paths}，null ${nulls}）`);
	assert(paths > 200 && nulls > 50, 'coarseTube 样本里「有路径 / null」两类都有');
}

const ms = Date.now() - t0;
console.log(`\n${passed} passed, ${failed} failed (${ms} ms)`);
if (failed > 0) process.exit(1);
