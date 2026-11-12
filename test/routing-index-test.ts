/**
 * 空间索引单元测试（src/router/spatial-index.ts + edge-router.stubFeasible 的索引路径）
 * 运行方式：npx tsx test/routing-index-test.ts
 *
 * 判据：索引查询必须与暴力线性扫描返回**完全相同的集合**（点 / 线段 / 圆 / 胶囊，
 * 带层语义，含删除后与重建后）。失败时 exit code 非零。
 */
import { SpatialGrid, ObstacleIndex, obstacleBox } from '../src/router/spatial-index';
import { stubFeasible, PadTerminal } from '../src/router/edge-router';
import type { RBObstacle } from '../src/router/rubberband-engine';

let passed = 0;
let failed = 0;
function assert(cond: boolean, msg: string): void {
	if (cond) { passed++; return; }
	failed++;
	console.error(`  ❌ ${msg}`);
}
function describe(name: string, fn: () => void): void {
	console.log(`\n▶ ${name}`);
	fn();
}

/** mulberry32 种子随机 */
function rng(seed: number): () => number {
	let a = seed >>> 0;
	return () => {
		a = (a + 0x6D2B79F5) >>> 0;
		let t = a;
		t = Math.imul(t ^ (t >>> 15), t | 1);
		t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
		return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
	};
}

interface Item { id: number; box: [number, number, number, number]; layer?: number }

/** 暴力基准：盒闭区间相交 + 层语义（条目 undefined = 全层；查询 undefined = 不限） */
function brute(items: Map<number, Item>, q: [number, number, number, number], layer?: number): number[] {
	const out: number[] = [];
	for (const it of items.values()) {
		const b = it.box;
		const finite = b.every(Number.isFinite);
		if (finite && (b[2] < q[0] || b[0] > q[2] || b[3] < q[1] || b[1] > q[3])) continue;
		if (layer !== undefined && it.layer !== undefined && it.layer !== layer) continue;
		out.push(it.id);
	}
	return out.sort((a, b) => a - b);
}

function sameSet(a: number[], b: number[]): boolean {
	if (a.length !== b.length) return false;
	const x = [...a].sort((p, q) => p - q);
	const y = [...b].sort((p, q) => p - q);
	return x.every((v, i) => v === y[i]);
}

function randBox(r: () => number, span: number): [number, number, number, number] {
	const kind = Math.floor(r() * 4);
	const x = (r() - 0.5) * span, y = (r() - 0.5) * span;
	if (kind === 0) return [x, y, x, y];                                     // 点
	if (kind === 1) {                                                         // 线段
		const x2 = x + (r() - 0.5) * 120, y2 = y + (r() - 0.5) * 120;
		return [Math.min(x, x2), Math.min(y, y2), Math.max(x, x2), Math.max(y, y2)];
	}
	const rad = 1 + r() * 30;                                                 // 圆
	if (kind === 2) return [x - rad, y - rad, x + rad, y + rad];
	const big = 200 + r() * 400;                                              // 偶发大圆（走 always 路径）
	return [x - big, y - big, x + big, y + big];
}

describe('SpatialGrid：随机插入 / 查询 / 删除 / 重建 与暴力扫描一致', () => {
	for (const seed of [1, 2, 3, 4, 5]) {
		const r = rng(seed);
		const grid = new SpatialGrid(seed % 2 ? 40 : 17);
		const items = new Map<number, Item>();
		let nextId = 0;
		const check = (tag: string): void => {
			for (let k = 0; k < 60; k++) {
				const q = randBox(r, 2400);
				const layer = r() < 0.4 ? undefined : (r() < 0.5 ? 1 : 2);
				const got = grid.query(q[0], q[1], q[2], q[3], layer);
				const want = brute(items, q, layer);
				assert(sameSet(got, want), `seed ${seed} ${tag} query#${k} layer=${layer} got ${got.length} want ${want.length}`);
				assert(new Set(got).size === got.length, `seed ${seed} ${tag} query#${k} 结果无重复`);
				const sorted = grid.querySorted(q[0], q[1], q[2], q[3], layer);
				assert(sorted.every((v, i) => i === 0 || sorted[i - 1] <= v) && sameSet(sorted, want), `seed ${seed} ${tag} querySorted#${k}`);
			}
		};
		for (let i = 0; i < 400; i++) {
			const box = randBox(r, 2400);
			const layer = r() < 0.5 ? undefined : (r() < 0.5 ? 1 : 2);
			const id = nextId++;
			grid.insert(id, box[0], box[1], box[2], box[3], layer);
			items.set(id, { id, box, layer });
		}
		check('插入后');
		// 删除一半 + 同 id 重新插入（替换语义）
		const ids = [...items.keys()];
		for (const id of ids) {
			const x = r();
			if (x < 0.5) {
				assert(grid.remove(id) === true, `seed ${seed} 删除 ${id} 返回 true`);
				items.delete(id);
			} else if (x < 0.65) {
				const box = randBox(r, 2400);
				grid.insert(id, box[0], box[1], box[2], box[3], 2);
				items.set(id, { id, box, layer: 2 });
			}
		}
		assert(grid.remove(999999) === false, `seed ${seed} 删除不存在 id 返回 false`);
		assert(grid.size === items.size, `seed ${seed} size 与基准一致`);
		check('删除/替换后');
		// 重建（clear 后全部重新插入）
		const snapshot = [...items.values()];
		grid.clear();
		assert(grid.size === 0, `seed ${seed} clear 后为空`);
		for (const it of snapshot) grid.insert(it.id, it.box[0], it.box[1], it.box[2], it.box[3], it.layer);
		check('重建后');
		// 再追加 + 再删，确认删光后无残留
		for (const it of snapshot) grid.remove(it.id);
		items.clear();
		check('全部删除后');
		assert(grid.query(-1e6, -1e6, 1e6, 1e6).length === 0, `seed ${seed} 删光后全域查询为空`);
	}
});

describe('SpatialGrid：非有限 / 巨大坐标条目保守返回', () => {
	const g = new SpatialGrid(40);
	g.insert(1, NaN, 0, 10, 10);
	g.insert(2, 0, 0, 5, 5, 1);
	g.insert(3, -1e9, -1e9, 1e9, 1e9);
	g.insert(4, 1e7, 1e7, 1e7 + 1, 1e7 + 1);
	const q = g.query(100, 100, 120, 120);
	assert(sameSet(q, [1, 3]), 'NaN 与超大条目对任意查询都返回，远处普通条目不返回');
	assert(sameSet(g.query(1e7, 1e7, 1e7, 1e7), [1, 3, 4]), '超出键空间的坐标仍能精确命中');
	assert(sameSet(g.query(0, 0, 1, 1, 2), [1, 3]), '层 2 查询排除 layer=1 条目');
	assert(sameSet(g.query(0, 0, 1, 1, 1), [1, 2, 3]), '层 1 查询包含 layer=1 条目');
	assert(g.remove(1) && g.remove(3), '删除 always 条目');
	assert(g.query(100, 100, 120, 120).length === 0, '删除 always 条目后不再返回');
});

describe('ObstacleIndex：圆 / 胶囊障碍 —— 谓词结果 与 线性扫描 一致', () => {
	const r = rng(77);
	const obs: RBObstacle[] = [];
	for (let i = 0; i < 600; i++) {
		const x = (r() - 0.5) * 1600, y = (r() - 0.5) * 1200;
		const layer = r() < 0.5 ? undefined : (r() < 0.5 ? 1 : 2);
		if (r() < 0.15) {
			obs.push({
				id: `c${i}`, kind: 'capsule', x, y, r: 3 + r() * 8, net: `N${i % 7}`, layer,
				x1: x, y1: y, x2: x + (r() - 0.5) * 80, y2: y + (r() - 0.5) * 80,
			} as RBObstacle);
		} else {
			obs.push({ id: `o${i}`, kind: 'circle', x, y, r: 2 + r() * 25, net: r() < 0.9 ? `N${i % 7}` : undefined, layer } as RBObstacle);
		}
	}
	const idx = new ObstacleIndex(obs);
	let hits = 0;
	for (let k = 0; k < 400; k++) {
		const x = (r() - 0.5) * 1600, y = (r() - 0.5) * 1200, reach = r() * 20;
		// 谓词：圆心距 < o.r + reach（viaObstacleClear 形态，仅圆）
		const want = obs.filter(o => o.kind !== 'capsule' && Math.hypot(x - o.x, y - o.y) < o.r + reach).map(o => o.id).sort();
		const got = idx.near(x - reach, y - reach, x + reach, y + reach)
			.filter(o => o.kind !== 'capsule' && Math.hypot(x - o.x, y - o.y) < o.r + reach).map(o => o.id).sort();
		assert(JSON.stringify(got) === JSON.stringify(want), `点查询#${k} 圆谓词集合一致`);
		hits += want.length;
		// 线段：中心到线段距离 < o.r（bottomFor 形态）
		const x2 = x + (r() - 0.5) * 300, y2 = y + (r() - 0.5) * 300;
		const segDist = (o: RBObstacle): number => {
			const dx = x2 - x, dy = y2 - y, l2 = dx * dx + dy * dy;
			let t = l2 < 1e-12 ? 0 : ((o.x - x) * dx + (o.y - y) * dy) / l2;
			t = Math.max(0, Math.min(1, t));
			return Math.hypot(o.x - (x + t * dx), o.y - (y + t * dy));
		};
		const wantS = obs.filter(o => o.kind !== 'capsule' && segDist(o) < o.r).map(o => o.id).sort();
		const gotS = idx.near(Math.min(x, x2), Math.min(y, y2), Math.max(x, x2), Math.max(y, y2))
			.filter(o => o.kind !== 'capsule' && segDist(o) < o.r).map(o => o.id).sort();
		assert(JSON.stringify(gotS) === JSON.stringify(wantS), `线段查询#${k} 集合一致`);
		// 胶囊：盒必须是真实几何上界 —— 胶囊上采样点到查询点 < r 的，必在候选里
		for (const o of obs) {
			if (o.kind !== 'capsule') continue;
			const c = o as RBObstacle & { x1: number; y1: number; x2: number; y2: number };
			let near = false;
			for (let s = 0; s <= 8 && !near; s++) {
				const t = s / 8;
				near = Math.hypot(x - (c.x1 + (c.x2 - c.x1) * t), y - (c.y1 + (c.y2 - c.y1) * t)) < o.r;
			}
			if (near) {
				assert(idx.near(x, y, x, y).includes(o), `胶囊 ${o.id} 覆盖点 (${x.toFixed(1)},${y.toFixed(1)}) 必在候选内`);
			}
		}
	}
	assert(hits > 0, '测试数据确有命中（非空洞通过）');
	// obstacleBox 对圆 / 胶囊是上界
	for (const o of obs.slice(0, 50)) {
		const b = obstacleBox(o);
		assert(b[0] <= o.x - o.r && b[2] >= o.x + o.r && b[1] <= o.y - o.r && b[3] >= o.y + o.r, `obstacleBox(${o.id}) 覆盖圆`);
	}
});

describe('ObstacleIndex.of：缓存与长度变化重建', () => {
	const arr: RBObstacle[] = [
		{ id: 'a', kind: 'circle', x: 0, y: 0, r: 10 } as RBObstacle,
		{ id: 'b', kind: 'circle', x: 500, y: 500, r: 10 } as RBObstacle,
	];
	const i1 = ObstacleIndex.of(arr);
	assert(ObstacleIndex.of(arr) === i1, '同一数组、同长度 → 命中缓存');
	assert(i1.near(-5, -5, 5, 5).length === 1, '初始查询 1 个');
	arr.push({ id: 'c', kind: 'circle', x: 2, y: 2, r: 5 } as RBObstacle);
	const i2 = ObstacleIndex.of(arr);
	assert(i2 !== i1, '数组增长 → 重建');
	assert(i2.near(-5, -5, 5, 5).length === 2, '重建后包含新增条目');
	arr.pop();
	const i3 = ObstacleIndex.of(arr);
	assert(i3 !== i2 && i3.near(-5, -5, 5, 5).length === 1, '数组缩短 → 重建且结果同暴力扫描');
	const other = [...arr];
	assert(ObstacleIndex.of(other) !== i3, '不同数组 → 独立索引');
});

describe('stubFeasible：索引路径 与 线性路径 结果一致', () => {
	const r = rng(2024);
	const obs: RBObstacle[] = [];
	for (let i = 0; i < 500; i++) {
		const x = (r() - 0.5) * 700, y = (r() - 0.5) * 700;
		const net = r() < 0.15 ? 'SELF' : `N${i % 9}`;
		const base = { id: `s${i}`, x, y, r: 3 + r() * 12, net, layer: r() < 0.5 ? undefined : 1 } as RBObstacle;
		const roll = r();
		if (roll < 0.1) obs.push({ ...base, soft: true } as RBObstacle);
		else if (roll < 0.15) obs.push({ ...base, forceBlock: true } as RBObstacle);
		else if (roll < 0.25) obs.push({ ...base, kind: 'capsule', x1: x, y1: y, x2: x + (r() - 0.5) * 60, y2: y + (r() - 0.5) * 60 } as RBObstacle);
		else obs.push({ ...base, kind: 'circle' } as RBObstacle);
	}
	const idx = new ObstacleIndex(obs);
	let trues = 0, falses = 0;
	for (let k = 0; k < 600; k++) {
		const pad: PadTerminal = {
			x: (r() - 0.5) * 500, y: (r() - 0.5) * 500, net: 'SELF',
			outline: [{ x: 0, y: 0, r: 4 + r() * 8 }, { x: (r() - 0.5) * 20, y: 0, r: 3 + r() * 5 }],
		};
		const len = r() * 160;
		const a = r() * Math.PI * 2;
		const x2 = pad.x + Math.cos(a) * len, y2 = pad.y + Math.sin(a) * len;
		const lin = stubFeasible(pad, x2, y2, obs, 'SELF');
		const fast = stubFeasible(pad, x2, y2, obs, 'SELF', idx);
		assert(lin === fast, `stubFeasible#${k} 索引=${fast} 线性=${lin}`);
		if (lin) trues++; else falses++;
	}
	assert(trues > 20 && falses > 20, `两种结果都有覆盖 (true=${trues}, false=${falses})`);
});

console.log(`\n通过 ${passed}，失败 ${failed}`);
if (failed > 0) process.exit(1);
