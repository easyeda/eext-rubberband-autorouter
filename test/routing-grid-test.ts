/**
 * 网格 A* 兜底求解器（grid-router）回归测试
 * 运行方式：npx tsx test/routing-grid-test.ts
 *
 * 覆盖：空场直连 / 带缺口的墙 / 封死终点（须快速失败）/ 需过孔的双层路由 /
 * 不同窗口尺寸背靠背重复调用（捕捉缓冲池陈旧数据）/ 可达性预检 A/B 对照。
 */

import { gridRoute, gridStats, GridRouteRequest, GridRouteResult } from '../src/router/grid-router';
import type { RBObstacle } from '../src/router/rubberband-engine';

let passed = 0;
let failed = 0;

function assert(condition: boolean, message: string): void {
	if (condition) {
		console.log(`  ✅ ${message}`);
		passed++;
	} else {
		console.error(`  ❌ ${message}`);
		failed++;
	}
}

async function describe(name: string, fn: () => Promise<void>): Promise<void> {
	console.log(`\n▶ ${name}`);
	await fn();
}

// 可复现伪随机（mulberry32）
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

let oid = 0;
const circle = (x: number, y: number, r: number, extra: Partial<RBObstacle> = {}): RBObstacle =>
	({ id: `c${oid++}`, kind: 'circle', x, y, r, net: 'OTHER', ...extra });
const capsule = (x1: number, y1: number, x2: number, y2: number, r: number, extra: Partial<RBObstacle> = {}): RBObstacle =>
	({ id: `k${oid++}`, kind: 'capsule', x: x1, y: y1, x1, y1, x2, y2, r, net: 'OTHER', ...extra });

/** 与调用位置无关的结果指纹（debug 字符串不参与） */
function fp(r: GridRouteResult): string {
	return JSON.stringify({ ok: r.ok, prims: r.prims, vias: r.vias, cell: r.cell, crossed: r.crossed });
}

function baseReq(over: Partial<GridRouteRequest>): GridRouteRequest {
	return {
		src: { x: 0, y: 0 }, dst: { x: 400, y: 0 },
		obstacles: [], net: 'N', layers: [1, 2],
		cell: 8, viaCostMil: 200, budgetMs: 4000,
		...over,
	};
}

/** 'off' = 关闭预检；'default' = 生产参数；'eager' = 第 1 个弹出起即交错 BFS（小场景也走全路径） */
function setPrecheck(mode: 'off' | 'default' | 'eager'): void {
	(globalThis as unknown as { __RB_GRID_PRECHECK__?: number }).__RB_GRID_PRECHECK__ =
		mode === 'off' ? 0 : mode === 'eager' ? 2 : undefined;
}

function pathLen(r: GridRouteResult): number {
	let s = 0;
	for (const p of r.prims) if (p.kind === 'line') s += Math.hypot(p.x2! - p.x1!, p.y2! - p.y1!);
	return s;
}

(async () => {
	await describe('空场直连', async () => {
		const r = await gridRoute(baseReq({ layers: [1] }));
		assert(r.ok, '空场：成功');
		assert(r.vias.length === 0, '空场：无过孔');
		assert(pathLen(r) >= 400 && pathLen(r) < 412, `空场：长度≈400（实得 ${pathLen(r).toFixed(3)}）`);
		assert(r.prims.every(p => p.layer === 1), '空场：全在 L1');
	});

	await describe('带缺口的墙', async () => {
		// 竖墙 x=200，y∈[-300,300]，仅在 y≈100 处留缺口
		const wall: RBObstacle[] = [];
		for (let y = -300; y <= 300; y += 10) {
			if (y >= 80 && y <= 120) continue;
			wall.push(circle(200, y, 12));
		}
		const r = await gridRoute(baseReq({ layers: [1], obstacles: wall }));
		assert(r.ok, '缺口墙：成功');
		// 路径必须穿过 x=200 处的缺口（y≈100）
		let crossY = NaN;
		for (const p of r.prims) {
			if (p.kind !== 'line') continue;
			const lo = Math.min(p.x1!, p.x2!), hi = Math.max(p.x1!, p.x2!);
			if (lo <= 200 && hi >= 200 && lo !== hi) {
				crossY = p.y1! + (p.y2! - p.y1!) * (200 - p.x1!) / (p.x2! - p.x1!);
			}
		}
		assert(crossY >= 70 && crossY <= 130, `缺口墙：在缺口处过墙（y=${crossY.toFixed(1)}）`);
		assert(pathLen(r) > 400, '缺口墙：绕行比直线长');
		// 同网障碍不挡路
		const same = wall.map(o => ({ ...o, net: 'N' }));
		const r2 = await gridRoute(baseReq({ layers: [1], obstacles: same }));
		assert(r2.ok && pathLen(r2) >= 400 && pathLen(r2) < 412, '同网障碍不挡路');
	});

	await describe('封死终点（快速失败）', async () => {
		const ring: RBObstacle[] = [];
		for (let a = 0; a < 360; a += 10) {
			ring.push(circle(400 + 40 * Math.cos(a * Math.PI / 180), 40 * Math.sin(a * Math.PI / 180), 14));
		}
		const t0 = Date.now();
		const r = await gridRoute(baseReq({ layers: [1, 2], obstacles: ring.map(o => ({ ...o, layer: undefined })) }));
		const dt = Date.now() - t0;
		assert(!r.ok && r.prims.length === 0 && r.vias.length === 0, '封死终点：失败且无图元');
		assert(dt < 1500, `封死终点：快速失败（${dt}ms）`);
		// 起点被封死同理
		const ring2 = ring.map(o => circle(o.x - 400, o.y, o.r));
		const r2 = await gridRoute(baseReq({ layers: [1], obstacles: ring2 }));
		assert(!r2.ok, '封死起点：失败');
	});

	await describe('需要过孔的双层路由', async () => {
		// L1 上一道完整的墙（窗口内无缺口）；L2 空 → 必须经过孔换层
		const wall: RBObstacle[] = [];
		for (let y = -1200; y <= 1200; y += 10) wall.push(circle(200, y, 12, { layer: 1 }));
		const r = await gridRoute(baseReq({ layers: [1, 2], obstacles: wall, srcPadLayers: [1], dstPadLayers: [1] }));
		assert(r.ok, '过孔：成功');
		assert(r.vias.length === 2, `过孔：两个过孔（实得 ${r.vias.length}）`);
		assert(r.prims.some(p => p.layer === 2), '过孔：中段在 L2');
		// 只有 L1 → 失败
		const r1 = await gridRoute(baseReq({ layers: [1], obstacles: wall }));
		assert(!r1.ok, '单层被墙封死：失败');
		// viaExtraMil 把过孔落点推离障碍：窄缝放不下过孔时必须避开
		const rv = await gridRoute(baseReq({
			layers: [1, 2], obstacles: wall, srcPadLayers: [1], dstPadLayers: [1], viaExtraMil: 6,
		}));
		assert(rv.ok && rv.vias.length === 2, '过孔：viaExtraMil 路径仍成功');
	});

	await describe('罚分障碍（协商拥挤）', async () => {
		const pen: RBObstacle[] = [];
		for (let y = -60; y <= 60; y += 10) pen.push(circle(200, y, 12, { layer: 1, net: 'VICTIM' }));
		const r = await gridRoute(baseReq({
			layers: [1], penaltyObstacles: pen, penaltyOf: () => 300,
		}));
		assert(r.ok, '罚分：成功');
		assert(Array.isArray(r.crossed), '罚分：crossed 有值');
	});

	await describe('背靠背重复调用（陈旧缓冲检测）', async () => {
		const rnd = rng(7);
		const fields: GridRouteRequest[] = [];
		for (let k = 0; k < 6; k++) {
			const obs: RBObstacle[] = [];
			const n = 20 + Math.floor(rnd() * 80);
			for (let i = 0; i < n; i++) obs.push(circle(rnd() * 600 - 100, rnd() * 300 - 150, 6 + rnd() * 14, { layer: rnd() < 0.5 ? 1 : undefined }));
			const span = 150 + k * 120;          // 窗口尺寸各不相同
			fields.push(baseReq({
				src: { x: 0, y: 0 }, dst: { x: span, y: (k % 2 ? 1 : -1) * 20 * k },
				obstacles: obs, layers: k % 3 === 0 ? [1] : [1, 2],
				cell: k % 2 ? 8 : 6, viaExtraMil: k % 2 ? 4 : 0,
			}));
		}
		const first = [] as string[];
		for (const f of fields) first.push(fp(await gridRoute(f)));
		// 其它请求（大窗、小窗）穿插
		await gridRoute(baseReq({ src: { x: 0, y: 0 }, dst: { x: 900, y: 500 }, cell: 5 }));
		await gridRoute(baseReq({ src: { x: 0, y: 0 }, dst: { x: 30, y: 10 } }));
		// 逆序、再正序重复：结果必须与首次逐项一致
		for (let k = fields.length - 1; k >= 0; k--) {
			assert(fp(await gridRoute(fields[k])) === first[k], `重复调用 #${k} 结果一致（逆序）`);
		}
		for (let k = 0; k < fields.length; k++) {
			assert(fp(await gridRoute(fields[k])) === first[k], `重复调用 #${k} 结果一致（正序）`);
		}
		assert(first.some(s => s.startsWith('{"ok":true')), '随机场中至少有成功样本');
	});

	await describe('可达性预检 A/B（随机障碍场）', async () => {
		const rnd = rng(20240607);
		let nOk = 0, nFail = 0, mismatch = 0, total = 0, eagerHits = 0;
		const t0 = Date.now();
		for (let k = 0; k < 36; k++) {
			const obs: RBObstacle[] = [];
			const n = 10 + Math.floor(rnd() * 70);
			for (let i = 0; i < n; i++) {
				const o = rnd() < 0.7
					? circle(rnd() * 360 - 20, rnd() * 160 - 80, 6 + rnd() * 18, { layer: rnd() < 0.6 ? 1 + Math.floor(rnd() * 2) : undefined })
					: capsule(rnd() * 360 - 20, rnd() * 160 - 80, rnd() * 360 - 20, rnd() * 160 - 80, 4 + rnd() * 8, { layer: rnd() < 0.6 ? 1 + Math.floor(rnd() * 2) : undefined });
				if (rnd() < 0.1) o.net = 'N';
				obs.push(o);
			}
			// 一部分场景把终点围住（含部分缺口），触发「不可达」
			if (k % 3 === 0) {
				const gap = rnd() < 0.5 ? -1 : Math.floor(rnd() * 12);
				for (let a = 0; a < 12; a++) {
					if (a === gap) continue;
					const ang = a * Math.PI / 6;
					obs.push(circle(340 + 36 * Math.cos(ang), 36 * Math.sin(ang), 14, k % 2 ? undefined : { layer: 1 }));
				}
			}
			const req = baseReq({
				src: { x: 0, y: 0 }, dst: { x: 340, y: 0 },
				obstacles: obs, layers: [1, 2],
				cell: k % 2 ? 8 : 5,
				viaExtraMil: k % 4 === 1 ? 5 : 0,
				srcPad: { x: 0, y: 0, r: 10, viaKeepoutR: 18 },
				dstPad: { x: 340, y: 0, r: 10, viaKeepoutR: 18 },
				srcPadLayers: k % 5 === 0 ? [1, 2] : [1],
				dstPadLayers: k % 7 === 0 ? [1, 2] : [1],
				srcStubs: k % 6 === 0 ? [{ x: 14, y: 4 }, { x: 14, y: -6 }] : undefined,
				dstStubs: k % 6 === 3 ? [{ x: 326, y: 5 }] : undefined,
				penaltyObstacles: k % 8 === 2 ? obs.slice(0, 6).map(o => ({ ...o, net: 'P' + o.id })) : undefined,
				viaInPad: k % 9 === 0,
			});
			setPrecheck('off');
			const a = await gridRoute(req);
			setPrecheck('eager');
			const b = await gridRoute(req);
			setPrecheck('default');
			const c = await gridRoute(req);
			total++;
			if (fp(a) !== fp(b) || fp(a) !== fp(c)) {
				mismatch++;
				console.error(`    场景 ${k}: 预检 关/急切/默认 结果不同 ok ${a.ok}/${b.ok}/${c.ok}`);
			}
			if (!a.ok && b.debug?.startsWith('no-path(unreachable)')) eagerHits++;
			if (a.ok) nOk++; else nFail++;
		}
		assert(mismatch === 0, `预检 关/急切/默认 ${total} 个场景结果一致`);
		assert(eagerHits > 3, `急切模式确实以「不可达」提前返回（${eagerHits} 次）`);
		assert(nOk > 5 && nFail > 5, `成功/失败样本均衡（ok=${nOk} fail=${nFail}）`);
		assert(Date.now() - t0 < 4000, `耗时 ${Date.now() - t0}ms < 4s`);
	});

	await describe('LOS 清距跳采样 A/B（__RB_GRID_LOSFAST__ 0 / 1 / 2 自检）', async () => {
		const g = globalThis as unknown as { __RB_GRID_LOSFAST__?: number; __RB_GRID_LOSFAST_STATS__?: { cmp: number; bad: number } };
		const rnd = rng(31337);
		let total = 0, mismatch = 0, nOk = 0;
		const t0 = Date.now();
		g.__RB_GRID_LOSFAST_STATS__ = { cmp: 0, bad: 0 };
		for (let k = 0; k < 40; k++) {
			const dense = k % 2 === 0;
			const nLayers = k % 3 === 0 ? 1 : 2;
			const cell = [4, 5, 8, 12][k % 4];
			const span = 500 + Math.floor(rnd() * 900);          // 长线穿空场
			const hgt = 150 + Math.floor(rnd() * 250);
			const obs: RBObstacle[] = [];
			const n = dense ? 60 + Math.floor(rnd() * 140) : 3 + Math.floor(rnd() * 12);
			for (let i = 0; i < n; i++) {
				const lay = nLayers === 1 ? 1 : (rnd() < 0.5 ? 1 + Math.floor(rnd() * 2) : undefined);
				const x = rnd() * (span + 40) - 20, y = (rnd() - 0.5) * hgt * 2;
				obs.push(rnd() < 0.7
					? circle(x, y, 4 + rnd() * (dense ? 14 : 30), { layer: lay })
					: capsule(x, y, x + (rnd() - 0.5) * 200, y + (rnd() - 0.5) * 200, 3 + rnd() * 8, { layer: lay }));
			}
			// 偶发整墙 + 缺口，逼出绕行/过孔
			if (k % 5 === 1) {
				const gy = (rnd() - 0.5) * hgt;
				for (let y = -hgt * 2; y <= hgt * 2; y += 8) {
					if (Math.abs(y - gy) < 24) continue;
					obs.push(circle(span / 2, y, 8, nLayers === 1 ? { layer: 1 } : {}));
				}
			}
			const req = baseReq({
				src: { x: 0, y: 0 }, dst: { x: span, y: (rnd() - 0.5) * hgt },
				obstacles: obs, layers: nLayers === 1 ? [1] : [1, 2],
				cell, viaExtraMil: k % 4 === 1 ? 5 : 0,
				srcPad: { x: 0, y: 0, r: 10, viaKeepoutR: 18 },
				dstPad: { x: span, y: 0, r: 10, viaKeepoutR: 18 },
				srcPadLayers: [1], dstPadLayers: [1],
			});
			g.__RB_GRID_LOSFAST__ = 0;
			const a = await gridRoute(req);
			g.__RB_GRID_LOSFAST__ = undefined;
			const b = await gridRoute(req);
			g.__RB_GRID_LOSFAST__ = 2;               // 自检：每次 LOS 同时算两遍，不一致即抛
			const c = await gridRoute(req);
			g.__RB_GRID_LOSFAST__ = undefined;
			total++;
			if (fp(a) !== fp(b) || fp(a) !== fp(c)) {
				mismatch++;
				console.error(`    场景 ${k}: LOSFAST 关/默认/自检 结果不同 ok ${a.ok}/${b.ok}/${c.ok}`);
			}
			if (a.ok) nOk++;
		}
		const st = g.__RB_GRID_LOSFAST_STATS__;
		assert(mismatch === 0, `LOSFAST 关/默认/自检 ${total} 个场景结果一致`);
		assert(st.bad === 0 && st.cmp > 1000, `自检模式比对 ${st.cmp} 次 LOS，不一致 ${st.bad} 次`);
		assert(nOk > 10, `成功样本足够（ok=${nOk}/${total}）`);
		assert(Date.now() - t0 < 6000, `耗时 ${Date.now() - t0}ms < 6s`);
		g.__RB_GRID_LOSFAST_STATS__ = undefined;
	});

	assert(gridStats.calls > 0, '统计计数在累加');
	console.log(`\n${passed} passed, ${failed} failed`);
	process.exit(failed === 0 ? 0 : 1);
})().catch(e => { console.error(e); process.exit(1); });
