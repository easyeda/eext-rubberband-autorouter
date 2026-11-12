/**
 * 拉紧内核（tighten-core / tighten-job）单元测试
 * 运行方式：npx tsx test/tighten-core-test.ts
 *
 * 每个手工夹具都用「独立的暴力检查器」验收（0.1mil 稠密采样 + 精确点到形状距离，
 * 不复用 tighten-core 的校验器）：
 *  1 尖角   2 拉紧（与解析最短路比对）   3 严格 DRC（实际线宽、不恶化）
 *  4 连通   5 不变长   6 幂等   7 同伦不变（旧路径∘新路径⁻¹ 对障碍点的环绕数为 0）
 */

import { runTightenJob, padsConnectedByCopper, TightenJobInput, TightenJobEvent } from '../src/router/tighten-job';
import * as C from '../src/router/tighten-core';
import { padShapeDistance, buildPolylineObstacle, buildPolygonObstacle, RBObstacle } from '../src/router/rubberband-engine';
import { PlanningEngine } from '../src/router/planning-engine';
import type { RenderedPrim, EnginePad, PadShape, PlannedVia } from '../src/router/planning-engine';
import type { DRCRules } from '../src/router/drc-rules';

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

const queue: Array<{ name: string; fn: () => Promise<void> | void }> = [];
function describe(name: string, fn: () => Promise<void> | void): void {
	queue.push({ name, fn });
}

// ─── 夹具工具 ───────────────────────────────────────────────────────────────

const drc: DRCRules = {
	traceWidthMil: 10, clearanceMil: 6, trackToTrackMil: 6, trackToLineMil: 6, trackToBoardMil: 10,
	trackToPadMil: 6, trackToFillMil: 6, holeToHoleMil: 12,
};
const L = (net: string, x1: number, y1: number, x2: number, y2: number, layer = 1): RenderedPrim =>
	({ kind: 'line', net, layer, x1, y1, x2, y2 });
const ARC = (net: string, cx: number, cy: number, r: number, a0: number, sw: number, layer = 1): RenderedPrim =>
	({ kind: 'arc', net, layer, cx, cy, r, startAngle: a0, sweep: sw });
const poly = (net: string, pts: number[][], layer = 1): RenderedPrim[] =>
	pts.slice(1).map((p, i) => L(net, pts[i][0], pts[i][1], p[0], p[1], layer));

let padIdx = 0;
function pad(net: string, x: number, y: number, shape: any[], padShapes: Record<string, PadShape>, opts: Partial<EnginePad> & { rotation?: number; hole?: any[] | null } = {}): EnginePad {
	padShapes[`${x.toFixed(2)},${y.toFixed(2)}`] = { pad: shape, hole: opts.hole ?? null, rotation: opts.rotation ?? 0 };
	const r = Math.hypot(Number(shape[1]) || 0, Number(shape[2]) || 0) / 2;
	return {
		x, y, net, id: `P${padIdx}`, layer: opts.layer ?? 1, padType: opts.padType ?? 'smd',
		holeDiameter: opts.holeDiameter ?? 0, radius: r, rotation: opts.rotation ?? 0, index: padIdx++,
	};
}

interface Run { id: number; net: string; layer: number; width?: number; a: { x: number; y: number }; b: { x: number; y: number }; prims: RenderedPrim[] }
function mkRun(id: number, net: string, prims: RenderedPrim[], width?: number): Run {
	const s = C.primStart(prims[0]), e = C.primEnd(prims[prims.length - 1]);
	return { id, net, layer: prims[0].layer, width, a: s, b: e, prims };
}

interface JobOut { events: TightenJobEvent[]; final: Map<number, RenderedPrim[]>; ms: number; done: any }
async function job(input: Partial<TightenJobInput> & { runs: Run[] }): Promise<JobOut> {
	const msg: TightenJobInput = {
		type: 'tighten', drc, allPads: [], padShapes: {}, baseWorld: [], vias: [], ...input,
	};
	const events: TightenJobEvent[] = [];
	const t0 = Date.now();
	await runTightenJob(msg, m => events.push(m), () => false);
	const final = new Map<number, RenderedPrim[]>();
	for (const r of input.runs) final.set(r.id, r.prims);
	for (const e of events) if (e.type === 'tightenRun' && e.prims) final.set(e.id, e.prims);
	return { events, final, ms: Date.now() - t0, done: events.find(e => e.type === 'tightenDone') };
}

// ─── 独立检查器 ─────────────────────────────────────────────────────────────

const TAU = Math.PI * 2;
function pS(p: RenderedPrim) {
	return p.kind === 'line' ? { x: p.x1!, y: p.y1! } : { x: p.cx! + p.r! * Math.cos(p.startAngle!), y: p.cy! + p.r! * Math.sin(p.startAngle!) };
}
function pE(p: RenderedPrim) {
	return p.kind === 'line' ? { x: p.x2!, y: p.y2! } : { x: p.cx! + p.r! * Math.cos(p.startAngle! + p.sweep!), y: p.cy! + p.r! * Math.sin(p.startAngle! + p.sweep!) };
}
function len(prims: RenderedPrim[]): number {
	return prims.reduce((s, p) => s + (p.kind === 'line' ? Math.hypot(p.x2! - p.x1!, p.y2! - p.y1!) : Math.abs(p.sweep!) * p.r!), 0);
}
function samples(prims: RenderedPrim[], step = 0.1): Array<{ x: number; y: number }> {
	const out: Array<{ x: number; y: number }> = [];
	for (const p of prims) {
		const l = p.kind === 'line' ? Math.hypot(p.x2! - p.x1!, p.y2! - p.y1!) : Math.abs(p.sweep!) * p.r!;
		const n = Math.max(1, Math.ceil(l / step));
		for (let k = 0; k <= n; k++) {
			const t = k / n;
			if (p.kind === 'line') out.push({ x: p.x1! + (p.x2! - p.x1!) * t, y: p.y1! + (p.y2! - p.y1!) * t });
			else {
				const a = p.startAngle! + p.sweep! * t;
				out.push({ x: p.cx! + p.r! * Math.cos(a), y: p.cy! + p.r! * Math.sin(a) });
			}
		}
	}
	return out;
}
function segD(px: number, py: number, x1: number, y1: number, x2: number, y2: number): number {
	const dx = x2 - x1, dy = y2 - y1, l2 = dx * dx + dy * dy;
	const t = l2 > 0 ? Math.max(0, Math.min(1, ((px - x1) * dx + (py - y1) * dy) / l2)) : 0;
	return Math.hypot(px - x1 - dx * t, py - y1 - dy * t);
}
function primD(p: RenderedPrim, x: number, y: number): number {
	if (p.kind === 'line') return segD(x, y, p.x1!, p.y1!, p.x2!, p.y2!);
	// 弧：极角落在扫角内 → |到圆心距 − r|，否则取两端点
	const ang = Math.atan2(y - p.cy!, x - p.cx!);
	let d = p.sweep! >= 0 ? ang - p.startAngle! : p.startAngle! - ang;
	d = ((d % TAU) + TAU) % TAU;
	if (d <= Math.abs(p.sweep!)) return Math.abs(Math.hypot(x - p.cx!, y - p.cy!) - p.r!);
	const s = pS(p), e = pE(p);
	return Math.min(Math.hypot(x - s.x, y - s.y), Math.hypot(x - e.x, y - e.y));
}

/** 检查器条目：d(x,y) = 中心线点到铜皮距离；rule = 边到边间距；gap = d − w/2 */
interface ChkItem { key: string; net?: string; layer?: number; rule: number; d: (x: number, y: number) => number; cx: number; cy: number; reach?: number }

function chkItems(input: Partial<TightenJobInput>, runsNow: Map<number, RenderedPrim[]>, runs: Run[]): ChkItem[] {
	const out: ChkItem[] = [];
	const ps = input.padShapes ?? {};
	for (const p of input.allPads ?? []) {
		const sh = ps[`${p.x.toFixed(2)},${p.y.toFixed(2)}`];
		const shape = { pad: sh?.pad ?? ['ELLIPSE', p.radius * 2, p.radius * 2], hole: sh?.hole ?? null, rotation: sh?.rotation ?? 0 };
		const all = p.layer === 12 || p.padType === 'th';
		out.push({ key: `pad:${p.id}`, net: p.net, layer: all ? undefined : p.layer, rule: drc.trackToPadMil, d: (x, y) => padShapeDistance(shape, x - p.x, y - p.y), cx: p.x, cy: p.y });
		if (p.holeDiameter > 0 && all) out.push({ key: `hole:${p.id}`, net: p.net, rule: drc.trackToPadMil, d: (x, y) => Math.hypot(x - p.x, y - p.y) - p.holeDiameter / 2, cx: p.x, cy: p.y });
	}
	(input.vias ?? []).forEach((v, i) => out.push({ key: `via:${i}`, net: v.net, rule: drc.trackToPadMil, d: (x, y) => Math.hypot(x - v.x, y - v.y) - v.dia / 2, cx: v.x, cy: v.y }));
	(input.tracks ?? []).forEach((t, i) => out.push({ key: `trk:${i}`, net: t.net, layer: t.layer, rule: drc.trackToTrackMil, d: (x, y) => primD(t.prim, x, y) - t.width / 2, cx: pS(t.prim).x, cy: pS(t.prim).y }));
	(input.baseWorld ?? []).forEach((o, i) => {
		const r0 = o.r - drc.traceWidthMil / 2;
		const d = o.kind === 'capsule'
			? (x: number, y: number) => segD(x, y, o.x1!, o.y1!, o.x2!, o.y2!) - r0
			: (x: number, y: number) => Math.hypot(x - o.x, y - o.y) - r0;
		out.push({ key: `st:${i}`, net: o.net, layer: o.layer, rule: 0, d, cx: o.x, cy: o.y });
	});
	for (const r of runs) {
		const prims = runsNow.get(r.id) ?? r.prims;
		const w = r.width ?? drc.traceWidthMil;
		const bb = C.chainBBox(prims);
		out.push({
			key: `run:${r.id}`, net: r.net, layer: r.layer, rule: drc.trackToTrackMil,
			d: (x, y) => { let m = Infinity; for (const p of prims) m = Math.min(m, primD(p, x, y)); return m - w / 2; },
			cx: (bb.x0 + bb.x1) / 2, cy: (bb.y0 + bb.y1) / 2, reach: Math.hypot(bb.x1 - bb.x0, bb.y1 - bb.y0) / 2 + w,
		});
	}
	return out;
}

/** 每条 run 对每个外网条目的最小 gap */
function gaps(input: Partial<TightenJobInput>, runs: Run[], geo: Map<number, RenderedPrim[]>): Map<string, number> {
	const items = chkItems(input, geo, runs);
	const out = new Map<string, number>();
	for (const r of runs) {
		const prims = geo.get(r.id) ?? r.prims;
		const w = r.width ?? drc.traceWidthMil;
		const ss = samples(prims, 0.1);
		const bb = C.chainBBox(prims);
		const rc = { x: (bb.x0 + bb.x1) / 2, y: (bb.y0 + bb.y1) / 2 }, rr = Math.hypot(bb.x1 - bb.x0, bb.y1 - bb.y0) / 2;
		for (const it of items) {
			if (it.net && it.net === r.net) continue;
			if (it.layer !== undefined && it.layer !== r.layer) continue;
			if (it.key === `run:${r.id}`) continue;
			if (it.reach !== undefined && Math.hypot(it.cx - rc.x, it.cy - rc.y) > rr + it.reach + 40) continue;
			if (it.reach === undefined && it.d(rc.x, rc.y) > rr + 40) continue;
			let g = Infinity;
			for (const s of ss) g = Math.min(g, it.d(s.x, s.y) - w / 2);
			out.set(`${r.id}|${it.key}`, g - it.rule);
		}
	}
	return out;
}

/** 环绕数：闭合折线绕点 (x,y) */
function winding(pts: Array<{ x: number; y: number }>, x: number, y: number): number {
	let a = 0;
	for (let i = 0; i < pts.length; i++) {
		const p = pts[i], q = pts[(i + 1) % pts.length];
		const a1 = Math.atan2(p.y - y, p.x - x), a2 = Math.atan2(q.y - y, q.x - x);
		let d = a2 - a1;
		while (d > Math.PI) d -= TAU;
		while (d < -Math.PI) d += TAU;
		a += d;
	}
	return Math.round(a / TAU);
}
function sameHomotopy(oldP: RenderedPrim[], newP: RenderedPrim[], pts: Array<{ x: number; y: number }>): boolean {
	const loop = [...samples(oldP, 0.5), ...samples(newP, 0.5).reverse()];
	return pts.every(p => winding(loop, p.x, p.y) === 0);
}

/** 尖角：转角 ≥30° 且不在端点 / 本网焊盘铜皮内 / 本网过孔上的接缝 */
function sharpCorners(prims: RenderedPrim[], net: string, input: Partial<TightenJobInput>): number {
	let n = 0;
	const ps = input.padShapes ?? {};
	for (let i = 1; i < prims.length; i++) {
		const J = pS(prims[i]);
		const fixed = (input.allPads ?? []).some(p => {
			if (p.net !== net) return false;
			const sh = ps[`${p.x.toFixed(2)},${p.y.toFixed(2)}`];
			return padShapeDistance({ pad: sh?.pad ?? ['ELLIPSE', p.radius * 2, p.radius * 2], rotation: sh?.rotation ?? 0 }, J.x - p.x, J.y - p.y) <= 0.01;
		}) || (input.vias ?? []).some(v => v.net === net && Math.hypot(v.x - J.x, v.y - J.y) < v.dia / 2);
		if (fixed) continue;
		const d1 = C.primDirEnd(prims[i - 1]), d2 = C.primDirStart(prims[i]);
		const t = Math.abs(Math.atan2(d1.x * d2.y - d1.y * d2.x, d1.x * d2.x + d1.y * d2.y));
		if (t >= (30 * Math.PI) / 180) n++;
	}
	return n;
}

/** 通用验收：3 DRC 不恶化、4 连通、5 不变长、连续性；返回各 run 新几何 */
function checkJob(name: string, input: Partial<TightenJobInput> & { runs: Run[] }, out: JobOut, opts: { sharp0?: boolean } = {}): void {
	let contiguous = true, ends = true, longer = 0, layerOk = true;
	for (const r of input.runs) {
		const np = out.final.get(r.id)!;
		for (let i = 1; i < np.length; i++) {
			const e = pE(np[i - 1]), s = pS(np[i]);
			if (Math.hypot(e.x - s.x, e.y - s.y) > 0.01) contiguous = false;
		}
		const s = pS(np[0]), e = pE(np[np.length - 1]);
		if (Math.hypot(s.x - r.a.x, s.y - r.a.y) > 0.01 || Math.hypot(e.x - r.b.x, e.y - r.b.y) > 0.01) ends = false;
		if (np.some(p => p.layer !== r.layer)) layerOk = false;
		longer = Math.max(longer, len(np) - len(r.prims));
	}
	assert(contiguous && ends && layerOk, `${name}: 链连续、端点不动、层不变`);
	assert(longer <= 0.01, `${name}: 每段都不变长（最大增量 ${longer.toFixed(4)}）`);
	const before = gaps(input, input.runs, new Map());
	const after = gaps(input, input.runs, out.final);
	let worst = Infinity, worstKey = '';
	for (const [k, ga] of after) {
		const gb = before.get(k) ?? Infinity;
		const margin = ga - Math.min(0, gb);
		if (margin < worst) { worst = margin; worstKey = `${k} before ${gb.toFixed(3)} after ${ga.toFixed(3)}`; }
	}
	assert(worst >= -0.01, `${name}: 严格 DRC（实际线宽）无新增/无恶化（最差余量 ${worst.toFixed(4)} ${worst < -0.01 ? worstKey : ''}）`);
	// 连通：旧走线接触过的本网焊盘/过孔，新走线仍接触
	let conn = true;
	for (const r of input.runs) {
		const w = r.width ?? drc.traceWidthMil;
		const items = chkItems({ allPads: input.allPads, padShapes: input.padShapes, vias: input.vias }, new Map(), []);
		for (const it of items) {
			if (it.net !== r.net || it.key.startsWith('hole')) continue;
			const touchedBefore = samples(r.prims, 0.1).some(s => it.d(s.x, s.y) < w / 2 - 0.05);
			const touchedAfter = samples(out.final.get(r.id)!, 0.1).some(s => it.d(s.x, s.y) < w / 2 - 0.05);
			if (touchedBefore && !touchedAfter) conn = false;
		}
	}
	assert(conn, `${name}: 本网焊盘/过孔连通不变`);
	if (opts.sharp0) {
		const n = input.runs.reduce((s, r) => s + sharpCorners(out.final.get(r.id)!, r.net, input), 0);
		assert(n === 0, `${name}: 无残留尖角（${n}）`);
	}
}

/** 解析：A→B 绕单圆（圆心 O 半径 R）的最短路长度；side = 经过方向 */
function wrapLen(A: { x: number; y: number }, B: { x: number; y: number }, O: { x: number; y: number }, R: number, ccw: boolean): number {
	const dA = Math.hypot(A.x - O.x, A.y - O.y), dB = Math.hypot(B.x - O.x, B.y - O.y);
	const aA = Math.atan2(A.y - O.y, A.x - O.x), aB = Math.atan2(B.y - O.y, B.x - O.x);
	let span = ccw ? aB - aA : aA - aB;
	while (span < 0) span += TAU;
	const phi = span - Math.acos(R / dA) - Math.acos(R / dB);
	return Math.sqrt(dA * dA - R * R) + Math.sqrt(dB * dB - R * R) + R * Math.max(0, phi);
}

// ─── 用例 ───────────────────────────────────────────────────────────────────

describe('开阔处直角 → 直线', async () => {
	const input = { runs: [mkRun(0, 'A', poly('A', [[0, 0], [100, 0], [100, 100]]))] };
	const out = await job(input);
	checkJob('直角', input, out, { sharp0: true });
	assert(Math.abs(len(out.final.get(0)!) - Math.hypot(100, 100)) < 0.01, `直角拉成对角直线（${len(out.final.get(0)!).toFixed(3)}）`);
	assert(out.done?.passes <= 4, 'tightenDone 带 passes');
});

describe('绕外网圆焊盘：切线 + 圆弧（解析最短路）', async () => {
	const ps: Record<string, PadShape> = {};
	const P = pad('B', 70, 30, ['ELLIPSE', 20, 20], ps);
	const input = { allPads: [P], padShapes: ps, runs: [mkRun(0, 'A', poly('A', [[0, 0], [100, 0], [100, 100]]))] };
	const out = await job(input);
	checkJob('绕盘', input, out, { sharp0: true });
	const exp = wrapLen({ x: 0, y: 0 }, { x: 100, y: 100 }, { x: 70, y: 30 }, 10 + 6 + 5 + C.RENDER_MARGIN, true);
	const got = len(out.final.get(0)!);
	assert(Math.abs(got - exp) < 0.05, `长度 = 解析最短路（${got.toFixed(3)} vs ${exp.toFixed(3)}）`);
	assert(sameHomotopy(input.runs[0].prims, out.final.get(0)!, [{ x: 70, y: 30 }]), '同伦不变（焊盘在同一侧）');
});

describe('非 10mil 线宽：20mil 走线贴焊盘、6mil 走线互相贴', async () => {
	const ps: Record<string, PadShape> = {};
	const P = pad('B', 70, 40, ['RECT', 20, 16], ps);
	const wide = mkRun(0, 'A', poly('A', [[0, 0], [100, 0], [100, 100]]), 20);
	const input = { allPads: [P], padShapes: ps, runs: [wide] };
	const out = await job(input);
	checkJob('20mil', input, out, { sharp0: true });
	const g = gaps(input, input.runs, out.final).get('0|pad:' + P.id)!;
	assert(g >= C.MIN_MARGIN - 0.01 && g < C.RENDER_MARGIN + 0.1, `20mil 走线按实际线宽贴着焊盘，并留出回写误差余量（gap−rule = ${g.toFixed(3)}）`);
	// 两条 6mil：C 是外网直段，挡在 A 的对角线上
	const a6 = mkRun(1, 'A', poly('A', [[0, 0], [100, 0], [100, 100]]), 6);
	const c6 = mkRun(2, 'C', poly('C', [[55, 20], [85, 20]]), 6);
	const input2 = { runs: [a6, c6] };
	const out2 = await job(input2);
	checkJob('6mil', input2, out2, { sharp0: true });
	const g2 = gaps(input2, input2.runs, out2.final).get('1|run:2')!;
	// 对方是走线：再留一份回写误差（两侧都可能被回写）
	assert(g2 >= C.MIN_MARGIN + C.WRITER_ERR - 0.01 && g2 < C.RENDER_MARGIN + C.WRITER_ERR + 0.1, `6mil 对 6mil 贴边 6mil 间距 + 回写误差余量（gap−rule = ${g2.toFixed(3)}）`);
	assert(sameHomotopy(a6.prims, out2.final.get(1)!, [{ x: 55, y: 20 }, { x: 85, y: 20 }, { x: 70, y: 20 }]), '同伦不变（不跳过 C）');
});

describe('长胶囊障碍：绕远端端帽，不穿中段', async () => {
	const trk = { net: 'B', layer: 1, width: 10, prim: L('B', 20, 30, 300, 30) };
	const run = mkRun(0, 'A', poly('A', [[100, 0], [320, 0], [320, 60], [100, 60]]));
	const input = { tracks: [trk], runs: [run] };
	const out = await job(input);
	checkJob('长胶囊', input, out, { sharp0: true });
	const pts = [];
	for (let x = 20; x <= 300; x += 10) pts.push({ x, y: 30 });
	assert(sameHomotopy(run.prims, out.final.get(0)!, pts), '同伦不变（没有穿过胶囊）');
	const R = 5 + 6 + 5 + C.RENDER_MARGIN + C.WRITER_ERR;   // 对方是走线：额外 WRITER_ERR
	const exp = wrapLen({ x: 100, y: 0 }, { x: 100, y: 60 }, { x: 300, y: 30 }, R, true);
	assert(Math.abs(len(out.final.get(0)!) - exp) < 0.05, `长度 = 绕端帽解析最短路（${len(out.final.get(0)!).toFixed(3)} vs ${exp.toFixed(3)}）`);
	// 旧口径：同一走线只出现在 baseWorld（预膨胀圆群）里——必须还原成整根胶囊，不能从腰部穿
	const r = 5 + 6 + 5 + 0.5;
	const base = buildPolylineObstacle([{ x: 150, y: 40 }, { x: 250, y: 40 }], r, 'B', 1, 'line:legacy');
	const run2 = mkRun(1, 'A', poly('A', [[40, 60], [200, -10], [360, 60]]));
	const input2 = { baseWorld: base, runs: [run2] };
	const out2 = await job(input2);
	checkJob('旧口径圆群', input2, out2, { sharp0: true });
	const capD = Math.min(...samples(out2.final.get(1)!, 0.1).map(s => segD(s.x, s.y, 150, 40, 250, 40)));
	assert(capD >= r - 0.01, `到骨架距离 ≥ 预膨胀半径（${capD.toFixed(3)} ≥ ${r}，圆群腰部只有 ${(r * 0.436).toFixed(1)}）`);
	assert(sameHomotopy(run2.prims, out2.final.get(1)!, [{ x: 150, y: 40 }, { x: 200, y: 40 }, { x: 250, y: 40 }]), '同伦不变（没有跳到障碍另一侧）');
});

describe('穿过中间本网焊盘：焊盘点固定，仍接触', async () => {
	const ps: Record<string, PadShape> = {};
	const P = pad('A', 100, 30, ['RECT', 20, 20], ps);
	const Q = pad('B', 100, -10, ['ELLIPSE', 10, 10], ps);
	const run = mkRun(0, 'A', poly('A', [[0, 0], [100, 30], [200, 0]]));
	const input = { allPads: [P, Q], padShapes: ps, runs: [run] };
	const out = await job(input);
	checkJob('穿盘', input, out, { sharp0: true });
	const np = out.final.get(0)!;
	const dIn = Math.min(...samples(np, 0.1).map(s => padShapeDistance({ pad: ['RECT', 20, 20] }, s.x - 100, s.y - 30)));
	assert(dIn < -0.4, `新走线中心线仍进入焊盘铜皮（${dIn.toFixed(2)}）`);
	assert(C.locateOnChain(np, 100, 30).d < 1e-6, '正穿焊盘中心：中心是硬途经点（该处尖角属允许）');
	// 偏心穿过（离中心 4mil）：固定点在链上离中心最近处，拐角在焊盘铜皮内倒圆
	const run2 = mkRun(1, 'A', poly('A', [[0, 0], [100, 26], [200, 0]]));
	const input2 = { allPads: [P, Q], padShapes: ps, runs: [run2] };
	const out2 = await job(input2);
	checkJob('偏心穿盘', input2, out2, { sharp0: true });
	const np2 = out2.final.get(1)!;
	const dIn2 = Math.min(...samples(np2, 0.1).map(s => padShapeDistance({ pad: ['RECT', 20, 20] }, s.x - 100, s.y - 30)));
	assert(dIn2 < -0.4 && len(np2) < len(run2.prims), `偏心穿盘：拐角倒圆变短且仍在焊盘铜皮内（${dIn2.toFixed(2)}）`);
	const turns = np2.slice(1).map((p, i) => {
		const u = C.primDirEnd(np2[i]), v = C.primDirStart(p);
		return Math.abs(Math.atan2(u.x * v.y - u.y * v.x, u.x * v.x + u.y * v.y));
	});
	assert(turns.every(t => t < (2 * Math.PI) / 180), '偏心穿盘：无任何非切向接缝');
});

describe('大半径圆弧贴焊盘：精确校验，不被 15° 采样骗', async () => {
	const ps: Record<string, PadShape> = {};
	const P = pad('B', 100, 5, ['ELLIPSE', 8, 8], ps);
	const R = 200, cy = -Math.sqrt(R * R - 100 * 100);
	const a0 = Math.atan2(-cy, -100), a1 = Math.atan2(-cy, 100);
	const run = mkRun(0, 'A', [ARC('A', 100, cy, R, a0, a1 - a0)]);
	const input = { allPads: [P], padShapes: ps, runs: [run] };
	const out = await job(input);
	checkJob('大弧', input, out, { sharp0: true });
	assert(sameHomotopy(run.prims, out.final.get(0)!, [{ x: 100, y: 5 }]), '同伦不变（焊盘仍在弧下方）');
	const exp = wrapLen({ x: 0, y: 0 }, { x: 200, y: 0 }, { x: 100, y: 5 }, 4 + 6 + 5 + C.RENDER_MARGIN, false);
	assert(Math.abs(len(out.final.get(0)!) - exp) < 0.05, `拉紧到绕盘最短路（${len(out.final.get(0)!).toFixed(3)} vs ${exp.toFixed(3)}）`);
});

describe('>120° 发夹折返（内有外网焊盘）', async () => {
	const ps: Record<string, PadShape> = {};
	const P = pad('B', 100, 20, ['ELLIPSE', 6, 6], ps);
	const run = mkRun(0, 'A', poly('A', [[0, 0], [200, 0], [0, 80]]));
	const input = { allPads: [P], padShapes: ps, runs: [run] };
	const out = await job(input);
	checkJob('发夹', input, out, { sharp0: true });
	assert(sameHomotopy(run.prims, out.final.get(0)!, [{ x: 100, y: 20 }]), '同伦不变（仍绕着焊盘折返）');
	const run2 = mkRun(1, 'A', poly('A', [[0, 0], [200, 0], [0, 80]]));
	const out2 = await job({ runs: [run2] });
	assert(Math.abs(len(out2.final.get(1)!) - Math.hypot(0, 80)) < 0.01, '无障碍时折返拉成直线');
});

describe('非切向线-弧接缝 → 圆角（内核直接调用）', () => {
	const world = new C.TightenWorld();
	const env: C.RunEnv = { world, net: 'A', layer: 1, width: 10, selfGroups: new Set(), origSlack: () => 1e9 };
	const la = [L('A', 0, 0, 50, 0), ARC('A', 80, 0, 30, Math.PI, -Math.PI / 2)];
	const fx: C.FixedPt[] = [{ x: 0, y: 0, kind: 'end' }, { x: 80, y: 30, kind: 'end' }];
	const r = C.tightenChain(env, la, fx, { taut: false })!;
	const st = C.cornerStats(r.prims, fx);
	assert(!!r && st.kinks === 0 && C.chainMaxGap(r.prims) < 1e-6, `线-弧非切向接缝被圆掉（kinks ${st.kinks}）`);
	assert(len(r.prims) < len(la), '圆角后更短');
	const al = [ARC('A', 0, 30, 30, -Math.PI / 2, Math.PI / 2), L('A', 30, 30, 30, 80)].map((p, i) => i === 0 ? p : p);
	// 弧-线：弧终点 (30,30) 切向向上，线向右上 → 改成向右
	const al2 = [ARC('A', 0, 30, 30, -Math.PI / 2, Math.PI / 2), L('A', 30, 30, 90, 30)];
	const fx2: C.FixedPt[] = [{ x: 0, y: 0, kind: 'end' }, { x: 90, y: 30, kind: 'end' }];
	const r2 = C.tightenChain(env, al2, fx2, { taut: false })!;
	assert(!!r2 && C.cornerStats(r2.prims, fx2).kinks === 0 && C.chainMaxGap(r2.prims) < 1e-6, '弧-线非切向接缝被圆掉');
	void al;
	// 带障碍：拉紧不可能完全拉直时，仍无尖角
	const job2Pads: Record<string, PadShape> = {};
	const P = pad('B', 62, 12, ['ELLIPSE', 6, 6], job2Pads);
	return job({ allPads: [P], padShapes: job2Pads, runs: [mkRun(0, 'A', la)] }).then(out => {
		checkJob('线弧+障碍', { allPads: [P], padShapes: job2Pads, runs: [mkRun(0, 'A', la)] }, out, { sharp0: true });
	});
});

describe('T 型分支：节点不动，各臂拉紧', async () => {
	const ps: Record<string, PadShape> = {};
	const pads = [pad('A', 0, 0, ['ELLIPSE', 16, 16], ps), pad('A', 200, 0, ['ELLIPSE', 16, 16], ps), pad('A', 100, 200, ['ELLIPSE', 16, 16], ps)];
	const runs = [
		mkRun(0, 'A', poly('A', [[0, 0], [0, 100], [100, 100]])),
		mkRun(1, 'A', poly('A', [[100, 100], [200, 100], [200, 0]])),
		mkRun(2, 'A', poly('A', [[100, 100], [100, 150], [100, 200]])),
	];
	const input = { allPads: pads, padShapes: ps, runs };
	const out = await job(input);
	checkJob('T 分支', input, out, { sharp0: true });
	const ok = [0, 1, 2].every(i => {
		const np = out.final.get(i)!;
		const s = pS(np[0]), e = pE(np[np.length - 1]);
		return Math.hypot(s.x - runs[i].a.x, s.y - runs[i].a.y) < 1e-6 && Math.hypot(e.x - runs[i].b.x, e.y - runs[i].b.y) < 1e-6;
	});
	assert(ok, 'T 节点 (100,100) 三臂共用');
	assert(Math.abs(len(out.final.get(0)!) - Math.hypot(100, 100)) < 0.01 && Math.abs(len(out.final.get(1)!) - Math.hypot(100, 100)) < 0.01, '两臂拉成直线');
	// T 接在中段（非节点）：分支端点落在主干中间
	const runsT = [
		mkRun(3, 'A', poly('A', [[0, 0], [0, 100], [200, 100], [200, 0]])),
		mkRun(4, 'A', poly('A', [[100, 100], [100, 200]])),
	];
	const outT = await job({ allPads: pads, padShapes: ps, runs: runsT });
	checkJob('中段 T', { allPads: pads, padShapes: ps, runs: runsT }, outT);
	const dT = C.locateOnChain(outT.final.get(3)!, 100, 100).d;
	assert(dT < 0.01, `主干仍经过分支接点（${dT.toFixed(4)}）`);
});

describe('45° 楼梯在两个外网焊盘之间：绕行，不跳过', async () => {
	const ps: Record<string, PadShape> = {};
	const P = pad('B', 40, 10, ['ELLIPSE', 10, 10], ps);
	const Q = pad('B', 80, 30, ['ELLIPSE', 10, 10], ps);
	const run = mkRun(0, 'A', poly('A', [[0, 0], [40, 40], [80, 0], [120, 40], [160, 0]]));
	const input = { allPads: [P, Q], padShapes: ps, runs: [run] };
	const out = await job(input);
	checkJob('楼梯', input, out, { sharp0: true });
	assert(sameHomotopy(run.prims, out.final.get(0)!, [{ x: 40, y: 10 }, { x: 80, y: 30 }]), '同伦不变（P 在下、Q 在上）');
	assert(len(out.final.get(0)!) < len(run.prims) - 10, `明显变短（${len(run.prims).toFixed(1)} → ${len(out.final.get(0)!).toFixed(1)}）`);
});

describe('1.5mil 碎段：链保持连续', async () => {
	const ps: Record<string, PadShape> = {};
	const P = pad('B', 45, 22, ['ELLIPSE', 10, 10], ps);
	const run = mkRun(0, 'A', poly('A', [[0, 0], [50, 0], [51, 1.1], [51, 60]]));
	const input = { allPads: [P], padShapes: ps, runs: [run] };
	const out = await job(input);
	checkJob('碎段', input, out, { sharp0: true });
	const world = new C.TightenWorld();
	const env: C.RunEnv = { world, net: 'A', layer: 1, width: 10, selfGroups: new Set(), origSlack: () => 1e9 };
	const fx: C.FixedPt[] = [{ x: 0, y: 0, kind: 'end' }, { x: 51, y: 60, kind: 'end' }];
	const r = C.tightenChain(env, run.prims, fx, { taut: false })!;
	assert(C.chainMaxGap(r.prims) < 1e-6 && C.cornerStats(r.prims, fx).sharp === 0, '只圆角时碎段不丢、无断口、无尖角');
});

describe('回写误差预算：原几何只比规则宽 0.03 时，新几何须离规则 ≥ MIN_MARGIN（或不动）', async () => {
	const ps: Record<string, PadShape> = {};
	const P = pad('B', 100 - 21.03, 40, ['ELLIPSE', 20, 20], ps);   // 竖边 x=100 离盘铜皮 11.03 → gap − rule = 0.03
	const run = mkRun(0, 'A', poly('A', [[0, 0], [100, 0], [100, 100]]));
	const input = { allPads: [P], padShapes: ps, runs: [run] };
	const before = gaps(input, input.runs, new Map()).get('0|pad:' + P.id)!;
	const out = await job(input);
	checkJob('窄余量', input, out, { sharp0: true });
	const after = gaps(input, input.runs, out.final).get('0|pad:' + P.id)!;
	assert(Math.abs(before - 0.03) < 1e-6, `（前提）原几何余量 0.03（${before.toFixed(4)}）`);
	assert(len(out.final.get(0)!) < len(run.prims) - 1, `仍然拉紧（${len(run.prims).toFixed(1)} → ${len(out.final.get(0)!).toFixed(1)}）`);
	assert(after >= C.MIN_MARGIN - 0.01, `新几何离规则 ≥ MIN_MARGIN ${C.MIN_MARGIN.toFixed(2)}（回写 ≤ ${C.WRITER_ERR}mil 误差 + EDA 口径之后仍合规）：${after.toFixed(4)}`);
	assert(sameHomotopy(run.prims, out.final.get(0)!, [{ x: P.x, y: P.y }]), '同伦不变');
	// 原几何违规（−0.5）：不恶化即可（回写误差无法再补偿）——见下一个用例
	assert(C.MIN_MARGIN >= C.WRITER_ERR + 0.05 - 1e-12 && C.RENDER_MARGIN >= C.MIN_MARGIN + 0.05 - 1e-12, '常量：MIN_MARGIN ≥ 回写误差 + 0.05；RENDER_MARGIN ≥ MIN_MARGIN + 0.05');
});

describe('既有违规不恶化', async () => {
	const ps: Record<string, PadShape> = {};
	const P = pad('B', 90, 10, ['ELLIPSE', 10, 10], ps); // 距竖边中心 10 → gap 0 < 6
	const run = mkRun(0, 'A', poly('A', [[0, 0], [100, 0], [100, 100]]));
	const input = { allPads: [P], padShapes: ps, runs: [run] };
	const before = gaps(input, input.runs, new Map()).get('0|pad:' + P.id)!;
	const out = await job(input);
	checkJob('既有违规', input, out);
	const after = gaps(input, input.runs, out.final).get('0|pad:' + P.id)!;
	assert(before < 0 && after >= before - 0.01, `违规深度不加深（${before.toFixed(3)} → ${after.toFixed(3)}）`);
	assert(sameHomotopy(run.prims, out.final.get(0)!, [{ x: 90, y: 10 }]), '同伦不变');
});

describe('旧副本：baseWorld 里的只读走线副本让位给精确铜皮', async () => {
	const trk = { net: 'B', layer: 1, width: 10, prim: L('B', 40, 30, 300, 30) };
	const copy = buildPolylineObstacle([{ x: 40, y: 30 }, { x: 300, y: 30 }], 5 + 6 + 5 + 0.5, 'B', 1, 'line:copy');
	const run = mkRun(0, 'A', poly('A', [[100, 0], [320, 0], [320, 60], [100, 60]]));
	const input = { tracks: [trk], baseWorld: copy, runs: [run] };
	const out = await job(input);
	const d = Math.min(...samples(out.final.get(0)!, 0.1).map(s => segD(s.x, s.y, 40, 30, 300, 30)));
	assert(d < 16.4 && d >= 16 + C.MIN_MARGIN + C.WRITER_ERR - 0.01, `按精确铜皮 16mil 贴边（${d.toFixed(3)}，副本口径会是 16.5）`);
});

describe('过孔与 TH 钻孔：精确圆，全层', async () => {
	const ps: Record<string, PadShape> = {};
	const P = pad('B', 60, 25, ['ELLIPSE', 30, 30], ps, { layer: 12, padType: 'th', holeDiameter: 16 });
	const vias: PlannedVia[] = [{ net: 'C', x: 75, y: 75, dia: 20, hole: 10 }];
	const run = mkRun(0, 'A', poly('A', [[0, 0], [100, 0], [100, 100]]), 8);
	const runB = mkRun(1, 'A', poly('A', [[0, 0], [0, 100], [100, 100]], 2), 8);
	const input = { allPads: [P], padShapes: ps, vias, runs: [run, runB] };
	const out = await job(input);
	checkJob('过孔/TH', input, out, { sharp0: true });
	assert(sameHomotopy(runB.prims, out.final.get(1)!, [{ x: 75, y: 75 }]), '底层段同伦不变（过孔全层阻挡）');
});

describe('幂等：结果再拉一次几乎不变', async () => {
	const ps: Record<string, PadShape> = {};
	const pads = [pad('B', 70, 30, ['ELLIPSE', 20, 20], ps), pad('A', 300, 30, ['RECT', 20, 20], ps), pad('C', 340, 10, ['ELLIPSE', 10, 10], ps), pad('C', 380, 30, ['ELLIPSE', 10, 10], ps)];
	const runs = [
		mkRun(0, 'A', poly('A', [[0, 0], [100, 0], [100, 100]])),
		mkRun(1, 'A', poly('A', [[200, 0], [300, 30], [400, 0]])),
		mkRun(2, 'D', poly('D', [[300, 100], [340, 140], [380, 100], [420, 140], [460, 100]])),
		mkRun(3, 'E', poly('E', [[0, 200], [60, 200], [60, 260], [120, 260]]), 15),
	];
	const input = { allPads: pads, padShapes: ps, runs };
	const out1 = await job(input);
	checkJob('第一次', input, out1, { sharp0: true });
	const runs2 = runs.map(r => ({ ...r, prims: out1.final.get(r.id)! }));
	const input2 = { allPads: pads, padShapes: ps, runs: runs2 };
	const out2 = await job(input2);
	checkJob('第二次', input2, out2, { sharp0: true });
	const t1 = runs.reduce((s, r) => s + len(out1.final.get(r.id)!), 0);
	const t2 = runs.reduce((s, r) => s + len(out2.final.get(r.id)!), 0);
	assert(Math.abs(t1 - t2) <= 0.5, `二次总长变化 ≤0.5mil（${t1.toFixed(3)} → ${t2.toFixed(3)}）`);
	const adopted2 = out2.events.filter(e => e.type === 'tightenRun' && e.prims).length;
	console.log(`    （二次采纳 ${adopted2} 次）`);
});

describe('静态填充（多边形含内部）与多段板框', async () => {
	// 填充：预膨胀 r 的多边形组；板框：4 条独立线共用 group 'board'
	const rFill = 6 + 5 + 2;
	const fill = buildPolygonObstacle([{ x: 60, y: 20 }, { x: 90, y: 20 }, { x: 90, y: 50 }, { x: 60, y: 50 }, { x: 60, y: 20 }], rFill, 'F', 1, 'fill:1', true);
	const rB = 10 + 5;
	const board: RBObstacle[] = [
		...buildPolylineObstacle([{ x: -20, y: -20 }, { x: 130, y: -20 }], rB, undefined, undefined, 'board', true),
		...buildPolylineObstacle([{ x: 130, y: -20 }, { x: 130, y: 130 }], rB, undefined, undefined, 'board', true),
		...buildPolylineObstacle([{ x: 130, y: 130 }, { x: -20, y: 130 }], rB, undefined, undefined, 'board', true),
		...buildPolylineObstacle([{ x: -20, y: 130 }, { x: -20, y: -20 }], rB, undefined, undefined, 'board', true),
	];
	const run = mkRun(0, 'A', poly('A', [[0, 0], [110, 0], [110, 100]]));
	const input = { baseWorld: [...fill, ...board], runs: [run] };
	const out = await job(input);
	checkJob('填充/板框', input, out, { sharp0: true });
	const sq = [{ x: 60, y: 20 }, { x: 90, y: 20 }, { x: 90, y: 50 }, { x: 60, y: 50 }];
	let dMin = Infinity;
	for (const s of samples(out.final.get(0)!, 0.1)) {
		for (let k = 0; k < 4; k++) dMin = Math.min(dMin, segD(s.x, s.y, sq[k].x, sq[k].y, sq[(k + 1) % 4].x, sq[(k + 1) % 4].y));
	}
	assert(dMin >= rFill - 0.01, `与填充多边形边距 ≥ 预膨胀半径（${dMin.toFixed(3)} ≥ ${rFill}）`);
	assert(sameHomotopy(run.prims, out.final.get(0)!, [{ x: 75, y: 35 }, { x: 60, y: 20 }, { x: 90, y: 50 }]), '同伦不变（填充仍在同一侧）');
	let dB = Infinity;
	for (const s of samples(out.final.get(0)!, 0.1)) dB = Math.min(dB, 130 - s.x, s.y + 20);
	assert(dB >= rB - 0.01, `离板框 ≥ 预膨胀半径（${dB.toFixed(3)}）`);
});

describe('本网静态铜（同网铺铜）：接触着的走线不被拉离', async () => {
	const fill = buildPolygonObstacle([{ x: 60, y: 20 }, { x: 90, y: 20 }, { x: 90, y: 50 }, { x: 60, y: 50 }, { x: 60, y: 20 }], 13, 'A', 1, 'fill:own', true);
	const run = mkRun(0, 'A', poly('A', [[0, 0], [75, 35], [150, 0]]));
	const out = await job({ baseWorld: fill, runs: [run] });
	const sq = [{ x: 60, y: 20 }, { x: 90, y: 20 }, { x: 90, y: 50 }, { x: 60, y: 50 }];
	const inside = samples(out.final.get(0)!, 0.1).some(s => s.x > 60 && s.x < 90 && s.y > 20 && s.y < 50);
	assert(inside, '新走线中心线仍进入本网铺铜');
	void sq;
	const np = out.final.get(0)!;
	assert(len(np) <= len(run.prims) + 0.01 && C.chainMaxGap(np) < 0.01, '不变长、连续');
});

describe('取消：未处理的段也上报（null）并发 tightenDone', async () => {
	const runs = [0, 1, 2].map(i => mkRun(i, 'A', poly('A', [[0, i * 50], [100, i * 50], [100, i * 50 + 30]])));
	const events: TightenJobEvent[] = [];
	await runTightenJob({ type: 'tighten', drc, allPads: [], padShapes: {}, baseWorld: [], vias: [], runs }, m => events.push(m), () => true);
	const ids = new Set(events.filter(e => e.type === 'tightenRun').map(e => (e as any).id));
	assert(ids.size === 3 && events[events.length - 1].type === 'tightenDone', '取消后每段仍上报一次，最后是 tightenDone');
});

describe('splitRunsForTighten：<0.8mil 微图元不打断连续段', () => {
	const eng = new PlanningEngine();
	eng.initTightenWorld(drc, [], [], {});
	// 线 → 0.6mil 小弧（两端落进同一 0.8mil 节点簇）→ 线
	const r = 11, sw = -0.055;
	const a0 = Math.PI / 2;
	const arc = ARC('A', 100, -r, r, a0, sw);
	const e = pE(arc);
	const prims = [L('A', 0, 0, 100, 0), arc, L('A', e.x, e.y, e.x + 80, e.y - 80 * Math.tan(-sw))];
	const segs = eng.splitRunsForTighten(prims, []);
	assert(segs.length === 1 && segs[0].prims.length === 3, `一段、含全部 3 个图元（${segs.length} 段）`);
});

describe('可选阶段 2 / 整网重布（传入 PlanningEngine）：改拓扑但仍守 DRC/连通/不变长', async () => {
	// 阶段 2：绕盘段另一侧有干净直线（A* 会找到），采纳后仍须满足 1/3/4/5
	const ps: Record<string, PadShape> = {};
	const P = pad('B', 70, 30, ['ELLIPSE', 20, 20], ps);
	const input = { allPads: [P], padShapes: ps, runs: [mkRun(0, 'A', poly('A', [[0, 0], [100, 0], [100, 100]]))] };
	const events: TightenJobEvent[] = [];
	await runTightenJob({ type: 'tighten', drc, baseWorld: [], vias: [], ...input }, m => events.push(m), () => false, new PlanningEngine());
	const final = new Map<number, RenderedPrim[]>([[0, input.runs[0].prims]]);
	for (const e of events) if (e.type === 'tightenRun' && e.prims) final.set(e.id, e.prims);
	checkJob('阶段2', input, { events, final, ms: 0, done: events[events.length - 1] }, { sharp0: true });
	assert(len(final.get(0)!) < 174, `A* 换到焊盘另一侧更短（${len(final.get(0)!).toFixed(2)}）`);
	// 整网 MST：三焊盘网被布成远处 T 型大兜圈
	const ps2: Record<string, PadShape> = {};
	const pads2 = [pad('M', 0, 0, ['ELLIPSE', 16, 16], ps2), pad('M', 100, 0, ['ELLIPSE', 16, 16], ps2), pad('M', 0, 40, ['ELLIPSE', 16, 16], ps2)];
	const runs2 = [
		mkRun(10, 'M', poly('M', [[0, 0], [-40, 300], [50, 300]])),
		mkRun(11, 'M', poly('M', [[50, 300], [140, 300], [100, 0]])),
		mkRun(12, 'M', poly('M', [[50, 300], [60, 200], [0, 40]])),
	];
	const ev2: TightenJobEvent[] = [];
	await runTightenJob({ type: 'tighten', drc, baseWorld: [], vias: [], allPads: pads2, padShapes: ps2, runs: runs2 }, m => ev2.push(m), () => false, new PlanningEngine());
	const rw = ev2.find(e => e.type === 'netRewrite') as any;
	assert(!!rw && rw.newLen < 160, `整网改写为 MST（${rw ? rw.newLen.toFixed(1) : '未改写'}）`);
	if (rw) {
		const touched = pads2.every(p => samples(rw.prims, 0.1).some(s => Math.hypot(s.x - p.x, s.y - p.y) < 8));
		assert(touched, '改写后每个焊盘都被接触');
	}
});

describe('精确距离：步进器 vs 稠密采样（弧对矩形/胶囊/弧）', () => {
	let worst = 0;
	let seed = 7;
	const rnd = () => { seed = (seed * 16807) % 2147483647; return seed / 2147483647; };
	for (let k = 0; k < 300; k++) {
		const arc = ARC('A', rnd() * 100, rnd() * 100, 5 + rnd() * 200, rnd() * TAU, (rnd() - 0.5) * 2 * Math.PI);
		const shapes: C.Shape[] = [
			{ t: 'box', x: rnd() * 100, y: rnd() * 100, hx: 2 + rnd() * 20, hy: 2 + rnd() * 20, c: Math.cos(k), s: Math.sin(k) },
			{ t: 'seg', x1: rnd() * 100, y1: rnd() * 100, x2: rnd() * 100, y2: rnd() * 100, rho: rnd() * 5 },
			{ t: 'arc', cx: rnd() * 100, cy: rnd() * 100, r: 3 + rnd() * 50, a0: rnd() * TAU, sw: (rnd() - 0.5) * 6, rho: rnd() * 5 },
		];
		for (const sh of shapes) {
			const m = C.primShapeMin(arc, sh, 1e9);
			let brute = Infinity;
			for (const s of samples([arc], 0.01)) brute = Math.min(brute, C.shapeDist(sh, s.x, s.y));
			// 真值 ∈ [m − 0.002, m]；稠密采样只会偏大
			worst = Math.max(worst, (m - 0.002) - brute);
		}
	}
	assert(worst <= 1e-6, `步进器下界从不高于稠密采样（最大超出 ${worst.toExponential(2)}）`);
});

describe('性能：~150 段 / 80 网合成板', async () => {
	const ps: Record<string, PadShape> = {};
	const pads: EnginePad[] = [];
	const runs: Run[] = [];
	let id = 0;
	for (let n = 0; n < 80; n++) {
		const gx = (n % 10) * 220, gy = Math.floor(n / 10) * 220;
		const net = `N${n}`;
		pads.push(pad(net, gx, gy, ['RECT', 16, 24], ps));
		pads.push(pad(net, gx + 160, gy + 120, ['ELLIPSE', 20, 20], ps));
		pads.push(pad(`X${n}`, gx + 90, gy + 40, ['ELLIPSE', 14, 14], ps));
		runs.push(mkRun(id++, net, poly(net, [[gx, gy], [gx + 40, gy + 40], [gx + 80, gy], [gx + 120, gy + 40], [gx + 160, gy + 40], [gx + 160, gy + 80]])));
		if (n % 7 !== 0) runs.push(mkRun(id++, net, poly(net, [[gx + 160, gy + 80], [gx + 160, gy + 100], [gx + 160, gy + 120]])));
	}
	const input = { allPads: pads, padShapes: ps, runs };
	const out = await job(input);
	console.log(`    ${runs.length} 段：${out.ms}ms，passes ${out.done?.passes}，采纳 ${out.done?.adopted}，省 ${Number(out.done?.saved).toFixed(0)}mil`);
	assert(out.ms < 30000, `150 段在 30s 内（${out.ms}ms）`);
	const reported = new Set(out.events.filter(e => e.type === 'tightenRun').map(e => (e as any).id));
	assert(reported.size === runs.length, '每段至少上报一次');
	checkJob('合成板', input, out, { sharp0: true });
});

// ─── 评审修复回归（chainShapeMin cap / 整网重布 / 步进器与采样细节） ───────────────

describe('评审1：chainShapeMin 对「线穿过矩形/多边形焊盘中段、两端在外」必须给出真最小值', () => {
	const ps: Record<string, PadShape> = {};
	const P = pad('A', 100, 0, ['RECT', 40, 20], ps);
	const box = C.padToShape(P, ps[`${P.x.toFixed(2)},${P.y.toFixed(2)}`]);
	const line = L('A', 0, 0, 200, 0);
	const mBox = C.chainShapeMin([line], box);
	assert(mBox <= 0 && Math.abs(mBox - -10) < 0.01, `直线穿过 RECT 焊盘中心：≈ −10（${mBox}）`);
	// 带旋转的矩形 + 偏心穿过
	const ps2: Record<string, PadShape> = {};
	const P2 = pad('A', 100, 0, ['RECT', 40, 20], ps2, { rotation: 30 });
	const box2 = C.padToShape(P2, ps2[`${P2.x.toFixed(2)},${P2.y.toFixed(2)}`]);
	const m2 = C.chainShapeMin([L('A', 40, -8, 160, 8)], box2);
	assert(m2 <= 0, `斜线穿过旋转 RECT 焊盘：≤ 0（${m2}）`);
	// 多边形焊盘（EDA POLYGON → 矩形）与 poly 形状（填充）
	const ps3: Record<string, PadShape> = {};
	const P3 = pad('A', 100, 0, ['POLYGON', [80, -10, 120, -10, 120, 10, 80, 10]], ps3);
	const sh3 = C.padToShape(P3, ps3[`${P3.x.toFixed(2)},${P3.y.toFixed(2)}`]);
	const m3 = C.chainShapeMin([L('A', 0, 0, 200, 0)], sh3);
	assert(m3 <= 0, `直线穿过 POLYGON 焊盘（${sh3.t}）：≤ 0（${m3}）`);
	const polySh: C.Shape = { t: 'poly', pts: [{ x: 80, y: -10 }, { x: 120, y: -10 }, { x: 125, y: 10 }, { x: 80, y: 12 }] };
	const m4 = C.chainShapeMin([L('A', 0, 0, 200, 0)], polySh);
	assert(m4 <= 0, `直线穿过 poly 形状：≤ 0（${m4}）`);
	// 弧穿过（弧两端在外）
	const arc = ARC('A', 100, -50, 50, Math.PI / 2 - 0.6, 1.2);
	const m5 = C.chainShapeMin([arc], box);
	assert(m5 <= 0, `圆弧穿过 RECT 焊盘：≤ 0（${m5}）`);
	// 不接触时给出真距离（不是端点距离）
	const m6 = C.chainShapeMin([L('A', 0, 30, 200, 30)], box);
	assert(Math.abs(m6 - 20) < 0.01, `平行线离盘 20mil（${m6}）`);
});

describe('评审1：整条 run 直穿本网 RECT 焊盘中段（无顶点）——取直后仍接触该焊盘', async () => {
	const ps: Record<string, PadShape> = {};
	const own = pad('A', 100, 0, ['RECT', 40, 20], ps);
	const foreign = pad('X', 100, 45, ['ELLIPSE', 10, 10], ps);
	// 原走线从外网小盘「下方」穿过自己的焊盘中段（两端点在焊盘之外）；取直路径会被外网盘顶到上方 y≈29，离开焊盘
	const run = mkRun(0, 'A', poly('A', [[0, 50], [50, 0], [150, 0], [200, 50]]));
	const input = { allPads: [own, foreign], padShapes: ps, runs: [run] };
	const out = await job(input);
	const np = out.final.get(0)!;
	const shape = { pad: ['RECT', 40, 20], hole: null, rotation: 0 };
	const touched = samples(np, 0.1).some(s => padShapeDistance(shape, s.x - 100, s.y) <= 0);
	assert(touched, `新走线仍穿过/接触自己的焊盘（最近 ${Math.min(...samples(np, 0.1).map(s => padShapeDistance(shape, s.x - 100, s.y))).toFixed(2)}）`);
	checkJob('直穿焊盘', input, out);
});

describe('评审2：整网重布连通性复核（纯函数 padsConnectedByCopper）', () => {
	const ps: Record<string, PadShape> = {};
	const pads = [pad('M', 0, 0, ['ELLIPSE', 16, 16], ps), pad('M', 100, 0, ['ELLIPSE', 16, 16], ps), pad('M', 0, 40, ['ELLIPSE', 16, 16], ps)];
	const shapes = pads.map(p => C.padToShape(p, ps[`${p.x.toFixed(2)},${p.y.toFixed(2)}`]));
	const w = 10;
	// 候选：焊盘 1-2 之间一条链 + 焊盘 3 处一个短桩。每个焊盘都被接触、每条链两端都落在焊盘上，但是两个连通块
	const chain1 = [L('M', 0, 0, 100, 0)];
	const stub = [L('M', 0, 40, 5, 40)];
	const oldTouch = shapes.every(sh => C.chainShapeMin([...chain1, ...stub], sh) <= 0);
	const oldEnds = [chain1, stub].every(ch => [C.primStart(ch[0]), C.primEnd(ch[ch.length - 1])].every(p => shapes.some(sh => C.shapeDist(sh, p.x, p.y) <= 0)));
	assert(oldTouch && oldEnds, '（前提）旧判据「每盘被接触 + 链端落在焊盘上」会放行该候选');
	assert(padsConnectedByCopper([chain1, stub], shapes, w) === false, '两个连通块的候选被拒绝');
	// 连成一块：桩换成焊盘 3 → 焊盘 1
	assert(padsConnectedByCopper([chain1, [L('M', 0, 40, 0, 0)]], shapes, w) === true, '焊盘 3→1 的链使其连通 → 通过');
	// T 接：桩的端点落在链 1 中段
	assert(padsConnectedByCopper([chain1, [L('M', 0, 40, 50, 0)]], shapes, w) === true, '链端点落在另一链中段（T 接）→ 通过');
	// 端点相接（< 0.8mil）
	assert(padsConnectedByCopper([[L('M', 0, 0, 50, 0)], [L('M', 50.4, 0, 100, 0)], [L('M', 0, 40, 0, 0)]], shapes, w) === true, '链首尾相接（0.4mil）→ 通过');
	// 间断 12mil（> 线宽）的两条链：不连通
	assert(padsConnectedByCopper([[L('M', 0, 0, 44, 0)], [L('M', 56, 0, 100, 0)], [L('M', 0, 40, 0, 0)]], shapes, w) === false, '间断 12mil → 不连通');
	// 链穿过焊盘中段（无顶点）也算接触该焊盘
	assert(padsConnectedByCopper([[L('M', -30, 0, 130, 0)], [L('M', 0, -30, 0, 60)]], shapes, w) === true, '链穿过焊盘中段也算连到该焊盘');
});

describe('评审2：整网重布不得丢掉同网静态铜（铺铜/填充）上的锚点', async () => {
	const mk = (withFill: boolean) => {
		const ps2: Record<string, PadShape> = {};
		const pads2 = [pad('M', 0, 0, ['ELLIPSE', 16, 16], ps2), pad('M', 100, 0, ['ELLIPSE', 16, 16], ps2), pad('M', 0, 40, ['ELLIPSE', 16, 16], ps2)];
		const runs2 = [
			mkRun(10, 'M', poly('M', [[0, 0], [-40, 300], [50, 300]])),
			mkRun(11, 'M', poly('M', [[50, 300], [140, 300], [100, 0]])),
			mkRun(12, 'M', poly('M', [[50, 300], [60, 200], [0, 40]])),
		];
		const fill = withFill
			? buildPolygonObstacle([{ x: -60, y: 280 }, { x: -20, y: 280 }, { x: -20, y: 320 }, { x: -60, y: 320 }, { x: -60, y: 280 }], 13, 'M', 1, 'fill:own', true)
			: [];
		return { allPads: pads2, padShapes: ps2, baseWorld: fill, runs: runs2 };
	};
	const run = async (inp: ReturnType<typeof mk>) => {
		const ev: TightenJobEvent[] = [];
		await runTightenJob({ type: 'tighten', drc, vias: [], ...inp } as TightenJobInput, m => ev.push(m), () => false, new PlanningEngine());
		return ev;
	};
	const ctl = await run(mk(false));
	assert(ctl.some(e => e.type === 'netRewrite'), '对照：无同网铺铜时整网被改写');
	const ev = await run(mk(true));
	assert(!ev.some(e => e.type === 'netRewrite'), '走线接触同网静态铜 → 跳过整网重布');
});

describe('评审3a：步进器迭代耗尽时返回保守下界（不是已采样的最小值）', () => {
	// 一条 10000mil 的线以 5e-4 斜率逼近一个矩形：真最小值在终点 0；步进 ≥ 0.004 → 需 ~2.5M 步 > 守卫
	// （用 poly 形状：线-矩形走凸函数三分的闭式路径，不经过步进器）
	const box: C.Shape = { t: 'poly', pts: [{ x: -20000, y: -1 }, { x: 20000, y: -1 }, { x: 20000, y: 1 }, { x: -20000, y: 1 }] };
	const line = L('A', -5000, 6, 5000, 1);
	const m = C.primShapeMin(line, box, Infinity);
	assert(m <= 0, `守卫耗尽时返回值 ≤ 真最小值 0（${m.toFixed(3)}）`);
});

describe('评审3b：步进器误差 ≤ TRACE_TAU（由两侧 Lipschitz 可证；V 形最小值的对抗测试）', () => {
	// 直线穿过矩形中心：signed distance 在中心处是斜率 ±1 的尖 V 形最小值；起点相位随机，检验返回值 − 真值 ∈ [0, 0.002]
	let seed = 11;
	const rnd = () => { seed = (seed * 16807) % 2147483647; return seed / 2147483647; };
	let worstOver = 0, worstUnder = 0;
	for (let k = 0; k < 4000; k++) {
		const hx = 5 + rnd() * 30, hy = 2 + rnd() * 20, oy = (rnd() - 0.5) * 2 * hy * 0.5;
		const x0 = -hx - 1 - rnd() * 0.2 - rnd() * 3, x1 = hx + 5;
		const box: C.Shape = { t: 'box', x: 0, y: 0, hx, hy, c: 1, s: 0 };
		// poly 形状走球面步进器（box 对线段走凸函数闭式路径，另测）
		const pl: C.Shape = { t: 'poly', pts: [{ x: -hx, y: -hy }, { x: hx, y: -hy }, { x: hx, y: hy }, { x: -hx, y: hy }] };
		const trueMin = Math.max(-hx, Math.abs(oy) - hy);
		const line = L('A', x0, oy, x1, oy);
		const m = C.primShapeMin(line, pl, Infinity);
		worstOver = Math.max(worstOver, m - trueMin);
		worstUnder = Math.max(worstUnder, trueMin - m);
		const mb = C.primShapeMin(line, box, Infinity);
		worstOver = Math.max(worstOver, mb - trueMin);
		worstUnder = Math.max(worstUnder, trueMin - mb);
	}
	assert(worstUnder <= 1e-9, `返回值从不低于真值（${worstUnder.toExponential(2)}）`);
	assert(worstOver <= 0.002 + 1e-9, `返回值高于真值不超过 TRACE_TAU（${worstOver.toFixed(5)}）`);
});

describe('评审3c：regionSwallows 不能只看代表点（焊盘中心在外、角在切角区内）', () => {
	const ps: Record<string, PadShape> = {};
	// 外网 60×60 焊盘压着原走线（既有违规）：中心 (60,−20) 在切角区外，上边角 (90,10) 在切角区内
	const P = pad('B', 60, -20, ['RECT', 60, 60], ps);
	const world = new C.TightenWorld();
	C.padItems(world, [P], ps, drc);
	const prims = poly('A', [[0, 0], [100, 0], [100, 100]]);
	const env: C.RunEnv = {
		world, net: 'A', layer: 1, width: 10, selfGroups: new Set(),
		origSlack: g => C.slackOfPrimsVsItems(prims, 10, world.groupItems(g)),
	};
	const out = C.roundCorners(env, prims, [{ x: 0, y: 0, kind: 'end' }, { x: 100, y: 100, kind: 'end' }]);
	const corners = [{ x: 90, y: 10 }, { x: 30, y: 10 }, { x: 30, y: -50 }, { x: 90, y: -50 }];
	assert(out.some(p => p.kind === 'arc'), `仍然圆角了（${out.length} 个图元）`);
	assert(sameHomotopy(prims, out, corners), '圆角没有把焊盘的角划到走线另一侧');
});

describe('评审3d：arcSteps 的弦高界不被 720 步封顶破坏（大半径）', () => {
	let worst = 0;
	for (const [s, span, sag] of [[1e4, TAU, 0.002], [3000, TAU, 0.02], [1e5, Math.PI, 0.02], [500, TAU, 0.002]] as const) {
		const n = C.arcSteps(s, span, sag);
		const sagActual = s * (1 - Math.cos(span / n / 2));
		worst = Math.max(worst, sagActual / sag);
	}
	assert(worst <= 1 + 1e-9, `弦高 ≤ 目标弦高（最大比值 ${worst.toFixed(3)}）`);
});

(async () => {
	for (const t of queue) {
		console.log(`\n▶ ${t.name}`);
		try {
			await t.fn();
		} catch (err) {
			console.error(err);
			assert(false, `${t.name}: 抛出异常`);
		}
	}
	console.log(`\n${passed} passed, ${failed} failed`);
	if (failed > 0) process.exit(1);
})();
