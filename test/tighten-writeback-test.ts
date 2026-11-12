/**
 * 拉紧回写单元测试（几何匹配 + 先删后建 + 写后校验 + 恢复），画布用 test/tighten-eda-fake.ts 仿真真机行为
 * 运行方式：npx tsx test/tighten-writeback-test.ts
 */

import {
	splitRunByWidth, TightenWriteback, SerialQueue, WbRunState,
	formatTightenReport, startWorkerSession, RepresentedSet, isRepresentedExactly, lineEndpointsUsable,
	WorkerLike, planEmit, emitError, canvasArcToRendered, canvasLineToRendered, arcFromChord, primEnds, isNetPadEntry, MIN_LINE_MIL,
	WRITER_ERR, mergeIssues, widthOrDefault, WbSummary, CopperIssue,
} from '../src/router/tighten-writeback';
import type { RenderedPrim } from '../src/router/planning-engine';
import { FakeEda } from './tighten-eda-fake';

let passed = 0;
let failed = 0;

function assert(condition: boolean, message: string): void {
	if (condition) { console.log(`  ✅ ${message}`); passed++; }
	else { console.error(`  ❌ ${message}`); failed++; }
}

async function describe(name: string, fn: () => void | Promise<void>): Promise<void> {
	console.log(`\n▶ ${name}`);
	await fn();
}

const near = (a: number, b: number, eps = 1e-6): boolean => Math.abs(a - b) < eps;
const onGridV = (v: number): boolean => Math.abs(v * 10 - Math.round(v * 10)) < 1e-9;

const L = (x1: number, y1: number, x2: number, y2: number, w?: number): RenderedPrim =>
	({ kind: 'line', net: 'N', layer: 1, x1, y1, x2, y2, __width: w } as RenderedPrim);
const line = L;

/** 画布当前几何（按读回口径转换） */
async function canvasPrims(f: FakeEda, net: string): Promise<RenderedPrim[]> {
	const out: RenderedPrim[] = [];
	for (const l of f.listLines(net)) { const p = canvasLineToRendered(l, net); if (p) out.push(p); }
	for (const a of f.listArcs(net)) { const p = canvasArcToRendered(a, net); if (p) out.push(p); }
	return out;
}

/** 简化版切段（与引擎同口径：度 ≠ 2 的顶点为边界）：从画布读出各网的 run */
async function runsFromCanvas(f: FakeEda, specs: Array<[string, number, number]>): Promise<WbRunState[]> {
	const runs: WbRunState[] = [];
	for (const [net, layer, width] of specs) {
		const ps = (await canvasPrims(f, net)).filter(p => p.layer === layer);
		const key = (q: { x: number; y: number }): string => `${Math.round(q.x * 1e4)},${Math.round(q.y * 1e4)}`;
		const ends = ps.map(p => primEnds(p).map(key) as [string, string]);
		const deg = new Map<string, number>();
		for (const [a, b] of ends) { deg.set(a, (deg.get(a) ?? 0) + 1); deg.set(b, (deg.get(b) ?? 0) + 1); }
		const used = new Set<number>();
		const walk = (node: string, first: number): RenderedPrim[] => {
			const out: RenderedPrim[] = [];
			let cur = node, i = first;
			for (;;) {
				used.add(i); out.push(ps[i]);
				cur = ends[i][0] === cur ? ends[i][1] : ends[i][0];
				if (deg.get(cur) !== 2) break;
				const nx = ps.findIndex((_, j) => !used.has(j) && (ends[j][0] === cur || ends[j][1] === cur));
				if (nx < 0) break;
				i = nx;
			}
			return out;
		};
		const chains: RenderedPrim[][] = [];
		ps.forEach((_, i) => {
			for (const n of ends[i]) if (!used.has(i) && deg.get(n) !== 2) chains.push(walk(n, i));
		});
		ps.forEach((_, i) => { if (!used.has(i)) chains.push(walk(ends[i][0], i)); });
		for (const prims of chains) {
			const len = prims.reduce((s, p) => s + (p.kind === 'line' ? Math.hypot(p.x2! - p.x1!, p.y2! - p.y1!) : Math.abs(p.r! * p.sweep!)), 0);
			runs.push({ id: runs.length, net, layer, width, prims, origLen: len });
		}
	}
	return runs;
}

type Snap = { kind: 'line' | 'arc'; net: string; sx: number; sy: number; ex: number; ey: number; deg?: number; width: number };
function snapshot(f: FakeEda): Snap[] {
	return [
		...[...f.lines.values()].map(l => ({ kind: 'line' as const, net: l.net, sx: l.sx, sy: l.sy, ex: l.ex, ey: l.ey, width: l.width })),
		...[...f.arcs.values()].map(a => ({ kind: 'arc' as const, net: a.net, sx: a.sx, sy: a.sy, ex: a.ex, ey: a.ey, deg: a.deg, width: a.width })),
	];
}
function toPrim(s: Snap): RenderedPrim {
	return s.kind === 'line' ? L(s.sx, s.sy, s.ex, s.ey) : arcFromChord(s.net, 1, s.sx, s.sy, s.ex, s.ey, s.deg!)!;
}
function sameSnapshot(a: Snap[], b: Snap[]): boolean {
	if (a.length !== b.length) return false;
	const eq = (x: Snap, y: Snap): boolean => {
		if (x.kind !== y.kind || x.net !== y.net || x.width !== y.width) return false;
		const f = (p: number, q: number): boolean => Math.abs(p - q) < 1e-9;
		const fwd = f(x.sx, y.sx) && f(x.sy, y.sy) && f(x.ex, y.ex) && f(x.ey, y.ey) && (x.kind === 'line' || f(x.deg!, y.deg!));
		const rev = f(x.sx, y.ex) && f(x.sy, y.ey) && f(x.ex, y.sx) && f(x.ey, y.sy) && (x.kind === 'line' || f(x.deg!, -y.deg!));
		return fwd || rev;
	};
	const left = [...b];
	for (const x of a) {
		const i = left.findIndex(y => eq(x, y));
		if (i < 0) return false;
		left.splice(i, 1);
	}
	return true;
}
function exactSame(c: RenderedPrim, m: RenderedPrim): boolean {
	if (c.kind !== m.kind) return false;
	const [cs, ce] = primEnds(c), [ms, me] = primEnds(m);
	const f = (p: number, q: number): boolean => Math.abs(p - q) < 1e-9;
	const fwd = f(cs.x, ms.x) && f(cs.y, ms.y) && f(ce.x, me.x) && f(ce.y, me.y);
	const rev = f(cs.x, me.x) && f(cs.y, me.y) && f(ce.x, ms.x) && f(ce.y, ms.y);
	if (c.kind === 'line') return fwd || rev;
	// 带符号比扫角：正向对上 → 扫角相同；反向对上 → 扫角相反（同端点、反向鼓起的镜像弧不算相同）
	return (fwd && f(c.sweep!, m.sweep!)) || (rev && f(c.sweep!, -m.sweep!));
}

/** 典型求解器产物：线 + 切线圆角（圆心不在网格上）+ 切线 */
function filletRun(): RenderedPrim[] {
	const r = 13.37, sw = 0.7353;
	const arc: RenderedPrim = { kind: 'arc', net: 'N', layer: 1, cx: 50, cy: r, r, startAngle: -Math.PI / 2, sweep: sw };
	const a = -Math.PI / 2 + sw;
	const e = { x: 50 + r * Math.cos(a), y: r + r * Math.sin(a) };
	return [L(0, 0, 50, 0), arc, L(e.x, e.y, e.x - 60 * Math.sin(a), e.y + 60 * Math.cos(a))];
}

async function main(): Promise<void> {
	await describe('仿真画布自检：复现真机合并/拆分/读回取整', () => {
		const f = new FakeEda();
		// 近共线 0.57° → 丢公共顶点
		f.createLine('N', 1, 0, 0, 200, 0, 8);
		f.createLine('N', 1, 200, 0, 400, 2, 8);
		const ls = f.listLines('N');
		assert(ls.length === 1 && ls[0].endX === 400 && ls[0].endY === 2 && ls[0].startX === 0, '近共线两线 → 一根 (0,0)-(400,2)，顶点被丢');
		// 链式坍缩
		const g = new FakeEda();
		for (const [a, b, c, e] of [[0, 0, 100, 0], [100, 0, 200, 1], [200, 1, 300, 3], [300, 3, 400, 6]]) g.createLine('N', 1, a, b, c, e, 8);
		const gl = g.listLines('N').map(l => `${l.startX},${l.startY}-${l.endX},${l.endY}`).sort();
		assert(gl.join('|') === '0,0-300,3|300,3-400,6', '链 (0,0)…(400,6) → (0,0)-(300,3) + (300,3)-(400,6)（与真机一致）');
		// 子段被吞、交叉拆分换 id
		const h = new FakeEda();
		const big = h.createLine('N', 1, 0, 0, 100, 0, 8);
		const sub = h.createLine('N', 1, 20, 0, 60, 0, 8);
		assert(!h.lines.has(sub) && h.lines.has(big), '子段被吞（返回的 id 不存在）');
		h.createLine('N', 1, 50, -50, 50, 50, 8);
		assert(!h.lines.has(big) && h.lines.size === 4, '同网交叉 → 两线都拆开、全换新 id');
		const k = new FakeEda();
		k.createLine('M', 1, 0, 0, 100, 0, 8);
		k.createLine('N', 1, 50, -50, 50, 50, 8);
		assert(k.lines.size === 2, '异网交叉不拆');
		// 第三根线/过孔在顶点 → 不合并；线宽不同不合并
		const v = new FakeEda();
		v.addVia('N', 200, 0);
		v.createLine('N', 1, 0, 0, 200, 0, 8);
		v.createLine('N', 1, 200, 0, 400, 2, 8);
		assert(v.lines.size === 2, '顶点有过孔 → 不合并');
		const w = new FakeEda();
		w.createLine('N', 1, 0, 0, 200, 0, 8);
		w.createLine('N', 1, 200, 0, 400, 2, 10);
		assert(w.lines.size === 2, '线宽不同 → 不合并');
		// 弧：重复被吞；读回取整；扫角 0.04° 读回 0
		const a = new FakeEda();
		const a1 = a.createArc('N', 1, 0, 0, 10.04, 10.06, 90.04, 8);
		a.createArc('N', 1, 0, 0, 10.04, 10.06, 90.04, 8);
		const ar = a.listArcs('N');
		assert(ar.length === 1 && ar[0].primitiveId === a1, '重复弧：新弧被吞、旧 id 不变');
		assert(ar[0].endX === 10 && ar[0].endY === 10.1 && ar[0].arcAngle === 90, '弧读回取整到 0.1mil / 0.1°');
		a.createArc('N', 1, 100, 0, 200, 0, 0.04, 8);
		assert(a.listArcs('N').some(x => x.arcAngle === 0), '|扫角| < 0.05° 读回 0');
		let threw = 0;
		for (const fn of [() => a.createLine('N', 1, NaN, 0, 1, 1, 8), () => a.createLine('N', 1, 0, 0, 0, 0, 8),
			() => a.createArc('N', 1, 0, 0, 1, 1, 0, 8), () => a.createArc('N', 1, 0, 0, 1, 1, 360, 8), () => a.createLine('N', 0.5, 0, 0, 1, 1, 8)]) {
			try { fn(); } catch { threw++; }
		}
		assert(threw === 5, '非法输入（NaN/零长/扫角 0/360/非法层）抛异常');
		const tiny = a.createLine('N', 1, 500, 0, 500.04, 0, 8);
		assert(!a.lines.has(tiny), '0.04mil 线返回 id 但不落盘');
		assert(a.deleteLines(['nope']) === true && a.deleteArcs(['nope']) === false, '线 delete 永远 true；弧 delete 未知 id 返回 false');
	});

	await describe('读侧：扫角读回 0 的弧按直线；arcFromChord / isNetPadEntry', () => {
		const p = canvasArcToRendered({ startX: 0, startY: 0, endX: 100, endY: 0, arcAngle: 0, layer: 1 }, 'N');
		assert(!!p && p.kind === 'line' && p.x2 === 100, 'arcAngle 0 → 端点间直线（不再丢弃）');
		assert(canvasArcToRendered({ startX: 0, startY: 0, endX: 0, endY: 0, arcAngle: 0 }, 'N') === null, '端点重合 → null');
		const q = canvasArcToRendered({ startX: 0, startY: 0, endX: 10, endY: -10, arcAngle: 90, layer: 2 }, 'N')!;
		const [s, e] = primEnds(q);
		assert(q.kind === 'arc' && near(s.x, 0) && near(e.x, 10) && near(e.y, -10) && q.layer === 2, '正常弧换算端点一致');
		assert(isNetPadEntry({ type: 1, center: { x: 1, y: 2 } }) && isNetPadEntry({ type: 2 }) && isNetPadEntry({ x: 1 }), '焊盘（type 1/2/缺省）收');
		assert(!isNetPadEntry({ pcbItemPrimitiveType: 'Track', startX: 0, type: undefined }), 'Track 不是焊盘');
		assert(!isNetPadEntry({ pcbItemPrimitiveType: 'ArcTrack', center: { x: 9010, y: 9010 } }), 'ArcTrack 不是焊盘');
		assert(!isNetPadEntry({ pcbItemPrimitiveType: 'Via', type: 0, center: { x: 0, y: 0 } }), 'Via 不是焊盘');
		assert(!isNetPadEntry({ type: 0 }) && !isNetPadEntry(null), 'type 0 / null 不是焊盘');
	});

	await describe('planEmit：防合并 + 弧精度（≤0.1mil）', () => {
		// 偏差 ≤ 0.02 的近共线 → 自己先合并
		const pm = planEmit([L(0, 0, 200, 0), L(200, 0, 400, 0.01)], 'N', 1)!;
		assert(pm.calls.length === 1 && pm.calls[0].kind === 'line', '偏差 0.005mil 的浅折线 → 预合并为一根');
		// 偏差大的浅折线 → 插隔离弧
		const sh = [L(0, 0, 200, 0), L(200, 0, 400, 2)];
		const fp = planEmit(sh, 'N', 1)!;
		assert(fp.calls.length === 3 && fp.calls[1].kind === 'arc', '0.57° 浅折线 → 线-隔离弧-线');
		assert(emitError(sh, fp.model) <= 0.1, `位置误差 ≤ 0.1mil（${emitError(sh, fp.model).toFixed(4)}）`);
		const arcCall = fp.calls[1] as any;
		assert([arcCall.sx, arcCall.sy, arcCall.ex, arcCall.ey].every(onGridV) && near(arcCall.deg * 10, Math.round(arcCall.deg * 10), 1e-9) && Math.abs(arcCall.deg) >= 0.1,
			`隔离弧四个端点坐标都在网格上、扫角为 0.1° 整倍数（${arcCall.deg}°）`);		// 短线不建
		const sp = planEmit([L(0, 0, 100, 0), L(100, 0, 100.05, 0), L(100.05, 0, 100.05, 50)], 'N', 1)!;
		assert(sp.calls.every(c => Math.hypot(c.ex - c.sx, c.ey - c.sy) >= 0.1), '< 0.1mil 的线不创建');
		// 非法 / 层不一致
		assert(planEmit([L(0, 0, NaN, 0)], 'N', 1) === null, 'NaN → null');
		assert(planEmit([{ ...L(0, 0, 1, 0), layer: 2 }], 'N', 1) === null, '层不一致 → null');
		// 求解器弧（圆心不在网格上）
		const sol = filletRun();
		const ap = planEmit(sol, 'N', 1)!;
		const arcs = ap.calls.filter(c => c.kind === 'arc') as any[];
		assert(arcs.length === 1 && [arcs[0].sx, arcs[0].sy, arcs[0].ex, arcs[0].ey].every(onGridV) && near(arcs[0].deg * 10, Math.round(arcs[0].deg * 10), 1e-9),
			'求解器弧：端点吸附 0.1mil、扫角 0.1° 整倍数');
		const li = ap.calls.findIndex(c => c.kind === 'arc');
		assert(ap.calls[li - 1].ex === arcs[0].sx && ap.calls[li - 1].ey === arcs[0].sy && ap.calls[li + 1].sx === arcs[0].ex && ap.calls[li + 1].sy === arcs[0].ey,
			'相邻线端点精确落在吸附后的弧端点上');
		assert(emitError(sol, ap.model) <= 0.1, `弧链位置误差 ≤ 0.1mil（${emitError(sol, ap.model).toFixed(4)}）`);
		// 大弧
		const bigArc: RenderedPrim = { kind: 'arc', net: 'N', layer: 1, cx: 1000.123, cy: 1000.456, r: 777.77, startAngle: 0.1234, sweep: 2.5 };
		const be = primEnds(bigArc)[1];
		const chainBig = [bigArc, L(be.x, be.y, be.x - 300 * Math.sin(0.1234 + 2.5), be.y + 300 * Math.cos(0.1234 + 2.5))];
		const bp = planEmit(chainBig, 'N', 1)!;
		assert(emitError(chainBig, bp.model) <= 0.1, `大弧（r≈778）误差 ≤ 0.1mil（${emitError(chainBig, bp.model).toFixed(4)}）`);
	});

	await describe('[A] 未变部分重写不留洞（旧方案：新线被旧线吞掉，删旧时连带删掉）', async () => {
		const f = new FakeEda();
		f.seedLine('N', 1, 8, 0, 0, 100, 0);
		f.seedLine('N', 1, 8, 100, 0, 100, 100);
		const runs = await runsFromCanvas(f, [['N', 1, 8]]);
		const wb = new TightenWriteback(f.api(), runs);
		const np = [L(0, 0, 60, 0), L(60, 0, 100, 100)];
		const out = await wb.applyRun({ id: 0, prims: np, newLen: 60 + Math.hypot(40, 100) });
		assert(out.applied, '写入成功');
		const cm = await canvasPrims(f, 'N');
		assert(emitError(np, cm) <= 0.1, `画布 == 新几何（误差 ${emitError(np, cm).toFixed(4)}）`);
		assert(!cm.some(p => p.kind === 'line' && (p.x2 === 100 && p.y2 === 0 || p.x1 === 100 && p.y1 === 0 && p.y2 === 0)), '旧线 (60..100,0) 段不在画布上');
		assert(wb.summary().verified === 1, '写后校验通过 1 段');
		assert((await wb.audit()).length === 0, '结束核对无差异');
	});

	await describe('[B] 邻段 create 拆分/换 id 后，旧铜照样按几何删掉（不留新旧并存）', async () => {
		const f = new FakeEda();
		f.seedLine('N', 1, 8, 0, 0, 200, 0);                // run0
		f.seedLine('N', 1, 8, 50, 50, 250, 50);             // run1（绕远，不与 run0 相交）
		f.seedLine('N', 1, 8, 250, 50, 250, -50);
		f.seedLine('N', 1, 8, 250, -50, 150, -50);
		const runs = await runsFromCanvas(f, [['N', 1, 8]]);
		const r0 = runs.find(r => r.prims.length === 1)!, r1 = runs.find(r => r.prims.length === 3)!;
		const wb = new TightenWriteback(f.api(), runs);
		const n1 = [L(50, 50, 150, -50)];
		assert((await wb.applyRun({ id: r1.id, prims: n1, newLen: Math.hypot(100, 100) })).applied, 'run1 斜线写入（与 run0 同网交叉 → EDA 拆开 run0，id 全换）');
		assert(f.listLines('N').length === 4, '画布上 run0、run1 都被拆成两段');
		const n0 = [L(0, 0, 100, 20), L(100, 20, 200, 0)];
		const out = await wb.applyRun({ id: r0.id, prims: n0, newLen: 2 * Math.hypot(100, 20) });
		assert(out.applied, 'run0 写入成功（旧 id 早已失效，靠几何匹配）');
		const cm = await canvasPrims(f, 'N');
		const model = [...n0, ...n1];
		assert(emitError(model, cm) <= 0.1, `画布 == 两段新几何（误差 ${emitError(model, cm).toFixed(4)}）`);
		assert(!cm.some(p => p.kind === 'line' && Math.abs(p.y1!) < 1e-9 && Math.abs(p.y2!) < 1e-9 && Math.abs(p.x2! - p.x1!) > 1), '旧 (0,0)-(200,0) 的碎片全部删掉');
		assert((await wb.audit()).length === 0 && wb.summary().verified === 2, '结束核对无差异；校验 2 段');
	});

	await describe('[B2] 画布一根线承载两段模型（跨段合并）：删后把另一段补回', async () => {
		const f = new FakeEda();
		f.seedLine('N', 1, 8, 0, 0, 200, 0);
		const wb = new TightenWriteback(f.api(), [
			{ id: 0, net: 'N', layer: 1, width: 8, prims: [L(0, 0, 100, 0)], origLen: 100 },
			{ id: 1, net: 'N', layer: 1, width: 8, prims: [L(100, 0, 200, 0)], origLen: 100 },
		]);
		const n0 = [L(0, 0, 50, 30), L(50, 30, 100, 0)];
		assert((await wb.applyRun({ id: 0, prims: n0, newLen: 2 * Math.hypot(50, 30) })).applied, '写入成功');
		const cm = await canvasPrims(f, 'N');
		assert(emitError([...n0, L(100, 0, 200, 0)], cm) <= 0.1, 'run1 的 (100,0)-(200,0) 被补回，run0 的旧部分已删');
	});

	await describe('[C] 中途创建失败：恢复原几何（不多不少），中止', async () => {
		const f = new FakeEda();
		f.seedLine('N', 1, 8, 0, 0, 100, 0);
		f.seedArc('N', 1, 8, 100, 0, 110, 10, 90);
		f.seedLine('N', 1, 8, 110, 10, 110, 100);
		f.seedLine('M', 1, 8, 0, 50, 100, 50);
		const before = snapshot(f);
		const runs = await runsFromCanvas(f, [['N', 1, 8], ['M', 1, 8]]);
		const logs: Array<[string, string]> = [];
		let halts = 0;
		const wb = new TightenWriteback(f.api(), runs, (m, lv) => logs.push([m, lv]), () => { halts++; });
		f.failAt = f.creates + 2;   // 第 2 个新图元创建时抛异常
		const out = await wb.applyRun({ id: 0, prims: [L(0, 0, 50, 0), L(50, 0, 110, 60), L(110, 60, 110, 100)], newLen: 150 });
		assert(!out.applied && out.rolledBack === true, '未采纳、已回滚');
		assert(wb.halted && halts === 1, '中止且 onHalt 一次');
		const after = snapshot(f);
		assert(after.length === before.length, `图元数恢复（${before.length} → ${after.length}），无多余图元`);
		assert(sameSnapshot(before, after), '每个图元几何与写前完全一致（含弧）');
		const s = wb.summary();
		assert(s.restored === 1 && s.unrestored.length === 0 && s.failed === 1, '汇总：已恢复 1 次、无未恢复差异');
		assert(logs.some(([m]) => m.includes('已恢复原走线')), '日志写明已恢复原走线');
		const rep = formatTightenReport(s, false);
		assert(rep.stage === '已中止' && rep.level === 'error' && rep.message.includes('失败段已恢复原走线') && !rep.message.includes('拉紧完成'), '终态文案：已中止 + 已恢复');
		// 中止后：事件忽略、画布零调用
		const c0 = f.creates, r0 = f.reads, d0 = f.deletes;
		const ig = await wb.applyRun({ id: 1, prims: [L(0, 50, 100, 50.5)], newLen: 100 });
		const ig2 = await wb.applyNetRewrite({ net: 'M', prims: [L(0, 50, 100, 50.5)], newLen: 100 });
		assert(ig.ignored === true && ig2.ignored === true && f.creates === c0 && f.reads === r0 && f.deletes === d0, '中止后 tightenRun/netRewrite 被忽略、画布零调用');
		const s2 = wb.summary();
		assert(s2.ignored === 2 && s2.done === 1 && s2.pending === 1 && s2.total === 2, '汇总：忽略 2、已处理 1、未处理 1');
	});

	await describe('[C4] 恢复按原样重建：线端比读回取整的弧起点长 0.028mil（真机 LCD3），不留碎线、不少图元', async () => {
		// 画布：线 (1589.8→1593.6279138305003) + 弧（存储起点 1593.6279…，读回 1593.6）+ 线。
		// 新线终点 1593.6000000000079（取自弧起点计算值）：写新线时旧线已删、新线/新弧先建好，之后创建失败。
		// 原样建回时弧起点（读回取整 1593.6）落在线（到 1593.6279）内部 → EDA 把线拆开留下碎线（仿真画布已复现该行为）。
		const NET = 'LCD3', X0 = 1593.6279138305003;
		const board = (overlap: boolean): { f: FakeEda; st: WbRunState; np: RenderedPrim[] } => {
			const f = new FakeEda();
			f.overlapKeepsRemainder = overlap;
			f.seedLine(NET, 1, 10, 1589.8, 1075.5, X0, 1075.5);
			f.seedArc(NET, 1, 10, X0, 1075.5, 1609.5, 1080, 31.9);
			f.seedLine(NET, 1, 10, 1609.5, 1080, 1650, 1080);
			const ln = (a: number, b: number, c: number, d: number): RenderedPrim => ({ kind: 'line', net: NET, layer: 1, x1: a, y1: b, x2: c, y2: d });
			const arc = canvasArcToRendered(f.listArcs(NET)[0], NET)!;   // 读回口径：起点 1593.6
			const prims = [ln(1589.8, 1075.5, X0, 1075.5), arc, ln(1609.5, 1080, 1650, 1080)];
			const st: WbRunState = { id: 0, net: NET, layer: 1, width: 10, prims, origLen: 100 };
			const np = [ln(1589.8, 1075.5, 1593.6000000000079, 1075.5), arc, ln(1609.5, 1080, 1650, 1100)];
			return { f, st, np };
		};
		const lineKey = (f: FakeEda): string[] => f.listLines(NET).map(l => `${l.startX},${l.startY}>${l.endX},${l.endY}`).sort();
		const arcKey = (f: FakeEda): string[] => f.listArcs(NET).map(a => `${a.startX},${a.startY}>${a.endX},${a.endY}@${a.arcAngle}`).sort();
		for (const overlap of [false, true]) {
			for (const k of [1, 2, 3]) {
				const tag = `${overlap ? '真机口径(重叠留碎线)' : '合成一根口径'}，第 ${k} 次创建失败`;
				const { f, st, np } = board(overlap);
				const beforeL = lineKey(f), beforeA = arcKey(f);
				const api = f.api();
				const shortCreates: number[] = [];
				const cl = api.createLine;
				api.createLine = async (n, ly, sx, sy, ex, ey, w) => { const len = Math.hypot(ex - sx, ey - sy); if (len < MIN_LINE_MIL) shortCreates.push(len); return cl(n, ly, sx, sy, ex, ey, w); };
				const logs: Array<[string, string]> = [];
				const wb = new TightenWriteback(api, [{ ...st, prims: [...st.prims] }], (m, lv) => logs.push([m, lv]));
				f.failAt = f.creates + k;
				const out = await wb.applyRun({ id: 0, prims: np, newLen: 90 });
				f.failAt = undefined;
				assert(!out.applied && out.rolledBack === true && wb.summary().restored === 1 && wb.summary().unrestored.length === 0, `${tag}：已回滚、已恢复`);
				assert(f.lines.size === 2 && f.arcs.size === 1, `${tag}：图元数不变（线 ${f.lines.size}、弧 ${f.arcs.size}，应为 2 + 1）`);
				assert(JSON.stringify(lineKey(f)) === JSON.stringify(beforeL), `${tag}：线端点与写前完全相同（无碎线）：${lineKey(f).join(' | ')}`);
				assert(JSON.stringify(arcKey(f)) === JSON.stringify(beforeA), `${tag}：弧读回与写前相同`);
				assert(shortCreates.length === 0, `${tag}：从未创建短于 MIN_LINE_MIL 的线`);
				// 同样的回写不带故障再来一次：成功并通过写后校验、结束核对无差异
				const wb2 = new TightenWriteback(api, [{ ...st, prims: [...st.prims] }]);
				const out2 = await wb2.applyRun({ id: 0, prims: np, newLen: 90 });
				assert(out2.applied && wb2.summary().verified === 1 && (await wb2.audit()).length === 0, `${tag}：之后无故障的同样回写成功并校验通过`);
			}
		}
	});

	await describe('[C5] 结束核对读画布失败：报该网缺铜 + 告警，不当作没问题', async () => {
		const f = new FakeEda();
		f.seedLine('N', 1, 8, 0, 0, 100, 0);
		f.seedLine('M', 1, 8, 0, 300, 100, 300);
		const runs = await runsFromCanvas(f, [['N', 1, 8], ['M', 1, 8]]);
		const logs: Array<[string, string]> = [];
		const api = f.api();
		let failN = false;
		const ll = api.listLines;
		api.listLines = async (n) => { if (failN && n === 'N') throw new Error('read failure'); return ll(n); };
		const wb = new TightenWriteback(api, runs, (m, lv) => logs.push([m, lv]));
		assert((await wb.applyRun({ id: 0, prims: [L(0, 0, 50, 10), L(50, 10, 100, 0)], newLen: 102 })).applied, '写入 N 成功');
		assert((await wb.applyRun({ id: 1, prims: [L(0, 300, 50, 310), L(50, 310, 100, 300)], newLen: 102 })).applied, '写入 M 成功');
		failN = true;   // 只有 N 的终检读失败
		const au = await wb.audit();
		assert(au.length === 1 && au[0].net === 'N' && au[0].kind === 'missing' && Number.isFinite(au[0].x) && Number.isFinite(au[0].y), `报 N 的缺铜（带坐标）：${JSON.stringify(au)}`);
		assert(logs.some(([m, lv]) => lv === 'warn' && m.includes('网络 N') && m.includes('无法重新读取画布')), 'warn 日志写明该网无法重读');
		assert(au.every(i => i.net !== 'M'), 'M 读成功、无差异，不受影响');
		const rep = formatTightenReport(wb.summary(), false);
		assert(rep.level === 'warn' && rep.message.includes('网络 N') && rep.message.includes('画布与模型不一致'), '终态文案：warn + 点名该网');
	});

	await describe('[C2] 恢复也失败：明确报告网络与坐标', async () => {
		const f = new FakeEda();
		f.seedLine('N', 1, 8, 0, 0, 100, 0);
		const runs = await runsFromCanvas(f, [['N', 1, 8]]);
		const logs: Array<[string, string]> = [];
		const wb = new TightenWriteback(f.api(), runs, (m, lv) => logs.push([m, lv]));
		f.failFrom = f.creates + 1;   // 之后所有创建都失败（含恢复）
		await wb.applyRun({ id: 0, prims: [L(0, 0, 50, 10), L(50, 10, 100, 0)], newLen: 102 });
		const s = wb.summary();
		assert(s.halted && s.unrestored.length > 0 && s.unrestored.every(i => i.net === 'N' && Number.isFinite(i.x)), '未恢复差异带网络与坐标');
		assert(logs.some(([m, lv]) => lv === 'error' && m.includes('未能恢复') && m.includes('缺铜@(')), 'error 级日志含「未能恢复」与坐标');
		const rep = formatTightenReport(s, false);
		assert(rep.message.includes('未能恢复原走线') && rep.message.includes('网络 N'), '终态文案写明未能恢复及位置');
	});

	await describe('[C3] 写后校验不符（EDA 吞掉/改形）→ 视为写失败并恢复', async () => {
		const f = new FakeEda();
		f.seedLine('N', 1, 8, 0, 0, 100, 0);
		const runs = await runsFromCanvas(f, [['N', 1, 8]]);
		const api = f.api();
		// 模拟：create 返回 id 但图元没落盘
		let n = 0;
		const orig = api.createLine;
		api.createLine = async (...a) => (n++ === 1 ? 'ghost' : orig(...a));
		const wb = new TightenWriteback(api, runs);
		const out = await wb.applyRun({ id: 0, prims: [L(0, 0, 50, 10), L(50, 10, 100, 0)], newLen: 102 });
		assert(!out.applied && wb.halted && /校验不符/.test(String(out.reason)), '校验发现缺铜 → 写失败');
		const cm = await canvasPrims(f, 'N');
		assert(emitError([L(0, 0, 100, 0)], cm) <= 1e-9 && wb.summary().restored === 1, '原走线已恢复');
	});

	await describe('[D] 整网重写：同一机制覆盖该网全部段', async () => {
		const f = new FakeEda();
		f.seedLine('N', 1, 8, 0, 0, 100, 0);
		f.seedLine('N', 1, 8, 100, 0, 100, 100);
		f.seedLine('N', 1, 8, 100, 100, 200, 100);
		f.seedLine('N', 1, 8, 100, 0, 200, 0);
		f.seedLine('N', 1, 8, 200, 0, 200, 50);
		const runs = await runsFromCanvas(f, [['N', 1, 8]]);
		const wb = new TightenWriteback(f.api(), runs);
		const np = [L(0, 0, 200, 0), L(100, 0, 200, 100), L(200, 0, 200, 50)];
		const out = await wb.applyNetRewrite({ net: 'N', prims: np, oldLen: 550, newLen: 341 });
		assert(out.applied, '整网重写成功');
		const cm = await canvasPrims(f, 'N');
		assert(emitError(np, cm) <= 0.1, `画布 == 新树（误差 ${emitError(np, cm).toFixed(4)}；T 接处被 EDA 拆开也不影响）`);
		const s = wb.summary();
		assert(s.adopted === runs.length && s.adopted <= s.total && s.verified === runs.length, `该网 ${runs.length} 段全部采纳并校验`);
		const late = await wb.applyRun({ id: 0, prims: [L(0, 0, 1, 1)], newLen: 1 });
		assert(!late.applied, '整网重写后该网的 tightenRun 被忽略');
		// 线宽不一致 → 跳过，不碰画布
		const g = new FakeEda();
		const wb2 = new TightenWriteback(g.api(), [
			{ id: 0, net: 'N', layer: 1, width: 8, prims: [L(0, 0, 10, 0)], origLen: 10 },
			{ id: 1, net: 'N', layer: 1, width: 12, prims: [L(10, 0, 20, 0)], origLen: 10 },
		]);
		const o2 = await wb2.applyNetRewrite({ net: 'N', prims: [L(0, 0, 5, 0)], newLen: 5 });
		assert(!o2.applied && g.reads === 0 && g.creates === 0, '线宽不一致 → 跳过，不碰画布');
		// 整网重写创建失败 → 恢复全部段
		const h = new FakeEda();
		h.seedLine('N', 1, 8, 0, 0, 100, 0);
		h.seedLine('N', 1, 8, 100, 0, 100, 100);
		h.seedLine('N', 1, 8, 100, 0, 200, 0);
		const before = snapshot(h);
		const wb3 = new TightenWriteback(h.api(), await runsFromCanvas(h, [['N', 1, 8]]));
		h.failAt = 2;
		const o3 = await wb3.applyNetRewrite({ net: 'N', prims: [L(0, 0, 200, 0), L(100, 0, 150, 100)], newLen: 312 });
		assert(!o3.applied && wb3.summary().failed === 3 && wb3.adopted === 0, '整网重写失败：3 段计失败、不计采纳');
		assert(emitError(before.map(toPrim), snapshot(h).map(toPrim)) <= 1e-9, '整网原几何恢复');
	});

	await describe('[E] 浅折线（< 1°）写出后几何保持在 0.1mil 内', async () => {
		for (const pts of [[[0, 0], [200, 0], [400, 2]], [[0, 0], [100, 0], [200, 1], [300, 3], [400, 6]]]) {
			const f = new FakeEda();
			f.seedLine('N', 1, 8, 0, 0, 0, 100);
			const runs = await runsFromCanvas(f, [['N', 1, 8]]);
			const wb = new TightenWriteback(f.api(), runs);
			const np: RenderedPrim[] = [];
			for (let i = 0; i + 1 < pts.length; i++) np.push(L(pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1]));
			const out = await wb.applyRun({ id: 0, prims: np, newLen: 400 });
			const cm = await canvasPrims(f, 'N');
			const err = emitError(np, cm);
			assert(out.applied && err <= 0.1, `${pts.length - 1} 段浅折线：写入成功，画布误差 ${err.toFixed(4)} ≤ 0.1mil`);
			// 对照：不做防合并直接写，EDA 会丢顶点
			const raw = new FakeEda();
			for (const p of np) raw.createLine('N', 1, p.x1!, p.y1!, p.x2!, p.y2!, 8);
			const rawErr = emitError(np, await canvasPrims(raw, 'N'));
			assert(rawErr > 0.5, `对照：直接写则被 EDA 合并变形 ${rawErr.toFixed(2)}mil`);
		}
	});

	await describe('[F] 弧往返：读回 == 写入；第二遍拉紧读到同样几何；重复写入是几何 no-op', async () => {
		const f = new FakeEda();
		f.seedLine('N', 1, 8, 0, 0, 0, 100);
		const runs = await runsFromCanvas(f, [['N', 1, 8]]);
		const wb = new TightenWriteback(f.api(), runs);
		const sol = filletRun();
		assert((await wb.applyRun({ id: 0, prims: sol, newLen: 100 })).applied, '写入成功');
		const model = wb.primsOf(0);
		const cm = await canvasPrims(f, 'N');
		assert(cm.length === model.length && model.every(m => cm.some(c => exactSame(c, m))), '画布读回的每个图元与写入模型精确相同（弧端点/扫角无取整误差）');
		assert(emitError(sol, cm) <= 0.1, `相对求解器 ≤ 0.1mil（${emitError(sol, cm).toFixed(4)}）`);
		// 再写一次同样的几何：不碰画布
		const c0 = f.creates, d0 = f.deletes;
		const again = await wb.applyRun({ id: 0, prims: sol, newLen: 100 });
		assert(again.applied && f.creates === c0 && f.deletes === d0, '同样几何再写：零创建/零删除');
		// 第二遍拉紧：从画布重新读出 run，原样写回 = no-op
		const runs2 = await runsFromCanvas(f, [['N', 1, 8]]);
		const wb2 = new TightenWriteback(f.api(), runs2);
		const c1 = f.creates;
		await wb2.applyRun({ id: 0, prims: runs2[0].prims, newLen: 100 });
		assert(f.creates === c1 && emitError(cm, await canvasPrims(f, 'N')) <= 1e-9, '第二遍读到同样几何，写回不变');
	});

	await describe('[G] 汇总口径：adopted ≤ total、savedMil、校验数、核对差异', async () => {
		const f = new FakeEda();
		f.seedLine('N', 1, 8, 0, 0, 100, 0);
		f.seedLine('N', 1, 8, 100, 0, 100, 100);
		f.seedLine('N', 1, 8, 100, 0, 200, 0);
		f.seedLine('M', 1, 8, 0, 300, 100, 300);
		f.seedLine('M', 1, 8, 100, 300, 100, 400);
		const runs = await runsFromCanvas(f, [['N', 1, 8], ['M', 1, 8]]);
		const wb = new TightenWriteback(f.api(), runs);
		const mId = runs.find(r => r.net === 'M')!.id;
		await wb.applyRun({ id: mId, prims: [L(0, 300, 100, 400)], newLen: 141.4 });
		await wb.applyRun({ id: mId, prims: [L(0, 300, 50, 360), L(50, 360, 100, 400)], newLen: 142 });
		await wb.applyRun({ id: 0, prims: null, newLen: 100 });
		const s = wb.summary();
		assert(s.adopted === 1 && s.total === 4 && s.done === 2, '同一段改写两次只计 1 次采纳；null 只计已处理');
		assert(Math.abs(s.savedMil - 58) < 1e-9, 'savedMil 用最新长度：200-142=58');
		assert(s.verified === 1, '校验 1 段');
		await wb.applyNetRewrite({ net: 'N', prims: [L(0, 0, 200, 0), L(100, 0, 100, 100)], newLen: 300 });
		const s2 = wb.summary();
		assert(s2.adopted === 4 && s2.adopted <= s2.total, '整网重写 N（3 段）+ M 1 段 = 4/4');
		// 外部改动：有人在 M 上多画了一根 → 结束核对报多余铜
		f.seedLine('M', 1, 8, 0, 300, 0, 350);
		const au = await wb.audit();
		assert(au.length === 1 && au[0].kind === 'extra' && au[0].net === 'M', '结束核对：报 M 多余铜（带坐标）');
		const rep = formatTightenReport(wb.summary(), false);
		assert(rep.stage === '完成' && rep.level === 'warn' && rep.message.includes('4/4 段有改善') && rep.message.includes('多余铜@('), '完成文案带差异与坐标、warn 级');
		assert(formatTightenReport(wb.summary(), true).stage === '已停止', '取消 → 已停止');
		const ok = new TightenWriteback(new FakeEda().api(), [{ id: 0, net: 'K', layer: 1, width: 8, prims: [L(0, 0, 1, 0)], origLen: 1 }]);
		await ok.audit();
		const okRep = formatTightenReport(ok.summary(), false);
		assert(okRep.level === 'success' && okRep.message.includes('0/1 段有改善'), '无差异 → success');
	});

	await describe('[H] 不认识的铜绝不删：只部分落在旧路径上的外来线保留', async () => {
		const f = new FakeEda();
		f.seedLine('N', 1, 8, 0, 0, 100, 0);
		const runs = await runsFromCanvas(f, [['N', 1, 8]]);
		f.seedLine('N', 1, 8, 50, 0, 50, 80);   // 模型外（读完之后用户画的）——T 在旧路径中点
		const wb = new TightenWriteback(f.api(), runs);
		await wb.applyRun({ id: 0, prims: [L(0, 0, 50, -20), L(50, -20, 100, 0)], newLen: 108 });
		assert([...f.lines.values()].some(l => l.sx === 50 && l.ey === 80), '外来线仍在画布上');
		assert(wb.summary().adopted === 1, '本段照常写入（外来铜不算「旧铜残留」）');
		const au = await wb.audit();
		assert(au.some(i => i.kind === 'extra' && Math.abs(i.x - 50) < 1e-6), '结束核对把模型外的铜报为多余铜（带坐标）');
	});

	await describe('[I] 误差预算：写出几何偏离求解结果 > WRITER_ERR → 不写、不算失败、不中止', async () => {
		// 大半径单弧（链上只有它：终点固定、前面没有线可滑）：扫角量化到 0.1° 带来的弦高误差 ≈ 弦长·Δθ/8 ≫ 0.1mil
		const R = 100000, sw = 2.949 * Math.PI / 180;   // 量化到 2.9°：Δ = 0.049°，弦长 ≈ 5147mil → 弦高误差 ≈ 0.55mil
		const bulky: RenderedPrim = { kind: 'arc', net: 'N', layer: 1, cx: 1000, cy: 1000 + R, r: R, startAngle: -Math.PI / 2 - sw / 2, sweep: sw };
		const plan = planEmit([bulky], 'N', 1)!;
		const err = emitError([bulky], plan.model);
		assert(err > WRITER_ERR, `构造的弧量化后超出预算（${err.toFixed(3)}mil > ${WRITER_ERR}）`);
		const [s, e] = primEnds(bulky);
		const f = new FakeEda();
		f.seedLine('N', 1, 8, s.x, s.y, e.x, e.y);
		f.seedLine('M', 1, 8, 0, 3000, 100, 3000);
		const runs = await runsFromCanvas(f, [['N', 1, 8], ['M', 1, 8]]);
		const before = snapshot(f);
		const logs: Array<[string, string]> = [];
		let halts = 0;
		const wb = new TightenWriteback(f.api(), runs, (m, lv) => logs.push([m, lv]), () => { halts++; });
		const c0 = f.creates, d0 = f.deletes;
		const out = await wb.applyRun({ id: 0, prims: [bulky], newLen: R * sw });
		assert(!out.applied && out.overBudget === true && !out.rolledBack, '未采纳、标记超预算、不是回滚');
		assert(f.creates === c0 && f.deletes === d0 && sameSnapshot(before, snapshot(f)), '画布零创建/零删除，原样不动');
		const sm = wb.summary();
		assert(!wb.halted && halts === 0 && sm.failed === 0 && sm.rollbacks === 0 && sm.adopted === 0 && sm.overBudget === 1 && sm.done === 1,
			'不算失败、不中止；已处理 1 段、超预算 1 次');
		assert(logs.some(([m, lv]) => lv === 'warn' && m.includes('超出回写误差预算')), 'warn 日志写明超预算');
		const mId = runs.find(r => r.net === 'M')!.id;
		assert((await wb.applyRun({ id: mId, prims: [L(0, 3000, 50, 3010), L(50, 3010, 100, 3000)], newLen: 102 })).applied, '之后其它段照常写入');
		assert(formatTightenReport(wb.summary(), false).message.includes('1 次结果超出回写误差预算未写入'), '终态文案写明超预算未写入');
		// 对照：正常求解器弧链不受影响
		assert(emitError(filletRun(), planEmit(filletRun(), 'N', 1)!.model) <= WRITER_ERR, '正常圆角链在预算内');
	});

	await describe('[J] 连续覆盖：长线中间缺一小段（采样点之间）也能查出', async () => {
		// 2000mil 的线：旧实现最多 400 个采样点（间距 5mil），采样点 1000 / 1005 都有铜，1002~1003.5 的洞查不出来
		// (a) 写后校验：EDA「吞掉」了新长线中间 1.5mil
		{
			const f = new FakeEda();
			f.seedLine('N', 1, 8, 0, 0, 1000, 10);
			f.seedLine('N', 1, 8, 1000, 10, 2000, 0);
			const runs = await runsFromCanvas(f, [['N', 1, 8]]);
			const r = runs[0];
			const api = f.api();
			const cl = api.createLine;
			api.createLine = async (n, ly, sx, sy, ex, ey, w) => {
				if (Math.hypot(ex - sx, ey - sy) > 1500 && sy === ey) {
					const lo = Math.min(sx, ex), hi = Math.max(sx, ex);
					await cl(n, ly, lo, sy, 1002, sy, w);
					return cl(n, ly, 1003.5, sy, hi, sy, w);
				}
				return cl(n, ly, sx, sy, ex, ey, w);
			};
			const wb = new TightenWriteback(api, runs);
			const out = await wb.applyRun({ id: r.id, prims: [L(0, 0, 2000, 0)], newLen: 2000 });
			assert(!out.applied && out.rolledBack === true && /校验不符/.test(String(out.reason)), `写后校验查出 1.5mil 的洞 → 写失败：${out.reason}`);
			assert(/缺铜@\(100[23]\./.test(String(out.reason)), '报告的缺铜坐标就在洞里');
		}
		// (b) 结束核对：模型里的长线在画布上缺一小段（读之后被改），audit 报缺铜
		{
			const f = new FakeEda();
			f.seedLine('N', 1, 8, 0, 0, 100, 0);
			const wb = new TightenWriteback(f.api(), [
				{ id: 0, net: 'N', layer: 1, width: 8, prims: [L(0, 0, 100, 0)], origLen: 100 },
				{ id: 1, net: 'N', layer: 1, width: 8, prims: [L(0, 500, 2000, 500)], origLen: 2000 },
			]);
			f.seedLine('N', 1, 8, 0, 500, 1002, 500);
			f.seedLine('N', 1, 8, 1003.5, 500, 2000, 500);
			assert((await wb.applyRun({ id: 0, prims: [L(0, 0, 50, 10), L(50, 10, 100, 0)], newLen: 102 })).applied, '另一段写入成功（洞不是这次写造成的）');
			const au = await wb.audit();
			assert(au.some(i => i.kind === 'missing' && i.y === 500 && i.x > 1002 && i.x < 1003.5), `结束核对报出洞的位置：${JSON.stringify(au)}`);
		}
		// (c) 多余铜同样连续检查：长画布线中间有一段偏出模型（采样点之间）
		{
			const f = new FakeEda();
			f.seedLine('N', 1, 8, 0, 0, 100, 0);
			const wb = new TightenWriteback(f.api(), [
				{ id: 0, net: 'N', layer: 1, width: 8, prims: [L(0, 0, 100, 0)], origLen: 100 },
				{ id: 1, net: 'N', layer: 1, width: 8, prims: [L(0, 500, 2000, 500)], origLen: 2000 },
			]);
			f.seedLine('N', 1, 8, 0, 500, 2000, 500);
			f.seedLine('N', 1, 8, 1002, 500, 1002.5, 501.5);   // 0.5mil 长、偏出 1.5mil 的毛刺
			f.seedLine('N', 1, 8, 1002.5, 501.5, 1003, 500);
			await wb.applyRun({ id: 0, prims: [L(0, 0, 50, 10), L(50, 10, 100, 0)], newLen: 102 });
			const au = await wb.audit();
			assert(au.filter(i => i.kind === 'extra').length === 2, `毛刺报为多余铜：${JSON.stringify(au)}`);
		}
	});

	await describe('[K] 写前就不一致的其它模型铜：不算本次失败、不补建、结束核对如实上报', async () => {
		// (a) 线宽口径统一：画布 lineWidth 读成 0，模型按默认线宽 10（与读侧 trackWidth 同口径）
		assert(widthOrDefault(0, 10) === 10 && widthOrDefault(undefined, 10) === 10 && widthOrDefault('8', 10) === 8 && widthOrDefault(NaN, 10) === 10, 'widthOrDefault：0/缺失/NaN → 默认线宽');
		{
			const f = new FakeEda();
			f.seedLine('N', 1, 10, 0, 0, 100, 0);
			f.seedLine('N', 1, 0, 0, 200, 100, 200);   // lineWidth 读成 0
			const runs: WbRunState[] = [
				{ id: 0, net: 'N', layer: 1, width: 10, prims: [L(0, 0, 100, 0)], origLen: 100 },
				{ id: 1, net: 'N', layer: 1, width: widthOrDefault(0, 10), prims: [L(0, 200, 100, 200)], origLen: 100 },
			];
			const wb = new TightenWriteback(f.api(), runs, undefined, undefined, [], 10);
			const out = await wb.applyRun({ id: 0, prims: [L(0, 0, 50, 10), L(50, 10, 100, 0)], newLen: 102 });
			assert(out.applied && wb.summary().verified === 1, '同一口径：线宽 0 的另一段不影响本段写入与校验');
			assert((await wb.audit()).length === 0 && f.lines.size === 3, '结束核对无差异，没有多建图元');
		}
		// (b) 不传默认线宽（旧口径 0 vs 10 不一致）：照样写入成功，不补建，结束核对如实报
		{
			const f = new FakeEda();
			f.seedLine('N', 1, 10, 0, 0, 100, 0);
			f.seedLine('N', 1, 0, 0, 200, 100, 200);
			const logs: Array<[string, string]> = [];
			const wb = new TightenWriteback(f.api(), [
				{ id: 0, net: 'N', layer: 1, width: 10, prims: [L(0, 0, 100, 0)], origLen: 100 },
				{ id: 1, net: 'N', layer: 1, width: 10, prims: [L(0, 200, 100, 200)], origLen: 100 },
			], (m, lv) => logs.push([m, lv]));
			const out = await wb.applyRun({ id: 0, prims: [L(0, 0, 50, 10), L(50, 10, 100, 0)], newLen: 102 });
			assert(out.applied && !wb.halted, '写前就线宽不一致的另一段：不算本次失败、不中止');
			assert(![...f.lines.values()].some(l => l.sy === 200 && l.width === 10), '没有按模型补建一根 10mil 的副本');
			assert(logs.some(([m, lv]) => lv === 'warn' && m.includes('写前就与画布不一致')), 'warn 日志写明写前不一致');
			const au = await wb.audit();
			assert(au.length === 1 && au[0].kind === 'missing' && au[0].y === 200, `结束核对如实报该处（线宽不一致 = 缺铜）：${JSON.stringify(au)}`);
		}
		// (c) 模型里的另一段写前就不在画布上（读之后被删）：写入成功、绝不凭空建出来；失败恢复时也不建
		for (const fault of [false, true]) {
			const f = new FakeEda();
			f.seedLine('N', 1, 8, 0, 0, 100, 0);
			const wb = new TightenWriteback(f.api(), [
				{ id: 0, net: 'N', layer: 1, width: 8, prims: [L(0, 0, 100, 0)], origLen: 100 },
				{ id: 1, net: 'N', layer: 1, width: 8, prims: [L(0, 300, 100, 300)], origLen: 100 },
			], undefined, undefined, [{ net: 'N', layer: 1, width: 8, prim: L(200, 0, 200, 100) }]);
			if (fault) f.failAt = f.creates + 2;
			const out = await wb.applyRun({ id: 0, prims: [L(0, 0, 50, 10), L(50, 10, 100, 0)], newLen: 102 });
			const tag = fault ? '创建失败 → 恢复' : '正常写入';
			if (fault) {
				const s = wb.summary();
				assert(!out.applied && s.restored === 1 && s.unrestored.length === 0, `${tag}：原走线已恢复（写前缺的那段不计入恢复校验）`);
			} else {
				assert(out.applied && wb.summary().verified === 1, `${tag}：写入并校验通过`);
			}
			assert(![...f.lines.values()].some(l => l.sy === 300 || l.sx === 200), `${tag}：写前缺的 run / 静态铜没有被凭空建出来`);
			const au = await wb.audit();
			assert(au.filter(i => i.kind === 'missing').length === 2 && au.some(i => i.y === 300) && au.some(i => i.x === 200), `${tag}：结束核对如实报两处缺铜`);
		}
	});

	await describe('[L] 恢复按带符号扫角认领：同端点反向鼓起的弧不当成原弧', async () => {
		const f = new FakeEda();
		f.seedArc('N', 1, 8, 0, 0, 10, 10, 90);
		f.seedLine('N', 1, 8, 10, 10, 10, 100);
		const before = snapshot(f);
		const runs = await runsFromCanvas(f, [['N', 1, 8]]);
		const wb = new TightenWriteback(f.api(), runs);
		const mirror = arcFromChord('N', 1, 0, 0, 10, 10, -90)!;
		f.failAt = f.creates + 2;   // 镜像弧建好后，下一根线创建失败
		const out = await wb.applyRun({ id: 0, prims: [mirror, L(10, 10, 20, 100)], newLen: 100 });
		const s = wb.summary();
		assert(!out.applied && s.restored === 1 && s.unrestored.length === 0, '已恢复原走线');
		assert(sameSnapshot(before, snapshot(f)), `画布与写前一致（原弧 +90°，没有留下镜像弧）：${JSON.stringify(snapshot(f))}`);
		// exactSame（测试辅助）同样按带符号扫角比
		const a = arcFromChord('N', 1, 0, 0, 10, 10, 90)!;
		assert(!exactSame(a, mirror) && exactSame(a, a), 'exactSame：镜像弧不相等、自身相等');
		const rv: RenderedPrim = { ...a, startAngle: a.startAngle! + a.sweep!, sweep: -a.sweep! };
		assert(exactSame(a, rv), 'exactSame：同一弧反向表示仍相等');
	});

	await describe('[M] 汇报去重：未恢复差异与结束核对同一处只计一次；中止时其它网的核对差异照常显示', () => {
		const u: CopperIssue[] = [{ net: 'N', layer: 1, kind: 'missing', x: 10, y: 0 }];
		const au: CopperIssue[] = [
			{ net: 'N', layer: 1, kind: 'missing', x: 10.4, y: 0 },
			{ net: 'M', layer: 1, kind: 'extra', x: 500, y: 500 },
		];
		const merged = mergeIssues(u, au);
		assert(merged.length === 2 && merged.some(i => i.net === 'M'), '合并后 2 处（同一处去重）');
		const sum: WbSummary = {
			total: 5, done: 2, adopted: 1, failed: 1, rollbacks: 1, verified: 1, restored: 0, unrestored: u, audit: au,
			auditedNets: 2, ignored: 0, overBudget: 0, pending: 3, halted: true, haltReason: 'x', savedMil: 3,
		};
		const rep = formatTightenReport(sum, false);
		assert(rep.stage === '已中止' && rep.message.includes('网络 M 层1 多余铜@(500.0, 500.0)'), '中止文案仍列出其它网的核对差异');
		assert((rep.message.match(/网络 N 层1 缺铜/g) ?? []).length === 1, '同一处缺铜只出现一次');
	});

	await describe('仿真画布：弧端点拆线（seedArc / createArc 一致）、重叠留碎线开关', () => {
		for (const how of ['seed', 'create'] as const) {
			const f = new FakeEda();
			f.seedLine('N', 1, 8, 0, 0, 100, 0);
			if (how === 'seed') f.seedArc('N', 1, 8, 50, 0, 60, 10, 90);
			else f.createArc('N', 1, 50, 0, 60, 10, 90, 8);
			const ls = f.listLines('N').map(l => `${l.startX}-${l.endX}`).sort();
			assert(ls.join('|') === '0-50|50-100', `${how}Arc：弧端点落在线内部 → 线在该点拆开`);
		}
		const g = new FakeEda();
		g.overlapKeepsRemainder = true;
		const keep = g.createLine('N', 1, 0, 0, 100, 0, 8);
		g.createLine('N', 1, 80, 0, 150, 0, 8);
		const gl = g.listLines('N').map(l => `${l.startX}-${l.endX}`).sort();
		assert(g.lines.has(keep) && gl.join('|') === '0-100|100-150', `overlapKeepsRemainder：已有线保留、新线只补剩余部分（${gl.join('|')}）`);
		const h = new FakeEda();
		h.createLine('N', 1, 0, 0, 100, 0, 8);
		h.createLine('N', 1, 80, 0, 150, 0, 8);
		assert(h.listLines('N').length === 1, '默认（探测记录）：部分重叠合成一根');
	});

	await describe('splitRunByWidth', () => {
		const widthOf = (p: RenderedPrim): number => Number((p as any).__width) || 8;
		const same = splitRunByWidth({ prims: [line(0, 0, 10, 0, 8), line(10, 0, 20, 0, 8)], a: { x: 0, y: 0 }, b: { x: 20, y: 0 }, layer: 1 }, widthOf);
		assert(same.length === 1 && same[0].width === 8 && same[0].prims.length === 2, '线宽一致 → 单段');

		const prims = [line(0, 0, 10, 0, 8), line(10, 0, 20, 0, 12), line(20, 0, 30, 0, 12), line(30, 0, 40, 0, 8)];
		const parts = splitRunByWidth({ prims, a: { x: 0, y: 0 }, b: { x: 40, y: 0 }, layer: 1 }, widthOf);
		assert(parts.length === 3, '8/12/8 → 3 段');
		assert(parts.map(p => p.width).join() === '8,12,8', '各段线宽保留');
		assert(parts.map(p => p.prims.length).join() === '1,2,1', '图元数 1,2,1');
		assert(parts[0].a.x === 0 && parts[0].b.x === 10, '首段端点 0→10');
		assert(parts[1].a.x === 10 && parts[1].b.x === 30, '中段端点 10→30');
		assert(parts[2].a.x === 30 && parts[2].b.x === 40, '末段端点 30→40（沿用原 b）');

		// 图元方向反着存（从远端到近端）：端点仍沿 a→b 行进方向
		const rev = splitRunByWidth({
			prims: [line(10, 0, 0, 0, 8), line(20, 0, 10, 0, 12)], a: { x: 0, y: 0 }, b: { x: 20, y: 0 }, layer: 1,
		}, widthOf);
		assert(rev.length === 2 && rev[0].b.x === 10 && rev[1].a.x === 10, '反向存储的图元也按行进方向取端点');
		assert(splitRunByWidth({ prims: [], a: { x: 0, y: 0 }, b: { x: 0, y: 0 }, layer: 1 }, widthOf).length === 0, '空段 → 空');
	});

	await describe('[3] Worker 出错：等写入队列排空后才 reject', async () => {
		const sleep = (ms: number) => new Promise<void>(r => setTimeout(r, ms));
		const mk = () => {
			const w: WorkerLike & { terminated: number; posted: unknown[] } = {
				onmessage: null, onerror: null, terminated: 0, posted: [],
				postMessage(m) { this.posted.push(m); }, terminate() { this.terminated++; },
			};
			return w;
		};
		for (const via of ['message', 'onerror'] as const) {
			const w = mk();
			const q = new SerialQueue();
			let writing = false;
			const events: string[] = [];
			const ses = startWorkerSession(w, q, { type: 'tighten' }, (m) => {
				events.push(m.type);
				void q.push(async () => { writing = true; await sleep(40); writing = false; });
			});
			assert(w.posted.length === 1, `${via}：任务已 post`);
			w.onmessage!({ data: { type: 'tightenRun', id: 0 } });
			await sleep(5);   // 队列正在写
			let writingAtReject: boolean | null = null;
			const p = ses.done.then(() => 'resolved', () => { writingAtReject = writing; return 'rejected'; });
			if (via === 'message') w.onmessage!({ data: { type: 'error', message: 'oops' } });
			else w.onerror!({ message: 'oops' });
			assert(w.terminated >= 1, `${via}：立即终止 Worker`);
			// 出错之后迟到的消息不再入队
			w.onmessage!({ data: { type: 'tightenRun', id: 1 } });
			const r = await p;
			assert(r === 'rejected', `${via}：最终 reject`);
			assert(writingAtReject === false, `${via}：reject 时队列已排空（不在写画布）`);
			assert(events.join() === 'tightenRun', `${via}：出错后到达的消息被忽略`);
		}

		// 正常完成：tightenDone 排在已到达消息之后
		const w = mk();
		const q = new SerialQueue();
		const order: string[] = [];
		const ses = startWorkerSession(w, q, {}, (m) => {
			if (m.type === 'tightenRun') void q.push(async () => { await sleep(20); order.push('write'); });
		});
		w.onmessage!({ data: { type: 'tightenRun', id: 0 } });
		w.onmessage!({ data: { type: 'tightenDone' } });
		await ses.done;
		order.push('done');
		assert(order.join() === 'write,done', '完成：写入全部落盘后才 resolve');
		assert(w.terminated >= 1, '完成后终止 Worker');

		// halt：终止 Worker、忽略后续消息、等队列排空后 resolve
		const w2 = mk();
		const q2 = new SerialQueue();
		const seen: string[] = [];
		const ses2 = startWorkerSession(w2, q2, {}, (m) => { seen.push(m.type); });
		ses2.halt();
		w2.onmessage!({ data: { type: 'tightenRun', id: 5 } });
		await ses2.done;
		assert(w2.terminated >= 1 && seen.length === 0, 'halt：终止 Worker，之后的消息被忽略');
		// cancel
		const w3 = mk();
		const q3 = new SerialQueue();
		let slow = true;
		void q3.push(async () => { await sleep(30); slow = false; });
		const ses3 = startWorkerSession(w3, q3, {}, () => { /* */ });
		ses3.cancel();
		await ses3.done;
		assert(!slow, 'cancel：等已入队任务完成后才 resolve');
	});

	await describe('[4] 静态世界排除：只排除真的已精确表示的图元', () => {
		const rep = new RepresentedSet();
		rep.add('line', 'L1'); rep.add('arc', 'A1'); rep.add('polyline', 'P1');
		rep.add('line', '');   // 空 id 不登记
		assert(isRepresentedExactly(rep, 'line', 'N', 1, 'L1'), '登记过的带网络铜层线 → 排除');
		assert(isRepresentedExactly(rep, 'arc', 'N', 2, 'A1'), '登记过的弧 → 排除');
		assert(isRepresentedExactly(rep, 'polyline', 'N', 1, 'P1'), '登记过的折线 → 排除');
		assert(!isRepresentedExactly(rep, 'arc', 'N', 1, 'A_BAD'), '转换失败（未登记）的弧 → 保留为障碍');
		assert(!isRepresentedExactly(rep, 'line', 'N', 1, 'L_BAD'), '未登记的线 → 保留');
		assert(!isRepresentedExactly(rep, 'polyline', 'N', 1, 'P_SHORT'), '未登记的折线 → 保留');
		assert(!isRepresentedExactly(rep, 'line', 'N', 1, ''), '无 id → 保留');
		assert(!isRepresentedExactly(rep, 'arc', 'N', 1, 'L1'), '种类不同不串（line 的 id 不能排除 arc）');
		assert(!isRepresentedExactly(rep, 'line', '', 1, 'L1'), '无网络 → 保留');
		assert(!isRepresentedExactly(rep, 'line', undefined, 1, 'L1'), '网络 undefined → 保留');
		assert(!isRepresentedExactly(rep, 'line', 'N', 12, 'L1'), '非铜层(12 等) → 保留');
		assert(!isRepresentedExactly(null, 'line', 'N', 1, 'L1'), '布线口径(null)永不排除，行为与旧版一致');

		assert(lineEndpointsUsable({ startX: 0, startY: 1, endX: 2, endY: 3 }), '正常线坐标可用');
		assert(lineEndpointsUsable({ x: 0, y: 1, endX: 2, endY: 3 }), 'x/y 回退可用');
		assert(!lineEndpointsUsable({ startX: 0, startY: 1, endX: 2 }), '缺终点 y → 不可用');
		assert(!lineEndpointsUsable({ startX: NaN, startY: 1, endX: 2, endY: 3 }), 'NaN → 不可用');
	});

	await describe('SerialQueue：严格按到达顺序串行', async () => {
		const q = new SerialQueue();
		const order: string[] = [];
		const sleep = (ms: number) => new Promise<void>(r => setTimeout(r, ms));
		q.push(async () => { order.push('1s'); await sleep(30); order.push('1e'); });
		q.push(async () => { order.push('2s'); throw new Error('x'); });
		q.push(async () => { order.push('3s'); await sleep(1); order.push('3e'); });
		await q.idle();
		assert(order.join() === '1s,1e,2s,3s,3e', '后一任务在前一任务结束后才开始，异常不中断后续');
	});

	console.log(`\n${passed} 通过，${failed} 失败`);
	if (failed > 0) process.exit(1);
}

main().catch(err => { console.error(err); process.exit(1); });
