/**
 * 全局协商网格收尾（negotiated-grid）回归测试
 * 运行方式：npx tsx test/routing-negotiated-test.ts
 *
 * 覆盖：双层交叉（必须换层）/ 单层交叉（无解 → 只交出两两无冲突的子集）/
 * 固定网封口（被穿越的固定网升级重布）/ 结果的精确净距（走线↔走线、走线↔过孔、
 * 走线↔焊盘、过孔↔过孔）/ 端点落在焊盘中心 / 确定性（两次结果逐字一致）/
 * 链排序（相邻链不首尾相接）/ 升级网被剔时原铜皮退回且无人压占 / 扩展总量上限 /
 * 焊盘线段精确距离 / 引擎提交复核的静态障碍检查（长胶囊中段、圆弧）/
 * 引擎提交路径：失败（含取消）的部分布通网保留残段与焊盘连接数、求解器漏收的 pending 网被放回。
 */

import { negotiatedGridRoute, orderChains, padSegMin, NGRequest, NGPad, NGRules, NGResult, NGStatic } from '../src/router/negotiated-grid';
import { PlanningEngine } from '../src/router/planning-engine';

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

function describe(name: string, fn: () => void): void {
	console.log(`\n▶ ${name}`);
	fn();
}

function segDist(px: number, py: number, ax: number, ay: number, bx: number, by: number): number {
	const dx = bx - ax, dy = by - ay;
	const l2 = dx * dx + dy * dy;
	let t = l2 > 0 ? ((px - ax) * dx + (py - ay) * dy) / l2 : 0;
	t = Math.max(0, Math.min(1, t));
	return Math.hypot(px - (ax + dx * t), py - (ay + dy * t));
}

/** 线段间最小距离（采样 0.25mil，测试用足够精确） */
function segSeg(a: number[], b: number[]): number {
	const n = Math.max(1, Math.ceil(Math.hypot(a[2] - a[0], a[3] - a[1]) / 0.25));
	let best = Infinity;
	for (let k = 0; k <= n; k++) {
		const t = k / n;
		best = Math.min(best, segDist(a[0] + (a[2] - a[0]) * t, a[1] + (a[3] - a[1]) * t, b[0], b[1], b[2], b[3]));
	}
	return best;
}

// 与引擎换算口径同量级的规则（w=10 / tt=4 / tp=6 / via 24/12 / h2h 12）
const RULES: NGRules = {
	trkTrk: 14.5, trkPad: 11.2, trkHoleExtra: 11.2, trkVia: 24.6,
	viaVia: 34.4, viaViaSame: 24.2, viaPad: 19.2, viaPadSame: 16,
	viaHoleExtra: 18.2, staticTrkExtra: 0.4, staticViaExtra: 10.2, slack: 0.4,
};

const roundPad = (x: number, y: number, net: string, r = 8, layers = [1]): NGPad =>
	({ x, y, net, layers, extent: r, holeD: 0, dist: (px, py) => Math.hypot(px - x, py - y) - r });

/** 板框：矩形外圈（圆心在边上，r 已含间距） */
function boardRing(x0: number, y0: number, x1: number, y1: number): NGStatic[] {
	const out: NGStatic[] = [];
	const step = 8;
	for (let x = x0; x <= x1; x += step) { out.push({ kind: 'circle', x, y: y0, r: 16, board: true }); out.push({ kind: 'circle', x, y: y1, r: 16, board: true }); }
	for (let y = y0; y <= y1; y += step) { out.push({ kind: 'circle', x: x0, y, r: 16, board: true }); out.push({ kind: 'circle', x: x1, y, r: 16, board: true }); }
	return out;
}

/** 结果的精确净距审计：返回违规描述列表 */
function audit(req: NGRequest, res: NGResult): string[] {
	const bad: string[] = [];
	const segs: Array<{ net: string; layer: number; s: number[] }> = [];
	const vias: Array<{ net: string; x: number; y: number }> = [];
	for (const r of res.routed) {
		for (const ch of r.chains) for (let k = 1; k < ch.pts.length; k++) segs.push({ net: r.name, layer: ch.layer, s: [ch.pts[k - 1].x, ch.pts[k - 1].y, ch.pts[k].x, ch.pts[k].y] });
		for (const v of r.vias) vias.push({ net: r.name, ...v });
	}
	for (const f of req.fixed) {
		if (res.escalated.includes(f.name)) continue;
		for (const s of f.segs) segs.push({ net: f.name, layer: s.layer, s: [s.x1, s.y1, s.x2, s.y2] });
		for (const v of f.vias) vias.push({ net: f.name, ...v });
	}
	for (let i = 0; i < segs.length; i++) {
		for (let j = i + 1; j < segs.length; j++) {
			const a = segs[i], b = segs[j];
			if (a.net === b.net || a.layer !== b.layer) continue;
			const d = segSeg(a.s, b.s);
			if (d < RULES.trkTrk - 0.05) bad.push(`trk-trk ${a.net}/${b.net} ${d.toFixed(2)}`);
		}
		for (const v of vias) {
			if (v.net === segs[i].net) continue;
			const d = segDist(v.x, v.y, segs[i].s[0], segs[i].s[1], segs[i].s[2], segs[i].s[3]);
			if (d < RULES.trkVia - 0.05) bad.push(`trk-via ${segs[i].net}/${v.net} ${d.toFixed(2)}`);
		}
		for (const p of req.pads) {
			if (p.net === segs[i].net || !p.layers.includes(segs[i].layer)) continue;
			const s = segs[i].s;
			const n = Math.max(1, Math.ceil(Math.hypot(s[2] - s[0], s[3] - s[1]) / 0.25));
			for (let k = 0; k <= n; k++) {
				const t = k / n;
				const d = p.dist(s[0] + (s[2] - s[0]) * t, s[1] + (s[3] - s[1]) * t);
				if (d < RULES.trkPad - 0.05) { bad.push(`trk-pad ${segs[i].net}/${p.net} ${d.toFixed(2)}`); break; }
			}
		}
	}
	for (let i = 0; i < vias.length; i++) {
		for (let j = i + 1; j < vias.length; j++) {
			const d = Math.hypot(vias[i].x - vias[j].x, vias[i].y - vias[j].y);
			const need = vias[i].net === vias[j].net ? RULES.viaViaSame : RULES.viaVia;
			if (d < need - 0.05) bad.push(`via-via ${vias[i].net}/${vias[j].net} ${d.toFixed(2)}`);
		}
		for (const p of req.pads) {
			const d = p.dist(vias[i].x, vias[i].y);
			const need = p.net === vias[i].net ? RULES.viaPadSame : RULES.viaPad;
			if (d < need - 0.05) bad.push(`via-pad ${vias[i].net}/${p.net} ${d.toFixed(2)}`);
		}
	}
	return bad;
}

/** 每个网的每个焊盘中心都是某条链的端点（引擎终审按焊盘铜皮连通，端点落焊盘中心最稳） */
function padsAnchored(req: NGRequest, res: NGResult): boolean {
	for (const r of res.routed) {
		const net = req.nets.find(n => n.name === r.name)!;
		for (const pi of net.pads) {
			const p = req.pads[pi];
			// 链端点必须精确落在焊盘中心（1e-6），且链所在层是该焊盘的铜皮层
			const hit = r.chains.some(ch => p.layers.includes(ch.layer) &&
				[ch.pts[0], ch.pts[ch.pts.length - 1]].some(q => Math.hypot(q.x - p.x, q.y - p.y) < 1e-6));
			if (!hit) return false;
		}
	}
	return true;
}

function baseReq(over: Partial<NGRequest>): NGRequest {
	return {
		cell: 4, layerCount: 2, pads: [], nets: [], participants: [], fixed: [],
		statics: boardRing(-40, -40, 440, 240), rules: RULES, viaInPad: false,
		...over,
	};
}

describe('双层交叉：一网必须换层', () => {
	// A 两端贴板框、B 两端贴上下板框（端头外侧都放不下走线）：只能换层穿过
	const pads = [roundPad(-10, 100, 'A'), roundPad(410, 100, 'A'), roundPad(200, 10, 'B'), roundPad(200, 190, 'B')];
	const req = baseReq({
		pads,
		nets: [{ name: 'A', pads: [0, 1] }, { name: 'B', pads: [2, 3] }],
		participants: ['A', 'B'],
		statics: boardRing(-40, -15, 440, 215),
	});
	const res = negotiatedGridRoute(req);
	assert(res.ok && res.converged, `收敛（${res.iterations} 轮）`);
	assert(res.routed.length === 2 && res.failed.length === 0, '两网都布通');
	const nv = res.routed.reduce((s, r) => s + r.vias.length, 0);
	assert(nv >= 2, `换层用了过孔（${nv} 个）`);
	const bad = audit(req, res);
	assert(bad.length === 0, `精确净距无违规${bad.length ? '：' + bad.slice(0, 3).join('; ') : ''}`);
	assert(padsAnchored(req, res), '焊盘中心都是链端点');
	const again = negotiatedGridRoute(req);
	assert(JSON.stringify(again) === JSON.stringify(res), '确定性：两次结果逐字一致');
});

describe('单层交叉：无解时只交出两两无冲突的子集', () => {
	const pads = [roundPad(-10, 100, 'A'), roundPad(410, 100, 'A'), roundPad(200, 10, 'B'), roundPad(200, 190, 'B')];
	const req = baseReq({
		layerCount: 1, pads,
		nets: [{ name: 'A', pads: [0, 1] }, { name: 'B', pads: [2, 3] }],
		participants: ['A', 'B'],
		statics: boardRing(-40, -15, 440, 215),
		maxIter: 12,
	});
	const res = negotiatedGridRoute(req);
	assert(res.ok && !res.converged, '不收敛');
	assert(res.routed.length === 1 && res.failed.length === 1, `一网布通一网失败（布通 ${res.routed.map(r => r.name)}）`);
	assert(audit(req, res).length === 0, '交出的子集无违规');
});

describe('固定网封口：被穿越的固定网升级并一起重布', () => {
	// C 的固定走线横贯顶层（两端贴板框），正好隔开 A 的两个顶层焊盘；底层 y=60
	// 有不可升级的 D：A 在 C 上方换层的过孔无处可放（离 C、D 都要 24.6mil），
	// A 只能穿 C → C 升级，自己换层从 A 下方钻过去
	const pads = [
		roundPad(200, 20, 'A'), roundPad(200, 180, 'A'),
		roundPad(-10, 100, 'C'), roundPad(410, 100, 'C'),
		roundPad(-10, 60, 'D', 8, [2]), roundPad(410, 60, 'D', 8, [2]),
	];
	const req = baseReq({
		pads,
		nets: [{ name: 'A', pads: [0, 1] }, { name: 'C', pads: [2, 3] }, { name: 'D', pads: [4, 5] }],
		participants: ['A'],
		fixed: [
			{ name: 'C', segs: [{ layer: 1, x1: -10, y1: 100, x2: 410, y2: 100 }], vias: [], escalatable: true },
			{ name: 'D', segs: [{ layer: 2, x1: -10, y1: 60, x2: 410, y2: 60 }], vias: [], escalatable: false },
		],
		statics: boardRing(-40, -40, 440, 240),
	});
	const res = negotiatedGridRoute(req);
	assert(res.ok && res.converged, `收敛（${res.iterations} 轮）`);
	assert(res.escalated.includes('C'), `C 被升级（升级 ${res.escalated.join(',')}）`);
	assert(!res.escalated.includes('D'), '不可升级的 D 保持不动');
	assert(res.routed.some(r => r.name === 'A') && res.routed.some(r => r.name === 'C'), 'A 与 C 都布通');
	const bad = audit(req, res);
	assert(bad.length === 0, `精确净距无违规${bad.length ? '：' + bad.slice(0, 3).join('; ') : ''}`);
	assert(padsAnchored(req, res), '焊盘中心都是链端点');
});

describe('多焊盘网：树生长 + 拉紧后仍合法', () => {
	const pads = [
		roundPad(20, 20, 'N'), roundPad(380, 30, 'N'), roundPad(200, 180, 'N'), roundPad(60, 170, 'N'),
		roundPad(20, 120, 'M'), roundPad(380, 140, 'M'),
	];
	const req = baseReq({
		pads,
		nets: [{ name: 'N', pads: [0, 1, 2, 3] }, { name: 'M', pads: [4, 5] }],
		participants: ['N', 'M'],
	});
	const res = negotiatedGridRoute(req);
	assert(res.ok && res.routed.length === 2, '两网布通');
	const bad = audit(req, res);
	assert(bad.length === 0, `精确净距无违规${bad.length ? '：' + bad.slice(0, 3).join('; ') : ''}`);
	assert(padsAnchored(req, res), '四个焊盘中心都是链端点');
	// 拉紧：空场里不该残留网格楼梯（每条链顶点数很少）
	const maxPts = Math.max(...res.routed.flatMap(r => r.chains.map(c => c.pts.length)));
	assert(maxPts <= 6, `链已拉紧（最长链 ${maxPts} 点）`);
});

describe('orderChains：相邻链尽量不首尾相接', () => {
	const J = { x: 100, y: 100 };
	const chains = [
		{ layer: 1, pts: [{ x: 0, y: 100 }, J] },
		{ layer: 1, pts: [J, { x: 200, y: 100 }] },
		{ layer: 1, pts: [J, { x: 100, y: 0 }] },
		{ layer: 1, pts: [{ x: 300, y: 300 }, { x: 400, y: 300 }] },
	];
	const out = orderChains(chains);
	let touching = 0;
	for (let i = 1; i < out.length; i++) {
		const a = out[i - 1].pts[out[i - 1].pts.length - 1], b = out[i].pts[0];
		if (Math.hypot(a.x - b.x, a.y - b.y) <= 15) touching++;
	}
	assert(out.length === 4, '链数不变');
	assert(touching <= 1, `相接的相邻链 ≤1（实得 ${touching}）`);
});

describe('升级网被剔：原铜皮退回固定网，幸存结果与它无冲突', () => {
	// 单层板：A 只能横穿固定网 C → C 升级；只给 1 轮，C 来不及重布 → 收尾剔除 C，
	// C 必须原样退回（不在 failed / escalated 里），压在 C 旧走廊上的 A 随之被剔
	const pads = [
		roundPad(200, 20, 'A'), roundPad(200, 180, 'A'),
		roundPad(-10, 100, 'C'), roundPad(410, 100, 'C'),
	];
	const req = baseReq({
		layerCount: 1, pads,
		nets: [{ name: 'A', pads: [0, 1] }, { name: 'C', pads: [2, 3] }],
		participants: ['A'],
		fixed: [{ name: 'C', segs: [{ layer: 1, x1: -10, y1: 100, x2: 410, y2: 100 }], vias: [], escalatable: true }],
		maxIter: 1,
	});
	const logs: string[] = [];
	const res = negotiatedGridRoute({ ...req, log: m => logs.push(m) });
	assert(res.ok && !res.converged, '1 轮不收敛');
	assert(logs.some(m => /升级 1/.test(m)) && logs.some(m => /退回固定网.*C/.test(m)), 'C 确实被升级过、又被退回');
	assert(res.failed.includes('A'), 'A（压 C 旧走廊）被剔');
	assert(!res.escalated.includes('C') && !res.failed.includes('C'), `C 退回固定网（escalated=${res.escalated} failed=${res.failed}）`);
	assert(!res.routed.some(r => r.name === 'C'), 'C 不交出新几何（引擎不撕它）');
	// audit 会把未升级的固定网 C 的原几何算进去：结果与它无冲突 = 原样放回必过
	const bad = audit(req, res);
	assert(bad.length === 0, `与 C 原铜皮无冲突${bad.length ? '：' + bad.slice(0, 3).join('; ') : ''}`);
	// 布通网数不低于阶段前（阶段前只有 C）：交出的网 + 保持原样的固定网
	const routedAfter = res.routed.length + req.fixed.filter(f => !res.escalated.includes(f.name)).length;
	assert(routedAfter >= 1, `布通网数不降（前 1 → 后 ${routedAfter}）`);
});

describe('扩展总量上限在搜索内部生效', () => {
	const pads = [roundPad(-10, 100, 'A'), roundPad(410, 100, 'A'), roundPad(200, 10, 'B'), roundPad(200, 190, 'B')];
	const req = baseReq({
		pads,
		nets: [{ name: 'A', pads: [0, 1] }, { name: 'B', pads: [2, 3] }],
		participants: ['A', 'B'],
		statics: boardRing(-40, -15, 440, 215),
		maxTotalExpand: 300,
	});
	const res = negotiatedGridRoute(req);
	assert(res.ok, '正常返回');
	assert(res.expansions <= 300 + 1, `总扩展不超上限（${res.expansions}）`);
	assert(audit(req, res).length === 0, '交出的部分无违规');
});

describe('padSegMin：矩形焊盘角附近精确（粗采样会漏 ~0.25mil）', () => {
	// 10×10 方盘，线段从角外斜掠：真最小距离 = 角点到直线的距离
	const sq: NGPad = {
		x: 0, y: 0, net: 'P', layers: [1], extent: Math.hypot(5, 5), holeD: 0,
		dist: (px, py) => { const qx = Math.abs(px) - 5, qy = Math.abs(py) - 5; return qx > 0 && qy > 0 ? Math.hypot(qx, qy) : Math.max(qx, qy); },
	};
	const ax = -20, ay = 20, bx = 30, by = 0;   // y = 12 - 0.4x：从右上角外侧斜掠，最近点就是角点
	const exact = segDist(5, 5, ax, ay, bx, by);
	const got = padSegMin(sq, ax, ay, bx, by, 20);
	assert(Math.abs(got - exact) < 1e-4, `最小距离 ${got.toFixed(5)} ≈ 角点距离 ${exact.toFixed(5)}`);
});

describe('引擎提交复核：静态障碍按真实几何（长胶囊中段 / 圆弧）', () => {
	const eng: any = new PlanningEngine();
	eng.staticObstacles = [{ id: 'k', kind: 'capsule', x: 0, y: 0, x1: 0, y1: 0, x2: 1000, y2: 0, r: 16, group: 'board' }];
	const line = (x1: number, y1: number, x2: number, y2: number) => ({ kind: 'line', net: 'A', layer: 1, x1, y1, x2, y2 });
	assert(eng.primsStaticViolation([line(500, -100, 500, 100)], 'A') !== null, '横穿长胶囊中段：拒');
	assert(eng.primsStaticViolation([line(500, 14, 600, 40)], 'A') !== null, '贴近长胶囊中段（14mil < 16.4）：拒');
	assert(eng.primsStaticViolation([line(500, 40, 600, 40)], 'A') === null, '离中段 40mil：放行');
	// 圆弧：圆心 (500,60) 半径 50，下半圆弧最低点 (500,10) 距胶囊轴 10 < 16.4
	const arc = { kind: 'arc', net: 'A', layer: 1, cx: 500, cy: 60, r: 50, startAngle: Math.PI * 0.75, sweep: -Math.PI * 0.5 };
	const arcDown = { ...arc, startAngle: -Math.PI * 0.75, sweep: Math.PI * 0.5 };
	assert(eng.primsStaticViolation([arcDown], 'A') !== null, '圆弧扫过胶囊中段：拒');
	assert(eng.primsStaticViolation([arc], 'A') === null, '圆弧在远侧：放行');
});

/**
 * 引擎侧提交路径（negotiatedGridPhase）：用替身求解器构造「部分布通网失败」。
 * P = 3 焊盘网，入口时只布了 p0–p1（部分布通）；Q = 已布通网（被升级、交出新几何）。
 */
async function phaseCase(opts: { omitP?: boolean; cancelOnRip?: boolean }) {
	const drc = { traceWidthMil: 10, clearanceMil: 6, trackToTrackMil: 4, trackToLineMil: 6, trackToBoardMil: 12, trackToPadMil: 6, trackToFillMil: 10, holeToHoleMil: 11.8 };
	const mk = (id: string, net: string, x: number, y: number, index: number) =>
		({ id, net, x, y, layer: 1, padType: 'smd', holeDiameter: 0, radius: 8, bodyRadius: 8, index });
	const p0 = mk('p0', 'P', 0, 0, 0), p1 = mk('p1', 'P', 200, 0, 1), p2 = mk('p2', 'P', 400, 300, 2);
	const q0 = mk('q0', 'Q', 0, 200, 3), q1 = mk('q1', 'Q', 200, 200, 4);
	const networks = [{ name: 'P', pads: [p0, p1, p2] }, { name: 'Q', pads: [q0, q1] }];
	let cancel = false;
	const eng: any = new PlanningEngine();
	eng.drc = drc; eng.allPads = [p0, p1, p2, q0, q1]; eng.padShapes = {}; eng.staticObstacles = [];
	eng.callbacks = { isCancelled: () => cancel, onRipup: () => { if (opts.cancelOnRip) cancel = true; } };
	const pLine = { kind: 'line', net: 'P', layer: 1, x1: 0, y1: 0, x2: 200, y2: 0 };
	eng.netRendered.set('P', [pLine]);
	eng.netRendered.set('Q', [{ kind: 'line', net: 'Q', layer: 1, x1: 0, y1: 200, x2: 200, y2: 200 }]);
	const [a, b] = eng.computeMST(networks[0].pads)[0];
	eng.routedEdges.set('P', new Set([`${networks[0].pads[a].id}->${networks[0].pads[b].id}`]));
	eng.markAllEdgesDone(networks[1]);
	eng.routedOrder = ['Q'];
	eng.addRenderedToObstacles([...eng.netRendered.get('P'), ...eng.netRendered.get('Q')]);
	const padsP0 = eng.padProgress(networks[0]).routed;
	eng.ngridSolver = (): NGResult => ({
		ok: true, iterations: 1, converged: false, expansions: 0,
		routed: [{ name: 'Q', chains: [{ layer: 1, pts: [{ x: 0, y: 200 }, { x: 200, y: 200 }] }], vias: [] }],
		failed: opts.omitP ? [] : ['P'],
		escalated: ['Q'],
	});
	await eng.negotiatedGridPhase(networks, Number.POSITIVE_INFINITY);
	const ps = eng.netRendered.get('P') ?? [];
	return {
		keptPartial: ps.some((p: any) => p.kind === 'line' && Math.hypot(p.x1 - 0, p.y1 - 0) < 1e-9 && Math.hypot(p.x2 - 200, p.y2 - 0) < 1e-9),
		padsBefore: padsP0, padsAfter: eng.padProgress(networks[0]).routed,
		qRouted: eng.routedOrder.includes('Q'),
	};
}

(async () => {
	console.log('\n▶ 引擎提交路径：部分布通网在本阶段失败');
	{
		const r = await phaseCase({});
		assert(r.qRouted, 'Q 仍布通');
		assert(r.padsAfter >= r.padsBefore, `P 焊盘连接数不降（${r.padsBefore} → ${r.padsAfter}）`);
		assert((r.keptPartial || r.padsAfter > r.padsBefore), 'P 残段放回（或已补成更多连接）');
	}
	console.log('\n▶ 引擎提交路径：部分布通网失败 + 撕网后即取消');
	{
		const r = await phaseCase({ cancelOnRip: true });
		assert(r.keptPartial, 'P 残段原样放回');
		assert(r.padsAfter === r.padsBefore, `P 焊盘连接数不变（${r.padsBefore} → ${r.padsAfter}）`);
		assert(r.qRouted, 'Q 仍布通');
	}
	console.log('\n▶ 引擎提交路径：求解器既没交出也没报失败的 pending 网');
	{
		const r = await phaseCase({ omitP: true, cancelOnRip: true });
		assert(r.keptPartial, 'P 残段原样放回（leftovers = touched − committed）');
		assert(r.padsAfter === r.padsBefore, `P 焊盘连接数不变（${r.padsBefore} → ${r.padsAfter}）`);
	}
	console.log(`\n${passed} passed, ${failed} failed`);
	if (failed) process.exit(1);
})();
