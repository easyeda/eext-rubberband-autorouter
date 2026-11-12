/**
 * 板边双面同网焊盘（上下层同坐标 SMD 孪生盘）+ 换端点备选 回归测试
 * 运行方式：npx tsx test/routing-twinpad-test.ts   （< 20 s，纯合成数据）
 *
 * 复现（缩微版 clock-pcb3 LCD 总线）：网 SIG = 引脚 P（顶层小 SMD）+ 右侧孪生盘 R1(L1)/R2(L2)
 * + 左侧远端孪生盘 L1(L1)/L2(L2)。P 与 R1 被异网顶层焊盘围成的窄槽包住（槽宽只够走线，
 * 槽上只有一个刚好放下一个过孔的凹位）。MST = P–R1、R1–R2（换层，占掉凹位唯一的过孔位）、
 * P–L1（P 出不了槽、槽内再无过孔位 → 必败）、L1–L2。
 * 期望：P–L1 失败后由「同分量换端点」改走 R2–L2（底层直连）补上，整网布通，严格净距 0 违规；
 * 关掉备选（__RB_NO_ALT_ENDPOINT__）时 SIG 布不通（证明测试确实覆盖到该机制）。
 * 另断言 MST 换层罚分：引脚到两个孪生盘几乎等距时选同层（顶层）孪生盘。
 */
import { PlanningEngine } from '../src/router/planning-engine';

let passed = 0;
let failed = 0;
function assert(cond: boolean, msg: string): void {
	if (cond) { console.log(`  ✅ ${msg}`); passed++; } else { console.error(`  ❌ ${msg}`); failed++; }
}

const drc = {
	traceWidthMil: 10, clearanceMil: 6, trackToTrackMil: 4, trackToLineMil: 6,
	trackToBoardMil: 12, trackToPadMil: 6, trackToFillMil: 10, holeToHoleMil: 11.8,
};
type Pad = { x: number; y: number; net: string; id: string; layer: number; padType: 'smd'; holeDiameter: 0; radius: number; bodyRadius: number; index: number };
const pads: Pad[] = [];
const padShapes: Record<string, unknown> = {};
const addPad = (id: string, net: string, x: number, y: number, layer: number, w: number, h: number): Pad => {
	const p: Pad = { x, y, net, id, layer, padType: 'smd', holeDiameter: 0, radius: Math.hypot(w, h) / 2, bodyRadius: Math.min(w, h) / 2, index: pads.length };
	pads.push(p);
	padShapes[`${x.toFixed(2)},${y.toFixed(2)}`] = { pad: ['RECT', w, h, 0], hole: null, rotation: 0 };
	return p;
};

function buildInput(): any {
	pads.length = 0;
	for (const k of Object.keys(padShapes)) delete padShapes[k];
	const sig = [
		addPad('P', 'SIG', 0, 0, 1, 20, 12),
		addPad('R1', 'SIG', 150, 0, 1, 30, 20),
		addPad('R2', 'SIG', 150, 0, 2, 30, 20),
		addPad('L1', 'SIG', -1500, 0, 1, 30, 20),
		addPad('L2', 'SIG', -1500, 0, 2, 30, 20),
	];
	// 顶层异网围墙：16mil 方格，空格 = 槽（y∈[-16,16]，x∈[-16,208]）+ 凹位（x∈[48,96]，y∈[-32,-16]）
	const C = 16;
	const free = (i: number, j: number) =>
		(j >= -1 && j <= 0 && i >= -1 && i <= 12) || (j === -2 && i >= 3 && i <= 5);
	let w = 0;
	for (let i = -3; i <= 14; i++) for (let j = -4; j <= 2; j++) {
		if (free(i, j)) continue;
		// 只留与空格相邻的一圈（够封口，障碍少）
		let edge = false;
		for (let di = -1; di <= 1; di++) for (let dj = -1; dj <= 1; dj++) if (free(i + di, j + dj)) edge = true;
		if (edge) addPad(`W${w++}`, 'WALL', i * C + C / 2, j * C + C / 2, 1, C, C);
	}
	return {
		drc, allPads: pads, networks: [{ name: 'SIG', pads: sig }], staticObstacles: [], padShapes,
		copperLayers: 2, budgetMs: 1800000,
		options: { connectivityFirst: true, optimize: true, allowViaFallback: true, allowViaInPad: false, escapeClearanceMil: 0, ripupRounds: 3, powerNetsFirst: false, netOrder: 'auto' },
	};
}

// ── 严格边到边审计（同 bench/route-dump.mjs 口径的缩略版：走线-走线 / 走线-焊盘 / 走线-过孔，同层、异网）──
function segDist(px: number, py: number, x1: number, y1: number, x2: number, y2: number): number {
	const dx = x2 - x1, dy = y2 - y1, L = dx * dx + dy * dy;
	const t = L ? Math.max(0, Math.min(1, ((px - x1) * dx + (py - y1) * dy) / L)) : 0;
	return Math.hypot(px - x1 - t * dx, py - y1 - t * dy);
}
function samples(prims: any[]): Array<{ x: number; y: number; layer: number }> {
	const out: Array<{ x: number; y: number; layer: number }> = [];
	for (const p of prims) {
		if (p.kind === 'line') {
			const k = Math.max(1, Math.ceil(Math.hypot(p.x2 - p.x1, p.y2 - p.y1) / 1));
			for (let i = 0; i <= k; i++) out.push({ x: p.x1 + (p.x2 - p.x1) * i / k, y: p.y1 + (p.y2 - p.y1) * i / k, layer: p.layer });
		} else {
			const k = Math.max(2, Math.ceil(Math.abs(p.sweep) * p.r));
			for (let i = 0; i <= k; i++) { const a = p.startAngle + p.sweep * i / k; out.push({ x: p.cx + p.r * Math.cos(a), y: p.cy + p.r * Math.sin(a), layer: p.layer }); }
		}
	}
	return out;
}
function edgeViolations(input: any, res: any): string[] {
	const bad: string[] = [];
	const hw = drc.traceWidthMil / 2;
	for (const n of res.nets) {
		for (const s of samples(n.prims)) {
			for (const pd of input.allPads) {
				if (pd.net === n.name || pd.layer !== s.layer) continue;
				const sh = input.padShapes[`${pd.x.toFixed(2)},${pd.y.toFixed(2)}`].pad;
				const ox = Math.abs(s.x - pd.x) - sh[1] / 2, oy = Math.abs(s.y - pd.y) - sh[2] / 2;
				const d = (ox > 0 && oy > 0 ? Math.hypot(ox, oy) : Math.max(ox, oy)) - hw;
				if (d < drc.trackToPadMil - 0.5) { bad.push(`${n.name}-pad ${pd.id} ${d.toFixed(2)} @${s.x.toFixed(0)},${s.y.toFixed(0)}`); break; }
			}
			for (const v of res.vias) {
				if (v.net === n.name) continue;
				const d = Math.hypot(v.x - s.x, v.y - s.y) - v.dia / 2 - hw;
				if (d < drc.trackToPadMil - 0.5) bad.push(`${n.name}-via ${d.toFixed(2)}`);
			}
		}
	}
	// 过孔 ↔ 异网焊盘（铜到铜）与同网孔-孔
	for (const v of res.vias) {
		for (const pd of input.allPads) {
			if (pd.net === v.net) continue;
			const sh = input.padShapes[`${pd.x.toFixed(2)},${pd.y.toFixed(2)}`].pad;
			const ox = Math.abs(v.x - pd.x) - sh[1] / 2, oy = Math.abs(v.y - pd.y) - sh[2] / 2;
			const d = (ox > 0 && oy > 0 ? Math.hypot(ox, oy) : Math.max(ox, oy)) - v.dia / 2;
			if (d < drc.trackToPadMil - 0.5) bad.push(`via-pad ${pd.id} ${d.toFixed(2)}`);
		}
		for (const u of res.vias) {
			if (u === v) continue;
			const d = Math.hypot(u.x - v.x, u.y - v.y) - (u.hole + v.hole) / 2;
			if (d < drc.holeToHoleMil - 0.5) bad.push(`hole-hole ${d.toFixed(2)}`);
		}
	}
	// 焊盘覆盖：每个 SIG 焊盘都有同层铜皮端点落在焊盘内
	return bad;
}

async function run(alt: boolean, ngrid = true): Promise<{ res: any; input: any; ms: number }> {
	(globalThis as any).__RB_NO_ALT_ENDPOINT__ = alt ? undefined : 1;
	// 全局协商网格收尾（negotiated-grid）也能解开这个局面：复现用例要单独关掉它
	(globalThis as any).__RB_NO_NGRID__ = ngrid ? undefined : 1;
	const input = buildInput();
	const t0 = Date.now();
	const res = await new PlanningEngine().plan(input, { isCancelled: () => false });
	return { res, input, ms: Date.now() - t0 };
}

(async () => {
	const quiet = console.warn; console.warn = () => {};
	console.log('\n▶ MST 换层罚分：近似等距孪生盘选同层');
	{
		const eng: any = new PlanningEngine();
		eng.drc = drc;
		const p = [
			{ x: 0, y: 0, net: 'N', id: 'pin', layer: 1, padType: 'smd', holeDiameter: 0, radius: 6, index: 0 },
			{ x: 133.5, y: -8.4, net: 'N', id: 'top', layer: 1, padType: 'smd', holeDiameter: 0, radius: 20, index: 1 },
			{ x: 133.2, y: -6.8, net: 'N', id: 'bot', layer: 2, padType: 'smd', holeDiameter: 0, radius: 20, index: 2 },
		];
		const e: Array<[number, number]> = eng.computeMST(p);
		const touchesPin = e.filter(([a, b]) => a === 0 || b === 0).map(([a, b]) => p[a === 0 ? b : a].id);
		assert(touchesPin.length === 1 && touchesPin[0] === 'top', `引脚连顶层孪生盘（实际 ${touchesPin.join(',')}）`);
	}

	console.log('\n▶ 槽内引脚 + 板边孪生盘：关闭换端点备选与全局协商网格（复现）');
	const off = await run(false, false);
	console.log(`     ${off.ms}ms routed=${off.res.routedNets.join(',') || '-'} vias=${off.res.vias.length}`);
	assert(!off.res.routedNets.includes('SIG'), 'SIG 在无备选时布不通（场景确实触发 P–L1 失败）');

	console.log('\n▶ 槽内引脚 + 板边孪生盘：关闭换端点备选、开启全局协商网格');
	const ng = await run(false, true);
	console.log(`     ${ng.ms}ms routed=${ng.res.routedNets.join(',') || '-'} vias=${ng.res.vias.length}`);
	assert(ng.res.routedNets.includes('SIG'), 'SIG 由全局协商网格收尾布通');
	const ngBad = edgeViolations(ng.input, ng.res);
	assert(ngBad.length === 0, `严格净距 0 违规${ngBad.length ? '：' + ngBad.slice(0, 4).join(' | ') : ''}`);

	console.log('\n▶ 槽内引脚 + 板边孪生盘：开启换端点备选（关闭全局协商网格，单测备选机制本身）');
	const on = await run(true, false);
	console.log(`     ${on.ms}ms routed=${on.res.routedNets.join(',') || '-'} vias=${on.res.vias.length}`);
	assert(on.res.routedNets.includes('SIG'), 'SIG 布通');
	const bad = edgeViolations(on.input, on.res);
	assert(bad.length === 0, `严格净距 0 违规${bad.length ? '：' + bad.slice(0, 4).join(' | ') : ''}`);
	const sig = on.res.nets.find((n: any) => n.name === 'SIG');
	const bottomLong = (sig?.prims ?? []).some((p: any) => p.layer === 2 && p.kind === 'line' && Math.min(p.x1, p.x2) < -1400 && Math.max(p.x1, p.x2) > 100);
	assert(bottomLong, '长连接走底层（R2 → L2 孪生盘之间）');
	assert(on.ms < 20000, `耗时 < 20 s（实际 ${on.ms}ms）`);
	console.warn = quiet;

	console.log(`\n${'─'.repeat(50)}`);
	console.log(`测试结果：${passed} 通过，${failed} 失败`);
	process.exit(failed > 0 ? 1 : 0);
})();
