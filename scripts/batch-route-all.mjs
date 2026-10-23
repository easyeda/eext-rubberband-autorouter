// 批量离线布线 + AI 可检查结果输出（不启动 EDA）
// 用法: node tmp/batch-route-all.mjs [--budget 毫秒] [--filter <子串>] [--limit N] [--redump]
//  - 读 tmp/dumps/*-dump.json（structVersion 不匹配时警告；--redump 自动重导该工程）
//  - 每工程输出两个文件（tmp/results/<case名>/）：
//      digest.json  = AI 专用微型摘要（<5KB）：布通率/失败网/DRC top10/接缝缺陷/部分网清单
//                     ⚠ AI 检查结果只许读 digest.json 与 _summary.json，禁止整读 result.json
//      result.json  = 全量数据（含全部图元），仅供脚本/写回使用，不进 AI 上下文
//  - 汇总 tmp/results/_summary.json（按布通率升序，最差在前，每工程一行）
import * as esbuild from 'esbuild';
import * as fs from 'fs';
import * as path from 'path';
import { fileURLToPath } from 'url';
import { spawnSync, spawn as nodeSpawn } from 'child_process';
import { STRUCT_VERSION } from './batch-dump-all.mjs';

// 与 batch-dump-all.mjs 同源的 STRUCT_VERSION（单一来源）；不匹配的 dump 会被跳过/重导
const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');

const argOf = (k, dflt) => {
	const i = process.argv.indexOf(k);
	return i >= 0 ? process.argv[i + 1] : dflt;
};
const budgetMs = Number(argOf('--budget', 240000));
const filter = process.argv.includes('--filter') ? process.argv[process.argv.indexOf('--filter') + 1] : null;
const limit = Number(argOf('--limit', 0)) || Infinity;
const redump = process.argv.includes('--redump');

const dumpsDir = path.join(ROOT, 'tmp', 'dumps');
const resultsDir = path.join(ROOT, 'tmp', 'results');

let dumps = fs.readdirSync(dumpsDir).filter(f => f.endsWith('-dump.json')).map(f => f.replace(/-dump\.json$/, ''));
if (filter) dumps = dumps.filter(n => n.includes(filter));
dumps.sort();

function cli(args, timeoutMs = 60000) {
	const r = spawnSync('lceda-pro', args, { timeout: timeoutMs, maxBuffer: 256 * 1024 * 1024, encoding: 'utf8' });
	const out = (r.stdout ?? '').trim();
	const jsonLine = out.split('\n').reverse().find(l => l.trimStart().startsWith('{'));
	let parsed = null;
	try { parsed = jsonLine ? JSON.parse(jsonLine) : null; } catch { /* 非 JSON */ }
	return { json: parsed };
}
function spawn(cmd, args) {
	return new Promise((resolve) => {
		const p = nodeSpawn(cmd, args, { stdio: 'inherit' });
		p.on('exit', () => resolve());
	});
}

// ── 离线间距审计（圆/胶囊近似，供 AI 分诊；最终口径以 EDA DRC 为准）──
function drcAudit(input, result) {
	const drc = input.drc;
	const needTT = drc.trackToTrackMil;
	const needTP = drc.trackToPadMil;
	const CELL = 96;
	const grid = new Map();
	const key = (x, y) => `${Math.floor(x / CELL)},${Math.floor(y / CELL)}`;
	const add = (x, y, item) => {
		const k = key(x, y);
		let b = grid.get(k);
		if (!b) { b = []; grid.set(k, b); }
		b.push({ x, y, ...item });
	};
	const STEP = 2.5;
	const sampleLine = (p, net) => {
		const d = Math.hypot((p.x2 ?? 0) - (p.x1 ?? 0), (p.y2 ?? 0) - (p.y1 ?? 0));
		const n = Math.max(1, Math.ceil(d / STEP));
		for (let i = 0; i <= n; i++) {
			const t = i / n;
			add((p.x1 ?? 0) + ((p.x2 ?? 0) - (p.x1 ?? 0)) * t, (p.y1 ?? 0) + ((p.y2 ?? 0) - (p.y1 ?? 0)) * t, { kind: 'trk', net });
		}
	};
	const sampleArc = (p, net) => {
		const r = p.r ?? 0, sw = p.sweep ?? 0;
		const n = Math.max(2, Math.ceil(Math.abs(sw) * r / STEP));
		for (let i = 0; i <= n; i++) {
			const a = (p.startAngle ?? 0) + (sw * i) / n;
			add((p.cx ?? 0) + r * Math.cos(a), (p.cy ?? 0) + r * Math.sin(a), { kind: 'trk', net });
		}
	};
	for (const n of result.nets) {
		for (const p of n.prims) {
			if (p.kind === 'line') sampleLine(p, n.name); else sampleArc(p, n.name);
		}
	}
	// 焊盘/过孔也入网格（圆近似：r 取外接半径）
	for (const p of input.allPads) {
		add(p.x, p.y, { kind: 'pad', net: p.net, r: p.radius });
	}
	for (const v of result.vias) {
		add(v.x, v.y, { kind: 'via', net: v.net, r: v.dia / 2 });
	}
	const pairMin = new Map();   // 'netA|netB' → {min, x, y, kinds}
	const consider = (na, nb, d, x, y, ka, kb) => {
		if (na === nb) return;
		const k = na < nb ? `${na}|${nb}` : `${nb}|${na}`;
		const kinds = na < nb ? `${ka}-${kb}` : `${kb}-${ka}`;
		const cur = pairMin.get(k);
		if (!cur || d < cur.min) pairMin.set(k, { min: d, x, y, kinds });
	};
	for (const b of grid.values()) {
		for (let i = 0; i < b.length; i++) {
			const A = b[i];
			// 同格内两两 + 右/下方向邻格（避免重复遍历）
			for (let j = i + 1; j < b.length; j++) checkPair(A, b[j]);
			const gx = Math.floor(A.x / CELL), gy = Math.floor(A.y / CELL);
			for (const [dx, dy] of [[1, 0], [0, 1], [1, 1], [1, -1]]) {
				const nb2 = grid.get(`${gx + dx},${gy + dy}`);
				if (nb2) for (const B of nb2) checkPair(A, B);
			}
		}
	}
	function checkPair(A, B) {
		if (A.net === B.net) return;
		const d = Math.hypot(A.x - B.x, A.y - B.y) - (A.kind === 'trk' ? 0 : A.r) - (B.kind === 'trk' ? 0 : B.r);
		if (d > needTP + 2) return;
		consider(A.net, B.net, d, (A.x + B.x) / 2, (A.y + B.y) / 2, A.kind, B.kind);
	}
	const violations = [];
	for (const [k, v] of pairMin) {
		const [na, nb] = k.split('|');
		const bothTrk = v.kinds === 'trk-trk';
		const need = bothTrk ? needTT : needTP;
		if (v.min < need - 0.5) violations.push({ netA: na, netB: nb, kind: v.kinds, min: Math.round(v.min * 10) / 10, need, x: Math.round(v.x), y: Math.round(v.y) });
	}
	violations.sort((a, b) => a.min - b.min);
	return { violations: violations.slice(0, 80), violationCount: violations.length };
}

// ── 接缝缺陷审计（开口/锐角/近整圆弧）──
function jointAudit(result) {
	const TAU = Math.PI * 2;
	const ptOf = (p, end) => {
		if (p.kind === 'line') return end ? { x: p.x2, y: p.y2 } : { x: p.x1, y: p.y1 };
		const a = (p.startAngle ?? 0) + (end ? (p.sweep ?? 0) : 0);
		return { x: (p.cx ?? 0) + (p.r ?? 0) * Math.cos(a), y: (p.cy ?? 0) + (p.r ?? 0) * Math.sin(a) };
	};
	const tanOf = (p, end) => {
		if (p.kind === 'line') {
			const dx = (p.x2 ?? 0) - (p.x1 ?? 0), dy = (p.y2 ?? 0) - (p.y1 ?? 0);
			const l = Math.hypot(dx, dy) || 1;
			return { x: dx / l, y: dy / l };
		}
		const a = (p.startAngle ?? 0) + (end ? (p.sweep ?? 0) : 0);
		const s = (p.sweep ?? 0) >= 0 ? 1 : -1;
		return end ? { x: Math.sin(a) * s, y: -Math.cos(a) * s } : { x: -Math.sin(a) * s, y: Math.cos(a) * s };
	};
	const angDiff = (a, b) => {
		let d = Math.abs(Math.atan2(a.y, a.x) - Math.atan2(b.y, b.x)) % TAU;
		if (d > Math.PI) d = TAU - d;
		return d * 180 / Math.PI;
	};
	const issues = [];
	let joints = 0, gaps = 0, sharp = 0, bigArcs = 0;
	for (const net of result.nets) {
		const prims = net.prims;
		if (prims.length < 2) continue;
		for (let i = 0; i < prims.length - 1; i++) {
			const a = prims[i], b = prims[i + 1];
			const ae = ptOf(a, true), bs = ptOf(b, false);
			joints++;
			const gap = Math.hypot(ae.x - bs.x, ae.y - bs.y);
			if (gap > 0.5) { gaps++; issues.push({ net: net.name, type: 'gap', mil: Math.round(gap * 10) / 10, at: [Math.round(ae.x), Math.round(ae.y)] }); }
			const turn = angDiff(tanOf(a, true), tanOf(b, false));
			if (turn > 30) { sharp++; issues.push({ net: net.name, type: 'sharp', deg: Math.round(turn), at: [Math.round(ae.x), Math.round(ae.y)] }); }
		}
		for (const p of prims) {
			if (p.kind === 'arc' && Math.abs((p.sweep ?? 0) * 180 / Math.PI) > 350) { bigArcs++; issues.push({ net: net.name, type: 'nearFullCircle' }); }
		}
	}
	return { joints, gaps, sharp, bigArcs, issues: issues.slice(0, 60) };
}

const built = await esbuild.build({
	entryPoints: ['src/router/planning-engine.ts'],
	bundle: true, format: 'esm', write: false, platform: 'neutral',
});
(globalThis).__RB_VERBOSE__ = false;
const mod = await import('data:text/javascript;base64,' + Buffer.from(built.outputFiles[0].text).toString('base64'));

const summary = [];
let idx = 0;
for (const name of dumps) {
	idx++;
	if (idx > limit) break;
	const dumpFile = path.join(dumpsDir, `${name}-dump.json`);
	let wrapped;
	try { wrapped = JSON.parse(fs.readFileSync(dumpFile, 'utf8')); }
	catch { summary.push({ case: name, status: 'dumpBroken' }); continue; }
	if (wrapped.structVersion !== STRUCT_VERSION) {
		if (redump) {
			console.error(`[${idx}/${dumps.length}] ${name}: dump 过期（${wrapped.structVersion ?? 1} ≠ ${STRUCT_VERSION}），重新导出...`);
			await spawn('node', ['tmp/batch-dump-all.mjs', '--filter', name, '--force']);
			wrapped = JSON.parse(fs.readFileSync(dumpFile, 'utf8'));
			if (wrapped.structVersion !== STRUCT_VERSION) { summary.push({ case: name, status: 'staleDump' }); continue; }
		} else {
			console.error(`[${idx}/${dumps.length}] ${name}: dump 结构版本 ${wrapped.structVersion ?? 1} ≠ ${STRUCT_VERSION}，跳过（--redump 自动重导）`);
			summary.push({ case: name, status: 'staleDump' });
			continue;
		}
	}
	const input = wrapped.input;
	input.budgetMs = budgetMs;
	const t0 = Date.now();
	let result = null;
	try {
		const eng = new mod.PlanningEngine();
		result = await eng.plan(input, { isCancelled: () => false });
	} catch (e) {
		console.error(`[${idx}/${dumps.length}] ${name}: 引擎异常 — ${String(e?.message ?? e).slice(0, 200)}`);
		summary.push({ case: name, status: 'engineError', error: String(e?.message ?? e).slice(0, 200) });
		continue;
	}
	const wallS = Math.round((Date.now() - t0) / 100) / 10;
	const routed = result.routedNets.length;
	const total = input.networks.length;
	const drc = drcAudit(input, result);
	const joints = jointAudit(result);
	const outDir = path.join(resultsDir, name);
	fs.mkdirSync(outDir, { recursive: true });
	const payload = {
		case: name,
		structVersion: STRUCT_VERSION,
		budgetMs,
		summary: {
			routed, total,
			rate: Math.round(routed / Math.max(1, total) * 1000) / 10,
			failed: result.failed,
			partial: result.partial,
			vias: result.vias.length,
			wallS,
		},
		drc,
		joints,
		nets: result.nets.map(n => ({ name: n.name, prims: n.prims })),
		routedOrder: result.routedNets,
	};
	fs.writeFileSync(path.join(outDir, 'result.json'), JSON.stringify(payload), 'utf8');
	// AI 专用微型摘要：不含任何图元坐标明细，控制在几 KB
	const digest = {
		case: name,
		routed, total,
		rate: payload.summary.rate,
		failed: result.failed,
		partial: result.partial.map(n => ({
			name: n,
			prims: result.nets.find(x => x.name === n)?.prims.length ?? 0,
		})),
		vias: result.vias.length,
		wallS,
		drcCount: drc.violationCount,
		drcTop: drc.violations.slice(0, 10),
		joints: {
			gaps: joints.gaps, sharp: joints.sharp, bigArcs: joints.bigArcs,
			issues: joints.issues.slice(0, 8),
		},
	};
	fs.writeFileSync(path.join(outDir, 'digest.json'), JSON.stringify(digest, null, 1), 'utf8');
	console.log(`[${idx}/${dumps.length}] ${name}: ${routed}/${total}（${payload.summary.rate}%）DRC违规 ${drc.violationCount} 接缝缺陷 ${joints.gaps + joints.sharp + joints.bigArcs}，${wallS}s`);
	summary.push({ case: name, routed, total, rate: payload.summary.rate, drc: drc.violationCount, joints: joints.gaps + joints.sharp + joints.bigArcs, wallS });
}

summary.sort((a, b) => (a.rate ?? 1e9) - (b.rate ?? 1e9));
fs.writeFileSync(path.join(resultsDir, '_summary.json'), JSON.stringify(summary, null, 1), 'utf8');
const done = summary.filter(s => s.routed !== undefined);
const avgRate = done.length ? Math.round(done.reduce((s, x) => s + x.rate, 0) / done.length * 10) / 10 : 0;
console.log(`[batch-route] ${done.length} 工程，均布通率 ${avgRate}%，结果 → tmp/results/（_summary.json 按布通率升序）`);
