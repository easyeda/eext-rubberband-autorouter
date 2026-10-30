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
import { STRUCT_VERSION } from './struct-version.mjs';

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
// --only a|b|c：只跑指定工程（按名字精确匹配，用于重点复测）
const onlyArg = process.argv.includes('--only') ? process.argv[process.argv.indexOf('--only') + 1] : null;
const onlySet = onlyArg ? new Set(onlyArg.split('|')) : null;
// --reaudit：不重跑引擎，用既有 result.json + dump 重算 DRC/接缝审计并刷新 digest
const reaudit = process.argv.includes('--reaudit');

const dumpsDir = path.join(ROOT, 'tmp', 'dumps');
const resultsDir = path.join(ROOT, 'tmp', 'results');

let dumps = fs.readdirSync(dumpsDir).filter(f => f.endsWith('-dump.json')).map(f => f.replace(/-dump\.json$/, ''));
if (filter) dumps = dumps.filter(n => n.includes(filter));
if (onlySet) dumps = dumps.filter(n => onlySet.has(n));
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

import { drcAudit, jointAudit } from './audit-lib.mjs';

const built = await esbuild.build({
	entryPoints: ['src/router/planning-engine.ts'],
	bundle: true, format: 'esm', write: false, platform: 'neutral',
});
(globalThis).__RB_VERBOSE__ = false;
const mod = await import('data:text/javascript;base64,' + Buffer.from(built.outputFiles[0].text).toString('base64'));

const summary = [];
let idx = 0;
if (reaudit) {
	// 审计口径迭代后离线刷新全部 digest（引擎结果不动，秒级/工程）
	for (const name of dumps) {
		idx++;
		const dir = path.join(resultsDir, name);
		const resultFile = path.join(dir, 'result.json');
		if (!fs.existsSync(resultFile)) { summary.push({ case: name, status: 'noResult' }); continue; }
		const result = JSON.parse(fs.readFileSync(resultFile, 'utf8'));
		const dump = JSON.parse(fs.readFileSync(path.join(dumpsDir, `${name}-dump.json`), 'utf8'));
		const input = dump.input ?? dump;
		const drc = drcAudit(input, result, mod.polygonToRect);
		const joints = jointAudit(result);
		const digest = {
			case: name,
			routed: result.summary.routed, total: result.summary.total,
			rate: result.summary.rate,
			failed: result.summary.failed,
			partial: (result.summary.partial ?? []).map(n => ({ name: n, prims: result.nets.find(x => x.name === n)?.prims.length ?? 0 })),
			vias: (result.vias ?? []).length,
			wallS: result.summary.wallS,
			drcCount: drc.violationCount,
			drcTop: drc.violations.slice(0, 10),
			joints: { gaps: joints.gaps, sharp: joints.sharp, bigArcs: joints.bigArcs, issues: joints.issues.slice(0, 8) },
		};
		result.drc = drc;
		result.joints = joints;
		fs.writeFileSync(resultFile, JSON.stringify(result), 'utf8');
		fs.writeFileSync(path.join(dir, 'digest.json'), JSON.stringify(digest, null, 1), 'utf8');
		console.log(`[${idx}/${dumps.length}] ${name}: DRC ${drc.violationCount} 接缝 ${joints.gaps + joints.sharp + joints.bigArcs}`);
		summary.push({ case: name, routed: result.summary.routed, total: result.summary.total, rate: result.summary.rate, drc: drc.violationCount, joints: joints.gaps + joints.sharp + joints.bigArcs, wallS: result.summary.wallS });
	}
	summary.sort((a, b) => (a.rate ?? 1e9) - (b.rate ?? 1e9));
	fs.writeFileSync(path.join(resultsDir, '_summary.json'), JSON.stringify(summary, null, 1), 'utf8');
	console.log(`[reaudit] ${summary.length} 工程审计已刷新（层感知 + 矩形精复核口径）`);
	process.exit(0);
}
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
	// 与 EDA 侧 clearExistingTraces 对齐：待布网络的既有走线在布线前会被清理，
	// 离线把这类静态障碍剔除，否则脏板会把所有网络互相封死（实测 1/14 vs 9/14）
	{
		const routed = new Set(input.networks.map(n => n.name));
		const before = input.staticObstacles.length;
		input.staticObstacles = input.staticObstacles.filter(o =>
			!(o.group?.startsWith('line:') && o.net && routed.has(o.net)));
		if (input.staticObstacles.length < before) {
			console.log(`  [clean] ${name}: 移除 ${before - input.staticObstacles.length} 个旧走线障碍（重新布线语义）`);
		}
	}
	const t0 = Date.now();
	let result = null;
	const padStats = new Map();
	try {
		const eng = new mod.PlanningEngine();
		const engineLog = [];
		result = await eng.plan(input, {
			isCancelled: () => false,
			onNetUpdate: (name, _prims, _vias, _ok, pads) => { if (pads) padStats.set(name, pads); },
			onLog: (m, level) => {
				engineLog.push(`[${level}] ${m}`);
				// 只回显关键行，避免刷屏；完整日志落盘到结果目录
				if (/\[耗时\]|未找到路径|网格兜底|hoist|接受新布局|时间预算/.test(m)) console.log(`    ${m}`);
			},
		});
		try {
			const dir = path.join(resultsDir, name);
			fs.mkdirSync(dir, { recursive: true });
			fs.writeFileSync(path.join(dir, 'engine.log'), engineLog.join('\n'), 'utf8');
		} catch { /* 日志落盘失败不影响结果 */ }
	} catch (e) {
		console.error(`[${idx}/${dumps.length}] ${name}: 引擎异常 — ${String(e?.message ?? e).slice(0, 200)}`);
		summary.push({ case: name, status: 'engineError', error: String(e?.message ?? e).slice(0, 200) });
		continue;
	}
	const wallS = Math.round((Date.now() - t0) / 100) / 10;
	const routed = result.routedNets.length;
	const total = input.networks.length;
	const drc = drcAudit(input, result, mod.polygonToRect);
	let padsTotal = 0, padsRouted = 0;
	for (const n of input.networks) {
		const st = padStats.get(n.name);
		padsTotal += st ? st.total : n.pads.length;
		padsRouted += st ? st.routed : 0;
	}
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
			padRouted: padsRouted,
			padTotal: padsTotal,
			padRate: Math.round((padsRouted / Math.max(1, padsTotal)) * 1000) / 10,
			wallS,
		},
		drc,
		joints,
		nets: result.nets.map(n => ({ name: n.name, prims: n.prims })),
		vias: result.vias,
		routedOrder: result.routedNets,
	};
	fs.writeFileSync(path.join(outDir, 'result.json'), JSON.stringify(payload), 'utf8');
	// AI 专用微型摘要：不含任何图元坐标明细，控制在几 KB
	const digest = {
		case: name,
		routed, total,
		rate: payload.summary.rate,
		padRouted: padsRouted,
		padTotal: padsTotal,
		padRate: payload.summary.padRate,
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
	console.log(`[${idx}/${dumps.length}] ${name}: ${routed}/${total}（${payload.summary.rate}%）焊盘 ${padsRouted}/${padsTotal}（${payload.summary.padRate}%）DRC违规 ${drc.violationCount} 接缝缺陷 ${joints.gaps + joints.sharp + joints.bigArcs}，${wallS}s`);
	summary.push({ case: name, routed, total, rate: payload.summary.rate, drc: drc.violationCount, joints: joints.gaps + joints.sharp + joints.bigArcs, wallS });
}

summary.sort((a, b) => (a.rate ?? 1e9) - (b.rate ?? 1e9));
fs.writeFileSync(path.join(resultsDir, '_summary.json'), JSON.stringify(summary, null, 1), 'utf8');
const done = summary.filter(s => s.routed !== undefined);
const avgRate = done.length ? Math.round(done.reduce((s, x) => s + x.rate, 0) / done.length * 10) / 10 : 0;
console.log(`[batch-route] ${done.length} 工程，均布通率 ${avgRate}%，结果 → tmp/results/（_summary.json 按布通率升序）`);
