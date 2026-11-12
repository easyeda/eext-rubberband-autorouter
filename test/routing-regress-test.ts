/**
 * 布线回归测试（小板，离线，< 30 s）
 * 运行方式：npx tsx test/routing-regress-test.ts
 *
 * 用 scripts/bench-offline.mjs 的阶段 A（PlanningEngine.plan()，合成板，固定种子）跑两块小板，
 * 引擎自然结束（预算 1800000 ms 远大于耗时，结果确定、不受墙钟截断影响）。断言：
 *   - 进程退出码 0（引擎无异常）
 *   - 严格边到边审计 drcEdge 违规 = 0，宽松 drcAudit 违规 = 0
 *   - 布通网络数 / 布通焊盘数不低于观测基线
 * 不断言 hash 与耗时（只打印）；hash 变化请用 bench/route-bench.mjs 对照。
 * 观测基线（本机，seed 固定）：nets=8 seed=1 → 8/8 网络、24/24 焊盘、~6 s；
 *                              nets=10 seed=3 → 10/10 网络、24/24 焊盘、~9 s。
 */
import { spawnSync } from 'child_process';
import * as fs from 'fs';
import * as os from 'os';
import * as path from 'path';

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

const ROOT = path.resolve(typeof __dirname === 'string' ? __dirname : path.join(process.cwd(), 'test'), '..');
const BENCH = path.join(ROOT, 'scripts', 'bench-offline.mjs');

interface Case { nets: number; seed: number; minNets: number; minPads: number }
const CASES: Case[] = [
	{ nets: 8, seed: 1, minNets: 8, minPads: 24 },
	{ nets: 10, seed: 3, minNets: 10, minPads: 24 },
];

for (const c of CASES) {
	console.log(`\n▶ 合成板 nets=${c.nets} seed=${c.seed}（2 层，预算 1800000 ms，自然结束）`);
	const outFile = path.join(os.tmpdir(), `routing-regress-${process.pid}-${c.nets}-${c.seed}.json`);
	const t0 = Date.now();
	const r = spawnSync(process.execPath, [
		BENCH, '--phases', 'a', '--runs', '1', '--nets', String(c.nets), '--seed', String(c.seed),
		'--budget', '1800000', '--layers', '2', '--out', outFile,
	], { cwd: ROOT, encoding: 'utf8', maxBuffer: 64 * 1024 * 1024, timeout: 55_000 });
	const wall = Date.now() - t0;

	assert(r.status === 0, `bench-offline 退出码 0（实际 ${r.status}${r.error ? ` ${r.error.message}` : ''}）`);
	let run: any = null;
	try {
		const doc = JSON.parse(fs.readFileSync(outFile, 'utf8'));
		if (doc.error) console.error(String(doc.error).slice(0, 600));
		run = doc.phaseA?.runs?.[0] ?? null;
	} catch { /* 下面断言会报告 */ }
	try { fs.unlinkSync(outFile); } catch { /* ignore */ }
	assert(run !== null, '读到阶段 A 结果 JSON');
	if (!run) {
		if (r.stderr) console.error(String(r.stderr).slice(-800));
		continue;
	}
	console.log(`     hash=${run.resultHash} engine=${run.wallMs}ms total=${wall}ms vias=${run.vias} len=${Math.round(run.lengthMil)}mil`);
	assert(run.drcEdge.violationCount === 0, `严格 drcEdge 违规 = 0（实际 ${run.drcEdge.violationCount}）`);
	assert(run.drc.violationCount === 0, `宽松 drc 违规 = 0（实际 ${run.drc.violationCount}）`);
	assert(run.nets.routed >= c.minNets, `布通网络 ${run.nets.routed}/${run.nets.total} >= ${c.minNets}`);
	assert(run.pads.routed >= c.minPads, `布通焊盘 ${run.pads.routed}/${run.pads.total} >= ${c.minPads}`);
}

console.log(`\n${'─'.repeat(50)}`);
console.log(`测试结果：${passed} 通过，${failed} 失败`);
if (failed > 0) {
	process.exit(1);
}
