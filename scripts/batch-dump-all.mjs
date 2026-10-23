// 批量导出 case/ 全部 PCB 的布线初始数据（离线 dump，免去每次启动 EDA CLI）
// 用法: node scripts/batch-dump-all.mjs [--force] [--filter <子串>]
//  - 已有 dump 且 structVersion 匹配 → 跳过（--force 强制重导）
//  - PlanningInput 结构变化时把 STRUCT_VERSION 加一，下次运行自动重导全部
//  - 结果: tmp/dumps/<case名>-dump.json（含 structVersion 包装），tmp/dumps/_dump-summary.json
import * as fs from 'fs';
import * as path from 'path';
import { fileURLToPath } from 'url';
import { spawnSync } from 'child_process';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
// PlanningInput 结构版本：字段/口径变化（如新增 padShapes 口径、网络字段改名）时 +1
export const STRUCT_VERSION = 2;

const force = process.argv.includes('--force');
const filterIdx = process.argv.indexOf('--filter');
const filter = filterIdx >= 0 ? process.argv[filterIdx + 1] : null;

const caseDir = path.join(ROOT, 'case');
const dumpsDir = path.join(ROOT, 'tmp', 'dumps');
fs.mkdirSync(dumpsDir, { recursive: true });

const cases = fs.readdirSync(caseDir).filter(n => {
	const p = path.join(caseDir, n);
	if (!fs.statSync(p).isDirectory()) return false;
	if (filter && !n.includes(filter)) return false;
	return fs.readdirSync(p).some(f => f.endsWith('.eprj3'));
});
console.log(`[batch-dump] 共 ${cases.length} 个工程${filter ? `（filter=${filter}）` : ''}`);

function cli(args, timeoutMs = 60000) {
	const r = spawnSync('lceda-pro', args, { timeout: timeoutMs, maxBuffer: 256 * 1024 * 1024, encoding: 'utf8' });
	if (r.error && r.error.code === 'ETIMEDOUT') throw new Error(`CLI 超时: ${args.join(' ')}`);
	const out = (r.stdout ?? '').trim();
	const jsonLine = out.split('\n').reverse().find(l => l.trimStart().startsWith('{'));
	let parsed = null;
	try { parsed = jsonLine ? JSON.parse(jsonLine) : null; } catch { /* 非 JSON 输出 */ }
	return { raw: out, json: parsed, status: r.status, stderr: (r.stderr ?? '').trim() };
}

const summary = [];
let done = 0;
for (const name of cases) {
	done++;
	const outFile = path.join(dumpsDir, `${name}-dump.json`);
	if (!force && fs.existsSync(outFile)) {
		try {
			const old = JSON.parse(fs.readFileSync(outFile, 'utf8'));
			if (old.structVersion === STRUCT_VERSION) {
				console.log(`[${done}/${cases.length}] ${name}: 缓存有效，跳过`);
				summary.push({ case: name, status: 'cached', pads: old.totalPads, nets: old.nets?.length });
				continue;
			}
			console.log(`[${done}/${cases.length}] ${name}: 结构版本过期（${old.structVersion ?? 1} ≠ ${STRUCT_VERSION}），重导`);
		} catch { console.log(`[${done}/${cases.length}] ${name}: dump 损坏，重导`); }
	}

	const srcDir = path.join(caseDir, name);
	const eprj = fs.readdirSync(srcDir).find(f => f.endsWith('.eprj3'));
	// 直接打开原目录会加载不出 PCB 文档（历史坑），必须先复制副本
	const workDir = path.join(ROOT, 'tmp', 'testwork-dump', name);
	fs.rmSync(workDir, { recursive: true, force: true });
	fs.mkdirSync(path.dirname(workDir), { recursive: true });
	fs.cpSync(srcDir, workDir, { recursive: true });
	const workEprj = path.join(workDir, eprj).replace(/\\/g, '/');

	const t0 = Date.now();
	let sessionId = null;
	try {
		const open = cli(['open', '--path', workEprj, '--headless', 'true'], 120000);
		sessionId = open.json?.value?.sessionId;
		if (!sessionId) throw new Error(`打开失败: ${open.raw.slice(0, 300)}`);
		const invoke = cli([
			'invoke', '--session', sessionId, '--ext-uuid', 'eda',
			'--code-file', path.join(ROOT, 'scripts', 'cli-dump62.js'),
			'--timeout', '300000',
		], 330000);
		if (!invoke.json?.ok) throw new Error(`invoke 失败: ${(invoke.json ? JSON.stringify(invoke.json).slice(0, 500) : invoke.raw.slice(0, 500))}`);
		const v = invoke.json.value ?? {};
		if (!v.input) throw new Error(`返回缺少 input`);
		const wrapped = {
			structVersion: STRUCT_VERSION,
			dumpedAt: new Date().toISOString(),
			caseName: name,
			totalPads: v.totalPads,
			added: v.added,
			nets: v.nets,
			input: v.input,
		};
		fs.writeFileSync(outFile, JSON.stringify(wrapped, null, 1), 'utf8');
		console.log(`[${done}/${cases.length}] ${name}: 焊盘 ${v.totalPads}（注入无网络 ${v.added}）网络 ${v.nets?.length}（${Math.round((Date.now() - t0) / 100) / 10}s）`);
		summary.push({ case: name, status: 'dumped', pads: v.totalPads, nets: v.nets?.length, sec: Math.round((Date.now() - t0) / 1000) });
	} catch (e) {
		console.error(`[${done}/${cases.length}] ${name}: 失败 — ${String(e?.message ?? e).slice(0, 300)}`);
		summary.push({ case: name, status: 'failed', error: String(e?.message ?? e).slice(0, 300) });
	} finally {
		if (sessionId) { try { cli(['session', 'close', '--session', sessionId, '--destroy'], 60000); } catch { /* 已退出 */ } }
		fs.rmSync(workDir, { recursive: true, force: true });
	}
}

fs.writeFileSync(path.join(dumpsDir, '_dump-summary.json'), JSON.stringify(summary, null, 1), 'utf8');
const ok = summary.filter(s => s.status !== 'failed').length;
console.log(`[batch-dump] 完成：${ok}/${cases.length} 可用 → tmp/dumps/（版本 v${STRUCT_VERSION}）`);
