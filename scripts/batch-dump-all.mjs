// 批量导出 case/ 全部 PCB 的布线初始数据（离线 dump，免去每次启动 EDA CLI）
// 用法: node scripts/batch-dump-all.mjs [--force] [--filter <子串>] [--data-root <目录>]
//  - --data-root（或环境变量 RB_DATA_ROOT）: case/ 与 tmp/ 所在目录，默认仓库根；
//    它们是 git 忽略的所有者数据，在 git worktree 里跑时必须指向主检出目录
//  - 环境变量 LCEDA_PRO_BIN: 指定客户端 CLI；缺省时 linux 服务器走 scripts/lceda-env.sh（自动拉起 Xvfb+服务端），
//    否则用 PATH 里的 lceda-pro（见 scripts/lceda-cli.mjs）
//  - 每次运行都先无条件重建 scripts/eda-dump-bundle.mjs（node scripts/build-eda-dump.mjs），失败则中止
//  - 客户端是 scripts/lceda-env.sh 封装时，首次导出前先 `start` 预热（冷启动 Xvfb+服务端最长约 105s，
//    超过 open 的 120s 超时余量，故单独用 200s 超时预热）；其它客户端（PATH 里的 lceda-pro 等）不预热
//  - 已有 dump 且 structVersion 匹配 → 跳过（--force 强制重导）
//  - PlanningInput 结构变化时把 STRUCT_VERSION 加一，下次运行自动重导全部
//  - 结果: tmp/dumps/<case名>-dump.json（含 structVersion 包装），tmp/dumps/_dump-summary.json
import * as fs from 'fs';
import * as path from 'path';
import { fileURLToPath } from 'url';
import { spawnSync } from 'child_process';
import { STRUCT_VERSION } from './struct-version.mjs';
import { cli, LCEDA_BIN, ROOT, DATA_ROOT } from './lceda-cli.mjs';

const SCRIPTS_DIR = path.dirname(fileURLToPath(import.meta.url));

const force = process.argv.includes('--force');
const filterIdx = process.argv.indexOf('--filter');
const filter = filterIdx >= 0 ? process.argv[filterIdx + 1] : null;

// dump 驱动 bundle：每次运行都无条件重建（esbuild 很快）。
// 不能靠 mtime 判断新旧：bundle 内联了 src/router/*.ts，且 git checkout 不保证 mtime 单调；
// 构建失败则中止，避免用陈旧 bundle 导出。
const bundleFile = path.join(SCRIPTS_DIR, 'eda-dump-bundle.mjs');
console.log('[batch-dump] 重建 eda-dump-bundle.mjs ...');
{
	const b = spawnSync(process.execPath, [path.join(SCRIPTS_DIR, 'build-eda-dump.mjs')], { cwd: ROOT, stdio: 'inherit' });
	if (b.status !== 0) {
		console.error('[batch-dump] bundle 重建失败，中止（请手动运行 node scripts/build-eda-dump.mjs 排查）');
		process.exit(1);
	}
}
// 传给 EDA 内脚本的 bundle 绝对路径（正斜杠，Windows/Linux 通用）
const bundlePath = bundleFile.replace(/\\/g, '/');

const caseDir = path.join(DATA_ROOT, 'case');
const dumpsDir = path.join(DATA_ROOT, 'tmp', 'dumps');
if (!fs.existsSync(caseDir)) {
	console.error(`[batch-dump] 找不到 ${caseDir}（可用 --data-root 或 RB_DATA_ROOT 指定 case/ 所在目录）`);
	process.exit(1);
}
fs.mkdirSync(dumpsDir, { recursive: true });

const cases = fs.readdirSync(caseDir).filter(n => {
	const p = path.join(caseDir, n);
	if (!fs.statSync(p).isDirectory()) return false;
	if (filter && !n.includes(filter)) return false;
	return fs.readdirSync(p).some(f => f.endsWith('.eprj3'));
});
console.log(`[batch-dump] 共 ${cases.length} 个工程${filter ? `（filter=${filter}）` : ''}`);

// 冷启动预热：封装脚本的 `start` 子命令幂等，已就绪时几乎立即返回
let warmed = false;
function warmUp() {
	if (warmed) return;
	warmed = true;
	if (path.basename(LCEDA_BIN) !== 'lceda-env.sh') return;
	console.log('[batch-dump] 预热客户端（lceda-env.sh start，最长 200s）...');
	let r;
	try { r = cli(['start'], 200000); } catch (e) { console.error(`[batch-dump] 预热失败: ${e.message}`); process.exit(1); }
	if (r.status !== 0) {
		console.error(`[batch-dump] 预热失败（退出码 ${r.status}）: ${(r.stderr || r.raw).slice(0, 300)}`);
		process.exit(1);
	}
	console.log(`[batch-dump] ${(r.raw || r.stderr).split('\n').pop()}`);
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

	warmUp();
	const srcDir = path.join(caseDir, name);
	const eprj = fs.readdirSync(srcDir).find(f => f.endsWith('.eprj3'));
	// 直接打开原目录会加载不出 PCB 文档（历史坑），必须先复制副本
	const workDir = path.join(DATA_ROOT, 'tmp', 'testwork-dump', name);
	fs.rmSync(workDir, { recursive: true, force: true });
	fs.mkdirSync(path.dirname(workDir), { recursive: true });
	fs.cpSync(srcDir, workDir, { recursive: true });
	const workEprj = path.join(workDir, eprj).replace(/\\/g, '/');

	const t0 = Date.now();
	let sessionId = null;
	try {
		const open = cli(['open', '--path', workEprj, '--headless', 'true'], 120000);
		sessionId = open.json?.value?.sessionId;
		if (!sessionId) throw new Error(`打开失败: ${(open.raw || open.stderr).slice(0, 300)}`);
		const invoke = cli([
			'invoke', '--session', sessionId, '--ext-uuid', 'eda',
			'--code-file', path.join(SCRIPTS_DIR, 'cli-dump62.js'),
			'--args', JSON.stringify({ mode: 'compute', bundlePath }),
			'--timeout', '600000',
		], 630000);
		if (!invoke.json?.ok) throw new Error(`invoke 失败: ${(invoke.json ? JSON.stringify(invoke.json).slice(0, 500) : invoke.raw.slice(0, 500))}`);
		const meta = invoke.json.value ?? {};
		if (!meta.token || !meta.chunkCount) throw new Error(`返回缺少分块元数据: ${JSON.stringify(meta).slice(0, 300)}`);
		// 分块取回（单次响应上限 8MB）
		let json = '';
		for (let i = 0; i < meta.chunkCount; i++) {
			const part = cli([
				'invoke', '--session', sessionId, '--ext-uuid', 'eda',
				'--code-file', path.join(SCRIPTS_DIR, 'cli-dump62.js'),
				'--args', JSON.stringify({ mode: 'chunk', token: meta.token, i }),
				'--timeout', '120000',
			], 150000);
			if (!part.json?.ok) throw new Error(`块 ${i}/${meta.chunkCount} 取回失败: ${JSON.stringify(part.json).slice(0, 300)}`);
			json += part.json.value.data;
			if (part.json.value.done) break;
		}
		const v = JSON.parse(json);
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
		console.log(`[${done}/${cases.length}] ${name}: 焊盘 ${v.totalPads}（注入无网络 ${v.added}）网络 ${v.nets?.length}（${Math.round((Date.now() - t0) / 100) / 10}s，${meta.chunkCount} 块）`);
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
console.log(`[batch-dump] 完成：${ok}/${cases.length} 可用 → ${dumpsDir}（版本 v${STRUCT_VERSION}）`);
