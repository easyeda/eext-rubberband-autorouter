// 嘉立创 EDA 客户端 CLI 的公共封装（batch-dump-all / batch-route-all 共用）
//
// 导出:
//  - LCEDA_BIN  客户端可执行入口，解析顺序:
//      1. 环境变量 LCEDA_PRO_BIN（显式指定）
//      2. linux 且存在 scripts/lceda-env.sh 与 $LCEDA_PRO_HOME(默认 ~/tools/lceda-pro)/lceda-pro/lceda-pro
//         → 用 scripts/lceda-env.sh（自动拉起 Xvfb 与服务端，并追加 --no-sandbox）
//      3. 否则 'lceda-pro'（走 PATH，开发机 Windows 上的原始行为）
//  - DATA_ROOT  case/ 与 tmp/ 所在目录（这两者是 git 忽略的所有者数据，worktree 里没有）
//      优先级: argv 的 --data-root <dir> > 环境变量 RB_DATA_ROOT > 仓库根（scripts/ 的上一级）
//  - ROOT       仓库根（scripts/ 的上一级）
//  - cli(args, timeoutMs)  同步调用客户端 CLI，返回 { raw, json, status, stderr }
//      json 为 stdout 最后一行以 { 开头的 JSON 信封（解析失败为 null）；超时抛错
import * as fs from 'fs';
import * as os from 'os';
import * as path from 'path';
import { fileURLToPath } from 'url';
import { spawnSync } from 'child_process';

const SCRIPTS_DIR = path.dirname(fileURLToPath(import.meta.url));
export const ROOT = path.resolve(SCRIPTS_DIR, '..');

function resolveBin() {
	if (process.env.LCEDA_PRO_BIN) return process.env.LCEDA_PRO_BIN;
	if (process.platform === 'linux') {
		const wrapper = path.join(SCRIPTS_DIR, 'lceda-env.sh');
		const home = process.env.LCEDA_PRO_HOME || path.join(os.homedir(), 'tools', 'lceda-pro');
		if (fs.existsSync(wrapper) && fs.existsSync(path.join(home, 'lceda-pro', 'lceda-pro'))) return wrapper;
	}
	return 'lceda-pro';
}
export const LCEDA_BIN = resolveBin();

function resolveDataRoot() {
	const i = process.argv.indexOf('--data-root');
	const v = i >= 0 ? process.argv[i + 1] : process.env.RB_DATA_ROOT;
	if (i >= 0 && !v) throw new Error('--data-root 缺少目录参数');
	return v ? path.resolve(v) : ROOT;
}
export const DATA_ROOT = resolveDataRoot();

export function cli(args, timeoutMs = 60000) {
	const r = spawnSync(LCEDA_BIN, args, { timeout: timeoutMs, maxBuffer: 256 * 1024 * 1024, encoding: 'utf8' });
	if (r.error && r.error.code === 'ETIMEDOUT') throw new Error(`CLI 超时: ${args.join(' ')}`);
	if (r.error && r.error.code === 'ENOENT') throw new Error(`找不到客户端 CLI: ${LCEDA_BIN}（可用环境变量 LCEDA_PRO_BIN 指定）`);
	const out = (r.stdout ?? '').trim();
	const jsonLine = out.split('\n').reverse().find(l => l.trimStart().startsWith('{'));
	let parsed = null;
	try { parsed = jsonLine ? JSON.parse(jsonLine) : null; } catch { /* 非 JSON 输出 */ }
	return { raw: out, json: parsed, status: r.status, stderr: (r.stderr ?? '').trim() };
}
