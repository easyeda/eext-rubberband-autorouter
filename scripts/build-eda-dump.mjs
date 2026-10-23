// 构建 EDA 内运行的 dump 驱动 bundle：scripts/eda-dump-driver.ts → scripts/eda-dump-bundle.mjs
// 用法: node scripts/build-eda-dump.mjs
import { build } from 'esbuild';
import { fileURLToPath } from 'url';
import * as path from 'path';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
await build({
	entryPoints: [path.join(ROOT, 'scripts', 'eda-dump-driver.ts')],
	outfile: path.join(ROOT, 'scripts', 'eda-dump-bundle.mjs'),
	bundle: true,
	format: 'esm',
	platform: 'neutral',
	legalComments: 'none',
	banner: { js: 'const __edaShim = (globalThis.eda ?? null);' },
	// drc-rules / planning-engine 等把 `eda` 当全局用：neutral 平台下声明为外部全局
	define: {},
	footer: { js: '' },
});
console.log('[build-eda-dump] scripts/eda-dump-bundle.mjs 已生成');
