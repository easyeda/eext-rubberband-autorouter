// 逐字段比较两个布线 dump 的 input（例如 live CLI 导出 vs 离线 epcb2 转换结果）
//
// 用法: node scripts/compare-dumps.mjs <a.json> <b.json> [--tol 0.001] [--match-tol 1] [--ignore-id]
//  - a 视为基准（live / 真值），b 为被检对象；输出里 "仅 a"=a 有 b 无，"仅 b" 相反
//  - --tol        数值比较容差（绝对值，默认 0.001）
//  - --match-tol  焊盘按「同层 + 位置」配对时允许的最大距离（默认 1 mil，只用于配对，不代表字段相等）
//  - --ignore-id  不比较焊盘 id 字符串（两端 id 约定不同时用）
//  - 退出码: 0 = 容差内完全一致；1 = 有差异；2 = 参数/读取错误
//
// 比较范围（均只看 input）: 顶层键、drc、copperLayers、options、allPads（配对 + 逐字段 + 顺序/index）、
// networks（顺序、逐网焊盘数、逐网焊盘序列）、staticObstacles（按 group/kind/net/layer 分桶后比几何）、
// padShapes（键集合 + 内容）。每类差异只打印计数和最多 5 个例子，绝不输出整份文件。
import * as fs from 'fs';

const argv = process.argv.slice(2);
const flagVal = (k, def) => {
	const i = argv.indexOf(k);
	if (i < 0) return def;
	const v = Number(argv[i + 1]);
	if (!Number.isFinite(v)) { console.error(`${k} 需要数值参数`); process.exit(2); }
	return v;
};
const TOL = flagVal('--tol', 0.001);
const MATCH_TOL = flagVal('--match-tol', 1);
const IGNORE_ID = argv.includes('--ignore-id');
const valueFlags = new Set(['--tol', '--match-tol']);
const files = argv.filter((a, i) => !a.startsWith('--') && !valueFlags.has(argv[i - 1]));
if (files.length !== 2) {
	console.error('用法: node scripts/compare-dumps.mjs <a.json> <b.json> [--tol 0.001] [--match-tol 1] [--ignore-id]');
	process.exit(2);
}

function load(file) {
	let d;
	try { d = JSON.parse(fs.readFileSync(file, 'utf8').replace(/^﻿/, '')); }
	catch (e) { console.error(`读取失败 ${file}: ${e.message}`); process.exit(2); }
	const inp = d.input ?? d;
	if (!Array.isArray(inp.allPads)) { console.error(`${file}: 不是 dump（缺少 input.allPads）`); process.exit(2); }
	return { wrapper: d, input: inp };
}
const A = load(files[0]).input;
const B = load(files[1]).input;

// ── 差异收集：class → { count, examples[] } ──
const diffs = new Map();
const MAX_EX = 5;
function add(cls, example) {
	let d = diffs.get(cls);
	if (!d) { d = { count: 0, examples: [] }; diffs.set(cls, d); }
	d.count++;
	if (d.examples.length < MAX_EX) d.examples.push(example);
}
const short = (v) => {
	const s = JSON.stringify(v);
	return s === undefined ? 'undefined' : (s.length > 90 ? s.slice(0, 87) + '...' : s);
};

/** 深度比较；数值按容差，其余严格相等。返回差异路径数组 */
function deepDiff(a, b, tol, p = '') {
	if (typeof a === 'number' && typeof b === 'number') return Math.abs(a - b) <= tol ? [] : [{ path: p, a, b }];
	if (a === b) return [];
	if (Array.isArray(a) && Array.isArray(b)) {
		if (a.length !== b.length) return [{ path: p + '.length', a: a.length, b: b.length }];
		return a.flatMap((x, i) => deepDiff(x, b[i], tol, `${p}[${i}]`));
	}
	if (a && b && typeof a === 'object' && typeof b === 'object' && !Array.isArray(a) && !Array.isArray(b)) {
		const out = [];
		for (const k of new Set([...Object.keys(a), ...Object.keys(b)])) {
			if (!(k in a)) out.push({ path: `${p}.${k}`, a: undefined, b: b[k] });
			else if (!(k in b)) out.push({ path: `${p}.${k}`, a: a[k], b: undefined });
			else out.push(...deepDiff(a[k], b[k], tol, `${p}.${k}`));
		}
		return out;
	}
	return [{ path: p, a, b }];
}
const fmtDiff = (d) => `${d.path || '(根)'}: a=${short(d.a)} b=${short(d.b)}`;

// ── 1. 顶层键 / copperLayers / options / drc ──
{
	const ka = Object.keys(A).sort(), kb = Object.keys(B).sort();
	if (ka.join() !== kb.join()) add('顶层键', `a=[${ka}] b=[${kb}]`);
	for (const d of deepDiff(A.copperLayers, B.copperLayers, TOL)) add('copperLayers', fmtDiff(d));
	for (const d of deepDiff(A.options, B.options, TOL)) add('options', fmtDiff(d));
	for (const d of deepDiff(A.drc, B.drc, TOL)) add('drc', fmtDiff(d));
	// 容差内相等但位级不等的 drc 值单独提示（说明换算链不同）
	let bitDiff = 0;
	for (const k of Object.keys(A.drc ?? {})) if (A.drc[k] !== B.drc?.[k]) bitDiff++;
	if (bitDiff && !diffs.has('drc')) add('drc（位级不等，容差内相等）', `${bitDiff} 个键的 double 值不完全相同（换算链/舍入不同）`);
}

// ── 2. 焊盘配对（同层 + 最近距离 ≤ MATCH_TOL，一对一） ──
const padLabel = (p) => `${p.id}@(${p.x},${p.y})L${p.layer}`;
const cell = Math.max(MATCH_TOL, 1e-6);
const grid = new Map();
const gk = (layer, cx, cy) => `${layer}|${cx}|${cy}`;
B.allPads.forEach((p, j) => {
	const k = gk(p.layer, Math.floor(p.x / cell), Math.floor(p.y / cell));
	(grid.get(k) ?? grid.set(k, []).get(k)).push(j);
});
const usedB = new Set();
const aToB = new Array(A.allPads.length).fill(-1);
A.allPads.forEach((p, i) => {
	const cx = Math.floor(p.x / cell), cy = Math.floor(p.y / cell);
	let best = -1, bd = Infinity;
	for (let dx = -1; dx <= 1; dx++) for (let dy = -1; dy <= 1; dy++) {
		for (const j of grid.get(gk(p.layer, cx + dx, cy + dy)) ?? []) {
			if (usedB.has(j)) continue;
			const q = B.allPads[j];
			const d = Math.hypot(p.x - q.x, p.y - q.y);
			if (d <= MATCH_TOL && d < bd) { bd = d; best = j; }
		}
	}
	if (best >= 0) { usedB.add(best); aToB[i] = best; }
});
const bToA = new Array(B.allPads.length).fill(-1);
aToB.forEach((j, i) => { if (j >= 0) bToA[j] = i; });
A.allPads.forEach((p, i) => { if (aToB[i] < 0) add('焊盘 仅 a（未配对）', padLabel(p)); });
B.allPads.forEach((p, j) => { if (bToA[j] < 0) add('焊盘 仅 b（未配对）', padLabel(p)); });
if (A.allPads.length !== B.allPads.length) add('焊盘总数', `a=${A.allPads.length} b=${B.allPads.length}`);

// 逐字段（x/y 也按容差比，舍入差会在这里出现）
A.allPads.forEach((p, i) => {
	const q = B.allPads[aToB[i]];
	if (!q) return;
	for (const k of new Set([...Object.keys(p), ...Object.keys(q)])) {
		if (k === 'id' && IGNORE_ID) continue;
		const d = deepDiff(p[k], q[k], TOL, k);
		for (const x of d) add(`焊盘字段 ${k}`, `${padLabel(p)} ${fmtDiff(x)}`);
	}
});

// 顺序：a 的第 i 个焊盘配到 b 的第几个
{
	let moved = 0;
	const ex = [];
	aToB.forEach((j, i) => { if (j >= 0 && j !== i) { moved++; if (ex.length < MAX_EX) ex.push(`a#${i}→b#${j}`); } });
	if (moved) { for (const e of ex) add('焊盘顺序（allPads 下标不同）', e); diffs.get('焊盘顺序（allPads 下标不同）').count = moved; }
}

// ── 3. networks ──
{
	const na = A.networks.map(n => n.name), nb = B.networks.map(n => n.name);
	const sa = new Set(na), sb = new Set(nb);
	for (const n of na) if (!sb.has(n)) add('网络 仅 a', n);
	for (const n of nb) if (!sa.has(n)) add('网络 仅 b', n);
	// 网络顺序：只比较两边共有的网络的相对顺序
	{
		const common = na.filter(n => sb.has(n));
		const commonB = nb.filter(n => sa.has(n));
		let moved = 0;
		const ex = [];
		common.forEach((n, i) => { if (commonB[i] !== n) { moved++; if (ex.length < MAX_EX) ex.push(`位置 ${i}: a=${n} b=${commonB[i]}`); } });
		if (moved) { for (const e of ex) add('网络顺序', e); diffs.get('网络顺序').count = moved; }
	}
	const mapB = new Map(B.networks.map(n => [n.name, n]));
	const padKey = (q) => `${q.id}|${q.x}|${q.y}|${q.layer}`;
	const keyA = new Map(A.allPads.map((q, i) => [padKey(q), i]));
	const keyB = new Map(B.allPads.map((q, i) => [padKey(q), i]));
	for (const n of A.networks) {
		const m = mapB.get(n.name);
		if (!m) continue;
		if (n.pads.length !== m.pads.length) { add('网络焊盘数', `${n.name}: a=${n.pads.length} b=${m.pads.length}`); continue; }
		// 逐网焊盘序列：用 x/y/layer 配到 allPads 下标，再比较两边序列
		const idxIn = (side, pads) => pads.map(pp => {
			const i = (side === 'a' ? keyA : keyB).get(padKey(pp));
			if (i === undefined) return -1;
			return side === 'a' ? aToB[i] : i;
		});
		const ia = idxIn('a', n.pads), ib = idxIn('b', m.pads);
		if (ia.some(v => v < 0) || ib.some(v => v < 0)) { add('网络焊盘 无法对应到 allPads', n.name); continue; }
		if (ia.join() !== ib.join()) {
			const asSet = new Set(ia);
			if (ib.length === ia.length && ib.every(v => asSet.has(v))) add('网络内焊盘顺序', `${n.name}: ${ia.length} 个焊盘集合相同、顺序不同`);
			else add('网络内焊盘集合', `${n.name}: 焊盘集合不同`);
		}
		// 网内焊盘条目与 allPads 同源，字段差异已在「焊盘字段」类里按配对比较，这里只比序列
	}
}

// ── 4. staticObstacles：按 group|kind|net|layer 分桶，桶内按几何配对 ──
{
	const geomKeys = (o) => Object.keys(o).filter(k => typeof o[k] === 'number' || Array.isArray(o[k]));
	const bucketKey = (o) => `${o.group ?? ''}|${o.kind ?? ''}|${o.net ?? ''}|${o.layer ?? ''}`;
	const bucketsB = new Map();
	B.staticObstacles.forEach((o, j) => (bucketsB.get(bucketKey(o)) ?? bucketsB.set(bucketKey(o), []).get(bucketKey(o))).push(j));
	const usedO = new Set();
	const oA2B = new Array(A.staticObstacles.length).fill(-1);
	const geomDist = (o, q) => {
		let s = 0;
		for (const k of new Set([...geomKeys(o), ...geomKeys(q)])) {
			if (k === 'id') continue;
			const d = deepDiff(o[k], q[k], Infinity);
			if (typeof o[k] === 'number' && typeof q[k] === 'number') s += Math.abs(o[k] - q[k]);
			else if (d.length) s += 1e6;
		}
		return s;
	};
	A.staticObstacles.forEach((o, i) => {
		let best = -1, bd = Infinity;
		for (const j of bucketsB.get(bucketKey(o)) ?? []) {
			if (usedO.has(j)) continue;
			const d = geomDist(o, B.staticObstacles[j]);
			if (d < bd) { bd = d; best = j; }
		}
		if (best >= 0) { usedO.add(best); oA2B[i] = best; }
	});
	const olabel = (o) => `${o.group ?? '?'}/${o.kind}${o.x !== undefined ? `@(${o.x},${o.y})` : ''}`;
	A.staticObstacles.forEach((o, i) => { if (oA2B[i] < 0) add('障碍 仅 a（同 group/kind/net/layer 无配对）', olabel(o)); });
	B.staticObstacles.forEach((o, j) => { if (!usedO.has(j)) add('障碍 仅 b（同 group/kind/net/layer 无配对）', olabel(o)); });
	A.staticObstacles.forEach((o, i) => {
		const q = B.staticObstacles[oA2B[i]];
		if (!q) return;
		for (const k of new Set([...Object.keys(o), ...Object.keys(q)])) {
			if (k === 'id') continue;
			for (const x of deepDiff(o[k], q[k], TOL, k)) add(`障碍字段 ${k}`, `${olabel(o)} ${fmtDiff(x)}`);
		}
	});
	let moved = 0;
	const ex = [];
	let last = -1;
	oA2B.forEach((j, i) => { if (j < 0) return; if (j !== i) { moved++; if (ex.length < MAX_EX) ex.push(`a#${i}→b#${j}`); } last = j; });
	if (moved) { for (const e of ex) add('障碍顺序（staticObstacles 下标不同）', e); diffs.get('障碍顺序（staticObstacles 下标不同）').count = moved; }
}

// ── 5. padShapes ──
{
	const ka = new Set(Object.keys(A.padShapes ?? {})), kb = new Set(Object.keys(B.padShapes ?? {}));
	for (const k of ka) if (!kb.has(k)) add('padShapes 键 仅 a', k);
	for (const k of kb) if (!ka.has(k)) add('padShapes 键 仅 b', k);
	for (const k of ka) {
		if (!kb.has(k)) continue;
		for (const x of deepDiff(A.padShapes[k], B.padShapes[k], TOL)) add('padShapes 内容', `${k} ${fmtDiff(x)}`);
	}
}

// ── 输出 ──
const nCmp = (a) => `${a.allPads.length} 焊盘 / ${a.networks.length} 网络 / ${a.staticObstacles.length} 障碍 / ${Object.keys(a.padShapes ?? {}).length} padShapes`;
console.log(`[compare] a = ${files[0]}\n[compare]   ${nCmp(A)}`);
console.log(`[compare] b = ${files[1]}\n[compare]   ${nCmp(B)}`);
console.log(`[compare] 容差 ${TOL}，配对距离 ${MATCH_TOL}${IGNORE_ID ? '，忽略焊盘 id' : ''}；已配对焊盘 ${usedB.size}/${A.allPads.length}`);
if (!diffs.size) {
	console.log('[compare] 一致：容差内无差异');
	process.exit(0);
}
const total = [...diffs.values()].reduce((s, d) => s + d.count, 0);
console.log(`[compare] 差异 ${diffs.size} 类，共 ${total} 处:`);
for (const [cls, d] of diffs) {
	console.log(`  - ${cls}: ${d.count}`);
	for (const e of d.examples) console.log(`      ${e}`);
	if (d.count > d.examples.length) console.log(`      ...（另 ${d.count - d.examples.length} 处）`);
}
process.exit(1);
