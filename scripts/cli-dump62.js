// dumpInput + 手工注入无网络焊盘（与 NetlessAwareRouter 同逻辑），落盘供离线复现
// 依赖 bundle：node scripts/build-eda-dump.mjs 生成（复用生产源码的障碍/规则构建器）
//
// 分块返回协议（结果可能超 CLI 单次响应 8MB 上限）：
//   invoke #1（无 args 或 mode='compute'）：计算 dump，JSON 串切块存 globalThis，
//     返回 { token, chunkCount, totalChars, totalPads, added, nets }
//   invoke #2..N（--args '{"mode":"chunk","token":"...","i":N}'）：返回 { data, done }
const bundlePath = 'D:/教程/easyeda-docs/github-eext/eext-rubberband-autorouter/scripts/eda-dump-bundle.mjs';
const args = (typeof __CLI__ !== 'undefined' && __CLI__.args) || {};

if (args.mode === 'chunk') {
	const store = globalThis.__RB_DUMP__;
	if (!store || store.token !== args.token) throw new Error('chunk store 不存在或 token 不匹配（会话状态丢失，请重跑 compute）');
	return { data: store.chunks[args.i], done: args.i >= store.chunks.length - 1 };
}

const file = await eda.sys_FileSystem.readFileFromFileSystem(bundlePath);
if (!file) throw new Error('读不到 dump bundle：' + bundlePath);
const buf = new Uint8Array(await file.arrayBuffer());
let bin = '';
for (let i = 0; i < buf.length; i += 0x8000) bin += String.fromCharCode.apply(null, buf.subarray(i, i + 0x8000));
const mod = await import('data:text/javascript;base64,' + btoa(unescape(encodeURIComponent(bin))));
mod.setup(eda);
await mod.ensurePcbTab();

const res = await mod.dumpInput({
	ripupRounds: 5,
	budgetMs: 4500000,
	optimize: true,
	allowViaFallback: true,
	allowViaInPad: false,
	netOrder: 'auto',
});
const input = res.input;

// 无网络焊盘注入（NC 脚/安装孔等，DRC 照查间距）
const all = await eda.pcb_PrimitivePad.getAll() ?? [];
const known = new Set(input.allPads.map(p => `${p.x.toFixed(2)},${p.y.toFixed(2)}`));
let added = 0;
for (const p of all) {
	const net = p.net ?? p.netName ?? '';
	if (net) continue;
	const x = p.x ?? p.center?.x ?? 0;
	const y = p.y ?? p.center?.y ?? 0;
	const key = `${Number(x).toFixed(2)},${Number(y).toFixed(2)}`;
	if (known.has(key)) continue;
	const pad = Array.isArray(p.pad) ? p.pad : [];
	const w = pad[1] ?? 0, h = pad[2] ?? 0;
	const hole = Array.isArray(p.hole) && p.hole.length >= 2 ? (p.hole[1] ?? 0) : 0;
	const rawLayer = p.layerId ?? p.layer ?? 1;
	input.allPads.push({
		x, y, net: '', id: p.primitiveId ?? `netless_${added}`,
		layer: rawLayer === 12 ? 12 : (rawLayer > 2 ? 1 : rawLayer),
		padType: rawLayer === 12 ? 'th' : 'smd',
		holeDiameter: hole,
		radius: Math.max(w / 2, h / 2, hole / 2),
		bodyRadius: Math.max(Math.min(w, h) / 2, hole / 2),
		rotation: p.rotation ?? 0,
		index: input.allPads.length,
	});
	added++;
}

const payload = { totalPads: input.allPads.length, added, nets: input.networks.map(n => n.name), input };
const json = JSON.stringify(payload);
const CHUNK = 4 * 1024 * 1024;
const chunks = [];
for (let i = 0; i < json.length; i += CHUNK) chunks.push(json.slice(i, i + CHUNK));
const token = `dump_${Date.now()}_${chunks.length}`;
globalThis.__RB_DUMP__ = { token, chunks };
return {
	token, chunkCount: chunks.length, totalChars: json.length,
	totalPads: payload.totalPads, added, nets: payload.nets,
};
