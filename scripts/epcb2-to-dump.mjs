// 离线转换：嘉立创 EDA 专业版 .epcb2 源文件 → batch-route-all.mjs 读取的布线 dump（PlanningInput）
//
// 用法: node scripts/epcb2-to-dump.mjs --case <名> [--data-root <目录>] [--svg <文件>]
//  - 读 <data-root>/case/<名>/pcb/*.epcb2（必须恰好一个），写 <data-root>/tmp/dumps/<名>-dump.json
//  - --data-root / 环境变量 RB_DATA_ROOT 由 scripts/lceda-cli.mjs 解析（与 batch-* 脚本同口径）
//  - --svg <文件>: 额外输出预览图（板框 / 禁布区 / 焊盘按层着色 / 每网飞线 / 图例与计数）
//
// 为什么存在：无 EDA 客户端（或客户端未激活）时 batch-dump-all.mjs（EDA 内运行 cli-dump62.js）跑不了；
// 有客户端时它也是一个可离线、可重复的对照来源（scripts/compare-dumps.mjs 逐字段比对两者）。
//
// 做法（零逻辑漂移）：不重写 dump 逻辑，而是把 .epcb2 解析成「EDA API 形状」的只读假 eda 对象，
// 然后原样执行生产采集代码：
//   1. esbuild 现场打包 scripts/eda-dump-driver.ts（与 build-eda-dump.mjs 同参数）；
//   2. 原样执行 scripts/cli-dump62.js（compute 模式：dumpInput + 无网络焊盘注入 + 分块），
//      其中 readFileFromFileSystem 返回步骤 1 的 bundle；
//   3. 按 batch-dump-all.mjs 的包装格式落盘（另加 source/sourceFile/sourceSha256 标记）。
// 因此焊盘字段口径、障碍构建（buildPolylineObstacle 等）、DRC 规则默认值与回退（fetchDRCRules）、
// 选项值（ripupRounds 5 等）全部出自生产源码；本脚本只负责「文件 → API 返回值」这一层。
//
// 「文件 → API」映射的依据（doc format / doc api / 客户端 ratlineworker.js 实现）：
//  - PCB / FOOTPRINT 文档坐标单位 mil、Y 向上，读盘不翻转（TYAxisDirection：PCB 段原样透传）。
//  - 器件封装解析：COMPONENT 的 ATTR「Footprint」优先，否则 ATTR「Device」→ DEVICE 文档 META.attributes.Footprint
//    （ratlineworker 的 finish()/getFootprint() 同序）；同 key 多条 ATTR 取 ticket 最大者。
//  - 器件焊盘世界坐标（ratlineworker getComponent + 器件 layerId setter + pad.flip）：
//      顶层: p = T + R(θ)·local，焊盘角 = padAngle + θ
//      底层: 先关于 x=0 镜像（x→-x），再 R(θ-180°)，焊盘角 = θ - padAngle；层 1↔2，12 不变
//  - 器件焊盘 id = 器件 id + 封装内局部 id（TCompositeElementId；PAD_NET 第 4 段即局部 id）。
//  - RULE 里的长度数值是 mil；API 按规则 unit 返回（mm = mil × 0.0254），
//    这里在假 API 层做同样换算，再由生产 fetchDRCRules 乘 39.3701 换回 mil（与 live 位级一致）。
//  - 假 API 层复现 live API 的取整/取值口径（2026-11-11 对 clock-pcb3 的 live CLI 导出实测）：
//    坐标与焊盘宽高取整到 0.1 mil；ELLIPSE 第一维为 mm（取 0.1）、第二维 mil；rotation 归一到 (-180,180]；
//    getAllNetsName / getAllPrimitivesByNet 的顺序见 netSeq。孔径不取整。
//  - 已确认项与推断项见文件末尾 CONFIRMED / INFERRED 清单，运行时也会打印。
import * as esbuild from 'esbuild';
import * as fs from 'fs';
import * as path from 'path';
import * as crypto from 'crypto';
import { fileURLToPath } from 'url';
import { STRUCT_VERSION } from './struct-version.mjs';
import { ROOT, DATA_ROOT } from './lceda-cli.mjs';

const SCRIPTS_DIR = path.dirname(fileURLToPath(import.meta.url));

const argOf = (k) => {
	const i = process.argv.indexOf(k);
	if (i < 0) return null;
	const v = process.argv[i + 1];
	if (!v || v.startsWith('--')) throw new Error(`${k} 缺少参数`);
	return v;
};
const caseName = argOf('--case');
const svgOut = argOf('--svg');
if (!caseName) {
	console.error('用法: node scripts/epcb2-to-dump.mjs --case <名> [--data-root <目录>] [--svg <文件>]');
	process.exit(2);
}

// ─────────────────────────────────────────────
// 1. 读 .epcb2：逐行「头JSON||体JSON|」，按 DOCHEAD 切文档，按 (type,id) 取 ticket 最大者
// ─────────────────────────────────────────────

const pcbDir = path.join(DATA_ROOT, 'case', caseName, 'pcb');
if (!fs.existsSync(pcbDir)) {
	console.error(`[epcb2] 找不到 ${pcbDir}（可用 --data-root 或 RB_DATA_ROOT 指定 case/ 所在目录）`);
	process.exit(1);
}
const epcbFiles = fs.readdirSync(pcbDir).filter(f => f.endsWith('.epcb2'));
if (epcbFiles.length !== 1) {
	console.error(`[epcb2] ${pcbDir} 下应恰好有 1 个 .epcb2，实际 ${epcbFiles.length} 个: ${epcbFiles.join(', ')}`);
	process.exit(1);
}
const srcFile = path.join(pcbDir, epcbFiles[0]);
const rawBytes = fs.readFileSync(srcFile);
const srcSha = crypto.createHash('sha256').update(rawBytes).digest('hex');   // 哈希对象是原始字节（含 BOM）
const rawText = rawBytes.toString('utf8').replace(/^\uFEFF/, '');          // 解析前去掉 UTF-8 BOM

/** 解析为文档数组：{ head, body, recs: Map<"type|id", {type,id,ticket,body}>, order: [] } */
function parseEpcb(text) {
	const docs = [];
	let cur = null;
	let lineNo = 0;
	for (const line of text.split(/\r?\n/)) {
		lineNo++;
		if (!line.trim()) continue;
		const sep = line.indexOf('}||');
		if (sep < 0) throw new Error(`第 ${lineNo} 行缺少 "||" 分隔`);
		const head = JSON.parse(line.slice(0, sep + 1));
		let bodyStr = line.slice(sep + 3);
		if (bodyStr.endsWith('|')) bodyStr = bodyStr.slice(0, -1);
		// 删除记录：体为空串（"" 或空）
		const body = (bodyStr === '' || bodyStr === '""') ? null : JSON.parse(bodyStr);
		if (head.type === 'DOCHEAD') {
			cur = { head, body, recs: new Map(), deleted: false };
			docs.push(cur);
			continue;
		}
		if (!cur) throw new Error(`第 ${lineNo} 行出现在任何 DOCHEAD 之前`);
		if (head.type === 'DELETE_DOC') { cur.deleted = true; continue; }
		const key = `${head.type}|${head.id ?? ''}`;
		const prev = cur.recs.get(key);
		const ticket = Number(head.ticket ?? 0);
		// 最终一致性：同 (type,id) 取 ticket 大者；ticket 相同取后出现者（同一文件同一 client）
		if (!prev || ticket >= prev.ticket) cur.recs.set(key, { type: head.type, id: head.id, ticket, body, lineNo });
	}
	return docs.filter(d => !d.deleted);
}

const docs = parseEpcb(rawText);
const recsOf = (doc, type) => [...doc.recs.values()].filter(r => r.type === type && r.body != null);

const pcbDocs = docs.filter(d => d.body?.docType === 'PCB');
if (pcbDocs.length !== 1) {
	console.error(`[epcb2] 文件里应恰好有 1 个 PCB 文档，实际 ${pcbDocs.length} 个`);
	process.exit(1);
}
const pcb = pcbDocs[0];

const footprints = new Map();   // uuid → { title, origin, pads: [{id, body}] }
const devices = new Map();      // uuid → footprint uuid
for (const d of docs) {
	const t = d.body?.docType;
	if (t === 'FOOTPRINT') {
		const meta = recsOf(d, 'META')[0]?.body ?? {};
		const can = recsOf(d, 'CANVAS')[0]?.body ?? {};
		footprints.set(d.body.uuid, {
			title: meta.title ?? d.body.uuid,
			origin: [Number(can.originX) || 0, Number(can.originY) || 0],
			pads: recsOf(d, 'PAD').map(r => ({ id: r.id, body: r.body })),
			copperOthers: ['FILL', 'POLY', 'LINE', 'ARC', 'VIA', 'REGION']
				.flatMap(tp => recsOf(d, tp)).filter(r => [1, 2, 12].includes(r.body.layerId) || r.type === 'VIA').length,
		});
	} else if (t === 'DEVICE') {
		const meta = recsOf(d, 'META')[0]?.body ?? {};
		devices.set(d.body.uuid, meta.attributes?.Footprint ?? null);
	}
}

const warnings = [];
const warn = (m) => { warnings.push(m); console.warn(`[epcb2] 警告: ${m}`); };

// ─────────────────────────────────────────────
// 2. 文件 → EDA API 形状
// ─────────────────────────────────────────────

const r4 = (v) => Math.round(v * 1e4) / 1e4 + 0;   // +0 消除 -0
/** 角度归一到 (-180, 180]：live 导出里 rotation 只出现 -90/0/90/180，没有 270 */
const normAngle = (a) => { let t = ((a % 360) + 360) % 360; if (t > 180) t -= 360; return r4(t); };
/** live API 取整：坐标与焊盘宽高保留 0.1 mil（如 1722.9579 → 1723，59.0551 → 59.1） */
const round1 = (v) => Number(v.toFixed(1)) + 0;   // toFixed 按二进制双精度的精确值舍入（53.15 → 53.1），与 live 一致；Math.round(v*10) 会得 53.2
const MM_PER_MIL = 0.0254;
/** live API 的焊盘外形口径：RECT/OVAL 两维 mil 取 0.1；ELLIPSE 第一维为 mm（取 0.1）、第二维为 mil（取 0.1） */
function apiPadShape(pad) {
	switch (pad[0]) {
		case 'ELLIPSE': return ['ELLIPSE', round1(pad[1] * MM_PER_MIL), round1(pad[2])];
		case 'OVAL': return ['OVAL', round1(pad[1]), round1(pad[2])];
		default: return [pad[0], round1(pad[1]), round1(pad[2]), round1(pad[3] ?? 0)];
	}
}

/** TPadDef → TPCB_PrimitivePadShape（doc api TPCB_PrimitivePadShape） */
function padShapeOf(def, where) {
	switch (def?.padType) {
		case 'ELLIPSE':
			if (Math.abs(def.width - def.height) > 1e-3) warn(`${where}: ELLIPSE 两维不等 ${def.width}×${def.height}（live 的混合单位口径只在圆形上实测过）`);
			return ['ELLIPSE', def.width, def.height];
		case 'OVAL': return ['OVAL', def.width, def.height];
		case 'RECT': return ['RECT', def.width, def.height, def.radius ?? 0];
		default:
			// POLYGON 焊盘的 API 顶点口径（绝对/相对、是否含旋转）未证实，宁可报错不猜
			throw new Error(`${where}: 不支持的焊盘外形 ${JSON.stringify(def?.padType)}（POLYGON 焊盘的 API 口径未证实）`);
	}
}

/** THoleDef → TPCB_PrimitivePadHole（ROUND: [ROUND, 直径]；SLOT: [SLOT, 直径, 槽长]） */
function holeOf(h, where) {
	if (!h) return null;
	const w = Number(h.width) || 0, d = Number(h.height) || 0;
	if (w === 0 && d === 0) return null;
	if (h.holeType === 'ROUND') return ['ROUND', d];
	if (h.holeType === 'SLOT') return ['SLOT', d, w];
	throw new Error(`${where}: 不支持的孔类型 ${JSON.stringify(h.holeType)}`);
}

// 器件 ATTR（同 key 取 ticket 最大）
const attrsByComp = new Map();
for (const r of recsOf(pcb, 'ATTR')) {
	const pid = r.body.parentId;
	if (!pid) continue;
	const m = attrsByComp.get(pid) ?? new Map();
	const prev = m.get(r.body.key);
	if (!prev || r.ticket >= prev.ticket) m.set(r.body.key, { value: r.body.value, ticket: r.ticket });
	attrsByComp.set(pid, m);
}

// PAD_NET：id = ["PAD_NET", 器件id, 焊盘编号, 封装内局部id]
const padNetByComp = new Map();
const padNetRecs = [];
for (const r of recsOf(pcb, 'PAD_NET')) {
	const [, compId, padNum, padId] = JSON.parse(r.id);
	const rec = { idx: padNetRecs.length, compId, padNum: String(padNum), padId: padId ?? '', net: r.body.padNet ?? '' };
	padNetRecs.push(rec);
	const arr = padNetByComp.get(compId) ?? [];
	arr.push(rec);
	padNetByComp.set(compId, arr);
}

const apiPads = [];        // pcb_PrimitivePad.getAll() 的元素（IPCB_PrimitivePad 字段名）
const padMeta = [];        // 并行：{ compId, designator, local:{x,y}, side } 供自检/预览
const componentsInfo = [];

// PCB 文档里的独立焊盘（先于器件焊盘：live 的 getAllPrimitivesByNet / 网络顺序都是独立焊盘在前，见 netSeq）
for (const r of recsOf(pcb, 'PAD')) {
	const b = r.body;
	const where = `PAD ${r.id}`;
	if (Array.isArray(b.specialPad) && b.specialPad.length) warn(`${where} 有 specialPad，只用 defaultPad`);
	apiPads.push({
		primitiveType: 'Pad',
		primitiveId: r.id,
		layer: b.layerId,
		padNumber: String(b.num),
		x: Number(b.centerX), y: Number(b.centerY),
		rotation: r4(Number(b.padAngle) || 0),
		pad: padShapeOf(b.defaultPad, where),
		net: b.netName ?? '',
		hole: holeOf(b.hole, where),
		holeOffsetX: Number(b.padOffsetX) || 0,
		holeOffsetY: Number(b.padOffsetY) || 0,
		holeRotation: Number(b.relativeAngle) || 0,
		metallization: b.plated !== false,
		padType: b.padType ?? 'NORMAL',
		primitiveLock: !!b.locked,
		free: true,
	});
	padMeta.push({ compId: null, designator: `PAD:${r.id}`, side: 'free', local: null, fpTitle: null });
}

for (const r of recsOf(pcb, 'COMPONENT')) {
	const c = r.body;
	const compId = r.id;
	const attrs = attrsByComp.get(compId) ?? new Map();
	const designator = attrs.get('Designator')?.value ?? compId;
	let fpUuid = attrs.get('Footprint')?.value ?? null;
	if (!fpUuid || !footprints.has(fpUuid)) {
		const dev = attrs.get('Device')?.value;
		if (dev && devices.has(dev)) fpUuid = devices.get(dev);
	}
	const fp = fpUuid ? footprints.get(fpUuid) : null;
	if (!fp) throw new Error(`器件 ${designator}(${compId}) 找不到封装（Footprint=${attrs.get('Footprint')?.value}, Device=${attrs.get('Device')?.value}）`);
	if (fp.origin[0] !== 0 || fp.origin[1] !== 0) warn(`封装 ${fp.title} 的 CANVAS 原点非零 ${fp.origin}（按格式文档原点不平移图元，这里忽略）`);
	if (fp.copperOthers > 0) warn(`封装 ${fp.title} 含 ${fp.copperOthers} 个铜层非焊盘图元（API getAll 不含封装内嵌图元，按生产口径不进障碍）`);
	if (c.layerIdReplacements && Object.keys(c.layerIdReplacements).length) warn(`器件 ${designator} 带 layerIdReplacements，未处理`);
	const side = c.layerId === 2 ? 'bottom' : 'top';
	if (c.layerId !== 1 && c.layerId !== 2) throw new Error(`器件 ${designator} 层号 ${c.layerId} 非顶/底层`);
	const th = (Number(c.angle) || 0) * Math.PI / 180;
	const nets = padNetByComp.get(compId) ?? [];
	const usedNet = new Set();
	componentsInfo.push({ compId, designator, footprint: fp.title, side, x: c.x, y: c.y, angle: c.angle, padCount: fp.pads.length });
	for (const fpad of fp.pads) {
		const b = fpad.body;
		const where = `${designator}.${b.num}`;
		if (Array.isArray(b.specialPad) && b.specialPad.length) warn(`${where} 有 specialPad（按层异形），只用 defaultPad`);
		let lx = Number(b.centerX), ly = Number(b.centerY);
		let rot, layer, ang;
		if (side === 'bottom') {
			lx = -lx;                         // 关于 x=0 镜像
			ang = th - Math.PI;
			rot = (Number(c.angle) || 0) - (Number(b.padAngle) || 0);
			layer = b.layerId === 1 ? 2 : b.layerId === 2 ? 1 : b.layerId;
		} else {
			ang = th;
			rot = (Number(b.padAngle) || 0) + (Number(c.angle) || 0);
			layer = b.layerId;
		}
		const cs = Math.cos(ang), sn = Math.sin(ang);
		const x = c.x + lx * cs - ly * sn;   // 保持全精度；live API 的 0.1 mil 取整在假 API 层（exposePad）做
		const y = c.y + lx * sn + ly * cs;
		// PAD_NET 匹配：优先局部 id，其次焊盘编号
		let pn = nets.find(n => n.padId && n.padId === fpad.id && !usedNet.has(n));
		if (!pn) pn = nets.find(n => !n.padId && n.padNum === String(b.num) && !usedNet.has(n));
		if (pn) {
			usedNet.add(pn);
			if (pn.padNum !== String(b.num)) warn(`${where}: PAD_NET 焊盘编号 ${pn.padNum} 与封装焊盘编号 ${b.num} 不一致（按局部 id ${fpad.id} 匹配）`);
		}
		apiPads.push({
			primitiveType: 'Pad',
			primitiveId: compId + fpad.id,
			layer,
			padNumber: String(b.num),
			x, y,
			rotation: r4(rot),
			pad: padShapeOf(b.defaultPad, where),
			net: pn?.net ?? '',
			hole: holeOf(b.hole, where),
			holeOffsetX: Number(b.padOffsetX) || 0,
			holeOffsetY: Number(b.padOffsetY) || 0,
			holeRotation: Number(b.relativeAngle) || 0,
			metallization: b.plated !== false,
			padType: b.padType ?? 'NORMAL',
			primitiveLock: !!b.locked,
			pnIndex: pn ? pn.idx : -1,
		});
		padMeta.push({ compId, designator, side, local: { x: b.centerX, y: b.centerY }, fpTitle: fp.title });
	}
	for (const n of nets) if (!usedNet.has(n)) warn(`${designator}: PAD_NET ${JSON.stringify([n.padNum, n.padId, n.net])} 在封装 ${fp.title} 里找不到对应焊盘`);
}

/** 假 API 层的 live 口径：坐标取整 0.1 mil、rotation 归一到 (-180,180]、焊盘外形按 apiPadShape（内部 apiPads 保持全精度供自检） */
function exposePad(p) {
	const { free, pnIndex, ...rest } = p;   // 内部标记不属于 API 字段
	return { ...rest, x: round1(p.x), y: round1(p.y), rotation: normAngle(p.rotation), pad: apiPadShape(p.pad) };
}

/** pcb_Net.getAllPrimitivesByNet 的焊盘元素：生产代码读 type/center/globalIndex/defaultPadShapeAspect/holeAspect/layerId（入参已是 exposePad 后的） */
function netPrimOf(p) {
	return {
		pcbItemPrimitiveType: 'PAD',
		type: p.layer === 12 ? 2 : 1,              // 1 = SMD，2 = 通孔（topological-router collectPads 注释）
		net: p.net,
		center: { x: p.x, y: p.y },
		globalIndex: p.primitiveId,
		layerId: p.layer,
		defaultPadShapeAspect: [p.pad[0], p.pad[1], p.pad[2]],
		holeAspect: p.hole ? [p.hole[0], p.hole[1], p.hole[2] ?? p.hole[1]] : undefined,
	};
}

// 网络与焊盘的枚举顺序（对 clock-pcb3 的 live 导出实测还原：64 网顺序与 195 个有网焊盘的逐个顺序完全一致）：
// 先是独立 PAD 记录（文件顺序），再是器件焊盘（按 PAD_NET 记录的文件顺序）。
//  - getAllNetsName 的顺序 = 该序列里网名的首次出现顺序（NET 记录里没有焊盘的网追加在末尾，对 dump 无影响）
//  - getAllPrimitivesByNet(net) 的顺序 = 该序列里属于该网的焊盘
// 只在这一块板上验证过；规律背后的真实机制（例如按图元类型/创建序枚举）未证实。
const netSeq = [
	...apiPads.filter(p => p.free && p.net),
	...apiPads.filter(p => !p.free && p.net).sort((a, b) => a.pnIndex - b.pnIndex),
];
const netNames = [];
{
	const seen = new Set();
	for (const p of netSeq) if (!seen.has(p.net)) { seen.add(p.net); netNames.push(p.net); }
	const inRecords = new Set();
	for (const r of recsOf(pcb, 'NET')) {
		const name = JSON.parse(r.id)[1];
		if (!name) continue;
		inRecords.add(name);
		if (!seen.has(name)) { seen.add(name); netNames.push(name); }
	}
	for (const n of netNames) if (!inRecords.has(n)) warn(`网络 ${n} 只出现在焊盘上、没有 NET 记录`);
}

// polygon-parse：生产解析器。本脚本只用它判定「源图元能否解析出几何」与板框自检，不参与 dump 生成
const { parsePolygonPath } = await import('data:text/javascript;base64,' + Buffer.from((await esbuild.build({
	entryPoints: [path.join(ROOT, 'src/router/polygon-parse.ts')], bundle: true, format: 'esm', platform: 'neutral', write: false,
})).outputFiles[0].text).toString('base64'));

// 走线类图元（本文件的 PCB 文档层面；封装内嵌图元按 API 文档不在 getAll 里）
const polylines = recsOf(pcb, 'POLY').map(r => ({
	primitiveType: 'Polyline', primitiveId: r.id,
	net: r.body.netName ?? '', layer: r.body.layerId,
	polygon: { polygon: r.body.path }, lineWidth: Number(r.body.width) || 0,
	polyType: r.body.polyType,
}));
const lines = recsOf(pcb, 'LINE').map(r => ({
	primitiveType: 'Line', primitiveId: r.id,
	net: r.body.netName ?? '', layer: r.body.layerId,
	startX: r.body.startX, startY: r.body.startY, endX: r.body.endX, endY: r.body.endY,
	lineWidth: Number(r.body.width) || 0,
}));
const arcs = recsOf(pcb, 'ARC').map(r => {
	if (r.body.arcType && r.body.arcType !== 'DOT') warn(`ARC ${r.id} arcType=${r.body.arcType}：API arcAngle 口径未证实`);
	return {
		primitiveType: 'Arc', primitiveId: r.id,
		net: r.body.netName ?? '', layer: r.body.layerId,
		startX: r.body.startX, startY: r.body.startY, endX: r.body.endX, endY: r.body.endY,
		arcAngle: r.body.angle, lineWidth: Number(r.body.width) || 0,
	};
});
/** FILL 的 path：扁平单子路径 ["R",…] / [x,y,"L",…]，或子路径数组 [[…],[…]]。统一拆成子路径列表 */
const subpathsOf = (p) => !Array.isArray(p) || p.length === 0 ? [] : (Array.isArray(p[0]) ? p.filter(Array.isArray) : [p]);
const fills = [];
for (const r of recsOf(pcb, 'FILL')) {
	const subs = subpathsOf(r.body.path);
	if (subs.length === 0) warn(`FILL ${r.id}: path 为空或无法识别，无法生成障碍（若在铜层，下方自检会失败）`);
	if (subs.length > 1) warn(`FILL ${r.id} 含 ${subs.length} 个子路径：生产读取只认单个扁平路径，这里拆成 ${subs.length} 个独立 Fill 图元（真实 API 对多子路径的返回形态未证实）`);
	(subs.length ? subs : [[]]).forEach((sub, k, all) => fills.push({
		primitiveType: 'Fill', primitiveId: all.length > 1 ? `${r.id}#${k}` : r.id, srcId: r.id,
		net: r.body.netName ?? '', layer: r.body.layerId, lineWidth: Number(r.body.width) || 0,
		complexPolygon: { polygon: sub },
	}));
}
const vias = recsOf(pcb, 'VIA').map(r => ({
	primitiveType: 'Via', primitiveId: r.id,
	net: r.body.netName ?? '', x: r.body.centerX, y: r.body.centerY,
	diameter: r.body.viaDiameter, holeDiameter: r.body.holeDiameter,
}));
if (lines.length || arcs.length || fills.length || vias.length) {
	warn(`PCB 文档含 LINE ${lines.length} / ARC ${arcs.length} / FILL ${fills.length} / VIA ${vias.length}：这些映射未在真板上验证（clock-pcb3 不含这几类）`);
}
for (const t of ['POUR', 'POURED', 'TEARDROP', 'BOARD']) {
	const n = recsOf(pcb, t).length;
	if (n) warn(`PCB 文档含 ${n} 个 ${t}：生产采集不读该类图元，dump 中不体现`);
}
const regions = recsOf(pcb, 'REGION').map(r => ({ id: r.id, ...r.body }));

// 层：IPCB_LayerItem（type: 顶/底/内层信号 = SIGNAL；layerStatus: 0 不用 / 1 使用并展示 / 2 使用不展示）
const layerItems = recsOf(pcb, 'LAYER').map(r => {
	const id = JSON.parse(r.id)[1];
	const lt = r.body.layerType;
	return {
		id, name: r.body.layerName,
		type: (lt === 'TOP' || lt === 'BOTTOM' || lt === 'SIGNAL') ? 'SIGNAL' : (lt === 'PLANE' ? 'PLANE' : String(lt)),
		layerStatus: r.body.use ? (r.body.show ? 1 : 2) : 0,
		locked: !!r.body.locked,
	};
});

// DRC：RULE → getCurrentRuleConfiguration().config 的形状（fetchDRCRules 读取的那几个键）
// 文件里 RULE 的长度数值是 mil。live API 按规则 unit 返回：mm = mil × 0.0254（实测：clock-pcb3 的
// 默认规则 stroDef 10 → 0.254、4.0157 → 0.10199878、11.8 → 0.29972，经生产 ×39.3701 后与 live 导出位级一致；mm 值不取整）。
// 行/列标签（SAFE_LABELS）：EDA「安全间距」表为 13×13 下三角矩阵，content[行][列]，列 0 = Track；
// 生产只读 content[行][0]。标签顺序是推断：live 对照只能确认下面 CONFIRMED 里列出的那些格子。
const SAFE_LABELS = ['Track', 'SMD Pad', 'TH Pad', 'SMD Test Point', 'TH Test Point', 'Via',
	'Fill Region/Teardrop', 'Copper/Plane Zone', 'Slot Region', 'Line', 'Text/Image', 'Board Outline', 'Hole'];
const UNIT_PER_MIL = { mil: 1, mm: MM_PER_MIL, inch: 0.001, in: 0.001, cm: 0.00254 };
const milToUnit = (v, unit) => {
	const k = UNIT_PER_MIL[String(unit ?? 'mm').toLowerCase()];
	if (!k) throw new Error(`未知规则单位 ${unit}`);
	return v * k;   // 不取整：live 的 mm 值就是 mil × 0.0254 的原始 double（试过 8/10 位取整，trackToBoardMil 会差 1 ulp）
};
const rules = recsOf(pcb, 'RULE').map(r => {
	const [, type, name] = JSON.parse(r.id);
	return { type, name, state: r.body.ruleState, ctx: r.body.ruleContext ?? {} };
});
const drcConfig = { Spacing: {}, Physics: {} };
{
	const safe = {};
	for (const r of rules.filter(x => x.type === 'SAFE')) {
		const unit = r.ctx.unit ?? 'mm';
		const tables = {};
		for (const t of r.ctx.safeSpacing ?? []) {
			tables[String(t.layerId)] = { content: (t.content ?? []).map(row => row.map(v => milToUnit(Number(v), unit))) };
		}
		safe[r.name] = {
			isSetDefault: r.state === 'DEFAULT', unit,
			row: SAFE_LABELS.slice(0, 12), column: SAFE_LABELS.slice(),
			tables,
		};
	}
	if (Object.keys(safe).length) drcConfig.Spacing['Safe Spacing'] = safe;
	const other = {};
	for (const r of rules.filter(x => x.type === 'OTHER')) {
		// 值按规则自己的 unit 返回；生产 fetchDRCRules 固定按 mm 读 hole2Hole（unit 非 mm 时是生产的口径问题，下面自检会提示）
		other[r.name] = { isSetDefault: r.state === 'DEFAULT', unit: r.ctx.unit, form: { hole2Hole: milToUnit(Number(r.ctx.holeClearance), r.ctx.unit ?? 'mm') } };
	}
	if (Object.keys(other).length) drcConfig.Spacing['Other Spacing'] = other;
	const track = {};
	for (const r of rules.filter(x => x.type === 'TRACK')) {
		const unit = r.ctx.unit ?? 'mm';
		// form.data 以层号为键（每个 content 行一项）；生产只读 data["1"]，没有第 1 层的行时生产会静默回退默认线宽
		const data = {};
		for (const row of r.ctx.track?.content ?? []) {
			data[String(row.layerId)] = {
				defaultValue: milToUnit(Number(row.stroDef), unit),
				minValue: milToUnit(Number(row.stroMin), unit),
				maxValue: milToUnit(Number(row.stroMax), unit),
			};
		}
		if (!data['1']) warn(`TRACK 规则 ${r.name} 没有 layerId=1 的行（层号 ${Object.keys(data).join(',') || '无'}）：生产 fetchDRCRules 只读 data["1"]，线宽会静默回退到默认值`);
		track[r.name] = { isSetDefault: r.state === 'DEFAULT', unit, form: { data } };
	}
	if (Object.keys(track).length) drcConfig.Physics.Track = track;
}

// ─────────────────────────────────────────────
// 3. 假 eda 对象 + 原样执行 cli-dump62.js（内部调用 eda-dump-driver.ts 的 dumpInput）
// ─────────────────────────────────────────────

const clone = (o) => JSON.parse(JSON.stringify(o));   // 每次调用返回新对象，防止生产代码改写共享数据
const padsByNet = new Map();
for (const p of netSeq) {
	const arr = padsByNet.get(p.net) ?? [];
	arr.push(p);
	padsByNet.set(p.net, arr);
}

const built = await esbuild.build({
	entryPoints: [path.join(SCRIPTS_DIR, 'eda-dump-driver.ts')],
	bundle: true, format: 'esm', platform: 'neutral', write: false, legalComments: 'none',
	banner: { js: 'const __edaShim = (globalThis.eda ?? null);' },
});
const bundleBytes = Buffer.from(built.outputFiles[0].contents);
const VIRTUAL_BUNDLE = '/virtual/eda-dump-bundle.mjs';

const fakeEda = {
	dmt_Pcb: { getAllPcbsInfo: async () => [{ uuid: pcb.body.uuid, name: path.basename(srcFile, '.epcb2') }] },
	dmt_EditorControl: { openDocument: async () => true },
	sys_FileSystem: {
		readFileFromFileSystem: async (p) => {
			if (p !== VIRTUAL_BUNDLE) throw new Error(`假 eda 只提供 dump bundle，不提供 ${p}`);
			return { arrayBuffer: async () => bundleBytes.buffer.slice(bundleBytes.byteOffset, bundleBytes.byteOffset + bundleBytes.byteLength) };
		},
	},
	pcb_Net: {
		getAllNetsName: async () => netNames.slice(),
		getAllPrimitivesByNet: async (net) => (padsByNet.get(net) ?? []).map(p => netPrimOf(exposePad(clone(p)))),
	},
	pcb_PrimitivePad: { getAll: async () => clone(apiPads).map(exposePad) },
	pcb_PrimitivePolyline: { getAll: async () => clone(polylines) },
	pcb_PrimitiveLine: { getAll: async () => clone(lines) },
	pcb_PrimitiveArc: { getAll: async () => clone(arcs) },
	pcb_PrimitiveFill: { getAll: async () => clone(fills) },
	pcb_PrimitiveVia: { getAll: async () => clone(vias) },
	pcb_Layer: { getAllLayers: async () => clone(layerItems) },
	pcb_Drc: { getCurrentRuleConfiguration: async () => ({ name: 'epcb2-offline', config: clone(drcConfig) }) },
};
globalThis.eda = fakeEda;
(globalThis).__RB_VERBOSE__ = false;

const cliCode = fs.readFileSync(path.join(SCRIPTS_DIR, 'cli-dump62.js'), 'utf8');
const AsyncFunction = Object.getPrototypeOf(async function () { /* 取构造器 */ }).constructor;
const runCli = new AsyncFunction('eda', '__CLI__', cliCode);
const meta = await runCli(fakeEda, { args: { mode: 'compute', bundlePath: VIRTUAL_BUNDLE } });
const store = globalThis.__RB_DUMP__;
if (!store || store.token !== meta.token) throw new Error('cli-dump62 未留下分块数据');
const v = JSON.parse(store.chunks.join(''));
const input = v.input;

// ─────────────────────────────────────────────
// 4. 自检（任一失败 → 不写 dump，退出码 1）
// ─────────────────────────────────────────────

const checks = [];
const check = (name, ok, detail) => { checks.push({ name, ok: !!ok, detail }); };

// 4.1 独立计数：只看原始记录（PAD_NET 的 padNet + 独立 PAD 的 netName；封装焊盘数 × 器件）
const tally = new Map();
for (const rec of padNetRecs) if (rec.net) tally.set(rec.net, (tally.get(rec.net) ?? 0) + 1);
let freePadCount = 0;
for (const r of recsOf(pcb, 'PAD')) {
	freePadCount++;
	if (r.body.netName) tally.set(r.body.netName, (tally.get(r.body.netName) ?? 0) + 1);
}
let compPadCount = 0;
for (const ci of componentsInfo) compPadCount += ci.padCount;
const physicalPads = compPadCount + freePadCount;
const tallyNetted = [...tally.values()].reduce((s, x) => s + x, 0);
check('PAD_NET 记录数 = 器件焊盘数', padNetRecs.length === compPadCount, `${padNetRecs.length} vs ${compPadCount}`);
check('dump 焊盘总数 = 文件物理焊盘数', input.allPads.length === physicalPads, `${input.allPads.length} vs ${physicalPads}（器件 ${compPadCount} + 独立 ${freePadCount}）`);
const dumpNetted = input.allPads.filter(p => p.net).length;
check('有网焊盘数 = PAD_NET/PAD 网名计数', dumpNetted === tallyNetted, `${dumpNetted} vs ${tallyNetted}`);
check('网络数 = 有焊盘的网名数', input.networks.length === tally.size, `${input.networks.length} vs ${tally.size}`);
{
	const bad = [];
	const seen = new Set();
	for (const n of input.networks) {
		seen.add(n.name);
		if (tally.get(n.name) !== n.pads.length) bad.push(`${n.name}: dump ${n.pads.length} / 文件 ${tally.get(n.name) ?? 0}`);
	}
	for (const name of tally.keys()) if (!seen.has(name)) bad.push(`${name}: dump 缺失`);
	check('逐网焊盘数一致', bad.length === 0, bad.length ? bad.slice(0, 10).join('; ') : `${input.networks.length} 网全部一致`);
}
// 网络过滤口径：driver 收 ≥1 个焊盘的网（netPads.length >= 1），单焊盘网也进 networks
const singlePadNets = input.networks.filter(n => n.pads.length < 2).map(n => n.name);
check('networks 每网 ≥1 焊盘（driver 口径）', input.networks.every(n => n.pads.length >= 1), `单焊盘网 ${singlePadNets.length} 个${singlePadNets.length ? '：' + singlePadNets.join(', ') : ''}`);

// 4.2 数值完整性
{
	let bad = 0;
	const finite = (x) => typeof x === 'number' && Number.isFinite(x);
	for (const p of input.allPads) {
		for (const k of ['x', 'y', 'radius', 'bodyRadius', 'holeDiameter', 'layer', 'index']) if (!finite(p[k])) bad++;
		if (typeof p.id !== 'string' || !p.id) bad++;
	}
	for (const o of input.staticObstacles) for (const k of ['x', 'y', 'r']) if (!finite(o[k])) bad++;
	for (const [k, s] of Object.entries(input.padShapes)) {
		if (!Array.isArray(s.pad) || !finite(s.pad[1]) || !finite(s.pad[2]) || !finite(s.rotation)) bad++;
		if (k.split(',').some(t => !Number.isFinite(Number(t)))) bad++;
	}
	for (const [k, val] of Object.entries(input.drc)) if (!finite(val)) bad++;
	check('无 NaN/undefined 坐标与尺寸', bad === 0, `${bad} 处异常`);
	const idSet = new Set(input.allPads.map(p => p.id));
	check('焊盘 id 唯一', idSet.size === input.allPads.length, `${idSet.size} / ${input.allPads.length}`);
}

// 4.3 板框：生产对 layer 11 的 POLY / LINE / ARC 都按板框障碍处理（eda-dump-driver 同口径），自检也三类都认
/** 圆弧按 eda-dump-driver 同公式取样成折线（仅用于板框自检，dump 本身仍由生产代码生成） */
function arcPoints(o) {
	const sx = o.startX ?? 0, sy = o.startY ?? 0, ex = o.endX ?? 0, ey = o.endY ?? 0;
	const sweep = (o.arcAngle ?? 0) * Math.PI / 180;
	const L = Math.hypot(ex - sx, ey - sy);
	if (!sweep || Math.abs(Math.sin(sweep / 2)) < 1e-6 || L < 1e-6) return [];
	const R = L / (2 * Math.abs(Math.sin(sweep / 2)));
	const sgn = sweep > 0 ? 1 : -1;
	const h = sgn * R * Math.cos(sweep / 2);
	const cx = (sx + ex) / 2 + h * (-(ey - sy) / L), cy = (sy + ey) / 2 + h * ((ex - sx) / L);
	const a0 = Math.atan2(sy - cy, sx - cx);
	const steps = Math.max(6, Math.ceil(Math.abs(sweep) / (Math.PI / 12)));
	const pts = [];
	for (let k = 0; k <= steps; k++) {
		const ang = a0 + (Math.abs(sweep) * k) / steps * sgn;
		pts.push({ x: cx + R * Math.cos(ang), y: cy + R * Math.sin(ang) });
	}
	return pts;
}
/** 把首尾相接的线段/圆弧串成环；返回 { rings, rest }，rest 是没能闭合的链 */
function chainRings(segs, eps = 0.05) {
	const left = segs.map(sg => sg.slice());
	const rings = [], rest = [];
	const near = (a, b) => Math.hypot(a.x - b.x, a.y - b.y) <= eps;
	while (left.length) {
		let chain = left.shift();
		for (let grown = true; grown && !near(chain[0], chain[chain.length - 1]);) {
			grown = false;
			for (let i = 0; i < left.length; i++) {
				const sg = left[i], end = chain[chain.length - 1];
				if (near(sg[0], end)) chain = chain.concat(sg.slice(1));
				else if (near(sg[sg.length - 1], end)) chain = chain.concat(sg.slice(0, -1).reverse());
				else continue;
				left.splice(i, 1); grown = true; break;
			}
		}
		(chain.length >= 4 && near(chain[0], chain[chain.length - 1]) ? rings : rest).push(chain);
	}
	return { rings, rest };
}
const outlinePolyRings = polylines.filter(p => p.layer === 11).map(p => parsePolygonPath(p.polygon.polygon)).filter(p => p.length >= 3);
const outlineLoose = [
	...lines.filter(l => l.layer === 11).map(l => [{ x: l.startX ?? 0, y: l.startY ?? 0 }, { x: l.endX ?? 0, y: l.endY ?? 0 }]),
	...arcs.filter(a => a.layer === 11).map(arcPoints),
].filter(sg => sg.length >= 2);
const chained = chainRings(outlineLoose);
const outlinePolys = [...outlinePolyRings, ...chained.rings];
const area = (poly) => { let a = 0; for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) a += poly[j].x * poly[i].y - poly[i].x * poly[j].y; return Math.abs(a) / 2; };
const outlinePts = outlinePolys.slice().sort((a, b) => area(b) - area(a))[0] ?? [];   // 外轮廓 = 面积最大者
const pip = (px, py, poly) => {
	let inside = false;
	for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
		const xi = poly[i].x, yi = poly[i].y, xj = poly[j].x, yj = poly[j].y;
		if (((yi > py) !== (yj > py)) && (px < ((xj - xi) * (py - yi)) / (yj - yi) + xi)) inside = !inside;
	}
	return inside;
};
let bbox = null;
if (outlinePts.length || chained.rest.length) {
	const bpts = outlinePts.length ? outlinePts : chained.rest.flat();
	const xs = bpts.map(p => p.x), ys = bpts.map(p => p.y);
	bbox = { minX: Math.min(...xs), maxX: Math.max(...xs), minY: Math.min(...ys), maxY: Math.max(...ys) };
}
check('存在板框（layer 11 的 POLY / LINE / ARC）', outlinePolys.length + chained.rest.length >= 1,
	`闭合环 ${outlinePolys.length}（POLY ${outlinePolyRings.length} + LINE/ARC 串成 ${chained.rings.length}），未闭合链 ${chained.rest.length}`);
if (outlinePolys.length > 1) warn(`有 ${outlinePolys.length} 个闭合板框环，内外判定只用面积最大的一个`);
if (!outlinePolys.length && chained.rest.length) warn('layer 11 的 LINE/ARC 没能串成闭合环：板框内外判定退化为包围盒（生产按线段障碍处理，不受影响）');
if (bbox) {
	const outBox = input.allPads.filter(p => p.x < bbox.minX || p.x > bbox.maxX || p.y < bbox.minY || p.y > bbox.maxY);
	check('全部焊盘中心在板框包围盒内', outBox.length === 0, `${outBox.length} 个在外${outBox.length ? '：' + outBox.slice(0, 5).map(p => p.id).join(',') : ''}`);
	if (outlinePts.length) {
		const outPoly = input.allPads.filter(p => !pip(p.x, p.y, outlinePts));
		check('全部焊盘中心在板框多边形内', outPoly.length === 0, `${outPoly.length} 个在外`);
	}
	// 焊盘外接圆越出板框（信息项，不判失败）
	const edgeOver = input.allPads.filter(p =>
		p.x - p.radius < bbox.minX || p.x + p.radius > bbox.maxX || p.y - p.radius < bbox.minY || p.y + p.radius > bbox.maxY);
	check('（信息）焊盘外接圆（radius）越出板框包围盒数', true, `${edgeOver.length}${edgeOver.length ? '：' + edgeOver.map(p => `${p.id}(${p.net || '无网络'})`).join(', ') : ''}`);
}

// 4.4 变换自检：每个器件的焊盘两两距离与封装内一致（刚体变换）；底层器件有向面积符号相对封装反转。
// 注意：这两项只是核对本脚本自己施加的变换没有写错（内部一致性），不是镜像约定的独立证据；
// 镜像/旋转约定的独立证据是与 live CLI 导出的逐焊盘比对（scripts/compare-dumps.mjs，clock-pcb3 的 42 个底层焊盘位置一致）。
{
	let distBad = 0, chiralBad = 0;
	const byComp = new Map();
	apiPads.forEach((p, i) => { const m = padMeta[i]; if (m.compId) { const a = byComp.get(m.compId) ?? []; a.push({ p, m }); byComp.set(m.compId, a); } });
	for (const arr of byComp.values()) {
		for (let i = 0; i < arr.length; i++) for (let j = i + 1; j < arr.length; j++) {
			const dl = Math.hypot(arr[i].m.local.x - arr[j].m.local.x, arr[i].m.local.y - arr[j].m.local.y);
			const dw = Math.hypot(arr[i].p.x - arr[j].p.x, arr[i].p.y - arr[j].p.y);
			if (Math.abs(dl - dw) > 1e-3) distBad++;
		}
		// 手性：取前三个不共线焊盘的有向面积，底层应与封装内相反
		const side = arr[0].m.side;
		for (let i = 0; i + 2 < arr.length; i++) {
			const [a, b, c] = [arr[i], arr[i + 1], arr[i + 2]];
			const al = (b.m.local.x - a.m.local.x) * (c.m.local.y - a.m.local.y) - (b.m.local.y - a.m.local.y) * (c.m.local.x - a.m.local.x);
			if (Math.abs(al) < 1) continue;
			const aw = (b.p.x - a.p.x) * (c.p.y - a.p.y) - (b.p.y - a.p.y) * (c.p.x - a.p.x);
			if ((side === 'bottom') !== (Math.sign(al) !== Math.sign(aw))) chiralBad++;
			break;
		}
	}
	check('器件焊盘两两距离与封装一致（刚体变换）', distBad === 0, `${distBad} 对不一致`);
	check('（内部一致性）本脚本的底层镜像 / 顶层不镜像与自身变换一致', chiralBad === 0, `${chiralBad} 个器件不符；非独立证据，独立证据见 live 比对`);
}

// 4.5 不同器件的焊盘铜皮不重叠（同层或含通孔；按焊盘短轴半径保守判定）
{
	const over = [];
	const shareLayer = (a, b) => a.layer === 12 || b.layer === 12 || a.layer === b.layer;
	for (let i = 0; i < apiPads.length; i++) for (let j = i + 1; j < apiPads.length; j++) {
		const a = apiPads[i], b = apiPads[j];
		const ma = padMeta[i], mb = padMeta[j];
		if (ma.compId && ma.compId === mb.compId) continue;
		if (!shareLayer(a, b)) continue;
		const ra = Math.min(a.pad[1], a.pad[2]) / 2, rb = Math.min(b.pad[1], b.pad[2]) / 2;
		const d = Math.hypot(a.x - b.x, a.y - b.y);
		if (d < ra + rb) over.push(`${ma.designator}.${a.padNumber}↔${mb.designator}.${b.padNumber} d=${d.toFixed(1)}`);
	}
	check('不同器件焊盘不重叠（短轴半径判定）', over.length === 0, over.length ? over.slice(0, 8).join('; ') : '0 处');
}

// 4.6 DRC 往返：dump 里的 mil 值应等于文件里默认规则的 mil 值（验证 mil→unit→mil 换算链与生产读取格子）。
// 期望值直接从文件取（默认规则 = 第一个 DEFAULT，否则第一个），格子行号按 SAFE_LABELS 查
{
	const pick = (type) => rules.find(r => r.type === type && r.state === 'DEFAULT') ?? rules.find(r => r.type === type);
	const col0 = (rule, label) => { const c = rule?.ctx.safeSpacing?.find(t => t.layerId === 1)?.content; const v = c?.[SAFE_LABELS.indexOf(label)]?.[0]; return typeof v === 'number' && v > 0 ? v : null; };
	const safeR = pick('SAFE'), trackR = pick('TRACK'), otherR = pick('OTHER');
	const exp = [];
	const sw = trackR?.ctx.track?.content?.find(r => r.layerId === 1)?.stroDef;
	if (typeof sw === 'number' && sw > 0) exp.push(['traceWidthMil', sw]);
	if (safeR?.ctx.safeSpacing?.some(t => t.layerId === 1)) {
		const at = (k, v) => { if (v != null) exp.push([k, v]); };
		at('trackToTrackMil', col0(safeR, 'Track'));
		at('trackToLineMil', col0(safeR, 'Line'));
		at('trackToBoardMil', col0(safeR, 'Board Outline'));
		at('trackToFillMil', col0(safeR, 'Copper/Plane Zone') ?? col0(safeR, 'Fill Region/Teardrop'));
		const padV = Math.max(col0(safeR, 'SMD Pad') ?? 0, col0(safeR, 'TH Pad') ?? 0, col0(safeR, 'Via') ?? 0);
		if (padV > 0) at('trackToPadMil', padV);
	}
	if (typeof otherR?.ctx.holeClearance === 'number' && otherR.ctx.holeClearance > 0) {
		if (String(otherR.ctx.unit ?? 'mm').toLowerCase() === 'mm') exp.push(['holeToHoleMil', otherR.ctx.holeClearance]);
		else warn(`OTHER 规则单位为 ${otherR.ctx.unit}（非 mm）：生产 fetchDRCRules 固定按 mm 读 hole2Hole，dump 的 holeToHoleMil 与文件值不同属生产口径，已跳过该项核对`);
	}
	const bad = exp.filter(([k, v]) => !(Math.abs(input.drc[k] - v) <= 1e-3));
	if (exp.length === 0) warn('文件里没有可核对的默认 RULE（SAFE/TRACK/OTHER），DRC 全部走生产回退默认值，DRC 往返自检无内容可比');
	check('DRC 往返：dump 的 mil 值 = 文件默认规则的 mil 值', bad.length === 0,
		exp.length === 0 ? '无可核对项（已 warn）' : (bad.length ? bad.map(([k, v]) => `${k}: dump ${input.drc[k]} vs 文件 ${v}`).join('; ') : `${exp.length} 项一致（${exp.map(([k]) => k.replace(/Mil$/, '')).join('/')}）`));
}

// 4.7 障碍源图元自检：把每个源图元单独交给生产 dumpInput（其它类全空），要求
//   (a) 「应产生障碍」的源图元数 = 实际产生 ≥1 个障碍的源图元数，且逐个一致（不允许静默丢失）；
//   (b) 各源图元单独产生的障碍数之和 = 整份 dump 的 staticObstacles 数（没有多出/漏掉的来源）。
// 「应产生障碍」的判据只看层号/坐标是否有效，与几何是否可解析无关 —— 所以解析不出几何的铜层图元会在这里失败
{
	const driverMod = await import('data:text/javascript;base64,' + bundleBytes.toString('base64'));
	const emptyApi = () => ({
		pcb_Net: { getAllNetsName: async () => [], getAllPrimitivesByNet: async () => [] },
		pcb_PrimitivePad: { getAll: async () => [] },
		pcb_PrimitivePolyline: { getAll: async () => [] },
		pcb_PrimitiveLine: { getAll: async () => [] },
		pcb_PrimitiveArc: { getAll: async () => [] },
		pcb_PrimitiveFill: { getAll: async () => [] },
		pcb_PrimitiveVia: { getAll: async () => [] },
		pcb_Layer: fakeEda.pcb_Layer,
		pcb_Drc: fakeEda.pcb_Drc,
	});
	const board = [1, 2, 11, 12], copper = [1, 2, 12];
	const sources = [
		...polylines.map(p => ({ kind: 'POLY', id: p.primitiveId, api: 'pcb_PrimitivePolyline', prim: p, expect: board.includes(p.layer) })),
		...lines.map(p => ({ kind: 'LINE', id: p.primitiveId, api: 'pcb_PrimitiveLine', prim: p, expect: board.includes(p.layer) })),
		...arcs.map(p => ({ kind: 'ARC', id: p.primitiveId, api: 'pcb_PrimitiveArc', prim: p, expect: board.includes(p.layer) })),
		...fills.map(p => ({ kind: 'FILL', id: p.primitiveId, api: 'pcb_PrimitiveFill', prim: p, expect: copper.includes(p.layer) })),
		...vias.map(p => ({ kind: 'VIA', id: p.primitiveId, api: 'pcb_PrimitiveVia', prim: p, expect: Number.isFinite(p.x) && Number.isFinite(p.y) })),
	];
	let expected = 0, produced = 0, sumObs = 0;
	const mism = [];
	const savedEda = globalThis.eda;
	try {
		for (const src of sources) {
			const api = emptyApi();
			api[src.api] = { getAll: async () => [clone(src.prim)] };
			globalThis.eda = api;
			const n = (await driverMod.dumpInput({})).input.staticObstacles.length;
			sumObs += n;
			if (src.expect) expected++;
			if (n >= 1) produced++;
			if ((n >= 1) !== src.expect) mism.push(`${src.kind} ${src.id}（层 ${src.prim.layer ?? '-'}）应${src.expect ? '' : '不'}产生障碍，实得 ${n} 个`);
		}
	} finally { globalThis.eda = savedEda; }
	for (const m of mism.slice(0, 5)) warn(`障碍源图元异常：${m}`);
	const byKind = {};
	for (const src of sources) if (src.expect) byKind[src.kind] = (byKind[src.kind] ?? 0) + 1;
	check('障碍源图元：应产生障碍的数 = 实际产生 ≥1 个障碍的数（逐个一致）', expected === produced && mism.length === 0,
		`应产生 ${expected} / 实际产生 ${produced}（${JSON.stringify(byKind)}）${mism.length ? '；不一致：' + mism.slice(0, 5).join('; ') : ''}`);
	check('各源图元单独产生的障碍数之和 = dump 障碍总数', sumObs === input.staticObstacles.length, `${sumObs} vs ${input.staticObstacles.length}`);
}

// ─────────────────────────────────────────────
// 5. 汇总输出
// ─────────────────────────────────────────────

const smd = input.allPads.filter(p => p.padType === 'smd').length;
const thp = input.allPads.filter(p => p.padType === 'th').length;
const obsKinds = {};
for (const o of input.staticObstacles) {
	const k = String(o.group ?? '').split(':')[0] || '(none)';
	obsKinds[k] = (obsKinds[k] ?? 0) + 1;
}
const shapeKeyCollisions = apiPads.length - new Set(apiPads.map(p => `${p.x.toFixed(2)},${p.y.toFixed(2)}`)).size;

console.log(`\n[epcb2] 源文件: ${srcFile}`);
console.log(`[epcb2] 器件 ${componentsInfo.length}（底层 ${componentsInfo.filter(c => c.side === 'bottom').length}），封装 ${footprints.size}，独立焊盘 ${freePadCount}`);
console.log(`[epcb2] 焊盘 ${input.allPads.length}（SMD ${smd} / 通孔 ${thp} / 注入无网络 ${v.added}），网络 ${input.networks.length}（单焊盘网 ${singlePadNets.length}），铜层 ${input.copperLayers}`);
console.log(`[epcb2] 静态障碍圆 ${input.staticObstacles.length}，按来源: ${JSON.stringify(obsKinds)}`);
if (bbox) console.log(`[epcb2] 板框包围盒 x ${bbox.minX.toFixed(2)}..${bbox.maxX.toFixed(2)} y ${bbox.minY.toFixed(2)}..${bbox.maxY.toFixed(2)}（${(bbox.maxX - bbox.minX).toFixed(2)} × ${(bbox.maxY - bbox.minY).toFixed(2)} mil = ${((bbox.maxX - bbox.minX) * 0.0254).toFixed(2)} × ${((bbox.maxY - bbox.minY) * 0.0254).toFixed(2)} mm）`);
console.log(`[epcb2] 禁布区 REGION ${regions.length}（禁止类型: ${[...new Set(regions.flatMap(r => r.prohibitType ?? []))].join('/')}；生产采集不读 REGION，未进障碍）`);
console.log(`[epcb2] DRC(mil): ${Object.entries(input.drc).map(([k, x]) => `${k}=${Number(x).toFixed(4)}`).join(' ')}`);
console.log(`[epcb2] options: ${JSON.stringify(input.options)}；padShapes ${Object.keys(input.padShapes).length}（位置键冲突 ${shapeKeyCollisions}）`);
console.log('\n[epcb2] 自检:');
for (const c of checks) console.log(`  ${c.ok ? 'PASS' : 'FAIL'}  ${c.name} — ${c.detail}`);
if (warnings.length) console.log(`\n[epcb2] 警告 ${warnings.length} 条（见上）`);

// ─────────────────────────────────────────────
// 6. 预览 SVG
// ─────────────────────────────────────────────

function mstEdges(pts) {
	const n = pts.length;
	if (n < 2) return [];
	const inT = new Array(n).fill(false), best = new Array(n).fill(Infinity), from = new Array(n).fill(-1);
	best[0] = 0;
	const edges = [];
	for (let k = 0; k < n; k++) {
		let u = -1;
		for (let i = 0; i < n; i++) if (!inT[i] && (u < 0 || best[i] < best[u])) u = i;
		inT[u] = true;
		if (from[u] >= 0) edges.push([from[u], u]);
		for (let i = 0; i < n; i++) {
			if (inT[i]) continue;
			const d = Math.hypot(pts[i].x - pts[u].x, pts[i].y - pts[u].y);
			if (d < best[i]) { best[i] = d; from[i] = u; }
		}
	}
	return edges;
}

function writeSvg(file) {
	const esc = (s) => String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
	const xs = [], ys = [];
	for (const p of outlinePts) { xs.push(p.x); ys.push(p.y); }
	for (const p of input.allPads) { xs.push(p.x); ys.push(p.y); }
	const pad = 60;
	const minX = Math.min(...xs) - pad, maxX = Math.max(...xs) + pad;
	const minY = Math.min(...ys) - pad, maxY = Math.max(...ys) + pad;
	const W = maxX - minX, H = maxY - minY;
	const legendH = 230;
	const out = [];
	out.push(`<svg xmlns="http://www.w3.org/2000/svg" viewBox="${minX} ${-maxY - legendH} ${W} ${H + legendH}" width="1800" height="${Math.round(1800 * (H + legendH) / W)}" font-family="'Noto Sans CJK SC','Microsoft YaHei','PingFang SC',sans-serif">`);
	out.push(`<rect x="${minX}" y="${-maxY - legendH}" width="${W}" height="${H + legendH}" fill="#fbfbf7"/>`);
	out.push(`<g transform="scale(1,-1)">`);
	if (outlinePts.length) out.push(`<polygon points="${outlinePts.map(p => `${p.x},${p.y}`).join(' ')}" fill="#eef3e6" stroke="#8a2be2" stroke-width="4"/>`);
	for (const r of regions) {
		for (const sp of r.path ?? []) {
			const pts = parsePolygonPath(sp);
			if (pts.length >= 3) out.push(`<polygon points="${pts.map(p => `${p.x},${p.y}`).join(' ')}" fill="${r.layerId === 2 ? '#9ecae1' : '#fdae6b'}" fill-opacity="0.25" stroke="#e6550d" stroke-width="2" stroke-dasharray="10 6"/>`);
		}
	}
	// 飞线（每网 MST）
	for (const n of input.networks) {
		for (const [i, j] of mstEdges(n.pads)) {
			out.push(`<line x1="${n.pads[i].x}" y1="${n.pads[i].y}" x2="${n.pads[j].x}" y2="${n.pads[j].y}" stroke="#555" stroke-width="1.6" stroke-opacity="0.55"/>`);
		}
	}
	// 焊盘：底层先画，顶层后画，通孔最后
	const order = (p) => (p.layer === 2 ? 0 : p.layer === 1 ? 1 : 2);
	const shapeOf = (p) => input.padShapes[`${p.x.toFixed(2)},${p.y.toFixed(2)}`];
	for (const p of [...input.allPads].sort((a, b) => order(a) - order(b))) {
		const s = shapeOf(p);
		const kind = String(s?.pad?.[0] ?? 'ELLIPSE');
		const w = Number(s?.pad?.[1]) || p.radius * 2, h = Number(s?.pad?.[2]) || p.radius * 2;
		const rot = Number(s?.rotation) || 0;
		const fill = p.padType === 'th' ? '#2ca02c' : (p.layer === 2 ? '#1f77b4' : '#d62728');
		const stroke = p.net ? 'none' : '#000';
		const tr = `rotate(${rot} ${p.x} ${p.y})`;
		const op = p.layer === 2 ? 0.55 : 0.8;
		if (kind === 'ELLIPSE') out.push(`<ellipse cx="${p.x}" cy="${p.y}" rx="${w / 2}" ry="${h / 2}" transform="${tr}" fill="${fill}" fill-opacity="${op}" stroke="${stroke}" stroke-width="2"/>`);
		else {
			const rx = kind === 'OVAL' ? Math.min(w, h) / 2 : 0;
			out.push(`<rect x="${p.x - w / 2}" y="${p.y - h / 2}" width="${w}" height="${h}" rx="${rx}" transform="${tr}" fill="${fill}" fill-opacity="${op}" stroke="${stroke}" stroke-width="2"/>`);
		}
		if (p.holeDiameter > 0) out.push(`<circle cx="${p.x}" cy="${p.y}" r="${p.holeDiameter / 2}" fill="#fff"/>`);
	}
	out.push('</g>');
	// 位号（不翻转的文字层）
	for (const ci of componentsInfo) {
		out.push(`<text x="${ci.x}" y="${-ci.y}" font-size="26" fill="#333" text-anchor="middle" dominant-baseline="middle" opacity="0.8">${esc(ci.designator)}${ci.side === 'bottom' ? '(B)' : ''}</text>`);
	}
	// 图例
	const lx = minX + 30, ly = -maxY - legendH + 45;
	const items = [
		['#d62728', '顶层 SMD 焊盘'], ['#1f77b4', '底层 SMD 焊盘'], ['#2ca02c', '通孔焊盘（白点=钻孔）'],
	];
	out.push(`<text x="${lx}" y="${ly}" font-size="34" font-weight="bold" fill="#222">${esc(caseName)} — epcb2 离线 dump 预览（mil，Y 向上，从顶面看）</text>`);
	items.forEach(([c, t], i) => {
		out.push(`<rect x="${lx + i * 520}" y="${ly + 22}" width="34" height="24" fill="${c}"/>`);
		out.push(`<text x="${lx + i * 520 + 46}" y="${ly + 42}" font-size="26" fill="#222">${esc(t)}</text>`);
	});
	out.push(`<line x1="${lx}" y1="${ly + 80}" x2="${lx + 34}" y2="${ly + 80}" stroke="#555" stroke-width="3"/><text x="${lx + 46}" y="${ly + 88}" font-size="26" fill="#222">飞线（每网 MST）</text>`);
	out.push(`<rect x="${lx + 520}" y="${ly + 66}" width="34" height="24" fill="none" stroke="#000" stroke-width="3"/><text x="${lx + 566}" y="${ly + 88}" font-size="26" fill="#222">黑边 = 无网络焊盘</text>`);
	out.push(`<rect x="${lx + 1040}" y="${ly + 66}" width="34" height="24" fill="#fdae6b" fill-opacity="0.4" stroke="#e6550d" stroke-dasharray="6 4" stroke-width="2"/><text x="${lx + 1086}" y="${ly + 88}" font-size="26" fill="#222">禁布区 REGION（禁覆铜，未进障碍）</text>`);
	out.push(`<text x="${lx}" y="${ly + 134}" font-size="26" fill="#222">焊盘 ${input.allPads.length}（SMD ${smd} / 通孔 ${thp} / 无网络 ${v.added}）  网络 ${input.networks.length}（单焊盘 ${singlePadNets.length}）  器件 ${componentsInfo.length}（底层 ${componentsInfo.filter(c => c.side === 'bottom').length}）  障碍圆 ${input.staticObstacles.length}  铜层 ${input.copperLayers}</text>`);
	if (bbox) out.push(`<text x="${lx}" y="${ly + 170}" font-size="26" fill="#222">板框 ${(bbox.maxX - bbox.minX).toFixed(1)} × ${(bbox.maxY - bbox.minY).toFixed(1)} mil；线宽 ${input.drc.traceWidthMil.toFixed(2)} mil，Track↔Track ${input.drc.trackToTrackMil.toFixed(2)}，Track↔Pad ${input.drc.trackToPadMil.toFixed(2)}，板框 ${input.drc.trackToBoardMil.toFixed(2)} mil；自检 ${checks.every(c => c.ok) ? '全部通过' : '有失败'}</text>`);
	out.push('</svg>');
	fs.mkdirSync(path.dirname(path.resolve(file)), { recursive: true });
	fs.writeFileSync(file, out.join('\n'), 'utf8');
	console.log(`[epcb2] 预览 → ${path.resolve(file)}`);
}
if (svgOut) writeSvg(svgOut);

// ─────────────────────────────────────────────
// 7. 落盘（自检全部通过才写）
// ─────────────────────────────────────────────

// 已确认：与 2026-11-11 对 clock-pcb3 的 live CLI 导出逐字段比对（scripts/compare-dumps.mjs）一致的口径
const CONFIRMED = [
	'已对照 live CLI 导出（clock-pcb3，2026-11-11）：pcb_PrimitivePad.getAll() 含器件焊盘（212 个焊盘位置/层/类型全部对上，含 17 个无网络焊盘）',
	'已对照 live：底层器件的镜像（x→-x，再 R(θ-180°)，层 1↔2）与旋转口径 —— 42 个底层 SMD 焊盘位置与 rotation 一致',
	'已对照 live：RULE 数值为 mil；API 按规则 unit 返回（mm = mil × 0.0254），经生产 fetchDRCRules 后 drc 全部 8 个值与 live 位级相同',
	'已对照 live：DRC 表格格子 —— Track（行 0，唯一的 4.0157）、Copper/Plane Zone（行 7，唯一的 10）、Board Outline ∈ {行 8, 行 11}（二者列 0 都是 11.811）；'
		+ 'SMD Pad/TH Pad/Via/Line 各自的值（5.9843）只能确认它们不在行 7/8/11；默认规则选择（1oz 默认，而非 2oz 的 7.9921）',
	'已对照 live：Other Spacing 的 hole2Hole = holeClearance（mil）× 0.0254（mm）',
	'已对照 live：上下两层同位置同网的边缘焊盘是两个 SMD 焊盘（layer 1 与 layer 2），不是一个通孔',
	'已对照 live：坐标与焊盘宽高取整到 0.1 mil；rotation 归一到 (-180,180]（live 只出现 -90/0/90/180）；孔径不取整（15.748 等 3 位小数原样）',
	'已对照 live：ELLIPSE 焊盘 API 形态为 [ELLIPSE, 第一维 mm, 第二维 mil]（如 [ELLIPSE,1.3,51.2]，来自 51.181mil 的圆）',
	'已对照 live：焊盘 id = 器件 id + 封装内局部 id（独立焊盘 = 记录 id）；getAllPrimitivesByNet 焊盘字段名（type 1/2、center、globalIndex、defaultPadShapeAspect、holeAspect）',
	'已对照 live：网络顺序与逐网焊盘顺序 —— 独立 PAD（文件顺序）在前、器件焊盘（PAD_NET 文件顺序）在后，网络按首次出现顺序（64 网 195 焊盘完全一致）',
	'已对照 live：整个 input 的 JSON 序列化与 live 导出逐字节相同（含键顺序、焊盘 id、padShapes 键）',
	'已对照 live：layer 11 的 POLY（R 旋转矩形）板框 → 207 个板框障碍与 live 完全一致',
];
const INFERRED = [
	'ELLIPSE 第一维 mm 的取整位数：live 只有 0.6 / 1.3 / 2.0 三个值（取 0.1 与取 0.001 无法区分）；两维不等的 ELLIPSE 是否同样处理未知',
	'RECT 圆角半径、OVAL 宽高的取整（RECT/OVAL 宽高取 0.1 已对照；圆角半径只见过 0）；SLOT 孔的 API 形态；POLYGON 焊盘（宁可报错）',
	'DRC 表格：SMD Pad / TH Pad / Via / Line 各自对应的行、Board Outline 在行 8 还是行 11、Fill Region/Teardrop 的行未被 live 值区分；column/row 标签数组本身是推断的 API 形态',
	'规则 unit 非 mm（mil / inch / cm）时 API 的返回口径；hole2Hole 在 unit 非 mm 时生产的读取口径',
	'LINE / ARC / FILL / VIA 的 API 字段映射（clock-pcb3 不含这些图元，只经生产代码路径自检，未对照 live）；多子路径 FILL 按子路径拆成多个 Fill 图元的假设',
	'独立 PAD 与器件焊盘在 pcb_PrimitivePad.getAll() 中的相对顺序（只影响无网络焊盘的注入顺序；clock-pcb3 的无网络焊盘全是器件焊盘，顺序已对上）',
	'网络/焊盘顺序规律只在一块板上验证，背后的真实枚举机制未证实；NET 记录里没有焊盘的网放在 getAllNetsName 末尾（对 dump 无影响）',
	'封装内嵌的非焊盘铜图元（API getAll 不含，按生产口径不进障碍）、specialPad 按层异形、layerIdReplacements、封装 CANVAS 非零原点：均未处理（会 warn）',
];

console.log('\n[epcb2] 已确认项（对照 2026-11-11 clock-pcb3 的 live CLI 导出）:');
for (const s of CONFIRMED) console.log(`  + ${s}`);
console.log('\n[epcb2] 仍为推断/未验证:');
for (const s of INFERRED) console.log(`  - ${s}`);

const failed = checks.filter(c => !c.ok);
if (failed.length) {
	console.error(`\n[epcb2] 自检失败 ${failed.length} 项，不写 dump`);
	process.exit(1);
}
const dumpsDir = path.join(DATA_ROOT, 'tmp', 'dumps');
fs.mkdirSync(dumpsDir, { recursive: true });
const outFile = path.join(dumpsDir, `${caseName}-dump.json`);
const wrapped = {
	structVersion: STRUCT_VERSION,
	dumpedAt: new Date().toISOString(),
	caseName,
	totalPads: v.totalPads,
	added: v.added,
	nets: v.nets,
	source: 'epcb2-offline',
	sourceFile: path.relative(DATA_ROOT, srcFile),
	sourceSha256: srcSha,
	inferred: INFERRED,
	input,
};
fs.writeFileSync(outFile, JSON.stringify(wrapped, null, 1), 'utf8');
console.log(`\n[epcb2] dump → ${outFile}（焊盘 ${v.totalPads}，注入无网络 ${v.added}，网络 ${v.nets.length}，structVersion ${STRUCT_VERSION}）`);
