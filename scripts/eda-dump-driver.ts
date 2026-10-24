/**
 * EDA 内运行的 dump 驱动（源码）：采集 PlanningEngine 的 PlanningInput。
 *
 * 构建：node scripts/build-eda-dump.mjs → scripts/eda-dump-bundle.mjs
 * 调用：scripts/cli-dump62.js（data:URL import 本 bundle 后 setup(eda) + dumpInput）
 *
 * 复用生产源码（rubberband-engine 障碍构建器 + drc-rules 规则读取），零逻辑漂移。
 * 本模块不修改画布，只读枚举。
 */
import { buildPolylineObstacle, buildPolygonObstacle, RBObstacle } from '../src/router/rubberband-engine';
import { fetchDRCRules } from '../src/router/drc-rules';
import { isGroundNetName, isPowerNetName } from '../src/router/planning-engine';

export function setup(eda: any): void {
	(globalThis as any).eda = eda;
}

/** 实测时长睡眠：EDA 沙箱后台把 setTimeout 节流到 ~15ms/次，必须按墙钟补齐 */
async function sleep(ms: number): Promise<void> {
	const t = Date.now();
	while (Date.now() - t < ms) await new Promise(r => setTimeout(r, 0));
}

/** 打开并激活 PCB 文档，等待网络数据就绪（工程加载 + 文档解析都有延迟，全程重试） */
export async function ensurePcbTab(timeoutMs = 60000): Promise<void> {
	const eda = (globalThis as any).eda;
	const t0 = Date.now();
	let opened = false;
	for (;;) {
		// 工程加载完成后 getAllPcbsInfo 才有值；拿到 uuid 就打开 PCB 文档
		if (!opened) {
			try {
				const pcbs = await eda.dmt_Pcb.getAllPcbsInfo();
				const uuid = Array.isArray(pcbs) ? pcbs[0]?.uuid : undefined;
				if (uuid) {
					await eda.dmt_EditorControl.openDocument(uuid);
					opened = true;
				}
			} catch { /* 未就绪，继续等 */ }
		}
		try {
			const nets = await eda.pcb_Net.getAllNetsName();
			if (Array.isArray(nets)) return;
		} catch { /* 文档未就绪 */ }
		if (Date.now() - t0 > timeoutMs) throw new Error('等待 PCB 文档就绪超时');
		await sleep(500);
	}
}

function parsePath(polygon: any[]): Array<{ x: number; y: number }> {
	if (!polygon || polygon.length < 3) return [];
	if (polygon[0] === 'R' && polygon.length >= 5) {
		const x = polygon[1], y = polygon[2], w = polygon[3], h = polygon[4];
		return [
			{ x, y: y - h }, { x: x + w, y: y - h },
			{ x: x + w, y }, { x, y }, { x, y: y - h },
		];
	}
	const pts: Array<{ x: number; y: number }> = [];
	let i = 0;
	while (i < polygon.length) {
		if (typeof polygon[i] === 'number' && typeof polygon[i + 1] === 'number') {
			pts.push({ x: polygon[i], y: polygon[i + 1] });
			i += 2;
		} else i++;
	}
	return pts;
}

export interface DumpOptions {
	ripupRounds?: number;
	budgetMs?: number;
	optimize?: boolean;
	allowViaFallback?: boolean;
	allowViaInPad?: boolean;
	netOrder?: 'auto' | 'river' | 'reverse' | 'max-x' | 'longest' | 'shortest';
}

export async function dumpInput(opts: DumpOptions = {}) {
	const eda = (globalThis as any).eda;
	const drc = await fetchDRCRules();

	// 焊盘原始形状缓存（getAllPrimitivesByNet 的焊盘没有 pad/hole 字段）
	const padShapes: Record<string, { pad: any[]; hole: any[] | null; rotation: number }> = {};
	try {
		const padPrims = await eda.pcb_PrimitivePad.getAll();
		for (const sp of padPrims ?? []) {
			const p = sp as any;
			if (Array.isArray(p.pad)) {
				const key = `${(p.x ?? 0).toFixed(2)},${(p.y ?? 0).toFixed(2)}`;
				padShapes[key] = {
					pad: p.pad,
					hole: Array.isArray(p.hole) ? p.hole : null,
					rotation: p.rotation ?? 0,
				};
			}
		}
	} catch { /* 形状缺失时引擎用默认圆 */ }

	// 焊盘与网络（与 topological-router.collectPads 同口径）
	const allNets = await eda.pcb_Net.getAllNetsName();
	const padMap = new Map<string, any[]>();
	const allPads: any[] = [];
	for (const netName of allNets ?? []) {
		try {
			const primitives = await eda.pcb_Net.getAllPrimitivesByNet(netName);
			if (!primitives || primitives.length === 0) continue;
			const netPads: any[] = [];
			for (const prim of primitives) {
				const p = prim as any;
				if (p.type !== undefined && p.type !== 1) continue;
				const x = p.center?.x ?? p.x ?? p.startX ?? 0;
				const y = p.center?.y ?? p.y ?? p.startY ?? 0;
				if (x === 0 && y === 0 && !p.center) continue;
				const id = p.globalIndex || p.uuid || `${netName}_${allPads.length}`;
				const padShape = p.pad ?? p.defaultPadShapeAspect ?? [];
				let wMil = 0, hMil = 0;
				if (Array.isArray(padShape) && padShape.length >= 3) {
					wMil = padShape[1] ?? 0;
					hMil = padShape[2] ?? 0;
				}
				const hole = p.hole ?? p.holeAspect ?? [];
				const holeMil = Array.isArray(hole) && hole.length >= 2 ? (hole[1] ?? 0) : 0;
				// 形状缺失回退：先用形状缓存（位置精确匹配 / 2mil 邻近匹配）；
				// 再不行才用默认尺寸。盲猜 32×34mil 在 0.5mm 间距的引脚排里
				// 会让相邻焊盘障碍圈互相吞没，把每个端点封死（实测整板 0 布通）。
				if (wMil === 0 && hMil === 0) {
					const cached = padShapes[`${x.toFixed(2)},${y.toFixed(2)}`];
					let shape = cached?.pad;
					if (!Array.isArray(shape)) {
						let best: any = null, bd = 2;
						for (const k of Object.keys(padShapes)) {
							const [kx, ky] = k.split(',').map(Number);
							const dd = Math.hypot(kx - x, ky - y);
							if (dd < bd) { bd = dd; best = padShapes[k].pad; }
						}
						shape = best;
					}
					if (Array.isArray(shape) && shape.length >= 3) {
						wMil = shape[1] ?? 0;
						hMil = shape[2] ?? 0;
					}
				}
				if (wMil === 0 && hMil === 0) { wMil = 32; hMil = 34; }
				const rawLayer: number = p.layerId ?? p.layer ?? 1;
				let padType: 'th' | 'smd' | 'via' = 'smd';
				if (holeMil > 0 || rawLayer === 12) padType = 'th';
				else if (p.isVia || p.type === 'via') padType = 'via';
				let layer: number = rawLayer;
				if (padType === 'smd' && (rawLayer === 12 || rawLayer > 2)) layer = 1;
				const info = {
					x, y, net: netName, id, layer, padType, holeDiameter: holeMil,
					radius: Math.max(wMil / 2, hMil / 2, holeMil / 2),
					bodyRadius: Math.max(Math.min(wMil, hMil) / 2, holeMil / 2),
					index: allPads.length,
				};
				allPads.push(info);
				netPads.push(info);
			}
			if (netPads.length >= 1) padMap.set(netName, netPads);
		} catch { /* 单个网络读取失败不影响其它 */ }
	}

	// 静态障碍（与 collectStaticObstacles 同口径）
	const obstacles: RBObstacle[] = [];
	const lineClear = Math.max(drc.trackToLineMil, 8);
	const fillClear = Math.max(drc.trackToFillMil, 10);
	const polylines = await eda.pcb_PrimitivePolyline.getAll();
	for (const pl of polylines ?? []) {
		const p = pl as any;
		const layer: number = p.layerId ?? p.layer ?? 1;
		const lw = p.lineWidth ?? 10;
		const net: string = p.net ?? '';
		if (layer === 11) {
			const r = lw / 2 + drc.trackToBoardMil + drc.traceWidthMil / 2;
			const pts = parsePath(p.polygon?.polygon ?? p.polygon ?? []);
			if (pts.length >= 2) obstacles.push(...buildPolylineObstacle(pts, r, undefined, undefined, 'board', true));
			continue;
		}
		if (layer !== 1 && layer !== 2 && layer !== 12) continue;
		const r = lw / 2 + lineClear + drc.traceWidthMil / 2 + 2;
		const pts = parsePath(p.polygon?.polygon ?? p.polygon ?? []);
		if (pts.length >= 2) {
			obstacles.push(...buildPolylineObstacle(pts, r, net || undefined, layer, `poly:${pts[0]?.x},${pts[0]?.y}`));
		}
	}
	const lines = await eda.pcb_PrimitiveLine.getAll();
	for (const line of lines ?? []) {
		const l = line as any;
		const net: string = l.net ?? '';
		const layer: number = l.layerId ?? l.layer ?? 1;
		if (layer !== 1 && layer !== 2 && layer !== 12) continue;
		const lw = l.lineWidth ?? 10;
		const r = lw / 2 + lineClear + drc.traceWidthMil / 2 + 2;
		obstacles.push(...buildPolylineObstacle(
			[{ x: l.startX ?? 0, y: l.startY ?? 0 }, { x: l.endX ?? 0, y: l.endY ?? 0 }],
			r, net || undefined, layer, `line:${l.primitiveId ?? ''}`,
		));
	}
	const fills = await eda.pcb_PrimitiveFill.getAll();
	for (const fill of fills ?? []) {
		const f = fill as any;
		const layer: number = f.layerId ?? f.layer ?? 1;
		if (layer !== 1 && layer !== 2 && layer !== 12) continue;
		const lw = f.lineWidth ?? 0;
		const net: string = f.net ?? '';
		const r = lw / 2 + fillClear + drc.traceWidthMil / 2 + 2;
		const poly = f.complexPolygon?.polygon ?? f.complexPolygon ?? f.polygon?.polygon ?? f.polygon ?? [];
		const pts = parsePath(poly);
		if (pts.length >= 3) {
			obstacles.push(...buildPolygonObstacle(pts, r, net || undefined, layer, `fill:${pts[0]?.x},${pts[0]?.y}`, true));
		}
	}

	// 铜层数（停用内层过滤：layerStatus === 1）
	let copperLayers = 2;
	try {
		const all = await eda.pcb_Layer.getAllLayers();
		const copper = (all || []).filter((l: any) => l?.type === 'SIGNAL' && l.layerStatus === 1);
		if (copper.length >= 1) copperLayers = copper.length;
	} catch { /* 保守按双面板 */ }

	const networks = Array.from(padMap.entries()).map(([name, pads]) => ({ name, pads }));
	return {
		input: {
			drc,
			allPads,
			networks,
			staticObstacles: obstacles,
			padShapes,
			copperLayers,
			options: {
				ripupRounds: opts.ripupRounds ?? 5,
				optimize: opts.optimize ?? true,
				allowViaFallback: opts.allowViaFallback ?? true,
				allowViaInPad: opts.allowViaInPad ?? false,
				netOrder: opts.netOrder ?? 'auto',
			},
		},
		totalNetted: allPads.length,
		netCount: networks.length,
		groundNets: networks.filter(n => isGroundNetName(n.name)).length,
		powerNets: networks.filter(n => isPowerNetName(n.name) && !isGroundNetName(n.name)).length,
	};
}
