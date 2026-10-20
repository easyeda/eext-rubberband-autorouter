/**
 * 拓扑橡皮筋路由器（RBR 精确版）
 *
 * 算法（对齐 Tal Dayan 博士论文 / Stefan Salewski Ruby-PCB-Router 的效果）：
 * 1. 读取 DRC 规则 → 收集焊盘与障碍物（圆群模型）
 * 2. 可视图 + Dijkstra 求绕障碍凸包的最短路径（切线 + 圆弧，绳子拉紧效果）
 * 3. 已布走线加入动态障碍，保证后续走线不交叉、满足间距
 * 4. 走线以精确图元（直线 + 圆弧）写入 PCB
 */

import {
	RBObstacle, RBPrimitive, capsuleToCircles, padShapeToCircles,
	buildPolylineObstacle, buildPolygonObstacle, computeEscapePoint, computeEscapeAlternatives, pointSegDist, fanoutFillet, padExitModel,
} from './RubberbandEngine';
import { routeConnection } from './VisibilityRouter';
import { RoutingOptions, RoutingProgress } from '../types';

export type ProgressCallback = (progress: RoutingProgress) => void;
export type LogCallback = (message: string, level: string) => void;
export type StatusCallback = (status: 'connected' | 'disconnected') => void;

// PCB 单位：1mil = 0.0254mm
const MM_TO_MIL = 39.3701;
// 整轮布线的时间预算：超过后停止重试与撕毁重布，直接收尾。
// 协作式让出保证 UI 不冻结，但长尾重试仍会拖长总时长，需要硬上限。
const ROUTE_BUDGET_MS = 120000;

// DRC 回退默认值
const FALLBACK_TRACE_WIDTH_MIL = 8;
const FALLBACK_CLEARANCE_MIL = 8;

interface DRCRules {
	traceWidthMil: number;
	clearanceMil: number;
	/** Track↔Track 间距 */
	trackToTrackMil: number;
	/** Track↔Line 间距 */
	trackToLineMil: number;
	/** Track↔Board Outline 间距 */
	trackToBoardMil: number;
	/** Track↔SMD/TH Pad、Via 间距 */
	trackToPadMil: number;
	/** Track↔Fill/Zone 间距 */
	trackToFillMil: number;
}

export interface EnginePad {
	x: number;
	y: number;
	net: string;
	id: string;
	layer: number;       // 1 / 2 / 12（12 = 通孔多层）
	padType: 'th' | 'smd' | 'via';
	holeDiameter: number;
	radius: number;
	index: number;
}

interface RenderedPrim {
	kind: 'line' | 'arc';
	net: string;
	layer: number;
	// line
	x1?: number; y1?: number; x2?: number; y2?: number;
	// arc
	cx?: number; cy?: number; r?: number; startAngle?: number; sweep?: number;
}

/**
 * 确定两个焊盘之间的路由层（与旧版一致的规则）
 */
function getRoutingLayer(pad1: EnginePad, pad2: EnginePad): number {
	const l1 = pad1.layer, l2 = pad2.layer;
	if (l1 === 12 && l2 === 12) return 1;
	if (l1 === 12) return l2 === 2 ? 2 : 1;
	if (l2 === 12) return l1 === 2 ? 2 : 1;
	return l1;
}

/** 射线法：点是否在多边形内 */
function pointInPolygon(px: number, py: number, poly: Array<{ x: number; y: number }>): boolean {
	let inside = false;
	for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
		const xi = poly[i].x, yi = poly[i].y;
		const xj = poly[j].x, yj = poly[j].y;
		if (((yi > py) !== (yj > py)) && (px < ((xj - xi) * (py - yi)) / (yj - yi) + xi)) {
			inside = !inside;
		}
	}
	return inside;
}

// ─────────────────────────────────────────────

export class TopologicalRouter {
	private onProgress?: ProgressCallback;
	private onLog?: LogCallback;
	private onStatus?: StatusCallback;
	private isRouting = false;
	private shouldCancel = false;

	/** 已布走线的动态障碍（胶囊/圆群，含网络与层信息） */
	private dynamicObstacles: RBObstacle[] = [];
	private obstacleIdCounter = 0;
	/** 每网络已写入的 EDA 图元 ID（用于撕毁重布时删除） */
	private netPrimIds = new Map<string, string[]>();
	/** 每网络已写入的图元（几何） */
	private netRendered = new Map<string, RenderedPrim[]>();
	/** 已布网络顺序 */
	private routedOrder: string[] = [];

	constructor(
		onProgress?: ProgressCallback,
		onLog?: LogCallback,
		onStatus?: StatusCallback,
	) {
		this.onProgress = onProgress;
		this.onLog = onLog;
		this.onStatus = onStatus;
	}

	// ── 0. 从 DRC API 读取间距规则 ────────────────

	private async fetchDRCRules(): Promise<DRCRules> {
		try {
			const config = await eda.pcb_Drc.getCurrentRuleConfiguration();
			const cfg = (config as any)?.config;

			const spacing = cfg?.Spacing?.['Safe Spacing'];
			let trackToTrackMm = 0.1016;
			let trackToPadMm = 0.1524;
			let trackToLineMm = 0.1524;
			let trackToBoardMm = 0.3;
			let trackToFillMm = 0.254;

			if (spacing) {
				const tableKey = Object.keys(spacing).find(k => spacing[k].isSetDefault) ?? Object.keys(spacing)[0];
				const tbl = spacing[tableKey];
				const content: number[][] = tbl?.tables?.['1']?.content ?? [];
				const cols: string[] = tbl?.column ?? [];
				const rows: string[] = tbl?.row ?? [];
				const trackColIdx = cols.indexOf('Track');

				const readSpacing = (rowName: string): number | null => {
					const rowIdx = rows.indexOf(rowName);
					if (rowIdx < 0) return null;
					const val = content[rowIdx]?.[Math.min(trackColIdx, content[rowIdx]?.length - 1)];
					return typeof val === 'number' && val > 0 ? val : null;
				};

				trackToTrackMm = readSpacing('Track') ?? trackToTrackMm;
				const padVal = Math.max(
					readSpacing('SMD Pad') ?? 0,
					readSpacing('TH Pad') ?? 0,
					readSpacing('Via') ?? 0,
				);
				if (padVal > 0) trackToPadMm = padVal;
				trackToLineMm = readSpacing('Line') ?? trackToLineMm;
				trackToBoardMm = readSpacing('Board Outline') ?? trackToBoardMm;
				trackToFillMm = readSpacing('Copper/Plane Zone') ?? readSpacing('Fill Region/Teardrop') ?? trackToFillMm;
			}

			// 读取默认走线宽度
			const trackRules = cfg?.Physics?.Track;
			let traceWidthMm = 0.254;
			if (trackRules) {
				const defaultKey =
					Object.keys(trackRules).find(k => trackRules[k].isSetDefault) ??
					Object.keys(trackRules)[0];
				const tw = trackRules[defaultKey]?.form?.data?.['1']?.defaultValue;
				if (typeof tw === 'number' && tw > 0) traceWidthMm = tw;
			}

			const rules: DRCRules = {
				traceWidthMil: traceWidthMm * MM_TO_MIL,
				clearanceMil: Math.max(trackToTrackMm, trackToPadMm) * MM_TO_MIL,
				trackToTrackMil: trackToTrackMm * MM_TO_MIL,
				trackToLineMil: trackToLineMm * MM_TO_MIL,
				trackToBoardMil: trackToBoardMm * MM_TO_MIL,
				trackToPadMil: trackToPadMm * MM_TO_MIL,
				trackToFillMil: trackToFillMm * MM_TO_MIL,
			};

			this.onLog?.(
				`DRC 规则：线宽 ${rules.traceWidthMil.toFixed(2)}mil，Track间距 ${rules.trackToTrackMil.toFixed(2)}mil，` +
				`Pad间距 ${rules.trackToPadMil.toFixed(2)}mil，板框间距 ${rules.trackToBoardMil.toFixed(2)}mil`,
				'info',
			);
			return rules;
		} catch (err) {
			console.warn('[TopologicalRouter] 获取 DRC 规则失败，使用默认值:', err);
			this.onLog?.('读取 DRC 规则失败，使用默认值', 'warn');
			return {
				traceWidthMil: FALLBACK_TRACE_WIDTH_MIL,
				clearanceMil: FALLBACK_CLEARANCE_MIL,
				trackToTrackMil: FALLBACK_CLEARANCE_MIL,
				trackToLineMil: FALLBACK_CLEARANCE_MIL,
				trackToBoardMil: 12,
				trackToPadMil: FALLBACK_CLEARANCE_MIL,
				trackToFillMil: FALLBACK_CLEARANCE_MIL,
			};
		}
	}

	// ── 1. 收集焊盘与静态障碍 ────────────────────

	private async collectPads(): Promise<{
		padMap: Map<string, EnginePad[]>;
		allPads: EnginePad[];
	}> {
		const allNets = await eda.pcb_Net.getAllNetsName();
		if (!allNets || allNets.length === 0) {
			throw new Error('未找到任何网络，请确保 PCB 包含网络连接');
		}
		this.onLog?.(`找到 ${allNets.length} 个网络`, 'info');

		const padMap = new Map<string, EnginePad[]>();
		const allPads: EnginePad[] = [];

		for (const netName of allNets) {
			try {
				const primitives = await eda.pcb_Net.getAllPrimitivesByNet(netName);
				if (!primitives || primitives.length === 0) continue;

				const netPads: EnginePad[] = [];
				for (const prim of primitives) {
					const p = prim as any;
					if (p.type !== undefined && p.type !== 1) continue; // 只要焊盘
					const x: number = p.center?.x ?? p.x ?? p.startX ?? 0;
					const y: number = p.center?.y ?? p.y ?? p.startY ?? 0;
					if (x === 0 && y === 0 && !p.center) continue;
					const id: string = p.globalIndex || p.uuid || `${netName}_${allPads.length}`;

					const padShape = p.pad ?? [];
					let wMil = 0, hMil = 0;
					if (Array.isArray(padShape) && padShape.length >= 3) {
						wMil = padShape[1] ?? 0;
						hMil = padShape[2] ?? 0;
					}
					const hole = p.hole ?? [];
					const holeMil = Array.isArray(hole) && hole.length >= 2 ? (hole[1] ?? 0) : 0;
					if (wMil === 0 && hMil === 0) { wMil = 32; hMil = 34; }

					let padType: 'th' | 'smd' | 'via' = 'smd';
					if (holeMil > 0) padType = 'th';
					else if (p.isVia || p.type === 'via') padType = 'via';

					const rawLayer: number = p.layerId ?? p.layer ?? 1;
					let layer: number = rawLayer;
					if (padType === 'smd' && (rawLayer === 12 || rawLayer > 2)) layer = 1;

					const padRadius = Math.max(wMil / 2, hMil / 2, holeMil / 2);
					const info: EnginePad = {
						x, y, net: netName, id, layer,
						padType, holeDiameter: holeMil,
						radius: padRadius,
						index: allPads.length,
					};
					allPads.push(info);
					netPads.push(info);
				}

				if (netPads.length >= 1) padMap.set(netName, netPads);
			} catch (err) {
				console.warn(`[TopologicalRouter] 获取网络 ${netName} 失败:`, err);
			}
		}

		console.log(`[TopologicalRouter] 总计：${allPads.length} 个焊盘，${padMap.size} 个可布线网络`);
		return { padMap, allPads };
	}

	/** 收集静态铜层障碍（折线/线条/填充/板框），不含焊盘 */
	private async collectStaticObstacles(drc: DRCRules): Promise<RBObstacle[]> {
		const obstacles: RBObstacle[] = [];
		const parsePath = (polygon: any[]): Array<{ x: number; y: number }> => {
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
		};

		// 折线（含无网络走线与板框）：拐点圆可绕行 + 沿线阻挡圆
		const polylines = await eda.pcb_PrimitivePolyline.getAll();
		for (const pl of polylines) {
			const p = pl as any;
			const layer: number = p.layerId ?? p.layer ?? 1;
			const lw = p.lineWidth ?? 10;
			const net: string = p.net ?? '';

			if (layer === 11) {
				// 板框：阻挡所有层（layer=undefined）
				const r = lw / 2 + drc.trackToBoardMil + drc.traceWidthMil / 2;
				const pts = parsePath(p.polygon?.polygon ?? p.polygon ?? []);
				if (pts.length >= 2) {
					obstacles.push(...buildPolylineObstacle(pts, r, undefined, undefined, 'board', true));
				}
				continue;
			}
			if (layer !== 1 && layer !== 2 && layer !== 12) continue;
			const r = lw / 2 + drc.trackToLineMil + drc.traceWidthMil / 2;
			const pts = parsePath(p.polygon?.polygon ?? p.polygon ?? []);
			if (pts.length >= 2) {
				obstacles.push(...buildPolylineObstacle(pts, r, net || undefined, layer, `poly:${pts[0]?.x},${pts[0]?.y}`));
			}
		}

		// 线条
		const lines = await eda.pcb_PrimitiveLine.getAll();
		for (const line of lines) {
			const l = line as any;
			const net: string = l.net ?? '';
			const layer: number = l.layerId ?? l.layer ?? 1;
			if (layer !== 1 && layer !== 2 && layer !== 12) continue;
			const lw = l.lineWidth ?? 10;
			const r = lw / 2 + drc.trackToLineMil + drc.traceWidthMil / 2;
			obstacles.push(...buildPolylineObstacle(
				[{ x: l.startX ?? 0, y: l.startY ?? 0 }, { x: l.endX ?? 0, y: l.endY ?? 0 }],
				r, net || undefined, layer, `line:${l.primitiveId ?? ''}`,
			));
		}

		// 填充（铜皮矩形/多边形，无网络视为禁布）
		// 顶点圆可绕行（凸包切线沿边自然产生）；边线 + 内部网格圆仅阻挡，
		// 防止路径从填充中间穿过
		const fills = await eda.pcb_PrimitiveFill.getAll();
		for (const fill of fills) {
			const f = fill as any;
			const layer: number = f.layerId ?? f.layer ?? 1;
			if (layer !== 1 && layer !== 2 && layer !== 12) continue;
			const lw = f.lineWidth ?? 0;
			const net: string = f.net ?? '';
			const r = lw / 2 + drc.trackToFillMil + drc.traceWidthMil / 2;
			const poly = f.complexPolygon?.polygon ?? f.complexPolygon ?? f.polygon?.polygon ?? f.polygon ?? [];
			const pts = parsePath(poly);
			if (pts.length >= 3) {
				obstacles.push(...buildPolygonObstacle(pts, r, net || undefined, layer, `fill:${pts[0]?.x},${pts[0]?.y}`, true));
			}
		}

		this.onLog?.(`静态障碍：${obstacles.length} 个圆（阻挡 ${obstacles.filter(o => o.blockerOnly).length}）`, 'info');
		return obstacles;
	}

	/** 焊盘 → 障碍圆群（不含指定网络的焊盘） */
	private padObstacles(pads: EnginePad[], drc: DRCRules, excludeNet: string): RBObstacle[] {
		const out: RBObstacle[] = [];
		let id = 0;
		const infl = drc.trackToPadMil + drc.traceWidthMil / 2 + 0.3; // 安全余量
		for (const p of pads) {
			if (p.net === excludeNet) continue;
			const layer = p.layer === 12 ? undefined : p.layer; // 通孔阻挡所有层
			const group = `pad:${p.net}:${p.x},${p.y}`;
			const shapeKey = `${p.x.toFixed(2)},${p.y.toFixed(2)}`;
			const cached = this.padShapeCache.get(shapeKey);
			const shape = {
				pad: cached?.pad ?? ['ELLIPSE', p.radius * 2, p.radius * 2],
				hole: cached?.hole ?? (p.holeDiameter > 0 ? ['ROUND', p.holeDiameter] : null),
			};
			for (const c of padShapeToCircles(shape)) {
				out.push({ id: `p${id++}`, kind: 'circle', x: p.x + c.x, y: p.y + c.y, r: c.r + infl, net: p.net, layer, group });
			}
		}
		return out;
	}

	/** 焊盘原始形状缓存（按位置键，含 pad 与 hole 与 rotation） */
	private padShapeCache = new Map<string, { pad: any[]; hole: any[] | null; rotation: number }>();

	/** 渲染图元 → 动态障碍（端点圆可绕行 + 沿线阻挡圆；弧按整圆可绕行） */
	private addRenderedToObstacles(prims: RenderedPrim[], drc: DRCRules): void {
		const r = drc.traceWidthMil / 2 + drc.trackToTrackMil + drc.traceWidthMil / 2 + 0.1;
		for (const pr of prims) {
			const group = `trk:${this.obstacleIdCounter++}`;
			if (pr.kind === 'line') {
				// 直线：端点圆可绕行 + 沿线阻挡圆（保持图规模小）
				this.dynamicObstacles.push(...buildPolylineObstacle(
					[{ x: pr.x1!, y: pr.y1! }, { x: pr.x2!, y: pr.y2! }],
					r, pr.net, pr.layer, group,
				));
			} else {
				// 弧：沿弧采样为折线圆群（全部可绕行，形成平滑河流）
				const nSeg = Math.max(2, Math.ceil(Math.abs(pr.sweep!) / 0.35));
				const arcPts: Array<{ x: number; y: number }> = [];
				for (let sIdx = 0; sIdx <= nSeg; sIdx++) {
					const a = pr.startAngle! + (pr.sweep! * sIdx) / nSeg;
					arcPts.push({ x: pr.cx! + pr.r! * Math.cos(a), y: pr.cy! + pr.r! * Math.sin(a) });
				}
				this.dynamicObstacles.push(...buildPolylineObstacle(arcPts, r, pr.net, pr.layer, group));
			}
		}
	}

	// ── 主流程 ──────────────────────────────────

	async route(options: RoutingOptions): Promise<boolean> {
		if (this.isRouting) throw new Error('布线正在进行中');
		this.isRouting = true;
		this.shouldCancel = false;
		this.dynamicObstacles = [];
		this.obstacleIdCounter = 0;
		this.netPrimIds.clear();
		this.netRendered.clear();
		this.routedOrder = [];
		this.escapeReservationCache = null;

		try {
			// Step 0: DRC 规则
			this.onProgress?.({ stage: '读取规则', percentage: 3, message: '正在读取 DRC 间距规则...' });
			const drc = await this.fetchDRCRules();

			// 缓存焊盘原始形状（按位置键；getAllPrimitivesByNet 的焊盘没有 pad/hole 字段，
			// 必须用 pcb_PrimitivePad.getAll() 的完整数据补充）
			try {
				const padPrims = await eda.pcb_PrimitivePad.getAll();
				for (const sp of padPrims) {
					const p = sp as any;
					if (Array.isArray(p.pad)) {
						const key = `${(p.x ?? 0).toFixed(2)},${(p.y ?? 0).toFixed(2)}`;
						this.padShapeCache.set(key, { pad: p.pad, hole: Array.isArray(p.hole) ? p.hole : null, rotation: p.rotation ?? 0 });
					}
				}
			} catch { /* 形状缓存失败则用默认圆 */ }

			// Step 1: 收集焊盘
			this.onProgress?.({ stage: '解析 PCB', percentage: 8, message: '正在获取 PCB 数据...' });
			const { padMap, allPads } = await this.collectPads();
			if (padMap.size === 0) throw new Error('没有可布线的网络');

			// Step 2: 静态障碍
			this.onProgress?.({ stage: '构建障碍', percentage: 15, message: '正在构建障碍物...' });
			const staticObstacles = await this.collectStaticObstacles(drc);
			this.onLog?.(`障碍统计：静态 ${staticObstacles.length}（阻挡 ${staticObstacles.filter(o => o.blockerOnly).length}，noPrune ${staticObstacles.filter(o => o.noPrune).length}）`, 'info');

			// Step 3: 网络排序（电源优先，其余按几何位置河流式排序）
			const networks = Array.from(padMap.entries()).map(([name, pads]) => ({ name, pads }));
			networks.sort((a, b) => {
				const pa = a.pads, pb = b.pads;
				const isPowerA = this.isPowerNet(a.name), isPowerB = this.isPowerNet(b.name);
				if (isPowerA && !isPowerB) return -1;
				if (!isPowerA && isPowerB) return 1;
				// 河流式排序：按最左焊盘升序（内层先布），同列按右边界升序
				const aMin = Math.min(...pa.map(p => p.x)), aMax = Math.max(...pa.map(p => p.x));
				const bMin = Math.min(...pb.map(p => p.x)), bMax = Math.max(...pb.map(p => p.x));
				if (aMin !== bMin) return aMin - bMin;
				return aMax - bMax;
			});

			// 心跳监测：记录主线程被同步阻塞超过 300ms 的阶段（诊断卡死用）
			{
				const blocks: Array<{ gap: number; phase: string }> = [];
				(globalThis as any).__RB_BLOCKS__ = blocks;
				let lastBeat = Date.now();
				const hb = setInterval(() => {
					const now = Date.now();
					const gap = now - lastBeat;
					if (gap > 300) blocks.push({ gap, phase: (globalThis as any).__RB_PHASE__ ?? '?' });
					lastBeat = now;
				}, 100);
				(globalThis as any).__RB_HB__ = hb;
			}

			// Step 4: 逐网络布线
			const createdCounter = { lines: 0, vias: 0 };
			const routeDeadline = Date.now() + ROUTE_BUDGET_MS;
			const pendingNets: Array<{ name: string; pads: EnginePad[] }> = [];

			for (let ni = 0; ni < networks.length; ni++) {
				if (this.shouldCancel) break;
				if (Date.now() > routeDeadline) {
					this.onLog?.('布线时间预算用尽，跳过剩余网络', 'warn');
					break;
				}
				const net = networks[ni];
				(globalThis as any).__RB_PHASE__ = 'router:net:' + net.name;
				const pct = 20 + Math.floor((ni / networks.length) * 55);
				this.onProgress?.({ stage: '布线中', percentage: pct, message: `布线网络 ${ni + 1}/${networks.length}: ${net.name}` });

				(globalThis as any).__RB_PHASE__ = 'router:' + net.name;
				const ok = await this.routeNetEdges(net, staticObstacles, allPads, drc, createdCounter);
				if (!ok) pendingNets.push(net);
			}

			// 多轮撕毁重布
			for (let pass = 0; pass < 2 && pendingNets.length; pass++) {
				if (Date.now() > routeDeadline) {
					this.onLog?.('布线时间预算用尽，停止撕毁重布', 'warn');
					break;
				}
				const next: typeof pendingNets = [];
				for (const net of pendingNets) {
					if (this.shouldCancel) break;
					const ends = [net.pads[0], net.pads[net.pads.length - 1]];
					const seals = new Set<string>();
					for (const o of this.dynamicObstacles) {
						if (o.net && o.net !== net.name &&
							ends.some(e => Math.hypot(o.x - e.x, o.y - e.y) < o.r + 70)) {
							seals.add(o.net);
						}
					}
					const rip = this.routedOrder.filter(n => seals.has(n)).slice(-3);
					if (!rip.length) { next.push(net); continue; }
					this.onLog?.(`第 ${pass + 1} 轮：网络 ${net.name} 撕毁 ${rip.join(', ')} 重试`, 'warn');
					for (const n of rip) await this.ripupNet(n);
					(globalThis as any).__RB_PHASE__ = 'router:' + net.name;
				const ok = await this.routeNetEdges(net, staticObstacles, allPads, drc, createdCounter);
					let routed = ok;
					for (let i = rip.length - 1; i >= 0; i--) {
						const ripped = networks.find(nn => nn.name === rip[i])!;
						const rok = await this.routeNetEdges(ripped, staticObstacles, allPads, drc, createdCounter);
						if (!rok) next.push(ripped);
					}
					if (!routed) next.push(net);
				}
				pendingNets.length = 0;
				pendingNets.push(...next);
			}

			// 最终成组重布：拆除所有封路网，失败网按外向优先（maxX 降序）重布，再重布被拆网
			if (pendingNets.length && Date.now() <= routeDeadline) {
				const sealsAll = new Set<string>();
				for (const net of pendingNets) {
					const [pa, pb] = [net.pads[0], net.pads[net.pads.length - 1]];
					for (const o of this.dynamicObstacles) {
						if (o.net && o.net !== net.name &&
							(Math.hypot(o.x - pa.x, o.y - pa.y) < o.r + 70 ||
							 Math.hypot(o.x - pb.x, o.y - pb.y) < o.r + 70)) {
							sealsAll.add(o.net);
						}
					}
				}
				for (const n of [...sealsAll]) if (pendingNets.some(pn => pn.name === n)) sealsAll.delete(n);
				const rip = this.routedOrder.filter(n => sealsAll.has(n));
				if (rip.length) {
					this.onLog?.(`成组重布：拆除 ${rip.join(', ')}`, 'warn');
					for (const n of rip) await this.ripupNet(n);
					const ordered = [...pendingNets].sort((na, nb) => {
						const xa = Math.max(...na.pads.map(p => p.x));
						const xb = Math.max(...nb.pads.map(p => p.x));
						return xb - xa;
					});
					for (const net of ordered) {
						if (!(await this.routeNetEdges(net, staticObstacles, allPads, drc, createdCounter))) {
							this.onLog?.(`成组重布：${net.name} 仍失败`, 'error');
						}
					}
					for (const n of rip) {
						const ripped = networks.find(nn => nn.name === n)!;
						if (!(await this.routeNetEdges(ripped, staticObstacles, allPads, drc, createdCounter))) {
							this.onLog?.(`成组重布：重布 ${n} 失败`, 'error');
						}
					}
					pendingNets.length = 0;
				}
			}
			try { const hb = (globalThis as any).__RB_HB__; if (hb) clearInterval(hb); } catch { /* ignore */ }

			// 以实际已布网络集合为准统计（撕毁重布不会重复计数）
			const routedNets = this.routedOrder.length;
			const failedNets = networks.filter(n => !this.routedOrder.includes(n.name)).map(n => n.name);

			this.onProgress?.({ stage: '布线完成', percentage: 98, message: '布线完成' });
			this.onLog?.(
				`布线完成：${routedNets}/${networks.length} 个网络，写入 ${createdCounter.lines} 段走线，${createdCounter.vias} 个过孔` +
				(failedNets.length ? `；失败网络：${failedNets.join(', ')}` : ''),
				failedNets.length ? 'warn' : 'success',
			);

			this.isRouting = false;
			this.onStatus?.('disconnected');
			return failedNets.length === 0 && createdCounter.lines > 0;

		} catch (error) {
			console.error('[TopologicalRouter] 布线错误:', error);
			this.onLog?.('布线失败: ' + (error instanceof Error ? error.message : String(error)), 'error');
			this.isRouting = false;
			this.onStatus?.('disconnected');
			throw error;
		}
	}

	/**
	 \* 单焊盘逃逸点：SMD 长焊盘从端帽沿长轴垂直出线（考虑长宽/旋转），
	 \* 圆形/方形焊盘退回 scored 方向。返回 null 表示该方向被堵。
	 */
	private padEscape(
		pad: { x: number; y: number; radius: number; layer: number },
		shape: { pad: any[]; hole: any[] | null; rotation: number } | undefined,
		obstacles: RBObstacle[],
		net: string,
		toward: { x: number; y: number },
		escClear: number,
		routingLayer: number,
	): { x: number; y: number } | null {
		if (shape) {
			const em = padExitModel(pad.x, pad.y, { pad: shape.pad, hole: shape.hole, rotation: shape.rotation }, toward);
			if (em) {
				const tipX = em.capX + em.axisX * (em.r + escClear);
				const tipY = em.capY + em.axisY * (em.r + escClear);
				// 尖端与出线段不得侵入异网络障碍
				let ok = true;
				for (const o of obstacles) {
					if (o.net === net) continue;
					if (o.layer !== undefined && o.layer !== routingLayer) continue;
					if (Math.hypot(tipX - o.x, tipY - o.y) < o.r - 0.05) { ok = false; break; }
					if (pointSegDist(o.x, o.y, pad.x, pad.y, tipX, tipY) < o.r - 0.05) { ok = false; break; }
				}
				if (ok) return { x: tipX, y: tipY };
				return null; // 轴向被堵：SMD 长焊盘不做斜出（保持垂直扇出美观）
			}
		}
		return computeEscapePoint(pad.x, pad.y, pad.radius, obstacles, net, routingLayer, escClear, toward);
	}

	/** 逃逸通道预约：每个焊盘的出线段注册为该网络的专属阻挡（其他网络不得横穿） */
	private escapeReservationCache: RBObstacle[] | null = null;
	private getEscapeReservations(allPads: EnginePad[], drc: DRCRules): RBObstacle[] {
		if (this.escapeReservationCache) return this.escapeReservationCache;
		const out: RBObstacle[] = [];
		const rRes = drc.traceWidthMil + drc.trackToTrackMil + 0.1;
		const escClear = drc.trackToPadMil + drc.traceWidthMil + 20;
		const padObstaclesAll = this.padObstacles(allPads, drc, ' none');
		for (const p of allPads) {
			const layer = p.layer === 12 ? undefined : p.layer;
			const obs = padObstaclesAll.filter(o => o.layer === undefined || o.layer === layer);
			const shape = this.padShapeCache.get(`${p.x.toFixed(2)},${p.y.toFixed(2)}`);
			// 预约方向的 sideHint：暂无目标信息，取当前逃逸点方向（先 scored 再覆盖）
			const esc0 = computeEscapePoint(p.x, p.y, p.radius, obs, p.net, layer ?? 1, escClear);
			if (!esc0) continue;
			const esc = this.padEscape(p, shape, obs, p.net, { x: esc0.x - p.x, y: esc0.y - p.y }, escClear, layer ?? 1) ?? esc0;
			if (!esc) continue;
			for (const c of capsuleToCircles(p.x, p.y, esc.x, esc.y, rRes)) {
				out.push({ id: `res${out.length}`, kind: 'circle', x: c.x, y: c.y, r: rRes, net: p.net, layer, group: `res:${p.net}:${p.x},${p.y}`, blockerOnly: true });
			}
		}
		this.escapeReservationCache = out;
		return out;
	}

	/**
	 * 布一个网络的全部 MST 连接（含逃逸段与撕毁重布支持的图元跟踪）
	 */
	private async routeNetEdges(
		net: { name: string; pads: EnginePad[] },
		staticObstacles: RBObstacle[],
		allPads: EnginePad[],
		drc: DRCRules,
		createdCounter: { lines: number; vias: number },
	): Promise<boolean> {
		const edges = this.computeMST(net.pads);
		let netOk = true;

		for (const [si, di] of edges) {
			if (this.shouldCancel) return false;
			const src = net.pads[si], dst = net.pads[di];
			const routingLayer = getRoutingLayer(src, dst);

			const layerFiltered = [
				...staticObstacles,
				...this.padObstacles(allPads, drc, net.name),
				...this.dynamicObstacles,
				...this.getEscapeReservations(allPads, drc),
			].filter(o => o.layer === undefined || o.layer === routingLayer);

			// 逃逸点：SMD 长焊盘从端帽沿长轴垂直出线（扇出效果+防封死相邻焊盘）；圆形焊盘沿 scored 方向
			const escClear = drc.trackToPadMil + drc.traceWidthMil + 20;
			const shapeSrc = this.padShapeCache.get(`${src.x.toFixed(2)},${src.y.toFixed(2)}`);
			const shapeDst = this.padShapeCache.get(`${dst.x.toFixed(2)},${dst.y.toFixed(2)}`);
			const escSrc = this.padEscape(src, shapeSrc, layerFiltered, net.name, dst, escClear, routingLayer)
				?? computeEscapePoint(src.x, src.y, src.radius, layerFiltered, net.name, routingLayer, escClear, dst);
			const escDst = this.padEscape(dst, shapeDst, layerFiltered, net.name, src, escClear, routingLayer)
				?? computeEscapePoint(dst.x, dst.y, dst.radius, layerFiltered, net.name, routingLayer, escClear, src);

			let stub1: RenderedPrim | null = null;
			let stub2: RenderedPrim | null = null;
			let result: { ok: boolean; prims: RBPrimitive[] } = { ok: false, prims: [] };
			if (escSrc && escDst) {
				stub1 = { kind: 'line', net: net.name, layer: routingLayer, x1: src.x, y1: src.y, x2: escSrc.x, y2: escSrc.y };
				stub2 = { kind: 'line', net: net.name, layer: routingLayer, x1: escDst.x, y1: escDst.y, x2: dst.x, y2: dst.y };
				let stubsOK = true;
				for (const st of [stub1, stub2]) {
					for (const o of layerFiltered) {
						if (o.net === net.name) continue;
						if (pointSegDist(o.x, o.y, st.x1!, st.y1!, st.x2!, st.y2!) < o.r - 0.05) {
							stubsOK = false;
							break;
						}
					}
					if (!stubsOK) break;
				}
				if (stubsOK) {
					result = await routeConnection(escSrc, escDst, layerFiltered, net.name, [150, 500]);
				}
				// 首选逃逸方向失败：尝试备选逃逸方向（次优/第三优）
				if ((!result.ok || result.prims.length === 0) && escSrc && escDst) {
					const alts = computeEscapeAlternatives(src.x, src.y, src.radius, layerFiltered, net.name, routingLayer, escClear, dst, 3);
					const altsD = computeEscapeAlternatives(dst.x, dst.y, dst.radius, layerFiltered, net.name, routingLayer, escClear, src, 3);
					outer: for (const a1 of alts.slice(1)) {
						for (const a2 of altsD.slice(1)) {
							const s1 = { kind: 'line' as const, net: net.name, layer: routingLayer, x1: src.x, y1: src.y, x2: a1.x, y2: a1.y };
							const s2 = { kind: 'line' as const, net: net.name, layer: routingLayer, x1: a2.x, y1: a2.y, x2: dst.x, y2: dst.y };
							let ok2 = true;
							for (const st of [s1, s2]) {
								for (const o of layerFiltered) {
									if (o.net === net.name) continue;
									if (pointSegDist(o.x, o.y, st.x1!, st.y1!, st.x2!, st.y2!) < o.r - 0.05) { ok2 = false; break; }
								}
								if (!ok2) break;
							}
							if (ok2) {
								const r2 = await routeConnection(a1, a2, layerFiltered, net.name, [150]);
								if (r2.ok && r2.prims.length > 0) {
									stub1 = s1; stub2 = s2; result = r2;
									break outer;
								}
							}
						}
					}
				}
			}
			if (!result.ok || result.prims.length === 0) {
				stub1 = stub2 = null;
				result = await routeConnection({ x: src.x, y: src.y }, { x: dst.x, y: dst.y }, layerFiltered, net.name);
			}

			if (!result.ok || result.prims.length === 0) {
				this.onLog?.(`网络 ${net.name}: ${src.id}(${src.x.toFixed(0)},${src.y.toFixed(0)}) → ${dst.id}(${dst.x.toFixed(0)},${dst.y.toFixed(0)}) 未找到路径 (layerFiltered=${layerFiltered.length})`, 'warn');
				return false;
			}

			// 写入 EDA
			const rendered: RenderedPrim[] = [];
			const primIds: string[] = [];
			try {
				for (const st of [stub1, stub2]) {
					if (!st) continue;
					const createdLine = await eda.pcb_PrimitiveLine.create(net.name, routingLayer, st.x1!, st.y1!, st.x2!, st.y2!, drc.traceWidthMil, false);
					const pid = (createdLine as any)?.primitiveId;
					if (pid) primIds.push(pid);
					rendered.push(st);
					createdCounter.lines++;
				}
				for (const pr of result.prims) {
					if (pr.kind === 'line') {
						const createdLine = await eda.pcb_PrimitiveLine.create(
							net.name, routingLayer, pr.x1, pr.y1, pr.x2, pr.y2, drc.traceWidthMil, false,
						);
						const pid = (createdLine as any)?.primitiveId;
						if (pid) primIds.push(pid);
						rendered.push({ kind: 'line', net: net.name, layer: routingLayer, x1: pr.x1, y1: pr.y1, x2: pr.x2, y2: pr.y2 });
					} else {
						const sx = pr.cx + pr.r * Math.cos(pr.startAngle);
						const sy = pr.cy + pr.r * Math.sin(pr.startAngle);
						const ex = pr.cx + pr.r * Math.cos(pr.startAngle + pr.sweep);
						const ey = pr.cy + pr.r * Math.sin(pr.startAngle + pr.sweep);
								// EDA 两点式圆弧角度约定与数学正方向一致（实测：正角=逆时针凸向南）
								const deg = pr.sweep * 180 / Math.PI;
						if (Math.abs(deg) < 0.05 || Math.abs(Math.abs(deg) - 360) < 0.05) continue;
						const createdArc = await eda.pcb_PrimitiveArc.create(net.name, routingLayer, sx, sy, ex, ey, deg, drc.traceWidthMil);
						const pid = (createdArc as any)?.primitiveId;
						if (pid) primIds.push(pid);
						rendered.push({ kind: 'arc', net: net.name, layer: routingLayer, cx: pr.cx, cy: pr.cy, r: pr.r, startAngle: pr.startAngle, sweep: pr.sweep });
					}
					createdCounter.lines++;
				}
			} catch (err) {
				const emsg = err instanceof Error ? err.message : String(err);
				console.warn(`[TopologicalRouter] 创建走线失败 (${net.name}):`, err);
				this.onLog?.(`网络 ${net.name}: 走线写入失败: ${emsg.slice(0, 120)}`, 'error');
				// 回滚本次已创建的图元，避免残留孤儿走线变成障碍
				for (const pid of primIds) {
					try { await eda.pcb_PrimitiveLine.delete(pid); } catch {
						try { await (eda as any).pcb_PrimitiveArc.delete(pid); } catch { /* 忽略 */ }
					}
				}
				return false;
			}

			// 过孔：跨层时在终点创建
			const srcLayer = src.layer === 12 ? routingLayer : src.layer;
			const dstLayer = dst.layer === 12 ? routingLayer : dst.layer;
			if (srcLayer !== dstLayer) {
				try {
					const viaD = Math.max(20, drc.traceWidthMil * 2);
					const viaHole = Math.max(10, drc.traceWidthMil);
					await eda.pcb_PrimitiveVia.create(net.name, dst.x, dst.y, viaHole, viaD);
					createdCounter.vias++;
				} catch (err) {
					console.warn(`[TopologicalRouter] 创建过孔失败 (${net.name}):`, err);
				}
			}

			// 扇出圆角：出线段/入线段与路径首末段之间插入切线圆弧
			try {
				const first = rendered.find(r2 => r2.kind === 'line' && r2 !== stub1 && r2 !== stub2);
				if (stub1 && first && (first as any).x1 === stub1.x2 && (first as any).y1 === stub1.y2) {
					const f1 = fanoutFillet(
						{ x1: stub1.x1!, y1: stub1.y1!, x2: stub1.x2!, y2: stub1.y2! },
						{ x1: (first as any).x1, y1: (first as any).y1, x2: (first as any).x2, y2: (first as any).y2 },
						Math.min(30, drc.traceWidthMil * 3),
					);
					if (f1) {
						const l0 = f1.prims[0], ar = f1.prims[1], l2 = f1.prims[2];
						if (l0.kind === 'line' && ar.kind === 'arc' && l2.kind === 'line') {
							const fi = rendered.indexOf(first);
							rendered[fi] = { kind: 'line', net: net.name, layer: routingLayer, x1: l0.x1, y1: l0.y1, x2: l0.x2, y2: l0.y2 };
							rendered.splice(fi + 1, 0,
								{ kind: 'arc', net: net.name, layer: routingLayer, cx: ar.cx, cy: ar.cy, r: ar.r, startAngle: ar.startAngle, sweep: ar.sweep },
								{ kind: 'line', net: net.name, layer: routingLayer, x1: l2.x1, y1: l2.y1, x2: l2.x2, y2: l2.y2 });
							// EDA 侧同步：删旧首段，画圆角新段
							const oldId = primIds.shift();
							if (oldId) { try { await eda.pcb_PrimitiveLine.delete(oldId); } catch { /* ignore */ } }
							for (const nl of [l0, l2]) {
								const cl = await eda.pcb_PrimitiveLine.create(net.name, routingLayer, nl.x1, nl.y1, nl.x2, nl.y2, drc.traceWidthMil, false);
								const nid = (cl as any)?.primitiveId;
								if (nid) primIds.unshift(nid);
								createdCounter.lines++;
							}
							const ca = await (eda as any).pcb_PrimitiveArc.create(net.name, routingLayer,
								ar.cx + ar.r * Math.cos(ar.startAngle),
								ar.cy + ar.r * Math.sin(ar.startAngle),
								ar.cx + ar.r * Math.cos(ar.startAngle + ar.sweep),
								ar.cy + ar.r * Math.sin(ar.startAngle + ar.sweep),
								ar.sweep * 180 / Math.PI, drc.traceWidthMil);
							const caid = (ca as any)?.primitiveId;
							if (caid) primIds.splice(1, 0, caid);
							createdCounter.lines++;
						}
					}
				}
			} catch { /* 圆角失败不影响主流程 */ }

			// 记录并加入动态障碍
			const ids = this.netPrimIds.get(net.name) ?? [];
			ids.push(...primIds);
			this.netPrimIds.set(net.name, ids);
			const rds = this.netRendered.get(net.name) ?? [];
			rds.push(...rendered);
			this.netRendered.set(net.name, rds);
			this.addRenderedToObstacles(rendered, drc);
		}

		if (netOk && !this.routedOrder.includes(net.name)) this.routedOrder.push(net.name);
		return netOk;
	}

	/**
	 * 撕毁网络：删除其 EDA 图元与动态障碍
	 */
	private async ripupNet(netName: string): Promise<void> {
		const ids = this.netPrimIds.get(netName) ?? [];
		for (const pid of ids) {
			try {
				await eda.pcb_PrimitiveLine.delete(pid);
			} catch {
				try { await (eda as any).pcb_PrimitiveArc.delete(pid); } catch { /* 已删除 */ }
			}
		}
		this.netPrimIds.delete(netName);
		this.netRendered.delete(netName);
		this.dynamicObstacles = this.dynamicObstacles.filter(o => o.net !== netName);
		const ro = this.routedOrder.indexOf(netName);
		if (ro >= 0) this.routedOrder.splice(ro, 1);
	}

	private isPowerNet(name: string): boolean {
		const upper = name.toUpperCase();
		return upper === 'GND' || upper === 'VCC' || upper === 'VDD' ||
			upper.startsWith('GND') || upper.startsWith('VCC') ||
			upper.startsWith('VDD') || upper.startsWith('PWR') ||
			upper.startsWith('+') || upper.startsWith('-');
	}

	/** Prim MST：网络内连接顺序 */
	private computeMST(pads: EnginePad[]): Array<[number, number]> {
		const n = pads.length;
		if (n < 2) return [];
		const inMST = new Array<boolean>(n).fill(false);
		const minDist = new Array<number>(n).fill(Infinity);
		const minFrom = new Array<number>(n).fill(-1);
		minDist[0] = 0;
		const edges: Array<[number, number]> = [];
		for (let iter = 0; iter < n; iter++) {
			let u = -1;
			for (let i = 0; i < n; i++) {
				if (!inMST[i] && (u === -1 || minDist[i] < minDist[u])) u = i;
			}
			inMST[u] = true;
			if (minFrom[u] !== -1) edges.push([minFrom[u], u]);
			for (let v = 0; v < n; v++) {
				if (inMST[v]) continue;
				const d = Math.hypot(pads[u].x - pads[v].x, pads[u].y - pads[v].y);
				if (d < minDist[v]) { minDist[v] = d; minFrom[v] = u; }
			}
		}
		return edges;
	}

	cancel(): void {
		if (this.isRouting) {
			this.shouldCancel = true;
			this.onLog?.('正在取消布线...', 'warn');
		}
	}

	isActive(): boolean { return this.isRouting; }
}
