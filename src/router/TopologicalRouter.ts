/**
 * 拓扑橡皮筋路由器（增强版）
 * 算法：读取 DRC 规则 → Delaunay 三角剖分建图 → A* 寻路（避障）→ 橡皮筋几何修正 → 路径优化
 * 
 * 改进内容：
 * 1. 增强 PCB 数据解析 - 支持更多焊盘类型和元件
 * 2. 完整的布线结果导入 - 支持过孔生成
 * 3. 优化 Dijkstra 算法 - 改进启发式，提高路由成功率
 * 4. 路径优化功能 - 橡皮筋算法优化路径质量
 * 5. 增强错误处理 - 重试机制和详细错误信息
 */

import Delaunator from 'delaunator';
import { ConnectionStatus, RoutingOptions, RoutingProgress } from '../types';
import { PathOptimizer } from './PathOptimizer';
import { DRCChecker } from './DRCChecker';
import { ObstacleBypasser } from './ObstacleBypasser';
import { RoutingOrchestrator } from './RoutingOrchestrator';
import {
	ErrorHandler,
	ErrorType,
	ErrorSeverity,
	createStandardError,
	ErrorMessages,
	RoutingError,
} from '../utils/ErrorHandler';

export type ProgressCallback = (progress: RoutingProgress) => void;
export type LogCallback = (message: string, level: string) => void;
export type StatusCallback = (status: ConnectionStatus) => void;

// PCB 单位：1mil = 0.0254mm
const MM_TO_MIL = 39.3701;
const EXTRA_MARGIN = 2; // mils — 绕障额外余量

// DRC 回退默认值
const FALLBACK_TRACE_WIDTH_MIL = 8;
const FALLBACK_CLEARANCE_MIL = 8;

interface DRCRules {
	traceWidthMil: number;
	clearanceMil: number;
}

interface PadInfo {
	x: number;
	y: number;
	net: string;
	id: string;
	layer: number;
	exclusion: number;
	index: number;
	padType: 'th' | 'smd' | 'via';
	holeDiameter: number;
	radius: number;
}

interface Point { 
	x: number; 
	y: number; 
	layer?: number;
}

interface TrackPolyline {
	net: string;
	points: Point[];
	layer: number;
	width: number;
}

interface Obstacle {
	x: number;
	y: number;
	radius: number;
	layer?: number;
	type: 'pad' | 'via' | 'keepout' | 'track';
	net?: string;
	startX?: number;
	startY?: number;
	endX?: number;
	endY?: number;
}

interface ViaInfo {
	x: number;
	y: number;
	fromLayer: number;
	toLayer: number;
	net: string;
	diameter: number;
	holeDiameter: number;
}

/**
 * 计算焊盘边缘的点（用于走线起点/终点）
 */
function getPadEdgePoint(pad: PadInfo, targetPoint: Point, clearanceMil: number): Point {
	const dx = targetPoint.x - pad.x;
	const dy = targetPoint.y - pad.y;
	const dist = Math.sqrt(dx * dx + dy * dy);

	if (dist < 0.001) {
		// 目标点与焊盘中心重合，返回中心点
		return { x: pad.x, y: pad.y, layer: pad.layer };
	}

	// 焊盘半径 + DRC 间距（走线应该在焊盘边缘 + 间距处开始）
	const edgeDist = pad.radius + clearanceMil;

	// 在焊盘边缘的点
	return {
		x: pad.x + (dx / dist) * edgeDist,
		y: pad.y + (dy / dist) * edgeDist,
		layer: pad.layer
	};
}

/**
 * 确定两个焊盘之间的路由层
 * 规则：
 * - Top(1) → Top(1): 层1
 * - Bottom(2) → Bottom(2): 层2
 * - Top(1) → Bottom(2): 层1（需要过孔）
 * - Multi(12) → Top(1): 层1
 * - Multi(12) → Bottom(2): 层2
 * - Multi(12) → Multi(12): 层1
 */
function getRoutingLayer(pad1: PadInfo, pad2: PadInfo): number {
	const l1 = pad1.layer;
	const l2 = pad2.layer;

	// 如果都是多层，默认顶层
	if (l1 === 12 && l2 === 12) return 1;

	// 如果一个是多层，使用另一个的层
	if (l1 === 12) return l2 === 2 ? 2 : 1;
	if (l2 === 12) return l1 === 2 ? 2 : 1;

	// 都是单层，使用起点层
	return l1;
}

/**
 * 获取焊盘的实际出线层（通孔焊盘12层转换为1或2）
 */
function getPadRoutingLayer(pad: PadInfo, targetLayer: number): number {
	return pad.layer === 12 ? targetLayer : pad.layer;
}

// ─────────────────────────────────────────────

export class TopologicalRouter {
	private onProgress?: ProgressCallback;
	private onLog?: LogCallback;
	private onStatus?: StatusCallback;
	private isRouting = false;
	private shouldCancel = false;
	
	// 动态障碍物（已布线的走线）
	private dynamicObstacles: Obstacle[] = [];
	
	// 错误处理器
	private errorHandler: ErrorHandler;
	
	// 路径优化器
	private pathOptimizer: PathOptimizer;
	
	// DRC 检查器
	private drcChecker: DRCChecker | null = null;

	// 障碍物绕行器
	private obstacleBypasser: ObstacleBypasser | null = null;

	// DRC 物理规则（过孔尺寸等）
	private drcRules = {
		physics: {
			viaDiameter: 20,
			viaHoleDiameter: 10,
		}
	};

	constructor(
		onProgress?: ProgressCallback,
		onLog?: LogCallback,
		onStatus?: StatusCallback,
	) {
		this.onProgress = onProgress;
		this.onLog = onLog;
		this.onStatus = onStatus;
		
		// 初始化错误处理器
		this.errorHandler = new ErrorHandler(
			(details) => {
				const level = details.severity === ErrorSeverity.CRITICAL ? 'error' :
					details.severity === ErrorSeverity.ERROR ? 'error' :
					details.severity === ErrorSeverity.WARNING ? 'warn' : 'info';
				this.onLog?.(`[${details.type}] ${details.message}`, level);
			},
			{
				maxRetries: 2,
				initialDelay: 200,
				maxDelay: 2000,
				backoffFactor: 2,
			}
		);
		
		// 初始化路径优化器
		this.pathOptimizer = new PathOptimizer({
			enableRubberband: true,
			enableViaOptimization: true,
			enableSmoothing: false,
			minSegmentLength: 5,
			viaCostWeight: 1.5,
		});
		
		// DRCChecker 和 ObstacleBypasser 将在 route() 中初始化（需要 DRC 规则）
	}

	// ── 0. 从 DRC API 读取间距规则 ────────────────

	private async fetchDRCRules(): Promise<DRCRules> {
		try {
			const config = await eda.pcb_Drc.getCurrentRuleConfiguration();
			const cfg = (config as any)?.config;

			// 读取 Track↔Pad 间距
			const spacing = cfg?.Spacing?.['Safe Spacing'];
			let clearanceMm = 0.2;

			if (spacing) {
				const tableKey = Object.keys(spacing)[0];
				const tbl = spacing[tableKey];
				const content: number[][] = tbl?.tables?.['1']?.content ?? [];
				const cols: string[] = tbl?.column ?? [];
				const rows: string[] = tbl?.row ?? [];

				const trackColIdx = cols.indexOf('Track');
				const clearVals: number[] = [];

				for (const padType of ['SMD Pad', 'TH Pad', 'Via']) {
					const rowIdx = rows.indexOf(padType);
					const val = content[rowIdx]?.[trackColIdx];
					if (typeof val === 'number' && val > 0) clearVals.push(val);
				}

				if (clearVals.length > 0) {
					clearanceMm = Math.max(...clearVals);
				}
			}

			// 读取默认走线宽度
			const trackRules = cfg?.Physics?.Track;
			let traceWidthMm = 0.2;

			if (trackRules) {
				const defaultKey =
					Object.keys(trackRules).find(k => trackRules[k].isSetDefault) ??
					Object.keys(trackRules)[0];
				const tw = trackRules[defaultKey]?.form?.data?.['1']?.defaultValue;
				if (typeof tw === 'number' && tw > 0) traceWidthMm = tw;
			}

			const clearanceMil = clearanceMm * MM_TO_MIL;
			const traceWidthMil = traceWidthMm * MM_TO_MIL;

			this.onLog?.(
				`DRC 规则：间距 ${clearanceMm.toFixed(3)}mm (${clearanceMil.toFixed(1)}mil)` +
				`，走线宽度 ${traceWidthMm.toFixed(3)}mm (${traceWidthMil.toFixed(1)}mil)`,
				'info',
			);

			return { clearanceMil, traceWidthMil };
		} catch (err) {
			console.warn('[TopologicalRouter] 获取 DRC 规则失败，使用默认值:', err);
			this.onLog?.(`读取 DRC 规则失败，使用默认值（间距 ${FALLBACK_CLEARANCE_MIL}mil，宽度 ${FALLBACK_TRACE_WIDTH_MIL}mil）`, 'warn');
			return { clearanceMil: FALLBACK_CLEARANCE_MIL, traceWidthMil: FALLBACK_TRACE_WIDTH_MIL };
		}
	}

	// ── 1. 收集所有网络的焊盘（增强版）────────────

	private async collectPads(clearanceMil: number): Promise<{
		padMap: Map<string, PadInfo[]>;
		allPads: PadInfo[];
		obstacles: Obstacle[];
	}> {
		const allNets = await eda.pcb_Net.getAllNetsName();
		if (!allNets || allNets.length === 0) {
			throw new Error('未找到任何网络，请确保 PCB 包含网络连接');
		}

		this.onLog?.(`找到 ${allNets.length} 个网络`, 'info');

		const padMap = new Map<string, PadInfo[]>();
		const allPads: PadInfo[] = [];
		const obstacles: Obstacle[] = [];

		// 获取所有网络
		for (const netName of allNets) {
			try {
				const primitives = await eda.pcb_Net.getAllPrimitivesByNet(netName);
				console.log(`[TopologicalRouter] 网络 ${netName}: ${primitives ? primitives.length : 0} 个基元`);
				
				if (!primitives || primitives.length === 0) {
					console.warn(`[TopologicalRouter] 网络 ${netName} 没有基元`);
					continue;
				}

				const netPads: PadInfo[] = [];

				for (const prim of primitives) {
					const p = prim as any;

					// 调试：打印基元信息
					console.log(`[TopologicalRouter] 基元类型:`, p.type || 'unknown', '属性:', Object.keys(p));

					// 尝试多种可能的坐标属性
					const x: number = p.center?.x ?? p.x ?? p.startX ?? 0;
					const y: number = p.center?.y ?? p.y ?? p.startY ?? 0;
					const id: string = p.globalIndex || p.uuid || `${netName}_${allPads.length}`;

					// 焊盘尺寸（mm 转 mil）- 尝试多种可能的属性名
					const wMil = (p.topWidth || p.width || p.w || 0) * MM_TO_MIL;
					const hMil = (p.topHeight || p.height || p.h || 0) * MM_TO_MIL;
					const holeMil = (p.holeRadius || p.holeDiameter || p.drill || 0) * 2 * MM_TO_MIL;

					// 确定焊盘类型
					let padType: 'th' | 'smd' | 'via' = 'smd';
					if (holeMil > 0) {
						padType = 'th';
					} else if (p.isVia || p.type === 'via') {
						padType = 'via';
					}

					// 获取原始层号
					const rawLayer: number = p.layerId ?? p.layer ?? 1;

					// 根据焊盘类型确定路由层：
					// - 通孔(th/via)在多层(12)时，保持12，路由时动态选择1或2
					// - SMD焊盘必须在其指定层(1或2)
					let layer: number = rawLayer;
					if (padType === 'smd' && (rawLayer === 12 || rawLayer > 2)) {
						layer = 1; // SMD默认顶层
					}

					const padRadius = Math.max(wMil / 2, hMil / 2, holeMil / 2);
					const exclusion = (padRadius > 0 ? padRadius : 40) + clearanceMil;

					console.log(`[TopologicalRouter] 焊盘：(${x},${y}), 尺寸：${wMil}x${hMil}mil, 孔径：${holeMil}mil, 类型：${padType}`);

					const info: PadInfo = {
						x, y, net: netName, id, layer, exclusion,
						index: allPads.length,
						padType,
						holeDiameter: holeMil,
						radius: padRadius
					};

					allPads.push(info);
					netPads.push(info);

					// 添加为障碍物（用于后续布线）
					const obsType: Obstacle['type'] = padType === 'via' ? 'via' : 'pad';
					obstacles.push({
						x, y, radius: exclusion, layer, type: obsType, net: netName
					});
				}

				console.log(`[TopologicalRouter] 网络 ${netName}: ${netPads.length} 个焊盘`);
				
				// 修改：只要有焊盘就添加到 padMap，不一定需要 2 个
				if (netPads.length >= 1) {
					padMap.set(netName, netPads);
				} else {
					console.warn(`[TopologicalRouter] 网络 ${netName} 没有足够的焊盘进行布线`);
				}
			} catch (err) {
				console.warn(`[TopologicalRouter] 获取网络 ${netName} 失败:`, err);
			}
		}

		console.log(`[TopologicalRouter] 总计：${allPads.length} 个焊盘，${padMap.size} 个可布线网络`);
		
		return { padMap, allPads, obstacles };
	}

	// ── 2. Delaunay 三角剖分建邻接图 ───────────────

	private buildGraph(allPads: PadInfo[]): Map<number, Set<number>> {
		const graph = new Map<number, Set<number>>();
		for (let i = 0; i < allPads.length; i++) graph.set(i, new Set());

		if (allPads.length < 2) return graph;
		if (allPads.length === 2) {
			graph.get(0)!.add(1); graph.get(1)!.add(0);
			return graph;
		}

		try {
			const d = Delaunator.from(allPads.map(p => [p.x, p.y]));
			const tri = d.triangles;
			
			for (let e = 0; e < tri.length; e++) {
				const p = tri[e];
				const q = tri[e % 3 === 2 ? e - 2 : e + 1];
				graph.get(p)!.add(q);
				graph.get(q)!.add(p);
			}
		} catch (err) {
			console.warn('[TopologicalRouter] Delaunay 三角剖分失败，使用全连接图:', err);
			// 回退到全连接图
			for (let i = 0; i < allPads.length; i++) {
				for (let j = i + 1; j < allPads.length; j++) {
					graph.get(i)!.add(j);
					graph.get(j)!.add(i);
				}
			}
		}
		
		return graph;
	}

	// ── 3. Prim MST 确定网络内连接顺序 ────────────

	private computeMST(pads: PadInfo[]): Array<[number, number]> {
		const n = pads.length;
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
				const dx = pads[u].x - pads[v].x;
				const dy = pads[u].y - pads[v].y;
				const d = Math.sqrt(dx * dx + dy * dy);
				if (d < minDist[v]) { minDist[v] = d; minFrom[v] = u; }
			}
		}
		return edges;
	}

	// ── 4. A* 寻路（增强版 - 改进启发式和避障）──────

	private findPath(
		graph: Map<number, Set<number>>,
		startIdx: number,
		endIdx: number,
		allPads: PadInfo[],
		traceWidthMil: number,
		clearanceMil: number,
		obstacles: Obstacle[] = [],
	): number[] | null {
		const euclidean = (a: number, b: number) => {
			const dx = allPads[a].x - allPads[b].x;
			const dy = allPads[a].y - allPads[b].y;
			return Math.sqrt(dx * dx + dy * dy);
		};

		// 通道宽度检查（增强版 - 考虑动态障碍物）
		const edgePassable = (u: number, v: number): boolean => {
			if (u === startIdx || u === endIdx || v === startIdx || v === endIdx) return true;
			
			const gap = euclidean(u, v) - allPads[u].exclusion - allPads[v].exclusion;
			if (gap < traceWidthMil + clearanceMil) return false;
			
			// 检查动态障碍物
			const midX = (allPads[u].x + allPads[v].x) / 2;
			const midY = (allPads[u].y + allPads[v].y) / 2;
			
			for (const obs of obstacles) {
				if (obs.net === allPads[startIdx].net) continue; // 跳过同一网络
				const distToObs = Math.sqrt((midX - obs.x) ** 2 + (midY - obs.y) ** 2);
				if (distToObs < obs.radius + traceWidthMil / 2) return false;
			}
			
			return true;
		};

		// A* 算法（使用改进的启发式）
		const gScore = new Map<number, number>();
		const fScore = new Map<number, number>();
		const cameFrom = new Map<number, number>();
		const openSet = new Set<number>();

		gScore.set(startIdx, 0);
		fScore.set(startIdx, euclidean(startIdx, endIdx));
		openSet.add(startIdx);

		let iterations = 0;
		const maxIterations = allPads.length * 100; // 防止无限循环

		while (openSet.size > 0 && iterations < maxIterations) {
			iterations++;
			
			let current = -1, minF = Infinity;
			for (const n of openSet) {
				const f = fScore.get(n) ?? Infinity;
				if (f < minF) { minF = f; current = n; }
			}

			if (current === endIdx) {
				const path: number[] = [];
				let c: number | undefined = current;
				while (c !== undefined) {
					path.unshift(c);
					const prev = cameFrom.get(c);
					if (prev === startIdx) { path.unshift(startIdx); break; }
					c = prev;
				}
				return path;
			}

			openSet.delete(current);

			for (const neighbor of graph.get(current) ?? []) {
				if (!edgePassable(current, neighbor)) continue;

				let edgeCost = euclidean(current, neighbor);
				
				// 启发式优化：
				// 1. 经过异网络焊盘增加代价
				if (allPads[neighbor].net !== allPads[startIdx].net && neighbor !== endIdx) {
					edgeCost *= 1.5;
				}
				
				// 2. 优先选择同层焊盘（减少过孔）
				if (allPads[neighbor].layer !== allPads[startIdx].layer) {
					edgeCost *= 1.2;
				}
				
				// 3. 朝向目标的边给予奖励
				const dx = allPads[neighbor].x - allPads[current].x;
				const dy = allPads[neighbor].y - allPads[current].y;
				const targetDx = allPads[endIdx].x - allPads[current].x;
				const targetDy = allPads[endIdx].y - allPads[current].y;
				const dotProduct = dx * targetDx + dy * targetDy;
				const distToTarget = Math.sqrt(targetDx * targetDx + targetDy * targetDy);
				const edgeLen = Math.sqrt(dx * dx + dy * dy);
				
				if (distToTarget > 0 && edgeLen > 0) {
					const alignment = dotProduct / (distToTarget * edgeLen);
					if (alignment > 0.5) {
						edgeCost *= (1 - alignment * 0.3); // 最多 30% 奖励
					}
				}

				const tentG = (gScore.get(current) ?? Infinity) + edgeCost;
				if (tentG < (gScore.get(neighbor) ?? Infinity)) {
					cameFrom.set(neighbor, current);
					gScore.set(neighbor, tentG);
					fScore.set(neighbor, tentG + euclidean(neighbor, endIdx) * 0.9); // 略微乐观的启发式
					openSet.add(neighbor);
				}
			}
		}
		
		// 如果 A* 失败，尝试直接连接（降级策略）
		this.onLog?.(`A* 寻路失败，尝试直接连接`, 'warn');
		return null;
	}

	// ── 5. 橡皮筋几何：计算绕行点（增强版）──────

	private computeBypass(prev: Point, obstacle: PadInfo, next: Point): Point[] {
		const r = obstacle.exclusion + EXTRA_MARGIN;
		const cx = obstacle.x, cy = obstacle.y;

		const dx1 = prev.x - cx, dy1 = prev.y - cy;
		const d1 = Math.sqrt(dx1 * dx1 + dy1 * dy1);
		const dx2 = next.x - cx, dy2 = next.y - cy;
		const d2 = Math.sqrt(dx2 * dx2 + dy2 * dy2);
		
		if (d1 < 0.001 || d2 < 0.001) return [];

		const ux1 = dx1 / d1, uy1 = dy1 / d1;
		const ux2 = dx2 / d2, uy2 = dy2 / d2;

		const entry: Point = { x: cx + ux1 * r, y: cy + uy1 * r };
		const exit_: Point = { x: cx + ux2 * r, y: cy + uy2 * r };

		// 计算角平分线上的圆弧中点
		const cosA = ux1 * ux2 + uy1 * uy2;
		const halfA = Math.acos(Math.max(-1, Math.min(1, cosA))) / 2;
		
		if (r * Math.cos(halfA) < obstacle.exclusion) {
			let bx = ux1 + ux2, by = uy1 + uy2;
			const bl = Math.sqrt(bx * bx + by * by);
			if (bl > 0.001) { bx /= bl; by /= bl; }
			else { bx = -uy1; by = ux1; }
			return [entry, { x: cx + bx * r, y: cy + by * r }, exit_];
		}
		return [entry, exit_];
	}

	private buildPolyline(path: number[], allPads: PadInfo[], netName: string): Point[] {
		if (path.length < 2) return [];

		const pts: Point[] = [];
		pts.push({ x: allPads[path[0]].x, y: allPads[path[0]].y, layer: allPads[path[0]].layer });

		for (let i = 1; i < path.length - 1; i++) {
			const prev = allPads[path[i - 1]];
			const curr = allPads[path[i]];
			const next = allPads[path[i + 1]];

			if (curr.net === netName) {
				pts.push({ x: curr.x, y: curr.y, layer: curr.layer });
			} else {
				const bypassPoints = this.computeBypass(
					{ x: prev.x, y: prev.y }, curr, { x: next.x, y: next.y },
				);
				for (const bp of bypassPoints) {
					pts.push({ ...bp, layer: curr.layer });
				}
			}
		}

		pts.push({ x: allPads[path[path.length - 1]].x, y: allPads[path[path.length - 1]].y, layer: allPads[path[path.length - 1]].layer });
		return pts;
	}

	// ── 6. 路径优化（橡皮筋算法）──────

	private optimizePath(points: Point[]): Point[] {
		if (points.length <= 2) return points;
		
		const optimized: Point[] = [points[0]];
		let current = 0;
		
		while (current < points.length - 1) {
			// 尝试跳过中间点，直接连接到更远的点
			let furthest = current + 1;
			
			for (let j = current + 2; j < points.length; j++) {
				// 检查从 current 到 j 的线段是否与任何障碍物相交
				if (this.isLineClear(points[current], points[j], current === 0 || j === points.length - 1)) {
					furthest = j;
				} else {
					break;
				}
			}
			
			if (furthest > current + 1) {
				// 可以跳过中间点
				optimized.push(points[furthest]);
				current = furthest;
			} else {
				// 不能跳过，添加下一个点
				optimized.push(points[current + 1]);
				current++;
			}
		}
		
		return optimized;
	}

	private isLineClear(p1: Point, p2: Point, allowTouchEndpoints: boolean): boolean {
		const obstacles = this.dynamicObstacles;
		const minGap = 5; // 最小间隙 mils
		
		for (const obs of obstacles) {
			// 计算点到线段的最短距离
			const dist = this.pointToSegmentDistance(obs.x, obs.y, p1.x, p1.y, p2.x, p2.y);
			
			if (allowTouchEndpoints) {
				// 允许接触端点
				const distToP1 = Math.sqrt((obs.x - p1.x) ** 2 + (obs.y - p1.y) ** 2);
				const distToP2 = Math.sqrt((obs.x - p2.x) ** 2 + (obs.y - p2.y) ** 2);
				if (distToP1 < obs.radius || distToP2 < obs.radius) continue;
			}
			
			if (dist < obs.radius + minGap) {
				return false;
			}
		}
		
		return true;
	}

	private pointToSegmentDistance(
		px: number, py: number,
		x1: number, y1: number, x2: number, y2: number
	): number {
		const dx = x2 - x1;
		const dy = y2 - y1;
		const len2 = dx * dx + dy * dy;
		
		if (len2 === 0) {
			return Math.sqrt((px - x1) ** 2 + (py - y1) ** 2);
		}
		
		let t = ((px - x1) * dx + (py - y1) * dy) / len2;
		t = Math.max(0, Math.min(1, t));
		
		const projX = x1 + t * dx;
		const projY = y1 + t * dy;
		
		return Math.sqrt((px - projX) ** 2 + (py - projY) ** 2);
	}

	// ── 7. 创建过孔（增强功能）──────

	private async createVia(
		x: number, y: number,
		fromLayer: number, toLayer: number,
		net: string,
		viaDiameter: number, holeDiameter: number
	): Promise<boolean> {
		try {
			// 使用 EDA API 创建过孔
			// API参数: (net, x, y, holeDiameter, diameter, ...)
			await eda.pcb_PrimitiveVia.create(
				net,
				x, y,
				holeDiameter,
				viaDiameter
			);
			return true;
		} catch (err) {
			console.warn(`[TopologicalRouter] 创建过孔失败 (${net}):`, err);
			return false;
		}
	}

	// ── 8. 主流程（增强版）───────────────────────────────

	async route(options: RoutingOptions): Promise<boolean> {
		if (this.isRouting) throw new Error('布线正在进行中');
		this.isRouting = true;
		this.shouldCancel = false;
		this.dynamicObstacles = [];
		this.errorHandler.clearErrorLog();
		this.onStatus?.('connected');

		try {
			// Step 0: 读取 DRC 规则（带重试）
			this.onProgress?.({ stage: '读取规则', percentage: 3, message: '正在读取 DRC 间距规则...' });
			const { clearanceMil, traceWidthMil } = await this.errorHandler.withRetry(
				() => this.fetchDRCRules(),
				ErrorType.DRC_ERROR,
				{ stage: 'fetchDRCRules' }
			);

			// 初始化 DRC 检查器和绕行器
			this.drcChecker = new DRCChecker(clearanceMil, traceWidthMil);
			this.obstacleBypasser = new ObstacleBypasser(clearanceMil, traceWidthMil);
			// 同步过孔物理规则
			this.drcRules.physics.viaDiameter = Math.max(20, traceWidthMil * 2.5);
			this.drcRules.physics.viaHoleDiameter = Math.max(10, traceWidthMil * 1.2);

			// Step 1: 收集焊盘（带重试）
			this.onProgress?.({ stage: '解析 PCB', percentage: 8, message: '正在获取 PCB 数据...' });
			const { padMap, allPads, obstacles } = await this.errorHandler.withRetry(
				() => this.collectPads(clearanceMil),
				ErrorType.PCB_API_ERROR,
				{ stage: 'collectPads' }
			);
			if (padMap.size === 0) throw new Error('没有可布线的网络');

			// Step 2: 建图
			this.onProgress?.({ stage: '建图', percentage: 22, message: '正在构建 Delaunay 拓扑图...' });
			const graph = this.buildGraph(allPads);
			this.onLog?.(`Delaunay 图构建完成，${allPads.length} 个节点`, 'info');

			// Step 3: 寻路
			this.onProgress?.({ stage: '布线中', percentage: 35, message: '正在计算布线路径...' });
			const polylines: TrackPolyline[] = [];
			const vias: ViaInfo[] = [];

			// 使用 RoutingOrchestrator 对网络排序（关键网络优先）
			const orchestrator = new RoutingOrchestrator();
			const networks = Array.from(padMap.entries()).map(([name, pads]) => ({ name, pads }));
			const sortedNetworks = orchestrator.sortNetworks(networks);
			const netNames = sortedNetworks.map(n => n.name);

			let routedNets = 0, directFallback = 0, failedNets = 0;

			for (let ni = 0; ni < netNames.length; ni++) {
				if (this.shouldCancel) {
					this.onLog?.('布线已取消', 'warn');
					break;
				}
				
				const netName = netNames[ni];
				const pads = padMap.get(netName)!;
				const pct = 35 + Math.floor((ni / netNames.length) * 40);
				this.onProgress?.({ stage: '布线中', percentage: pct, message: `布线网络 ${ni + 1}/${netNames.length}: ${netName}` });

				let netSuccess = false;

				for (const [si, di] of this.computeMST(pads)) {
					const src = pads[si], dst = pads[di];

					// 确定路由层
					const routingLayer = getRoutingLayer(src, dst);

					// 获取焊盘实际出线层（通孔12层转为1或2）
					const srcLayer = getPadRoutingLayer(src, routingLayer);
					const dstLayer = getPadRoutingLayer(dst, routingLayer);

					// 检查是否需要跨层
					const needsVia = srcLayer !== dstLayer;

					console.log(`[TopologicalRouter] 网络 ${netName}: ${src.id}(layer=${src.layer}→${srcLayer}) -> ${dst.id}(layer=${dst.layer}→${dstLayer}), routingLayer=${routingLayer}, needsVia=${needsVia}`);

					// 尝试 1: A* 寻路（基于 Delaunay 图）
					const path = this.findPath(
						graph, src.index, dst.index, allPads,
						traceWidthMil, clearanceMil,
						this.dynamicObstacles
					);

					let points: Point[] = [];
					let pathFound = false;
					let useBypass = false;

					if (path && path.length >= 2) {
						// A* 找到路径，构建折线
						const polylinePoints = this.buildPolyline(path, allPads, netName);

						// 使用焊盘中心和实际出线层
						if (polylinePoints.length >= 2) {
							polylinePoints[0] = { x: src.x, y: src.y, layer: srcLayer };

							if (needsVia) {
								// 需要跨层：在终点前添加过孔
								const viaPoint = polylinePoints[polylinePoints.length - 1];
								polylinePoints[polylinePoints.length - 1] = { x: viaPoint.x, y: viaPoint.y, layer: srcLayer };
								polylinePoints.push({ x: dst.x, y: dst.y, layer: dstLayer });
							} else {
								polylinePoints[polylinePoints.length - 1] = { x: dst.x, y: dst.y, layer: dstLayer };
							}
						}
						
						// 检查 A* 路径是否与动态障碍物冲突
						if (this.obstacleBypasser) {
							let isSafe = true;
							let conflictCount = 0;
							for (let i = 0; i < polylinePoints.length - 1; i++) {
								if (!this.obstacleBypasser.isSegmentSafe(
									polylinePoints[i], polylinePoints[i + 1],
									this.dynamicObstacles, netName
								)) {
									isSafe = false;
									conflictCount++;
								}
							}
							
							if (!isSafe) {
								// A* 路径不安全，尝试绕行算法
								useBypass = true;
								this.onLog?.(`网络 ${netName}: A* 路径发现 ${conflictCount} 处冲突，使用绕行算法`, 'warn');
							} else {
								points = polylinePoints;
								pathFound = true;
								netSuccess = true;
							}
						} else {
							points = polylinePoints;
							pathFound = true;
							netSuccess = true;
						}
					}

					// 尝试 2: 使用障碍物绕行算法生成路径
					if (useBypass || !pathFound) {
						if (this.obstacleBypasser) {
							// 使用焊盘中心和实际出线层
							const startPoint = { x: src.x, y: src.y, layer: srcLayer };
							const endPoint = needsVia
								? { x: dst.x, y: dst.y, layer: srcLayer }
								: { x: dst.x, y: dst.y, layer: dstLayer };

							console.log(`[TopologicalRouter] 网络 ${netName}: 尝试绕行 (${startPoint.x.toFixed(0)},${startPoint.y.toFixed(0)}) -> (${endPoint.x.toFixed(0)},${endPoint.y.toFixed(0)})`);

							const bypassPath = this.obstacleBypasser.generateBypassPath(
								startPoint, endPoint,
								this.dynamicObstacles,
								netName
							);

							if (bypassPath && bypassPath.length >= 2) {
								// 为绕行点添加层信息
								points = bypassPath.map(p => ({ ...p, layer: srcLayer }));

								if (needsVia) {
									// 添加目标层的终点
									points.push({ x: dst.x, y: dst.y, layer: dstLayer });
								}

								pathFound = true;
								netSuccess = true;
								useBypass = true;
								this.onLog?.(`网络 ${netName}: 使用绕行算法（${bypassPath.length} 个点）`, 'info');
								console.log(`[TopologicalRouter] 网络 ${netName}: 绕行成功，${bypassPath.length} 个点`);
							} else {
								console.log(`[TopologicalRouter] 网络 ${netName}: 绕行失败`);
							}
						}
					}

					if (!pathFound) {
						// 尝试 3: 降级直连
						if (needsVia) {
							// 跨层直连：起点层 -> 过孔 -> 终点层
							points = [
								{ x: src.x, y: src.y, layer: srcLayer },
								{ x: dst.x, y: dst.y, layer: srcLayer },
								{ x: dst.x, y: dst.y, layer: dstLayer }
							];
						} else {
							points = [
								{ x: src.x, y: src.y, layer: srcLayer },
								{ x: dst.x, y: dst.y, layer: dstLayer }
							];
						}
						directFallback++;
						this.onLog?.(`网络 ${netName}: 未找到绕障路径，降级直连`, 'warn');
					}

					if (points.length >= 2) {
						// 优化路径 - 使用 PathOptimizer
						const optimizedPoints = this.pathOptimizer.optimize(points);

						console.log(`[TopologicalRouter] 网络 ${netName}: 优化后路径点数=${optimizedPoints.length}`);
						optimizedPoints.forEach((p, i) => {
							console.log(`  点${i}: (${p.x.toFixed(1)}, ${p.y.toFixed(1)}) layer=${p.layer}`);
						});

						// 检查是否需要过孔（层变化）
						for (let i = 0; i < optimizedPoints.length - 1; i++) {
							const p1 = optimizedPoints[i];
							const p2 = optimizedPoints[i + 1];

							if (p1.layer !== p2.layer) {
								// 需要过孔，确保层号不是12
								const fromLayer = p1.layer === 12 ? 1 : p1.layer!;
								const toLayer = p2.layer === 12 ? 1 : p2.layer!;

								console.log(`[TopologicalRouter] 网络 ${netName}: 添加过孔 at (${p2.x.toFixed(1)}, ${p2.y.toFixed(1)}) from layer ${p1.layer} to ${p2.layer}, 修正为 ${fromLayer}->${toLayer}`);

								vias.push({
									x: p2.x, y: p2.y,
									fromLayer: fromLayer,
									toLayer: toLayer,
									net: netName,
									diameter: this.drcRules.physics.viaDiameter,
									holeDiameter: this.drcRules.physics.viaHoleDiameter
								});

								// 修正点的层号
								if (p2.layer === 12) {
									p2.layer = toLayer;
								}
							}
						}

						polylines.push({
							net: netName,
							points: optimizedPoints,
							layer: routingLayer,
							width: traceWidthMil
						});
					}
				}
				
				if (netSuccess) {
					routedNets++;
				} else {
					failedNets++;
					this.onLog?.(`网络 ${netName}: 布线失败`, 'error');
				}
			}

			this.onLog?.(
				`路径计算完成：${routedNets}/${netNames.length} 个网络成功` +
				(directFallback ? `（${directFallback} 条降级直连）` : '') +
				(failedNets ? `（${failedNets} 条失败）` : ''),
				'success',
			);

			// Step 4: 写入 PCB（增强版 - 支持过孔）
			this.onProgress?.({ stage: '导入走线', percentage: 78, message: '正在写入走线...' });
			let created = 0;
			let createdVias = 0;
			const totalSegs = polylines.reduce((s, pl) => s + pl.points.length - 1, 0);

			for (const pl of polylines) {
				for (let i = 0; i < pl.points.length - 1; i++) {
					if (this.shouldCancel) break;

					try {
						const p1 = pl.points[i];
						const p2 = pl.points[i + 1];

						const lineLayer = p1.layer ?? pl.layer;
						console.log(`[TopologicalRouter] 写入导线: net=${pl.net}, layer=${lineLayer}, (${p1.x.toFixed(1)},${p1.y.toFixed(1)}) -> (${p2.x.toFixed(1)},${p2.y.toFixed(1)})`);

						await eda.pcb_PrimitiveLine.create(
							pl.net,
							lineLayer,
							p1.x, p1.y,
							p2.x, p2.y,
							pl.width, false
						);
						created++;

						// 添加为动态障碍物（用于后续布线）- 使用线段模型
						this.dynamicObstacles.push({
							x: (p1.x + p2.x) / 2, // 中点（用于兼容）
							y: (p1.y + p2.y) / 2,
							radius: pl.width / 2 + clearanceMil,
							layer: p1.layer ?? pl.layer,
							type: 'track',
							net: pl.net,
							startX: p1.x,
							startY: p1.y,
							endX: p2.x,
							endY: p2.y
						});
					} catch (err) {
						console.warn(`[TopologicalRouter] 创建走线失败 (${pl.net}):`, err);
					}

					if (created % 20 === 0) {
						const pct = 78 + Math.floor((created / Math.max(totalSegs, 1)) * 20);
						this.onProgress?.({ stage: '导入走线', percentage: pct, message: `已写入 ${created} 段...` });
					}
				}
			}
			
			// 创建过孔（去重）
			if (vias.length > 0) {
				console.log(`[Router2] 过孔去重前: ${vias.length} 个`);
				const uniqueVias = new Map<string, typeof vias[0]>();
				for (const via of vias) {
					const key = `${via.x.toFixed(2)}_${via.y.toFixed(2)}_${via.fromLayer}_${via.toLayer}_${via.net}`;
					if (!uniqueVias.has(key)) {
						uniqueVias.set(key, via);
					} else {
						console.log(`[Router2] 过孔重复: net=${via.net}, pos=(${via.x.toFixed(2)},${via.y.toFixed(2)}), layers=${via.fromLayer}->${via.toLayer}`);
					}
				}

				const viaList = Array.from(uniqueVias.values());
				console.log(`[Router2] 过孔去重后: ${viaList.length} 个`);
				this.onProgress?.({ stage: '创建过孔', percentage: 95, message: `正在创建 ${viaList.length} 个过孔...` });

				for (const via of viaList) {
					if (this.shouldCancel) break;

					console.log(`[Router2] 创建过孔: net=${via.net}, pos=(${via.x.toFixed(2)},${via.y.toFixed(2)}), diameter=${via.diameter.toFixed(2)}mil, hole=${via.holeDiameter.toFixed(2)}mil, layers=${via.fromLayer}->${via.toLayer}`);
					try {
						await this.createVia(
							via.x, via.y,
							via.fromLayer, via.toLayer,
							via.net,
							via.diameter, via.holeDiameter
						);
						createdVias++;
					} catch (err) {
						console.warn(`[TopologicalRouter] 创建过孔失败:`, err);
					}
				}

				this.onLog?.(`创建 ${createdVias}/${viaList.length} 个过孔（原始${vias.length}个）`, 'info');
			}

			const finalPct = 98 + Math.floor((created / Math.max(totalSegs, 1)) * 2);
			this.onProgress?.({ stage: '布线完成', percentage: finalPct, message: '布线完成' });
			this.onLog?.(
				`布线完成！共写入 ${created}/${totalSegs} 段走线，${createdVias} 个过孔`,
				'success'
			);

			this.isRouting = false;
			this.onStatus?.('disconnected');
			return created > 0;

		} catch (error) {
			console.error('[TopologicalRouter] 布线错误:', error);
			
			// 使用错误处理器生成友好的错误消息
			let friendlyMessage: string;
			
			if (error instanceof RoutingError) {
				friendlyMessage = error.toFriendlyMessage();
			} else {
				const routingError = createStandardError(
					ErrorType.ROUTING_ERROR,
					ErrorSeverity.ERROR,
					error instanceof Error ? error : undefined,
					{ stage: 'route' }
				);
				friendlyMessage = routingError.toFriendlyMessage();
			}
			
			this.onLog?.(friendlyMessage, 'error');
			this.isRouting = false;
			this.onStatus?.('disconnected');
			throw error;
		}
	}

	cancel(): void {
		if (this.isRouting) {
			this.shouldCancel = true;
			this.onLog?.('正在取消布线...', 'warn');
		}
	}

	isActive(): boolean { return this.isRouting; }
}
