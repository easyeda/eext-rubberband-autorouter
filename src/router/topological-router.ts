/**
 * 拓扑橡皮筋路由器（EDA 宿主侧）
 *
 * 职责划分：
 *  - 主线程：读取 DRC/焊盘/障碍、创建与删除画布图元、统计、进度回调
 *  - Worker：纯几何规划（MST、逃逸点、可视图 Dijkstra、撕毁重布、圆角）
 *
 * 规划每完成一个网络就把结果 postMessage 回主线程，主线程立即写入画布，
 * 因此用户可以在布线过程中看到走线逐步出现（动态刷新）。
 */

import {
	RBObstacle,
	buildPolylineObstacle, buildPolygonObstacle,
} from './rubberband-engine';
import { DRCRules, fetchDRCRules, applyRouterOptionsToDRC } from './drc-rules';
import {
	PlanningEngine, PlanningInput, PlanningResult,
	RenderedPrim, PlannedVia, PadShape, EnginePad,
	mergePrims, isGroundNetName, isPowerNetName,
} from './planning-engine';
import { createPlanningWorker } from './worker-source';
import { parsePolygonPath } from './polygon-parse';
import {
	RouterUiOptions, RouteStats, RouteSummary, BoardInspection,
} from '../types/router-params';
import { RoutingProgress } from '../types';

export type ProgressCallback = (progress: RoutingProgress) => void;
export type LogCallback = (message: string, level: string) => void;
export type StatsCallback = (stats: RouteStats) => void;

interface PrimIdRecord {
	lines: any[];
	arcs: any[];
	vias: any[];
}

function primitiveIdOf(created: unknown): string {
	const c = created as any;
	try {
		const id = c?.getState_PrimitiveId?.();
		if (typeof id === 'string' && id) return id;
	} catch { /* ignore */ }
	const fallback = c?.primitiveId ?? c?.id ?? c?.uuid ?? '';
	return typeof fallback === 'string' ? fallback : '';
}

/** 让出主线程，保证写入过程中画布可重绘 */
function breathe(): Promise<void> {
	return new Promise<void>(resolve => setTimeout(resolve, 0));
}

export class TopologicalRouter {
	private onProgress?: ProgressCallback;
	private onLog?: LogCallback;
	private onStats?: StatsCallback;

	private isRouting = false;
	private shouldCancel = false;

	/** 焊盘原始形状缓存（按位置键，含 pad 与 hole 与 rotation） */
	private padShapeCache = new Map<string, PadShape>();

	/** 每网络已写入画布的图元 ID（用于实时替换/撕毁） */
	private netPrimIds = new Map<string, PrimIdRecord>();
	/** 本轮真正写过画布的网络（用于残留清理判断） */
	private touchedNets = new Set<string>();
	/** 每网络最终图形规模（统计用） */
	private netGeom = new Map<string, { segments: number; vias: number }>();
	/** 非实时预览模式下暂存的规划结果 */
	private pendingPlanned = new Map<string, { prims: RenderedPrim[]; vias: PlannedVia[] }>();

	/** 已布成功的网络（仅用于实时统计） */
	private routedNames = new Set<string>();
	private failedNames = new Set<string>();
	/** 每网络焊盘级飞线进度（total = 参与布线焊盘数，routed = 已消除全部飞线） */
	private netPadStats = new Map<string, { total: number; routed: number }>();
	private totalPads = 0;

	/**
	 * 主线程写入调度器（差量合并 + 限速）：引擎在 worker 里只管输出，
	 * 这里的写入绝不反压引擎。同网合并 = 队列中该网只保留最新任务
	 * （任务自含「删旧写新」语义，最新状态覆盖旧状态）；写入间隔让出
	 * 30ms CPU，避免 EDA 写盘的同步开销挤占 worker 的算力
	 * （实测不节流时快扫 26s → 56s）。
	 */
	/** 定稿闸门：收尾对账开始后，在途的逐网消息一律忽略（否则旧几何会把终态覆盖回去） */
	private finalizing = false;
	/** 单网最后写入时刻（限频用） */
	private lastWriteAt = new Map<string, number>();
	/** 单网已写入几何的签名（收尾时跳过无变化的重写） */
	private netWrittenSig = new Map<string, string>();
	/** 节拍刷新的待写队列（name → 最新几何；prims 为空表示该网要删除） */
	private livePending = new Map<string, { prims: RenderedPrim[]; vias: PlannedVia[] }>();
	private liveTimer: ReturnType<typeof setInterval> | null = null;
	/** 画布刷新间隔：全局统一 5 秒一次（用户指定） */
	private readonly LIVE_INTERVAL_MS = 5000;

	private ensureLiveTimer(): void {
		if (this.liveTimer) return;
		this.liveTimer = setInterval(() => { void this.flushLive(); }, this.LIVE_INTERVAL_MS);
	}

	/** 节拍到点：把累积的最新几何批量写入画布 */
	private async flushLive(): Promise<void> {
		if (this.finalizing || !this.livePending.size) return;
		const batch = [...this.livePending.entries()];
		this.livePending.clear();
		for (const [name, g] of batch) {
			if (this.finalizing) { this.livePending.clear(); return; }
			try {
				if (!g.prims.length) await this.deleteNet(name);
				else await this.writeNet(name, g.prims, g.vias);
			} catch (err) {
				console.warn('[TopologicalRouter] 节拍写入失败:', err);
			}
		}
	}

	private stopLiveTimer(): void {
		if (this.liveTimer) {
			clearInterval(this.liveTimer);
			this.liveTimer = null;
		}
		this.livePending.clear();
	}

	private applyPending = new Map<string, () => Promise<void>>();
	private applyDraining = false;
	private applyWaiters: Array<() => void> = [];

	private enqueueWrite(key: string, task: () => Promise<void>): void {
		this.applyPending.set(key, task);   // 同 key 合并：引擎最新状态覆盖旧任务
		void this.drainApplies();
	}

	private async drainApplies(): Promise<void> {
		if (this.applyDraining) return;
		this.applyDraining = true;
		for (;;) {
			const next = this.applyPending.entries().next();
			if (next.done || !next.value) break;
			const key = next.value[0];
			const task = next.value[1];
			this.applyPending.delete(key);
			try {
				await task();
			} catch (err) {
				console.warn('[TopologicalRouter] 写入任务失败:', err);
				this.onLog?.('写入图元失败：' + (err instanceof Error ? err.message : String(err)), 'error');
			}
			if (this.applyPending.size) await new Promise<void>(r => setTimeout(r, 30));
		}
		this.applyDraining = false;
		const waiters = this.applyWaiters.splice(0);
		for (const w of waiters) w();
	}

	/** 等待写入队列排空（run 收尾/finish 前调用） */
	private applyIdle(): Promise<void> {
		if (!this.applyDraining && !this.applyPending.size) return Promise.resolve();
		return new Promise<void>(resolve => this.applyWaiters.push(resolve));
	}

	private worker: Worker | null = null;
	private engine: PlanningEngine | null = null;
	private forceFinish: (() => void) | null = null;
	private cancelTimer: ReturnType<typeof setTimeout> | null = null;

	private startedAt = 0;
	private totalNets = 0;
	private currentOptions: RouterUiOptions | null = null;

	constructor(
		onProgress?: ProgressCallback,
		onLog?: LogCallback,
		onStats?: StatsCallback,
	) {
		this.onProgress = onProgress;
		this.onLog = onLog;
		this.onStats = onStats;
	}

	// ── 0. 收集焊盘与静态障碍 ────────────────────

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
					// type: 1 = SMD 焊盘，2 = 通孔焊盘（旧实现只收 1，TH 焊盘对路由器隐形 ——
					// 实测走线穿过 RST 通孔焊盘 0.2mil，EDA DRC 报 TH Pad to Track）
					if (p.type !== undefined && p.type !== 1 && p.type !== 2) continue;
					const x: number = p.center?.x ?? p.x ?? p.startX ?? 0;
					const y: number = p.center?.y ?? p.y ?? p.startY ?? 0;
					if (x === 0 && y === 0 && !p.center) continue;
					const id: string = p.globalIndex || p.uuid || `${netName}_${allPads.length}`;

					// 焊盘形状：pcb_Net.getAllPrimitivesByNet 的原语没有 pad/hole 字段，
					// 但带 defaultPadShapeAspect/holeAspect（[形状, 宽, 高]，单位与坐标一致），
					// 取不到时再靠 pcb_PrimitivePad.getAll 的位置缓存兜底。
					const padShape = p.pad ?? p.defaultPadShapeAspect ?? [];
					let wMil = 0, hMil = 0;
					if (Array.isArray(padShape) && padShape.length >= 3) {
						wMil = padShape[1] ?? 0;
						hMil = padShape[2] ?? 0;
					}
					const hole = p.hole ?? p.holeAspect ?? [];
					const holeMil = Array.isArray(hole) && hole.length >= 2 ? (hole[1] ?? 0) : 0;
					// 形状缺失回退：优先用焊盘形状缓存（同坐标 / 2mil 邻近），
					// 盲猜 32×34mil 在 0.5mm 间距引脚排里会让相邻焊盘障碍圈互相
					// 吞没、封死所有端点（实测整板 0 布通）
					if (wMil === 0 && hMil === 0) {
						const keyOfPad = `${x.toFixed(2)},${y.toFixed(2)}`;
						const cached = this.padShapeCache.get(keyOfPad);
						let shape: any[] | null = cached?.pad ?? null;
						if (!Array.isArray(shape)) {
							let bd = 2;
							for (const [k, v] of this.padShapeCache) {
								const [kx, ky] = k.split(',').map(Number);
								const dd = Math.hypot(kx - x, ky - y);
								if (dd < bd) { bd = dd; shape = v.pad; }
							}
						}
						if (Array.isArray(shape) && shape.length >= 3) {
							wMil = shape[1] ?? 0;
							hMil = shape[2] ?? 0;
						}
					}
					if (wMil === 0 && hMil === 0) { wMil = 32; hMil = 34; }

					const rawLayer: number = p.layerId ?? p.layer ?? 1;
					// 通孔焊盘：有钻孔，或层号为 12（多层贯穿）。只看钻孔会把
					// getAllPrimitivesByNet 里缺 hole 字段的 TH 焊盘误判成 SMD，
					// 进而被下面的强制改层压成顶层——底层从此对它不设防，
					// 过孔兜底也把它当顶层 SMD 处理（多打一个过孔）。
					let padType: 'th' | 'smd' | 'via' = 'smd';
					if (holeMil > 0 || rawLayer === 12) padType = 'th';
					else if (p.isVia || p.type === 'via') padType = 'via';

					let layer: number = rawLayer;
					if (padType === 'smd' && (rawLayer === 12 || rawLayer > 2)) layer = 1;

					const padRadius = Math.max(wMil / 2, hMil / 2, holeMil / 2);
					const bodyRadius = Math.max(Math.min(wMil, hMil) / 2, holeMil / 2);
					const info: EnginePad = {
						x, y, net: netName, id, layer,
						padType, holeDiameter: holeMil,
						radius: padRadius,
						bodyRadius,
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

		// 无网络焊盘（NC 脚、安装孔、屏蔽框脚、半孔工艺脚……）不是可布线对象，
		// 但铜皮真实存在：EDA 的 DRC 照查间距，走线/过孔压上去就是 0mil 违规
		//（实测 FPC 空脚、电流表 U2 NC 脚十余处 min=0.0mil）。上面的
		// getAllPrimitivesByNet 按网络枚举永远拿不到它们，这里从
		// pcb_PrimitivePad.getAll() 补齐：只进 allPads 当障碍（net=''），
		// 不进 padMap（无网络不可布线）。
		try {
			const seenNetless = new Set(allPads.map(p => `${p.x.toFixed(2)},${p.y.toFixed(2)}`));
			const padPrims = await eda.pcb_PrimitivePad.getAll();
			let netlessAdded = 0;
			for (const sp of padPrims) {
				const p = sp as any;
				const net: string = p.net ?? p.netName ?? '';
				if (net) continue;   // 有网络的焊盘已在上面按网络枚举收集
				const x: number = p.x ?? p.center?.x ?? 0;
				const y: number = p.y ?? p.center?.y ?? 0;
				if (!x && !y) continue;
				const key = `${x.toFixed(2)},${y.toFixed(2)}`;
				if (seenNetless.has(key)) continue;
				seenNetless.add(key);
				const padShape = Array.isArray(p.pad) ? p.pad : [];
				let wMil = padShape.length >= 3 ? (padShape[1] ?? 0) : 0;
				let hMil = padShape.length >= 3 ? (padShape[2] ?? 0) : 0;
				const hole = p.hole;
				const holeMil = Array.isArray(hole) && hole.length >= 2 ? (hole[1] ?? 0) : 0;
				if (wMil === 0 && hMil === 0) { wMil = 32; hMil = 34; }
				const rawLayer: number = p.layerId ?? p.layer ?? 1;
				// TH 判定只认 layer 12：层 1/2 SMD 带封装库干扰小孔（如 LQFP 的
				// hole=8.5mil 假孔）不是通孔，误判会把它挤出 SMD 出线候选名单
				let padType: 'th' | 'smd' | 'via' = 'smd';
				if (rawLayer === 12) padType = 'th';
				else if (p.isVia || p.type === 'via') padType = 'via';
				let layer: number = rawLayer;
				if (padType === 'smd' && (rawLayer === 12 || rawLayer > 2)) layer = 1;
				allPads.push({
					x, y, net: '', id: p.globalIndex || p.uuid || `netless_${allPads.length}`,
					layer, padType, holeDiameter: holeMil,
					radius: Math.max(wMil / 2, hMil / 2, holeMil / 2),
					bodyRadius: Math.max(Math.min(wMil, hMil) / 2, holeMil / 2),
					rotation: p.rotation ?? 0,
					index: allPads.length,
				});
				netlessAdded++;
			}
			if (netlessAdded) {
				this.onLog?.(`补充 ${netlessAdded} 个无网络焊盘作为障碍（NC 脚/安装孔等，DRC 照查间距）`, 'info');
			}
		} catch (err) {
			console.warn('[TopologicalRouter] 无网络焊盘收集失败:', err);
		}

		this.onLog?.(`解析完成：${allPads.length} 个焊盘，${padMap.size} 个可布线网络`, 'info');
		return { padMap, allPads };
	}

	/**
	 * 探测板子的铜层数：单面板不能往底层（layer 2）出图。
	 *
	 * `getAllLayers()` 会把**停用的内层**也返回（16 铜层板可能返回 34 个 SIGNAL），
	 * 真正在用的铜层要按 `layerStatus === 1` 过滤。
	 * API 为 BETA，取不到时按双面板处理（保持历史行为）。
	 */
	private async detectCopperLayers(): Promise<number> {
		try {
			const layers = await eda.pcb_Layer.getAllLayers();
			if (Array.isArray(layers)) {
				const copper = layers.filter(l =>
					l?.type === 'SIGNAL' && (l as any).layerStatus === 1);
				if (copper.length >= 1) {
					this.onLog?.(`铜层数：${copper.length}`, 'info');
					return copper.length;
				}
			}
		} catch (err) {
			this.onLog?.('无法读取铜层数，按双面板处理', 'info');
		}
		return 2;
	}

	/**
	 * 参数弹窗打开时读取板级信息：可用铜层与可布线网络清单。
	 *
	 * 只做只读枚举，不修改画布；取不到时退回「顶层 + 底层」的保守清单。
	 */
	async inspectBoard(): Promise<BoardInspection> {
		let layers: BoardInspection['layers'] = [
			{ id: 1, name: '顶层 F.Cu', available: true },
			{ id: 2, name: '底层 B.Cu', available: true },
		];
		try {
			const all = await eda.pcb_Layer.getAllLayers();
			const copper = (all || []).filter((l: any) => l?.type === 'SIGNAL' && l.layerStatus === 1);
			if (copper.length >= 1) {
				const top = copper.find((l: any) => l.id === 1);
				const bottom = copper.find((l: any) => l.id === 2);
				layers = [{
					id: 1,
					name: top?.name ? `顶层 ${top.name}` : '顶层 F.Cu',
					available: true,
				}];
				if (bottom) {
					layers.push({ id: 2, name: `底层 ${bottom.name}`, available: true });
				}
			}
		} catch { /* 保持保守清单 */ }

		const nets: BoardInspection['nets'] = [];
		try {
			const names = await eda.pcb_Net.getAllNetsName();
			for (const name of names || []) {
				try {
					const primitives = await eda.pcb_Net.getAllPrimitivesByNet(name);
					let padCount = 0;
					for (const prim of primitives || []) {
						const p = prim as any;
						if (p.type !== undefined && p.type !== 1) continue;
						const x = p.center?.x ?? p.x ?? p.startX ?? 0;
						const y = p.center?.y ?? p.y ?? p.startY ?? 0;
						if (x === 0 && y === 0 && !p.center) continue;
						padCount++;
					}
					if (padCount >= 2) {
						nets.push({
							name,
							padCount,
							kind: isGroundNetName(name) ? 'ground' : (isPowerNetName(name) ? 'power' : 'signal'),
						});
					}
				} catch { /* 单个网络读取失败不影响其它 */ }
			}
		} catch { /* 无网络时返回空清单 */ }
		nets.sort((a, b) => a.name.localeCompare(b.name, 'zh-CN', { numeric: true }));

		return { layers, nets };
	}

	/** 收集静态铜层障碍（折线/线条/填充/板框），不含焊盘 */
	private async collectStaticObstacles(drc: DRCRules): Promise<RBObstacle[]> {
		const obstacles: RBObstacle[] = [];
		const parsePath = parsePolygonPath;

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
			const lineClear = Math.max(drc.trackToLineMil, 8);
			const r = lw / 2 + lineClear + drc.traceWidthMil / 2 + 2;
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
			if (layer === 11) {
				// 板框常用「4 条独立 Line」绘制（不是闭合 Polyline）——漏收
				// 就是过孔打在板框上、走线贴边（实测用户板）。口径与 Polyline
				// 板框一致：阻挡所有层 + trackToBoard 间距。
				const lw = l.lineWidth ?? 10;
				const r = lw / 2 + drc.trackToBoardMil + drc.traceWidthMil / 2;
				obstacles.push(...buildPolylineObstacle(
					[{ x: l.startX ?? 0, y: l.startY ?? 0 }, { x: l.endX ?? 0, y: l.endY ?? 0 }],
					r, undefined, undefined, 'board', true,
				));
				continue;
			}
			if (layer !== 1 && layer !== 2 && layer !== 12) continue;
			const lw = l.lineWidth ?? 10;
			const lineClear = Math.max(drc.trackToLineMil, 8);
			const r = lw / 2 + lineClear + drc.traceWidthMil / 2 + 2;
			obstacles.push(...buildPolylineObstacle(
				[{ x: l.startX ?? 0, y: l.startY ?? 0 }, { x: l.endX ?? 0, y: l.endY ?? 0 }],
				r, net || undefined, layer, `line:${l.primitiveId ?? ''}`,
			));
		}

		// 圆弧（板框圆角 + 圆弧走线）：采样成折线障碍
		const arcs = await eda.pcb_PrimitiveArc.getAll();
		for (const arc of arcs) {
			const o = arc as any;
			const net: string = o.net ?? '';
			const layer: number = o.layerId ?? o.layer ?? 1;
			const lw = o.lineWidth ?? 10;
			const sx = o.startX ?? 0, sy = o.startY ?? 0;
			const ex = o.endX ?? 0, ey = o.endY ?? 0;
			const sweep = (o.arcAngle ?? 0) * Math.PI / 180;
			const L = Math.hypot(ex - sx, ey - sy);
			if (!sweep || Math.abs(Math.sin(sweep / 2)) < 1e-6 || L < 1e-6) continue;
			// 由弦 + 扫角反推圆心（sweep 正 = 数学正方向，与 EDA 渲染约定一致）
			const R = L / (2 * Math.abs(Math.sin(sweep / 2)));
			const sgn = sweep > 0 ? 1 : -1;
			const h = sgn * R * Math.cos(sweep / 2);
			const nx = -(ey - sy) / L, ny = (ex - sx) / L;
			const cx = (sx + ex) / 2 + h * nx, cy = (sy + ey) / 2 + h * ny;
			const a0 = Math.atan2(sy - cy, sx - cx);
			const pts: Array<{ x: number; y: number }> = [];
			const steps = Math.max(6, Math.ceil(Math.abs(sweep) / (Math.PI / 12)));   // ≤15°/步
			for (let k = 0; k <= steps; k++) {
				const ang = a0 + (Math.abs(sweep) * k) / steps * sgn;
				pts.push({ x: cx + R * Math.cos(ang), y: cy + R * Math.sin(ang) });
			}
			if (layer === 11) {
				const r = lw / 2 + drc.trackToBoardMil + drc.traceWidthMil / 2;
				obstacles.push(...buildPolylineObstacle(pts, r, undefined, undefined, 'board', true));
			} else if (layer === 1 || layer === 2 || layer === 12) {
				const lineClear = Math.max(drc.trackToLineMil, 8);
				const r = lw / 2 + lineClear + drc.traceWidthMil / 2 + 2;
				obstacles.push(...buildPolylineObstacle(pts, r, net || undefined, layer, `arc:${sx.toFixed(0)},${sy.toFixed(0)}`));
			}
		}

		// 填充（铜皮矩形/多边形，无网络视为禁布）
		const fills = await eda.pcb_PrimitiveFill.getAll();
		for (const fill of fills) {
			const f = fill as any;
			const layer: number = f.layerId ?? f.layer ?? 1;
			if (layer !== 1 && layer !== 2 && layer !== 12) continue;
			const lw = f.lineWidth ?? 0;
			const net: string = f.net ?? '';
			const fillClear = Math.max(drc.trackToFillMil, 10);
			const r = lw / 2 + fillClear + drc.traceWidthMil / 2 + 2;
			const poly = f.complexPolygon?.polygon ?? f.complexPolygon ?? f.polygon?.polygon ?? f.polygon ?? [];
			const pts = parsePath(poly);
			if (pts.length >= 3) {
				obstacles.push(...buildPolygonObstacle(pts, r, net || undefined, layer, `fill:${pts[0]?.x},${pts[0]?.y}`, true));
			}
		}

		// 既有过孔（设计里手工放置 / 上一轮残留）：贯通各层的铜环，布线必须避让。
		// 旧实现完全没收集——实测 39 个既有过孔对路由器隐形，走线直接压过
		// （EDA DRC 报 Hole to Track / SMD Pad to Via）。
		// 注：clearExistingTraces 会先删除待布网络的过孔，故此处只剩真正需要避让的。
		const viaPrims = await eda.pcb_PrimitiveVia.getAll();
		const viaClear = Math.max(drc.trackToTrackMil, drc.trackToPadMil);
		for (const v of viaPrims ?? []) {
			const vp = v as any;
			const vx = vp.x ?? vp.center?.x;
			const vy = vp.y ?? vp.center?.y;
			if (typeof vx !== 'number' || typeof vy !== 'number') continue;
			const dia = Number(vp.diameter) || 24;
			obstacles.push({
				id: `via${obstacles.length}`,
				kind: 'circle',
				x: vx, y: vy,
				r: dia / 2 + viaClear + drc.traceWidthMil / 2 + 0.1,
				net: vp.net || undefined,
				layer: undefined,
				group: `via:${vp.net ?? ''}:${vx.toFixed(1)},${vy.toFixed(1)}`,
				noPrune: true,
			});
		}

		this.onLog?.(`静态障碍：${obstacles.length} 个圆（阻挡 ${obstacles.filter(o => o.blockerOnly).length}）`, 'info');
		return obstacles;
	}

	// ── 主流程 ──────────────────────────────────

	async route(options: RouterUiOptions): Promise<RouteSummary> {
		if (this.isRouting) throw new Error('布线正在进行中');
		this.isRouting = true;
		this.shouldCancel = false;
		this.startedAt = Date.now();
		this.currentOptions = options;
		this.worker = null;
		this.engine = null;
		this.forceFinish = null;

		try {
			// 重新布线：先清理上一轮由本扩展写入画布的图元。否则旧走线会在本轮
			// 被 collectStaticObstacles 当成静态障碍，迫使新走线绕行甚至叠画。
			if (this.netPrimIds.size || this.touchedNets.size) {
				const previous = new Set([...this.netPrimIds.keys(), ...this.touchedNets]);
				this.onProgress?.({ stage: '清理画布', percentage: 2, message: '正在清理上一轮走线...' });
				for (const name of previous) await this.deleteNet(name);
				this.onLog?.(`重新布线：已清理上一轮 ${previous.size} 个网络的走线`, 'info');
			}

			this.netPrimIds.clear();
			this.touchedNets.clear();
			this.netGeom.clear();
			this.pendingPlanned.clear();
			this.routedNames.clear();
			this.failedNames.clear();
			this.netPadStats.clear();
			this.totalPads = 0;
			this.padShapeCache.clear();
			this.applyPending.clear();
			this.applyDraining = false;
			const ws = this.applyWaiters.splice(0);
			for (const w of ws) w();
			this.totalNets = 0;

			this.onProgress?.({ stage: '读取规则', percentage: 3, message: '正在读取 DRC 间距规则...' });
			const drcBase = await fetchDRCRules(this.onLog);
			const drc = applyRouterOptionsToDRC(drcBase, options);

			// 缓存焊盘原始形状（getAllPrimitivesByNet 的焊盘没有 pad/hole 字段）
			try {
				const padPrims = await eda.pcb_PrimitivePad.getAll();
				for (const sp of padPrims) {
					const p = sp as any;
					if (Array.isArray(p.pad)) {
						const key = `${(p.x ?? 0).toFixed(2)},${(p.y ?? 0).toFixed(2)}`;
						this.padShapeCache.set(key, {
							pad: p.pad,
							hole: Array.isArray(p.hole) ? p.hole : null,
							rotation: p.rotation ?? 0,
						});
					}
				}
			} catch { /* 形状缓存失败则用默认圆 */ }

			this.onProgress?.({ stage: '解析 PCB', percentage: 8, message: '正在获取 PCB 数据...' });
			const { padMap, allPads } = await this.collectPads();
			if (padMap.size === 0) throw new Error('没有可布线的网络');

			const excluded = new Set(options.excludeNets ?? []);
			const networks = Array.from(padMap.entries())
				.filter(([name]) => !excluded.has(name))
				.map(([name, pads]) => ({ name, pads }));
			if (excluded.size) {
				this.onLog?.(`按设置排除 ${Array.from(excluded).filter(n => padMap.has(n)).length} 个网络`, 'info');
			}
			if (networks.length === 0) throw new Error('所有网络都被排除，没有可布线的网络');

			// 清掉本轮要布的网络在画布上的既有走线/过孔（含上一次会话留下的）。
			// 否则它们会被 collectStaticObstacles 当成永久静态铜：每个焊盘旁都在
			// 相邻网络的旧线 → 新布线自堵，整板 0 布通（2026-10-24 实测
			// case/rubberband-router：468 段旧线把 14 个网络全部封死）。
			// 这与「重新布线」语义一致；排除网络的手工走线不受影响。
			if (options.clearExistingTraces !== false) {
				this.onProgress?.({ stage: '清理旧走线', percentage: 12, message: '正在清理待布网络的既有走线...' });
				try {
					const purged = await this.purgeNetsBatch(new Set(networks.map(n => n.name)));
					if (purged.prims > 0) {
						this.onLog?.(`已清理 ${purged.nets} 个网络的 ${purged.prims} 段既有走线（重新布线）`, 'info');
					}
				} catch { /* 清理失败不阻断布线 */ }
			}

			this.onProgress?.({ stage: '构建障碍', percentage: 15, message: '正在构建障碍物...' });
			const staticObstacles = await this.collectStaticObstacles(drc);
			this.totalNets = networks.length;
			// 飞线布通率的分母：参与布线的焊盘总数（排除网络不计）
			this.totalPads = networks.reduce((sum, n) => sum + n.pads.length, 0);
			this.netPadStats.clear();
			for (const n of networks) this.netPadStats.set(n.name, { total: n.pads.length, routed: 0 });

			const padShapes: Record<string, PadShape> = {};
			for (const [key, value] of this.padShapeCache) padShapes[key] = value;

			// 布线层设置：当前引擎只有顶层（1）与底层（2）。关闭底层即单层布线，
			// 同时禁用一切过孔换层路径；底层不存在时以实际板层为准。
			const detectedLayers = await this.detectCopperLayers();
			const bottomEnabled = (options.routeLayers ?? [1, 2]).includes(2) && detectedLayers >= 2;
			if (!bottomEnabled && detectedLayers >= 2) {
				this.onLog?.('按设置仅使用顶层布线（底层已关闭）', 'info');
			}

			const input: PlanningInput = {
				drc,
				allPads,
				networks,
				staticObstacles,
				padShapes,
				budgetMs: options.budgetMs,
				copperLayers: bottomEnabled ? detectedLayers : 1,
				options: {
					// 首轮只求连通；连通完成后再做中心到中心最短路拉紧优化
					connectivityFirst: true,
					optimize: options.finalOptimize !== false,
					allowViaFallback: options.viaFallback !== false,
					allowViaInPad: options.allowViaInPad === true,
					escapeClearanceMil: options.escapeClearanceMil,
					ripupRounds: options.ripupRounds,
					powerNetsFirst: options.powerNetsFirst,
					netOrder: options.netOrder,
				},
			};

			this.onProgress?.({
				stage: '规划中',
				percentage: 18,
				message: options.livePreview ? '正在规划并实时写入走线...' : '正在规划走线...',
			});

			const worker = createPlanningWorker();
			let result: PlanningResult;
			if (worker) {
				this.onLog?.('规划引擎：Web Worker（主线程不参与计算）', 'info');
				result = await this.planInWorker(worker, input);
			} else {
				this.onLog?.('当前环境不支持 Web Worker，回退到主线程计算', 'warn');
				result = await this.planOnMainThread(input);
			}

			// 等待所有实时写入完成
			await this.applyIdle();

			// 最终对账：完全失败的网不得残留图元；部分连通的网要保留已布走线
			// （否则一条连接失败会把整网其它已布好的线段一并删掉）。
			const partial = new Set(result.partial ?? []);
			for (const name of result.failed) {
				if (partial.has(name)) continue;
				this.netGeom.delete(name);
				if (options.livePreview === false) {
					this.pendingPlanned.delete(name);
					continue;
				}
				await this.deleteNet(name);
			}

			// 收尾对账：EDA 的批量删除偶发不生效，运行中途被替换掉的尝试走线
			// 可能残留在画布上（与最终走线叠画、引发 DRC 冲突）。规划结束后
			// 文档已空闲，按**引擎终态**（result）逐网清写一遍——不能用消息
			// 历史（pendingPlanned）：引擎结尾还有终局修复（拆除违规过孔等），
			// 消息历史停在修复之前，画布会残留已被引擎丢弃的图元（实测过孔
			// 间距违规在画布上复现、引擎侧却已清除）。
			if (options.livePreview !== false && !this.shouldCancel) {
				this.finalizing = true;   // 定稿：之后的迟到消息不再写画布
				const finalPrims = new Map<string, RenderedPrim[]>();
				const finalVias = new Map<string, PlannedVia[]>();
				for (const n of result.nets) finalPrims.set(n.name, n.prims);
				for (const v of result.vias) {
					const arr = finalVias.get(v.net) ?? [];
					arr.push(v);
					finalVias.set(v.net, arr);
				}
				// 一次性全局清理：逐网 purgeNetFromDocument 会为每个网全文档
				// 扫描 3 类图元 × 2 轮（64 个网 = 数百次全扫，实测收尾吃掉
				// ~400s）。这里把所有托管网络一次扫完批量删，然后直接写入。
				const managed = new Set<string>([
					...this.routedNames, ...this.failedNames,
					...finalPrims.keys(), ...finalVias.keys(),
					...this.touchedNets, ...this.netPrimIds.keys(),
				]);
				const purgedTotal = await this.purgeManagedNets(managed);
				if (purgedTotal) this.onLog?.(`收尾：一次性清理 ${purgedTotal} 个图元`, 'info');
				// 记录已清空：writeNet 内部的 deleteNet 会因此直接返回，
				// 不再触发按网名的全文档扫描
				this.netPrimIds.clear();
				this.touchedNets.clear();
				for (const name of managed) {
					const prims = finalPrims.get(name) ?? [];
					const vias = finalVias.get(name) ?? [];
					if (!prims.length) continue;
					await this.writeNet(name, prims, vias, true);
				}
				// 关键：清空 pendingPlanned。它保存的是消息历史（终局修复之前
				// 的几何），下面的 flushPending 若无脑重写，会把刚写好的终态
				// 覆盖回旧几何 —— 这正是「引擎已去重、画布仍残留违规过孔」的元凶。
				this.pendingPlanned.clear();
			}

			// 无论是否实时预览，都必须等画布写入队列排空再报完成 ——
			// 否则进度条已 100%、按钮可点，画布却还在逐网重绘（用户可见的
			// 「拆线重试」假象，2026-10-24 实测）
			await this.flushPending();

			// 兜底核对：画布必须与引擎终态完全一致。实测 EDA 的按 ID 删除
			// 偶发漏删、重复写入会留下同坐标幽灵过孔（引擎侧已去重，画布仍在，
			// EDA DRC 照样报 Track to Via）。这里按终态扫描一次，删掉多余的，
			// 数量不符的整网重写。
			if (options.livePreview !== false && !this.shouldCancel) {
				await this.reconcileCanvas(result);
			}

			// 以规划结果为准刷新统计
			if (!this.shouldCancel) {
				this.routedNames = new Set(result.routedNets);
				this.failedNames = new Set(result.failed);
				this.emitStats();
			}

			const stats = this.snapshotStats();
			this.onProgress?.({
				stage: this.shouldCancel ? '已停止' : '完成',
				percentage: 100,
				message: this.shouldCancel ? '布线已停止' : '布线完成',
			});

			return {
				...stats,
				cancelled: this.shouldCancel,
				failed: result.failed,
			};
		} catch (error) {
			console.error('[TopologicalRouter] 布线错误:', error);
			this.onLog?.('布线失败: ' + (error instanceof Error ? error.message : String(error)), 'error');
			throw error;
		} finally {
			if (this.cancelTimer) {
				clearTimeout(this.cancelTimer);
				this.cancelTimer = null;
			}
			try { (this.worker as any)?.terminate?.(); } catch { /* ignore */ }
			this.worker = null;
			this.engine = null;
			this.forceFinish = null;
			this.finalizing = false;
			this.stopLiveTimer();
			this.isRouting = false;
		}
	}

	/** 在 Worker 中规划；每完成一个网络即把结果交给主线程写入 */
	private planInWorker(worker: Worker, input: PlanningInput): Promise<PlanningResult> {
		return new Promise<PlanningResult>((resolve, reject) => {
			let settled = false;
			this.worker = worker;

			const finish = (fn: () => void): void => {
				if (settled) return;
				settled = true;
				this.forceFinish = null;
				// 等实时写入队列清空再收尾
				this.applyIdle().then(fn, fn);
			};

			this.forceFinish = () => finish(() => resolve({
				nets: [],
				vias: [],
				failed: [...this.failedNames],
				partial: [],
				routedNets: [...this.routedNames],
				logs: [],
			}));

			worker.onmessage = (event: MessageEvent) => {
				const msg = event.data as any;
				if (!msg) return;
				switch (msg.type) {
					case 'log':
						this.onLog?.(msg.message, msg.level);
						break;
					case 'progress':
						this.onProgress?.({
							stage: '规划中',
							percentage: msg.percentage,
							message: msg.message,
						});
						break;
					case 'net':
						if (msg.ok !== false) {
							this.routedNames.add(msg.name);
							this.failedNames.delete(msg.name);
						} else {
							this.routedNames.delete(msg.name);
							this.failedNames.add(msg.name);
						}
						if (msg.pads) {
							this.netPadStats.set(msg.name, {
								total: Number(msg.pads.total) || 0,
								routed: Number(msg.pads.routed) || 0,
							});
						}
						this.enqueueWrite('net:' + msg.name, () => this.applyNet(msg.name, msg.prims ?? [], msg.vias ?? [], msg.ok !== false));
						break;
					case 'ripup':
						for (const name of msg.names ?? []) {
							this.routedNames.delete(name);
							this.failedNames.add(name);
							const padStat = this.netPadStats.get(name);
							if (padStat) padStat.routed = 0;
						}
						this.enqueueWrite('ripup:' + (msg.names ?? []).join(','), () => this.applyRipup(msg.names ?? []));
						break;
					case 'done':
						finish(() => resolve(msg.result as PlanningResult));
						break;
					case 'error':
						finish(() => reject(new Error(msg.message ?? 'Worker 规划失败')));
						break;
					default:
						break;
				}
			};

			worker.onerror = (event: ErrorEvent) => {
				const message = event?.message || 'Worker 运行出错';
				finish(() => reject(new Error(message)));
			};

			worker.postMessage({ type: 'plan', input });
		});
	}

	/** Worker 不可用时的主线程回退（同样支持实时写入） */
	private async planOnMainThread(input: PlanningInput): Promise<PlanningResult> {
		const engine = new PlanningEngine();
		this.engine = engine;
		return engine.plan(input, {
			onLog: (message, level) => this.onLog?.(message, level),
			onProgress: (percentage, message) => this.onProgress?.({ stage: '规划中', percentage, message }),
			onNetUpdate: (name, prims, vias, ok, pads) => {
				if (ok) {
					this.routedNames.add(name);
					this.failedNames.delete(name);
				} else {
					this.failedNames.add(name);
				}
				if (pads) this.netPadStats.set(name, { total: pads.total, routed: pads.routed });
				this.enqueueWrite('net:' + name, () => this.applyNet(name, prims, vias, ok));
			},
			onRipup: (names) => {
				for (const name of names) {
					this.routedNames.delete(name);
					this.failedNames.add(name);
					const padStat = this.netPadStats.get(name);
					if (padStat) padStat.routed = 0;
				}
				this.enqueueWrite('ripup:' + names.join(','), () => this.applyRipup(names));
			},
			isCancelled: () => this.shouldCancel,
		});
	}

	// ── 画布写入 ────────────────────────────────

	/** 实时模式：替换该网络的画布图元；预览关闭时仅暂存 */
	private async applyNet(name: string, prims: RenderedPrim[], vias: PlannedVia[], ok: boolean): Promise<void> {
		if (this.finalizing) return;   // 收尾后不再接受逐网消息
		if (!ok) {
			// 失败网络的残缺图元不写入，避免半截走线
			if (this.currentOptions?.livePreview !== false) await this.deleteNet(name);
			else this.pendingPlanned.delete(name);
			return;
		}
		// 记录该网最终规划（收尾对账用它全量清写，见 run() 末尾）
		this.pendingPlanned.set(name, { prims, vias });
		if (this.currentOptions?.livePreview === false) {
			this.netGeom.set(name, { segments: prims.length, vias: vias.length });
			this.emitStats();
			return;
		}
		// 全局节拍刷新：画布统一每 5 秒刷新一次（用户指定）。不再「引擎报一次
		// 就写一次」——撕毁重试中同一网络一轮会输出 5+ 次，每次都 delete+create
		// 全部图元并触发 EDA 重算飞线，是预览模式最大的时间开销。现在是把期间
		// 累积的最新几何存进 livePending，节拍到了一次性批量写入。
		// 最终几何仍由收尾对账（引擎终态）保证，刷新节拍不会丢更新。
		this.livePending.set(name, { prims, vias });
		this.ensureLiveTimer();
	}

	private async applyRipup(names: string[]): Promise<void> {
		if (this.finalizing) return;
		for (const name of names) {
			this.netGeom.delete(name);
			this.pendingPlanned.delete(name);
			this.lastWriteAt.delete(name);
			this.netWrittenSig.delete(name);
			if (this.currentOptions?.livePreview === false) continue;
			// 删除也进节拍队列（立即删同样触发 EDA 重算），prims 为空表示删除
			this.livePending.set(name, { prims: [], vias: [] });
			this.ensureLiveTimer();
		}
		this.emitStats();
	}

	/** 删除某网络已写入画布的全部图元（ID 批量删除 + 按网络扫描兜底） */
	private async deleteNet(name: string): Promise<void> {
		const rec = this.netPrimIds.get(name);
		this.netPrimIds.delete(name);
		if (rec) {
			try {
				if (rec.lines.length) await eda.pcb_PrimitiveLine.delete(rec.lines);
			} catch (err) {
				console.warn('[TopologicalRouter] 删除走线失败:', err);
			}
			try {
				if (rec.arcs.length) await eda.pcb_PrimitiveArc.delete(rec.arcs);
			} catch (err) {
				console.warn('[TopologicalRouter] 删除圆弧失败:', err);
			}
			try {
				if (rec.vias.length) await eda.pcb_PrimitiveVia.delete(rec.vias);
			} catch (err) {
				console.warn('[TopologicalRouter] 删除过孔失败:', err);
			}
		}
		if (!rec && !this.touchedNets.has(name)) return;

		// EDA 的批量删除偶发只生效一部分；按网络名再扫描清理，避免残留走线
		const leftover = await this.purgeNetFromDocument(name);
		if (leftover > 0) {
			this.onLog?.(`已清理网络 ${name} 的 ${leftover} 个残留图元`, 'info');
		}
		this.touchedNets.delete(name);
	}

	/**
	 * 批量清理一组网络的既有走线/过孔（重新布线语义）。
	 * 一次扫描文档按网络名过滤，避免逐网 getAll 的 O(网数 × 图元数) 开销。
	 */
	private async purgeNetsBatch(netNames: Set<string>): Promise<{ nets: number; prims: number }> {
		const hits = new Set<string>();
		let prims = 0;
		const collect = async (getAll: () => Promise<any[]>, del: (ids: string[]) => Promise<unknown>) => {
			try {
				const list = await getAll();
				const ids: string[] = [];
				for (const it of list ?? []) {
					const n = (it as any)?.net;
					if (!n || !netNames.has(n)) continue;
					const id = (it as any)?.primitiveId;
					if (typeof id === 'string' && id) { ids.push(id); hits.add(n); }
				}
				if (ids.length) {
					await del(ids);
					prims += ids.length;
				}
			} catch { /* 该类图元读取/删除失败不影响其它 */ }
		};
		await collect(() => eda.pcb_PrimitiveLine.getAll(), (ids) => eda.pcb_PrimitiveLine.delete(ids));
		await collect(() => eda.pcb_PrimitiveArc.getAll(), (ids) => eda.pcb_PrimitiveArc.delete(ids));
		await collect(() => eda.pcb_PrimitiveVia.getAll(), (ids) => eda.pcb_PrimitiveVia.delete(ids));
		return { nets: hits.size, prims };
	}

	/**
	 * 画布 ↔ 引擎终态兜底核对。
	 * EDA 的按 ID 删除偶发漏删、重复写入会留下同坐标幽灵过孔（引擎侧已去重，
	 * 画布仍在 → EDA DRC 照样报 Track to Via）。这里按终态扫描：删掉多余的
	 * 过孔，走线数量不符的整网重写（每个网最多重写一次，防止死循环）。
	 */
	private async reconcileCanvas(result: PlanningResult): Promise<void> {
		const primsOf = new Map<string, RenderedPrim[]>();
		const viasOf = new Map<string, PlannedVia[]>();
		for (const n of result.nets) primsOf.set(n.name, n.prims);
		for (const v of result.vias) {
			const arr = viasOf.get(v.net) ?? [];
			arr.push(v);
			viasOf.set(v.net, arr);
		}
		const managed = new Set<string>([...this.touchedNets, ...this.netPrimIds.keys(), ...primsOf.keys(), ...viasOf.keys()]);
		const rewritten = new Set<string>();
		for (let round = 0; round < 3; round++) {
			let changed = 0;
			// 1) 过孔：不在终态里的、超额的，一律删除
			try {
				const want = new Map<string, number>();
				for (const v of result.vias) {
					const k = `${v.net}|${v.x.toFixed(1)}|${v.y.toFixed(1)}`;
					want.set(k, (want.get(k) ?? 0) + 1);
				}
				const seen = new Map<string, number>();
				const del: string[] = [];
				for (const v of (await eda.pcb_PrimitiveVia.getAll()) ?? []) {
					const anyV = v as any;
					const net = String(anyV.net ?? '');
					if (!managed.has(net)) continue;
					const x = Number(anyV.x ?? anyV.center?.x);
					const y = Number(anyV.y ?? anyV.center?.y);
					if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
					const k = `${net}|${x.toFixed(1)}|${y.toFixed(1)}`;
					const remain = want.get(k) ?? 0;
					const used = seen.get(k) ?? 0;
					const id = anyV.primitiveId;
					if (!remain || used >= remain) { if (id) del.push(id); }
					else seen.set(k, used + 1);
				}
				if (del.length) {
					await eda.pcb_PrimitiveVia.delete(del);
					changed += del.length;
					this.onLog?.(`画布核对：删除 ${del.length} 个多余过孔`, 'info');
				}
			} catch { /* ignore */ }
			// 2) 走线/圆弧数量核对：不符的整网重写
			try {
				const lines = (await eda.pcb_PrimitiveLine.getAll()) ?? [];
				const arcs = (await eda.pcb_PrimitiveArc.getAll()) ?? [];
				const count = new Map<string, number>();
				for (const l of lines as any[]) { if (managed.has(l.net)) count.set(l.net, (count.get(l.net) ?? 0) + 1); }
				for (const a of arcs as any[]) { if (managed.has(a.net)) count.set(a.net, (count.get(a.net) ?? 0) + 1); }
				const rewrite: string[] = [];
				for (const name of managed) {
					if (rewritten.has(name)) continue;
					const expect = (primsOf.get(name) ?? []).length;
					const actual = count.get(name) ?? 0;
					if (actual !== expect) rewrite.push(name);
				}
				for (const name of rewrite) {
					rewritten.add(name);
					await this.purgeNetFromDocument(name);
					const ps = primsOf.get(name) ?? [];
					if (ps.length) await this.writeNet(name, ps, viasOf.get(name) ?? [], true);
					changed++;
				}
				if (rewrite.length) this.onLog?.(`画布核对：重写 ${rewrite.length} 个网络（数量与引擎终态不符）`, 'warn');
			} catch { /* ignore */ }
			if (!changed) break;
			await new Promise(r => setTimeout(r, 200));
		}
	}

	/** 一次性清理多个网络的全部图元（各扫描一次，批量删除） */
	private async purgeManagedNets(nets: Set<string>): Promise<number> {
		let removed = 0;
		const classes = ['pcb_PrimitiveLine', 'pcb_PrimitiveArc', 'pcb_PrimitiveVia'] as const;
		for (const cls of classes) {
			try {
				const items = await (eda as any)[cls].getAll();
				const ids = (items ?? [])
					.filter((o: any) => nets.has(String(o?.net ?? '')))
					.map((o: any) => o?.primitiveId)
					.filter((id: any): id is string => typeof id === 'string' && !!id);
				if (ids.length) {
					await (eda as any)[cls].delete(ids);
					removed += ids.length;
				}
			} catch { /* ignore */ }
		}
		return removed;
	}
	/** 按网络名扫描文档并删除其全部图元，返回实际发现的残留数量 */
	private async purgeNetFromDocument(name: string): Promise<number> {
		let removed = 0;
		for (let round = 0; round < 2; round++) {
			let batch = 0;
			try {
				const lines = await eda.pcb_PrimitiveLine.getAll();
				const ids = lines
					.filter((l: any) => l.net === name)
					.map((l: any) => l.primitiveId)
					.filter((id: any): id is string => typeof id === 'string' && !!id);
				if (ids.length) {
					await eda.pcb_PrimitiveLine.delete(ids);
					batch += ids.length;
				}
			} catch { /* ignore */ }
			try {
				const arcs = await eda.pcb_PrimitiveArc.getAll();
				const ids = arcs
					.filter((l: any) => l.net === name)
					.map((l: any) => l.primitiveId)
					.filter((id: any): id is string => typeof id === 'string' && !!id);
				if (ids.length) {
					await eda.pcb_PrimitiveArc.delete(ids);
					batch += ids.length;
				}
			} catch { /* ignore */ }
			try {
				const vias = await eda.pcb_PrimitiveVia.getAll();
				const ids = vias
					.filter((l: any) => l.net === name)
					.map((l: any) => l.primitiveId)
					.filter((id: any): id is string => typeof id === 'string' && !!id);
				if (ids.length) {
					await eda.pcb_PrimitiveVia.delete(ids);
					batch += ids.length;
				}
			} catch { /* ignore */ }
			removed += batch;
			if (batch === 0) break;
			await new Promise(r => setTimeout(r, 150));
		}
		return removed;
	}

	/** 写入一个网络的图元（先删除旧图元，再按链式顺序接续创建） */
	private async writeNet(name: string, primsRaw: RenderedPrim[], vias: PlannedVia[], force = false): Promise<void> {
		await this.deleteNet(name);
		if (this.currentOptions?.livePreview === false && !force) return;

		const drcWidth = this.currentOptions?.traceWidthMil ?? 10;
		const prims = mergePrims(primsRaw);
		const rec: PrimIdRecord = { lines: [], arcs: [], vias: [] };
		this.netPrimIds.set(name, rec);

		// 先建过孔再建走线：EDA 不会把已存在的走线自动连接到后创建的过孔
		for (const via of vias) {
			try {
				const created = await eda.pcb_PrimitiveVia.create(via.net, via.x, via.y, via.hole, via.dia);
				const id = primitiveIdOf(created);
				if (created) rec.vias.push(id || created);
			} catch (err) {
				console.warn(`[TopologicalRouter] 创建过孔失败 (${via.net}):`, err);
			}
		}

		let lastEnd: { x: number; y: number } | null = null;
		let count = 0;
		let written = 0;

		for (const pr of prims) {
			try {
				if (pr.kind === 'line') {
					// 极短线段也不能丢：它是切线链的接续段，丢了会断网
					let sx = pr.x1!, sy = pr.y1!;
					const ex = pr.x2!, ey = pr.y2!;
					if (lastEnd && Math.hypot(sx - lastEnd.x, sy - lastEnd.y) < 0.5) {
						sx = lastEnd.x; sy = lastEnd.y;
					}
					const created = await eda.pcb_PrimitiveLine.create(name, pr.layer, sx, sy, ex, ey, drcWidth, false);
					const ce = created as any;
					lastEnd = (ce && typeof ce.endX === 'number') ? { x: ce.endX, y: ce.endY } : { x: ex, y: ey };
					const id = primitiveIdOf(created);
					if (created) rec.lines.push(id || created);
					written++;
				} else {
					const deg = (pr.sweep ?? 0) * 180 / Math.PI;
					let sx = pr.cx! + pr.r! * Math.cos(pr.startAngle!);
					let sy = pr.cy! + pr.r! * Math.sin(pr.startAngle!);
					// 极小的弧：降级为等效直线段（不能跳过，否则切线链断网）
					if (Math.abs(deg) < 0.05) {
						const ex0 = pr.cx! + pr.r! * Math.cos(pr.startAngle! + (pr.sweep ?? 0));
						const ey0 = pr.cy! + pr.r! * Math.sin(pr.startAngle! + (pr.sweep ?? 0));
						if (lastEnd && Math.hypot(sx - lastEnd.x, sy - lastEnd.y) < 0.5) {
							sx = lastEnd.x; sy = lastEnd.y;
						}
						const createdL = await eda.pcb_PrimitiveLine.create(name, pr.layer, sx, sy, ex0, ey0, drcWidth, false);
						const ceL = createdL as any;
						lastEnd = (ceL && typeof ceL.endX === 'number') ? { x: ceL.endX, y: ceL.endY } : { x: ex0, y: ey0 };
						const idL = primitiveIdOf(createdL);
						if (createdL) rec.lines.push(idL || createdL);
						written++;
					} else if (Math.abs(Math.abs(deg) - 360) >= 0.05) {
						// 近整圆的弧无法用两点式表达：跳过（起终点几乎重合，不影响连通）
						const ex = pr.cx! + pr.r! * Math.cos(pr.startAngle! + (pr.sweep ?? 0));
						const ey = pr.cy! + pr.r! * Math.sin(pr.startAngle! + (pr.sweep ?? 0));
						if (lastEnd && Math.hypot(sx - lastEnd.x, sy - lastEnd.y) < 0.5) {
							sx = lastEnd.x; sy = lastEnd.y;
						}
						const created = await eda.pcb_PrimitiveArc.create(name, pr.layer, sx, sy, ex, ey, deg, drcWidth);
						const ce = created as any;
						lastEnd = (ce && typeof ce.endX === 'number') ? { x: ce.endX, y: ce.endY } : { x: ex, y: ey };
						const id = primitiveIdOf(created);
						if (created) rec.arcs.push(id || created);
						written++;
					}
				}
			} catch (err) {
				console.warn(`[TopologicalRouter] 写入图元失败 (${name}):`, err);
			}

			// 每写 32 个图元让渲染器呼吸一次，实现动态刷新
			if ((++count & 31) === 0) await breathe();
		}

		this.netGeom.set(name, { segments: written, vias: rec.vias.length });
		this.netWrittenSig.set(name, this.netSig(prims, vias));
		if (rec.lines.length || rec.arcs.length || rec.vias.length) this.touchedNets.add(name);
		this.emitStats();
	}

	/** 几何签名：用于跳过无变化的重写（坐标加权和 + 数量） */
	private netSig(prims: RenderedPrim[], vias: PlannedVia[]): string {
		let acc = 0;
		for (const p of prims) {
			acc += (p.x1 ?? p.cx ?? 0) * 0.37 + (p.y1 ?? p.cy ?? 0) * 0.11
				+ (p.x2 ?? p.r ?? 0) * 0.07 + (p.y2 ?? p.startAngle ?? 0) * 0.03 + (p.sweep ?? 0) * 0.5;
		}
		for (const v of vias) acc += v.x * 0.37 + v.y * 0.11;
		return prims.length + '|' + vias.length + '|' + acc.toFixed(2);
	}

	private async flushPending(): Promise<void> {
		for (const [name, planned] of this.pendingPlanned) {
			await this.writeNet(name, planned.prims, planned.vias, true);
		}
		this.pendingPlanned.clear();
	}

	// ── 统计与取消 ──────────────────────────────

	private snapshotStats(): RouteStats {
		let segments = 0;
		let vias = 0;
		for (const g of this.netGeom.values()) {
			segments += g.segments;
			vias += g.vias;
		}
		let routedPads = 0;
		let totalPads = 0;
		for (const p of this.netPadStats.values()) {
			routedPads += Math.min(p.routed, p.total);
			totalPads += p.total;
		}
		return {
			totalNets: this.totalNets,
			routedNets: this.routedNames.size,
			failedNets: this.failedNames.size,
			// 以参与布线网络的实际焊盘数为准；初始化前回退到预算值
			totalPads: totalPads || this.totalPads,
			routedPads,
			segments,
			vias,
			elapsedMs: Date.now() - this.startedAt,
		};
	}

	private emitStats(): void {
		this.onStats?.(this.snapshotStats());
	}

	cancel(): void {
		if (!this.isRouting) return;
		this.shouldCancel = true;
		this.onLog?.('正在停止布线...', 'warn');
		this.engine?.cancel();
		if (this.worker) {
			try { this.worker.postMessage({ type: 'cancel' }); } catch { /* ignore */ }
			// Worker 若在长同步段中未能响应，超时后强制结束并保留已写入结果
			if (!this.cancelTimer) {
				this.cancelTimer = setTimeout(() => {
					this.cancelTimer = null;
					try { this.worker?.terminate(); } catch { /* ignore */ }
					this.worker = null;
					this.forceFinish?.();
				}, 15000);
			}
		}
	}

	isActive(): boolean { return this.isRouting; }
}
