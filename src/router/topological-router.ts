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
	mergePrims, isGroundNetName, isPowerNetName, renderedLength,
} from './planning-engine';
import { createPlanningWorker, isWorkerSupported } from './worker-source';
import { parsePolygonPath } from './polygon-parse';
import {
	RouterUiOptions, RouteStats, RouteSummary, BoardInspection, normalizeRouterOptions,
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

/**
 * 画布弧图元（两点式：startX/startY/endX/endY/arcAngle°）→ 引擎弧
 *（cx/cy/r/startAngle/sweep）。换算与 collectStaticObstacles 的弧采样同式。
 * 之前直接读 centerX/cx/sweep——EDA 弧根本没有这些字段，读回全是 undefined，
 * 拉紧回写后画布圆角全灭（实测 case9 81 弧 → 0）。
 */
function canvasArcToPrim(a: any, net: string): RenderedPrim | null {
	const layer: number = a.layer ?? a.layerId ?? 1;
	const sx = a.startX, sy = a.startY, ex = a.endX, ey = a.endY;
	const sweep = (Number(a.arcAngle ?? a.sweep ?? 0) * Math.PI) / 180;
	if (typeof sx !== 'number' || typeof sy !== 'number' ||
		typeof ex !== 'number' || typeof ey !== 'number') return null;
	const L = Math.hypot(ex - sx, ey - sy);
	if (!sweep || Math.abs(Math.sin(sweep / 2)) < 1e-6 || L < 1e-6) return null;
	const R = L / (2 * Math.abs(Math.sin(sweep / 2)));
	const sgn = sweep > 0 ? 1 : -1;
	const h = sgn * R * Math.cos(sweep / 2);
	const nx = -(ey - sy) / L, ny = (ex - sx) / L;
	const cx = (sx + ex) / 2 + h * nx, cy = (sy + ey) / 2 + h * ny;
	const a0 = Math.atan2(sy - cy, sx - cx);
	return { kind: 'arc', net, layer, cx, cy, r: R, startAngle: a0, sweep };
}

/**
 * 拉紧后几何是否与原几何「实质相同」：图元数/长度/端点都在容差内。
 * 相同的网跳过重写——否则 50 个网全删全建，观感就是「只看到删除重建」。
 */
function sameNetGeometry(
	origPrims: RenderedPrim[], origVias: PlannedVia[],
	newPrims: RenderedPrim[], newVias: PlannedVia[],
): boolean {
	if (origPrims.length !== newPrims.length || origVias.length !== newVias.length) return false;
	if (Math.abs(renderedLength(origPrims) - renderedLength(newPrims)) > 0.5) return false;
	for (let i = 0; i < origPrims.length; i++) {
		const a = origPrims[i], b = newPrims[i];
		if (a.kind !== b.kind || a.layer !== b.layer) return false;
		if (a.kind === 'line' && b.kind === 'line') {
			if (Math.abs((a.x1 ?? 0) - (b.x1 ?? 0)) > 0.5 || Math.abs((a.y1 ?? 0) - (b.y1 ?? 0)) > 0.5 ||
				Math.abs((a.x2 ?? 0) - (b.x2 ?? 0)) > 0.5 || Math.abs((a.y2 ?? 0) - (b.y2 ?? 0)) > 0.5) return false;
		} else if (a.kind === 'arc' && b.kind === 'arc') {
			if (Math.abs((a.cx ?? 0) - (b.cx ?? 0)) > 0.5 || Math.abs((a.cy ?? 0) - (b.cy ?? 0)) > 0.5 ||
				Math.abs((a.r ?? 0) - (b.r ?? 0)) > 0.5) return false;
		}
	}
	for (let i = 0; i < origVias.length; i++) {
		if (Math.hypot(origVias[i].x - newVias[i].x, origVias[i].y - newVias[i].y) > 0.5) return false;
	}
	return true;
}

export class TopologicalRouter {
	private onProgress?: ProgressCallback;
	private onLog?: LogCallback;
	private onStats?: StatsCallback;

	private isRouting = false;
	/** 拉紧已有布线进行中（独立菜单，不占 isRouting） */
	private tightening = false;
	/** 拉紧 Worker（求解在此，主线程只读写画布） */
	private tightenWorker: Worker | null = null;
	/** 取消时唤醒等待 Worker 的 Promise（否则终止 worker 后 await 永挂） */
	private tightenCancel: (() => void) | null = null;
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
	/** 混合修复递归深度（J/H 失败网转 RRouter 重布；限 1 层防环） */
	private hybridRepairDepth = 0;
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
			// 停止后（worker terminate 与清队列之间的窗口里可能还有迟到消息入队）
			// 彻底排空任务表并退出循环：停止 = 画布零写入
			if (this.shouldCancel) { this.applyPending.clear(); break; }
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

	/** 缓存焊盘原始形状（pcb_PrimitivePad.getAll 的 pad/hole 数组，按位置键） */
	private async cachePadShapes(): Promise<void> {
		this.padShapeCache.clear();
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
						holeRotation: p.holeRotation ?? 0,
					});
				}
			}
		} catch { /* 形状缓存失败则用默认圆 */ }
	}

	/**
	 * 拉紧已有布线（独立菜单命令）：对当前 PCB 上所有已布网络做
	 * 真拉紧（可视图逐边重解）+ 切线圆角圆润，然后回写画布。
	 * 复用 J/H 引擎结果的形态优化管线——让用户无需完整布线即可
	 * 单独查看/使用橡皮筋优化效果（含另外两个引擎的产物）。
	 */
	public async tightenExistingRouting(): Promise<{ nets: number; changed: number; savedMil: number }> {
		this.shouldCancel = false;
		this.tightening = true;
		try {
			this.onProgress?.({ stage: '读取规则', percentage: 5, message: '正在读取 DRC 规则与画布几何...' });
			const drcBase = await fetchDRCRules(this.onLog);
			const drc = applyRouterOptionsToDRC(drcBase, normalizeRouterOptions({}));
			await this.cachePadShapes();
			const { padMap, allPads } = await this.collectPads();
			if (!padMap.size) throw new Error('当前 PCB 没有网络');

			const lines = (await eda.pcb_PrimitiveLine.getAll()) ?? [];
			const arcs = (await eda.pcb_PrimitiveArc.getAll()) ?? [];
			const vias = (await eda.pcb_PrimitiveVia.getAll()) ?? [];
			const byNet = new Map<string, { prims: RenderedPrim[]; vias: PlannedVia[] }>();
			for (const l of lines as any[]) {
				const net = String(l.net ?? '');
				if (!net || !padMap.has(net)) continue;
				const e = byNet.get(net) ?? { prims: [], vias: [] };
				e.prims.push({
					kind: 'line', net, layer: l.layer ?? l.layerId ?? 1,
					x1: l.startX ?? l.x, y1: l.startY ?? l.y, x2: l.endX, y2: l.endY,
					__id: String(l.primitiveId ?? ''), __width: Number(l.lineWidth) || undefined,
				} as RenderedPrim);
				byNet.set(net, e);
			}
			for (const a of arcs as any[]) {
				const net = String(a.net ?? '');
				if (!net || !padMap.has(net)) continue;
				const e = byNet.get(net) ?? { prims: [], vias: [] };
				const prim = canvasArcToPrim(a, net);
				if (prim) {
					(prim as any).__id = String(a.primitiveId ?? '');
					(prim as any).__width = Number(a.lineWidth) || undefined;
					e.prims.push(prim);
				}
				byNet.set(net, e);
			}
			for (const v of vias as any[]) {
				const net = String(v.net ?? '');
				if (!net || !padMap.has(net)) continue;
				const e = byNet.get(net) ?? { prims: [], vias: [] };
				e.vias.push({ net, x: v.x, y: v.y, dia: v.viaDiameter ?? 24, hole: v.holeDiameter ?? 12 });
				byNet.set(net, e);
			}
			const entries = [...byNet.entries()].map(([name, e]) => ({ name, ...e })).filter(e => e.prims.length);
			if (!entries.length) throw new Error('画布上没有可拉紧的走线');

			this.onProgress?.({ stage: '收集世界', percentage: 25, message: `收集静态世界与 ${entries.length} 个网络的走线...` });
			const staticWorld = await this.collectStaticObstacles(drc, true);

			// ── 切成「连续导线段」（分支点/过孔为边界）──
			const splitEngine = new PlanningEngine();
			splitEngine.initTightenWorld(drc, allPads, [], Object.fromEntries(this.padShapeCache));
			interface RunUnit {
				id: number; net: string; layer: number; width: number;
				prims: RenderedPrim[]; a: { x: number; y: number }; b: { x: number; y: number };
				ids: Array<{ kind: 'line' | 'arc'; id: string }>;
				oldLen: number;
			}
			const runs: RunUnit[] = [];
			for (const e of entries) {
				for (const seg of splitEngine.splitRunsForTighten(e.prims, e.vias)) {
					const ids = seg.prims
						.map(p => ({ kind: p.kind as 'line' | 'arc', id: String((p as any).__id ?? '') }))
						.filter(x => !!x.id);
					const width = Number((seg.prims[0] as any).__width) || drc.traceWidthMil;
					runs.push({
						id: runs.length, net: e.name, layer: seg.layer, width,
						prims: seg.prims, a: seg.a, b: seg.b, ids,
						oldLen: renderedLength(seg.prims),
					});
				}
			}
			if (!runs.length) throw new Error('没有可拉紧的连续导线段');
			const byId = new Map(runs.map(r => [r.id, r]));
			const total = runs.length;
			let done = 0, adopted = 0, savedMil = 0;

			const deleteRun = async (r: RunUnit): Promise<void> => {
				const lineIds = r.ids.filter(x => x.kind === 'line').map(x => x.id);
				const arcIds = r.ids.filter(x => x.kind === 'arc').map(x => x.id);
				try { if (lineIds.length) await eda.pcb_PrimitiveLine.delete(lineIds); } catch { /* ignore */ }
				try { if (arcIds.length) await eda.pcb_PrimitiveArc.delete(arcIds); } catch { /* ignore */ }
			};
			const writeRun = async (r: RunUnit, prims: RenderedPrim[]): Promise<void> => {
				let last: { x: number; y: number } | null = null;
				const newIds: Array<{ kind: 'line' | 'arc'; id: string }> = [];
				for (const pr of prims) {
					try {
						if (pr.kind === 'line') {
							let sx = pr.x1!, sy = pr.y1!;
							const ex = pr.x2!, ey = pr.y2!;
							if (last && Math.hypot(sx - last.x, sy - last.y) < 0.5) { sx = last.x; sy = last.y; }
							const created = await eda.pcb_PrimitiveLine.create(r.net, pr.layer ?? r.layer, sx, sy, ex, ey, r.width, false);
							last = { x: ex, y: ey };
							const id = primitiveIdOf(created);
							if (id) newIds.push({ kind: 'line', id });
						} else {
							const deg = (pr.sweep ?? 0) * 180 / Math.PI;
							const sx = pr.cx! + pr.r! * Math.cos(pr.startAngle!);
							const sy = pr.cy! + pr.r! * Math.sin(pr.startAngle!);
							const ex = pr.cx! + pr.r! * Math.cos(pr.startAngle! + (pr.sweep ?? 0));
							const ey = pr.cy! + pr.r! * Math.sin(pr.startAngle! + (pr.sweep ?? 0));
							if (Math.abs(deg) < 0.05) {
								const created = await eda.pcb_PrimitiveLine.create(r.net, pr.layer ?? r.layer, sx, sy, ex, ey, r.width, false);
								const id = primitiveIdOf(created);
								if (id) newIds.push({ kind: 'line', id });
							} else {
								const created = await eda.pcb_PrimitiveArc.create(r.net, pr.layer ?? r.layer, sx, sy, ex, ey, deg, r.width);
								const id = primitiveIdOf(created);
								if (id) newIds.push({ kind: 'arc', id });
							}
							last = { x: ex, y: ey };
						}
					} catch (err) {
						console.warn('[Tighten] 写入图元失败:', err);
					}
				}
				r.ids = newIds;
			};

			const onRun = async (msg: { id: number; prims: RenderedPrim[] | null; oldLen: number; newLen: number }): Promise<void> => {
				const r = byId.get(msg.id);
				done++;
				if (r && msg.prims && msg.prims.length) {
					await deleteRun(r);
					await writeRun(r, msg.prims);
					adopted++;
					savedMil += Math.max(0, msg.oldLen - msg.newLen);
				}
				this.onProgress?.({
					stage: '逐段拉紧', percentage: 30 + Math.round((done / total) * 65),
					message: `${r ? r.net : ''} ${done}/${total}（${(msg.newLen ?? msg.oldLen).toFixed(0)}mil）`,
					done, total, changed: adopted, savedMil,
				});
			};

			this.onProgress?.({
				stage: '逐段拉紧', percentage: 30,
				message: `共 ${total} 个连续导线段，按可回收余量排序处理...`,
				done: 0, total, changed: 0, savedMil: 0,
			});

			// ── 求解在 Worker（不卡主线程）；主线程只做删除/写入/刷新 ──
			const worker = isWorkerSupported() ? createPlanningWorker() : null;
			if (worker) {
				this.tightenWorker = worker;
				await new Promise<void>((resolve, reject) => {
					const w = worker;
					const pending = new Set<Promise<void>>();
					let settled = false;
					this.tightenCancel = () => { if (!settled) { settled = true; resolve(); } };
					w.onmessage = (ev: MessageEvent) => {
						const m = ev.data;
						if (!m) return;
						if (m.type === 'tightenRun') {
							const p = onRun(m).catch(() => { /* 单段失败不影响继续 */ }).then(() => breathe());
							pending.add(p);
							void p.finally(() => pending.delete(p));
						} else if (m.type === 'netRewrite') {
							// 整网拆线重布：等该网逐段写入落盘后，删掉全部旧段、写入新链路
							const p = Promise.all([...pending]).then(async () => {
								const rs = runs.filter(r => r.net === m.net);
								if (!rs.length || !m.prims?.length) return;
								for (const r of rs) await deleteRun(r);
								await writeRun(rs[0], m.prims);
								adopted++;
								savedMil += Math.max(0, (m.oldLen ?? 0) - (m.newLen ?? 0));
								this.onProgress?.({
									stage: '逐段拉紧', percentage: 30 + Math.round((done / total) * 65),
									message: `整网重布 ${m.net}：${Number(m.oldLen ?? 0).toFixed(0)} → ${Number(m.newLen ?? 0).toFixed(0)}mil`,
									done, total, changed: adopted, savedMil,
								});
							}).catch(() => { /* 单网失败不影响继续 */ }).then(() => breathe());
							pending.add(p);
							void p.finally(() => pending.delete(p));
						} else if (m.type === 'tightenDone') {
							this.onLog?.(
								`重解阶段：尝试 ${m.solveAttempts ?? 0} 段，采纳 ${m.solved ?? 0} 段，整网重布 ${m.rewritten ?? 0} 网，耗时 ${m.solveMs ?? 0}ms`,
								'info',
							);
							// 等所有写入落盘再收尾，保证统计与画布一致
							void Promise.all([...pending]).then(() => {
								this.tightenWorker = null;
								this.tightenCancel = null;
								try { w.terminate(); } catch { /* ignore */ }
								if (!settled) { settled = true; resolve(); }
							});
						} else if (m.type === 'error') {
							if (!settled) { settled = true; reject(new Error(String(m.message ?? 'worker error'))); }
						}
					};
					w.onerror = (e: any) => reject(new Error(String(e?.message ?? 'worker error')));
					w.postMessage({
						type: 'tighten', drc, allPads, baseWorld: staticWorld,
						padShapes: Object.fromEntries(this.padShapeCache),
						vias: entries.flatMap(e => e.vias),
						runs: runs.map(r => ({ id: r.id, net: r.net, layer: r.layer, a: r.a, b: r.b, prims: r.prims })),
					});
				});
			} else {
				// 主线程回退（无 Worker 环境）
				const eng = new PlanningEngine();
				eng.initTightenWorld(drc, allPads, staticWorld, Object.fromEntries(this.padShapeCache));
				const groupsById = new Map<number, string[]>();
				const currentById = new Map<number, RenderedPrim[]>();
				for (const r of runs) {
					groupsById.set(r.id, eng.addRunToWorld(r.prims));
					currentById.set(r.id, r.prims);
				}
				for (const e of entries) for (const v of e.vias) eng.addViaToWorld(v);
				const sorted = [...runs].sort((p, q) =>
					(q.oldLen - Math.hypot(q.b.x - q.a.x, q.b.y - q.a.y)) - (p.oldLen - Math.hypot(p.b.x - p.a.x, p.b.y - p.a.y)));
				for (const r of sorted) {
					if (this.shouldCancel) break;
					const oldLen = r.oldLen;
					const origPad = eng.padViolations(r.prims, r.net);
					let next: RenderedPrim[] | null = null;
					let newLen = oldLen;
					// 1) 拉直 + 圆润（便宜，对所有段）：string-pulling 后把拐角换弧
					const pulled = eng.tautenRun(r.prims, r.net);
					const pulledLen = renderedLength(pulled);
					const base = pulledLen < oldLen - 0.1 ? pulled : r.prims;
					const sm = eng.smoothRun(base, r.net);
					const candidate = renderedLength(sm) < renderedLength(base) - 0.1 ? sm : base;
					const candLen = renderedLength(candidate);
					const origArcN = r.prims.filter(p => p.kind === 'arc').length;
					const candArcN = candidate.filter(p => p.kind === 'arc').length;
					const improved = candArcN > origArcN || candLen < oldLen - 0.1;
					// 不做整条 run 的 primsClearOfOthers（见 worker 同处注释）：
					// 圆角/拉直已逐弧/逐线过了净距，只卡焊盘违规不恶化。
					if (improved && candLen <= oldLen + 0.5 &&
						eng.padViolations(candidate, r.net) <= origPad) {
						next = candidate; newLen = candLen;
					}
					// 2) 拆线重布（A*）：拉直没把拓扑改短的明显绕路段（不能用 !next：
					//    第 1 阶段几乎每段都能加出圆角，会让 A* 永远不跑）。
					const straight = Math.hypot(r.b.x - r.a.x, r.b.y - r.a.y);
					const detour = oldLen - straight;
					const stillDetoured = candLen > straight + Math.max(15, straight * 0.08);
					if (detour >= 25 && straight > 5 && stillDetoured) {
						const sc = await eng.solveRunSegment(r.a, r.b, r.layer, r.net);
						if (sc) {
							const sm2 = eng.smoothRun(sc, r.net);
							const tryCand = sm2.length && renderedLength(sm2) <= renderedLength(sc) + 0.5 ? sm2 : sc;
							const tl = renderedLength(tryCand);
							const tp = eng.padViolations(tryCand, r.net);
							const okShort = tl < oldLen - 0.5 && tp === 0;
							const okRepair = tp === 0 && origPad > 0;
							const better = !next || tl < newLen - 2;
							if (better && (okShort || okRepair) && await eng.primsClearOfOthers(tryCand, r.net)) {
								next = tryCand; newLen = tl;
							}
						}
					}
					if (next) {
						eng.removeWorldGroups(groupsById.get(r.id) ?? []);
						groupsById.set(r.id, eng.addRunToWorld(next));
						currentById.set(r.id, next);
					}
					await onRun({ id: r.id, prims: next, oldLen, newLen });
					await breathe();
				}

				// ── 整网拆线重布（MST 焊盘对）：解释见 worker 同处 ──
				const viasByNet = new Set(entries.flatMap(e => e.vias).map(v => v.net));
				const byNetRuns = new Map<string, RunUnit[]>();
				for (const r of runs) {
					const arr = byNetRuns.get(r.net);
					if (arr) arr.push(r); else byNetRuns.set(r.net, [r]);
				}
				for (const [net, rs] of byNetRuns) {
					if (this.shouldCancel) break;
					if (viasByNet.has(net)) continue;
					const layer = rs[0].layer;
					if (rs.some(q => q.layer !== layer)) continue;
					const oldLen = rs.reduce((s, q) => s + renderedLength(currentById.get(q.id) ?? q.prims), 0);
					if (oldLen <= 0) continue;
					const cand = await eng.rerouteNetMST(net, layer);
					if (!cand) continue;
					const newLen = renderedLength(cand);
					if (newLen > oldLen - Math.max(10, oldLen * 0.08)) continue;
					if (eng.padViolations(cand, net) > 0) continue;
					if (!await eng.primsClearOfOthers(cand, net)) continue;
					for (const q of rs) {
						eng.removeWorldGroups(groupsById.get(q.id) ?? []);
						await deleteRun(q);
					}
					eng.addRunToWorld(cand);
					await writeRun(rs[0], cand);
					adopted++;
					savedMil += oldLen - newLen;
					this.onProgress?.({
						stage: '逐段拉紧', percentage: 30 + Math.round((done / total) * 65),
						message: `整网重布 ${net}：${oldLen.toFixed(0)} → ${newLen.toFixed(0)}mil`,
						done, total, changed: adopted, savedMil,
					});
					await breathe();
				}
			}

			const cancelled = this.shouldCancel;
			this.onLog?.(`拉紧完成：${total} 段（采纳 ${adopted} 段，共省 ${savedMil.toFixed(0)}mil）`, 'success');
			this.onProgress?.({
				stage: cancelled ? '已停止' : '完成', percentage: 100,
				message: cancelled
					? `拉紧已停止：已处理 ${done}/${total} 段`
					: `拉紧完成：${adopted}/${total} 段有改善，共省 ${savedMil.toFixed(0)}mil`,
				done, total, changed: adopted, savedMil,
			});
			return { nets: entries.length, changed: adopted, savedMil };
		} finally {
			this.tightening = false;
		}
	}

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
			// 瞬时失败重试一次：getAllPrimitivesByNet 在文档忙时可能返回空/抛错，
			// 静默 continue 会让整个网的焊盘对路由器隐形 —— 走线/过孔直接压上
			// 去（case12 E2E 实测 D- 底层线压 GND 通孔焊盘 USB2_14，minDistance=0，
			// 离线永不复现 = 只有 E2E 实时采集才会踩中）。
			let primitives: any[] | null = null;
			for (let attempt = 0; attempt < 2; attempt++) {
				try { primitives = await eda.pcb_Net.getAllPrimitivesByNet(netName); } catch { primitives = null; }
				if (primitives && primitives.length) break;
				if (attempt === 0) await new Promise(r => setTimeout(r, 120));
			}
			if (!primitives || primitives.length === 0) {
				this.onLog?.(`网络 ${netName} 焊盘枚举为空（API 瞬时失败？重试后仍空）——该网焊盘对本轮隐形`, 'warn');
				continue;
			}

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
				// 采集期诊断：定位「按网枚举确定性丢 pad」（case9 P02 的
				// e189e13 每次 E2E 都丢 —— MST 退化为 pad1-pad3 直连）
				if ((globalThis as any).__RB_PAD_DEBUG__ && /^P0[012]$/.test(netName)) {
					const types: Record<string, number> = {};
					for (const prim of primitives) {
						const t = String((prim as any).type ?? (prim as any).pcbItemPrimitiveType ?? '?');
						types[t] = (types[t] ?? 0) + 1;
					}
					this.onLog?.(`[pad-debug] ${netName}: 原语 ${primitives.length}，type 分布 ${JSON.stringify(types)}，收 pad ${netPads.length}（@${netPads.map(p => `${p.x.toFixed(1)},${p.y.toFixed(1)}`).join(' ')}）`, 'info');
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
			// 按网络枚举收集到的焊盘位置集（用于补漏比对）
			const seenNetted = new Set(allPads.filter(p => p.net).map(p => `${p.x.toFixed(2)},${p.y.toFixed(2)}`));
			let netlessAdded = 0;
			let nettedPatched = 0;
			for (const sp of padPrims) {
				const p = sp as any;
				const net: string = p.net ?? p.netName ?? '';
				const x: number = p.x ?? p.center?.x ?? 0;
				const y: number = p.y ?? p.center?.y ?? 0;
				if (!x && !y) continue;
				const key = `${x.toFixed(2)},${y.toFixed(2)}`;
				if (!net) {
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
					continue;
				}
				// 有网焊盘不做全局枚举补漏：两个枚举源的位置字段/精度不同，
				// toFixed(2) key 对不齐时会加入**重复焊盘**（case12 实测
				// 73→69）。P02 的 pad 丢失已证伪为采集问题（采集期三 pad
				// 齐全），真凶在布线后的几何改写，由终审兜底。
				if (seenNetted.has(key) || net) continue;
				seenNetted.add(key);
				const padShape2 = Array.isArray(p.pad) ? p.pad : [];
				let w2 = padShape2.length >= 3 ? (padShape2[1] ?? 0) : 0;
				let h2 = padShape2.length >= 3 ? (padShape2[2] ?? 0) : 0;
				const hole2 = p.hole;
				const holeMil2 = Array.isArray(hole2) && hole2.length >= 2 ? (hole2[1] ?? 0) : 0;
				if (w2 === 0 && h2 === 0) { w2 = 32; h2 = 34; }
				const rawLayer2: number = p.layerId ?? p.layer ?? 1;
				let padType2: 'th' | 'smd' | 'via' = 'smd';
				if (holeMil2 > 0 || rawLayer2 === 12) padType2 = 'th';
				else if (p.isVia || p.type === 'via') padType2 = 'via';
				let layer2: number = rawLayer2;
				if (padType2 === 'smd' && (rawLayer2 === 12 || rawLayer2 > 2)) layer2 = 1;
				const info: EnginePad = {
					x, y, net, id: p.globalIndex || p.uuid || `${net}_patch_${allPads.length}`,
					layer: layer2, padType: padType2, holeDiameter: holeMil2,
					radius: Math.max(w2 / 2, h2 / 2, holeMil2 / 2),
					bodyRadius: Math.max(Math.min(w2, h2) / 2, holeMil2 / 2),
					rotation: p.rotation ?? 0,
					index: allPads.length,
				};
				allPads.push(info);
				const arr = padMap.get(net) ?? [];
				arr.push(info);
				padMap.set(net, arr);
				nettedPatched++;
				if ((globalThis as any).__RB_PAD_DEBUG__) {
					this.onLog?.(`[pad-debug] 补漏 ${net} @(${x.toFixed(2)},${y.toFixed(2)}) type=${p.type} primType=${p.pcbItemPrimitiveType} pad=${JSON.stringify(p.pad ?? null)}`, 'info');
				}
			}
			if (netlessAdded) {
				this.onLog?.(`补充 ${netlessAdded} 个无网络焊盘作为障碍（NC 脚/安装孔等，DRC 照查间距）`, 'info');
			}
			if (nettedPatched) {
				this.onLog?.(`焊盘补漏：${nettedPatched} 个有网焊盘被按网络枚举瞬时漏收，已从全局枚举补齐`, 'warn');
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
	private async collectStaticObstacles(drc: DRCRules, tighten = false): Promise<RBObstacle[]> {
		const obstacles: RBObstacle[] = [];
		const parsePath = parsePolygonPath;
		// 拉紧口径：圆角弧严格内切于原折角，比原铜皮更远离外部障碍；用 trackToTrack
		// 正确间距 + 极薄余量，避免布线口径的「trackToLine 下限 8 + 2mil 余量」把
		// 本来 DRC 干净的圆角弧误判为越界（实测 262 尖角只圆出 4 个的根因）。
		const lineClear = tighten ? drc.trackToTrackMil : Math.max(drc.trackToLineMil, 8);
		const copperMargin = tighten ? 0.5 : 2;

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
			const r = lw / 2 + lineClear + drc.traceWidthMil / 2 + copperMargin;
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
			const r = lw / 2 + lineClear + drc.traceWidthMil / 2 + copperMargin;
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
				const r = lw / 2 + lineClear + drc.traceWidthMil / 2 + copperMargin;
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
			await this.cachePadShapes();

			this.onProgress?.({ stage: '解析 PCB', percentage: 8, message: '正在获取 PCB 数据...' });
			const { padMap, allPads } = await this.collectPads();
			if (padMap.size === 0) throw new Error('没有可布线的网络');

			const excluded = new Set(options.excludeNets ?? []);
			// 默认排除地网络（GND/VSS/AGND/DGND…）：地通常是铺铜/手工走线，
			// 自动布线只会把它当围墙并和信号抢通道。排除后既不参与布线，
			// 也不在 clearExistingTraces 里被清理（排除网的既有走线保持现状）。
			if (options.excludeGnd !== false) {
				const groundNames = [...padMap.keys()].filter(isGroundNetName);
				const nonGround = padMap.size - groundNames.length;
				if (groundNames.length && nonGround > 0) {
					for (const n of groundNames) excluded.add(n);
					this.onLog?.(`按设置排除地网络 ${groundNames.length} 个（${groundNames.slice(0, 6).join(', ')}${groundNames.length > 6 ? '…' : ''}）`, 'info');
				} else if (groundNames.length) {
					this.onLog?.('板内网络全部是地网络，已跳过「排除 GND」（否则没有可布网络）', 'warn');
				}
			}
			const networks = Array.from(padMap.entries())
				.filter(([name]) => !excluded.has(name))
				.map(([name, pads]) => ({ name, pads }));
			if (excluded.size) {
				this.onLog?.(`按设置排除 ${Array.from(excluded).filter(n => padMap.has(n)).length} 个网络`, 'info');
			}
			if (networks.length === 0) throw new Error('所有网络都被排除，没有可布线的网络');

			// **整机委托模式**（delegateAll）：清理后直接调内置自动布线器处理
			// 全部待布网络。内置路由器（WASM）在密排板上实测远强于本引擎
			//（case12：内置 120s 除 GND 全通 vs 本引擎 71-73/98）。本引擎继续
			// 负责：GND/排除网口径、既有走线清理、DRC 验证与统计上报。
			if (options.delegateAll === true || options.engine === 'jrouter' || options.engine === 'hrouter') {
				const engineLabel = options.engine === 'hrouter' ? 'HRouter' : 'JRouter';
				this.onProgress?.({ stage: '委托内置路由器', percentage: 20, message: `${engineLabel} 处理 ${networks.length} 个网络...` });
				if (options.clearExistingTraces !== false) {
					try {
						await this.purgeNetsBatch(new Set(networks.map(n => n.name)));
					} catch { /* 清理失败不阻断 */ }
				}
				const t0 = Date.now();
				// 引擎映射：JRouter = COMPLETION(1) 布通率优先；HRouter = FASTER(0) 速度优先
				//（EPCB_AutoRoutingOptimization；BETA API 不接受未文档化的 engine 属性）
				// 拆批委托：BETA API 对大清单不稳定——case12 12 网批实测「4 批全满后
				// 连续两批挂死」（批 5 挂 11min 才吐 undefined）。批级 race 超时 600s
				//（case9 最难 7 网批 445s 仍在界内），超时/undefined 的批**二分细分
				// 重试**（12→6→3，floor 3）：小批已知能成。
				const allNames = networks.map(n => n.name);
				const BATCH = 12;
				const routedAcc = new Set<string>();
					// 违规但保留几何的网（不进修复名单，防价值摧毁）
					const violatingAcc = new Set<string>();
				const failedAcc = new Set<string>();
				const callChunk = (chunk: string[]): Promise<unknown> => {
					const others = allNames.filter(n => !chunk.includes(n));
					return Promise.race([
						eda.pcb_Document.autoRouting({
							RoutingNets: chunk,
							ignoreNets: [...excluded, ...others],
							optimization: options.engine === 'hrouter' ? 0 : 1,
						} as never),
						new Promise<undefined>(r => setTimeout(() => r(undefined), 600_000)),
					]);
				};
				const delegateChunk = async (chunk: string[], depth: number, label: string): Promise<void> => {
					if (this.shouldCancel) return;
					let dr: any = await callChunk(chunk);
					if (!dr || typeof dr !== 'object') {
						if (chunk.length > 3 && depth < 3) {
							this.onLog?.(`${engineLabel} ${label} 超时/未返回，二分为 ${Math.ceil(chunk.length / 2)}+${Math.floor(chunk.length / 2)} 网重试`, 'warn');
							const mid = Math.ceil(chunk.length / 2);
							await delegateChunk(chunk.slice(0, mid), depth + 1, `${label}a`);
							await delegateChunk(chunk.slice(mid), depth + 1, `${label}b`);
							return;
						}
						this.onLog?.(`${engineLabel} ${label} 仍未返回，${chunk.length} 网记为失败`, 'error');
						for (const nm of chunk) failedAcc.add(nm);
						return;
					}
					const failedChunk = new Set<string>(dr.failedNets ?? []);
					for (const nm of chunk) (failedChunk.has(nm) ? failedAcc : routedAcc).add(nm);
					this.onLog?.(
						`${engineLabel} ${label} 完成：成功 ${chunk.length - failedChunk.size}/${chunk.length}（累计 ${Math.round((Date.now() - t0) / 1000)}s）`,
						failedChunk.size < chunk.length ? 'success' : 'warn',
					);
				};
				for (let i = 0; i < allNames.length && !this.shouldCancel; i += BATCH) {
					const chunk = allNames.slice(i, i + BATCH);
					const batchNo = Math.floor(i / BATCH) + 1;
					await delegateChunk(chunk, 0, `批 ${batchNo}/${Math.ceil(allNames.length / BATCH)}`);
					this.onProgress?.({
						stage: '委托内置路由器',
						percentage: 20 + Math.round(70 * Math.min(1, (i + BATCH) / allNames.length)),
						message: `${engineLabel} 已处理 ${Math.min(i + BATCH, allNames.length)}/${allNames.length} 个网络...`,
					});
				}
				const dur = Date.now() - t0;
				this.onLog?.(`${engineLabel} 委托完成：成功 ${routedAcc.size}/${allNames.length}（${Math.round(dur / 1000)}s），未解 ${failedAcc.size}`, routedAcc.size ? 'success' : 'warn');
				// DRC 复核（与引擎路径同口径的上报）
				this.routedNames = routedAcc;
				this.failedNames = failedAcc;
				this.totalNets = networks.length;
				try {
					const drcRes = await eda.pcb_Drc.check(true, false, true);
					const connErrNets = new Set<string>();
					const walk = (node: any): void => {
						if (!node) return;
						if (Array.isArray(node)) { node.forEach(walk); return; }
						if (node.errorType === 'Connection Error' && node.net && this.routedNames.has(node.net)) connErrNets.add(node.net);
						if (node.list) walk(node.list);
					};
					walk(drcRes);
					if (connErrNets.size) {
						this.onLog?.(`DRC 复核：${connErrNets.size} 个「已布」网仍有 Connection Error（内置成功语义宽于全连通）：${[...connErrNets].slice(0, 8).join(', ')}`, 'warn');
						for (const nm of connErrNets) {
							this.routedNames.delete(nm);
							this.failedNames.add(nm);
						}
					}
				} catch { /* DRC 复核失败不阻断 */ }

				// ── 后处理：读回画布几何 → 拉紧 + 圆润（适配三个引擎的产物）→ 回写
				// 决策：先落盘再优化再刷新（内置路由器直接写画布，无插入前钩子）。
				if (!this.shouldCancel && this.routedNames.size) {
					try {
						this.onProgress?.({ stage: '形态优化', percentage: 90, message: '对内置路由结果做拉紧/圆润优化...' });
						const lines = (await eda.pcb_PrimitiveLine.getAll()) ?? [];
						const arcs = (await eda.pcb_PrimitiveArc.getAll()) ?? [];
						const vias = (await eda.pcb_PrimitiveVia.getAll()) ?? [];
						const byNet = new Map<string, { prims: RenderedPrim[]; vias: PlannedVia[] }>();
						for (const l of lines as any[]) {
							const net = String(l.net ?? '');
							if (!this.routedNames.has(net)) continue;
							const e = byNet.get(net) ?? { prims: [], vias: [] };
							e.prims.push({ kind: 'line', net, layer: l.layer ?? l.layerId ?? 1, x1: l.startX ?? l.x, y1: l.startY ?? l.y, x2: l.endX, y2: l.endY });
							byNet.set(net, e);
						}
						for (const a of arcs as any[]) {
							const net = String(a.net ?? '');
							if (!this.routedNames.has(net)) continue;
							const e = byNet.get(net) ?? { prims: [], vias: [] };
							const prim = canvasArcToPrim(a, net);
							if (prim) e.prims.push(prim);
							byNet.set(net, e);
						}
						for (const v of vias as any[]) {
							const net = String(v.net ?? '');
							if (!this.routedNames.has(net)) continue;
							const e = byNet.get(net) ?? { prims: [], vias: [] };
							e.vias.push({ net, x: v.x, y: v.y, dia: v.viaDiameter ?? 24, hole: v.holeDiameter ?? 12 });
							byNet.set(net, e);
						}
						const entries = [...byNet.entries()].map(([name, e]) => ({ name, ...e }));
						// 后处理管线（引擎公开入口）：摘本网旧几何 → 可视图逐边重解
						//（真拉紧）→ 更短才采纳 → 圆润。allPads 传真实焊盘（MST/保护集），
						// baseWorld 传全画布静态世界（邻网/排除网焊盘走线铺铜板框——
						// 缺了它重解路径会合法地压上这些铜，DRC 违规暴涨）。
						const staticWorld = await this.collectStaticObstacles(drc);
						const eng = new PlanningEngine();
						const optimized = await eng.postProcessImportedNets(entries, drc, networks.flatMap(n => n.pads), staticWorld, (m, l) => this.onLog?.(m, l));
						// 回写：整批清掉原几何，逐网写优化结果
						await this.purgeManagedNets(new Set(entries.map(e => e.name)));
						this.netPrimIds.clear();
						for (const e of optimized) {
							if (!e.prims.length) continue;
							await this.writeNet(e.name, e.prims, e.vias, true);
						}
						const changed = optimized.filter(e => e.changed).length;
						this.onLog?.(`形态优化完成：${optimized.length} 网（${changed} 网有缩短）`, 'info');

						// ── DRC 违规归属降级（诚实口径）：内置引擎产物可能带间距违规
						//（case9 拆批实测 ~70 处 SMD Pad to Track / Track to Via 等）。
						// errData.position 单位是 10mil（×10 转画布 mil），obj1/obj2 的
						// net 常缺失 → 按违规点到已布网图元的最近距离（≤30mil）归属。
						// 违规网从「已布」降级为「失败」，后续可做选择性混合修复。
						try {
							const drcRes2 = await eda.pcb_Drc.check(true, false, true);
							const viol: Array<{ x: number; y: number; kind: string }> = [];
							const walkV = (node: any): void => {
								if (!node) return;
								if (Array.isArray(node)) { node.forEach(walkV); return; }
								// 连接类错误（errorType=Connection Error，未连接焊盘的
								// errorObjType 是 "SMD Pad"/"TH Pad" 且无 minDistance）
								// 不是走线间距违规——c12v143 实测误归属把 77 网降到 20。
								if (node.errorType === 'Connection Error') return;
								if (node.errorObjType && node.errorObjType !== 'Connection Error' && node.errorObjType !== 'Board Outline to SMD Pad') {
									const ed = node.explanation?.errData ?? {};
									// 真间距违规必带 minDistance/clearance；缺的是连接/其它类
									if (!Number.isFinite(ed.minDistance) && !Number.isFinite(ed.clearance)) return;
									const p = ed.position;
									if (p && Number.isFinite(p.x) && Number.isFinite(p.y)) viol.push({ x: p.x * 10, y: p.y * 10, kind: String(node.errorObjType) });
									return;
								}
								if (node.list) walkV(node.list);
							};
							walkV(drcRes2);
							if (viol.length) {
								const segDist = (px: number, py: number, x1: number, y1: number, x2: number, y2: number): number => {
									const dx = x2 - x1, dy = y2 - y1;
									const L2 = dx * dx + dy * dy;
									const t = L2 ? Math.max(0, Math.min(1, ((px - x1) * dx + (py - y1) * dy) / L2)) : 0;
									return Math.hypot(px - (x1 + t * dx), py - (y1 + t * dy));
								};
								const bad = new Map<string, number>();
								for (const v of viol) {
									let bestNet = '', bestD = 30;
									for (const e of optimized) {
										for (const pr of e.prims) {
											if (pr.kind === 'line') {
												const d2 = segDist(v.x, v.y, pr.x1 ?? 0, pr.y1 ?? 0, pr.x2 ?? pr.x1 ?? 0, pr.y2 ?? pr.y1 ?? 0);
												if (d2 < bestD) { bestD = d2; bestNet = e.name; }
											}
										}
										for (const pv of e.vias) {
											const d2 = Math.hypot(v.x - pv.x, v.y - pv.y);
											if (d2 < bestD) { bestD = d2; bestNet = e.name; }
										}
									}
									if (bestNet && this.routedNames.has(bestNet)) bad.set(bestNet, (bad.get(bestNet) ?? 0) + 1);
								}
								if (bad.size) {
									for (const nm of bad.keys()) {
										this.routedNames.delete(nm);
										this.failedNames.add(nm);
										// 违规网的几何保留在画布上（连通但 DRC 脏）——
										// c12v142 实测把它们拆给修复引擎重布、预算不足时
										// 64 网悬空（80→34 的价值摧毁）。修复名单只收
										// 无几何的网，违规网通过 excludeNets 保护。
										violatingAcc.add(nm);
									}
									this.onLog?.(`DRC 违规归属：${bad.size} 网带间距违规已降级为失败（${[...bad].slice(0, 6).map(([n, c]) => `${n}×${c}`).join(', ')}）`, 'warn');
								}
							}
						} catch { /* 违规归属失败不阻断 */ }
					} catch (err) {
						this.onLog?.('后处理失败（保留内置路由原几何）：' + (err instanceof Error ? err.message : String(err)), 'warn');
					}
				}

				// ── 混合修复：J/H 全部成功 → 收工；有失败/违规网 → 受限递归一次
				// RRouter（成功网进 excludeNets：不再布、其走线留画布当静态障碍，
				// 修复网绕行）。复用整条主管线，避免在本块内复制规划/写入逻辑。
				if (!this.failedNames.size || this.shouldCancel || this.hybridRepairDepth > 0) {
					this.emitStats();
					const stats = this.snapshotStats();
					this.onProgress?.({ stage: '完成', percentage: 100, message: '委托布线完成' });
					return {
						...stats,
						cancelled: this.shouldCancel,
						failed: [...this.failedNames],
						logs: [],
					} as never;
				}
				const jhRouted = routedAcc.size;
				const jhFailed = [...this.failedNames];
				// 修复名单 = 无几何的网（未解 + 连通降级）；违规网几何保留，
				// 通过 excludeNets 挡在修复运行外（其走线照旧当静态障碍）
				const repairCount = jhFailed.filter(n => !violatingAcc.has(n)).length;
				this.onLog?.(`混合修复：${repairCount} 网无几何转 RRouter 重布；${violatingAcc.size} 网违规保留几何（DRC 脏但连通）`, 'info');
				// 子 route() 开头会按 netPrimIds/touchedNets 清「上一轮走线」——
				// 先清空账本让它跳过，J/H 成功走线留在画布当修复网的静态障碍
				//（排除网的走线 collectStaticObstacles 照收，README 语义）。
				this.netPrimIds.clear();
				this.touchedNets.clear();
				// 子 route() 的 isRouting 守卫会拦住递归（外层还挂着 true）——
				// 递归是本运行的最后动作，临时放开，外层 finally 再兜底复位。
				this.isRouting = false;
				this.hybridRepairDepth++;
				try {
					const sub = await this.route({
						...options,
						engine: 'rrouter',
						hybridDelegate: false,   // 修复阶段不再回委托 BETA API（刚被它坑过）
						excludeNets: [...excluded, ...routedAcc, ...violatingAcc],
					});
					return {
						...sub,
						totalNets: allNames.length,
						routedNets: jhRouted + sub.routedNets,
						failedNets: sub.failedNets,
						elapsedMs: (sub.elapsedMs ?? 0) + (Date.now() - t0),
					} as never;
				} finally {
					this.hybridRepairDepth--;
				}
			}

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
				// 混合委托模式：引擎只拿总预算的 40%（且至少留 240s 给委托阶段）
				budgetMs: options.hybridDelegate
					? Math.max(60_000, Math.min(options.budgetMs * 0.4, options.budgetMs - 240_000))
					: options.budgetMs,
				copperLayers: bottomEnabled ? detectedLayers : 1,
				options: {
					// 首轮只求连通；连通完成后再做中心到中心最短路拉紧优化
					connectivityFirst: true,
					optimize: options.finalOptimize !== false,
					allowViaFallback: options.viaFallback !== false,
					allowViaInPad: false,
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
			// 停止语义：不做任何画布删改（失败网残留的半截走线保留 —— 用户
			// 要的是「保留画布内容」；下次布线会按 clearExistingTraces 自动清理）
			if (!this.shouldCancel) {
				for (const name of result.failed) {
					if (partial.has(name)) continue;
					this.netGeom.delete(name);
					if (options.livePreview === false) {
						this.pendingPlanned.delete(name);
						continue;
					}
					await this.deleteNet(name);
				}
			}

			// 收尾对账：EDA 的批量删除偶发不生效，运行中途被替换掉的尝试走线
			// 可能残留在画布上（与最终走线叠画、引发 DRC 冲突）。规划结束后
			// 文档已空闲，按**引擎终态**（result）逐网清写一遍——不能用消息
			// 历史（pendingPlanned）：引擎结尾还有终局修复（拆除违规过孔等），
			// 消息历史停在修复之前，画布会残留已被引擎丢弃的图元（实测过孔
			// 间距违规在画布上复现、引擎侧却已清除）。
			// **预览关闭（NOPREVIEW）同样必须走对账**：该模式只有 flushPending
			// 这一次裸写（deleteNet 无 ID 记录不删任何东西，逐图元 create 偶发
			// 失败只 console.warn），没有任何核对 —— 实测 VDD3V3 画布只剩 8 条线
			// + 9 过孔（引擎终态 96 prims），7 个孤立过孔被 EDA 判 Connection
			// Error。引擎终态权威，两种模式同一对账路径。
			if (!this.shouldCancel) {
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
			// 「拆线重试」假象，2026-10-24 实测）。停止路径相反：丢弃全部排队
			// 几何立即返回（几十 ms），画布停留在停止瞬间的样子。
			if (this.shouldCancel) {
				this.pendingPlanned.clear();
				this.applyPending.clear();
			} else {
				await this.flushPending();
			}

			// 兜底核对：画布必须与引擎终态完全一致。实测 EDA 的按 ID 删除
			// 偶发漏删、重复写入会留下同坐标幽灵过孔（引擎侧已去重，画布仍在，
			// EDA DRC 照样报 Track to Via）。这里按终态扫描一次，删掉多余的，
			// 数量不符的整网重写。预览关闭同样核对（见上方收尾对账注释）。
			if (!this.shouldCancel) {
				await this.reconcileCanvas(result);
			}

			// 混合模式：引擎失败网委托内置自动布线器（BETA API）。内置路由器
			// 对残留网的处理与画布上已有铜皮天然兼容（existingPrimitiveMode
			// 默认保留），case9 实测 7 个失败网委托后 Via 错误 9 → 5（约 4 网
			// 被接通）。耗时不可控（case12 的 29 网委托 >590s 未完成），故为
			// 显式选项（hybridDelegate，默认关）。
			if (this.currentOptions?.hybridDelegate && !this.shouldCancel &&
				Array.isArray(result.failed) && result.failed.length) {
				this.onProgress?.({
					stage: '委托内置路由器',
					percentage: 99,
					message: `内置路由器处理 ${result.failed.length} 个残留网络（耗时不可控）...`,
				});
				try {
					const dr = await eda.pcb_Document.autoRouting({ RoutingNets: result.failed });
					if (!dr || typeof dr !== 'object') {
						this.onLog?.(`委托内置路由器返回异常：${JSON.stringify(dr) ?? 'undefined'}（BETA API 静默失败，跳过委托）`, 'warn');
					} else {
						this.onLog?.(
							`委托内置路由器：成功 ${dr.successNetsCount}/${dr.totalNetsCount}（${Math.round((dr.duration || 0) / 1000)}s），未解 ${(dr.failedNets ?? []).length}`,
							dr.successNetsCount ? 'success' : 'warn',
						);
					}
				} catch (err) {
					this.onLog?.('委托内置路由器失败：' + (err instanceof Error ? err.message : String(err)), 'error');
				}
			}

			// 以规划结果为准刷新统计
			if (!this.shouldCancel) {
				this.routedNames = new Set(result.routedNets);
				this.failedNames = new Set(result.failed);
				this.emitStats();
			}

			const stats = this.snapshotStats();
			// 诊断出口：E2E/测试台审计引擎终态用（逐网 prims/vias）
			(this as unknown as { lastResult?: PlanningResult }).lastResult = result;
			// RRouter 布通率偏低时建议切换内置引擎（需求：引擎选择引导）
			if (!this.shouldCancel && this.totalNets > 0 && result.failed.length / this.totalNets >= 0.15) {
				this.onLog?.(
					`RRouter 布通率偏低（${this.totalNets - result.failed.length}/${this.totalNets}）：建议在参数弹窗顶部切换 JRouter / HRouter（EDA 内置引擎，密排板布通率更高）`,
					'warn',
				);
			}
			// 终态百分比 = 布通网络数 / 总网络数（有失败网时不再显示 100%）
			const doneNets = result.routedNets?.length ?? 0;
			const allNets = Math.max(1, this.totalNets || doneNets + (result.failed?.length ?? 0));
			const donePct = Math.min(100, Math.floor((doneNets / allNets) * 100));
			this.onProgress?.({
				stage: this.shouldCancel ? '已停止' : '完成',
				percentage: this.shouldCancel ? Math.min(99, donePct) : donePct,
				message: this.shouldCancel
					? `布线已停止：${doneNets}/${allNets} 网布通`
					: `布线完成：${doneNets}/${allNets} 网布通`,
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
				// 停止路径不等写入队列排空（用户语义：几十 ms 内结束，画布保留现状）
				if (this.shouldCancel) { fn(); return; }
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
			// 2) 走线/圆弧数量核对：不符的整网重写。
			// 期望数必须按 **mergePrims 之后** 的数量（与 writeNet 的写入口径
			// 一致）：原始数量含共线/重叠段，合并后变少 —— 拿原始数当期望
			// 会把「合并变少」误判成丢图元（每轮白白整网重写 3 遍），反而把
			// 真正的静默丢失淹没在噪声里（实测 VDD3V3 丢 88 段没被发现）。
			try {
				const lines = (await eda.pcb_PrimitiveLine.getAll()) ?? [];
				const arcs = (await eda.pcb_PrimitiveArc.getAll()) ?? [];
				const count = new Map<string, number>();
				for (const l of lines as any[]) { if (managed.has(l.net)) count.set(l.net, (count.get(l.net) ?? 0) + 1); }
				for (const a of arcs as any[]) { if (managed.has(a.net)) count.set(a.net, (count.get(a.net) ?? 0) + 1); }
				const rewrite: string[] = [];
				for (const name of managed) {
					if (rewritten.has(name)) continue;
					const expect = mergePrims(primsOf.get(name) ?? []).length;
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
		let missed = 0;
		// EDA 的 create 偶发静默失败（返回 null 不抛错）：立即重试一次，
		// 仍失败计入 missed —— 终态审计与 reconcileCanvas 靠它发现丢图元
		const retryCreate = async (fn: () => Promise<unknown>): Promise<unknown> => {
			let r = await fn();
			if (!r) {
				missed++;
				await new Promise(res => setTimeout(res, 20));
				r = await fn();
			}
			return r;
		};

		// 先建过孔再建走线：EDA 不会把已存在的走线自动连接到后创建的过孔
		for (const via of vias) {
			try {
				const created = await retryCreate(() => eda.pcb_PrimitiveVia.create(via.net, via.x, via.y, via.hole, via.dia));
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
					const created = await retryCreate(() => eda.pcb_PrimitiveLine.create(name, pr.layer, sx, sy, ex, ey, drcWidth, false));
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
						const createdL = await retryCreate(() => eda.pcb_PrimitiveLine.create(name, pr.layer, sx, sy, ex0, ey0, drcWidth, false));
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
						const created = await retryCreate(() => eda.pcb_PrimitiveArc.create(name, pr.layer, sx, sy, ex, ey, deg, drcWidth));
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
		if (missed) this.onLog?.(`写入 ${name}: ${missed} 个图元创建失败（重试后仍缺，交画布核对重写）`, 'warn');
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
		// 拉紧进行中（isRouting 为 false）：同样走协同取消，已写入部分保留
		if (!this.isRouting) {
			if (this.tightening) {
				this.shouldCancel = true;
				try { this.tightenWorker?.terminate(); } catch { /* ignore */ }
				this.tightenWorker = null;
				this.tightenCancel?.();
				this.tightenCancel = null;
				this.onLog?.('拉紧已停止', 'warn');
			}
			return;
		}
		this.shouldCancel = true;
		// 停止语义（用户拍板）：点击停止 = 立刻杀掉 worker（几十 ms）、画布保留
		// 已写入内容、此后不再有任何写入。旧实现依赖引擎在 checkCancelled
		// 检查点协同退出 —— GND 单网求解/驱逐循环可达分钟级，期间画布还在被
		// 实时节拍继续刷，表现为「点了停止还要等很久且线还在长」。
		this.onLog?.('布线已停止', 'warn');
		this.engine?.cancel();   // 无 Worker 的主线程回退仍走协同取消
		if (this.worker) {
			try { this.worker.terminate(); } catch { /* ignore */ }
			this.worker = null;
		}
		if (this.cancelTimer) {
			clearTimeout(this.cancelTimer);
			this.cancelTimer = null;
		}
		this.stopLiveTimer();          // 停 5s 节拍并丢弃排队几何：之后画布零写入
		this.applyPending.clear();     // 未执行的写入任务全部作废
		this.forceFinish?.();          // 立即兑现规划 Promise（不等写入队列排空）
	}

	isActive(): boolean { return this.isRouting; }
}
