/**
 * PlanningEngine - 纯几何布线规划引擎（零 EDA 依赖）
 *
 * 与 TopologicalRouter 的「EDA 读写」彻底分离，可运行在：
 *  - Web Worker（推荐：不占用 EDA 主线程）
 *  - 主线程（Worker 不可用时的回退路径）
 *
 * 输入输出均为普通对象，可直接 postMessage 序列化。
 */

import {
	RBObstacle, RBPrimitive, capsuleToCircles,
	buildPolylineObstacle,
	pointSegDist, padExitModel, padShapeToCircles, padShapeDistance, fanoutFillet,
} from './rubberband-engine';
import { routeConnection } from './visibility-router';
import { buildExitCandidates, routePadEdge, stubFeasible, PadTerminal } from './edge-router';
import { DRCRules } from './drc-rules';

// ─────────────────────────────────────────────
// 数据类型
// ─────────────────────────────────────────────

/** 布线算法的可调参数 */
export interface PlanOptions {
	/** 出线额外余量（mil）：在焊盘边界之外再外扩的量，0 = 严格按最小距离出线 */
	escapeClearanceMil?: number;
	/** 撕毁重布轮数 */
	ripupRounds?: number;
	/** 电源网络优先 */
	powerNetsFirst?: boolean;
	/** 首轮连通优先（保留兼容：两种阶段现在走同一套中心到中心寻路） */
	connectivityFirst?: boolean;
	/** 最后对已布网络做中心到中心最短路优化（失败则保留原走线） */
	optimize?: boolean;
	/** 兼容旧参数：新算法始终使用「焊盘中心 → 最短路 → 焊盘中心」 */
	directConnect?: boolean;
	/** 网络排序：auto（专业自动规划）/ river / reverse / max-x / longest / shortest */
	netOrder?: 'auto' | 'river' | 'reverse' | 'max-x' | 'longest' | 'shortest';
	/** 失败网络允许过孔换层（底层布线）兜底 */
	allowViaFallback?: boolean;
	/** 允许过孔落在焊盘铜皮上（默认 false：过孔铜环必须避开所有焊盘） */
	allowViaInPad?: boolean;
}

export interface EnginePad {
	x: number;
	y: number;
	net: string;
	id: string;
	layer: number;       // 1 / 2 / 12（12 = 通孔多层）
	padType: 'th' | 'smd' | 'via';
	holeDiameter: number;
	/** 外接半径（用于焊盘障碍与兜底逃逸） */
	radius: number;
	/** 短轴半径（通用逃逸评分更合适，避免长焊盘逃逸点被推得过远） */
	bodyRadius?: number;
	/** 焊盘旋转角（度，EDA 约定逆时针为正） */
	rotation?: number;
	index: number;
}

export interface RenderedPrim {
	kind: 'line' | 'arc';
	net: string;
	layer: number;
	// line
	x1?: number; y1?: number; x2?: number; y2?: number;
	// arc
	cx?: number; cy?: number; r?: number; startAngle?: number; sweep?: number;
}

export interface PlannedVia {
	net: string;
	x: number;
	y: number;
	dia: number;
	hole: number;
}

export interface PadShape {
	pad: any[];
	hole: any[] | null;
	rotation: number;
}

export interface PlanningInput {
	drc: DRCRules;
	allPads: EnginePad[];
	networks: Array<{ name: string; pads: EnginePad[] }>;
	staticObstacles: RBObstacle[];
	/** 焊盘形状缓存（键 = "x,y" 保留两位小数） */
	padShapes: Record<string, PadShape>;
	/** 整轮规划时间预算（毫秒，0 = 不限制） */
	budgetMs?: number;
	/** 板子的铜层数（>=2 才有底层可供换层走线；缺省按双面板处理） */
	copperLayers?: number;
	/** 算法可调参数 */
	options?: PlanOptions;
}

/** 网络焊盘级飞线进度：total = 该网络参与布线的焊盘数，routed = 已消除全部飞线的焊盘数 */
export interface NetPadProgress {
	total: number;
	routed: number;
}

export interface PlanningCallbacks {
	onLog?: (message: string, level: string) => void;
	onProgress?: (percentage: number, message: string) => void;
	/** 某个网络完成（或重新完成）规划：主线程据此实时写入画布 */
	onNetUpdate?: (name: string, prims: RenderedPrim[], vias: PlannedVia[], ok: boolean, pads?: NetPadProgress) => void;
	/** 撕毁网络：主线程需删除该网络已写入的图元 */
	onRipup?: (names: string[]) => void;
	isCancelled?: () => boolean;
}

export interface PlannedNetResult {
	name: string;
	prims: RenderedPrim[];
}

export interface PlanningResult {
	nets: PlannedNetResult[];
	vias: PlannedVia[];
	failed: string[];
	/** 部分连通：有有效走线但仍有 MST 连接失败的网（保留在画布上） */
	partial: string[];
	/** 已布网络名（按布线顺序） */
	routedNets: string[];
	/** 规划日志（Worker 模式下由主线程读取） */
	logs: Array<{ level: string; message: string }>;
}

const CLR_EPS = 0.05;

/**
 * 地网络识别：GND / VSS 及其常见前缀变体（AGND、DGND、PGND、SGND、
 * GND_1、SIGNAL_GND、CHASSIS_GND ...），以及 0V / 地 / EARTH。
 * 地网络要最后布，识别错一个都可能把后布网络封死。
 */
export function isGroundNetName(name: string): boolean {
	const upper = name.trim().toUpperCase();
	if (!upper) return false;
	if (upper === '0V' || upper === '地' || upper === 'EARTH') return true;
	return /(^|[_\-. /])(A|D|P|S|CHASSIS)?(GND|VSS)([_\-. 0-9]|$)/.test(upper);
}

/** 电源网络（不含地）：VCC/VDD/VBAT/VBUS/VIN/VOUT/VREF/PWR、+3V3、3V3、-5V 等 */
export function isPowerNetName(name: string): boolean {
	if (isGroundNetName(name)) return false;
	const upper = name.trim().toUpperCase();
	if (!upper) return false;
	return /^(VCC|VDD|VBAT|VBUS|VIN|VOUT|VREF|PWR|VSYS)([_\-. 0-9]|$)/.test(upper) ||
		/^[+-]\d/.test(upper) || /^\d+V\d*$/.test(upper);
}

function getRoutingLayer(pad1: EnginePad, pad2: EnginePad): number {
	const l1 = pad1.layer, l2 = pad2.layer;
	if (l1 === 12 && l2 === 12) return 1;
	if (l1 === 12) return l2 === 2 ? 2 : 1;
	if (l2 === 12) return l1 === 2 ? 2 : 1;
	return l1;
}

/** 该焊盘能否从底层接入（金属化孔或底层贴片） */
function bottomCapable(pad: EnginePad): boolean {
	return pad.layer === 12 || pad.layer === 2;
}

/** 合并同圆相邻弧与共线相邻线段：减少接缝（编辑器坐标取整会造成 0.1mil 级断点） */
export function mergePrims(prims: RenderedPrim[]): RenderedPrim[] {
	const out: RenderedPrim[] = [];
	for (const p of prims) {
		// 微段（同一障碍圆上相邻两段弧之间 ~1mil 的切点残留）：直接丢弃，
		// 让两侧的弧有机会按同圆合并
		if (p.kind === 'line' && Math.hypot((p.x2 ?? 0) - (p.x1 ?? 0), (p.y2 ?? 0) - (p.y1 ?? 0)) < 2) continue;
		const last = out[out.length - 1];
		if (p.kind === 'arc' && last && last.kind === 'arc' && last.layer === p.layer &&
			Math.abs((last.cx ?? 0) - (p.cx ?? 0)) < 0.01 &&
			Math.abs((last.cy ?? 0) - (p.cy ?? 0)) < 0.01 &&
			Math.abs((last.r ?? 0) - (p.r ?? 0)) < 0.01) {
			const lastEnd = (last.startAngle ?? 0) + (last.sweep ?? 0);
			let diff = Math.abs(lastEnd - (p.startAngle ?? 0)) % (Math.PI * 2);
			if (diff > Math.PI) diff = Math.PI * 2 - diff;
			if (diff < 0.03) {
				out[out.length - 1] = { ...last, sweep: (last.sweep ?? 0) + (p.sweep ?? 0) };
				continue;
			}
		}
		if (p.kind === 'line' && last && last.kind === 'line' && last.layer === p.layer) {
			const sameEnd = Math.hypot((last.x2 ?? 0) - (p.x1 ?? 0), (last.y2 ?? 0) - (p.y1 ?? 0)) < 0.3;
			if (sameEnd) {
				const ax = (p.x2 ?? 0) - (p.x1 ?? 0), ay = (p.y2 ?? 0) - (p.y1 ?? 0);
				const bx = (last.x2 ?? 0) - (last.x1 ?? 0), by = (last.y2 ?? 0) - (last.y1 ?? 0);
				const cross = ax * by - ay * bx;
				const dot = ax * bx + ay * by;
				const la = Math.hypot(ax, ay), lb = Math.hypot(bx, by);
				if (la > 1e-6 && lb > 1e-6 && Math.abs(cross) / (la * lb) < 0.01 && dot > 0) {
					out[out.length - 1] = { ...last, x2: p.x2, y2: p.y2 };
					continue;
				}
			}
		}
		out.push(p);
	}
	return out;
}

/** RenderedPrim 总长（mil） */
export function renderedLength(prims: RenderedPrim[]): number {
	let total = 0;
	for (const p of prims) {
		if (p.kind === 'line') {
			total += Math.hypot((p.x2 ?? 0) - (p.x1 ?? 0), (p.y2 ?? 0) - (p.y1 ?? 0));
		} else {
			total += Math.abs(p.sweep ?? 0) * (p.r ?? 0);
		}
	}
	return total;
}

// ─────────────────────────────────────────────
// 规划引擎
// ─────────────────────────────────────────────

export class PlanningEngine {
	private drc!: DRCRules;
	private allPads: EnginePad[] = [];
	private staticObstacles: RBObstacle[] = [];
	private padShapes: Record<string, PadShape> = {};
	private callbacks: PlanningCallbacks = {};

	/** 已布走线的动态障碍 */
	private dynamicObstacles: RBObstacle[] = [];
	private obstacleIdCounter = 0;
	/** 焊盘障碍缓存（键 = 排除网络名；焊盘在规划期间不变） */
	private padObstacleCache = new Map<string, RBObstacle[]>();
	/** 并排焊盘行信息（键 = 焊盘 id）：排向单位向量 + 中位间距 + 排内深度（0 端点 → 1 中心） */
	private rowInfo = new Map<string, { dx: number; dy: number; gap: number; depth: number }>();
	/** 全盘焊盘空间网格（加速过孔落点的近邻净距查询） */
	private padGrid = new Map<string, EnginePad[]>();
	private readonly PAD_GRID_CELL = 64;
	/** 每网络的规划图元 */
	private netRendered = new Map<string, RenderedPrim[]>();
	/** 每个网络已完成的 MST 连接（键 = 两个焊盘 id），用于部分连通后继续重试 */
	private routedEdges = new Map<string, Set<string>>();
	/** 已布网络顺序 */
	private routedOrder: string[] = [];
	/** 规划的过孔 */
	private plannedVias: PlannedVia[] = [];
	/** 过孔槽位驱逐已试组合（「失败网|被拆网」），防乒乓 */
	private evictionTried = new Set<string>();
	/** 过孔槽位驱逐总尝试次数上限（每次尝试 = 拆网 + 两次重布，代价大） */
	private evictionAttempts = 0;
	/** 当前规划截止时间（驱逐等尾段逻辑也需要感知预算） */
	private planDeadline = Number.POSITIVE_INFINITY;
	private logs: Array<{ level: string; message: string }> = [];
	private cancelled = false;
	private planOptions: PlanOptions = {};
	/** 板子铜层数（1 = 单面板，没有底层可用） */
	private copperLayers = 2;

	/**
	 * 出线额外余量（mil）：出线候选点在焊盘边界之外的额外外扩量。
	 * 0 = 严格按「最短距离」出线（默认值），正值会让走线略微变长但更宽松。
	 */
	private get escapeMargin(): number {
		const v = this.planOptions.escapeClearanceMil;
		return typeof v === 'number' && v > 0 ? v : 0;
	}

	/** 是否有底层可供换层走线（单面板时不能往 layer 2 出图） */
	private get hasBottomLayer(): boolean {
		return this.copperLayers >= 2;
	}

	/** 是否允许过孔落在焊盘铜皮上（默认关闭） */
	private get allowViaInPad(): boolean {
		return this.planOptions.allowViaInPad === true;
	}

	/** 执行完整规划：排序 → 逐网 → 撕毁重布 → 成组重布 */
	async plan(input: PlanningInput, callbacks: PlanningCallbacks = {}): Promise<PlanningResult> {
		this.drc = input.drc;
		this.allPads = input.allPads;
		this.staticObstacles = this.triageBoardObstacles(input.staticObstacles);
		this.padShapes = input.padShapes || {};
		// 焊盘空间网格：过孔落点校验每次要过「全部焊盘」的外接圆预筛，密集板
		//（600+ 焊盘）× 网格/射线数千候选点是纯浪费 —— 只查 3×3 邻域格
		//（64mil 格 × 3 = 192mil 覆盖窗，远大于 need0 + 最大焊盘外接半径）
		this.padGrid.clear();
		for (const p of this.allPads) {
			const k = `${Math.floor(p.x / this.PAD_GRID_CELL)},${Math.floor(p.y / this.PAD_GRID_CELL)}`;
			const b = this.padGrid.get(k);
			if (b) b.push(p); else this.padGrid.set(k, [p]);
		}
		this.padObstacleCache.clear();
		this.rowInfo.clear();
		this.callbacks = callbacks;
		this.planOptions = input.options ?? {};
		this.copperLayers = Math.max(1, Math.round(input.copperLayers ?? 2));
		this.dynamicObstacles = [];
		this.obstacleIdCounter = 0;
		this.evictionTried.clear();
		this.evictionAttempts = 0;
		this.planDeadline = Number.POSITIVE_INFINITY;
		this.netRendered.clear();
		this.routedEdges.clear();
		this.routedOrder = [];
		this.plannedVias = [];
		this.logs = [];
		this.cancelled = false;

		const budgetMs = input.budgetMs ?? 0;
		const deadline = budgetMs > 0 ? Date.now() + budgetMs : Number.POSITIVE_INFINITY;
		this.planDeadline = deadline;

		// 网络排序（auto = 专业自动规划；地网络永远最后）
		const networks = this.sortNetworks(input.networks);

		this.log(`网络顺序（${this.planOptions.netOrder ?? 'auto'}）：${networks.map(n => n.name).join(' → ')}`, 'info');
		const pendingNets: Array<{ name: string; pads: EnginePad[] }> = [];

		// 第一轮：逐网络布线
		for (let ni = 0; ni < networks.length; ni++) {
			if (this.checkCancelled()) break;
			if (Date.now() > deadline) {
				this.log('规划时间预算用尽，跳过剩余网络', 'warn');
				for (let k = ni; k < networks.length; k++) pendingNets.push(networks[k]);
				break;
			}
			const net = networks[ni];
			const pct = 20 + Math.floor((ni / networks.length) * 55);
			this.progress(pct, `布线网络 ${ni + 1}/${networks.length}: ${net.name}`);
			// 第一遍·线段级快扫：先把「一布就通」的边全部收割（不追求单网
			// 完整再下一网）；有剩余边的网进入撕毁重布/兜底轮做全链收尾
			const ok = await this.routeNetEdges(net, 'quick');
			if (!ok) pendingNets.push(net);
		}

		// 多轮撕毁重布
		const ripupRounds = Math.max(0, Math.round(this.planOptions.ripupRounds ?? 3));
		for (let pass = 0; pass < ripupRounds && pendingNets.length; pass++) {
			if (this.checkCancelled()) break;
			// 快者先走：剩余未完成边少的网排前 —— 易网在每轮头部快速收割布通，
			// 难网（往往要靠成组重布/兜底）只消耗尾部预算，避免预算耗尽时
			// 一批「其实一布就通」的网还压在队尾没轮到
			pendingNets.sort((a, b) => this.netPendingEdges(a) - this.netPendingEdges(b));
			if (Date.now() > deadline) {
				this.log('规划时间预算用尽，停止撕毁重布', 'warn');
				break;
			}
			const next: typeof pendingNets = [];
			for (let pi = 0; pi < pendingNets.length; pi++) {
				const net = pendingNets[pi];
				if (this.checkCancelled()) break;
				// 前一轮动作里已布通（含底层兜底救回）的网络不再重试
				if (this.routedOrder.includes(net.name)) continue;
				// 预算见底就停：单网撕毁重布 = 1+2N 次寻路，分钟级；只在轮头检查
				// 会让一轮内部顶穿预算几十分钟（实测某板超预算 55 分钟）
				if (Date.now() > deadline) {
					this.log('规划时间预算用尽，剩余网络不再重试', 'warn');
					for (let k = pi; k < pendingNets.length; k++) next.push(pendingNets[k]);
					break;
				}
				const ends = [net.pads[0], net.pads[net.pads.length - 1]];
				const seals = new Set<string>();
				for (const o of this.dynamicObstacles) {
					if (o.soft) continue;
					if (o.net && o.net !== net.name &&
						ends.some(e => Math.hypot(o.x - e.x, o.y - e.y) < o.r + 70)) {
						seals.add(o.net);
					}
				}
				const rip = this.routedOrder.filter(n => seals.has(n)).slice(-3);
				if (!rip.length) {
					// 快扫遗留：未被封口的网补一次完整链尝试（快扫只试直连，
					// 这些网还没享受过边界候选退避/换层链）；仍失败才顺延
					const rok = await this.routeNetEdges(net);
					if (!rok) next.push(net);
					continue;
				}
				this.log(`第 ${pass + 1} 轮：网络 ${net.name} 撕毁 ${rip.join(', ')} 重试`, 'warn');
				// 快照被撕网络的原走线：本动作绝不净损失已布网络——
				// 任一被撕网重布失败即整体回滚到动作前的已知好状态
				//（被撕网的新走线可能与回滚网冲突，必须一并撤销）。
				const savedOrder = [...this.routedOrder];
				const saved = new Map<string, { prims: RenderedPrim[]; vias: PlannedVia[] }>();
				for (const n of rip) {
					saved.set(n, {
						prims: [...(this.netRendered.get(n) ?? [])],
						vias: this.plannedVias.filter(v => v.net === n),
					});
				}
				for (const n of rip) this.ripupNet(n);
				// 失败网自身可能有部分走线（保留有效走线）：重布会在其上继续补线，
				// 回滚时按此快照恢复，避免连原有铜皮一起丢掉
				const savedPending = {
					prims: [...(this.netRendered.get(net.name) ?? [])],
					vias: this.plannedVias.filter(v => v.net === net.name),
					doneEdges: new Set(this.routedEdges.get(net.name) ?? []),
				};
				const ok = await this.routeNetEdges(net);
				let sealFailed = false;
				for (let i = rip.length - 1; i >= 0; i--) {
					const ripped = networks.find(nn => nn.name === rip[i])!;
					const rok = await this.routeNetEdges(ripped);
					if (!rok) { next.push(ripped); sealFailed = true; }
				}
				if (sealFailed) {
					// 有被撕网重布失败：两焊盘墙先试底层兜底（失败网的新走线往往已
					// 封死它原来的顶层通道）；全部救回则接受当前局面，否则整体回滚
					let unrecoverable = false;
					for (let i = rip.length - 1; i >= 0; i--) {
						const n = rip[i];
						// 「残留部分走线」不算救回：完整布通的网络被拆成半成品就是净损失
						//（实测 F407 隔离板四个 3-pad 墙全以 1/2 残留被误判「已救回」而接受，
						// 布通率 8 → 7）。只有 routedOrder 里真正布通（重布或两焊盘兜底）才算。
						if (this.routedOrder.includes(n)) continue;
						const wn = networks.find(nn => nn.name === n)!;
						if (wn.pads.length === 2 && await this.tryViaFallback(wn)) continue;
						unrecoverable = true;
					}
					if (!unrecoverable) {
						if (!ok) next.push(net);
						continue;
					}
				} else {
					if (!ok) next.push(net);
					continue;
				}
				// 整体回滚：pending 也没布通、且有被撕网重布失败 → 恢复整组原走线
				this.ripupNetInternal(net.name, true);
				if (savedPending.prims.length) {
					this.netRendered.set(net.name, [...savedPending.prims]);
					this.addRenderedToObstacles(mergePrims(savedPending.prims));
				}
				this.routedEdges.set(net.name, new Set(savedPending.doneEdges));
				this.plannedVias = this.plannedVias.filter(v => v.net !== net.name).concat(savedPending.vias);
				for (const n of rip) this.ripupNetInternal(n, true);
				for (const n of rip) {
					const sv = saved.get(n)!;
					if (sv.prims.length) {
						this.netRendered.set(n, [...sv.prims]);
						this.addRenderedToObstacles(mergePrims(sv.prims));
					}
					this.plannedVias = this.plannedVias.filter(v => v.net !== n).concat(sv.vias);
					const nn = networks.find(x => x.name === n);
					if (nn) this.markAllEdgesDone(nn);
					const idx = savedOrder.indexOf(n);
					if (idx >= 0) this.routedOrder.splice(Math.min(idx, this.routedOrder.length), 0, n);
				}
				for (const v of this.plannedVias) {
					if (!rip.includes(v.net) && v.net !== net.name) continue;
					this.dynamicObstacles.push({
						id: `via${this.obstacleIdCounter++}`, kind: 'circle',
						x: v.x, y: v.y, r: v.dia / 2 + Math.max(this.drc.trackToTrackMil, this.drc.trackToPadMil) + this.drc.traceWidthMil / 2 + 0.1,
						net: v.net, layer: undefined, group: `via:${v.net}:${v.x},${v.y}`, noPrune: true,
					});
				}
				for (const n of rip) {
					const sv = saved.get(n)!;
					const nn = networks.find(x => x.name === n);
					this.callbacks.onNetUpdate?.(
						n, mergePrims(sv.prims), sv.vias, sv.prims.length > 0,
						nn ? this.padProgress(nn) : undefined,
					);
				}
				this.callbacks.onNetUpdate?.(
					net.name, mergePrims(savedPending.prims), savedPending.vias, savedPending.prims.length > 0,
					this.padProgress(net),
				);
				next.push(net);
			}
			pendingNets.length = 0;
			pendingNets.push(...next);
		}

		// 最终成组重布：以失败网的「焊盘中心连线」为走廊，撕掉走廊上其它网络的
		// 走线/过孔（端点附近探测抓不到中段阻挡 —— 例如扇出排里外层网络的斜线
		// 正好切过内层网络的必经通道），失败网按内向优先重布，再重布被拆网
		// （它们会自然向外嵌套，恢复平面序）。
		if (pendingNets.length && Date.now() <= deadline) {
			const sealsAll = new Set<string>();
			const corridor = this.drc.trackToTrackMil + this.drc.traceWidthMil + 30;
			for (const net of pendingNets) {
				for (const [ia, ib] of this.computeMST(net.pads)) {
					const pa = net.pads[ia], pb = net.pads[ib];
					for (const o of this.dynamicObstacles) {
						if (o.soft) continue;
						if (!o.net || o.net === net.name) continue;
						if (pointSegDist(o.x, o.y, pa.x, pa.y, pb.x, pb.y) < o.r + corridor) {
							sealsAll.add(o.net);
						}
					}
				}
			}
			for (const n of [...sealsAll]) if (pendingNets.some(pn => pn.name === n)) sealsAll.delete(n);
			const rip = this.routedOrder.filter(n => sealsAll.has(n)).slice(0, 8);
			if (rip.length) {
				this.log(`成组重布：拆除 ${rip.join(', ')}`, 'warn');
				// 快照整组：任一被拆网重布失败即整体回滚（不净损失已布网络）。
				// 被拆网的原走线与失败网的新走线可能重叠，无法只回滚一部分。
				const savedOrder = [...this.routedOrder];
				const saved = new Map<string, { prims: RenderedPrim[]; vias: PlannedVia[] }>();
				for (const n of rip) {
					saved.set(n, {
						prims: [...(this.netRendered.get(n) ?? [])],
						vias: this.plannedVias.filter(v => v.net === n),
					});
				}
				for (const n of rip) this.ripupNet(n);
				const ordered = [...pendingNets].sort((na, nb) => {
					const xa = Math.max(...na.pads.map(p => p.x));
					const xb = Math.max(...nb.pads.map(p => p.x));
					return xa - xb;   // 内向优先：内侧网络先占天然走廊，外侧网络向外嵌套
				});
				// 失败网多为部分走线状态（保留有效走线）：整体回滚时按此快照恢复，
				// 否则它们本次的新走线连同原有部分走线会被一起清空
				const savedPending = new Map<string, { prims: RenderedPrim[]; vias: PlannedVia[]; doneEdges: Set<string> }>();
				for (const net of ordered) {
					savedPending.set(net.name, {
						prims: [...(this.netRendered.get(net.name) ?? [])],
						vias: this.plannedVias.filter(v => v.net === net.name),
						doneEdges: new Set(this.routedEdges.get(net.name) ?? []),
					});
				}
				for (const net of ordered) {
					if (!(await this.routeNetEdges(net))) this.log(`成组重布：${net.name} 仍失败`, 'error');
				}
				const failedWalls: string[] = [];
				for (const n of rip) {
					const ripped = networks.find(nn => nn.name === n)!;
					if (!(await this.routeNetEdges(ripped))) { this.log(`成组重布：重布 ${n} 失败`, 'error'); failedWalls.push(n); }
				}
				// 被拆网重布失败：两焊盘墙先试底层兜底（失败网的新走廊通常已封死
				// 它原来的顶层通道）；全部救回就接受当前局面（失败网保持布通），
				// 否则整体回滚 —— 恢复被拆网原走线，失败网恢复到动作前的部分状态
				// 「残留部分走线」不算救回：完整网被拆成半成品就是净损失，
				// 只有 routedOrder 里真正布通（重布或两焊盘兜底）才接受新布局
				let unrecoverable = false;
				for (const n of failedWalls) {
					if (this.routedOrder.includes(n)) continue;
					const wn = networks.find(nn => nn.name === n)!;
					if (wn.pads.length === 2 && await this.tryViaFallback(wn)) continue;
					unrecoverable = true;
				}
				if (failedWalls.length && !unrecoverable) {
					this.log(`成组重布：被拆网经底层兜底全部救回，接受新布局（${failedWalls.join(', ')}）`, 'success');
				}
				if (unrecoverable) {
					// 整体回滚：恢复被拆网原走线；失败网恢复到本动作前的部分状态
					//（只清本次新增的图形会留下与新走线重叠的残缺画面，整网撕掉再按快照恢复）
					for (const net of ordered) this.ripupNetInternal(net.name, true);
					for (const n of rip) this.ripupNetInternal(n, true);
					for (const n of rip) {
						const sv = saved.get(n)!;
						if (sv.prims.length) {
							this.netRendered.set(n, [...sv.prims]);
							this.addRenderedToObstacles(mergePrims(sv.prims));
						}
						this.plannedVias = this.plannedVias.filter(v => v.net !== n).concat(sv.vias);
						const nn = networks.find(x => x.name === n);
						if (nn) this.markAllEdgesDone(nn);
					}
					for (const net of ordered) {
						const sp = savedPending.get(net.name)!;
						if (sp.prims.length) {
							this.netRendered.set(net.name, [...sp.prims]);
							this.addRenderedToObstacles(mergePrims(sp.prims));
						}
						this.routedEdges.set(net.name, new Set(sp.doneEdges));
						this.plannedVias = this.plannedVias.filter(v => v.net !== net.name).concat(sp.vias);
					}
					for (const v of this.plannedVias) {
						if (!rip.includes(v.net) && !savedPending.has(v.net)) continue;
						this.dynamicObstacles.push({
							id: `via${this.obstacleIdCounter++}`, kind: 'circle',
							x: v.x, y: v.y, r: v.dia / 2 + Math.max(this.drc.trackToTrackMil, this.drc.trackToPadMil) + this.drc.traceWidthMil / 2 + 0.1,
							net: v.net, layer: undefined, group: `via:${v.net}:${v.x},${v.y}`, noPrune: true,
						});
					}
					this.routedOrder = savedOrder;
					for (const n of rip) {
						const sv = saved.get(n)!;
						const nn = networks.find(x => x.name === n);
						this.callbacks.onNetUpdate?.(
							n, mergePrims(sv.prims), sv.vias, sv.prims.length > 0,
							nn ? this.padProgress(nn) : undefined,
						);
					}
					for (const net of ordered) {
						const sp = savedPending.get(net.name)!;
						this.callbacks.onNetUpdate?.(
							net.name, mergePrims(sp.prims), sp.vias, sp.prims.length > 0,
							this.padProgress(net),
						);
					}
				}
				pendingNets.length = 0;
			}
		}

		// 最后优化阶段：对已布网络重新求一次「中心到中心」最短路径，
		// 成功且更短才替换，失败保留首轮连通走线。
		if (this.planOptions.optimize === true && this.routedOrder.length > 0 &&
			!this.checkCancelled() && Date.now() <= deadline) {
			await this.optimizeRoutes(networks);
		}

		// 冲突消除：被别的网络挡住而绕远的网络，重新安置到更短的通道
		if (!this.checkCancelled() && Date.now() <= deadline) {
			await this.resolveContention(networks);
		}

		// 过孔换层兜底：单层容量不足时，失败网络走底层，两端用逃逸段+过孔连接
		if (this.planOptions.allowViaFallback !== false && !this.checkCancelled() && Date.now() <= deadline) {
			// 部分布通网络先做最后一轮逐边重试：回滚机制保证其有效走线完好，
			// 障碍格局定型后未完成的边可能直接布通（routeNetEdges 跳过已完成边，不会重复叠线）
			const partialNets = networks.filter(n =>
				!this.routedOrder.includes(n.name) &&
				(this.netRendered.get(n.name) ?? []).length > 0);
			for (const net of partialNets) {
				if (this.checkCancelled()) break;
				if (Date.now() > this.planDeadline) break;
				await this.routeNetEdges(net);
			}
			const stillFailed = networks.filter(n =>
				!this.routedOrder.includes(n.name) &&
				// 已部分布通的网络已保留有效走线，逐边重试时已包含过孔候选；
				// 再跑「首尾焊盘」兜底会在已有铜皮上重复叠线。
				(this.netRendered.get(n.name) ?? []).length === 0);
			for (const net of stillFailed) {
				if (this.checkCancelled()) break;
				// 每网兜底都含多次路由器求解，逐网检查预算防止收尾顶穿
				if (Date.now() > this.planDeadline) break;
				// 先试过孔槽位驱逐（落点被单一网络的过孔独占封死时拆网重排），
				// 没有可驱逐对象再走普通过孔兜底
				if (!(await this.tryViaSlotEviction(net, networks))) {
					await this.tryViaFallback(net);
				}
			}
		}

		const failed = networks.filter(n => !this.routedOrder.includes(n.name)).map(n => n.name);
		const partial = networks
			.filter(n => !this.routedOrder.includes(n.name) && (this.netRendered.get(n.name) ?? []).length > 0)
			.map(n => n.name);
		return {
			nets: networks
				.filter(n => (this.netRendered.get(n.name) ?? []).length > 0)
				.map(n => ({
					name: n.name,
					prims: mergePrims(this.netRendered.get(n.name) ?? []),
				})),
			vias: [...this.plannedVias],
			failed,
			partial,
			routedNets: [...this.routedOrder],
			logs: [...this.logs],
		};
	}

	cancel(): void { this.cancelled = true; }

	// ── 内部实现 ─────────────────────────────

	/**
	 * 网络排序：布线顺序直接决定绕路总量与拓扑美观度。
	 *
	 * auto（默认，按 PCB 布局专业规则自动规划）分四层：
	 *  0. 短小两焊盘连接 —— 可动空间最小，先布最稳（如串阻、去耦短线）；
	 *  1. 一般信号 —— 区域内按 river 束状排序，同源扇出逐级向外嵌套；
	 *  2. 电源（非地）—— 走线通常较宽、跨距大，放在信号之后；
	 *  3. 地（GND / VSS / AGND / DGND ...）—— **永远最后布**。地网络焊盘多、
	 *     覆盖全板，先布会像围墙一样把后来的信号全部封死；最后布时其余网络
	 *     已经定型，地网络再沿剩余通道或底层收尾。
	 *
	 * 无论用户选哪种显式顺序，地网络都保持在最后 —— 这是硬约束。
	 * 显式几何顺序仍可指定 river / shortest / longest / max-x / reverse，
	 * 层内比较规则与 river 相同（按 minX 聚束），只是全局扫描方向不同。
	 */
	private sortNetworks(nets: Array<{ name: string; pads: EnginePad[] }>): Array<{ name: string; pads: EnginePad[] }> {
		const mode = this.planOptions.netOrder ?? 'auto';
		const span = (n: { name: string; pads: EnginePad[] }) => {
			const edges = this.computeMST(n.pads);
			let total = 0;
			for (const [i, j] of edges) total += Math.hypot(n.pads[i].x - n.pads[j].x, n.pads[i].y - n.pads[j].y);
			return total;
		};
		const allX = nets.flatMap(n => n.pads.map(p => p.x));
		const allY = nets.flatMap(n => n.pads.map(p => p.y));
		const boardDiag = allX.length
			? Math.hypot(Math.max(...allX) - Math.min(...allX), Math.max(...allY) - Math.min(...allY))
			: 0;
		const shortLimit = boardDiag * 0.22;

		const keyed = nets.map(n => {
			const xs = n.pads.map(p => p.x);
			const len = span(n);
			const ground = this.isGroundNet(n.name);
			const power = !ground && this.isPowerNet(n.name);
			let tier: number;
			if (mode === 'auto') {
				tier = ground ? 3 : power ? 2 : (n.pads.length === 2 && len <= shortLimit ? 0 : 1);
			} else {
				// 显式顺序：地网络硬性最后；powerNetsFirst 仅作为旧参数兼容
				tier = ground ? 3 : (this.planOptions.powerNetsFirst !== false && power ? 0 : 1);
			}
			return {
				net: n,
				minX: Math.min(...xs),
				maxX: Math.max(...xs),
				len,
				tier,
				ground,
			};
		});
		keyed.sort((a, b) => {
			if (a.tier !== b.tier) return a.tier - b.tier;
			switch (mode) {
				case 'longest': return b.len - a.len;
				case 'shortest': return a.len - b.len;
				case 'max-x': return b.maxX - a.maxX;
				case 'reverse': return b.maxX - a.maxX || b.minX - a.minX;
				case 'river':
				case 'auto':
				default: return a.minX - b.minX || a.maxX - b.maxX;
			}
		});

		// 并排焊盘封装（连接器、IC 引脚排）：先走中间的焊盘。
		// 原因：同一排外侧焊盘先出线时，走线容易绕到排外形成包围，把中间
		// 焊盘唯一的垂直出口封死；中间焊盘先垂直出线后，外侧焊盘仍可从
		// 排两端出线。信号网络（层级 0/1）跨层级合并重排，电源/地的层级
		// 约束不受影响（地仍最后）。
		const rowCount = mode === 'auto' || mode === 'river'
			? this.prioritizeRowInteriorPads(keyed)
			: 0;

		if (mode === 'auto') {
			const count = (t: number) => keyed.filter(k => k.tier === t).length;
			this.log(
				`自动规划：短连接 ${count(0)} → 一般信号 ${count(1)} → 电源 ${count(2)} → 地 ${count(3)}` +
				(count(3) ? '（地网络最后布，避免先布封路）' : ''),
				'info',
			);
			if (rowCount) this.log(`并排焊盘：${rowCount} 个网络改为中间焊盘优先出线`, 'info');
		}
		return keyed.map(k => k.net);
	}

	/**
	 * 检测共线焊盘排，并把排内网络按「中间 → 两端」重排。
	 *
	 * 返回被重排的网络数量（0 表示没有检测到并排焊盘）。
	 * 同层级的网络才会互相换位，避免把地/电源提前。
	 */
	private prioritizeRowInteriorPads<T extends {
		net: { name: string; pads: EnginePad[] };
		tier: number;
	}>(keyed: T[]): number {
		const pads = keyed.flatMap(k => k.net.pads);
		if (pads.length < 3) return 0;

		const rowScore = new Map<string, number>();
		// 并查集：同一排（以及同一封装的两排，如连接器上下两排共用网络）的网络
		// 属于同一组，组与组之间保持原来的相对位置，避免把两个独立连接器的
		// 网络块交错成一片、破坏成束走线。
		const parent = new Map<string, string>();
		const find = (x: string): string => {
			let r = x;
			while (parent.get(r) !== undefined && parent.get(r) !== r) r = parent.get(r)!;
			while (parent.get(x) !== undefined && parent.get(x) !== x) {
				const nx = parent.get(x)!;
				parent.set(x, r);
				x = nx;
			}
			return r;
		};
		const union = (a: string, b: string): void => {
			if (!parent.has(a)) parent.set(a, a);
			if (!parent.has(b)) parent.set(b, b);
			const ra = find(a), rb = find(b);
			if (ra !== rb) parent.set(ra, rb);
		};
		const markRow = (run: EnginePad[], crossAxis: 'x' | 'y', gap: number): void => {
			if (run.length < 3) return;
			if (new Set(run.map(p => p.net)).size < 2) return;
			const mid = (run.length - 1) / 2;
			const rdx = crossAxis === 'x' ? 1 : 0;
			const rdy = crossAxis === 'y' ? 1 : 0;
			run.forEach((p, i) => {
				// 1 = 排中心，0 = 排两端
				const t = 1 - Math.abs(i - mid) / Math.max(mid, 1);
				rowScore.set(p.net, Math.max(rowScore.get(p.net) ?? 0, t));
				// 行几何：出线预留（中部焊盘垂直于排向冲出引脚区）要用
				this.rowInfo.set(p.id, { dx: rdx, dy: rdy, gap, depth: t });
				if (!parent.has(p.net)) parent.set(p.net, p.net);
				if (i > 0) union(run[0].net, p.net);
			});
		};

		// axis = 排的走向坐标（x 相同 → 竖排；y 相同 → 横排）
		const scanRows = (axis: 'x' | 'y'): void => {
			const cross = axis === 'x' ? 'y' : 'x';
			const sorted = [...pads].sort((a, b) => a[axis] - b[axis]);
			const aligned: EnginePad[] = [];
			let clusterCoord = Number.NaN;
			const flushCluster = (): void => {
				if (aligned.length < 3) return;
				const runPads = [...aligned].sort((a, b) => a[cross] - b[cross]);
				// 真连接器/IC 引脚排的特征：等间距、等大小。按「间距一致性」分段，
				// 避免密集板上恰好共线但不相关的焊盘被链成一整排。
				const gaps: number[] = [];
				for (let i = 1; i < runPads.length; i++) {
					gaps.push(runPads[i][cross] - runPads[i - 1][cross]);
				}
				const medianGap = [...gaps].sort((a, b) => a - b)[Math.floor(gaps.length / 2)];
				let run: EnginePad[] = [runPads[0]];
				for (let i = 1; i < runPads.length; i++) {
					const gap = runPads[i][cross] - runPads[i - 1][cross];
					// 间距偏离中位间距过多 → 不是同一排，断开
					if (gap > medianGap * 1.8 + 2 || gap < medianGap * 0.4) {
						markRow(run, cross, medianGap);
						run = [];
					}
					run.push(runPads[i]);
				}
				markRow(run, cross, medianGap);
			};
			for (const p of sorted) {
				if (Number.isNaN(clusterCoord)) {
					clusterCoord = p[axis];
				} else if (p[axis] - clusterCoord > 4) {
					flushCluster();
					aligned.length = 0;
					clusterCoord = p[axis];
				}
				aligned.push(p);
			}
			flushCluster();
		};
		scanRows('x');
		scanRows('y');

		if (rowScore.size < 2) return 0;

		// 每个「焊盘排组」内部按「中间 → 两端」重排：只在这些网络原来占用的
		// 位置上换位，其它网络的位置不动。
		// 层级 0/1（短连接 + 一般信号）合并为一个池跨层级重排：排内短连接
		// 网络若按层级先行，会抢在中间焊盘之前出线（如 FPC 排两端焊盘的
		// 2-pad 网络），从排侧包住整排，封死中间焊盘的唯一出口。电源（层级 2）
		// 仍只在电源内部重排，地（层级 3）保持最后不动。
		let moved = 0;
		const components = new Map<string, Set<string>>();
		for (const name of rowScore.keys()) {
			const root = find(name);
			let set = components.get(root);
			if (!set) { set = new Set(); components.set(root, set); }
			set.add(name);
		}
		const byInteriority = (a: T, b: T) =>
			(rowScore.get(b.net.name) ?? 0) - (rowScore.get(a.net.name) ?? 0);
		for (const members of components.values()) {
			const signalSlots: number[] = [];
			const powerSlots: number[] = [];
			keyed.forEach((k, i) => {
				if (!members.has(k.net.name)) return;
				if (k.tier <= 1) signalSlots.push(i);
				else if (k.tier === 2) powerSlots.push(i);
			});
			for (const slots of [signalSlots, powerSlots]) {
				if (slots.length < 2) continue;
				const ordered = slots.map(i => keyed[i]).sort(byInteriority);
				slots.forEach((slot, i) => { keyed[slot] = ordered[i]; });
				moved += slots.length;
			}
		}
		return moved;
	}

	private checkCancelled(): boolean {
		if (this.cancelled || this.callbacks.isCancelled?.()) {
			this.cancelled = true;
			return true;
		}
		return false;
	}

	private log(message: string, level: string): void {
		this.logs.push({ level, message });
		this.callbacks.onLog?.(message, level);
	}

	private progress(percentage: number, message: string): void {
		this.callbacks.onProgress?.(percentage, message);
	}

	private isGroundNet(name: string): boolean {
		return isGroundNetName(name);
	}

	private isPowerNet(name: string): boolean {
		return isPowerNetName(name);
	}

	/** Prim MST：网络内连接顺序 */
	private computeMST(pads: EnginePad[]): Array<[number, number]> {
		const n = pads.length;
		if (n < 2) return [];
		const inMST = new Array<boolean>(n).fill(false);
		const minDist = new Array<number>(n).fill(Number.POSITIVE_INFINITY);
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

	/** 焊盘 → 障碍圆群（不含指定网络的焊盘） */
	/**
	 * 板框层内部圈线甄别：板框层折线统一按「阻挡所有层 + 走廊不剪枝」收集，
	 * 这对外轮廓是正确的，但拼版 V-CUT / 网格填充线也会落在板框层 ——
	 * 它们横贯板面、首尾顶到板边，把板面切成互不连通的条带，任何跨条带
	 * 网络都无解（实测某拼版工程 0% 布通）。布线以整份工程为一张板，
	 * 内部切割线不是电气障碍：只保留贴着整体包围盒边缘的圈（真外轮廓），
	 * 内部圈丢弃。纯外轮廓的板一个圈都不会丢，零行为差异。
	 */
	private triageBoardObstacles(obs: RBObstacle[]): RBObstacle[] {
		const board = obs.filter(o => o.group === 'board');
		if (!board.length) return obs;
		let mnx = Infinity, mxx = -Infinity, mny = Infinity, mxy = -Infinity;
		for (const o of board) {
			mnx = Math.min(mnx, o.x - o.r); mxx = Math.max(mxx, o.x + o.r);
			mny = Math.min(mny, o.y - o.r); mxy = Math.max(mxy, o.y + o.r);
		}
		const BAND = 60; // mil：外轮廓圈带宽（外轮廓圈心距板边 ≤ 半径+间距）
		const kept = obs.filter(o => {
			if (o.group !== 'board') return true;
			const de = Math.min(o.x - mnx, mxx - o.x, o.y - mny, mxy - o.y);
			return de <= BAND;
		});
		if (kept.length < obs.length) {
			this.log(`板框层内部圈线 ${obs.length - kept.length} 处非外轮廓，按非电气障碍忽略（拼版/V-CUT 线）`, 'info');
		}
		return kept;
	}

	private padObstacles(pads: EnginePad[], excludeNet: string): RBObstacle[] {
		// 焊盘障碍只依赖排除网络名，规划期间焊盘不变 —— 逐边重建上千个
		// 形状圆是纯浪费（每次 directEdgeRoute / viaEdgeCandidate 都会调用）。
		const hit = this.padObstacleCache.get(excludeNet);
		if (hit) return hit;
		const out: RBObstacle[] = [];
		let id = 0;
		const drc = this.drc;
		const infl = drc.trackToPadMil + drc.traceWidthMil / 2 + 0.3; // 安全余量
		for (const p of pads) {
			if (p.net === excludeNet) continue;
			const layer = p.layer === 12 ? undefined : p.layer;
			const group = `pad:${p.net}:${p.x},${p.y}`;
			const cached = this.padShapes[`${p.x.toFixed(2)},${p.y.toFixed(2)}`];
			const shape = {
				pad: cached?.pad ?? ['ELLIPSE', p.radius * 2, p.radius * 2],
				hole: cached?.hole ?? (p.holeDiameter > 0 ? ['ROUND', p.holeDiameter] : null),
				rotation: cached?.rotation ?? p.rotation ?? 0,
			};
			for (const c of padShapeToCircles(shape)) {
				out.push({
					id: `p${id++}`, kind: 'circle', x: p.x + c.x, y: p.y + c.y, r: c.r + infl,
					net: p.net, layer, group, ...(c.blockerOnly ? { blockerOnly: true } : {}),
				});
			}
		}
		this.padObstacleCache.set(excludeNet, out);
		return out;
	}

	/** 点到焊盘铜皮的有符号距离（内部为负），用于过孔落点校验 */
	private padCopperDistance(pad: EnginePad, x: number, y: number): number {
		const cached = this.padShapes[`${pad.x.toFixed(2)},${pad.y.toFixed(2)}`];
		const shape = {
			pad: cached?.pad ?? ['ELLIPSE', pad.radius * 2, pad.radius * 2],
			hole: cached?.hole ?? null,
			rotation: cached?.rotation ?? pad.rotation ?? 0,
		};
		return padShapeDistance(shape, x - pad.x, y - pad.y);
	}

	/**
	 * 过孔落点 → 全部焊盘的精确间距校验（直通孔在顶层/底层都有铜皮）。
	 *
	 *  - 异网焊盘：过孔铜环到焊盘铜皮 ≥ Track↔Pad DRC 间距，避免贴边违规；
	 *  - 同网焊盘：默认同样要求过孔铜环完全离开焊盘铜皮（不允许焊盘打孔）；
	 *    只有显式开启 allowViaInPad 后才放开同网焊盘。
	 *
	 * 这里按焊盘真实形状（矩形/椭圆/旋转）计算，而不是障碍圆近似，
	 * 避免矩形焊盘角部被圆群低估后过孔压进隔壁焊盘。
	 */
	/** 查询点 3×3 邻域格内的焊盘（viaPadClear 的空间索引） */
	private nearPads(x: number, y: number): EnginePad[] {
		const cs = this.PAD_GRID_CELL;
		const gx = Math.floor(x / cs), gy = Math.floor(y / cs);
		const out: EnginePad[] = [];
		for (let i = gx - 1; i <= gx + 1; i++) {
			for (let j = gy - 1; j <= gy + 1; j++) {
				const b = this.padGrid.get(`${i},${j}`);
				if (b) out.push(...b);
			}
		}
		return out;
	}

	private viaPadClear(x: number, y: number, viaR: number, netName: string): boolean {
		const drc = this.drc;
		// 外接圆预筛：外接圆之外的焊盘铜皮不可能违反净距，直接跳过 ——
		// 落点网格/射线扫描每个候选点都要过一遍邻近焊盘，形状求距
		//（旋转/多边形）才是大头，预筛能把绝大多数焊盘挡在求距之外。
		//（焊盘全集先经 64mil 空间网格收窄到 3×3 邻域，全量扫描 O(盘数)
		// × 数千候选点在密集板上纯烧时间）
		const need0 = viaR + drc.trackToPadMil + 1;
		for (const p of this.nearPads(x, y)) {
			// 过孔是贯穿孔：只关心顶层/底层/通孔焊盘
			if (p.layer !== 1 && p.layer !== 2 && p.layer !== 12 && p.padType !== 'th') continue;
			const sameNet = p.net === netName;
			if (sameNet && this.allowViaInPad) continue;
			const maxExtent = Math.max(p.radius, p.bodyRadius ?? 0);
			if (Math.hypot(p.x - x, p.y - y) - maxExtent > need0) continue;
			const d = this.padCopperDistance(p, x, y);
			const need = sameNet
				? viaR + 4
				: viaR + drc.trackToPadMil + 0.2;
			if (d < need) return false;
		}
		return true;
	}

	/**
	 * 焊盘终端：出线候选点的几何依据。
	 *  - 铜皮轮廓（圆群）由焊盘形状推出，用于确定各方向的出线边界
	 *  - 长焊盘的长轴方向作为常规出线方向（额外候选）
	 */
	private padTerminal(pad: EnginePad): PadTerminal {
		const cached = this.padShapes[`${pad.x.toFixed(2)},${pad.y.toFixed(2)}`];
		const shape = {
			pad: cached?.pad ?? ['ELLIPSE', pad.radius * 2, pad.radius * 2],
			hole: cached?.hole ?? (pad.holeDiameter > 0 ? ['ROUND', pad.holeDiameter] : null),
			rotation: cached?.rotation ?? 0,
		};
		const outline = padShapeToCircles(shape);
		// 长焊盘长轴（正负都会作为候选，故取哪一侧无所谓）
		const em = padExitModel(pad.x, pad.y, shape, { x: 1, y: 0 });
		return {
			x: pad.x,
			y: pad.y,
			net: pad.net,
			outline: outline.length ? outline : [{ x: 0, y: 0, r: Math.max(1, pad.bodyRadius || pad.radius) }],
			axisX: em ? em.axisX : undefined,
			axisY: em ? em.axisY : undefined,
		};
	}

	/** 逃逸通道预约已废弃：新算法由最短路自行决定出线方向 */

	/** 渲染图元 → 动态障碍（端点圆可绕行 + 沿线阻挡） */
	private addRenderedToObstacles(prims: RenderedPrim[]): void {
		const drc = this.drc;
		const r = drc.traceWidthMil / 2 + drc.trackToTrackMil + drc.traceWidthMil / 2 + 0.1;
		for (const pr of prims) {
			const group = `trk:${this.obstacleIdCounter++}`;
			if (pr.kind === 'line') {
				this.dynamicObstacles.push(...buildPolylineObstacle(
					[{ x: pr.x1!, y: pr.y1! }, { x: pr.x2!, y: pr.y2! }],
					r, pr.net, pr.layer, group,
				));
			} else {
				// 弧：沿弧加密阻挡圆（带弦→弧外凸补偿），另加一个「同心外包络圆」。
				// 包络圆与弧同心、半径 = 弧半径 + 间距半径，是一条与已布圆弧处处
				// 等距的曲线：后布网络可以相切地骑在它上面，形成论文图里那种
				// 同心嵌套的束状圆弧。它只参与建图（soft），不参与碰撞阻挡 ——
				// 否则会把这个大圆盘内部（圆角内侧的空地）封死。
				// 不加包络圆时，后布网络只能从前一网络弧的两端切点拉直线，
				// 要么切进弧内造成真实间距违规，要么被迫绕到障碍另一端出现钩子。
				const nSeg = Math.max(2, Math.ceil(Math.abs(pr.sweep!) / 0.35));
				const dTheta = pr.sweep! / nSeg;
				const bulge = pr.r! * (1 - Math.cos(Math.abs(dTheta) / 2));
				const rr = r + bulge;
				let px = pr.cx! + pr.r! * Math.cos(pr.startAngle!);
				let py = pr.cy! + pr.r! * Math.sin(pr.startAngle!);
				this.dynamicObstacles.push({ id: `dw${this.obstacleIdCounter++}`, kind: 'circle', x: px, y: py, r, net: pr.net, layer: pr.layer, group });
				for (let sIdx = 1; sIdx <= nSeg; sIdx++) {
					const a = pr.startAngle! + (pr.sweep! * sIdx) / nSeg;
					const nx = pr.cx! + pr.r! * Math.cos(a);
					const ny = pr.cy! + pr.r! * Math.sin(a);
					this.dynamicObstacles.push({
						id: `db${this.obstacleIdCounter++}`, kind: 'circle',
						x: nx, y: ny, r: rr, net: pr.net, layer: pr.layer, group,
						blockerOnly: true, bulge,
					});
					px = nx; py = ny;
				}
				this.dynamicObstacles.push({ id: `dw${this.obstacleIdCounter++}`, kind: 'circle', x: px, y: py, r, net: pr.net, layer: pr.layer, group });
				// 同心外包络（只建图、不阻挡）
				this.dynamicObstacles.push({
					id: `do${this.obstacleIdCounter++}`, kind: 'circle',
					x: pr.cx!, y: pr.cy!, r: pr.r! + rr, net: pr.net, layer: pr.layer,
					group, soft: true,
				});
			}
		}
		// 相邻图元共享端点，会生成位置完全重合的圆（线段端点 + 弧端点 + 线段的
		// capsule 端点）。可视图在重合圆上会各自生成切点，路径便沿一串几乎同心的
		// 小圆「扇贝式」抖动（0.05rad 级碎片弧 + 3mil 短线）。这里按几何去重，
		// 同一位置保留可用性最高的一个：可绕行 > 仅阻挡 > 只建图。
		const rank = (o: RBObstacle): number => (o.soft ? 2 : o.blockerOnly ? 1 : 0);
		const uniq = new Map<string, RBObstacle>();
		for (const o of this.dynamicObstacles) {
			const key = `${o.layer ?? ''}|${o.net ?? ''}|${o.x.toFixed(1)}|${o.y.toFixed(1)}|${o.r.toFixed(2)}${o.forceBlock ? '|f' : ''}`;
			const prev = uniq.get(key);
			if (!prev || rank(o) < rank(prev)) uniq.set(key, o);
		}
		this.dynamicObstacles = [...uniq.values()];
	}

	/**
	 * 焊盘级飞线进度。
	 *
	 * 「已布线焊盘」= 该焊盘在 MST 上的每条飞线都已布通（不再有飞线残留）；
	 * 多焊盘网络上只连了一部分的焊盘不计入，避免出现「100% 但仍有飞线」。
	 */
	private padProgress(
		net: { name: string; pads: EnginePad[] },
		edges: Array<[number, number]> = this.computeMST(net.pads),
	): NetPadProgress {
		const done = this.routedEdges.get(net.name) ?? new Set<string>();
		const degree = new Array<number>(net.pads.length).fill(0);
		const doneDegree = new Array<number>(net.pads.length).fill(0);
		for (const [si, di] of edges) {
			degree[si]++; degree[di]++;
			const key = `${net.pads[si].id}->${net.pads[di].id}`;
			if (done.has(key)) { doneDegree[si]++; doneDegree[di]++; }
		}
		let routed = 0;
		for (let i = 0; i < net.pads.length; i++) {
			if (doneDegree[i] >= degree[i]) routed++;
		}
		return { total: net.pads.length, routed };
	}

	/**
	 * 布一个网络的全部 MST 连接（纯规划；单条失败不再丢弃整网已布部分）。
	 *
	 * mode：
	 *  - 'quick'：线段级快扫 —— 每条边只试最便宜的单层直连（两级走廊在
	 *    同一次求解里），失败立即放弃此边，绝不烧退避/换层链。全网先扫完
	 *    一遍收割「一布就通」的边（用户策略：先完成能完成的，不追求单网
	 *    完整再下一网）。
	 *  - 'full'：完整链（边界候选退避 + 底层 + 过孔换层候选），收尾轮用。
	 */
	private async routeNetEdges(net: { name: string; pads: EnginePad[] }, mode: 'full' | 'quick' = 'full'): Promise<boolean> {
		const drc = this.drc;
		const edges = this.computeMST(net.pads);
		const edgeKey = (si: number, di: number) => `${net.pads[si].id}->${net.pads[di].id}`;
		// 部分连通的网络在撕毁/重试之间保留已完成的连接，只补未完成的边；
		// 否则重试会重复布线并叠加过孔。
		const done = this.routedEdges.get(net.name) ?? new Set<string>();
		// 本次尝试的图元/过孔先暂存：成功一条提交一条，失败连接不再拖累整网。
		const attemptRendered: RenderedPrim[] = [];
		const attemptVias: PlannedVia[] = [];

		for (const [si, di] of edges) {
			if (this.checkCancelled()) return false;
			const key = edgeKey(si, di);
			if (done.has(key)) continue;
			const src = net.pads[si], dst = net.pads[di];
			const routingLayer = getRoutingLayer(src, dst);
			const srcLayer = src.layer === 12 ? routingLayer : src.layer;
			const dstLayer = dst.layer === 12 ? routingLayer : dst.layer;
			// 顶层 SMD ↔ 底层 SMD：常规做法是在目标焊盘中心打孔（焊盘内过孔）。
			// 默认禁止焊盘打孔时必须改走「焊盘外过孔」路径；只有显式开启
			// allowViaInPad 才保留目标焊盘中心的过孔。
			const crossLayerSmd = src.padType === 'smd' && dst.padType === 'smd' &&
				(src.layer === 1 || src.layer === 2) && (dst.layer === 1 || dst.layer === 2) &&
				src.layer !== dst.layer;

			let rendered: RenderedPrim[] | null = null;
			const edgeVias: PlannedVia[] = [];

			if (mode === 'quick') {
				// 快扫只求直连；跨层 SMD 对必须过孔换层（无单层路径），留给收尾轮
				if (crossLayerSmd && !this.allowViaInPad) continue;
				rendered = await this.directEdgeRoute(src, dst, net.name, routingLayer);
				if (!rendered) continue;
			} else if (crossLayerSmd && !this.allowViaInPad) {
				// 顶层 SMD 引出段 + 焊盘外过孔 + 底层路径；不会在任一焊盘上打孔
				const viaCand = await this.viaEdgeCandidate(src, dst, net.name);
				if (viaCand) {
					rendered = viaCand.prims;
					edgeVias.push(...viaCand.vias);
				}
			} else {
				// 单层优先，两级退避：
				// ① 焊盘中心 → 可视图最短路 → 焊盘中心（首选，出线方向即最短路起始
				//    方向，除绕障必需的圆弧外整条走线没有人为折角 —— 橡皮筋拉紧效果）
				// ② 密集焊盘排里邻焊盘的外扩间距圈会吞没本焊盘中心（间距 < 两焊盘
				//    半径+外扩之和），中心起点的任何线段都判侵入 —— 退回焊盘边界
				//    出线候选（引出段 + 切线圆角）
				let topSolution = await this.directEdgeRoute(src, dst, net.name, routingLayer) ??
					await this.candidateEdgeRoute(src, dst, net.name, routingLayer);

				rendered = topSolution;

				// 并排焊盘（连接器/IC 引脚排）：仅当直连的出线方向贴着排向滑出
				//（会封死侧边焊盘唯一的垂直出口）时，改走法向预留出线；出线本来
				// 就垂直于排的直连保持原样 —— 切线出发拐角天然平滑，也省一次求解
				const srcRow = this.rowInfo.get(src.id);
				if (srcRow && srcRow.depth > 0.4) {
					const hugsRow = !rendered || this.exitHugsRow(rendered[0], src, srcRow);
					if (hugsRow) {
						const stubSol = await this.rowEscapeRoute(src, dst, srcRow, net.name, routingLayer);
						if (stubSol && (!rendered || renderedLength(stubSol) < renderedLength(rendered) * 1.35 + 20)) {
							topSolution = stubSol;
							rendered = stubSol;
						}
					}
				}

				// 「绕不过去」不只看走不走得通：顶层被障碍逼出 1.9 倍以上的大绕
				// （沿板边兜圈、在铺铜缝隙里穿针），继续贴层既难看又不合理 ——
				// 此时视为顶层绕不过去，比选换层方案，明显更短才换。
				const lb = Math.hypot(dst.x - src.x, dst.y - src.y);
				const topDetoured = rendered !== null && renderedLength(rendered) > lb * 1.55;

				// 顶层走不通（或绕得离谱）→ 换到底层再试：
				//  双端都能落底层（TH/底层焊盘）则底层直连，无需过孔；
				//  顶层 SMD 只能过孔换层兜底。
				if ((!rendered || topDetoured) && routingLayer === 1 && this.hasBottomLayer) {
					let altPrims: RenderedPrim[] | null = null;
					let altVias: PlannedVia[] = [];
					if (bottomCapable(src) && bottomCapable(dst)) {
						altPrims = await this.directEdgeRoute(src, dst, net.name, 2) ??
							await this.candidateEdgeRoute(src, dst, net.name, 2);
					} else {
						const viaCand = await this.viaEdgeCandidate(src, dst, net.name);
						if (viaCand) { altPrims = viaCand.prims; altVias = viaCand.vias; }
					}
					// 换层方案明显更短才采纳（过孔有工艺代价，省 50mil 不值得打孔）
					if (altPrims && (!rendered || renderedLength(altPrims) < renderedLength(rendered) - 50)) {
						rendered = altPrims;
						edgeVias.push(...altVias);
					}
				}

				// 两个单层都走不通 → 才用过孔换层兜底
				if (!rendered && this.hasBottomLayer) {
					const viaCand = await this.viaEdgeCandidate(src, dst, net.name);
					if (viaCand) {
						rendered = viaCand.prims;
						edgeVias.push(...viaCand.vias);
					}
				}

				// 跨层 SMD 对（顶层 ↔ 底层）：路径落在其中一层的焊盘中心，
				// 另一层焊盘靠过孔连接。该过孔位于焊盘内，仅在允许焊盘打孔时使用
				// （默认路径已在上面的 crossLayerSmd 分支改为焊盘外过孔）。
				if (rendered && srcLayer !== dstLayer) {
					const dia = Math.max(20, drc.traceWidthMil * 2);
					const hole = Math.max(10, drc.traceWidthMil);
					// 焊盘内过孔同样要避开隔壁异网焊盘；避不开就放弃这条连接
					if (!this.viaPadClear(dst.x, dst.y, dia / 2, net.name)) {
						rendered = null;
						edgeVias.length = 0;
					} else {
						edgeVias.push({ net: net.name, x: dst.x, y: dst.y, dia, hole });
					}
				}
			}

			if (!rendered) {
				this.log(`网络 ${net.name}: ${src.id}(${src.x.toFixed(0)},${src.y.toFixed(0)}) → ${dst.id}(${dst.x.toFixed(0)},${dst.y.toFixed(0)}) 未找到路径`, 'warn');
				continue;   // 其余连接继续布，已布部分不回滚
			}

			attemptRendered.push(...rendered);
			attemptVias.push(...edgeVias);
			done.add(key);
		}
		this.routedEdges.set(net.name, done);
		const complete = edges.every(([si, di]) => done.has(edgeKey(si, di)));

		// 成功连接立即提交图元、过孔与动态障碍（部分连通也保留在画布上）
		if (attemptRendered.length || attemptVias.length) {
			// 先合并再接缝：可视图会在同一障碍圆的多个切点之间切出碎片弧/微段，
			// 直接拿去建障碍会得到一串几乎重合的端点圆与包络圆，后布网络沿着
			// 它们走就会「扇贝式」抖动。合并后每段弧/线只登记一次。
			const mergedEdge = mergePrims(attemptRendered);
			const rds = this.netRendered.get(net.name) ?? [];
			rds.push(...mergedEdge);
			this.netRendered.set(net.name, rds);
			this.addRenderedToObstacles(mergedEdge);
			this.plannedVias.push(...attemptVias);
			// 过孔是贯通各层的孔洞：其它网络（任何层）都要给它让位
			for (const v of attemptVias) {
				this.dynamicObstacles.push({
					id: `via${this.obstacleIdCounter++}`, kind: 'circle', x: v.x, y: v.y,
					r: v.dia / 2 + Math.max(this.drc.trackToTrackMil, this.drc.trackToPadMil) + this.drc.traceWidthMil / 2 + 0.1,
					net: net.name, layer: undefined, group: `via:${net.name}:${v.x},${v.y}`, noPrune: true,
				});
			}
		}
		if (complete && !this.routedOrder.includes(net.name)) this.routedOrder.push(net.name);

		// 实时结果：主线程收到后立即写入/替换该网络的画布图元。
		// 只要网内有成功图元就按「可写」下发，否则失败网络会被整网删除 ——
		// 这正是「一条连接失败导致整网飞线都不布」的原因。
		const prims = mergePrims(this.netRendered.get(net.name) ?? []);
		this.callbacks.onNetUpdate?.(
			net.name, prims, this.plannedVias.filter(v => v.net === net.name),
			prims.length > 0, this.padProgress(net, edges),
		);
		this.log(
			complete
				? `网络 ${net.name} 布线完成（${edges.length} 条连接，${prims.length} 段图形）`
				: `网络 ${net.name} 部分连接失败（已布 ${done.size}/${edges.length} 条，保留有效走线）`,
			complete ? 'success' : 'warn',
		);
		return complete;
	}

	/** 把一个网络的全部 MST 连接登记为已完成（回滚「原本已完整布通」的走线时用） */
	private markAllEdgesDone(net: { name: string; pads: EnginePad[] }): void {
		const done = new Set<string>();
		for (const [si, di] of this.computeMST(net.pads)) {
			done.add(`${net.pads[si].id}->${net.pads[di].id}`);
		}
		this.routedEdges.set(net.name, done);
	}

	/** 网络还剩多少条 MST 连接未布通（易网优先排序用） */
	private netPendingEdges(net: { name: string; pads: EnginePad[] }): number {
		const edges = this.computeMST(net.pads);
		const done = this.routedEdges.get(net.name);
		if (!done) return edges.length;
		let pending = 0;
		for (const [si, di] of edges) {
			if (!done.has(`${net.pads[si].id}->${net.pads[di].id}`)) pending++;
		}
		return pending;
	}

	/** 当前层障碍全集（静态 + 其它网络焊盘 [+ 动态]） */
	private layerObstacles(netName: string, layer: number, includeDynamic: boolean): RBObstacle[] {
		const obs = [
			...this.staticObstacles,
			...this.padObstacles(this.allPads, netName),
			...(includeDynamic ? this.dynamicObstacles : []),
		];
		return obs.filter(o => o.layer === undefined || o.layer === layer);
	}

	/**
	 * 单层直连：焊盘中心 → 可视图最短路 → 焊盘中心（同一层，无过孔）。
	 *
	 * 本网络的焊盘不作为障碍（padObstacles 按网络排除），因此出线段与
	 * 路径首段天然共线/相切，整条走线除绕障必需的圆弧外没有任何折角。
	 */
	private async directEdgeRoute(
		src: EnginePad, dst: EnginePad, netName: string, layer: number,
		obs?: RBObstacle[],
	): Promise<RenderedPrim[] | null> {
		const o = obs ?? this.layerObstacles(netName, layer, true);
		const r = await routeConnection(
			{ x: src.x, y: src.y }, { x: dst.x, y: dst.y },
			o, netName, [150, 500], 3000,
			() => this.checkCancelled(),
		);
		if (!r.ok || !r.prims.length) return null;
		return r.prims.map(pr => pr.kind === 'line'
			? { kind: 'line' as const, net: netName, layer, x1: pr.x1, y1: pr.y1, x2: pr.x2, y2: pr.y2 }
			: { kind: 'arc' as const, net: netName, layer, cx: pr.cx, cy: pr.cy, r: pr.r, startAngle: pr.startAngle, sweep: pr.sweep });
	}

	/**
	 * 同层退避：焊盘边界出线候选 + 引出段 + 切线圆角（仅中心直连失败时使用）。
	 * 密集焊盘排里，邻焊盘的外扩间距圈会吞没本焊盘中心（间距 < 两焊盘半径+
	 * 外扩之和），中心起点的任何线段都判侵入 —— 此时从焊盘轮廓边界出线。
	 */
	private async candidateEdgeRoute(
		src: EnginePad, dst: EnginePad, netName: string, layer: number,
		obs?: RBObstacle[],
	): Promise<RenderedPrim[] | null> {
		const o = obs ?? this.layerObstacles(netName, layer, true);
		const edge = await routePadEdge(
			this.padTerminal(src), this.padTerminal(dst), o, netName,
			{
				expands: [150, 500],
				budgetMs: 3000,
				filletRadiusMil: Math.min(40, this.drc.traceWidthMil * 4),
				marginMil: this.escapeMargin,
				isCancelled: () => this.checkCancelled(),
			},
		);
		if (!edge.ok || !edge.prims.length) return null;
		return edge.prims.map(pr => pr.kind === 'line'
			? { kind: 'line' as const, net: netName, layer, x1: pr.x1, y1: pr.y1, x2: pr.x2, y2: pr.y2 }
			: { kind: 'arc' as const, net: netName, layer, cx: pr.cx, cy: pr.cy, r: pr.r, startAngle: pr.startAngle, sweep: pr.sweep });
	}

	/**
	 * 并排焊盘（连接器/IC 引脚排，贴片、间距近）的出线预留：
	 * 中部焊盘沿排法向先冲出引脚区，再交给可视图最短路。
	 * 直连最短路的出线方向是自由的，常贴着排向滑出去，正好把侧边
	 * 焊盘唯一的垂直出口封死（实测 FPC/排针/排阻区域）。
	 * 冲出长度 ≈ 焊盘包络 + 间距余量，让侧边焊盘随后能平行出线。
	 */
	private async rowEscapeRoute(
		pad: EnginePad, other: EnginePad,
		row: { dx: number; dy: number; gap: number; depth: number },
		netName: string, layer: number,
	): Promise<RenderedPrim[] | null> {
		const vx = other.x - pad.x, vy = other.y - pad.y;
		const dist = Math.hypot(vx, vy);
		if (dist < 1) return null;
		// 排法向 = 排向旋转 90°；目标几乎在排向上（贴排走线）时预留无意义
		const nx = -row.dy, ny = row.dx;
		const perp = Math.abs(vx * nx + vy * ny);
		if (perp < dist * 0.3) return null;
		const drc = this.drc;
		const L = Math.min(
			row.gap * 1.5,
			(pad.bodyRadius || pad.radius) + drc.trackToPadMil + drc.traceWidthMil / 2 + Math.max(10, row.gap * 0.5),
		);
		const term = this.padTerminal(pad);
		const obs = this.layerObstacles(netName, layer, true);
		const side = (vx * nx + vy * ny) >= 0 ? 1 : -1;
		for (const s of [side, -side]) {
			const ex = pad.x + nx * s * L, ey = pad.y + ny * s * L;
			if (!stubFeasible(term, ex, ey, obs, netName)) continue;
			const r = await routeConnection({ x: ex, y: ey }, { x: other.x, y: other.y }, obs, netName, [150, 500], 3000);
			if (!r.ok || !r.prims.length) continue;
			const prims: RenderedPrim[] = [
				{ kind: 'line', net: netName, layer, x1: pad.x, y1: pad.y, x2: ex, y2: ey },
				...r.prims.map(pr => pr.kind === 'line'
					? { kind: 'line' as const, net: netName, layer, x1: pr.x1, y1: pr.y1, x2: pr.x2, y2: pr.y2 }
					: { kind: 'arc' as const, net: netName, layer, cx: pr.cx, cy: pr.cy, r: pr.r, startAngle: pr.startAngle, sweep: pr.sweep }),
			];
			// 引出段→路径首段的拐点圆角（G1 切线过渡）：预留出线与后续路径
			// 常呈大角度拐弯，直角/尖角既难看又挤占侧边焊盘的出线净空
			const head = prims.length >= 2 && prims[0].kind === 'line' && prims[1].kind === 'line'
				? fanoutFillet(
					{ x1: prims[0].x1!, y1: prims[0].y1!, x2: prims[0].x2!, y2: prims[0].y2! },
					{ x1: prims[1].x1!, y1: prims[1].y1!, x2: prims[1].x2!, y2: prims[1].y2! },
					Math.min(24, this.drc.traceWidthMil * 3),
				)
				: null;
			return head ? [...(head.prims as RenderedPrim[]), ...prims.slice(2)] : prims;
		}
		return null;
	}

	/** 首段出线方向是否贴着排向滑出（|dot| > 0.5·|dir|）—— 会封死同排侧边焊盘的垂直出口 */
	private exitHugsRow(first: RenderedPrim, pad: EnginePad, row: { dx: number; dy: number }): boolean {
		let dx = 0, dy = 0;
		if (first.kind === 'line') {
			dx = (first.x2 ?? 0) - (first.x1 ?? 0);
			dy = (first.y2 ?? 0) - (first.y1 ?? 0);
		} else {
			// 弧：起点沿弧前进 1/4 段的弦方向
			const a0 = first.startAngle ?? 0;
			const a1 = a0 + (first.sweep ?? 0) * 0.25;
			const cx = first.cx ?? 0, cy = first.cy ?? 0, r = first.r ?? 0;
			dx = cx + r * Math.cos(a1) - (cx + r * Math.cos(a0));
			dy = cy + r * Math.sin(a1) - (cy + r * Math.sin(a0));
		}
		const len = Math.hypot(dx, dy);
		if (len < 1e-6) return true;   // 零长首段（罕见）：按贴排处理
		return Math.abs(dx * row.dx + dy * row.dy) > len * 0.5;
	}

	/**
	 * 「过孔换层」候选走线（单条连接，兜底）：
	 *   顶层引出段 → 过孔 → 底层路径 →（过孔）→ 顶层引出段
	 *
	 * 只在两个单层（顶层、底层直连）都走不通时才使用 —— 优先单层布线，
	 * 能不换层就不换层。顶层被先布的网络占满时，这是最后的通路。
	 */
	/** 过孔铜环间距判定（障碍圆半径已含对方半宽+间距，过孔比走线粗要补差值） */
	private viaObstacleClear(x: number, y: number, obs: RBObstacle[], viaR: number, netName: string): boolean {
		const viaExtra = viaR - this.drc.traceWidthMil / 2;
		for (const o of obs) {
			if (o.soft) continue;
			if (o.net === netName) continue;
			// 焊盘障碍已由 viaPadClear 按真实形状精确校验（圆外扩近似会冤杀
			// 1~5mil 的合法落点），这里只复查走线 / 静态 / 过孔障碍
			if (o.group?.startsWith('pad:')) continue;
			// EDA 的 Track↔Via 规则按焊盘间距执行（大于走线间距）
			if (Math.hypot(x - o.x, y - o.y) <
				o.r + viaExtra + Math.max(0, this.drc.trackToPadMil - this.drc.trackToTrackMil) + 0.1) return false;
		}
		return true;
	}

	/**
	 * 顶层 SMD 焊盘的过孔落点候选（焊盘外）：
	 *  1. 出线射线步进（原模型，每方向只留最近可行点）
	 *  2. 射线全军覆没时局部网格回���——密集焊盘排/大工艺焊盘封死外围时，
	 *     合法落点只剩相邻焊盘之间的毫米级窄缝，从焊盘中心出发的任何射线
	 *     都会先撞进邻盘的净距圈；网格直接在缝隙里找落点，按距焊盘距离排序。
	 * 落点均通过：自家铜皮距离、全盘净距（viaPadClear）、引出段可行、
	 * 顶层+底层过孔净距。
	 */
	private async viaSlotCandidates(
		pad: EnginePad, other: EnginePad, netName: string,
		topObs: RBObstacle[], bottomObs: RBObstacle[], viaR: number,
	): Promise<Array<{ x: number; y: number; stub: RenderedPrim[] }>> {
		const term = this.padTerminal(pad);
		const cands = buildExitCandidates(term, other, this.escapeMargin);
		const out: Array<{ x: number; y: number; stub: RenderedPrim[] }> = [];
		const lineStub = (x: number, y: number): RenderedPrim[] =>
			[{ kind: 'line', net: netName, layer: 1, x1: pad.x, y1: pad.y, x2: x, y2: y }];
		// 步进阶梯一直探到 400mil：0.5mm 间距的 QFP/连接器引脚区内几何上
		// 放不下任何过孔（孔+环+两侧净距 > 引脚间隙），过孔必须落在引脚区
		// 之外 —— 只探到 ~90mil 时整个引脚排区域被判「无槽位」，密集排
		// 附近的网络会集体失去换层兜底（实测某板 82 次网格全灭全是这个原因）。
		const slotSteps = [
			viaR + 4, viaR + 16, viaR + 32, viaR + 52, viaR + 78,
			viaR + 112, viaR + 152, viaR + 200, viaR + 256, viaR + 320, viaR + 400,
		];
		const dbg = !!(globalThis as any).__RB_DEBUG__;
		const rayDiag: string[] = [];
		for (const c of cands) {
			const deg = Math.round(Math.atan2(c.dirY, c.dirX) * 180 / Math.PI);
			if (!stubFeasible(term, c.x, c.y, topObs, netName)) {
				if (dbg) rayDiag.push(`${deg}°:出线即挡`);
				continue;
			}
			// 步进首档 ≥ 过孔半径 + 4mil，再沿方向续推直到铜环完全离开
			// 自家焊盘铜皮：斜向出线时距离增长率 < 1（45° 只有 0.707），
			// 只按 viaR 步进会让过孔铜环压进焊盘 2~4mil。
			let placed = false;
			for (const step of slotSteps) {
				let x = c.x + c.dirX * step, y = c.y + c.dirY * step;
				for (let i = 0; i < 16 && this.padCopperDistance(pad, x, y) < viaR + 4; i++) {
					x += c.dirX * 4;
					y += c.dirY * 4;
				}
				if (this.padCopperDistance(pad, x, y) < viaR + 4) {
					if (dbg) rayDiag.push(`${deg}°:自家铜皮@${step.toFixed(0)}`);
					continue;
				}
				if (!this.viaPadClear(x, y, viaR, netName)) {
					if (dbg) rayDiag.push(`${deg}°:焊盘净距@${step.toFixed(0)}`);
					continue;
				}
				if (!stubFeasible(term, x, y, topObs, netName)) {
					if (dbg) rayDiag.push(`${deg}°:引出段被挡@${step.toFixed(0)}`);
					continue;
				}
				if (!this.viaObstacleClear(x, y, topObs, viaR, netName)) {
					if (dbg) rayDiag.push(`${deg}°:顶层净距@${step.toFixed(0)}`);
					continue;
				}
				if (!this.viaObstacleClear(x, y, bottomObs, viaR, netName)) {
					if (dbg) rayDiag.push(`${deg}°:底层净距@${step.toFixed(0)}`);
					continue;
				}
				out.push({ x, y, stub: lineStub(x, y) });
				placed = true;
				break;   // 同一方向只保留最近的可行过孔
			}
			if (dbg && !placed && rayDiag[rayDiag.length - 1]?.startsWith(`${deg}°`) === false) {
				rayDiag.push(`${deg}°:阶梯尽@400`);
			}
		}
		if (dbg && out.length === 0 && rayDiag.length) {
			console.warn(`[viaSlot] ${netName} pad(${pad.x.toFixed(0)},${pad.y.toFixed(0)}) 射线全灭: ${rayDiag.join(' ')}`);
		}
		if (out.length === 0) {
			const R = Math.max(150, (pad.radius + viaR) * 2 + 40);
			const step = 8;
			const x0 = Math.ceil((pad.x - R) / step) * step;
			const y0 = Math.ceil((pad.y - R) / step) * step;
			// L 形引出段的第二段起点（拐角处没有焊盘铜皮可豁免，用零半径端点）
			const cornerTerm = (cx: number, cy: number): PadTerminal => ({
				...term, x: cx, y: cy, outline: [{ x: 0, y: 0, r: 0.5 }],
			});
			// 邻域子集：网格逐点检查只可能撞上焊盘附近 R+余量 内的障碍；全量障碍
			//（密集板 8000+）× 万级网格点 = 最坏上亿次几何判定，纯烧分钟
			const nearBox = (o: RBObstacle) =>
				Math.abs(o.x - pad.x) <= R + 120 && Math.abs(o.y - pad.y) <= R + 120;
			const localTop = topObs.filter(nearBox);
			const localBottom = bottomObs.filter(nearBox);
			const lFeasible = (cx: number, cy: number, gx2: number, gy2: number): boolean =>
				stubFeasible(term, cx, cy, localTop, netName) &&
				stubFeasible(cornerTerm(cx, cy), gx2, gy2, localTop, netName);
			type Slot = { x: number; y: number; d: number; kind: 'line' | 'bend' | 'router'; bend?: { x: number; y: number } };
			const grid: Slot[] = [];
			let rej = { own: 0, pads: 0, via: 0 };
			for (let gx = x0; gx <= pad.x + R; gx += step) {
				if (this.checkCancelled()) return out;
				for (let gy = y0; gy <= pad.y + R; gy += step) {
					const d = Math.hypot(gx - pad.x, gy - pad.y);
					if (d < pad.radius || d > R) continue;
					if (this.padCopperDistance(pad, gx, gy) < viaR + 4) { rej.own++; continue; }
					if (!this.viaPadClear(gx, gy, viaR, netName)) { rej.pads++; continue; }
					// 引出段形态：直线 → L 形拐 → 留给可视图路由器绕行
					let slot: Slot;
					if (stubFeasible(term, gx, gy, localTop, netName)) {
						slot = { x: gx, y: gy, d, kind: 'line' };
					} else if (lFeasible(gx, pad.y, gx, gy)) {
						slot = { x: gx, y: gy, d, kind: 'bend', bend: { x: gx, y: pad.y } };
					} else if (lFeasible(pad.x, gy, gx, gy)) {
						slot = { x: gx, y: gy, d, kind: 'bend', bend: { x: pad.x, y: gy } };
					} else {
						slot = { x: gx, y: gy, d, kind: 'router' };
					}
					if (!this.viaObstacleClear(gx, gy, localTop, viaR, netName) ||
						!this.viaObstacleClear(gx, gy, localBottom, viaR, netName)) { rej.via++; continue; }
					grid.push(slot);
				}
			}
			const rank = (s: Slot) => (s.kind === 'line' ? 0 : s.kind === 'bend' ? 1 : 2);
			grid.sort((p, q) => rank(p) - rank(q) || p.d - q.d);
			const good = grid.filter(s => s.kind !== 'router').slice(0, 6);
			for (const g of good) {
				let stub: RenderedPrim[];
				if (g.kind === 'line') {
					stub = lineStub(g.x, g.y);
				} else {
					const s1: RenderedPrim = { kind: 'line', net: netName, layer: 1, x1: pad.x, y1: pad.y, x2: g.bend!.x, y2: g.bend!.y };
					const s2: RenderedPrim = { kind: 'line', net: netName, layer: 1, x1: g.bend!.x, y1: g.bend!.y, x2: g.x, y2: g.y };
					// L 形拐角切线圆角：直角拐弯既难看又挤占邻侧净空
					const f = fanoutFillet(
						{ x1: s1.x1!, y1: s1.y1!, x2: s1.x2!, y2: s1.y2! },
						{ x1: s2.x1!, y1: s2.y1!, x2: s2.x2!, y2: s2.y2! },
						Math.min(24, this.drc.traceWidthMil * 3),
					);
					stub = f ? [...(f.prims as RenderedPrim[])] : [s1, s2];
				}
				out.push({ x: g.x, y: g.y, stub });
			}
			// 直线 / L 形全灭时落点仍可能可达（邻盘净距圈之间有斜向通道）：
			// 用可视图路由器铺顶层引出段，可绕开直线与单拐覆盖不到的形态。
			if (good.length === 0) {
				let routerOk = 0;
				for (const g of grid.filter(s => s.kind === 'router').slice(0, 6)) {
					if (this.checkCancelled()) return out;
					const r = await routePadEdge(term, cornerTerm(g.x, g.y), localTop, netName,
						{ isCancelled: () => this.checkCancelled() });
					if (!r.ok || r.prims.length === 0) continue;
					out.push({
						x: g.x, y: g.y,
						stub: r.prims.map(pr => pr.kind === 'line'
							? { kind: 'line', net: netName, layer: 1, x1: pr.x1, y1: pr.y1, x2: pr.x2, y2: pr.y2 }
							: {
								kind: 'arc', net: netName, layer: 1,
								cx: pr.cx, cy: pr.cy, r: pr.r, startAngle: pr.startAngle, sweep: pr.sweep,
							}),
					});
					routerOk++;
					if (routerOk >= 3) break;
				}
				if (routerOk === 0) {
					console.warn(`[viaSlot] ${netName} pad(${pad.x.toFixed(0)},${pad.y.toFixed(0)}) 网格全灭(含路由器兜底): ` +
						`own=${rej.own} pads=${rej.pads} via=${rej.via} R=${R.toFixed(0)}`);
				}
			}
		}
		return out;
	}

	private async viaEdgeCandidate(
		src: EnginePad, dst: EnginePad, netName: string,
	): Promise<{ prims: RenderedPrim[]; vias: PlannedVia[]; length: number } | null> {
		if (!this.hasBottomLayer) return null;
		const drc = this.drc;
		const topLayer = 1, bottomLayer = 2;
		const viaDia = Math.max(24, drc.traceWidthMil * 2);
		const viaHole = Math.max(12, drc.traceWidthMil);
		const viaR = viaDia / 2;

		const topObs = [
			...this.staticObstacles,
			...this.padObstacles(this.allPads, netName),
			...this.dynamicObstacles,
		].filter(o => o.layer === undefined || o.layer === topLayer);
		const bottomObs = [
			...this.staticObstacles,
			...this.padObstacles(this.allPads, netName),
			...this.dynamicObstacles,
		].filter(o => o.layer === undefined || o.layer === bottomLayer);

		// 引出段清障与过孔净距统一走 viaObstacleClear / viaSlotCandidates
		const viaClear = (x: number, y: number, obs: RBObstacle[]): boolean =>
			this.viaObstacleClear(x, y, obs, viaR, netName);

		interface Anchor {
			x: number; y: number;
			stub: RenderedPrim[];
			via: { x: number; y: number } | null;
			stubLen: number;
		}

		// 每个端点的落点候选：
		//  - 通孔 / 底层焊盘：直接在底层出图，无需过孔
		//  - 顶层 SMD：出线射线步进 + 网格回退（viaSlotCandidates），
		//    找既满足顶层引出段、又满足过孔间距（顶层 + 底层）的位置
		const stubLenOf = (prims: RenderedPrim[]): number => prims.reduce((s, p) =>
			p.kind === 'line'
				? s + Math.hypot((p.x2 ?? 0) - (p.x1 ?? 0), (p.y2 ?? 0) - (p.y1 ?? 0))
				: s + Math.abs(p.sweep ?? 0) * (p.r ?? 0), 0);
		const endAnchors = async (pad: EnginePad, other: EnginePad): Promise<Anchor[]> => {
			if (pad.layer === 12 || pad.layer === bottomLayer) {
				return [{ x: pad.x, y: pad.y, stub: [], via: null, stubLen: 0 }];
			}
			if (pad.layer !== topLayer) return [];
			const out: Anchor[] = [];
			// 允许焊盘打孔时，焊盘中心是最短的换层点（过孔直接落在 SMD 焊盘上）
			if (this.allowViaInPad &&
				this.viaPadClear(pad.x, pad.y, viaR, netName) &&
				viaClear(pad.x, pad.y, topObs) && viaClear(pad.x, pad.y, bottomObs)) {
				out.push({
					x: pad.x, y: pad.y, stub: [],
					via: { x: pad.x, y: pad.y }, stubLen: 0,
				});
			}
			for (const c of await this.viaSlotCandidates(pad, other, netName, topObs, bottomObs, viaR)) {
				out.push({
					x: c.x, y: c.y,
					stub: c.stub,
					via: { x: c.x, y: c.y },
					stubLen: stubLenOf(c.stub),
				});
			}
			return out;
		};

		const anchorsA = await endAnchors(src, dst);
		const anchorsB = await endAnchors(dst, src);
		if (!anchorsA.length || !anchorsB.length) return null;

		// 按「引出段总长 + 底层直线距离」升序逐对尝试落点：
		// 估值最优的一对底层未必走得通（会被先布网络的斜线封死），
		// 失败就换下一对，而不是直接放弃。
		const pairs: Array<{ a: Anchor; b: Anchor; est: number }> = [];
		for (const a of anchorsA) {
			for (const b of anchorsB) {
				pairs.push({ a, b, est: a.stubLen + b.stubLen + Math.hypot(a.x - b.x, a.y - b.y) });
			}
		}
		pairs.sort((p, q) => p.est - q.est);

		const bottomFor = async (A: Anchor, B: Anchor): Promise<RenderedPrim[] | null> => {
			// 底层路径：直连优先，被挡才跑可视图
			const bottomBlocked = bottomObs.some(o =>
				!o.soft && o.net !== netName &&
				pointSegDist(o.x, o.y, A.x, A.y, B.x, B.y) < o.r - 0.05);
			if (!bottomBlocked) {
				return [{
					kind: 'line', net: netName, layer: bottomLayer,
					x1: A.x, y1: A.y, x2: B.x, y2: B.y,
				}];
			}
			const r = await routeConnection(
				{ x: A.x, y: A.y }, { x: B.x, y: B.y },
				bottomObs, netName, [150, 500], 1500,
				() => this.checkCancelled(),
			);
			if (!r.ok || r.prims.length === 0) return null;
			return r.prims.map(pr => pr.kind === 'line'
				? { kind: 'line' as const, net: netName, layer: bottomLayer, x1: pr.x1, y1: pr.y1, x2: pr.x2, y2: pr.y2 }
				: {
					kind: 'arc' as const, net: netName, layer: bottomLayer,
					cx: pr.cx, cy: pr.cy, r: pr.r, startAngle: pr.startAngle, sweep: pr.sweep,
				});
		};

		let pickA: Anchor | null = null, pickB: Anchor | null = null;
		let bottomPrims: RenderedPrim[] | null = null;
		for (const { a, b } of pairs.slice(0, 8)) {
			if (this.checkCancelled()) return null;
			const bp = await bottomFor(a, b);
			if (bp) { pickA = a; pickB = b; bottomPrims = bp; break; }
		}
		if (!pickA || !pickB || !bottomPrims) return null;

		const prims: RenderedPrim[] = [];
		if (pickA.stub.length) prims.push(...pickA.stub);
		prims.push(...bottomPrims);
		if (pickB.stub.length) prims.push(...pickB.stub);
		const vias: PlannedVia[] = [];
		if (pickA.via) vias.push({ net: netName, x: pickA.via.x, y: pickA.via.y, dia: viaDia, hole: viaHole });
		if (pickB.via) vias.push({ net: netName, x: pickB.via.x, y: pickB.via.y, dia: viaDia, hole: viaHole });

		let length = pickA.stubLen + pickB.stubLen;
		for (const p of bottomPrims) {
			length += p.kind === 'line'
				? Math.hypot((p.x2 ?? 0) - (p.x1 ?? 0), (p.y2 ?? 0) - (p.y1 ?? 0))
				: Math.abs(p.sweep ?? 0) * (p.r ?? 0);
		}
		return { prims, vias, length };
	}

	/**
	 * 过孔槽位驱逐：失败网的顶层过孔落点若被「单一其它网络的已布铜皮」独占封死
	 * （剔除该网全部障碍后槽位即可用），撕毁该网重排 —— 过孔槽位是跨网互斥的
	 * 稀缺资源，贪心顺序一旦错位就会把后来者全部封死（62 板 6 网全要过孔、
	 * 槽位列完全重叠时，任何固定顺序都收敛在 3/6）。
	 *
	 * 条件从严（不为 hopeless 几何白拆已布网）：
	 *  - 只看完全未布的网络（零图元）；
	 *  - 只统计「直线 / L 形引出段可行 + 焊盘过滤通过」的落点，且其过孔净距
	 *    违反障碍恰好全部属于单一其它网络（板边圈等无主障碍视为永久阻挡）；
	 *  - 每对「失败网 × 被拆网」只试一次（防乒乓），每网最多拆 3 个对象网。
	 *
	 * 返回 true 表示本网已借此布通（被拆网已就地重布，失败则留待后续兜底）。
	 */
	private async tryViaSlotEviction(
		net: { name: string; pads: EnginePad[] },
		networks: Array<{ name: string; pads: EnginePad[] }>,
	): Promise<boolean> {
		if (!this.hasBottomLayer) return false;
		// 每次驱逐尝试 = 拆网 + 两次整网重布，代价大：全局限次，且剩余预算不足 120s 时不再尝试
		if (this.evictionAttempts >= 6) return false;
		if (Date.now() > this.planDeadline - 120_000) return false;
		const drc = this.drc;
		const viaDia = Math.max(24, drc.traceWidthMil * 2);
		const viaR = viaDia / 2;
		const viaExtra = viaR - drc.traceWidthMil / 2;
		const bonus = Math.max(0, drc.trackToPadMil - drc.trackToTrackMil);

		const topObs = [
			...this.staticObstacles,
			...this.padObstacles(this.allPads, net.name),
			...this.dynamicObstacles,
		].filter(o => o.layer === undefined || o.layer === 1);
		const bottomObs = [
			...this.staticObstacles,
			...this.padObstacles(this.allPads, net.name),
			...this.dynamicObstacles,
		].filter(o => o.layer === undefined || o.layer === 2);

		// 本网需要过孔的顶层 SMD 焊盘：找不到就不用驱逐（TH/底层盘直接走底层）
		const topSmd = net.pads.filter(p => p.layer === 1 && p.padType === 'smd');
		if (!topSmd.length) return false;

		// 与 viaSlotCandidates 网格回退同口径的落点扫描，这里只做「可驱逐性」归因：
		// 落点的过孔净距违反障碍若恰为单一其它网络 ⇒ 拆它即可解锁该落点
		const freedBy = new Map<string, number>();
		for (const pad of topSmd) {
			const other = [...net.pads]
				.filter(p => p !== pad)
				.sort((a, b) => Math.hypot(a.x - pad.x, a.y - pad.y) - Math.hypot(b.x - pad.x, b.y - pad.y))[0];
			if (!other) continue;
			const term = this.padTerminal(pad);
			const cornerTerm = (cx: number, cy: number): PadTerminal => ({
				...term, x: cx, y: cy, outline: [{ x: 0, y: 0, r: 0.5 }],
			});
			// 邻域子集：同 viaSlotCandidates 网格回退 —— 万级网格点 × 全量障碍
			// 是最坏上亿次几何判定，只查焊盘附近 R+余量 内的障碍
			const R = Math.max(150, (pad.radius + viaR) * 2 + 40);
			const nearBox = (o: RBObstacle) =>
				Math.abs(o.x - pad.x) <= R + 120 && Math.abs(o.y - pad.y) <= R + 120;
			const localTop = topObs.filter(nearBox);
			const localBottom = bottomObs.filter(nearBox);
			const lFeasible = (cx: number, cy: number, gx2: number, gy2: number): boolean =>
				stubFeasible(term, cx, cy, localTop, net.name) &&
				stubFeasible(cornerTerm(cx, cy), gx2, gy2, localTop, net.name);
			const step = 8;
			for (let gx = Math.ceil((pad.x - R) / step) * step; gx <= pad.x + R; gx += step) {
				if (this.checkCancelled()) return false;
				for (let gy = Math.ceil((pad.y - R) / step) * step; gy <= pad.y + R; gy += step) {
					const d = Math.hypot(gx - pad.x, gy - pad.y);
					if (d < pad.radius || d > R) continue;
					if (this.padCopperDistance(pad, gx, gy) < viaR + 4) continue;
					if (!this.viaPadClear(gx, gy, viaR, net.name)) continue;
					// 引出段本身进不去的落点，拆过孔也救不了（可视图绕行形态不纳入）
					const stubOk = stubFeasible(term, gx, gy, localTop, net.name) ||
						lFeasible(gx, pad.y, gx, gy) || lFeasible(pad.x, gy, gx, gy);
					if (!stubOk) continue;
					if (this.viaObstacleClear(gx, gy, localTop, viaR, net.name) &&
						this.viaObstacleClear(gx, gy, localBottom, viaR, net.name)) continue;   // 本来就可用的槽位
					// 归因：与 viaObstacleClear 同口径，找净距违反的障碍主
					const violating = new Set<string>();
					let permanent = false;
					for (const o of [...topObs, ...bottomObs]) {
						if (o.soft || o.net === net.name) continue;
						if (o.group?.startsWith('pad:')) continue;   // 焊盘净距由 viaPadClear 单独校验
						if (Math.hypot(gx - o.x, gy - o.y) <
							o.r + viaExtra + bonus + 0.1) {
							if (o.net) violating.add(o.net);
							else permanent = true;   // 板边圈等无主障碍拆不掉
						}
					}
					if (permanent || violating.size !== 1) continue;
					const b = [...violating][0];
					freedBy.set(b, (freedBy.get(b) ?? 0) + 1);
				}
			}
		}
		if (!freedBy.size) return false;

		const blockers = [...freedBy.entries()].sort((a, b) => b[1] - a[1]).slice(0, 3);
		for (const [blocker, slots] of blockers) {
			if (!this.routedOrder.includes(blocker)) continue;
			// 弱信号不值得拆：只解锁 1 个槽位时收益太小（62 板实测两次单槽驱逐全部白拆）
			if (slots < 2) continue;
			// 大网不拆：重布代价与其连接数成正比（GND 类电源地网撕一次要重布几十条）
			const blockerNet = networks.find(n => n.name === blocker);
			if (blockerNet && blockerNet.pads.length > 8) continue;
			const key = `${net.name}|${blocker}`;
			if (this.evictionTried.has(key)) continue;
			this.evictionTried.add(key);
			this.evictionAttempts++;
			this.log(`过孔槽位驱逐：${net.name} 的落点被 ${blocker} 独占封死（可解锁 ${slots} 个），拆网重排`, 'warn');
			// 快照被拆网络的原走线：本动作绝不净损失已布网络——
			// 目标网没布通且被拆网也没救回时，整体回滚到驱逐前的已知好状态
			const savedBlocker = {
				prims: [...(this.netRendered.get(blocker) ?? [])],
				vias: this.plannedVias.filter(v => v.net === blocker),
			};
			const savedOrderIdx = this.routedOrder.indexOf(blocker);
			this.ripupNet(blocker);
			const ok = await this.routeNetEdges(net);
			const rok = blockerNet ? await this.routeNetEdges(blockerNet) : false;
			// 被拆网重布失败不能悬空：stillFailed 快照里没有它，直接走普通过孔兜底
			if (!rok) {
				this.log(`过孔槽位驱逐：重布 ${blocker} 失败，转过孔兜底`, 'warn');
				if (blockerNet) await this.tryViaFallback(blockerNet);
			}
			if (ok) {
				this.log(`过孔槽位驱逐：${net.name} 布通`, 'success');
				return true;
			}
			// 目标网没布通且被拆网也没救回 → 恢复被拆网原走线，再试下一个候选
			if (blockerNet && !this.routedOrder.includes(blocker)) {
				this.ripupNetInternal(blocker, true);
				if (savedBlocker.prims.length) {
					this.netRendered.set(blocker, [...savedBlocker.prims]);
					this.addRenderedToObstacles(mergePrims(savedBlocker.prims));
				}
				this.plannedVias = this.plannedVias.filter(v => v.net !== blocker).concat(savedBlocker.vias);
				for (const v of savedBlocker.vias) {
					this.dynamicObstacles.push({
						id: `via${this.obstacleIdCounter++}`, kind: 'circle',
						x: v.x, y: v.y, r: v.dia / 2 + Math.max(this.drc.trackToTrackMil, this.drc.trackToPadMil) + this.drc.traceWidthMil / 2 + 0.1,
						net: v.net, layer: undefined, group: `via:${v.net}:${v.x},${v.y}`, noPrune: true,
					});
				}
				this.markAllEdgesDone(blockerNet);
				if (savedOrderIdx >= 0) {
					this.routedOrder.splice(Math.min(savedOrderIdx, this.routedOrder.length), 0, blocker);
				}
				this.callbacks.onNetUpdate?.(
					blocker, mergePrims(savedBlocker.prims), savedBlocker.vias,
					savedBlocker.prims.length > 0, this.padProgress(blockerNet),
				);
			}
		}
		return false;
	}

	/**
	 * 过孔换层兜底：顶层 SMD 焊盘 → 顶层引出段 → 过孔 → 底层最短路径 → 过孔 → 顶层引出段 → 顶层焊盘。
	 * 出线方向取自出线候选集（朝目标方向优先），仅在底层成功、且过孔/顶层引出段满足间距时提交。
	 */
	private async tryViaFallback(net: { name: string; pads: EnginePad[] }): Promise<boolean> {
		if (net.pads.length < 2) return false;
		if (!this.hasBottomLayer) return false;
		const a = net.pads[0], b = net.pads[net.pads.length - 1];
		const topLayer = 1, bottomLayer = 2;
		const drc = this.drc;
		// 过孔钻孔需满足板厂最小 0.3mm，环宽至少 6mil
		const viaDia = Math.max(24, drc.traceWidthMil * 2);
		const viaHole = Math.max(12, drc.traceWidthMil);
		const viaR = viaDia / 2;
		const viaExtra = viaR - drc.traceWidthMil / 2;

		const topObs = [
			...this.staticObstacles,
			...this.padObstacles(this.allPads, net.name),
			...this.dynamicObstacles,
		].filter(o => o.layer === undefined || o.layer === topLayer);
		const bottomObs = [
			...this.staticObstacles,
			...this.padObstacles(this.allPads, net.name),
			...this.dynamicObstacles,
		].filter(o => o.layer === undefined || o.layer === bottomLayer);

		const viaClear = (x: number, y: number, obs: RBObstacle[]): boolean => {
			for (const o of obs) {
				if (o.soft) continue;
				if (o.net === net.name) continue;
				// 焊盘障碍已由 viaPadClear 按真实形状精确校验（口径与 viaObstacleClear 一致）
				if (o.group?.startsWith('pad:')) continue;
				// EDA 的 Track↔Via 规则按焊盘间距执行（大于走线间距）
				if (Math.hypot(x - o.x, y - o.y) <
					o.r + viaExtra + Math.max(0, drc.trackToPadMil - drc.trackToTrackMil) + 0.1) return false;
			}
			return true;
		};

		interface EndSetup {
			via: { x: number; y: number } | null;
			stub: RenderedPrim[];
			bottom: { x: number; y: number };
		}

		// 顶层 SMD 焊盘需要引出段+过孔；TH / 底层焊盘可直接从焊盘中心走底层。
		// 返回全部可行候选（不止首个），供两端配对挑「引出段总长+底层跨距」最短的组合
		const setupEnds = async (pad: EnginePad, other: EnginePad): Promise<EndSetup[]> => {
			if (pad.layer === 12 || pad.layer === bottomLayer) {
				return [{ via: null, stub: [], bottom: { x: pad.x, y: pad.y } }];
			}
			if (pad.layer !== topLayer) return [];
			const ends: EndSetup[] = [];
			// 允许焊盘打孔时优先在焊盘中心换层
			if (this.allowViaInPad &&
				this.viaPadClear(pad.x, pad.y, viaR, net.name) &&
				viaClear(pad.x, pad.y, topObs) && viaClear(pad.x, pad.y, bottomObs)) {
				ends.push({ via: { x: pad.x, y: pad.y }, stub: [], bottom: { x: pad.x, y: pad.y } });
			}
			// 出线射线步进 + 网格回退（含 L 形 / 路由器绕行引出段），候选按序追加
			const cands = await this.viaSlotCandidates(pad, other, net.name, topObs, bottomObs, viaR);
			for (const c of cands.slice(0, 9)) {
				ends.push({
					via: { x: c.x, y: c.y },
					stub: c.stub,
					bottom: { x: c.x, y: c.y },
				});
			}
			return ends;
		};

		// 两端各取多个候选，按「引出段总长 + 底层跨距」从小到大配对，依次尝试
		const stubLen = (e: EndSetup): number => e.stub.reduce((s, pr) =>
			pr.kind === 'line'
				? s + Math.hypot((pr.x2 ?? 0) - (pr.x1 ?? 0), (pr.y2 ?? 0) - (pr.y1 ?? 0))
				: pr.kind === 'arc' ? s + Math.abs((pr.r ?? 0) * (pr.sweep ?? 0)) : s, 0);
		const endAs = await setupEnds(a, b);
		const endBs = await setupEnds(b, a);
		if (!endAs.length || !endBs.length) {
			this.log(`过孔兜底 ${net.name}：顶层出线或过孔位置不可用`, 'info');
			return false;
		}
		const pairs: Array<[EndSetup, EndSetup]> = [];
		for (const ea of endAs) {
			for (const eb of endBs) pairs.push([ea, eb]);
		}
		const pairCost = (p: [EndSetup, EndSetup]): number =>
			stubLen(p[0]) + stubLen(p[1]) + Math.hypot(p[0].bottom.x - p[1].bottom.x, p[0].bottom.y - p[1].bottom.y);
		pairs.sort((p, q) => pairCost(p) - pairCost(q));

		let routed = false;
		for (const [endA, endB] of pairs.slice(0, 4)) {
			if (this.checkCancelled()) return false;
			// 底层优先直连（任意角度最短线），被挡时才跑可视图
			const bottomBlocked = bottomObs.some(o =>
				!o.soft && o.net !== net.name &&
				pointSegDist(o.x, o.y, endA.bottom.x, endA.bottom.y, endB.bottom.x, endB.bottom.y) < o.r - 0.05);
			let bottomPrims: RenderedPrim[];
			if (!bottomBlocked) {
				bottomPrims = [{
					kind: 'line', net: net.name, layer: bottomLayer,
					x1: endA.bottom.x, y1: endA.bottom.y, x2: endB.bottom.x, y2: endB.bottom.y,
				}];
			} else {
				const bottomRoute = await routeConnection(endA.bottom, endB.bottom, bottomObs, net.name, [150, 500], 3000,
					() => this.checkCancelled());
				if (!bottomRoute.ok || bottomRoute.prims.length === 0) continue;
				bottomPrims = bottomRoute.prims.map(pr => pr.kind === 'line'
					? { kind: 'line', net: net.name, layer: bottomLayer, x1: pr.x1, y1: pr.y1, x2: pr.x2, y2: pr.y2 }
					: {
						kind: 'arc', net: net.name, layer: bottomLayer,
						cx: pr.cx, cy: pr.cy, r: pr.r, startAngle: pr.startAngle, sweep: pr.sweep,
					});
			}

			const rendered: RenderedPrim[] = [];
			if (endA.stub.length) rendered.push(...endA.stub);
			rendered.push(...bottomPrims);
			if (endB.stub.length) rendered.push(...endB.stub);

			const vias: PlannedVia[] = [];
			if (endA.via) vias.push({ net: net.name, x: endA.via.x, y: endA.via.y, dia: viaDia, hole: viaHole });
			if (endB.via) vias.push({ net: net.name, x: endB.via.x, y: endB.via.y, dia: viaDia, hole: viaHole });

			const mergedRender = mergePrims(rendered);
			const rds = this.netRendered.get(net.name) ?? [];
			rds.push(...mergedRender);
			this.netRendered.set(net.name, rds);
			this.plannedVias.push(...vias);
			this.addRenderedToObstacles(mergedRender);
			for (const v of vias) {
				this.dynamicObstacles.push({
					id: `via${this.obstacleIdCounter++}`,
					kind: 'circle', x: v.x, y: v.y,
					r: viaR + Math.max(drc.trackToTrackMil, drc.trackToPadMil) + drc.traceWidthMil / 2 + 0.1,
					net: net.name, layer: undefined, group: `via:${net.name}:${v.x},${v.y}`, noPrune: true,
				});
			}
			if (!this.routedOrder.includes(net.name)) this.routedOrder.push(net.name);
			// 兜底路径按整网布通处理（与 routedOrder / 布通率口径一致）
			this.callbacks.onNetUpdate?.(
				net.name, mergePrims(rds), this.plannedVias.filter(v => v.net === net.name),
				true, { total: net.pads.length, routed: net.pads.length },
			);
			this.log(`网络 ${net.name} ${vias.length ? '通过过孔换层' : '走底层'}布通`, 'success');
			routed = true;
			break;
		}
		if (!routed) {
			this.log(`过孔兜底 ${net.name}：底层寻路失败`, 'info');
			return false;
		}
		return true;
	}

	/** 撕毁网络：只撤销内存记录 */
	private ripupNet(netName: string): void {
		this.ripupNetInternal(netName, true);
	}

	private ripupNetInternal(netName: string, notify: boolean): void {
		if (notify) this.callbacks.onRipup?.([netName]);
		this.netRendered.delete(netName);
		this.routedEdges.delete(netName);
		this.plannedVias = this.plannedVias.filter(v => v.net !== netName);
		this.dynamicObstacles = this.dynamicObstacles.filter(o => o.net !== netName);
		const ro = this.routedOrder.indexOf(netName);
		if (ro >= 0) this.routedOrder.splice(ro, 1);
	}

	// ── 冲突消除 ─────────────────────────────

	/** 网络的直线下界（MST 各边直线距离之和） */
	private netLowerBound(net: { name: string; pads: EnginePad[] }): number {
		let lb = 0;
		for (const [i, j] of this.computeMST(net.pads)) {
			lb += Math.hypot(net.pads[i].x - net.pads[j].x, net.pads[i].y - net.pads[j].y);
		}
		return lb;
	}

	/**
	 * 「独走最优」：只考虑静态障碍与其它网络焊盘（不考虑其它网络的走线）时的最短路径。
	 * 这是每条连接在当前板子上能达到的真正下界 —— 直线被焊盘挡住时，
	 * 绕开该焊盘的最小绕行量就在这里给出。
	 */
	private async soloRoute(net: { name: string; pads: EnginePad[] }): Promise<{ prims: RBPrimitive[]; length: number } | null> {
		const prims: RBPrimitive[] = [];
		let length = 0;
		for (const [si, di] of this.computeMST(net.pads)) {
			const src = net.pads[si], dst = net.pads[di];
			const layer = getRoutingLayer(src, dst);
			const soloObs = (ly: number) => this.layerObstacles(net.name, ly, false);
			const attempt = async (ly: number) =>
				await this.directEdgeRoute(src, dst, net.name, ly, soloObs(ly)) ??
				await this.candidateEdgeRoute(src, dst, net.name, ly, soloObs(ly));
			let r = await attempt(layer);
			// 默认层走不通且两端都能落底层时，按底层评估独走下界
			if (!r && layer === 1 && this.hasBottomLayer && bottomCapable(src) && bottomCapable(dst)) {
				r = await attempt(2);
			}
			if (!r) return null;
			for (const p of r) {
				length += p.kind === 'line'
					? Math.hypot(p.x2! - p.x1!, p.y2! - p.y1!)
					: Math.abs(p.sweep!) * p.r!;
			}
			prims.push(...r as unknown as RBPrimitive[]);
		}
		return { prims, length };
	}

	/** 图元采样成点列（用于近似距离判定） */
	private primPoints(prims: RenderedPrim[], step = 3): Array<{ x: number; y: number }> {
		const pts: Array<{ x: number; y: number }> = [];
		for (const p of prims) {
			if (p.kind === 'line') {
				const d = Math.hypot(p.x2! - p.x1!, p.y2! - p.y1!);
				const n = Math.max(1, Math.ceil(d / step));
				for (let i = 0; i <= n; i++) {
					const t = i / n;
					pts.push({ x: p.x1! + (p.x2! - p.x1!) * t, y: p.y1! + (p.y2! - p.y1!) * t });
				}
			} else {
				const n = Math.max(2, Math.ceil(Math.abs(p.sweep!) * p.r! / step));
				for (let i = 0; i <= n; i++) {
					const a = p.startAngle! + (p.sweep! * i) / n;
					pts.push({ x: p.cx! + p.r! * Math.cos(a), y: p.cy! + p.r! * Math.sin(a) });
				}
			}
		}
		return pts;
	}

	/** 找出挡在 path 前面的其它网络（走线靠得太近即为阻挡） */
	private findBlockers(path: RBPrimitive[], excludeNet: string, layer: number): string[] {
		const drc = this.drc;
		const clear = drc.trackToTrackMil + drc.traceWidthMil;   // 两根走线中心线最小间距
		const pathPts = this.primPoints(path.map(p => ({ ...p } as any)) as RenderedPrim[]);
		const out: string[] = [];
		for (const name of this.routedOrder) {
			if (name === excludeNet) continue;
			const trks = (this.netRendered.get(name) ?? []).filter(t => t.layer === layer);
			const vias = this.plannedVias.filter(v => v.net === name).map(v => ({ x: v.x, y: v.y }));
			if (!trks.length && !vias.length) continue;
			const trkPts = [...this.primPoints(trks), ...vias];
			let hit = false;
			for (const a of pathPts) {
				for (const b of trkPts) {
					if (Math.hypot(a.x - b.x, a.y - b.y) < clear) { hit = true; break; }
				}
				if (hit) break;
			}
			if (hit) out.push(name);
		}
		return out;
	}

	/**
	 * 冲突消除阶段。
	 *
	 * 单网的最短路一旦被别的网络挡住，最短路就会退化成绕远路；而这条最短通道
	 * 往往只能被一条网络占用（窄走廊）。典型场景（离线基准实测）：
	 *   $H1_6 需要从 $H1_7 焊盘下方穿过，$H1_5/$H1_4 先走了并占了这条通道；
	 *   $H2_4 需要从铺铜左端穿过，$H2_5 先走了并占了这条通道。
	 * 逐网独立求解无法解决 —— 必须把「占道的网络」和「需要通道的网络」一起重排。
	 *
	 * 做法：
	 *  1. 找出实际长度明显超过直线下界的网络（可能绕远）
	 *  2. 计算它的「独走最优」；若实际并不比独走最优差 ⇒ 绕远是几何必然，跳过
	 *  3. 找出挡住独走通道的其它网络
	 *  4. 撕掉这一组，按「独走最优长度升序」（最受限的先走）重布
	 *  5. 组内总长变短才接受，否则整组回滚
	 */
	private async resolveContention(networks: Array<{ name: string; pads: EnginePad[] }>): Promise<void> {
		const maxIter = Math.max(0, Math.round(this.planOptions.ripupRounds ?? 3)) + 2;
		const soloCache = new Map<string, { prims: RBPrimitive[]; length: number } | null>();
		const soloOf = async (name: string) => {
			if (soloCache.has(name)) return soloCache.get(name)!;
			const net = networks.find(n => n.name === name);
			const v = net ? await this.soloRoute(net) : null;
			soloCache.set(name, v);
			return v;
		};

		const examined = new Set<string>();
		for (let iter = 0; iter < maxIter; iter++) {
			if (this.checkCancelled()) break;
			// 每轮迭代都要看预算：单轮 = 独走求解 + 整组两次重排，代价大，
			// 只在阶段入口查一次会让收尾阶段把时间预算顶穿几十分钟
			if (Date.now() > this.planDeadline - 5_000) {
				this.log('冲突消除：时间预算用尽，提前收尾', 'warn');
				break;
			}

			// 1. 候选：绕路倍率最高的网络
			let worst: { name: string; gap: number } | null = null;
			for (const name of this.routedOrder) {
				if (examined.has(name)) continue;
				const net = networks.find(n => n.name === name);
				if (!net) continue;
				const len = renderedLength(this.netRendered.get(name) ?? []);
				const lb = this.netLowerBound(net);
				if (lb <= 0 || len < lb * 1.08) continue;
				const gap = len / lb;
				if (!worst || gap > worst.gap) worst = { name, gap };
			}
			if (!worst) break;
			examined.add(worst.name);

			// 2. 独走最优：绕远若已接近下界 ⇒ 几何必然，跳过
			const solo = await soloOf(worst.name);
			if (!solo) break;
			const actual = renderedLength(this.netRendered.get(worst.name) ?? []);
			if (actual <= solo.length + Math.max(20, solo.length * 0.05)) continue;

			// 3. 阻挡者
			const worstNet = networks.find(n => n.name === worst!.name)!;
			const layer = getRoutingLayer(worstNet.pads[0], worstNet.pads[worstNet.pads.length - 1]);
			const blockers = this.findBlockers(solo.prims, worst.name, layer);
			if (!blockers.length) continue;
			const group = [worst.name, ...blockers];

			// 4. 整组撕毁重排（最受限的先走：独走长度升序）
			const soloLens = new Map<string, number>();
			for (const n of group) {
				const s = n === worst.name ? solo : await soloOf(n);
				soloLens.set(n, s ? s.length : Infinity);
			}
			const savedOrder = [...this.routedOrder];
			const saved = new Map<string, { prims: RenderedPrim[]; vias: PlannedVia[] }>();
			let savedTotal = 0;
			for (const n of group) {
				const prims = [...(this.netRendered.get(n) ?? [])];
				saved.set(n, { prims, vias: this.plannedVias.filter(v => v.net === n) });
				savedTotal += renderedLength(prims);
			}
			for (const n of group) this.ripupNetInternal(n, false);

			const prevOptions = this.planOptions;
			this.planOptions = { ...prevOptions, connectivityFirst: false, optimize: false };

			// 两种排布都试：最受限的先走（独走长度升序）/ 最长的先走（降序）。
			// 注意：第二次尝试会撕掉第一次的成果，引擎状态停在最后一次尝试上；
			// 若最优排布不是最后一次，必须按最优排布重布一遍恢复状态，
			// 否则提交的是失败半成品（组内网络凭空丢失）。
			const asc = [...group].sort((a, b) => (soloLens.get(a) ?? 0) - (soloLens.get(b) ?? 0));
			const desc = [...group].sort((a, b) => (soloLens.get(b) ?? 0) - (soloLens.get(a) ?? 0));
			let bestOrder: string[] | null = null;
			let lastOrder: string[] | null = null;
			let newTotal = Infinity;
			let bestViaCount = Infinity;
			for (const ordered of [asc, desc]) {
				lastOrder = ordered;
				for (const n of group) this.ripupNetInternal(n, false);
				let good = true;
				for (const n of ordered) {
					const nn = networks.find(x => x.name === n);
					if (!nn) { good = false; break; }
					let r = await this.routeNetEdges(nn);
					// 顶层实在走不通 → 允许换层兜底（底层往往空着）
					if (!r) r = await this.tryViaFallback(nn);
					if (!r) { good = false; break; }
				}
				const t = good ? group.reduce((a, n) => a + renderedLength(this.netRendered.get(n) ?? []), 0) : Infinity;
				const vc = good ? group.reduce((a, n) => a + this.plannedVias.filter(v => v.net === n).length, 0) : Infinity;
				if (good && (t < newTotal || (t === newTotal && vc < bestViaCount))) {
					newTotal = t; bestViaCount = vc; bestOrder = ordered;
				}
			}
			this.planOptions = prevOptions;

			const rerunOrder = async (ordered: string[]): Promise<boolean> => {
				for (const n of group) this.ripupNetInternal(n, false);
				for (const n of ordered) {
					const nn = networks.find(x => x.name === n);
					if (!nn) return false;
					let r = await this.routeNetEdges(nn);
					if (!r) r = await this.tryViaFallback(nn);
					if (!r) return false;
				}
				return true;
			};

			const newLens = new Map(group.map(n => [n, renderedLength(this.netRendered.get(n) ?? [])]));
			// 接受条件从严：
			//  - 总长须显著缩短（≥2% 且 ≥8mil）：几 mil 的微优化不值得翻转整束拓扑，
			//    重排的混乱会把漂亮的阶梯束搅成上下翻飞；
			//  - 换层次数不得增加：重排尝试里开了过孔兜底，若不拦，原本单层
			//    走通的网络会在重排失败后悄悄被打上过孔。
			const savedViaCount = group.reduce((a, n) => a + (saved.get(n)?.vias.length ?? 0), 0);
			if (bestOrder && newTotal < savedTotal - Math.max(8, savedTotal * 0.02) && bestViaCount <= savedViaCount &&
				(bestOrder === lastOrder || await rerunOrder(bestOrder)) &&
				group.every(n => (this.netRendered.get(n) ?? []).length > 0)) {
				this.log(`冲突消除：${worst.name} 组（${group.join(', ')}）总长 ${savedTotal.toFixed(0)} → ${newTotal.toFixed(0)} mil`, 'success');
				soloCache.clear();
			} else {
				// 回滚：恢复整组原走线
				for (const n of group) this.ripupNetInternal(n, false);
				for (const n of group) {
					const sv = saved.get(n)!;
					if (sv.prims.length) {
						this.netRendered.set(n, [...sv.prims]);
						this.addRenderedToObstacles(mergePrims(sv.prims));
					}
					this.plannedVias = this.plannedVias.filter(v => v.net !== n).concat(sv.vias);
					// 原走线是完整布通的：同步恢复「已完成连接」记录，避免后续撕毁
					// 重布时把已有边当成未布线又叠一遍。
					const nn = networks.find(x => x.name === n);
					if (nn) this.markAllEdgesDone(nn);
				}
				for (const v of this.plannedVias) {
					if (!group.includes(v.net)) continue;   // 只补组内网络：组外过孔的障碍从未被撕掉
					this.dynamicObstacles.push({
						id: `via${this.obstacleIdCounter++}`, kind: 'circle',
						x: v.x, y: v.y, r: v.dia / 2 + Math.max(this.drc.trackToTrackMil, this.drc.trackToPadMil) + this.drc.traceWidthMil / 2 + 0.1,
						net: v.net, layer: undefined, group: `via:${v.net}:${v.x},${v.y}`, noPrune: true,
					});
				}
				this.routedOrder = savedOrder;
				const detail = group.map(n => `${n}:独走${(soloLens.get(n) ?? 0).toFixed(0)}/原${renderedLength(saved.get(n)!.prims).toFixed(0)}/新${(newLens.get(n) ?? 0).toFixed(0)}`).join(' ');
				this.log(`冲突消除：${worst.name} 组重排无改善（${newTotal.toFixed(0)} vs ${savedTotal.toFixed(0)} mil）[${detail}]`, 'info');
				// 尝试期间各网络都实时写过中间结果（成功的尝试写入了新排布，
				// 失败的尝试留下了残缺画面），回滚后把原走线写回去保持一致
				for (const n of group) {
					const sv = saved.get(n)!;
					const nn = networks.find(x => x.name === n);
					this.callbacks.onNetUpdate?.(
						n, mergePrims(sv.prims), sv.vias, sv.prims.length > 0,
						nn ? this.padProgress(nn) : undefined,
					);
				}
				soloCache.clear();
			}
		}
	}

	/**
	 * 最终优化：逐个网络拆除后重布（中心到中心最短路）。
	 * 只有重布成功且总长不增加才替换；否则恢复首轮走线，
	 * 绝不降低连通率、也绝不增加走线长度。
	 */
	private async optimizeRoutes(networks: Array<{ name: string; pads: EnginePad[] }>): Promise<void> {
		const names = [...this.routedOrder];
		const prevOptions = this.planOptions;
		for (let i = 0; i < names.length; i++) {
			if (this.checkCancelled()) break;
			// 预算见底就停：后续的过孔兜底（连通率关键）还要留时间
			if (Date.now() > this.planDeadline - 5_000) {
				this.log('最终优化：时间预算用尽，提前收尾', 'warn');
				break;
			}
			const name = names[i];
			if (!this.routedOrder.includes(name)) continue;
			const net = networks.find(n => n.name === name);
			if (!net) continue;

			this.progress(80 + Math.floor((i / Math.max(1, names.length)) * 18), `优化网络 ${i + 1}/${names.length}: ${name}`);
			const savedPrims = [...(this.netRendered.get(name) ?? [])];
			const savedVias = this.plannedVias.filter(v => v.net === name);
			const savedLen = renderedLength(savedPrims);

			this.ripupNetInternal(name, false);
			this.planOptions = { ...prevOptions, connectivityFirst: false, optimize: false };
			const ok = await this.routeNetEdges(net);
			this.planOptions = prevOptions;

			const newLen = ok ? renderedLength(this.netRendered.get(name) ?? []) : Infinity;
			const newViaCount = ok ? this.plannedVias.filter(v => v.net === name).length : Infinity;
			// 采纳条件从严：显著更短（≥2% 且 ≥8mil）才换，且不得新增换层。
			// 几 mil 的微优化不值得翻转束内拓扑（近等长的备选往往上下翻飞），
			// 更不允许重布把原本单层走通的网络逼出过孔。
			if (!ok || newLen > savedLen - Math.max(8, savedLen * 0.02) || newViaCount > savedVias.length) {
				// 撤掉重布尝试注册的动态障碍（尝试失败时本就没注册，过滤是无害空操作），
				// 否则被否决的走线会以"幽灵障碍"的身份挤占后续网络的布线空间。
				this.dynamicObstacles = this.dynamicObstacles.filter(o => o.net !== name);
				this.netRendered.set(name, savedPrims);
				this.plannedVias = this.plannedVias.filter(v => v.net !== name).concat(savedVias);
				this.addRenderedToObstacles(mergePrims(savedPrims));
				this.markAllEdgesDone(net);
				// 撕网时该网过孔的动态障碍一并被移除，回滚需重新注册
				for (const v of savedVias) {
					this.dynamicObstacles.push({
						id: `via${this.obstacleIdCounter++}`, kind: 'circle', x: v.x, y: v.y,
						r: v.dia / 2 + Math.max(this.drc.trackToTrackMil, this.drc.trackToPadMil) + this.drc.traceWidthMil / 2 + 0.1,
						net: v.net, layer: undefined, group: `via:${v.net}:${v.x},${v.y}`, noPrune: true,
					});
				}
				if (!this.routedOrder.includes(name)) this.routedOrder.push(name);
				this.log(`网络 ${name} 优化未采纳（${ok ? newLen.toFixed(0) : '失败'} vs ${savedLen.toFixed(0)} mil），保留原走线`, 'info');
				// 画布还停留在刚被否决的重布结果上（routeNetEdges 已实时写入）：
				// 把原走线写回去，画布与引擎状态保持一致
				this.callbacks.onNetUpdate?.(name, mergePrims(savedPrims), savedVias, true, this.padProgress(net));
			}
		}
		this.progress(98, '优化完成');
	}
}
