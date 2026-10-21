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
	pointSegDist, padExitModel, padShapeToCircles,
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
	/** 网络排序：river/ reverse / max-x / longest / shortest */
	netOrder?: 'river' | 'reverse' | 'max-x' | 'longest' | 'shortest';
	/** 失败网络允许过孔换层（底层布线）兜底 */
	allowViaFallback?: boolean;
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

export interface PlanningCallbacks {
	onLog?: (message: string, level: string) => void;
	onProgress?: (percentage: number, message: string) => void;
	/** 某个网络完成（或重新完成）规划：主线程据此实时写入画布 */
	onNetUpdate?: (name: string, prims: RenderedPrim[], vias: PlannedVia[], ok: boolean) => void;
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
	/** 已布网络名（按布线顺序） */
	routedNets: string[];
	/** 规划日志（Worker 模式下由主线程读取） */
	logs: Array<{ level: string; message: string }>;
}

const CLR_EPS = 0.05;

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
	/** 每网络的规划图元 */
	private netRendered = new Map<string, RenderedPrim[]>();
	/** 已布网络顺序 */
	private routedOrder: string[] = [];
	/** 规划的过孔 */
	private plannedVias: PlannedVia[] = [];
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

	/** 执行完整规划：排序 → 逐网 → 撕毁重布 → 成组重布 */
	async plan(input: PlanningInput, callbacks: PlanningCallbacks = {}): Promise<PlanningResult> {
		this.drc = input.drc;
		this.allPads = input.allPads;
		this.staticObstacles = input.staticObstacles;
		this.padShapes = input.padShapes || {};
		this.callbacks = callbacks;
		this.planOptions = input.options ?? {};
		this.copperLayers = Math.max(1, Math.round(input.copperLayers ?? 2));
		this.dynamicObstacles = [];
		this.obstacleIdCounter = 0;
		this.netRendered.clear();
		this.routedOrder = [];
		this.plannedVias = [];
		this.logs = [];
		this.cancelled = false;

		const budgetMs = input.budgetMs ?? 0;
		const deadline = budgetMs > 0 ? Date.now() + budgetMs : Number.POSITIVE_INFINITY;

		// 网络排序（电源优先，其余按 netOrder 指定的策略）
		const networks = this.sortNetworks(input.networks);

		this.log(`网络顺序（${this.planOptions.netOrder ?? 'longest'}）：${networks.map(n => n.name).join(' → ')}`, 'info');
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
			const ok = await this.routeNetEdges(net);
			if (!ok) pendingNets.push(net);
		}

		// 多轮撕毁重布
		const ripupRounds = Math.max(0, Math.round(this.planOptions.ripupRounds ?? 2));
		for (let pass = 0; pass < ripupRounds && pendingNets.length; pass++) {
			if (this.checkCancelled()) break;
			if (Date.now() > deadline) {
				this.log('规划时间预算用尽，停止撕毁重布', 'warn');
				break;
			}
			const next: typeof pendingNets = [];
			for (const net of pendingNets) {
				if (this.checkCancelled()) break;
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
				this.log(`第 ${pass + 1} 轮：网络 ${net.name} 撕毁 ${rip.join(', ')} 重试`, 'warn');
				for (const n of rip) this.ripupNet(n);
				const ok = await this.routeNetEdges(net);
				for (let i = rip.length - 1; i >= 0; i--) {
					const ripped = networks.find(nn => nn.name === rip[i])!;
					const rok = await this.routeNetEdges(ripped);
					if (!rok) next.push(ripped);
				}
				if (!ok) next.push(net);
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
				for (const n of rip) this.ripupNet(n);
				const ordered = [...pendingNets].sort((na, nb) => {
					const xa = Math.max(...na.pads.map(p => p.x));
					const xb = Math.max(...nb.pads.map(p => p.x));
					return xa - xb;   // 内向优先：内侧网络先占天然走廊，外侧网络向外嵌套
				});
				for (const net of ordered) {
					if (!(await this.routeNetEdges(net))) this.log(`成组重布：${net.name} 仍失败`, 'error');
				}
				for (const n of rip) {
					const ripped = networks.find(nn => nn.name === n)!;
					if (!(await this.routeNetEdges(ripped))) this.log(`成组重布：重布 ${n} 失败`, 'error');
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
			const stillFailed = networks.filter(n => !this.routedOrder.includes(n.name));
			for (const net of stillFailed) {
				if (this.checkCancelled()) break;
				await this.tryViaFallback(net);
			}
		}

		const failed = networks.filter(n => !this.routedOrder.includes(n.name)).map(n => n.name);
		return {
			nets: this.routedOrder.map(name => ({
				name,
				prims: mergePrims(this.netRendered.get(name) ?? []),
			})),
			vias: [...this.plannedVias],
			failed,
			routedNets: [...this.routedOrder],
			logs: [...this.logs],
		};
	}

	cancel(): void { this.cancelled = true; }

	// ── 内部实现 ─────────────────────────────

	/**
	 * 网络排序：布线顺序直接决定绕路总量。
	 *
	 * 关键结论（离线基准 tmp/quality-test.ts 实测）：
	 *  一组「扇形」网络中，各条理想路径互不相交（源点在一条边上、目标点在
	 *  另一条边上，且顺序一致）。若按「从内到外」布线，先布的内层网络会
	 *  横在外层网络的理想路径上，把外层网络逼到绕远路 —— 这是绕路的主因。
	 *  因此默认按「连接总长降序」布线：最长的连接约束最多，先给它让出通道。
	 */
	private sortNetworks(nets: Array<{ name: string; pads: EnginePad[] }>): Array<{ name: string; pads: EnginePad[] }> {
		const mode = this.planOptions.netOrder ?? 'longest';
		const span = (n: { name: string; pads: EnginePad[] }) => {
			const edges = this.computeMST(n.pads);
			let total = 0;
			for (const [i, j] of edges) total += Math.hypot(n.pads[i].x - n.pads[j].x, n.pads[i].y - n.pads[j].y);
			return total;
		};
		const keyed = nets.map(n => {
			const xs = n.pads.map(p => p.x);
			return {
				net: n,
				minX: Math.min(...xs),
				maxX: Math.max(...xs),
				len: span(n),
			};
		});
		keyed.sort((a, b) => {
			if (this.planOptions.powerNetsFirst !== false) {
				const isPowerA = this.isPowerNet(a.net.name), isPowerB = this.isPowerNet(b.net.name);
				if (isPowerA && !isPowerB) return -1;
				if (!isPowerA && isPowerB) return 1;
			}
			switch (mode) {
				case 'longest': return b.len - a.len;
				case 'shortest': return a.len - b.len;
				case 'max-x': return b.maxX - a.maxX;
				case 'reverse': return b.maxX - a.maxX || b.minX - a.minX;
				case 'river':
				default: return a.minX - b.minX || a.maxX - b.maxX;
			}
		});
		return keyed.map(k => k.net);
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
	private padObstacles(pads: EnginePad[], excludeNet: string): RBObstacle[] {
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
			};
			for (const c of padShapeToCircles(shape)) {
				out.push({ id: `p${id++}`, kind: 'circle', x: p.x + c.x, y: p.y + c.y, r: c.r + infl, net: p.net, layer, group });
			}
		}
		return out;
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
				// 弧：两端可绕行 + 沿弧加密阻挡圆（带弦→弧外凸补偿）
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
					for (const c of capsuleToCircles(px, py, nx, ny, rr)) {
						this.dynamicObstacles.push({ id: `db${this.obstacleIdCounter++}`, kind: 'circle', x: c.x, y: c.y, r: rr, net: pr.net, layer: pr.layer, group, blockerOnly: true });
					}
					px = nx; py = ny;
				}
				this.dynamicObstacles.push({ id: `dw${this.obstacleIdCounter++}`, kind: 'circle', x: px, y: py, r, net: pr.net, layer: pr.layer, group });
			}
		}
	}

	/** 布一个网络的全部 MST 连接（纯规划） */
	private async routeNetEdges(net: { name: string; pads: EnginePad[] }): Promise<boolean> {
		const drc = this.drc;
		const edges = this.computeMST(net.pads);
		const netOk = true;
		// 本次尝试的图元/过孔先暂存：整网成功才提交为动态障碍并通知主线程，
		// 失败尝试不得留下"幽灵障碍"影响后续网络。
		const attemptRendered: RenderedPrim[] = [];
		const attemptVias: PlannedVia[] = [];
		if (!this.routedOrder.includes(net.name)) {
			this.plannedVias = this.plannedVias.filter(v => v.net !== net.name);
		}

		for (const [si, di] of edges) {
			if (this.checkCancelled()) return false;
			const src = net.pads[si], dst = net.pads[di];
			const routingLayer = getRoutingLayer(src, dst);

			// 单层优先，两级退避：
			// ① 焊盘中心 → 可视图最短路 → 焊盘中心（首选，出线方向即最短路起始
			//    方向，除绕障必需的圆弧外整条走线没有人为折角 —— 橡皮筋拉紧效果）
			// ② 密集焊盘排里邻焊盘的外扩间距圈会吞没本焊盘中心（间距 < 两焊盘
			//    半径+外扩之和），中心起点的任何线段都判侵入 —— 退回焊盘边界
			//    出线候选（引出段 + 切线圆角）
			let rendered = await this.directEdgeRoute(src, dst, net.name, routingLayer) ??
				await this.candidateEdgeRoute(src, dst, net.name, routingLayer);

			// 顶层绕不过去 → 换到底层再试（TH/底层焊盘无需过孔即可落到底层）
			if (!rendered && routingLayer === 1 && this.hasBottomLayer &&
				bottomCapable(src) && bottomCapable(dst)) {
				rendered = await this.directEdgeRoute(src, dst, net.name, 2) ??
					await this.candidateEdgeRoute(src, dst, net.name, 2);
			}

			// 两个单层都走不通 → 才用过孔换层兜底
			const edgeVias: PlannedVia[] = [];
			if (!rendered && this.hasBottomLayer) {
				const viaCand = await this.viaEdgeCandidate(src, dst, net.name);
				if (viaCand) {
					rendered = viaCand.prims;
					edgeVias.push(...viaCand.vias);
				}
			}

			if (!rendered) {
				this.log(`网络 ${net.name}: ${src.id}(${src.x.toFixed(0)},${src.y.toFixed(0)}) → ${dst.id}(${dst.x.toFixed(0)},${dst.y.toFixed(0)}) 未找到路径`, 'warn');
				return false;
			}

			// 跨层 SMD 对（顶层 ↔ 底层）：路径落在其中一层的焊盘中心，
			// 另一层焊盘靠焊盘内过孔连接
			const srcLayer = src.layer === 12 ? routingLayer : src.layer;
			const dstLayer = dst.layer === 12 ? routingLayer : dst.layer;
			if (srcLayer !== dstLayer) {
				edgeVias.push({
					net: net.name, x: dst.x, y: dst.y,
					dia: Math.max(20, drc.traceWidthMil * 2),
					hole: Math.max(10, drc.traceWidthMil),
				});
			}

			attemptRendered.push(...rendered);
			attemptVias.push(...edgeVias);
		}

		// 整网成功：提交图元、过孔与动态障碍
		if (netOk) {
			const rds = this.netRendered.get(net.name) ?? [];
			rds.push(...attemptRendered);
			this.netRendered.set(net.name, rds);
			this.addRenderedToObstacles(attemptRendered);
			this.plannedVias = this.plannedVias.filter(v => v.net !== net.name).concat(attemptVias);
			// 过孔是贯通各层的孔洞：其它网络（任何层）都要给它让位
			for (const v of attemptVias) {
				this.dynamicObstacles.push({
					id: `via${this.obstacleIdCounter++}`, kind: 'circle', x: v.x, y: v.y,
					r: v.dia / 2 + Math.max(this.drc.trackToTrackMil, this.drc.trackToPadMil) + this.drc.traceWidthMil / 2 + 0.1,
					net: net.name, layer: undefined, group: `via:${net.name}:${v.x},${v.y}`, noPrune: true,
				});
			}
			if (!this.routedOrder.includes(net.name)) this.routedOrder.push(net.name);
		}

		// 实时结果：主线程收到后立即写入/替换该网络的画布图元
		const prims = mergePrims(this.netRendered.get(net.name) ?? []);
		this.callbacks.onNetUpdate?.(net.name, prims, this.plannedVias.filter(v => v.net === net.name), netOk);
		this.log(
			netOk
				? `网络 ${net.name} 布线完成（${edges.length} 条连接，${prims.length} 段图形）`
				: `网络 ${net.name} 部分连接失败（${edges.length} 条连接）`,
			netOk ? 'success' : 'warn',
		);
		return netOk;
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
			},
		);
		if (!edge.ok || !edge.prims.length) return null;
		return edge.prims.map(pr => pr.kind === 'line'
			? { kind: 'line' as const, net: netName, layer, x1: pr.x1, y1: pr.y1, x2: pr.x2, y2: pr.y2 }
			: { kind: 'arc' as const, net: netName, layer, cx: pr.cx, cy: pr.cy, r: pr.r, startAngle: pr.startAngle, sweep: pr.sweep });
	}

	/**
	 * 「过孔换层」候选走线（单条连接，兜底）：
	 *   顶层引出段 → 过孔 → 底层路径 →（过孔）→ 顶层引出段
	 *
	 * 只在两个单层（顶层、底层直连）都走不通时才使用 —— 优先单层布线，
	 * 能不换层就不换层。顶层被先布的网络占满时，这是最后的通路。
	 */
	private async viaEdgeCandidate(
		src: EnginePad, dst: EnginePad, netName: string,
	): Promise<{ prims: RenderedPrim[]; vias: PlannedVia[]; length: number } | null> {
		if (!this.hasBottomLayer) return null;
		const drc = this.drc;
		const topLayer = 1, bottomLayer = 2;
		const viaDia = Math.max(24, drc.traceWidthMil * 2);
		const viaHole = Math.max(12, drc.traceWidthMil);
		const viaR = viaDia / 2;
		// 过孔铜环相对走线半宽多出的部分
		const viaExtra = viaR - drc.traceWidthMil / 2;

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

		// 引出段清障：与 routePadEdge 同规则（侵入段完全在自家焊盘铜皮内则可接受）
		const segClear = (pad: EnginePad, x2: number, y2: number, obs: RBObstacle[]): boolean =>
			stubFeasible(this.padTerminal(pad), x2, y2, obs, netName);
		const viaClear = (x: number, y: number, obs: RBObstacle[]): boolean => {
			for (const o of obs) {
				if (o.net === netName) continue;
				// 障碍圆半径已含「对方走线半宽 + 间距」，过孔比走线粗，多出的部分要补上；
				// EDA 的 Track↔Via 规则按焊盘间距执行（大于走线间距）
				if (Math.hypot(x - o.x, y - o.y) <
					o.r + viaExtra + Math.max(0, drc.trackToPadMil - drc.trackToTrackMil) + 0.1) return false;
			}
			return true;
		};

		interface Anchor {
			x: number; y: number;
			stub: RenderedPrim | null;
			via: { x: number; y: number } | null;
			stubLen: number;
		}

		// 每个端点的落点候选：
		//  - 通孔 / 底层焊盘：直接在底层出图，无需过孔
		//  - 顶层 SMD：沿出线候选方向外扩，找一个既满足顶层引出段、
		//    又满足过孔间距（顶层 + 底层）的位置
		const endAnchors = (pad: EnginePad, other: EnginePad): Anchor[] => {
			if (pad.layer === 12 || pad.layer === bottomLayer) {
				return [{ x: pad.x, y: pad.y, stub: null, via: null, stubLen: 0 }];
			}
			if (pad.layer !== topLayer) return [];
			const term = this.padTerminal(pad);
			const cands = buildExitCandidates(term, other, this.escapeMargin);
			const out: Anchor[] = [];
			for (const c of cands) {
				if (!segClear(pad, c.x, c.y, topObs)) continue;
				// 步进量从 10mil 起：过孔钻孔（r≈6mil）不要落在焊盘铜皮上
				for (const step of [10, 20, 35, 50, 70]) {
					const x = c.x + c.dirX * step, y = c.y + c.dirY * step;
					if (!segClear(pad, x, y, topObs)) continue;
					if (!viaClear(x, y, topObs) || !viaClear(x, y, bottomObs)) continue;
					out.push({
						x, y,
						stub: { kind: 'line', net: netName, layer: topLayer, x1: pad.x, y1: pad.y, x2: x, y2: y },
						via: { x, y },
						stubLen: Math.hypot(x - pad.x, y - pad.y),
					});
					break;   // 同一方向只保留最近的可行过孔
				}
			}
			return out;
		};

		const anchorsA = endAnchors(src, dst);
		const anchorsB = endAnchors(dst, src);
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
				o.net !== netName &&
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
			const bp = await bottomFor(a, b);
			if (bp) { pickA = a; pickB = b; bottomPrims = bp; break; }
		}
		if (!pickA || !pickB || !bottomPrims) return null;

		const prims: RenderedPrim[] = [];
		if (pickA.stub) prims.push(pickA.stub);
		prims.push(...bottomPrims);
		if (pickB.stub) prims.push(pickB.stub);
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
				if (o.net === net.name) continue;
				// EDA 的 Track↔Via 规则按焊盘间距执行（大于走线间距）
				if (Math.hypot(x - o.x, y - o.y) <
					o.r + viaExtra + Math.max(0, drc.trackToPadMil - drc.trackToTrackMil) + 0.1) return false;
			}
			return true;
		};
		// 引出段清障：与 routePadEdge 同规则（侵入段完全在自家焊盘铜皮内则可接受）
		const segClear = (pad: EnginePad, x2: number, y2: number, obs: RBObstacle[]): boolean =>
			stubFeasible(this.padTerminal(pad), x2, y2, obs, net.name);

		interface EndSetup {
			via: { x: number; y: number } | null;
			stub: RenderedPrim | null;
			bottom: { x: number; y: number };
		}

		// 顶层 SMD 焊盘需要引出段+过孔；TH / 底层焊盘可直接从焊盘中心走底层
		const setupEnd = (pad: EnginePad, other: EnginePad): EndSetup | null => {
			if (pad.layer === 12 || pad.layer === bottomLayer) {
				return { via: null, stub: null, bottom: { x: pad.x, y: pad.y } };
			}
			if (pad.layer !== topLayer) return null;
			const term = this.padTerminal(pad);
			const cands = buildExitCandidates(term, other, this.escapeMargin);
			for (const c of cands) {
				if (!segClear(pad, c.x, c.y, topObs)) continue;
				// 步进量从 10mil 起：过孔钻孔（r≈6mil）不要落在焊盘铜皮上
				for (const step of [10, 20, 35, 50, 70]) {
					const x = c.x + c.dirX * step, y = c.y + c.dirY * step;
					if (!segClear(pad, x, y, topObs)) continue;
					if (viaClear(x, y, topObs) && viaClear(x, y, bottomObs)) {
						return {
							via: { x, y },
							stub: { kind: 'line', net: net.name, layer: topLayer, x1: pad.x, y1: pad.y, x2: x, y2: y },
							bottom: { x, y },
						};
					}
				}
			}
			return null;
		};

		const endA = setupEnd(a, b);
		const endB = setupEnd(b, a);
		if (!endA || !endB) {
			this.log(`过孔兜底 ${net.name}：顶层出线或过孔位置不可用`, 'info');
			return false;
		}

		// 底层优先直连（任意角度最短线），被挡时才跑可视图
		const bottomBlocked = bottomObs.some(o =>
			o.net !== net.name &&
			pointSegDist(o.x, o.y, endA.bottom.x, endA.bottom.y, endB.bottom.x, endB.bottom.y) < o.r - 0.05);
		let bottomPrims: RenderedPrim[];
		if (!bottomBlocked) {
			bottomPrims = [{
				kind: 'line', net: net.name, layer: bottomLayer,
				x1: endA.bottom.x, y1: endA.bottom.y, x2: endB.bottom.x, y2: endB.bottom.y,
			}];
		} else {
			const bottomRoute = await routeConnection(endA.bottom, endB.bottom, bottomObs, net.name, [150, 500], 3000);
			if (!bottomRoute.ok || bottomRoute.prims.length === 0) {
				this.log(`过孔兜底 ${net.name}：底层寻路失败`, 'info');
				return false;
			}
			bottomPrims = bottomRoute.prims.map(pr => pr.kind === 'line'
				? { kind: 'line', net: net.name, layer: bottomLayer, x1: pr.x1, y1: pr.y1, x2: pr.x2, y2: pr.y2 }
				: {
					kind: 'arc', net: net.name, layer: bottomLayer,
					cx: pr.cx, cy: pr.cy, r: pr.r, startAngle: pr.startAngle, sweep: pr.sweep,
				});
		}

		const rendered: RenderedPrim[] = [];
		if (endA.stub) rendered.push(endA.stub);
		rendered.push(...bottomPrims);
		if (endB.stub) rendered.push(endB.stub);

		const vias: PlannedVia[] = [];
		if (endA.via) vias.push({ net: net.name, x: endA.via.x, y: endA.via.y, dia: viaDia, hole: viaHole });
		if (endB.via) vias.push({ net: net.name, x: endB.via.x, y: endB.via.y, dia: viaDia, hole: viaHole });

		const rds = this.netRendered.get(net.name) ?? [];
		rds.push(...rendered);
		this.netRendered.set(net.name, rds);
		this.plannedVias.push(...vias);
		this.addRenderedToObstacles(rendered);
		for (const v of vias) {
			this.dynamicObstacles.push({
				id: `via${this.obstacleIdCounter++}`,
				kind: 'circle', x: v.x, y: v.y,
				r: viaR + Math.max(drc.trackToTrackMil, drc.trackToPadMil) + drc.traceWidthMil / 2 + 0.1,
				net: net.name, layer: undefined, group: `via:${net.name}:${v.x},${v.y}`, noPrune: true,
			});
		}
		if (!this.routedOrder.includes(net.name)) this.routedOrder.push(net.name);
		this.callbacks.onNetUpdate?.(net.name, mergePrims(rds), this.plannedVias.filter(v => v.net === net.name), true);
		this.log(`网络 ${net.name} ${vias.length ? '通过过孔换层' : '走底层'}布通`, 'success');
		return true;
	}

	/** 撕毁网络：只撤销内存记录 */
	private ripupNet(netName: string): void {
		this.ripupNetInternal(netName, true);
	}

	private ripupNetInternal(netName: string, notify: boolean): void {
		if (notify) this.callbacks.onRipup?.([netName]);
		this.netRendered.delete(netName);
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
		const maxIter = Math.max(0, Math.round(this.planOptions.ripupRounds ?? 2)) + 2;
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
				if (good && t < newTotal) { newTotal = t; bestOrder = ordered; }
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
			if (bestOrder && newTotal < savedTotal - 0.5 &&
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
						this.addRenderedToObstacles(sv.prims);
					}
					this.plannedVias = this.plannedVias.filter(v => v.net !== n).concat(sv.vias);
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
					this.callbacks.onNetUpdate?.(n, mergePrims(sv.prims), sv.vias, sv.prims.length > 0);
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
			if (!ok || newLen > savedLen + 0.5) {
				// 撤掉重布尝试注册的动态障碍（尝试失败时本就没注册，过滤是无害空操作），
				// 否则被否决的走线会以"幽灵障碍"的身份挤占后续网络的布线空间。
				this.dynamicObstacles = this.dynamicObstacles.filter(o => o.net !== name);
				this.netRendered.set(name, savedPrims);
				this.plannedVias = this.plannedVias.filter(v => v.net !== name).concat(savedVias);
				this.addRenderedToObstacles(savedPrims);
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
				this.callbacks.onNetUpdate?.(name, mergePrims(savedPrims), savedVias, true);
			}
		}
		this.progress(98, '优化完成');
	}
}
