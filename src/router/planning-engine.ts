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
	buildPolylineObstacle, buildPolygonObstacle,
	computeEscapePoint, computeEscapeAlternatives, pointSegDist,
	fanoutFillet, padExitModel, padShapeToCircles,
} from './rubberband-engine';
import { routeConnection } from './visibility-router';
import { DRCRules } from './drc-rules';

// ─────────────────────────────────────────────
// 数据类型
// ─────────────────────────────────────────────

/** 布线算法的可调参数 */
export interface PlanOptions {
	/** 扇出圆角半径（mil），0 = 直角 */
	filletRadiusMil?: number;
	/** 焊盘逃逸线长度（mil） */
	escapeClearanceMil?: number;
	/** 撕毁重布轮数 */
	ripupRounds?: number;
	/** 电源网络优先 */
	powerNetsFirst?: boolean;
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

/** 合并同圆相邻弧与共线相邻线段：减少接缝（编辑器坐标取整会造成 0.1mil 级断点） */
export function mergePrims(prims: RenderedPrim[]): RenderedPrim[] {
	const out: RenderedPrim[] = [];
	for (const p of prims) {
		const last = out[out.length - 1];
		if (p.kind === 'arc' && last && last.kind === 'arc' &&
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
		if (p.kind === 'line' && last && last.kind === 'line') {
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
	private escapeReservationCache: RBObstacle[] | null = null;
	private logs: Array<{ level: string; message: string }> = [];
	private cancelled = false;
	private planOptions: PlanOptions = {};

	private get escapeClear(): number {
		if (typeof this.planOptions.escapeClearanceMil === 'number' && this.planOptions.escapeClearanceMil > 0) {
			return this.planOptions.escapeClearanceMil;
		}
		return this.drc.trackToPadMil + this.drc.traceWidthMil + 20;
	}

	private get filletRadius(): number {
		const v = this.planOptions.filletRadiusMil;
		if (typeof v === 'number') return Math.max(0, v);
		return Math.min(40, this.drc.traceWidthMil * 4);
	}

	/** 执行完整规划：排序 → 逐网 → 撕毁重布 → 成组重布 */
	async plan(input: PlanningInput, callbacks: PlanningCallbacks = {}): Promise<PlanningResult> {
		this.drc = input.drc;
		this.allPads = input.allPads;
		this.staticObstacles = input.staticObstacles;
		this.padShapes = input.padShapes || {};
		this.callbacks = callbacks;
		this.planOptions = input.options ?? {};
		this.dynamicObstacles = [];
		this.obstacleIdCounter = 0;
		this.netRendered.clear();
		this.routedOrder = [];
		this.plannedVias = [];
		this.escapeReservationCache = null;
		this.logs = [];
		this.cancelled = false;

		const budgetMs = input.budgetMs ?? 0;
		const deadline = budgetMs > 0 ? Date.now() + budgetMs : Number.POSITIVE_INFINITY;

		// 网络排序（电源优先，其余按几何位置河流式排序）
		const networks = [...input.networks].sort((a, b) => {
			const pa = a.pads, pb = b.pads;
			if (this.planOptions.powerNetsFirst !== false) {
				const isPowerA = this.isPowerNet(a.name), isPowerB = this.isPowerNet(b.name);
				if (isPowerA && !isPowerB) return -1;
				if (!isPowerA && isPowerB) return 1;
			}
			const aMin = Math.min(...pa.map(p => p.x)), aMax = Math.max(...pa.map(p => p.x));
			const bMin = Math.min(...pb.map(p => p.x)), bMax = Math.max(...pb.map(p => p.x));
			if (aMin !== bMin) return aMin - bMin;
			return aMax - bMax;
		});

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

		// 最终成组重布：拆除所有封路网，失败网按外向优先（maxX 降序）重布，再重布被拆网
		if (pendingNets.length && Date.now() <= deadline) {
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
				this.log(`成组重布：拆除 ${rip.join(', ')}`, 'warn');
				for (const n of rip) this.ripupNet(n);
				const ordered = [...pendingNets].sort((na, nb) => {
					const xa = Math.max(...na.pads.map(p => p.x));
					const xb = Math.max(...nb.pads.map(p => p.x));
					return xb - xa;
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

	/** 单焊盘逃逸点：SMD 长焊盘从端帽沿长轴垂直出线；圆形焊盘退回 scored 方向 */
	private padEscape(
		pad: { x: number; y: number; radius: number; layer: number },
		shape: PadShape | undefined,
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
				let ok = true;
				for (const o of obstacles) {
					if (o.net === net) continue;
					if (o.layer !== undefined && o.layer !== routingLayer) continue;
					if (Math.hypot(tipX - o.x, tipY - o.y) < o.r - CLR_EPS) { ok = false; break; }
					if (pointSegDist(o.x, o.y, pad.x, pad.y, tipX, tipY) < o.r - CLR_EPS) { ok = false; break; }
				}
				if (ok) return { x: tipX, y: tipY };
				return null; // 轴向被堵：SMD 长焊盘不做斜出（保持垂直扇出美观）
			}
		}
		return computeEscapePoint(pad.x, pad.y, pad.radius, obstacles, net, routingLayer, escClear, toward);
	}

	/** 逃逸通道预约：每个焊盘的出线段注册为该网络的专属阻挡（其他网络不得横穿） */
	private getEscapeReservations(): RBObstacle[] {
		if (this.escapeReservationCache) return this.escapeReservationCache;
		const out: RBObstacle[] = [];
		const drc = this.drc;
		const rRes = drc.traceWidthMil + drc.trackToTrackMil + 0.1;
		const escClear = this.escapeClear;
		const padObstaclesAll = this.padObstacles(this.allPads, ' none');
		for (const p of this.allPads) {
			const layer = p.layer === 12 ? undefined : p.layer;
			const obs = padObstaclesAll.filter(o => o.layer === undefined || o.layer === layer);
			const shape = this.padShapes[`${p.x.toFixed(2)},${p.y.toFixed(2)}`];
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
		// 失败尝试不得留下“幽灵障碍”影响后续网络。
		const attemptRendered: RenderedPrim[] = [];
		const attemptVias: PlannedVia[] = [];
		if (!this.routedOrder.includes(net.name)) {
			this.plannedVias = this.plannedVias.filter(v => v.net !== net.name);
		}

		for (const [si, di] of edges) {
			if (this.checkCancelled()) return false;
			const src = net.pads[si], dst = net.pads[di];
			const routingLayer = getRoutingLayer(src, dst);

			const layerFiltered = [
				...this.staticObstacles,
				...this.padObstacles(this.allPads, net.name),
				...this.dynamicObstacles,
				...this.getEscapeReservations(),
			].filter(o => o.layer === undefined || o.layer === routingLayer);

			const escClear = this.escapeClear;
			const shapeSrc = this.padShapes[`${src.x.toFixed(2)},${src.y.toFixed(2)}`];
			const shapeDst = this.padShapes[`${dst.x.toFixed(2)},${dst.y.toFixed(2)}`];
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
						if (pointSegDist(o.x, o.y, st.x1!, st.y1!, st.x2!, st.y2!) < o.r - CLR_EPS) {
							stubsOK = false;
							break;
						}
					}
					if (!stubsOK) break;
				}
				if (stubsOK) {
					result = await routeConnection(escSrc, escDst, layerFiltered, net.name, [150, 500]);
				}
				// 首选逃逸方向失败：尝试备选逃逸方向
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
									if (pointSegDist(o.x, o.y, st.x1!, st.y1!, st.x2!, st.y2!) < o.r - CLR_EPS) { ok2 = false; break; }
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
				this.log(`网络 ${net.name}: ${src.id}(${src.x.toFixed(0)},${src.y.toFixed(0)}) → ${dst.id}(${dst.x.toFixed(0)},${dst.y.toFixed(0)}) 未找到路径 (layerFiltered=${layerFiltered.length})`, 'warn');
				return false;
			}

			// 链式顺序：stub1 → 路径各段 → stub2
			const rendered: RenderedPrim[] = [];
			if (stub1) rendered.push(stub1);
			for (const pr of result.prims) {
				if (pr.kind === 'line') {
					rendered.push({ kind: 'line', net: net.name, layer: routingLayer, x1: pr.x1, y1: pr.y1, x2: pr.x2, y2: pr.y2 });
				} else {
					// 微小弧保留（写入时降级为直线），近整圆无法表达
					const deg = pr.sweep * 180 / Math.PI;
					if (Math.abs(Math.abs(deg) - 360) < 0.05) continue;
					rendered.push({ kind: 'arc', net: net.name, layer: routingLayer, cx: pr.cx, cy: pr.cy, r: pr.r, startAngle: pr.startAngle, sweep: pr.sweep });
				}
			}
			if (stub2) rendered.push(stub2);

			// 过孔：跨层时在终点规划一个过孔
			const srcLayer = src.layer === 12 ? routingLayer : src.layer;
			const dstLayer = dst.layer === 12 ? routingLayer : dst.layer;
			if (srcLayer !== dstLayer) {
				attemptVias.push({
					net: net.name, x: dst.x, y: dst.y,
					dia: Math.max(20, drc.traceWidthMil * 2),
					hole: Math.max(10, drc.traceWidthMil),
				});
			}

			// 扇出圆角（纯内存）
			try {
				const R = this.filletRadius;
				const first = rendered.find(r2 => r2.kind === 'line' && r2 !== stub1 && r2 !== stub2) as
					{ kind: 'line'; x1: number; y1: number; x2: number; y2: number } | undefined;
				if (stub1 && first && first.x1 === stub1.x2 && first.y1 === stub1.y2) {
					const f1 = fanoutFillet(
						{ x1: stub1.x1!, y1: stub1.y1!, x2: stub1.x2!, y2: stub1.y2! },
						first, R,
					);
					if (f1) {
						const l0 = f1.prims[0], ar = f1.prims[1], l2 = f1.prims[2];
						if (l0.kind === 'line' && ar.kind === 'arc' && l2.kind === 'line') {
							const fi = rendered.indexOf(first as any);
							const si = rendered.indexOf(stub1 as any);
							if (fi >= 0) rendered[fi] = { kind: 'line', net: net.name, layer: routingLayer, x1: l2.x1, y1: l2.y1, x2: l2.x2, y2: l2.y2 };
							if (si >= 0) {
								rendered[si] = { kind: 'line', net: net.name, layer: routingLayer, x1: l0.x1, y1: l0.y1, x2: l0.x2, y2: l0.y2 };
								rendered.splice(si + 1, 0, { kind: 'arc', net: net.name, layer: routingLayer, cx: ar.cx, cy: ar.cy, r: ar.r, startAngle: ar.startAngle, sweep: ar.sweep });
							}
						}
					}
				}
				const s2idx = stub2 ? rendered.indexOf(stub2 as any) : -1;
				if (stub2 && s2idx > 0) {
					const lastPath = rendered[s2idx - 1] as any;
					if (lastPath && lastPath.kind === 'line' && lastPath.x2 === stub2.x1 && lastPath.y2 === stub2.y1) {
						const f2 = fanoutFillet(
							{ x1: lastPath.x1, y1: lastPath.y1, x2: lastPath.x2, y2: lastPath.y2 },
							{ x1: stub2.x1!, y1: stub2.y1!, x2: stub2.x2!, y2: stub2.y2! },
							R,
						);
						if (f2) {
							const l0 = f2.prims[0], ar = f2.prims[1], l2 = f2.prims[2];
							if (l0.kind === 'line' && ar.kind === 'arc' && l2.kind === 'line') {
								const li = rendered.indexOf(lastPath);
								const s2i = rendered.indexOf(stub2 as any);
								if (li >= 0) rendered[li] = { kind: 'line', net: net.name, layer: routingLayer, x1: l0.x1, y1: l0.y1, x2: l0.x2, y2: l0.y2 };
								if (s2i >= 0) rendered[s2i] = { kind: 'line', net: net.name, layer: routingLayer, x1: l2.x1, y1: l2.y1, x2: l2.x2, y2: l2.y2 };
								if (li >= 0) rendered.splice(li + 1, 0, { kind: 'arc', net: net.name, layer: routingLayer, cx: ar.cx, cy: ar.cy, r: ar.r, startAngle: ar.startAngle, sweep: ar.sweep });
							}
						}
					}
				}
			} catch { /* 圆角失败不影响主流程 */ }

			attemptRendered.push(...rendered);
		}

		// 整网成功：提交图元、过孔与动态障碍
		if (netOk) {
			const rds = this.netRendered.get(net.name) ?? [];
			rds.push(...attemptRendered);
			this.netRendered.set(net.name, rds);
			this.addRenderedToObstacles(attemptRendered);
			this.plannedVias = this.plannedVias.filter(v => v.net !== net.name).concat(attemptVias);
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

	/** 撕毁网络：只撤销内存记录 */
	private ripupNet(netName: string): void {
		this.callbacks.onRipup?.([netName]);
		this.netRendered.delete(netName);
		this.plannedVias = this.plannedVias.filter(v => v.net !== netName);
		this.dynamicObstacles = this.dynamicObstacles.filter(o => o.net !== netName);
		const ro = this.routedOrder.indexOf(netName);
		if (ro >= 0) this.routedOrder.splice(ro, 1);
	}
}
