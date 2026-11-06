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
	slotCoverHalfExtents,
} from './rubberband-engine';
import { routeConnection, validatePrimsAgainst } from './visibility-router';
import { polygonToRect } from './rubberband-engine';
export { vrStats, vrStatsReset } from './visibility-router';
export { polygonToRect };

// 过孔动态障碍环余量：EDA 实测（JLC 两层板规则矩阵）Track↔Via 净距
// = Track↔Pad = 0.152mm(6mil) → 中心距要求 = dia/2 + 6 + traceWidth/2。
// 注册半径 = 要求 + VIA_WALL_EXTRA + 互检 0.4：与走线环的 +0.1 同量级。
// 旧值 1.2 把禁区虚高 ~1.6mil——0.5mm 间距焊盘排里这正是 P 行/LCD 排
// 走廊被一座过孔封死的宽度（E2E 实测互检反复撞 via r=24.2）。
const VIA_WALL_EXTRA = 1.2;

import { buildExitCandidates, routePadEdge, stubFeasible, PadTerminal } from './edge-router';
import { gridRoute, GridPrim } from './grid-router';
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
	/** SLOT 孔的轴向（度）：SLOT 焊盘铜皮覆盖按孔轴对齐，不用 pad rotation */
	holeRotation?: number;
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

/** 弧在角 a 处的行进切线（CCW: (-sin,cos)，CW: (sin,-cos)） */
function arcTangent(a: number, sweep: number): { x: number; y: number } {
	return sweep >= 0 ? { x: -Math.sin(a), y: Math.cos(a) } : { x: Math.sin(a), y: -Math.cos(a) };
}

/** 引出段末端（过孔处）的行进方向；空段返回 null */
function stubEndDir(stub: RenderedPrim[]): { x: number; y: number } | null {
	const p = stub[stub.length - 1];
	if (!p) return null;
	if (p.kind === 'line') {
		const dx = (p.x2 ?? 0) - (p.x1 ?? 0), dy = (p.y2 ?? 0) - (p.y1 ?? 0);
		const l = Math.hypot(dx, dy);
		return l > 1e-6 ? { x: dx / l, y: dy / l } : null;
	}
	return arcTangent((p.startAngle ?? 0) + (p.sweep ?? 0), p.sweep ?? 0);
}

/** 顶层图元若有端点落在 (px,py) 附近，返回从该点离开的单位方向；否则 null */
function primDepartDir(p: RenderedPrim, px: number, py: number, tol = 2): { x: number; y: number } | null {
	if (p.layer !== 1) return null;
	if (p.kind === 'line') {
		const x1 = p.x1 ?? 0, y1 = p.y1 ?? 0, x2 = p.x2 ?? 0, y2 = p.y2 ?? 0;
		const l = Math.hypot(x2 - x1, y2 - y1);
		if (l < 1e-6) return null;
		if (Math.hypot(x1 - px, y1 - py) <= tol) return { x: (x2 - x1) / l, y: (y2 - y1) / l };
		if (Math.hypot(x2 - px, y2 - py) <= tol) return { x: (x1 - x2) / l, y: (y1 - y2) / l };
		return null;
	}
	const cx = p.cx ?? 0, cy = p.cy ?? 0, r = p.r ?? 0, a0 = p.startAngle ?? 0, sw = p.sweep ?? 0;
	const a1 = a0 + sw;
	if (Math.hypot(cx + r * Math.cos(a0) - px, cy + r * Math.sin(a0) - py) <= tol) return arcTangent(a0, sw);
	if (Math.hypot(cx + r * Math.cos(a1) - px, cy + r * Math.sin(a1) - py) <= tol) {
		const t = arcTangent(a1, sw);
		return { x: -t.x, y: -t.y };
	}
	return null;
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

/** 反转图元方向（线交换端点；弧交换起终角并取反扫角） */
function reversePrim(p: RenderedPrim): RenderedPrim {
	if (p.kind === 'line') return { ...p, x1: p.x2, y1: p.y2, x2: p.x1, y2: p.y1 };
	return { ...p, startAngle: (p.startAngle ?? 0) + (p.sweep ?? 0), sweep: -(p.sweep ?? 0) };
}

/**
 * 把一段链图元按「起点→终点」定向为连续头尾相接。
 * splitIntoRuns 的 walkOut 左侧返回「邻接→远端」序（与中段相接的排最前），
 * 且 EDA 图元方向可能混向；smoothCorners 依赖 cur.x2≈nxt.x1 才能圆角，
 * 故统一从起点沿连通性重排并逐根定向，否则圆角会被整段跳过。
 */
function orderRunChain(seq: RenderedPrim[], sx: number, sy: number): RenderedPrim[] {
	const sOf = (p: RenderedPrim) => p.kind === 'line'
		? { x: p.x1 ?? 0, y: p.y1 ?? 0 }
		: { x: (p.cx ?? 0) + (p.r ?? 0) * Math.cos(p.startAngle ?? 0), y: (p.cy ?? 0) + (p.r ?? 0) * Math.sin(p.startAngle ?? 0) };
	const eOf = (p: RenderedPrim) => p.kind === 'line'
		? { x: p.x2 ?? 0, y: p.y2 ?? 0 }
		: { x: (p.cx ?? 0) + (p.r ?? 0) * Math.cos((p.startAngle ?? 0) + (p.sweep ?? 0)), y: (p.cy ?? 0) + (p.r ?? 0) * Math.sin((p.startAngle ?? 0) + (p.sweep ?? 0)) };
	const out: RenderedPrim[] = [];
	const rest = new Set<number>(seq.map((_, i) => i));
	let cx = sx, cy = sy;
	while (rest.size) {
		let pick = -1, fromStart = true;
		for (const i of rest) {
			const p = seq[i];
			if (Math.hypot(sOf(p).x - cx, sOf(p).y - cy) < 0.8) { pick = i; fromStart = true; break; }
			if (Math.hypot(eOf(p).x - cx, eOf(p).y - cy) < 0.8) { pick = i; fromStart = false; break; }
		}
		if (pick < 0) break;
		rest.delete(pick);
		const ori = fromStart ? seq[pick] : reversePrim(seq[pick]);
		out.push(ori);
		const e = eOf(ori);
		cx = e.x; cy = e.y;
	}
	for (const i of rest) out.push(seq[i]);
	return out;
}

/** 两段图元序列是否几何一致（容差 1e-6；只用于判断「圆角化是否真的改了几何」） */
function primsSeqEqual(a: RenderedPrim[], b: RenderedPrim[]): boolean {
	if (a.length !== b.length) return false;
	const eq = (x: number | undefined, y: number | undefined) => Math.abs((x ?? 0) - (y ?? 0)) < 1e-6;
	for (let i = 0; i < a.length; i++) {
		const p = a[i], q = b[i];
		if (p.kind !== q.kind || p.layer !== q.layer) return false;
		if (p.kind === 'line') {
			if (!eq(p.x1, q.x1) || !eq(p.y1, q.y1) || !eq(p.x2, q.x2) || !eq(p.y2, q.y2)) return false;
		} else if (!eq(p.cx, q.cx) || !eq(p.cy, q.cy) || !eq(p.r, q.r) ||
			!eq(p.startAngle, q.startAngle) || !eq(p.sweep, q.sweep)) return false;
	}
	return true;
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
	/** padHalfExtent 记忆化（按 allPads 下标；焊盘静态，整轮有效） */
	private padExtentCache = new Map<number, number>();
	/** 焊盘违规计数的空间网格（键 = 排除网络名；cell=256，避免每采样点扫全部焊盘） */
	private padViolationGrid = new Map<string, Map<string, EnginePad[]>>();
	/** 并排焊盘行信息（键 = 焊盘 id）：排向单位向量 + 中位间距 + 排内深度（0 端点 → 1 中心） */
	private rowInfo = new Map<string, { dx: number; dy: number; gap: number; depth: number }>();
	/** 全盘焊盘空间网格（加速过孔落点的近邻净距查询） */
	private padGrid = new Map<string, EnginePad[]>();
	private readonly PAD_GRID_CELL = 64;
	/** 每网络的规划图元 */
	private netRendered = new Map<string, RenderedPrim[]>();
	/** 每个网络已完成的 MST 连接（键 = 两个焊盘 id），用于部分连通后继续重试 */
	private routedEdges = new Map<string, Set<string>>();
	/**
	 * 单边求解结果缓存（只存无过孔的方案）：撕毁重布轮会把同一对焊盘反复
	 * 重解（网格 10~300ms + 可视图 ~100ms/次）。命中后先按当前世界复验净距，
	 * 仍合法就直接复用，省掉整次求解。
	 */
	private edgeSoloCache = new Map<string, RenderedPrim[]>();
	private edgeCacheStats = { hit: 0, stale: 0 };
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
	/** 优化/冲突消除阶段的截止（= mainDeadline，留出尾部兜底预算）：
	 * 实测最终优化用整轮 deadline 判断，会把 tailReserve 吃干，导致
	 * 过孔兜底/终检补救阶段 0 秒进场 */
	private optDeadline = Number.POSITIVE_INFINITY;
	/** 终检清空重布的网（优先补救：本来是布通的，只差过孔落点） */
	private droppedNets = new Set<string>();
	/** 网格兜底的总时间预算（毫秒）：占整轮预算的一小部分，防止兜底吃掉主线时间 */
	/** 网格 A* 兜底的剩余 ms 预算池（链内兜底与抢救阶段共享） */
	private gridBudgetMs = 0;
	/** 本轮网格兜底预算总额（网格抢救阶段按比例保底用） */
	private gridBudgetTotal = 0;
	/** 过孔候选搜索宽度：首次尝试 8（快），重试 12（准）——宽搜索单网更慢，
	 * 只在重试时用，避免吃光首轮预算（实测 case9 宽搜索全程 42/64 vs 自适应 55+） */
	private viaBreadth = 8;
	/** 尾部宽搜开关：换层兜底阶段对「屡试屡败」的网放宽过孔候选（12 个）。
	 * 宽搜索单网慢 3 倍，放在中段会拖垮轮次（case9 50→42），放在末尾
	 * 则是纯赚——这段时间本来就要花在同一批网身上。 */
	private tailWide = false;
	/** 首轮快扫单边求解预算（ms）：快扫只收「一布就通」的边，宁可放过也不要
	 * 在难边上耗 3 秒 —— 实测 F407 快扫 192s/240s 吃光预算，撕毁重布轮拿到 0s */
	private sweepEdgeBudget = 200;
	/** 撕毁重布阶段单网限时（ms，0 = 不限）：实测收紧到 3s 反而少布通
	 *（F407 30 分钟 82→78、case9 54→49）—— 难网本来就该多花时间，切时间
	 * 只是把「能布通」变成「布一半」。预算保障交给 cutoff（求解器按阶段
	 * 截止退出），不必逐网切。保留 __RB_NETCAP__ 环境变量供实验。 */
	private netCapMs = 0;
	/** 当前求解动作的硬截止：所有求解器都通过 checkCancelled() 感知，
	 * 到点即中止（返回「失败」而不是异常），这样单网限时才算真正生效——
	 * 只在边循环头查预算时，一条边内部的十几个候选仍会连跑几分钟。 */
	private cutoff = Number.POSITIVE_INFINITY;
	/** hoist 重排已尝试次数（全局限次，防时间失控） */
	private hoistAttempts = 0;
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
		//（600+ 焊盘）× 网格/射线数千候选点是纯浪费 —— 只查 3×3 邻域格。
		// ⚠ 必须按焊盘**包围盒**入格，不能只入中心格：长焊盘（连接器/天线）
		// 中心离查询点很远但边缘很近，只入中心格会整块漏判（实测过孔被放到
		// 距 LED/LCD 长焊盘 17.94mil，EDA 判定差 0.04mil 违规）。
		this.padGrid.clear();
		for (const p of this.allPads) {
			// ⚠ 入格外扩与 padCopperDistance/viaPadClear 同一口径（padHalfExtent：
			// 读 padShapes 真实形状的外接半对角线，旋转/长盘都不漏）。
			const ext = this.padHalfExtent(p) + 4;
			const i0 = Math.floor((p.x - ext) / this.PAD_GRID_CELL);
			const i1 = Math.floor((p.x + ext) / this.PAD_GRID_CELL);
			const j0 = Math.floor((p.y - ext) / this.PAD_GRID_CELL);
			const j1 = Math.floor((p.y + ext) / this.PAD_GRID_CELL);
			for (let i = i0; i <= i1; i++) {
				for (let j = j0; j <= j1; j++) {
					const k = `${i},${j}`;
					const b = this.padGrid.get(k);
					if (b) b.push(p); else this.padGrid.set(k, [p]);
				}
			}
		}
		this.padObstacleCache.clear();
		this.padViolationGrid.clear();
		this.padExtentCache.clear();
		this.rowInfo.clear();
		this.callbacks = callbacks;
		this.planOptions = input.options ?? {};
		// 快扫单边预算（默认 600ms）；大板上 98 条网的快扫会吃掉 1/3 预算，
		// 实验开关用于量化「快扫少花时间、把预算让给撕毁/网格」的收益。
		{
			const sweepMs = Number((globalThis as any).__RB_SWEEP_EDGE_MS__);
			this.sweepEdgeBudget = Number.isFinite(sweepMs) && sweepMs > 0 ? sweepMs : 200;
		}
		this.copperLayers = Math.max(1, Math.round(input.copperLayers ?? 2));
		this.dynamicObstacles = [];
		this.obstacleIdCounter = 0;
		this.evictionTried.clear();
		this.evictionAttempts = 0;
		this.planDeadline = Number.POSITIVE_INFINITY;
		this.netRendered.clear();
		this.routedEdges.clear();
		this.edgeSoloCache.clear();
		this.edgeCacheStats = { hit: 0, stale: 0 };
		this.routedOrder = [];
		this.plannedVias = [];
		this.logs = [];
		this.cancelled = false;

		// 「不限时」不等于无限：实测 F407 在不限时预算下跑了 9.8 小时，布通率
		// 还比 30 分钟预算低（50 vs 82）—— 多出来的时间全花在无望网上反复撕毁。
		// 这里把不限时折算成 60 分钟硬预算：足够跑完 3~4 轮撕毁 + 全部兜底，
		// 又不会让一次布线变成过夜任务。想要更久请显式设置预算。
		const UNLIMITED_CAP_MS = 60 * 60_000;
		const budgetMs = input.budgetMs && input.budgetMs > 0 ? input.budgetMs : UNLIMITED_CAP_MS;
		const deadline = Date.now() + budgetMs;
		this.planDeadline = deadline;
		this.hoistAttempts = 0;
		// 阶段预算纪律：快扫/撕毁重布只许花掉预算的大头，尾部阶段
		//（网格抢救/最终优化/冲突消除/换层兜底）保底一段专属时间。
		// 实测 120s 预算下 ripup 顶穿到 123s，四个收尾阶段全部 0s ——
		// 而网格抢救恰恰是把「部分布通」转成「布通」的最便宜手段。
		const tailReserve = Number.isFinite(deadline) && budgetMs > 0
			? Math.min(600_000, Math.max(30_000, budgetMs * 0.2))
			: 0;
		let mainDeadline = deadline - tailReserve;
		// 主连通阶段占比：撕毁重布实测会一路吃满主预算，把网格抢救/协商/优化
		// 全部挤成 0s。拥挤协商开启时默认只给主阶段 55%（协商实测是把
		// 66→71/98 的唯一增量来源，必须留足入场时间）；__RB_MAIN_FRAC__ 覆盖。
		const negEnabled = !(globalThis as any).__RB_NO_NEG__;
		{
			const frac = Number((globalThis as any).__RB_MAIN_FRAC__) || (negEnabled ? 0.55 : 0);
			if (frac > 0 && Number.isFinite(mainDeadline) && budgetMs > 0) {
				mainDeadline = Math.min(mainDeadline, Date.now() + budgetMs * frac);
			}
		}
		// 撕毁动作与成组重布的软截止：动作内部逐网求解受它约束，
		// 最多吃掉预留的一半，剩下一半保底给网格抢救/优化/冲突消除
		//（成组重布是原子动作、无内部检查时实测一次烧 30s，把抢救挤成 0s）
		const softMain = mainDeadline + tailReserve * 0.5;
		// 优化/冲突消除拿到 mainDeadline 就收手，tailReserve 全留给终检 + 换层兜底
		this.optDeadline = mainDeadline;
		// 网格兜底预算：有界预算取 20%（至少 60s），不限时给 600s 上限
		this.gridBudgetMs = Number.isFinite(budgetMs) && budgetMs > 0
			? Math.max(60_000, budgetMs * 0.2)
			: 600_000;
		this.gridBudgetTotal = this.gridBudgetMs;

		// 网络排序（auto = 专业自动规划；地网络永远最后）
		const networks = this.sortNetworks(input.networks);

		// ── 总线束前置（Topola antrouter 思路第一片）——实验否定：case9 离线
		// 52/63 vs 基线 56，重排覆盖面过大摧毁了调校过的 auto 顺序（与「顺序
		// 扰动有害」的既有结论一致）。默认关闭，__RB_BUNDLE__=1 显式启用。
		if ((globalThis as any).__RB_BUNDLE__ === 1 && networks.length > 8) {
			const bundled = this.reorderByBundles(networks);
			if (bundled) {
				networks.length = 0;
				networks.push(...bundled);
				this.log(`[bundle] 总线束前置重排：${bundled.map(n => n.name).join(' → ')}`, 'info');
			}
		}

		this.log(`网络顺序（${this.planOptions.netOrder ?? 'auto'}）：${networks.map(n => n.name).join(' → ')}`, 'info');
		const pendingNets: Array<{ name: string; pads: EnginePad[] }> = [];

		// __RB_LANE_RESERVE__=1：排网走廊预约（车道分配第一层）。
		// case9 墙的根因：密排连接器排（LCD/P0x）的逃逸扇区被横向穿越网
		//（+5V/$1N5xxx 等）压死。这里先算出每个行网的 solo 走廊，注册为
		// 硬障碍「预约车道」—— 其它网求解时绕行；行网自己布线时同网豁免，
		// 布通后回收自己的预约（换上真实走线障碍）。
		if ((globalThis as any).__RB_LANE_RESERVE__ && !this.checkCancelled()) {
			const t0r = Date.now();
			const laneCap = t0r + 30_000;
			let reservedNets = 0;
			for (const net of networks) {
				if (this.checkCancelled() || Date.now() > laneCap) break;
				if (!net.pads.some(p => this.rowInfo.get(p.id) && (this.rowInfo.get(p.id)?.gap ?? 99) <= 47)) continue;
				if (this.routedOrder.includes(net.name)) continue;
				const solo = await this.soloRoute(net);
				if (!solo) continue;
				const laneR = this.drc.traceWidthMil / 2 + Math.max(this.drc.trackToTrackMil, this.drc.trackToPadMil) + 2;
				for (const p of solo.prims) {
					if (p.kind !== 'line') continue;
					// soloRoute 的 RBLine 不带层信息：预约障碍跨层登记（保守）
					this.dynamicObstacles.push(...buildPolylineObstacle(
						[{ x: p.x1!, y: p.y1! }, { x: p.x2!, y: p.y2! }],
						laneR, net.name, undefined, `lane:${net.name}`,
					));
				}
				reservedNets++;
			}
			if (reservedNets) this.log(`[lane] 车道预约：${reservedNets} 个行网的 solo 走廊已注册（${Date.now() - t0r}ms）`, 'info');
		}

		let __tPhase = Date.now();
		const __t0 = __tPhase;
		const __phase = (label: string) => {
			const now = Date.now();
			// 不限时预算（budgetMs=0）下 deadline 是 +Infinity，用 (now-deadline+budget)
			// 算累计会打出 -Infinitys，改成按规划起点算真实墙钟
			const total = ((now - __t0) / 1000).toFixed(0);
			const cap = budgetMs > 0 ? ` /${(budgetMs / 1000).toFixed(0)}s` : '（不限时）';
			this.log(`[耗时] ${label}: ${((now - __tPhase) / 1000).toFixed(1)}s（累计 ${total}s${cap}）`, 'info');
			__tPhase = now;
		};

		// 快扫段限时：只占预算三分之一（不限时预算给 12 分钟）。快扫的任务是
		// 收割「一布就通」的边，不是攻克难边；实测 F407 快扫 192s/240s 吃光预算，
		// 导致撕毁重布轮 0 秒进场、布通率卡在 40%。
		const sweepStart = Date.now();
		const sweepDeadline = Number.isFinite(mainDeadline)
			? Math.min(mainDeadline, sweepStart + budgetMs / 3)
			: sweepStart + 12 * 60_000;
		let sweepSkipped = 0;
		// 第一轮：逐网络布线
		for (let ni = 0; ni < networks.length; ni++) {
			if (this.checkCancelled()) break;
			if (Date.now() > sweepDeadline) {
				this.log(`首轮快扫超时（${Math.round((Date.now() - sweepStart) / 1000)}s），剩余 ${networks.length - ni} 个网络转交撕毁重布轮`, 'warn');
				for (let k = ni; k < networks.length; k++) pendingNets.push(networks[k]);
				sweepSkipped = networks.length - ni;
				break;
			}
			const net = networks[ni];
			this.progress(
				this.routedPct(networks),
				`布线网络 ${ni + 1}/${networks.length}: ${net.name}（已布通 ${this.routedOrder.length}/${networks.length}）`,
			);
			// 第一遍·线段级快扫：先把「一布就通」的边全部收割（不追求单网
			// 完整再下一网）；有剩余边的网进入撕毁重布/兜底轮做全链收尾
			const ok = await this.routeNetEdges(net, 'quick');
			if (!ok) pendingNets.push(net);
		}

		this.log(`首轮快扫完成：布通 ${this.routedOrder.length}/${networks.length}${sweepSkipped ? `（跳过 ${sweepSkipped} 个网络）` : ''}`, 'info');
		__phase('首轮快扫');

		// 实验开关：快扫后立即做一次网格抢救（连通率优先）。
		// 动机：撕毁重布轮实测吃掉 60%~80% 预算，而尾部一次网格抢救只用
		// 1~2s 就能把大量「部分布通」转成布通。把抢救提前，尽早锁定连通性，
		// 撕毁重布只服务剩余难网。
		if ((globalThis as any).__RB_GRID_EARLY__ && !this.checkCancelled()) {
			this.gridBudgetMs = Math.max(this.gridBudgetMs, Math.min(this.gridBudgetTotal * 0.5, 90_000));
			await this.gridRescuePhase(networks);
			__phase('早鸟网格抢救');
		}
		// 多轮撕毁重布
		const ripupRounds = Math.max(0, Math.round(this.planOptions.ripupRounds ?? 3));
		const ripupPhaseStart = Date.now();
		// 停滞保护：连续一段时间没有「新布通网络」就提前结束撕毁重布，把时间
		// 交给网格抢救/尾部兜底。大板上撕毁重布会反复在同样的失败网/部分网上
		// 空转（case12 实测 120s 预算下耗尽整个主阶段而布通数不变）；30 分钟
		// 预算时更是能空转 20 分钟。阈值随预算缩放，最多 90s。
		const stagnationMs = Number.isFinite(mainDeadline)
			? Math.min(60_000, Math.max(20_000, budgetMs * 0.22))
			: 45_000;
		// __RB_STAG_MS__：实验旋钮。密板收尾期失败动作 2~5s/个，27s 窗口只够
		// 6~10 次尝试就放弃整轮 —— 放宽让评分撕毁/借道多试几轮。
		const stagOverride = Number((globalThis as any).__RB_STAG_MS__);
		const stagMs = Number.isFinite(stagOverride) && stagOverride > 0 ? stagOverride : stagnationMs;
		let lastRoutedCount = this.routedOrder.length;
		let lastNewNetAt = Date.now();
		for (let pass = 0; pass < ripupRounds && pendingNets.length; pass++) {
			if (this.checkCancelled()) break;
			// 不限时预算：撕毁重布总计封顶 30 分钟，避免整轮被无望网拖成小时级
			if (!Number.isFinite(mainDeadline) && Date.now() - ripupPhaseStart > 30 * 60_000) {
				this.log('撕毁重布累计超过 30 分钟，提前结束（剩余网转交成组重布/兜底）', 'warn');
				break;
			}
			// ── 无进展保护 ──
			// 不限时预算下，无望网会被一轮轮反复重试（实测 F407 156 网跑 9.8 小时
			// 只多布通个位数）。每轮记录「布通数 + 有部分走线的网数」，整轮零增长
			// 就直接收手，把时间让给后面的成组重布/兜底阶段。
			const roundStart = Date.now();
			const routedBefore = this.routedOrder.length;
			const partialBefore = pendingNets.filter(n => (this.netRendered.get(n.name) ?? []).length > 0).length;
			// 单轮时长上限（不限时预算时生效）：8 分钟。5 轮 × 8 分钟封顶 40 分钟，
			// 避免整个布线被无望网拖成小时级。
			const roundCapMs = Number.isFinite(mainDeadline) ? Infinity : 8 * 60_000;
			// 快者先走：剩余未完成边少的网排前 —— 易网在每轮头部快速收割布通，
			// 难网（往往要靠成组重布/兜底）只消耗尾部预算，避免预算耗尽时
			// 一批「其实一布就通」的网还压在队尾没轮到
			pendingNets.sort((a, b) => this.netPendingEdges(a) - this.netPendingEdges(b));
			if (Date.now() > mainDeadline) {
				this.log('规划时间预算用尽，停止撕毁重布', 'warn');
				break;
			}
			const next: typeof pendingNets = [];
			for (let pi = 0; pi < pendingNets.length; pi++) {
				if (roundCapMs !== Infinity && Date.now() - roundStart > roundCapMs) {
					this.log(`第 ${pass + 1} 轮超过 ${Math.round(roundCapMs / 60000)} 分钟上限，剩余 ${pendingNets.length - pi} 个网留待后续阶段`, 'warn');
					for (let k = pi; k < pendingNets.length; k++) next.push(pendingNets[k]);
					break;
				}
				const net = pendingNets[pi];
				if (this.checkCancelled()) break;
				// 前一轮动作里已布通（含底层兜底救回）的网络不再重试
				if (this.routedOrder.includes(net.name)) continue;
				if (this.routedOrder.length > lastRoutedCount) {
					lastRoutedCount = this.routedOrder.length;
					lastNewNetAt = Date.now();   // 有新布通就重置停滞计时
				}
				if (Date.now() - lastNewNetAt > stagMs) {
					this.log(`连续 ${Math.round(stagMs / 1000)}s 没有新布通网络，提前结束撕毁重布（剩余 ${pendingNets.length - pi} 个网转交网格抢救/兜底）`, 'warn');
					for (let k = pi; k < pendingNets.length; k++) next.push(pendingNets[k]);
					break;
				}
				// 预算见底就停：单网撕毁重布 = 1+2N 次寻路，分钟级；只在轮头检查
				// 会让一轮内部顶穿预算几十分钟（实测某板超预算 55 分钟）
				if (Date.now() > mainDeadline) {
					this.log('规划时间预算用尽，剩余网络不再重试', 'warn');
					for (let k = pi; k < pendingNets.length; k++) next.push(pendingNets[k]);
					break;
				}
				const ends = [net.pads[0], net.pads[net.pads.length - 1]];
				// ── 撕毁对象选择 ──
				// ① 首选「独走最优路径」挡路者：本网忽略其它网走线时的最短路
				//    （soloRoute 只避静态障碍+焊盘）上压着的已布/残段网，才是
				//    真正的最小阻挡集 —— 扇出/封口评分都在猜，这里直接看路。
				//    按重布代价升序（焊盘少的好重排）取前 K。
				// ② 没有独走路径（几何被焊盘挡死）才退回评分制（封口+2/扇出+3/
				//    走廊+1）—— 那时挡路者不在独走路径上，只能按邻近度猜。
				// 单网实测 case12 全部 40 个失败网 <3s 可解（solo-scan），差的全
				// 是占用 —— 撕对对象比多撕更重要。
				let rip: string[] = [];
				if ((globalThis as any).__RB_SOLO_RIP__ && net.pads.length <= 6) {
					const solo = await this.soloRoute(net);
					if (solo && solo.prims.length) {
						const hits = new Map<string, number>();
						for (const p of solo.prims) {
							const layer = (p as RenderedPrim).layer;
							const pts = this.primPoints([{ ...p, layer } as unknown as RenderedPrim], 6);
							for (const o of this.dynamicObstacles) {
								if (o.soft || !o.net || o.net === net.name) continue;
								const g = o.group ?? '';
								if (!g.startsWith('trk:') && !g.startsWith('via:')) continue;
								if (!this.netRendered.has(o.net)) continue;
								if (o.layer !== undefined && layer !== undefined && o.layer !== layer) continue;
								for (const pt of pts) {
									if (Math.hypot(o.x - pt.x, o.y - pt.y) < o.r) {
										hits.set(o.net, (hits.get(o.net) ?? 0) + 1);
										break;
									}
								}
							}
						}
						rip = [...hits.entries()]
							.map(([n, h]) => ({ n, h, pads: networks.find(x => x.name === n)?.pads.length ?? 99 }))
							.sort((a, b) => (a.pads - b.pads) || (b.h - a.h) || (a.n < b.n ? -1 : 1))
							.slice(0, Number((globalThis as any).__RB_RIP_TOP__) || 3)
							.map(e => e.n);
					}
				}
				if (!rip.length) {
				// 挡路网评分制（旧逻辑取「最近布的 3 个端点封口网」纯属运气）：
				//   端点封口 +2 / 焊盘排扇出带 +3 / MST 走廊 +1，取最高分前 3。
				// 只统计「可移动」障碍（trk:/via:）——焊盘是静态的，撕了网它家的
				// 焊盘还在，白撕一场。部分走线网（未布通但有残段）同样参选：
				// case12 实测 SD_CLK 的扇出带被邻排 SD_CMD/SD_DATA0 的部分走线
				// 占 24%、DVP_D3_1 被 AUDIO_I2S_MCLK 100% 横穿 —— 全是残段阻挡。
				const corridor = this.drc.trackToTrackMil + this.drc.traceWidthMil + 20;
				const fanHw = this.drc.traceWidthMil / 2;
				const fans: Array<{ ax: number; ay: number; bx: number; by: number }> = [];
				for (const e of ends) {
					const row = this.rowInfo.get(e.id);
					if (!row) continue;
					const L = row.gap * 1.5 + 40 + (e.bodyRadius || e.radius);
					const nx = -row.dy, ny = row.dx;
					fans.push({ ax: e.x, ay: e.y, bx: e.x + nx * L, by: e.y + ny * L });
					fans.push({ ax: e.x, ay: e.y, bx: e.x - nx * L, by: e.y - ny * L });
				}
				const mst = this.computeMST(net.pads);
				const score = new Map<string, number>();
				for (const o of this.dynamicObstacles) {
					if (o.soft || !o.net || o.net === net.name) continue;
					const g = o.group ?? '';
					if (!g.startsWith('trk:') && !g.startsWith('via:')) continue;
					if (!this.netRendered.has(o.net)) continue;   // 已被更早动作清掉的残留
					let w = 0;
					if (ends.some(e => Math.hypot(o.x - e.x, o.y - e.y) < o.r + 70)) w += 2;
					for (const f of fans) {
						if (pointSegDist(o.x, o.y, f.ax, f.ay, f.bx, f.by) < o.r + fanHw) { w += 3; break; }
					}
					for (const [ia, ib] of mst) {
						const pa = net.pads[ia], pb = net.pads[ib];
						if (pointSegDist(o.x, o.y, pa.x, pa.y, pb.x, pb.y) < o.r + corridor) { w += 1; break; }
					}
					if (w > 0) score.set(o.net, w);
				}
				const scored = [...score.entries()].sort((a, b) => b[1] - a[1] || (a[0] < b[0] ? -1 : 1))
					.slice(0, Number((globalThis as any).__RB_RIP_TOP__) || 3)
					.map(([n]) => n);
				rip = scored;
				}
				if (!rip.length) {
					// 快扫遗留：未被封口的网补一次完整链尝试（快扫只试直连，
					// 这些网还没享受过边界候选退避/换层链）；仍失败才顺延
					const rok = await this.routeNetEdges(net, 'full', this.netSoft(softMain));
					if (rok) continue;
					// hoist：端点附近没抓到封口，但 MST 走廊上可能压着别的网。
					// 把「堵在走廊上的网」挪到本网之后（先布本网再补它），
					// 这是拥塞板提高连通率的关键手段；失败自动回滚不净损失。
					if (this.hoistAttempts < (Number((globalThis as any).__RB_HOIST_MAX__) || 16) && Date.now() < mainDeadline - 15_000) {
						this.hoistAttempts++;
						if (await this.tryHoistReroute(net, networks)) continue;
					}
					next.push(net);
					continue;
				}
				// 借道优先：只拆挡路网走廊上的一段、断口原地重连。比整网撕毁
				// 便宜一个量级（整网重排经常无处可去 → 整场回滚）；失败再整撕。
				if ((globalThis as any).__RB_BYPASS__ && Date.now() < mainDeadline - 10_000) {
					let bypassed = false;
					for (const b of rip.slice(0, 4)) {
						if (await this.tryBypassReroute(net, b, networks)) { bypassed = true; break; }
					}
					if (bypassed) continue;
				}
				this.log(`第 ${pass + 1} 轮：网络 ${net.name} 撕毁 ${rip.join(', ')} 重试`, 'warn');
				// 快照被撕网络的原走线：本动作绝不净损失已布网络——
				// 任一被撕网重布失败即整体回滚到动作前的已知好状态
				//（被撕网的新走线可能与回滚网冲突，必须一并撤销）。
				const savedOrder = [...this.routedOrder];
				const saved = new Map<string, { prims: RenderedPrim[]; vias: PlannedVia[]; doneEdges: Set<string> }>();
				for (const n of rip) {
					saved.set(n, {
						prims: [...(this.netRendered.get(n) ?? [])],
						vias: this.plannedVias.filter(v => v.net === n),
						// 部分走线网也在撕毁之列：按快照恢复已完成的边集合，
						// 不能 markAllEdgesDone（会把残段误标成整网布通）
						doneEdges: new Set(this.routedEdges.get(n) ?? []),
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
				const ok = await this.routeNetEdges(net, 'full', this.netSoft(softMain));
				let sealFailed = false;
				for (let i = rip.length - 1; i >= 0; i--) {
					const ripped = networks.find(nn => nn.name === rip[i])!;
					const rok = await this.routeNetEdges(ripped, 'full', this.netSoft(softMain));
					if (!rok) { next.push(ripped); sealFailed = true; }
				}
				if (sealFailed) {
					// 有被撕网重布失败：两焊盘墙先试底层兜底（失败网的新走线往往已
					// 封死它原来的顶层通道）；「完整布通网」全部救回才接受，否则回滚。
					// 例外：**动作前就是部分走线网**的救不回不触发回滚 —— 它本来
					// 就是失败网，残段让位给目标网换整组推进，自身转交后续轮次在
					// 新格局里重试（case12 密排板的关键解锁：SD/DVP 排的邻网残段
					// 救回率低，一刀切回滚让撕毁动作整场白做）。
					// F407 的教训不受影响：那里误判的是「完整网被拆成半成品算救回」，
					// 这里仍然只有 routedOrder 里真正布通（重布或两焊盘兜底）才算。
					let unrecoverable = false;
					for (let i = rip.length - 1; i >= 0; i--) {
						const n = rip[i];
						// 「残留部分走线」不算救回：完整布通的网络被拆成半成品就是净损失
						//（实测 F407 隔离板四个 3-pad 墙全以 1/2 残留被误判「已救回」而接受，
						// 布通率 8 → 7）。只有 routedOrder 里真正布通（重布或两焊盘兜底）才算。
						if (this.routedOrder.includes(n)) continue;
						// 动作前就是部分走线网：不算「不可恢复」，接受本次推进
						if (!savedOrder.includes(n)) continue;
						// 恢复兜底同样受 softMain 约束：超时按「不可恢复」走整体回滚
						if (Date.now() > softMain) { unrecoverable = true; break; }
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
					this.routedEdges.set(n, new Set(sv.doneEdges));
					const idx = savedOrder.indexOf(n);
					if (idx >= 0) this.routedOrder.splice(Math.min(idx, this.routedOrder.length), 0, n);
				}
				for (const v of this.plannedVias) {
					if (!rip.includes(v.net) && v.net !== net.name) continue;
					this.dynamicObstacles.push({
						id: `via${this.obstacleIdCounter++}`, kind: 'circle',
						x: v.x, y: v.y, r: v.dia / 2 + Math.max(this.drc.trackToTrackMil, this.drc.trackToPadMil) + this.drc.traceWidthMil / 2 + VIA_WALL_EXTRA,
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
			// 轮末终检：回滚恢复的过孔可能已落在「这一轮新增铜皮」的近旁。
			// 早拆早救 —— 拆掉违规过孔后整网清空、立刻放回 pending，下一轮
			// 以新格局从零重布。若拖到全流程末尾才拆，只能降级成部分布通
			//（实测 F407 末尾一次性拆 9 个网，尾部只剩十几秒，一个都救不回）。
			// 幂等且便宜（过孔数 × 焊盘数），每轮跑一次不心疼。
			this.repairViaClearance();
			for (const nm of this.droppedNets) {
				if (this.routedOrder.includes(nm)) continue;
				const dn = networks.find(n => n.name === nm);
				if (dn && !pendingNets.some(p => p.name === nm)) pendingNets.push(dn);
			}
			this.droppedNets.clear();
			// 轮末进展检查：布通数与「有部分走线」的网数都没涨 → 后续轮次
			// 大概率也是空转，直接结束撕毁重布（失败网交给成组重布/兜底）
			const routedAfter = this.routedOrder.length;
			const partialAfter = pendingNets.filter(n => (this.netRendered.get(n.name) ?? []).length > 0).length;
			this.log(`第 ${pass + 1} 轮结束：布通 ${routedBefore} → ${routedAfter}，部分走线网 ${partialBefore} → ${partialAfter}（${Math.round((Date.now() - roundStart) / 1000)}s）`, 'info');
			this.progress(this.routedPct(networks), `撕毁重布第 ${pass + 1} 轮：已布通 ${routedAfter}/${networks.length}`);
			if (routedAfter === routedBefore && partialAfter <= partialBefore) {
				this.log('本轮无任何进展，提前结束撕毁重布（剩余网转交成组重布/过孔兜底）', 'warn');
				break;
			}
		}

		__phase('撕毁重布轮');
		// 最终成组重布：以失败网的「焊盘中心连线」为走廊，撕掉走廊上其它网络的
		// 走线/过孔（端点附近探测抓不到中段阻挡 —— 例如扇出排里外层网络的斜线
		// 正好切过内层网络的必经通道），失败网按内向优先重布，再重布被拆网
		// （它们会自然向外嵌套，恢复平面序）。
		// 成组重布只在主预算内入场：动作内部受 softMain 约束，被拆网恢复
		// 也一样 —— 超时即回滚，给网格抢救/优化/冲突消除留燃料
		if (pendingNets.length && Date.now() <= mainDeadline && !!(globalThis as any).__RB_GROUP_REROUTE__) {
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
					if (!(await this.routeNetEdges(net, 'full', this.netSoft(softMain)))) this.log(`成组重布：${net.name} 仍失败`, 'error');
				}
				const failedWalls: string[] = [];
				for (const n of rip) {
					const ripped = networks.find(nn => nn.name === n)!;
					if (!(await this.routeNetEdges(ripped, 'full', this.netSoft(softMain)))) { this.log(`成组重布：重布 ${n} 失败`, 'error'); failedWalls.push(n); }
				}
				// 被拆网重布失败：两焊盘墙先试底层兜底（失败网的新走廊通常已封死
				// 它原来的顶层通道）；全部救回就接受当前局面（失败网保持布通），
				// 否则整体回滚 —— 恢复被拆网原走线，失败网恢复到动作前的部分状态
				// 「残留部分走线」不算救回：完整网被拆成半成品就是净损失，
				// 只有 routedOrder 里真正布通（重布或两焊盘兜底）才接受新布局
				let unrecoverable = false;
				for (const n of failedWalls) {
					if (this.routedOrder.includes(n)) continue;
					// 恢复（两焊盘底层兜底）也要守 softMain：超时就整体回滚，
					// 否则无预算检查的兜底链会把尾部阶段挤成 0s（实测 case12 烧 13s+）
					if (Date.now() > softMain) { unrecoverable = true; break; }
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
							x: v.x, y: v.y, r: v.dia / 2 + Math.max(this.drc.trackToTrackMil, this.drc.trackToPadMil) + this.drc.traceWidthMil / 2 + VIA_WALL_EXTRA,
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

		__phase('成组重布');
		// 网格抢救：对仍有缺失边的网络，逐边用网格 A* 再试一次（一次性）。
		// 连通率优先于走线形态：排在最终优化/冲突消除之前（此前排在
		// optimizeRoutes 之后，预算紧张时最先被牺牲 —— 与注释本意相悖）。
		if (!this.checkCancelled() && Date.now() <= deadline) {
			// 链内兜底可能已把网格池烧掉大半：抢救阶段保底 40% 池（≤120s）
			this.gridBudgetMs = Math.max(this.gridBudgetMs, Math.min(this.gridBudgetTotal * 0.4, 120_000));
			await this.gridRescuePhase(networks);
		}

		__phase('网格抢救');
		// 拥挤协商终局：顺序撕毁饱和后的全局收敛手段（见 negotiatePhase 注释）。
		// 默认开启（__RB_NO_NEG__=1 关）。截止留 30s 给终检/尾部抢救/拉紧收尾
		// —— 协商赚到的网还要过终检微移/拆除，贴太紧会在尾部被拆回去。
		if (!(globalThis as any).__RB_NO_NEG__ && !this.checkCancelled() &&
			networks.some(n => !this.routedOrder.includes(n.name))) {
			const pfOn = !!(globalThis as any).__RB_PF__;
			const pfReserve = pfOn ? (Number((globalThis as any).__RB_PF_RESERVE__) || 75_000) : 30_000;
			const negDeadline = Number.isFinite(deadline)
				? deadline - pfReserve
				: Date.now() + 10 * 60_000;
			if (Date.now() < negDeadline) await this.negotiatePhase(networks, negDeadline);
			__phase('拥挤协商');
		}
		// PathFinder 终局（__RB_PF__=1）：失败网软提交意向路径（跨网重叠在
		// 阶段内合法存在，历史罚分跨迭代逼惯犯让位），阶段末统一清判转正/
		// 重布/拆除。与协商「每动作全或无」接受互补 —— 重叠不再要求一次让
		// 6 个网同时挪窝。清判在终检/尾部之前完成（重叠态不过终检口径）。
		if ((globalThis as any).__RB_PF__ && !this.checkCancelled() &&
			networks.some(n => !this.routedOrder.includes(n.name)) && Date.now() < deadline - 25_000) {
			this.pfTentative.clear();
			if ((globalThis as any).__RB_PF_ZONE__) {
				await this.pfZonePhase(networks, deadline - 25_000);
			} else {
				await this.pathfinderPhase(networks, deadline - 25_000);
			}
			__phase('PathFinder');
		}
		// 实验：主连通阶段被限流后，把一部分尾部时间明确分配给最终优化
		// （重布已布网络，只有更短且不新增过孔才采纳）。否则 optDeadline
		// 已经过去，优化永远 0s —— 用户反馈的「异常绕线」正是缺少这一轮。
		if ((globalThis as any).__RB_OPT_TAIL__ && this.planOptions.optimize === true) {
			this.optDeadline = Math.min(deadline - 8_000, Date.now() + budgetMs * 0.15);
		}
		// 最后优化阶段：对已布网络重新求一次「中心到中心」最短路径，
		// 成功且更短才替换，失败保留首轮连通走线。
		if (this.planOptions.optimize === true && this.routedOrder.length > 0 &&
			!this.checkCancelled() && Date.now() <= deadline) {
			await this.optimizeRoutes(networks);
		}

		__phase('最终优化');
		// 冲突消除：被别的网络挡住而绕远的网络，重新安置到更短的通道
		if (!this.checkCancelled() && Date.now() <= this.optDeadline) {
			await this.resolveContention(networks);
		}

		__phase('冲突消除');
		// 尾部抢救循环：实测 case9 E2E 在预算还剩 500s 时引擎就没活干了——
		// 抢救只跑一轮，终检拆孔清空的网（LCD5）再也得不到第二次机会。把
		// 「终检 → 清空网补救 → 部分网逐边重试 → 失败网驱逐/兜底」打包成一轮，
		// 只要本轮有新布通就再来一轮（≤4 轮或预算耗尽为止）。
		const rescueRound = async (): Promise<number> => {
			// 终检提前：回滚快照恢复的过孔可能落在「世界已变化」的位置（别的网后来
			// 布了线），越晚发现代价越大 —— 末尾一次性拆除会让整网降级为部分布通且
			// 没有时间补救（实测 F407 18 个过孔 → 9 个网丢失）。
			this.repairViaClearance();
			// 被终检清空的网络优先补救：它们此前已经布通，只是过孔落在回滚后的世界里，
			// 用当前定型的障碍格局重求一次，成功率远高于普通失败网。放在最前面，
			// 避免尾部预算被「部分布通网逐边重试」先吃掉。
			for (const nm of [...this.droppedNets]) {
				if (this.checkCancelled() || Date.now() > this.planDeadline - 2_000) break;
				const dn = networks.find(n => n.name === nm);
				if (!dn || this.routedOrder.includes(nm)) continue;
				if (!(await this.tryViaFallback(dn))) await this.routeNetEdges(dn);
				this.sanitizeNet(nm);
			}
			this.droppedNets.clear();
			// 过孔换层兜底：单层容量不足时，失败网络走底层，两端用逃逸段+过孔连接
			this.tailWide = true;
			if (this.planOptions.allowViaFallback !== false && !this.checkCancelled() && Date.now() <= deadline) {
				// 部分布通网络先做最后一轮逐边重试：回滚机制保证其有效走线完好，
				// 障碍格局定型后未完成的边可能直接布通（routeNetEdges 跳过已完成边，不会重复叠线）
				const partialNets = networks.filter(n =>
					!this.routedOrder.includes(n.name) &&
					(this.netRendered.get(n.name) ?? []).length > 0);
				for (const net of partialNets) {
					if (this.checkCancelled()) break;
					if (Date.now() > this.planDeadline) break;
					// 留 8s 尾窗给「终检拆孔后的补救」，否则最后放下的违规过孔
					// 会在轮末复验里把整网清掉且再没有时间救回（实测 LCD0）。
					await this.routeNetEdges(net, 'full', Math.min(this.planDeadline - 8_000, this.netSoft(softMain)));
				}
				const stillFailed = networks.filter(n =>
					!this.routedOrder.includes(n.name) &&
					// 已部分布通的网络已保留有效走线，逐边重试时已包含过孔候选；
					// 再跑「首尾焊盘」兜底会在已有铜皮上重复叠线。
					(this.netRendered.get(n.name) ?? []).length === 0);
				for (const net of stillFailed) {
					if (this.checkCancelled()) break;
					// 每网兜底都含多次路由器求解，逐网检查预算防止收尾顶穿；
					// 同样留 8s 给轮末拆孔补救。
					if (Date.now() > this.planDeadline - 8_000) break;
					// 先试过孔槽位驱逐（落点被单一网络的过孔独占封死时拆网重排），
					// 没有可驱逐对象再走普通过孔兜底
					if (!(await this.tryViaSlotEviction(net, networks))) {
						await this.tryViaFallback(net);
					}
					this.sanitizeNet(net.name);
				}
			}
			this.tailWide = false;
			// 轮末复验：本轮新放下的过孔要立即按最终世界复核，违规的在这里
			// 拆掉并把清空的网并入 droppedNets —— 下一轮开头会优先补救。
			// 若只在全部轮次结束后复验，最后一次的违规过孔会让整网净损失
			//（实测 120s 预算：尾部抢救到 47，最终复验掉回 45）。
			// 临近截止时用 final 口径：只拆孔、不清网，保留部分走线。
			this.repairViaClearance(this.planDeadline - Date.now() < 15_000);
			return this.routedOrder.length;
		};
		let prevRoutedNets = -1;
		for (let rr = 0; rr < 4; rr++) {
			const rrT0 = Date.now();
			const routedNow = await rescueRound();
			this.log(`尾部抢救第 ${rr + 1} 轮：布通 ${routedNow}/${networks.length}（${Date.now() - rrT0}ms）`, 'info');
			if (routedNow <= prevRoutedNets) break;   // 本轮没有新布通 → 停止
			prevRoutedNets = routedNow;
			if (this.checkCancelled() || Date.now() > this.planDeadline - 10_000) break;
		}

		// 尾窗补救：轮末终检拆掉的违规过孔可能让整网断开（LCD0 类）。
		// 预留的 8s 尾窗里，用「当前定型世界」最后重试一次；失败则恢复
		// 拆孔前的部分走线（不能让终检把已画的线也吞掉）。
		if (this.droppedNets.size && !this.checkCancelled() && Date.now() < this.planDeadline - 2_000) {
			for (const nm of [...this.droppedNets]) {
				if (Date.now() > this.planDeadline - 1_000) break;
				const dn = networks.find(n => n.name === nm);
				if (!dn || this.routedOrder.includes(nm)) continue;
				const saved = {
					prims: [...(this.netRendered.get(nm) ?? [])],
					vias: this.plannedVias.filter(v => v.net === nm),
				};
				this.ripupNetInternal(nm, true);
				let ok = await this.tryViaFallback(dn);
				if (!ok) ok = await this.routeNetEdges(dn, 'full', this.planDeadline);
				this.sanitizeNet(nm);
				if (!ok && !this.routedOrder.includes(nm) && saved.prims.length) {
					this.ripupNetInternal(nm, true);
					this.netRendered.set(nm, saved.prims);
					this.addRenderedToObstacles(mergePrims(saved.prims));
					this.plannedVias = this.plannedVias.filter(v => v.net !== nm).concat(saved.vias);
					this.markAllEdgesDone(dn);
					this.callbacks.onNetUpdate?.(
						nm, mergePrims(saved.prims), saved.vias, true, this.padProgress(dn),
					);
				}
			}
			this.droppedNets.clear();
		}

		// 尾轮已收敛但预算还有余量时，用剩余时间做形态优化（逐网重布，
		// 更短且不新增过孔才采纳）。放在尾轮之后是关键：尾轮之前的优化
		// 会挤占连通率所需时间（实测 120s: 51 vs 52）。
		if ((globalThis as any).__RB_OPT_TAIL__ !== false && this.planOptions.optimize === true &&
			!this.checkCancelled() && Date.now() < this.planDeadline - 15_000) {
			const remain = this.planDeadline - Date.now();
			this.optDeadline = Math.min(this.planDeadline - 5_000, Date.now() + remain * 0.5);
			this.log(`尾部余量 ${Math.round(remain / 1000)}s → 形态优化`, 'info');
			await this.optimizeRoutes(networks);
		}

		if ((globalThis as any).__RB_AUDIT_FINAL__) {
			(globalThis as any).__RB_FINAL_STATE__ = {
				vias: this.plannedVias.map((v: PlannedVia) => ({ net: v.net, x: +v.x.toFixed(2), y: +v.y.toFixed(2), dia: v.dia })),
				v5v: (this.netRendered.get('+5V') ?? []).map((p: RenderedPrim) => p.kind === 'line' ? [p.layer, +p.x1!.toFixed(1), +p.y1!.toFixed(1), +p.x2!.toFixed(1), +p.y2!.toFixed(1)] : [p.layer, 'arc']),
				netCount: this.netRendered.size,
			};
		}

		// 终局过孔净距修复：回滚恢复可能把旧图元放回已变化的世界（绕过放置
		// 校验），最后统一复验并拆除违规过孔（详见 repairViaClearance 注释）。
		// final=true：此时已没有重布阶段，拆除后不连通也保留剩余部分走线。
		this.repairViaClearance(true);

		if (this.edgeCacheStats.hit || this.edgeCacheStats.stale) {
			this.log(`单边复用缓存：命中 ${this.edgeCacheStats.hit} 次，失效 ${this.edgeCacheStats.stale} 次`, 'info');
		}

		// 收尾剪枝：删掉不接任何焊盘的悬空碎片（部分布通留下的孤岛）
		this.pruneDanglingFragments();

		// 橡皮筋拉紧：同层链上隔点直连可行即删顶点（网格楼梯/拼接折弯拉直）
		{
			const tautened: string[] = [];
			for (const nm of [...this.netRendered.keys()]) {
				if (this.tautenNet(nm)) tautened.push(nm);
			}
			if (tautened.length) {
				this.log(`橡皮筋拉紧：${tautened.length} 个网络删顶点拉直`, 'info');
				for (const nm of tautened) {
					const ps = this.netRendered.get(nm) ?? [];
					this.callbacks.onNetUpdate?.(
						nm, mergePrims(ps), this.plannedVias.filter(v => v.net === nm),
						ps.length > 0, undefined,
					);
				}
			}
		}

		// 收尾形态整备：跨尝试接缝的圆角化（净距逐点复核，不产 DRC 违规）
		this.cosmeticPass();

		// **焊盘覆盖终审**：routedOrder 只看 routedEdges 集合，而几何改写类
		// pass（拉直/优化/回滚恢复）从不复查「每个 pad 是否仍被铜皮接到」。
		// 实测 P02 被 markAllEdgesDone 谎报 + 拉直丢锚点后，布通网在 EDA 里
		// 是 pad 悬空的（Connection Error）。谓词用**铜皮级连通**
		//（netCopperConnected）—— netIsConnected 的端点量化图会把圆角接缝
		// 的 ≤2mil 端点错位误判成断连（铜皮上完全导通），整批误杀合法网
		//（实测 57→36）。不连通的降级为失败（残段保留 = 失败网语义），
		// 宁可少报布通也不给画布留假布通。
		for (const nm of [...this.routedOrder]) {
			// 单焊盘网：MST 无边，「零几何布通」是正确语义，跳过终审
			//（实测 P12-P37/LCD7 这批单 pad 网被 ps.length===0 误降级 13 个）
			if (this.allPads.filter(p => p.net === nm).length <= 1) continue;
			const ps = this.netRendered.get(nm) ?? [];
			const vs = this.plannedVias.filter(v => v.net === nm);
			if (ps.length && this.netCopperConnected(nm, ps, vs)) continue;
			const i = this.routedOrder.indexOf(nm);
			if (i >= 0) this.routedOrder.splice(i, 1);
			this.routedEdges.delete(nm);
			if ((globalThis as any).__RB_AUDIT_COPPER__) {
				// 诊断：列出各焊盘的连通分量归属与最近图元距离
				const pads = this.allPads.filter(p => p.net === nm);
				const pdDesc = pads.slice(0, 6).map(pd => {
					let best = 1e9, bestWhat = '';
					for (const p of ps) {
						if (p.layer !== pd.layer && pd.layer !== 12) continue;
						if (p.kind === 'line') {
							const d = Math.min(
								Math.hypot(p.x1! - pd.x, p.y1! - pd.y),
								Math.hypot(p.x2! - pd.x, p.y2! - pd.y),
								pointSegDist(pd.x, pd.y, p.x1!, p.y1!, p.x2!, p.y2!));
							if (d < best) { best = d; bestWhat = `line@(${p.x1!.toFixed(0)},${p.y1!.toFixed(0)})`; }
						}
					}
					for (const v of vs) {
						const d = Math.hypot(v.x - pd.x, v.y - pd.y);
						if (d < best) { best = d; bestWhat = 'via'; }
					}
					return `${pd.id.slice(-4)}L${pd.layer}:最近=${best.toFixed(1)}mil(${bestWhat})body=${(pd.bodyRadius || pd.radius).toFixed(1)}`;
				});
				this.log(`[审计诊断] ${nm}: ${pdDesc.join(' | ')}`, 'warn');
			}
			this.log(`终审: ${nm} 焊盘覆盖不完整（几何与账目不符），降级为失败`, 'warn');
		}

		// 终局审计（诊断）：用与放置同口径的模型复算「过孔 ↔ 异网走线/过孔」
		// 间距，把绕过了放置校验的配对暴露出来（谁先谁后、哪个阶段引入）。
		if ((globalThis as any).__RB_AUDIT_FINAL__) {
			const viaR = Math.max(24, this.drc.traceWidthMil * 2) / 2;
			for (const v of this.plannedVias) {
				for (const [name, prims] of this.netRendered) {
					if (name === v.net) continue;
					for (const p of prims) {
						if (p.kind !== 'line') continue;
						const d = pointSegDist(v.x, v.y, p.x1!, p.y1!, p.x2!, p.y2!);
						const need = viaR + this.drc.trackToPadMil + this.drc.traceWidthMil / 2 + 1.2;
						if (d < need) {
							this.log(`[审计] 过孔 ${v.net}(${v.x.toFixed(0)},${v.y.toFixed(0)}) ↔ ${name} 走线 中心距 ${d.toFixed(2)} < ${need.toFixed(2)}`, 'warn');
						}
					}
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
	/**
	 * 总线束重排：识别「同向 + 同源行 + 同目标带」的两焊盘网聚成束，
	 * 束内按目标横向位置排序（逃逸不交叉），大束优先置前；其余网保持
	 * 原相对顺序。无束则返回 null（不重排）。
	 */
	private reorderByBundles(networks: Array<{ name: string; pads: EnginePad[] }>): Array<{ name: string; pads: EnginePad[] }> | null {
		const PAR = 0.26;    // ≈15°：方向平行阈值（|cross|/|d1||d2|）
		const SRC_BAND = 130;  // 源焊盘垂直偏移带宽（mil）——同一「行」
		const DST_BAND = 220;  // 目标焊盘垂直偏移带宽（mil）
		const two = networks.filter(n => n.pads.length === 2);
		if (two.length < 3) return null;
		const vecOf = (n: { pads: EnginePad[] }): { dx: number; dy: number; len: number } => {
			const dx = n.pads[1].x - n.pads[0].x, dy = n.pads[1].y - n.pads[0].y;
			const len = Math.hypot(dx, dy) || 1;
			return { dx, dy, len };
		};
		const perpOf = (p: EnginePad, o: { x: number; y: number }, dir: { dx: number; dy: number; len: number }): number =>
			((p.x - o.x) * dir.dy - (p.y - o.y) * dir.dx) / dir.len;
		const used = new Set<string>();
		const bundles: Array<Array<{ name: string; pads: EnginePad[] }>> = [];
		for (const seed of two) {
			if (used.has(seed.name)) continue;
			const sv = vecOf(seed);
			if (sv.len < 150) continue;   // 太短的网不构成走廊束
			const members: Array<{ name: string; pads: EnginePad[] }> = [seed];
			used.add(seed.name);
			for (const cand of two) {
				if (used.has(cand.name)) continue;
				const cv = vecOf(cand);
				const cross = Math.abs(sv.dx * cv.dy - sv.dy * cv.dx) / (sv.len * cv.len);
				const dot = (sv.dx * cv.dx + sv.dy * cv.dy) / (sv.len * cv.len);
				if (cross > PAR || dot < 0.9) continue;
				// 同源行 + 同目标带（垂直于束方向的偏移都在带宽内）
				if (Math.abs(perpOf(cand.pads[0], seed.pads[0], sv)) > SRC_BAND) continue;
				if (Math.abs(perpOf(cand.pads[1], seed.pads[1], sv)) > DST_BAND) continue;
				members.push(cand);
				used.add(cand.name);
			}
			if (members.length >= 3) bundles.push(members);
			else members.forEach(m => used.delete(m.name));   // 不成束的放回
		}
		if (!bundles.length) return null;
		// 束内按目标端横向坐标排序（垂直于束方向），逃逸天然不交叉
		for (const b of bundles) {
			const dir = vecOf(b[0]);
			b.sort((a, c) => perpOf(a.pads[1], b[0].pads[1], dir) - perpOf(c.pads[1], b[0].pads[1], dir));
		}
		bundles.sort((a, c) => c.length - a.length);   // 大束优先
		const front = new Map<string, { name: string; pads: EnginePad[] }>();
		for (const b of bundles) for (const m of b) front.set(m.name, m);
		const rest = networks.filter(n => !front.has(n.name));
		return [...bundles.flat(), ...rest];
	}

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

		// __RB_ROW_FIRST__=1：密排焊盘排（连接器/IC 引脚排，间距 ≤47mil）的网络
		// 提到本层最前 —— 对齐内置布线器的 escape-first：整排先垂直出线占住
		// 车道，后布网从旁边绕，而不是先布网横穿扇出带、让排网全程靠撕毁救。
		// 稳定排序只动信号层（0/1）内部顺序，电源/地的层级约束不变。
		if ((globalThis as any).__RB_ROW_FIRST__ && (mode === 'auto' || mode === 'river')) {
			const isRowNet = (k: (typeof keyed)[number]): boolean => {
				if (k.tier >= 2) return false;
				return k.net.pads.some(p => {
					const r = this.rowInfo.get(p.id);
					return !!r && r.gap <= 47;
				});
			};
			keyed.sort((a, b) => (a.tier - b.tier) || ((isRowNet(b) ? 0 : 1) - (isRowNet(a) ? 0 : 1)));
		}

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
		const byInteriority = (a: T, b: T) => {
			const s = (rowScore.get(b.net.name) ?? 0) - (rowScore.get(a.net.name) ?? 0);
			// __RB_ROW_EDGE_FIRST__=1：端边优先（平面扇实验）——目的地序与焊盘序
			// 一致的排，端边先走、逐次向内，天然无交叉；中间优先会制造交叉墙
			return (globalThis as any).__RB_ROW_EDGE_FIRST__ ? -s : s;
		};
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
		// cutoff = 本动作（单网/单边）的截止：超时按「取消」处理让求解器尽快退出，
		// 但不置 this.cancelled（整轮还没结束，下一个动作要继续）
		if (Date.now() > this.cutoff) return true;
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

	/**
	 * 进度百分比口径：**已整网布通的网络数 / 总网络数**。
	 * 旧实现按「处理到第几个网」算（20 + index/total×55），处理完快扫后
	 * 无论布通多少都显示到 75%，用户看到画布上大量飞线却显示 88%。
	 */
	private routedPct(networks: Array<{ name: string }>): number {
		const total = Math.max(1, networks.length);
		return Math.min(99, Math.floor((this.routedOrder.length / total) * 100));
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
		const bw = mxx - mnx, bh = mxy - mny;
		// 默认丢弃全部内部板框圈线：保留内腔/开槽会把板面切成孤岛，实测
		// case12 布通从 64 掉到 54（10 个网被开槽挡死）。需要严格避开开槽时
		// 用 __RB_KEEP_CUTOUTS__=1 打开（局部闭合线圈保留、跨板假墙丢弃）。
		if ((globalThis as any).__RB_DROP_CUTOUTS__ === true) {
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
		// 内部圈线分两类：
		//  - 拼版 V-CUT / 网格线：跨越板面大范围，是「假墙」，必须丢弃
		//    （F407 实测把板面切成互不连通条带，跨条带网数学无解）；
		//  - 真实内腔/开槽（摄像头开口、USB 让位）：局部闭合线圈，必须保留
		//    （否则走线穿过板框开槽，EDA 报 Board Outline to Track）。
		// 用空间聚类 + 包围盒跨度区分：跨度超过板面 60% 的簇视为假墙。
		const internal: Array<{ o: RBObstacle; idx: number }> = [];
		const keep: boolean[] = new Array(obs.length).fill(true);
		for (let i = 0; i < obs.length; i++) {
			const o = obs[i];
			if (o.group !== 'board') continue;
			const de = Math.min(o.x - mnx, mxx - o.x, o.y - mny, mxy - o.y);
			if (de <= BAND) continue;         // 外轮廓：保留
			internal.push({ o, idx: i });
		}
		if (internal.length) {
			const parent = new Int32Array(internal.length);
			for (let i = 0; i < parent.length; i++) parent[i] = i;
			const find = (x: number): number => {
				let r = x;
				while (parent[r] !== r) r = parent[r];
				let c = x;
				while (parent[c] !== c) { const p = parent[c]; parent[c] = r; c = p; }
				return r;
			};
			const uni = (a: number, b: number): void => { const ra = find(a), rb = find(b); if (ra !== rb) parent[ra] = rb; };
			for (let i = 0; i < internal.length; i++) {
				for (let j = i + 1; j < internal.length; j++) {
					const a = internal[i].o, b = internal[j].o;
					if (Math.hypot(a.x - b.x, a.y - b.y) <= a.r + b.r + 10) uni(i, j);
				}
			}
			const clusters = new Map<number, { minx: number; maxx: number; miny: number; maxy: number; members: number[] }>();
			for (let i = 0; i < internal.length; i++) {
				const r = find(i);
				const o = internal[i].o;
				let c = clusters.get(r);
				if (!c) { c = { minx: Infinity, maxx: -Infinity, miny: Infinity, maxy: -Infinity, members: [] }; clusters.set(r, c); }
				c.minx = Math.min(c.minx, o.x - o.r); c.maxx = Math.max(c.maxx, o.x + o.r);
				c.miny = Math.min(c.miny, o.y - o.r); c.maxy = Math.max(c.maxy, o.y + o.r);
				c.members.push(internal[i].idx);
			}
			let dropped = 0;
			for (const c of clusters.values()) {
				const spansBoard = (c.maxx - c.minx) > bw * 0.6 || (c.maxy - c.miny) > bh * 0.6;
				if (spansBoard) {
					for (const idx of c.members) keep[idx] = false;
					dropped += c.members.length;
				}
			}
			if (dropped) {
				this.log(`板框层内部圈线 ${dropped} 处跨板（拼版/V-CUT 线），按非电气障碍忽略；其余 ${internal.length - dropped} 处内部开槽保留`, 'info');
			}
		}
		return obs.filter((_, i) => keep[i]);
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
				holeRotation: cached?.holeRotation ?? 0,
			};
			for (const c of padShapeToCircles(shape)) {
				out.push({
					id: `p${id++}`, kind: 'circle', x: p.x + c.x, y: p.y + c.y, r: c.r + infl,
					net: p.net, layer, group, ...(c.blockerOnly ? { blockerOnly: true } : {}),
				});
			}
			// 钻孔禁入区：NPTH/孔大于铜皮时铜皮障碍覆盖不到孔，走线/过孔会压钻孔
			//（EDA 报 Hole to Track / Hole to Via / Hole to Hole）。
			// SLOT 孔取长边（hole[1] 是短边宽，长槽沿孔轴延伸 ±hole[2]/2）。
			const holeD = Math.max(Number(cached?.hole?.[1]) || 0, Number(cached?.hole?.[2]) || 0) || p.holeDiameter || 0;
			if (holeD > 0) {
				out.push({
					id: `h${id++}`, kind: 'circle', x: p.x, y: p.y,
					r: holeD / 2 + Math.max(drc.trackToPadMil, 6) + 0.3,
					net: p.net, layer: undefined, group, blockerOnly: true,
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
			holeRotation: cached?.holeRotation ?? 0,
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
		// 大焊盘会横跨多格 → 同一焊盘可能被多个邻格命中，按 id 去重
		const seen = new Set<string>();
		const out: EnginePad[] = [];
		for (let i = gx - 1; i <= gx + 1; i++) {
			for (let j = gy - 1; j <= gy + 1; j++) {
				const b = this.padGrid.get(`${i},${j}`);
				if (!b) continue;
				for (const p of b) {
					if (seen.has(p.id)) continue;
					seen.add(p.id);
					out.push(p);
				}
			}
		}
		return out;
	}

	/**
	 * 焊盘外形的外接半对角线（对任意旋转的保守上界）。
	 * ⚠ 必须与 padCopperDistance 同一口径读 padShapes 缓存：形状字段缺失的
	 * 焊盘 p.radius 回退成 32×34 半值（~17mil），而邻近缓存会把真实长盘形状
	 * （LED/LCD 连接器 300mil）补给 padCopperDistance —— 用 p.radius 做预筛/
	 * 入格时，长盘远端对过孔校验整块隐形（实测过孔放到距 LED2/LCD4 焊盘
	 * 17.94mil 处放置放行、终检才被拆，网降级失败）。
	 */
	private padHalfExtent(p: EnginePad): number {
		let half = Math.max(p.radius, p.bodyRadius ?? 0, p.holeDiameter / 2);
		const cached = this.padShapes[`${p.x.toFixed(2)},${p.y.toFixed(2)}`];
		const sh = cached?.pad;
		if (Array.isArray(sh)) {
			if (sh[0] === 'POLYGON') {
				const r = polygonToRect(sh);
				if (r) half = Math.max(half, Math.hypot(r.w, r.h) / 2);
			} else {
				const w = Number(sh[1]) || 0;
				const h = Number(sh[2]) || 0;
				if (w > 0 && h > 0) half = Math.max(half, Math.hypot(w, h) / 2);
			}
		}
		// SLOT 孔取长边（长槽沿孔轴延伸）
		const hd = Math.max(Number(cached?.hole?.[1]) || 0, Number(cached?.hole?.[2]) || 0);
		if (hd > 0) half = Math.max(half, hd / 2);
		// SLOT 焊盘保守覆盖的对角半extent（OVAL 维按半extent 解读，见 slotCoverHalfExtents）
		const slot = slotCoverHalfExtents({ pad: sh ?? [], hole: cached?.hole ?? null, holeRotation: cached?.holeRotation ?? 0 });
		if (slot) half = Math.max(half, Math.hypot(slot.hx, slot.hy));
		return half;
	}

	/** padHalfExtent 的记忆化版本：viaPadClear 每候选点 × 每焊盘都要调用，
	 * 形状查找/开方成本不可忽略（焊盘静态，整轮缓存有效） */
	private padExtentOf(p: EnginePad): number {
		const key = p.index ?? -1;
		if (key >= 0) {
			const hit = this.padExtentCache.get(key);
			if (hit !== undefined) return hit;
		}
		const e = this.padHalfExtent(p);
		if (key >= 0) this.padExtentCache.set(key, e);
		return e;
	}

	/**
	 * 焊盘可落线层：TH（layer 12 / padType th）双层；SMD 只能自己的层。
	 * 网格求解必须按它限制起点/终点的层，否则顶层 SMD 会直接生成没有
	 * 过孔的底层走线（EDA 判「底层导线未打孔 → 断网」）。
	 */
	private padLayers(p: EnginePad): number[] {
		if (p.layer === 12 || p.padType === 'th') return this.hasBottomLayer ? [1, 2] : [1];
		if (p.layer === 2 && this.hasBottomLayer) return [2];
		return [1];
	}

	/** 焊盘禁孔半径（via-in-pad 关闭时）：过孔中心必须离焊盘铜皮 ≥ viaR + trackToPad */
	private padViaKeepout(p: EnginePad): number {
		if (this.allowViaInPad) return 0;
		const viaR = Math.max(24, this.drc.traceWidthMil * 2) / 2;
		return this.padHalfExtent(p) + viaR + this.drc.trackToPadMil + 0.5;
	}

	/**
	 * 网格求解的焊盘解禁圆：覆盖焊盘整条铜皮（形状缓存里的圆群），
	 * 不是只解禁中心一个小圆。长焊盘（0.5mm 排的 45×10mil 竖条）用中心
	 * 圆解禁时，起点会被左右邻盘的净距圈困在 12mil 的「口袋」里，
	 * A* 只能弹出个位数节点（case12 实测 pops=8）。
	 */
	private padReleases(p: EnginePad, cell: number): Array<{ x: number; y: number; r: number }> {
		const cached = this.padShapes[`${p.x.toFixed(2)},${p.y.toFixed(2)}`];
		const shape = {
			pad: cached?.pad ?? ['ELLIPSE', p.radius * 2, p.radius * 2],
			hole: cached?.hole ?? null,
			rotation: cached?.rotation ?? p.rotation ?? 0,
			holeRotation: cached?.holeRotation ?? 0,
		};
		let circles = padShapeToCircles(shape as any);
		if (!circles.length) circles = [{ x: 0, y: 0, r: p.bodyRadius || p.radius }];
		// 圆过多（POLYGON 长盘）时等距抽样，避免解禁循环过大
		if (circles.length > 12) {
			const step = Math.ceil(circles.length / 12);
			circles = circles.filter((_, i) => i % step === 0);
		}
		const rr = 1.0;
		const out = circles.map(c => ({ x: p.x + c.x, y: p.y + c.y, r: c.r + rr }));
		// 保证种子所在格（焊盘中心最近格，最坏偏 5.7mil）可通行；半径取 6mil，
		// 远小于旧实现的 bodyRadius+cell（长条盘会多解禁 ~8mil 的侧向空区，
		// A* 走斜线切进邻盘净距圈，最后被 segClear 判死整条边）。
		out.push({ x: p.x, y: p.y, r: 6.0 });
		return out;
	}

	/** 网格求解的过孔额外净空（与 viaObstacleClear / viaPadClear 同口径） */
	private gridViaExtras(): { viaExtraMil: number; viaExtraPadMil: number } {
		// 走线障碍半径 = 半线宽+tt+半线宽+0.1；过孔障碍半径 = viaR+max(tt,tp)+半线宽+VIA_WALL_EXTRA
		const viaR = Math.max(24, this.drc.traceWidthMil * 2) / 2;
		const halfW = this.drc.traceWidthMil / 2;
		return {
			viaExtraMil: viaR - halfW + Math.max(this.drc.trackToTrackMil, this.drc.trackToPadMil)
				- this.drc.trackToTrackMil + VIA_WALL_EXTRA - 0.1,
			// 与 viaPadClear 的口径对齐：pad 障碍 = 铜皮 + tp + halfW + 0.3，
			// viaPadClear 要求 铜皮 + viaR + tp + 1.2 → 额外补 viaR-halfW+0.9
			//（旧值 +0.1 少 0.8mil，网格放过、viaPadClear 判死，case12 SD_CMD 实锤）。
			viaExtraPadMil: viaR - halfW + 0.9,
		};
	}

	private viaPadClear(x: number, y: number, viaR: number, netName: string): boolean {
		const drc = this.drc;
		// 外接圆预筛：外接圆之外的焊盘铜皮不可能违反净距，直接跳过 ——
		// 落点网格/射线扫描每个候选点都要过一遍邻近焊盘，形状求距
		//（旋转/多边形）才是大头，预筛能把绝大多数焊盘挡在求距之外。
		// ⚠ 必须全量扫 allPads，不能用 nearPads 3×3 网格邻域：长焊盘
		//（case9 LCD/LED 陶瓷灯丝盘 16.5×65.7mil）在网格索引里有盲区，
		// 提交放过、终检 badAt（全量扫）再拆 —— case9 单轮 81 次焊盘类
		// 微移/拆除、5 个网被清空全靠救援轮兜底。预筛在外，成本可控
		//（hypot+比较 ~10ns/盘，形状求距只发生在预筛通过的少数盘上）。
		//（焊盘全集先经 64mil 空间网格收窄到 3×3 邻域，全量扫描 O(盘数)
		// × 数千候选点在密集板上纯烧时间）
		const need0 = viaR + drc.trackToPadMil + 1;
		for (const p of this.allPads) {
			// 过孔是贯穿孔：只关心顶层/底层/通孔焊盘
			if (p.layer !== 1 && p.layer !== 2 && p.layer !== 12 && p.padType !== 'th') continue;
			const sameNet = p.net === netName;
			if (sameNet && this.allowViaInPad) continue;
			// ⚠ 预筛外接半径必须用真实形状的对角半extent（padHalfExtent），
			// 不能用 p.radius（=max(w,h)/2，RECT 角部方向低估 ~6mil）：
			// case9 实测 LED 排 RECT 59.1×39.4 焊盘，过孔落在角部方向
			// 距铜皮 17.94mil（EDA 要求 17.98，差 0.04），预筛按 29.55 算
			// 50.92-29.55=21.37>18.98 直接跳过精确检查 → 提交漏检、终检再拆。
			// 预筛只许偏松（多进精确检查），绝不许偏紧。
			const maxExtent = Math.max(this.padExtentOf(p), p.radius, p.bodyRadius ?? 0);
			if (Math.hypot(p.x - x, p.y - y) - maxExtent > need0) continue;
			const d = this.padCopperDistance(p, x, y);
			const need = sameNet
				? viaR + 4
				: viaR + drc.trackToPadMil + 1.2;
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
			holeRotation: cached?.holeRotation ?? 0,
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

	/** 过孔 → 动态障碍（两层通挡；半径与主管线过孔入账同式） */
	private viaObstacleOf(v: PlannedVia): RBObstacle {
		return {
			id: `via${this.obstacleIdCounter++}`, kind: 'circle', x: v.x, y: v.y,
			r: v.dia / 2 + Math.max(this.drc.trackToTrackMil, this.drc.trackToPadMil) + this.drc.traceWidthMil / 2 + VIA_WALL_EXTRA,
			net: v.net, layer: undefined, group: `via:${v.net}:${v.x},${v.y}`, noPrune: true,
		};
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
			// 防御：坐标不全的障碍不进去重键（toFixed 会崩）、也不进世界——
			// 引擎自建障碍必然齐备，脏的只可能来自画布收集的静态世界
			if (!o || !Number.isFinite(o.x) || !Number.isFinite(o.y) || !Number.isFinite(o.r)) continue;
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
	/** 单网限时：取「阶段软截止」与「本网起算 netCapMs」的较早者。0/负数表示不限（调试） */
	private netSoft(soft: number): number {
		const cap = typeof process !== 'undefined' ? Number(process.env.__RB_NETCAP__) : NaN;
		const ms = Number.isFinite(cap) ? cap : this.netCapMs;
		return ms > 0 ? Math.min(soft, Date.now() + ms) : soft;
	}

	/**
	 * 布网入口：把本动作的截止时间挂到 cutoff 上，求解器内部的
	 * checkCancelled() 才能及时退出（否则单条边里的十几个候选会连跑几分钟，
	 * 单网限时形同虚设 —— 实测 F407 30 分钟预算下一轮仍烧 1320s）。
	 */
	private async routeNetEdges(net: { name: string; pads: EnginePad[] }, mode: 'full' | 'quick' = 'full', softDeadline?: number): Promise<boolean> {
		const prev = this.cutoff;
		this.cutoff = softDeadline ?? this.planDeadline;
		const __t0 = Date.now();
		try {
			return await this.routeNetEdgesInner(net, mode, softDeadline);
		} finally {
			this.cutoff = prev;
			if ((globalThis as any).__RB_PROFILE__) {
				this.log(`[prof] ${net.name} ${mode} ${Date.now() - __t0}ms pads=${net.pads.length}`, 'info');
			}
		}
	}

	private async routeNetEdgesInner(net: { name: string; pads: EnginePad[] }, mode: 'full' | 'quick' = 'full', softDeadline?: number): Promise<boolean> {
		const drc = this.drc;
		// 软截止：调用方（撕毁动作/成组重布）用它的目的 = 给尾部阶段
		//（网格抢救等）留燃料；不传则用整轮绝对 deadline
		const edgeDeadline = softDeadline ?? this.planDeadline;
		// 过孔候选宽度：实测 case9 窄搜索 50/64 vs 宽（12）42/64 —— 宽搜索单网慢 3 倍、
		// 还改变世界演化；F407 两端同为 62，收益在噪声内。默认 8，保留 env 供 A/B。
		const __vb = typeof process !== 'undefined' ? Number(process.env.__RB_VIABREADTH__) : NaN;
		this.viaBreadth = Number.isFinite(__vb) && __vb > 0 ? __vb : this.tailWide ? 12 : 8;
		const edges = this.computeMST(net.pads);
		// 注：曾试验「短边先布」改边序 —— 隔离板 9 → 6 回归（边序扰动改变
		// 各边走廊争夺结果），且目标问题（同走廊反平行）并未解决，已回退。
		const edgeKey = (si: number, di: number) => `${net.pads[si].id}->${net.pads[di].id}`;
		// 部分连通的网络在撕毁/重试之间保留已完成的连接，只补未完成的边；
		// 否则重试会重复布线并叠加过孔。
		const done = this.routedEdges.get(net.name) ?? new Set<string>();
		// 本次尝试的图元/过孔先暂存：成功一条提交一条，失败连接不再拖累整网。
		const attemptRendered: RenderedPrim[] = [];
		const attemptVias: PlannedVia[] = [];

		for (const [si, di] of edges) {
			if (this.checkCancelled() || Date.now() > edgeDeadline) return false;
			// 预算纪律：单网重布（尤其带网格兜底的重试）不应无限推进，
			// 否则收尾阶段会把整轮预算顶穿（实测超时 10 分钟以上）
			if (Date.now() > edgeDeadline) return false;
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

			// 单边结果缓存（只缓存无过孔方案）：撕毁重布轮会把同一对焊盘
			// 反复重解。命中后先按「当前世界」复验净距，仍合法就直接复用。
			const cacheKey = `${net.name}|${src.id}|${dst.id}|${routingLayer}`;
			if (!crossLayerSmd) {
				const hit = this.edgeSoloCache.get(cacheKey);
				if (hit) {
					if (this.countPadViolations(hit, net.name) === 0 && this.primsClearOfDynamic(hit, net.name)) {
						rendered = hit.map(p => ({ ...p }));
						this.edgeCacheStats.hit++;
					} else {
						this.edgeCacheStats.stale++;
					}
				}
			}
			if (!rendered) {
			if (mode === 'quick') {
				// 快扫只求直连；跨层 SMD 对必须过孔换层（无单层路径），留给收尾轮
				if (crossLayerSmd && !this.allowViaInPad) continue;
				// B 实验：迷宫快扫（__RB_MAZE_SWEEP__）——细格 + 出线候选点起步，
				// 测「一致网格世界」的速度与完成率；__RB_MAZE_FALLBACK__ 控制失配
				// 后是否回退可视图（默认不回退 = 纯迷宫世界口径）
				if ((globalThis as any).__RB_MAZE_SWEEP__) {
					const g = await this.mazeSweepEdge(src, dst, net.name);
					if (g) {
						rendered = g.prims;
						edgeVias.push(...g.vias);
					} else if (!(globalThis as any).__RB_MAZE_FALLBACK__) {
						continue;   // 纯迷宫口径：失败不回退可视图
					}
				}
				// 快扫只做「直线能不能过」：非直连边留给撕毁重布轮做完整求解。
				// 旧实现默认走完整可视图（每边建一次切线图），实测首轮快扫 18s；
				// 改直连快判后 0.3s，整板 81s → 54s，布通数不变。
				// 需要旧口径做 A/B 时设 __RB_SWEEP_FULL__=1。
				if (!rendered) rendered = await this.directEdgeRoute(
					src, dst, net.name, routingLayer, undefined,
					!(globalThis as any).__RB_SWEEP_FULL__, this.sweepEdgeBudget,
				);
				if (!rendered) continue;
			} else if (crossLayerSmd && !this.allowViaInPad) {
				// 顶层 SMD 引出段 + 焊盘外过孔 + 底层路径；不会在任一焊盘上打孔
				const viaCand = await this.viaEdgeCandidate(src, dst, net.name, attemptRendered);
				if (viaCand) {
					rendered = viaCand.prims;
					edgeVias.push(...viaCand.vias);
				}
			} else {
				// 网格优先（默认开启，含过孔，10~300ms/边）：大板/长线场景下
				// 比可视图建图快得多。成功即用，失败才走可视图完整链。
				// 关掉做 A/B：__RB_NO_GRID_PRIMARY__=1。
				// 直连快判：同层两焊盘之间直线已满足净距时，直线就是全局最短解
				//（端点代价固定），直接采纳 —— 跳过整张网格 A*（10~300ms/边）。
				// 实测单板 1321 次网格调用里绝大多数边可直连，这是最大单点提速。
				if (!(globalThis as any).__RB_NO_DIRECT_FIRST__ && srcLayer === dstLayer &&
					this.directLineClear(src, dst, routingLayer, net.name)) {
					rendered = [{ kind: 'line', net: net.name, layer: routingLayer, x1: src.x, y1: src.y, x2: dst.x, y2: dst.y }];
					if ((globalThis as any).__RB_PROFILE__) this.log(`[prof] ${net.name}: 直连快判命中 ${src.id}→${dst.id}`, 'info');
				}
				if (!rendered && (!(globalThis as any).__RB_NO_GRID_PRIMARY__ || (globalThis as any).__RB_GRID_ONLY__)) {
					const g = await this.gridFallbackEdge(src, dst, net.name);
					if (g) { rendered = g.prims; edgeVias.push(...g.vias); }
				}
				if (!rendered && !(globalThis as any).__RB_GRID_ONLY__) {
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
				// 就垂直于排的直连保持原样 —— 切线出发拐角天然平滑，也省一次求解。
				// 注：dst 端预留 / 密排补试候选两个分支实测引发隔离板 9→6 级联
				// 回归（额外求解改变各网走廊争夺），已回退 —— case12 发卡属
				// 拥塞相关的局部最优，留待路径级美学（折返惩罚）统一处理。
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
						const viaCand = await this.viaEdgeCandidate(src, dst, net.name, attemptRendered);
						if (viaCand) { altPrims = viaCand.prims; altVias = viaCand.vias; }
					}
					// 换层方案明显更短才采纳（过孔有工艺代价，省 50mil 不值得打孔）
					if (altPrims && (!rendered || renderedLength(altPrims) < renderedLength(rendered) - 50)) {
						rendered = altPrims;
						edgeVias.push(...altVias);
					}
				}

				// 两个单层都走不通 → 才用过孔换层兜底
				// 实验开关：先用网格兜底（含过孔，10~300ms/边）再考虑重型过孔候选链。
				// 动机：失败边往往在 viaEdgeCandidate 里跑 1~3s 才失败，网格通常更快
				// 且能直接给出过孔路径；代价是形态不如可视图（45° 拉直）。
				if (!rendered && this.hasBottomLayer && (globalThis as any).__RB_GRID_GATE__) {
					const g = await this.gridFallbackEdge(src, dst, net.name);
					if (g) { rendered = g.prims; edgeVias.push(...g.vias); }
				}
				if (!rendered && this.hasBottomLayer) {
					const viaCand = await this.viaEdgeCandidate(src, dst, net.name, attemptRendered);
					if (viaCand) {
						rendered = viaCand.prims;
						edgeVias.push(...viaCand.vias);
					}
				}

				// 跨层 SMD 对（顶层 ↔ 底层）：路径落在其中一层的焊盘中心，
				// 另一层焊盘靠过孔连接。该过孔位于焊盘内，仅在允许焊盘打孔时使用
				// （默认路径已在上面的 crossLayerSmd 分支改为焊盘外过孔）。
				if (rendered && srcLayer !== dstLayer) {
					// 与全引擎统一的标准过孔几何（max(24, 2×线宽) / max(12, 线宽)）：
					// 旧值 dia=max(20,·)/hole=max(10,·) 产出 20mil 异类过孔 ——
					// ① hole=10mil=0.254mm 低于板厂最小钻孔 0.3mm；
					// ② 终检按统一 viaR=12 判它，把 EDA 合法的 dia-20 过孔
					//（17.46/17.94mil 距焊盘）当违规微移/拆除（case9 单轮 84 次）。
					const dia = Math.max(24, drc.traceWidthMil * 2);
					const hole = Math.max(12, drc.traceWidthMil);
					// 焊盘内过孔同样要避开隔壁异网焊盘；避不开就放弃这条连接
					//（精确口径与终检一致：采样圆障碍有缝，见 viaExactBad 注释）
					if (!this.viaPadClear(dst.x, dst.y, dia / 2, net.name) ||
						this.viaExactBad(dst.x, dst.y, net.name, dia / 2, hole)) {
						rendered = null;
						edgeVias.length = 0;
					} else {
						edgeVias.push({ net: net.name, x: dst.x, y: dst.y, dia, hole });
					}
				}
				}
			}
			}

			// 网格兜底：可视图失败后接管。重试次数不限 —— 实测收益正来自
			// 「每轮盘面变化后再试」（限 3 次时基准板 9/14→7/14）；总代价由
			// gridBudgetMs（整轮预算的 20%）封顶，配合逐边 deadline 检查兜底。
			if (!rendered && mode !== 'quick') {
				const g = await this.gridFallbackEdge(src, dst, net.name);
				if (g) {
					rendered = g.prims;
					edgeVias.push(...g.vias);
				}
			}

			if (!rendered) {
				if ((globalThis as any).__RB_BLOCKERS__) this.logEdgeBlockers(src, dst, net.name);
				this.log(`网络 ${net.name}: ${src.id}(${src.x.toFixed(0)},${src.y.toFixed(0)}) → ${dst.id}(${dst.x.toFixed(0)},${dst.y.toFixed(0)}) 未找到路径`, 'warn');
				continue;   // 其余连接继续布，已布部分不回滚
			}

			// 提交前互检：新图元必须与已布铜皮保持间距（记账漏洞安全网，
			// 实测迷宫快扫模式两根异网走线同层交叉 0.2mil 漏网）
			if (!this.primsClearOfDynamic(rendered, net.name)) {
				this.log(`网络 ${net.name}: ${src.id}→${dst.id} 提交互检发现间距违规，弃用本边`, 'warn');
				continue;
			}

			// 入缓存（只存无过孔方案）：后续撕毁轮重解同一对焊盘时可直接复用
			if (!edgeVias.length && rendered.length <= 64) {
				this.edgeSoloCache.set(cacheKey, rendered.map(p => ({ ...p })));
			}

			attemptRendered.push(...rendered);
			attemptVias.push(...edgeVias);
			// 过孔即时入账（跨边互见）：旧口径整网结束才进 plannedVias，同网
			// 两条边各自落的过孔互相看不见 —— 4.1mil 重叠双双过提交检查，
			// 终检拆除断网（case12 实测 BL_LED_EN/DVP_D1_1/DVP_RSTB 三处，
			// 全是协商重布的多边网）。同边内部由 gridFallbackEdge 的局部
			// vias 查，跨边只能靠 plannedVias。同网检查均豁免自家
			//（primsClearOfDynamic / viaExactBad 的 netName 跳过），即时入账
			// 不会误杀自家连线。
			this.plannedVias.push(...edgeVias);
			for (const v of edgeVias) {
				this.dynamicObstacles.push({
					id: `via${this.obstacleIdCounter++}`, kind: 'circle', x: v.x, y: v.y,
					r: v.dia / 2 + Math.max(this.drc.trackToTrackMil, this.drc.trackToPadMil) + this.drc.traceWidthMil / 2 + VIA_WALL_EXTRA,
					net: net.name, layer: undefined, group: `via:${net.name}:${v.x},${v.y}`, noPrune: true,
				});
			}
			done.add(key);
		}
		this.routedEdges.set(net.name, done);
		const complete = edges.every(([si, di]) => done.has(edgeKey(si, di)));

		// 成功连接立即提交图元、过孔与动态障碍（部分连通也保留在画布上）
		if (attemptRendered.length || attemptVias.length) {
			// 防御：无层图元会让障碍横跨两层、渲染层错乱（fanoutFillet 裸图元
			// 教训）——提交前拦下并告警，不留进板
			const bad = attemptRendered.filter(p => p.layer === undefined);
			if (bad.length) {
				this.log(`网络 ${net.name}: ${bad.length} 个无层图元被拦截（构造缺陷，勿提交）`, 'warn');
				for (const p of bad) attemptRendered.splice(attemptRendered.indexOf(p), 1);
				if (!attemptRendered.length) {
					// 防御路径回滚：本调用即时入账的过孔一并撤回（图元已丢弃）
					const vset = new Set(attemptVias);
					this.plannedVias = this.plannedVias.filter(v => !vset.has(v));
					const vkeys = new Set(attemptVias.map(v => `via:${net.name}:${v.x},${v.y}`));
					this.dynamicObstacles = this.dynamicObstacles.filter(o => !vkeys.has(o.group ?? ''));
					this.routedEdges.set(net.name, done); return false;
				}
			}
			// 先合并再接缝：可视图会在同一障碍圆的多个切点之间切出碎片弧/微段，
			// 直接拿去建障碍会得到一串几乎重合的端点圆与包络圆，后布网络沿着
			// 它们走就会「扇贝式」抖动。合并后每段弧/线只登记一次。
			const mergedEdge = this.smoothCorners(mergePrims(attemptRendered), net.name);
			const rds = this.netRendered.get(net.name) ?? [];
			rds.push(...mergedEdge);
			this.netRendered.set(net.name, rds);
			this.addRenderedToObstacles(mergedEdge);
			// 车道预约回收：行网布通后撤掉自己的 solo 预约障碍（真实走线障碍
			// 已由 addRenderedToObstacles 登记），别让过时的预约墙约束后续网
			if (this.dynamicObstacles.some(o => o.group === `lane:${net.name}`)) {
				this.dynamicObstacles = this.dynamicObstacles.filter(o => o.group !== `lane:${net.name}`);
			}
			// 过孔已在逐边提交时即时入账（plannedVias + 动态障碍），这里不再重复
		}
		if (complete && !this.routedOrder.includes(net.name)) this.routedOrder.push(net.name);
		// 兜底链可能留下冗余过孔回路：提交前净化，实时回调拿到干净铜皮
		this.sanitizeNet(net.name);

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

	/**
	 * 网格抢救阶段：对每个仍有缺失边的网络，逐边用网格 A* 再试一次。
	 *
	 * 设计要点（实测教训）：网格求解的代价是「光栅化 + A*」，若放在
	 * routeNetEdges 的重试链里，同一条边会在每轮重试中反复光栅化，
	 * 大板上直接把整轮预算顶穿（F407 实测 30 分钟预算跑出 40 分钟 CPU）。
	 * 因此改为**一次性**：每条缺失边最多尝试一次，且受 gridBudgetMs 约束。
	 */
	private async gridRescuePhase(networks: Array<{ name: string; pads: EnginePad[] }>): Promise<void> {
		const __gdbg = !!(globalThis as any).__RB_GRID_DEBUG__;
		if (__gdbg) this.log(`[grid-debug] 抢救入口 budget=${Math.round(this.gridBudgetMs / 1000)}s routed=${this.routedOrder.length} nets=${networks.length}`, 'info');
		let __tried = 0, __ok = 0;
		for (const net of networks) {
			if (this.checkCancelled()) break;
			if (this.gridBudgetMs <= 0) break;
			if (Date.now() > this.planDeadline - 5_000) break;
			if (this.routedOrder.includes(net.name)) continue;
			const edges = this.computeMST(net.pads);
			const done = this.routedEdges.get(net.name) ?? new Set<string>();
			const missing = edges.filter(([si, di]) => !done.has(`${net.pads[si].id}->${net.pads[di].id}`));
			if (__gdbg) this.log(`[grid-debug] ${net.name} edges=${edges.length} done=${done.size} missing=${missing.length}`, 'info');
			if (!missing.length) continue;
			let rescued = 0;
			for (const [si, di] of missing) {
				if (this.checkCancelled() || this.gridBudgetMs <= 0) break;
				const src = net.pads[si], dst = net.pads[di];
				if (__gdbg) __tried++;
				const g = await this.gridFallbackEdge(src, dst, net.name);
				if (!g) continue;
				if (__gdbg) __ok++;
				// 提交互检：与 routeNetEdges 同一道安全网
				if (!this.primsClearOfDynamic(g.prims, net.name)) continue;
				const rds = this.netRendered.get(net.name) ?? [];
				const smoothG = this.smoothCorners(mergePrims(g.prims), net.name);
				rds.push(...smoothG);
				this.netRendered.set(net.name, rds);
				this.addRenderedToObstacles(smoothG);
				this.plannedVias.push(...g.vias);
				for (const v of g.vias) {
					this.dynamicObstacles.push({
						id: `via${this.obstacleIdCounter++}`, kind: 'circle', x: v.x, y: v.y,
						r: v.dia / 2 + Math.max(this.drc.trackToTrackMil, this.drc.trackToPadMil) + this.drc.traceWidthMil / 2 + VIA_WALL_EXTRA,
						net: v.net, layer: undefined, group: `via:${v.net}:${v.x},${v.y}`, noPrune: true,
					});
				}
				done.add(`${src.id}->${dst.id}`);
				rescued++;
			}
			if (rescued) {
				this.routedEdges.set(net.name, done);
				const complete = edges.every(([si, di]) => done.has(`${net.pads[si].id}->${net.pads[di].id}`));
				if (complete && !this.routedOrder.includes(net.name)) this.routedOrder.push(net.name);
				this.sanitizeNet(net.name);
				this.callbacks.onNetUpdate?.(
					net.name, mergePrims(this.netRendered.get(net.name) ?? []),
					this.plannedVias.filter(v => v.net === net.name), true, this.padProgress(net, edges),
				);
				if (complete) this.log(`网络 ${net.name} 由网格抢救布通`, 'success');
			}
		}
		if (__gdbg) this.log(`[grid-debug] 抢救结束 tried=${__tried} ok=${__ok} budget=${Math.round(this.gridBudgetMs / 1000)}s`, 'info');
	}

	/**
	 * 网级净化：删除冗余过孔回路。
	 * 兜底链（过孔候选/换层兜底/成组回滚）可能留下「pad→via→pad 小环」：
	 * 过孔及其全部衔接段移除后网络依然全连通，它们就是纯浪费 —— 多余的
	 * 过孔/接缝/铜皮，还占着走线空间。逐过孔试删（并查集连通性判定，
	 * 焊盘按铜皮半径容差接入；TH 焊盘桶壁连通双层），连通即删，直到稳定。
	 */
	private sanitizeNet(name: string): void {
		const prims = this.netRendered.get(name);
		if (!prims || prims.length < 2) return;
		if (!this.plannedVias.some(v => v.net === name)) return;
		const pads = this.allPads.filter(p => p.net === name);
		if (!pads.length) return;

		const round = (v: number) => Math.round(v * 10) / 10;
		const key = (x: number, y: number, layer: number) => `${round(x)}|${round(y)}|${layer}`;
		const endsOf = (p: RenderedPrim): Array<{ x: number; y: number }> => {
			if (p.kind === 'line') return [{ x: p.x1!, y: p.y1! }, { x: p.x2!, y: p.y2! }];
			const a0 = p.startAngle ?? 0, sw = p.sweep ?? 0, r = p.r ?? 0;
			return [
				{ x: (p.cx ?? 0) + r * Math.cos(a0), y: (p.cy ?? 0) + r * Math.sin(a0) },
				{ x: (p.cx ?? 0) + r * Math.cos(a0 + sw), y: (p.cy ?? 0) + r * Math.sin(a0 + sw) },
			];
		};

		// 并查集（trial 图元 + 过孔层桥 + 焊盘铜皮容差接入）
		const parent = new Map<string, string>();
		const find = (k: string): string => {
			let r = k;
			while (parent.get(r) !== r) r = parent.get(r)!;
			let c = k;
			while (parent.get(c) !== c) { const n = parent.get(c)!; parent.set(c, r); c = n; }
			return r;
		};
		const union = (a: string, b: string) => { const ra = find(a), rb = find(b); if (ra !== rb) parent.set(ra, rb); };
		const node = (k: string) => { if (!parent.has(k)) parent.set(k, k); return k; };
		const buildUF = (ps: RenderedPrim[], vs: PlannedVia[]): void => {
			parent.clear();
			for (const p of ps) {
				const e = endsOf(p);
				union(node(key(e[0].x, e[0].y, p.layer)), node(key(e[1].x, e[1].y, p.layer)));
			}
			for (const v of vs) union(node(key(v.x, v.y, 1)), node(key(v.x, v.y, 2)));
			for (const pd of pads) {
				const th = pd.layer === 12;
				const layers = th ? [1, 2] : [pd.layer];
				const body = pd.bodyRadius || pd.radius;
				const roots = layers.map(L => node(key(pd.x, pd.y, L)));
				for (let i = 1; i < roots.length; i++) union(roots[0], roots[i]);
				for (const p of ps) {
					if (!layers.includes(p.layer)) continue;
					for (const e of endsOf(p)) {
						if (Math.hypot(e.x - pd.x, e.y - pd.y) <= body) union(roots[0], node(key(e.x, e.y, p.layer)));
					}
				}
			}
		};
		const allConnected = (ps: RenderedPrim[], vs: PlannedVia[]): boolean => {
			if (pads.length <= 1) return true;
			buildUF(ps, vs);
			const roots = new Set<string>();
			for (const pd of pads) {
				const L = pd.layer === 12 ? 1 : pd.layer;
				roots.add(find(node(key(pd.x, pd.y, L))));
			}
			return roots.size <= 1;
		};

		let removed = 0;
		for (let guard = 0; guard < 4; guard++) {
			let changed = false;
			for (const v of this.plannedVias.filter(x => x.net === name)) {
				const cur = this.netRendered.get(name) ?? [];
				const incident = cur.filter(p => endsOf(p).some(e => Math.hypot(e.x - v.x, e.y - v.y) < 0.6));
				const keptVias = this.plannedVias.filter(x => x.net === name && x !== v);
				if (!incident.length) {
					// 孤儿过孔（衔接段已不存在）：纯残留，直接删
					this.plannedVias = this.plannedVias.filter(x => x !== v);
					changed = true; removed++;
					continue;
				}
				const trial = cur.filter(p => !incident.includes(p));
				if (!allConnected(trial, keptVias)) continue;
				this.netRendered.set(name, trial);
				this.plannedVias = this.plannedVias.filter(x => x !== v);
				changed = true; removed++;
			}
			if (!changed) break;
		}
		if (!removed) return;

		// 重建该网的动态障碍（走线 + 过孔），保持后续求解所见格局一致
		this.dynamicObstacles = this.dynamicObstacles.filter(o =>
			!(o.net === name && (o.group?.startsWith('trk:') || o.group?.startsWith('via:'))));
		const kept = this.netRendered.get(name) ?? [];
		if (kept.length) this.addRenderedToObstacles(mergePrims(kept));
		for (const v of this.plannedVias.filter(x => x.net === name)) {
			this.dynamicObstacles.push({
				id: `via${this.obstacleIdCounter++}`, kind: 'circle', x: v.x, y: v.y,
				r: v.dia / 2 + Math.max(this.drc.trackToTrackMil, this.drc.trackToPadMil) + this.drc.traceWidthMil / 2 + VIA_WALL_EXTRA,
				net: name, layer: undefined, group: `via:${name}:${v.x},${v.y}`, noPrune: true,
			});
		}
		this.log(`净化 ${name}: 移除 ${removed} 个冗余过孔及其衔接段`, 'info');
	}

	/**
	 * 快扫专用迷宫求解（B 实验）：细格（默认 4mil）+ 出线候选点起步 +
	 * 更高单边预算。与 gridFallbackEdge 的差异只在求解参数——障碍集、
	 * 净距语义完全一致。__RB_MAZE_CELL__ 可调格距。
	 */
	private async mazeSweepEdge(
		src: EnginePad, dst: EnginePad, netName: string,
	): Promise<{ prims: RenderedPrim[]; vias: PlannedVia[] } | null> {
		if (this.checkCancelled()) return null;
		const layers = this.hasBottomLayer ? [1, 2] : [1];
		const obstacles = [
			...this.staticObstacles,
			...this.padObstacles(this.allPads, netName),
			...this.dynamicObstacles,
		];
		const viaDia = Math.max(24, this.drc.traceWidthMil * 2);
		const viaHole = Math.max(12, this.drc.traceWidthMil);
		const srcTerm = this.padTerminal(src);
		const dstTerm = this.padTerminal(dst);
		const srcStubs = buildExitCandidates(srcTerm, { x: dst.x, y: dst.y })
			.filter(c => stubFeasible(srcTerm, c.x, c.y, obstacles, netName))
			.slice(0, 8)
			.map(c => ({ x: c.x, y: c.y }));
		const dstStubs = buildExitCandidates(dstTerm, { x: src.x, y: src.y })
			.filter(c => stubFeasible(dstTerm, c.x, c.y, obstacles, netName))
			.slice(0, 8)
			.map(c => ({ x: c.x, y: c.y }));
		const cell = Number((globalThis as any).__RB_MAZE_CELL__) || 4;
		const t0 = Date.now();
		const res = await gridRoute({
			src: { x: src.x, y: src.y },
			dst: { x: dst.x, y: dst.y },
			srcPad: { x: src.x, y: src.y, r: src.bodyRadius || src.radius, viaKeepoutR: this.padViaKeepout(src) },
			dstPad: { x: dst.x, y: dst.y, r: dst.bodyRadius || dst.radius, viaKeepoutR: this.padViaKeepout(dst) },
			srcPadLayers: this.padLayers(src),
			dstPadLayers: this.padLayers(dst),
			srcStubs,
			dstStubs,
			obstacles,
			net: netName,
			layers,
			cell,
			maxCells: 1_200_000,
			viaCostMil: Math.max(150, this.drc.traceWidthMil * 22),
			...this.gridViaExtras(),
			viaInPad: this.allowViaInPad,
			budgetMs: 1500,
			isCancelled: () => this.checkCancelled(),
		});
		this.gridBudgetMs -= Date.now() - t0;
		if (!res.ok || !res.prims.length) return null;
		const vias: PlannedVia[] = [];
		for (const v of res.vias) {
			if (!this.viaPadClear(v.x, v.y, viaDia / 2, netName)) return null;
			if (!this.viaObstacleClear(v.x, v.y, obstacles, viaDia / 2, netName)) return null;
			vias.push({ net: netName, x: v.x, y: v.y, dia: viaDia, hole: viaHole });
		}
		const prims: RenderedPrim[] = res.prims.map(p => p.kind === 'line'
			? { kind: 'line' as const, net: netName, layer: p.layer, x1: p.x1, y1: p.y1, x2: p.x2, y2: p.y2 }
			: {
				kind: 'arc' as const, net: netName, layer: p.layer,
				cx: p.cx, cy: p.cy, r: p.r, startAngle: p.startAngle, sweep: p.sweep,
			});
		return { prims, vias };
	}

	/**
	 * 提交前互检：新图元逐段对动态障碍（其它网的已布铜皮）做胶囊间距检查。
	 * 不变量 = 「已提交铜皮永不互相违规」——无论哪个求解器产出（可视图/
	 * 迷宫/兜底链），只要记账链路有任何时序漏洞，这里当场拦下。
	 * 违规时返回 false（调用方丢弃该边）。
	 */
	private primsClearOfDynamic(prims: RenderedPrim[], netName: string, obsOverride?: RBObstacle[]): boolean {
		const obs = obsOverride ?? this.dynamicObstacles;
		// 把图元统一拆成采样段（直线原样；圆弧 ≤15°/步采样），逐段检查
		const segs: Array<[number, number, number, number, number]> = [];   // [ax,ay,bx,by,layer]
		for (const p of prims) {
			if (p.kind === 'line') {
				segs.push([p.x1!, p.y1!, p.x2!, p.y2!, p.layer]);
				continue;
			}
			const cx = p.cx ?? 0, cy = p.cy ?? 0, r = p.r ?? 0;
			const a0 = p.startAngle ?? 0, sw = p.sweep ?? 0;
			const steps = Math.max(2, Math.ceil(Math.abs(sw) / (Math.PI / 12)));
			let px = cx + r * Math.cos(a0), py = cy + r * Math.sin(a0);
			for (let k = 1; k <= steps; k++) {
				const ang = a0 + (sw * k) / steps;
				const nx = cx + r * Math.cos(ang), ny = cy + r * Math.sin(ang);
				segs.push([px, py, nx, ny, p.layer]);
				px = nx; py = ny;
			}
		}
		for (const [ax, ay, bx, by, layer] of segs) {
			const segLen = Math.hypot(bx - ax, by - ay);
			const midX = (ax + bx) / 2, midY = (ay + by) / 2;
			const reach = segLen / 2 + 1;
			for (const o of obs) {
				if (o.soft) continue;
				if (o.net === netName) continue;
				if (o.layer !== undefined && o.layer !== layer) continue;
				const rr = o.r + 0.4;
				// 包围盒快拒：障碍锚点离线段中点超过「半长 + 半径」必不相交
				if (Math.abs(o.x - midX) > reach + rr + 2 || Math.abs(o.y - midY) > reach + rr + 2) continue;
				let hit = false;
				if (o.kind === 'circle') {
					hit = pointSegDist(o.x, o.y, ax, ay, bx, by) < rr;
				} else {
					const ox1 = o.x1 ?? o.x, oy1 = o.y1 ?? o.y;
					const ox2 = o.x2 ?? ox1, oy2 = o.y2 ?? oy1;
					const oLen = Math.hypot(ox2 - ox1, oy2 - oy1);
					const oSteps = Math.min(64, Math.max(1, Math.ceil(oLen / Math.max(0.9 * o.r, 0.5))));
					for (let s = 0; s <= oSteps && !hit; s++) {
						const u = s / oSteps;
						hit = pointSegDist(ox1 + (ox2 - ox1) * u, oy1 + (oy2 - oy1) * u, ax, ay, bx, by) < rr;
					}
				}
				if (hit) {
					this.log(`互检违规: ${netName} L${layer}(${ax.toFixed(0)},${ay.toFixed(0)})→(${bx.toFixed(0)},${by.toFixed(0)}) 撞 ${o.group ?? o.kind}@(${o.x.toFixed(0)},${o.y.toFixed(0)}) r=${o.r.toFixed(1)} net=${o.net ?? '-'} L${o.layer ?? '*'}`, 'warn');
					return false;
				}
			}
			// 焊盘/钻孔净距（精确形状）：dynamicObstacles 只含画布铜皮与板框，
			// padObstacles 按网惰性构建、不在其中 —— 不补这道，任何障碍口径
			// 有缺口的路径都能把走线压到焊盘上（case12 实测 D- 底层线压
			// GND 通孔焊盘 USB2_14，minDistance=0）。
			const nPts = Math.max(1, Math.ceil(segLen / 4));
			for (let s = 0; s <= nPts; s++) {
				const t = s / nPts;
				const px = ax + (bx - ax) * t, py = ay + (by - ay) * t;
				for (const pd of this.nearPads(px, py)) {
					if (pd.net === netName) continue;
					// 层语义：SMD 只挡本层，TH/多层（12）挡两层
					if (pd.layer !== 12 && pd.layer !== layer) continue;
					const need = this.drc.trackToPadMil + this.drc.traceWidthMil / 2 + 0.2;
					if (this.padCopperDistance(pd, px, py) < need) {
						this.log(`互检违规: ${netName} L${layer}(${ax.toFixed(0)},${ay.toFixed(0)})→(${bx.toFixed(0)},${by.toFixed(0)}) 撞焊盘 ${pd.net}@(${pd.x.toFixed(0)},${pd.y.toFixed(0)})`, 'warn');
						return false;
					}
					const cachedHole = Math.max(
				Number(this.padShapes[`${pd.x.toFixed(2)},${pd.y.toFixed(2)}`]?.hole?.[1]) || 0,
				Number(this.padShapes[`${pd.x.toFixed(2)},${pd.y.toFixed(2)}`]?.hole?.[2]) || 0);
					const holeD = (pd.holeDiameter ?? 0) || cachedHole;
					if (holeD > 0) {
						const needH = holeD / 2 + Math.max(this.drc.trackToPadMil, 6) + this.drc.traceWidthMil / 2 + 0.2;
						if (Math.hypot(pd.x - px, pd.y - py) < needH) {
							this.log(`互检违规: ${netName} L${layer} 撞钻孔 ${pd.net}@(${pd.x.toFixed(0)},${pd.y.toFixed(0)})`, 'warn');
							return false;
						}
					}
				}
			}
		}
		return true;
	}

	/**
	 * 拐角圆角化（提交前）：相邻直线折角 > 20° 时插入切线圆弧。
	 * 只做「安全」圆角——圆弧逐点过障碍检查（障碍半径已含线宽+间距），
	 * 任何一点越界就保留原折角。R 受两段长度限制（≤ 40% 段长）。
	 *
	 * 为什么在提交前做：可视图路径除绕障圆弧外天然平滑，锐角主要来自
	 * 迷宫兜底/逃逸段/撕毁重布的拼接；这里统一收敛，E2E 与离线同源。
	 */
	private smoothCorners(prims: RenderedPrim[], netName: string): RenderedPrim[] {
		if (prims.length < 2) return prims;
		const drc = this.drc;
		const rMax = Math.min(30, Math.max(20, drc.traceWidthMil * 3));
		const obsCache = new Map<number, RBObstacle[]>();
		const obsFor = (layer: number): RBObstacle[] => {
			let o = obsCache.get(layer);
			if (!o) {
				// 焊盘从圆近似里剔除，改用精确形状距离（arcPadClear）：
				// 圆比实际矩形角外扩约 6mil，会误杀 DRC 干净的圆角弧。
				o = this.layerObstacles(netName, layer, true).filter(ob => !(ob.group?.startsWith('pad:')));
				obsCache.set(layer, o);
			}
			return o;
		};
		// 焊盘精确净距：按真实形状（矩形/椭圆/旋转）算弧到铜皮的有符号距离，
		// 不用外包圆（圆在矩形角部外扩，误杀）。needCu = 间距 + 半线宽。
		const arcPadClear = (cx: number, cy: number, r: number, a0: number, sw: number, layer: number): boolean => {
			const needCu = this.drc.trackToPadMil + this.drc.traceWidthMil / 2;
			const near = this.allPads.filter(pd =>
				pd.net !== netName &&
				(pd.layer === 12 || pd.padType === 'th' || pd.layer === layer) &&
				Math.hypot(pd.x - cx, pd.y - cy) < r + 150);
			if (!near.length) return true;
			const steps = Math.max(3, Math.ceil(Math.abs(sw) / (Math.PI / 12)));
			for (let k = 0; k <= steps; k++) {
				const a = a0 + (sw * k) / steps;
				const px = cx + r * Math.cos(a), py = cy + r * Math.sin(a);
				for (const pd of near) {
					if (this.padCopperDistance(pd, px, py) < needCu) return false;
					if (pd.layer === 12 || pd.padType === 'th') {
						const holeD = Math.max(pd.holeDiameter, 0);
						if (holeD > 0 && Math.hypot(pd.x - px, pd.y - py) < holeD / 2 + needCu) return false;
					}
				}
			}
			return true;
		};
		// 圆弧净距检查（只查圆角邻域内的障碍，包围盒快拒）
		const arcClear = (cx: number, cy: number, r: number, a0: number, sw: number, layer: number): boolean => {
			const obs = obsFor(layer);
			const steps = Math.max(3, Math.ceil(Math.abs(sw) / (Math.PI / 12)));
			for (let k = 0; k <= steps; k++) {
				const a = a0 + (sw * k) / steps;
				const px = cx + r * Math.cos(a), py = cy + r * Math.sin(a);
				for (const o of obs) {
					if (o.soft) continue;
					if (o.net && o.net === netName) continue;
					if (o.layer !== undefined && o.layer !== layer) continue;
					const rr = o.r;
					if (Math.abs(o.x - px) > rr + 1 || Math.abs(o.y - py) > rr + 1) continue;
					if (Math.hypot(o.x - px, o.y - py) < rr) return false;
				}
			}
			return true;
		};
		const out: RenderedPrim[] = [];
		const seq = [...prims];   // 圆角会改写下一段头，先复制避免动到调用方数组
		// 受保护转角：焊盘/过孔中心不圆角化。fanoutFillet 不知道焊盘存在，
		// 圆角弧会把 pad 所在的角点从铜皮上切掉（弧与腿的切点间不再覆盖
		// pad 中心）—— pad 悬空的又一来源（P02 的 B 焊盘嫌疑路径）。
		const prot = new Set<string>();
		const kOf = (x: number, y: number) => `${Math.round(x * 2)}|${Math.round(y * 2)}`;
		for (const v of this.plannedVias) if (v.net === netName) prot.add(kOf(v.x, v.y));
		// 焊盘不进 prot：大焊盘中心有被铜皮盖住的隐藏硬角（实测剩 7 个 59~81mil
		// 焊盘中心）。改为圆角时按「切点仍在焊盘铜皮内」判定——小半径时切点落在
		// 焊盘上，圆过焊盘不断连，还能消掉隐藏硬角。
		const padAt = new Map<string, { radius: number; bodyRadius: number }>();
		for (const pd of this.allPads) if (pd.net === netName) padAt.set(kOf(pd.x, pd.y), { radius: pd.radius ?? 0, bodyRadius: pd.bodyRadius ?? 0 });
		for (let i = 0; i < seq.length; i++) {
			const cur = seq[i];
			const nxt = seq[i + 1];
			if (!nxt || cur.kind !== 'line' || nxt.kind !== 'line' || cur.layer !== nxt.layer) {
				out.push(cur);
				continue;
			}
			// 端点相接（同一折角）才处理
			if (Math.hypot((cur.x2 ?? 0) - (nxt.x1 ?? 0), (cur.y2 ?? 0) - (nxt.y1 ?? 0)) > 0.6) {
				out.push(cur);
				continue;
			}
			if (prot.has(kOf(cur.x2!, cur.y2!))) { out.push(cur); continue; }
			// 圆角半径从 rMax 递减重试：贴焊盘/邻线时用更小的弧（凸出量按 R 线性
			// 减小），不挪线也能塞进去——消除最后 0.36~0.4mil 差值的办法。
			let picked: ReturnType<typeof fanoutFillet> = null;
			const padHere = padAt.get(kOf(cur.x2!, cur.y2!));
			for (const rr of [rMax, rMax * 0.7, rMax * 0.5, rMax * 0.35, rMax * 0.25, 8, 5, 3]) {
				const ff = fanoutFillet(
					{ x1: cur.x1!, y1: cur.y1!, x2: cur.x2!, y2: cur.y2! },
					{ x1: nxt.x1!, y1: nxt.y1!, x2: nxt.x2!, y2: nxt.y2! },
					rr,
				);
				if (!ff || ff.prims.length < 3) continue;
				const aa = ff.prims[1];
				if (aa.kind !== 'arc') continue;
				const sw = Math.abs((aa.sweep ?? 0) * 180) / Math.PI;
				// 小折角（<12°）不值得 / 大角度折返（>120°）和小半径（<3mil）不做
				if (sw < 12 || sw > 120 || (aa.r ?? 0) < 3) continue;
				// 焊盘中心角：切点(距角 t=R·tan(半角))必须仍落在焊盘铜皮内，否则断连
				if (padHere) {
					const t = (aa.r ?? 0) * Math.tan(Math.abs(aa.sweep ?? 0) / 2);
					if (t > Math.min(padHere.radius, padHere.bodyRadius) - 1) continue;
				}
				if (!arcClear(aa.cx!, aa.cy!, aa.r!, aa.startAngle!, aa.sweep!, cur.layer)) continue;
				if (!arcPadClear(aa.cx!, aa.cy!, aa.r!, aa.startAngle!, aa.sweep!, cur.layer)) continue;
				picked = ff;
				break;
			}
			if (!picked) {
				out.push(cur);
				continue;
			}
			const f = picked;
			const arc = f.prims[1];
			// 替换本段尾 + 下一段头：本段尾截断 + 弧入列，下一段头换成截断后的
			// 剩余段，继续与再下一段圆角（链式）——否则长阶梯只圆角隔一个拐角，
			// 实测 45° 楼梯会剩一半硬折角。
			out.push(
				{ ...f.prims[0], net: netName, layer: cur.layer } as RenderedPrim,
				{ ...arc, net: netName, layer: cur.layer } as RenderedPrim,
			);
			seq[i + 1] = { ...f.prims[2], net: netName, layer: cur.layer } as RenderedPrim;
			// 下一轮 cur = 截断后的下一段（其尾仍是原接点），nxt = 再下一段
		}
		return out;
	}

		/**
		 * 收尾形态整备：对每个网络按**最终铜皮**再做一次圆角化。
		 * 提交期的 smoothCorners 每次只看到本次新增的 attemptRendered，
		 * 多次尝试/撕毁重布拼出来的网内接缝（旧链尾 → 新链头）从未被圆角化；
		 * 实测 240s 基准仍剩 46 处 >30° 硬折角，多数在这类接缝上。
		 * 净距逐点复核（越界保留原角），不会引入 DRC 违规。
		 */
		/**
		 * 橡皮筋拉紧（几何版 string-pulling）：同层连续直线链上，若隔点直连
		 * 不侵入任何障碍，就删掉中间顶点。网格兜底的 45° 楼梯、兜底链拼接的
		 * 小折弯都会被拉成最紧的折线 —— 这是「橡皮筋」形态的最后一道工序。
		 * 只删顶点、不动端点，连通性不变；逐段净距复核，不产生 DRC 违规。
		 */
		private tautenNet(name: string): boolean {
			const prims = this.netRendered.get(name);
			if (!prims || prims.length < 3) return false;
			const obsCache = new Map<number, RBObstacle[]>();
			const obsFor = (layer: number): RBObstacle[] => {
				let o = obsCache.get(layer);
				if (!o) {
					o = this.layerObstacles(name, layer, true);
					obsCache.set(layer, o);
				}
				return o;
			};
			const segClear = (x1: number, y1: number, x2: number, y2: number, layer: number): boolean => {
				let minx = Math.min(x1, x2), maxx = Math.max(x1, x2);
				let miny = Math.min(y1, y2), maxy = Math.max(y1, y2);
				for (const o of obsFor(layer)) {
					if (o.soft) continue;
					if (o.net && o.net === name) continue;
					if (o.layer !== undefined && o.layer !== layer) continue;
					const rr = o.r + 0.2;
					if (o.x + rr < minx || o.x - rr > maxx || o.y + rr < miny || o.y - rr > maxy) continue;
					if (o.kind === 'circle') {
						if (pointSegDist(o.x, o.y, x1, y1, x2, y2) < rr) return false;
					} else {
						// 胶囊：沿轴采样（障碍圆为主，胶囊罕见）
						const ox1 = o.x1 ?? o.x, oy1 = o.y1 ?? o.y;
						const ox2 = o.x2 ?? ox1, oy2 = o.y2 ?? oy1;
						const steps = Math.min(64, Math.max(1, Math.ceil(Math.hypot(ox2 - ox1, oy2 - oy1) / Math.max(0.9 * o.r, 0.5))));
						for (let s = 0; s <= steps; s++) {
							const t = s / steps;
							if (pointSegDist(ox1 + (ox2 - ox1) * t, oy1 + (oy2 - oy1) * t, x1, y1, x2, y2) < rr) return false;
						}
					}
				}
				return true;
			};
			const merged = mergePrims(prims);
			// 受保护顶点：焊盘中心 / 过孔中心 / 分支点（≥3 段共享端点）。
			// 隔点拉直绝不能跳过它们 —— 实测 P02 的 pad1→pad2→via 两段被拉直成
			// pad1→via 直接连线，pad2 的连接被拉没（画布上 pad 悬空，EDA 判
			// Connection Error；P00 同形态）。这是「布通网却报 Via 断连」的真凶。
			const prot = new Set<string>();
			const kOf = (x: number, y: number) => `${Math.round(x * 2)}|${Math.round(y * 2)}`;   // 0.5mil 量化
			for (const v of this.plannedVias) if (v.net === name) prot.add(kOf(v.x, v.y));
			for (const pd of this.allPads) if (pd.net === name) prot.add(kOf(pd.x, pd.y));
			const endCount = new Map<string, number>();
			for (const p of merged) {
				if (p.kind !== 'line') continue;
				const a = kOf(p.x1!, p.y1!), b = kOf(p.x2!, p.y2!);
				endCount.set(a, (endCount.get(a) ?? 0) + 1);
				endCount.set(b, (endCount.get(b) ?? 0) + 1);
			}
			for (const [k, c] of endCount) if (c >= 3) prot.add(k);
			// 逐条同层直线链做隔点拉直（链被弧/层切换/断点截断）
			const out: RenderedPrim[] = [];
			let i = 0;
			let changed = false;
			while (i < merged.length) {
				const cur = merged[i];
				if (cur.kind !== 'line') { out.push(cur); i++; continue; }
				// 收集链：同层、相邻 line、端点相接
				const chain: RenderedPrim[] = [cur];
				let j = i + 1;
				while (j < merged.length) {
					const nxt = merged[j];
					if (nxt.kind !== 'line' || nxt.layer !== cur.layer) break;
					const tail = chain[chain.length - 1];
					if (Math.hypot((tail.x2 ?? 0) - (nxt.x1 ?? 0), (tail.y2 ?? 0) - (nxt.y1 ?? 0)) > 0.6) break;
					chain.push(nxt);
					j++;
				}
				if (chain.length >= 2) {
					// 顶点序列（首尾保留）
					const pts: Array<{ x: number; y: number }> = [{ x: chain[0].x1!, y: chain[0].y1! }];
					for (const c of chain) pts.push({ x: c.x2!, y: c.y2! });
					// 隔点拉直：多轮直到稳定（轮数封顶防抖动）；受保护顶点不删
					for (let pass = 0; pass < 4; pass++) {
						let removed = false;
						for (let k = 1; k < pts.length - 1; k++) {
							if (prot.has(kOf(pts[k].x, pts[k].y))) continue;
							if (segClear(pts[k - 1].x, pts[k - 1].y, pts[k + 1].x, pts[k + 1].y, cur.layer)) {
								pts.splice(k, 1);
								removed = true;
								changed = true;
								k--;   // 继续尝试删掉前一个
							}
						}
						if (!removed) break;
					}
					const dedup: Array<{ x: number; y: number }> = [pts[0]];
					for (const q of pts) {
						const l = dedup[dedup.length - 1];
						if (Math.hypot(q.x - l.x, q.y - l.y) > 0.05) dedup.push(q);
					}
					for (let k = 1; k < dedup.length; k++) {
						out.push({ kind: 'line', net: name, layer: cur.layer, x1: dedup[k - 1].x, y1: dedup[k - 1].y, x2: dedup[k].x, y2: dedup[k].y });
					}
				} else {
					out.push(cur);
				}
				i = j;
			}
			if (!changed) return false;
			// 长度没变短（罕见）就不采纳，避免无谓翻转
			if (renderedLength(out) >= renderedLength(merged) - 0.5) return false;
			this.netRendered.set(name, out);
			// 重建该网动态障碍（只动走线，过孔障碍保留）
			this.dynamicObstacles = this.dynamicObstacles.filter(o => !(o.net === name && o.group?.startsWith('trk:')));
			this.addRenderedToObstacles(mergePrims(out));
			return true;
		}

		private cosmeticPass(): void {
			const touchedNames: string[] = [];
			for (const [name, prims] of [...this.netRendered]) {
				if (prims.length < 2) continue;
				const merged = mergePrims(prims);
				const smoothed = this.smoothCorners(merged, name);
				if (primsSeqEqual(merged, smoothed)) continue;
				this.netRendered.set(name, smoothed);
				touchedNames.push(name);
			}
			if (!touchedNames.length) return;
			this.log(`收尾形态整备：${touchedNames.length} 个网络重新圆角化`, 'info');
			for (const name of touchedNames) {
				const prims = this.netRendered.get(name) ?? [];
				this.callbacks.onNetUpdate?.(
					name, mergePrims(prims), this.plannedVias.filter(v => v.net === name),
					prims.length > 0, undefined,
				);
			}
		}

		/**
		 * 剪掉「不接任何焊盘」的悬空碎片。
		 *
		 * 部分布通/兜底链会留下「一段底层线 + 两个过孔 + 两截引出段」的
		 * 孤岛：几何上彼此相连，但不通到本网任何焊盘（EDA 报这些过孔/
		 * 线段未连接），画布上是悬空的短管——用户反馈的「异常打孔绕线」。
		 * 只保留至少含一个焊盘的连通分量；连通性不变，视觉/DRC 噪声大减。
		 */
		private pruneDanglingFragments(): void {
			const touched: string[] = [];
			for (const [name, prims] of [...this.netRendered]) {
				if (!prims.length) continue;
				const pads = this.allPads.filter(p => p.net === name);
				if (!pads.length) continue;
				const vias = this.plannedVias.filter(v => v.net === name);
				const n = prims.length, nv = vias.length;
				const parent = new Int32Array(n + nv);
				for (let i = 0; i < parent.length; i++) parent[i] = i;
				const find = (x: number): number => {
					let r = x;
					while (parent[r] !== r) r = parent[r];
					let c = x;
					while (parent[c] !== c) { const p = parent[c]; parent[c] = r; c = p; }
					return r;
				};
				const uni = (a: number, b: number): void => { const ra = find(a), rb = find(b); if (ra !== rb) parent[ra] = rb; };
				const ends = (p: RenderedPrim): Array<{ x: number; y: number }> => p.kind === 'line'
					? [{ x: p.x1!, y: p.y1! }, { x: p.x2!, y: p.y2! }]
					: [{ x: (p.cx ?? 0) + (p.r ?? 0) * Math.cos(p.startAngle ?? 0), y: (p.cy ?? 0) + (p.r ?? 0) * Math.sin(p.startAngle ?? 0) },
					   { x: (p.cx ?? 0) + (p.r ?? 0) * Math.cos((p.startAngle ?? 0) + (p.sweep ?? 0)), y: (p.cy ?? 0) + (p.r ?? 0) * Math.sin((p.startAngle ?? 0) + (p.sweep ?? 0)) }];
				// 图元↔图元 / 图元↔过孔：同层端点相接
				for (let i = 0; i < n; i++) {
					for (const e of ends(prims[i])) {
						for (let j = i + 1; j < n; j++) {
							if (prims[i].layer !== prims[j].layer) continue;
							if (ends(prims[j]).some(q => Math.hypot(q.x - e.x, q.y - e.y) < 0.8)) uni(i, j);
						}
						for (let k = 0; k < nv; k++) {
							if (Math.hypot(vias[k].x - e.x, vias[k].y - e.y) < 0.8) uni(i, n + k);
						}
					}
				}
				// 焊盘锚定：分量里任一端点落在焊盘铜皮内（层匹配）即算接上
				const anchored = new Set<number>();
				for (let i = 0; i < n; i++) {
					for (const e of ends(prims[i])) {
						for (const pd of pads) {
							if (pd.layer !== 12 && pd.layer !== prims[i].layer) continue;
							// 用外接半对角线（长盘沿长轴出线也锚得住），别用 bodyRadius
							// —— 连接器 59x39 盘沿长轴到边 29.6mil > bodyRadius 19.7，
							// 会把合法引出段误判成「不接焊盘」剪掉。
							if (Math.hypot(pd.x - e.x, pd.y - e.y) <= this.padHalfExtent(pd) + 0.5) anchored.add(find(i));
						}
					}
				}
				const keepPrim = prims.map((_, i) => anchored.has(find(i)));
				const keepVia = vias.map((_, k) => anchored.has(find(n + k)));
				if (keepPrim.every(Boolean) && keepVia.every(Boolean)) continue;
				const newPrims = prims.filter((_, i) => keepPrim[i]);
				const newVias = vias.filter((_, k) => keepVia[k]);
				const removed = (n - newPrims.length) + (nv - newVias.length);
				if (!removed) continue;
				if (newPrims.length) this.netRendered.set(name, newPrims); else this.netRendered.delete(name);
				this.plannedVias = this.plannedVias.filter(v => v.net !== name).concat(newVias);
				touched.push(name);
				this.log(`收尾剪枝 ${name}: 移除 ${removed} 个不接焊盘的悬空图元`, 'info');
			}
			for (const name of touched) {
				const ps = this.netRendered.get(name) ?? [];
				this.callbacks.onNetUpdate?.(name, mergePrims(ps), this.plannedVias.filter(v => v.net === name), ps.length > 0, undefined);
			}
		}

		/** 点到图元的最近距离（圆弧按 ≤15° 采样） */
	private primPointDist(p: RenderedPrim, x: number, y: number): number {		if (p.kind === 'line') {
			return pointSegDist(x, y, p.x1!, p.y1!, p.x2!, p.y2!);
		}
		const cx = p.cx ?? 0, cy = p.cy ?? 0, r = p.r ?? 0;
		const a0 = p.startAngle ?? 0, sw = p.sweep ?? 0;
		const steps = Math.max(2, Math.ceil(Math.abs(sw) / (Math.PI / 12)));
		let best = Infinity;
		for (let k = 0; k <= steps; k++) {
			const a = a0 + (sw * k) / steps;
			const d = Math.hypot(cx + r * Math.cos(a) - x, cy + r * Math.sin(a) - y);
			if (d < best) best = d;
		}
		return best;
	}

	/** 图元任一端点是否落在 (x,y) 附近 */
	private primEndsNear(p: RenderedPrim, x: number, y: number, tol: number): boolean {
		if (p.kind === 'line') {
			return Math.hypot(p.x1! - x, p.y1! - y) <= tol || Math.hypot(p.x2! - x, p.y2! - y) <= tol;
		}
		const cx = p.cx ?? 0, cy = p.cy ?? 0, r = p.r ?? 0;
		const a0 = p.startAngle ?? 0, sw = p.sweep ?? 0;
		const a1 = a0 + sw;
		return Math.hypot(cx + r * Math.cos(a0) - x, cy + r * Math.sin(a0) - y) <= tol
			|| Math.hypot(cx + r * Math.cos(a1) - x, cy + r * Math.sin(a1) - y) <= tol;
	}

	/**
	 * 终局过孔净距修复。
	 *
	 * 为什么需要：过孔放置时的校验（viaPadClear / viaObstacleClear）只看当时
	 * 的世界；撕毁重布的**回滚恢复**会把旧图元放回一个已经变化的世界（别的网
	 * 新增了走线/过孔），绕过校验。实测 EDA DRC 报 Track to Via / SMD Pad to
	 * Via 各若干处，间距缺口仅 0.3mil（临界）。
	 *
	 * 做法：用与放置同口径的模型复算「过孔 ↔ 异网走线/过孔」，违规的过孔连同
	 * 衔接段一起拆除；若该网因此不连通则退回部分布通（宁可少布，不出违规）。
	 */
	/**
	 * 线段平移后的复检：是否仍与所有异网非软障碍保持 o.r 清距。
	 * 与提交时的建模口径一致（障碍 r 已含对方半宽+间距+0.1 余量）；
	 * 焊盘障碍的圆近似偏保守，复检误拒只会让微移退回拆除（安全方向）。
	 */
	private lineStillClear(x1: number, y1: number, x2: number, y2: number, layer: number | undefined, netName: string): boolean {
		const minX = Math.min(x1, x2), maxX = Math.max(x1, x2);
		const minY = Math.min(y1, y2), maxY = Math.max(y1, y2);
		for (const o of this.dynamicObstacles) {
			if (o.soft || !o.net || o.net === netName) continue;
			if (o.layer !== undefined && layer !== undefined && o.layer !== layer) continue;
			if (o.x + o.r < minX || o.x - o.r > maxX || o.y + o.r < minY || o.y - o.r > maxY) continue;
			if (pointSegDist(o.x, o.y, x1, y1, x2, y2) < o.r) return false;
		}
		return true;
	}

	private repairViaClearance(final = false): void {
		if (!this.plannedVias.length) return;
		// 过孔去重（同网同坐标重复登记会让画布/统计出现幽灵过孔）
		{
			const seen = new Set<string>();
			this.plannedVias = this.plannedVias.filter(v => {
				const k = `${v.net}|${v.x.toFixed(2)}|${v.y.toFixed(2)}`;
				if (seen.has(k)) return false;
				seen.add(k);
				return true;
			});
		}
		const viaR = Math.max(24, this.drc.traceWidthMil * 2) / 2;
		const touched = new Set<string>();
		let removed = 0;
		let nudged = 0;
		// 在 (x,y) 处放这个过孔是否违规；返回违规描述（null=合法）。
		// 走线/钻孔/过孔对与提交口 viaExactBad 完全同源 —— 口径不一致会出
		// 「提交合法、终检拆除」级联（case12 实测协商收益全被拆掉）。
		// 异网焊盘保持全量精确求距：不能用 viaPadClear 的 nearPads 空间索引
		// —— 长焊盘（连接器/天线）中心离得远但边缘很近，索引会漏判
		//（实测 LED7/LCD5 焊盘 vs LED8/LCD6 过孔，缺口仅 0.3mil）。
		const badAt = (v: PlannedVia, x: number, y: number): string | null => {
			// 按该过孔的实际直径判（异类直径过孔用统一 viaR=12 会过严：
			// dia-20 过孔距焊盘 17.94mil 对 EDA 合法（10+5.98=15.98），
			// 按 12 判成违规 → 微移/拆除合法过孔，case9 单轮 84 次）
			const vR = (v.dia || 24) / 2;
			const exact = this.viaExactBad(x, y, v.net, vR, v.hole || 12, v);
			if (exact) return exact;
			for (const pd of this.allPads) {
				if (pd.net === v.net) continue;
				if (pd.layer !== 1 && pd.layer !== 2 && pd.layer !== 12 && pd.padType !== 'th') continue;
				const d = this.padCopperDistance(pd, x, y);
				if (d < vR + this.drc.trackToPadMil + 0.4) return `${pd.net} 焊盘 ${d.toFixed(2)}mil`;
			}
			return null;
		};
		for (const v of [...this.plannedVias]) {
			const bad = badAt(v, v.x, v.y);
			if (!bad) continue;
			// 先尝试微移（≤2mil）：缺口常常只有 0.04~0.5mil，过孔铜环半径
			// 12mil 足以覆盖原线段端点，挪一下既能过 DRC 又保住整网连通；
			// 直接拆孔会让 LCD0 这类只剩一条通道的网整网降级。
			let moved = false;
			for (const dist of [0.4, 0.8, 1.2, 1.8]) {
				for (let k = 0; k < 16 && !moved; k++) {
					const a = (k * Math.PI) / 8;
					const nx = v.x + Math.cos(a) * dist, ny = v.y + Math.sin(a) * dist;
					if (badAt(v, nx, ny)) continue;
					// 线段端点必须一起挪：EDA 的连通性图按「线端点 = 过孔中心」
					// 建边（不是铜皮重叠即可），只挪过孔会把整网判成断线
					//（实测 LED3/LED4 各 2 条 Track 断连）。弧端点无法平移，
					// 遇到弧就放弃微移、回到拆除逻辑。
					const prims = this.netRendered.get(v.net) ?? [];
					const oldX = v.x, oldY = v.y;
					const incident = prims.filter(p => this.primEndsNear(p, oldX, oldY, 1.0));
					if (incident.some(p => p.kind !== 'line')) continue;
					// 平移复检：端点挪动会横扫出新走廊，提交时的合法余量只有
					// 0.1mil（障碍 r 的 +0.1），1.8mil 的微移足以吃穿它变成
					// DRC 违规（case12 实测 TH Pad/Hole to Track 各 1 条）。
					// 挪完先复检，被挪线段扫进任何异网障碍就回退换方向。
					for (const p of incident) {
						if (p.kind !== 'line') continue;
						if (Math.hypot((p.x1 ?? 0) - oldX, (p.y1 ?? 0) - oldY) < 1.0) { p.x1 = nx; p.y1 = ny; }
						else if (Math.hypot((p.x2 ?? 0) - oldX, (p.y2 ?? 0) - oldY) < 1.0) { p.x2 = nx; p.y2 = ny; }
					}
					const swept = incident.filter((p): p is RenderedPrim & { kind: 'line' } => p.kind === 'line')
						.filter(p => !this.lineStillClear(p.x1 ?? 0, p.y1 ?? 0, p.x2 ?? 0, p.y2 ?? 0, p.layer, v.net));
					if (swept.length) {
						for (const p of incident) {
							if (p.kind !== 'line') continue;
							if (Math.hypot((p.x1 ?? 0) - nx, (p.y1 ?? 0) - ny) < 1.0) { p.x1 = oldX; p.y1 = oldY; }
							else if (Math.hypot((p.x2 ?? 0) - nx, (p.y2 ?? 0) - ny) < 1.0) { p.x2 = oldX; p.y2 = oldY; }
						}
						continue;
					}
					this.log(`终检: 过孔 ${v.net}(${oldX.toFixed(1)},${oldY.toFixed(1)}) 距 ${bad}，微移至 (${nx.toFixed(1)},${ny.toFixed(1)})`, 'info');
					v.x = nx; v.y = ny;
					moved = true;
					nudged++;
					touched.add(v.net);
				}
				if (moved) break;
			}
			if (moved) continue;
			this.log(`终检: 过孔 ${v.net}(${v.x.toFixed(0)},${v.y.toFixed(0)}) 距 ${bad}，拆除`, 'warn');
			const prims = this.netRendered.get(v.net) ?? [];
			const keep = prims.filter(p => !this.primEndsNear(p, v.x, v.y, 1.0));
			this.netRendered.set(v.net, keep);
			this.plannedVias = this.plannedVias.filter(x => x !== v);
			touched.add(v.net);
			removed++;
		}
		if (!removed && !nudged) return;
		for (const nm of touched) {
			const ps = this.netRendered.get(nm) ?? [];
			const vs = this.plannedVias.filter(v => v.net === nm);
			if (!this.netIsConnected(nm, ps, vs)) {
				const i = this.routedOrder.indexOf(nm);
				if (i >= 0) this.routedOrder.splice(i, 1);
				this.droppedNets.add(nm);
				if (final) {
					// 终局修复时不再清空：后面没有重布阶段了，保留剩余铜皮
					// 至少让画布可见有效线段（routedEdges 也不再复用）。
					this.routedEdges.delete(nm);
					this.log(`终检: ${nm} 拆除违规过孔后不再连通，保留部分走线`, 'warn');
					continue;
				}
				// 残缺铜皮既不通、也不再复用（routedEdges 会把它当成「已布好」
				// 跳过重试），留着还占走线空间。整网清空，交给后续兜底阶段
				// 以当前障碍格局从零重布 —— 实测 F407 只降级不清空，9 个网
				// 全部卡死在「有走线但不通」的状态里，兜底阶段一个都救不回来。
				this.netRendered.delete(nm);
				this.routedEdges.delete(nm);
				this.plannedVias = this.plannedVias.filter(v => v.net !== nm);
				this.dynamicObstacles = this.dynamicObstacles.filter(o => o.net !== nm);
				this.log(`终检: ${nm} 拆除违规过孔后不再连通，清空重布`, 'warn');
			}
		}
		// 微移/拆除都会改变画布几何：重建障碍登记 + 立即重新下发这些网络。
		// 陈旧障碍是重灾区：微移原地改过孔/线端点（各 ≤1.8mil）、拆除移除图元，
		// 但 dynamicObstacles 还停在旧位置 —— 后续提交按旧障碍做净距检查，
		// 实际能挤进新位置 3.6mil 以内，下一轮终检再把它整网拆掉
		//（case12 实测：协商阶段 73/98 → 终局 67，全丢在这个级联上）。
		for (const nm of touched) {
			const ps = this.netRendered.get(nm) ?? [];
			this.dynamicObstacles = this.dynamicObstacles.filter(
				o => !(o.net === nm && (o.group?.startsWith('trk:') || o.group?.startsWith('via:'))));
			if (ps.length) this.addRenderedToObstacles(mergePrims(ps));
			for (const v of this.plannedVias) {
				if (v.net !== nm) continue;
				this.dynamicObstacles.push({
					id: `via${this.obstacleIdCounter++}`, kind: 'circle', x: v.x, y: v.y,
					r: v.dia / 2 + Math.max(this.drc.trackToTrackMil, this.drc.trackToPadMil) + this.drc.traceWidthMil / 2 + VIA_WALL_EXTRA,
					net: nm, layer: undefined, group: `via:${nm}:${v.x},${v.y}`, noPrune: true,
				});
			}
			this.callbacks.onNetUpdate?.(
				nm, mergePrims(ps), this.plannedVias.filter(v => v.net === nm),
				ps.length > 0, undefined,
			);
		}
	}

	/**
	 * 网内连通性（并查集）：图元端点按 0.1mil 量化；过孔 L1/L2 桥接；
	 * 焊盘按铜皮半径接入（TH 焊盘双层互连）。
	 */
	private netIsConnected(name: string, ps: RenderedPrim[], vs: PlannedVia[]): boolean {
		const pads = this.allPads.filter(p => p.net === name);
		if (pads.length <= 1) return true;
		const round = (v: number) => Math.round(v * 10) / 10;
		const key = (x: number, y: number, layer: number) => `${round(x)}|${round(y)}|${layer}`;
		const parent = new Map<string, string>();
		const find = (k: string): string => {
			let r = k;
			while (parent.get(r) !== r) r = parent.get(r)!;
			let c = k;
			while (parent.get(c) !== c) { const n = parent.get(c)!; parent.set(c, r); c = n; }
			return r;
		};
		const node = (k: string) => { if (!parent.has(k)) parent.set(k, k); return k; };
		const union = (a: string, b: string) => { const ra = find(a), rb = find(b); if (ra !== rb) parent.set(ra, rb); };
		const endsOf = (p: RenderedPrim): Array<{ x: number; y: number }> => {
			if (p.kind === 'line') return [{ x: p.x1!, y: p.y1! }, { x: p.x2!, y: p.y2! }];
			const a0 = p.startAngle ?? 0, sw = p.sweep ?? 0, r = p.r ?? 0;
			return [
				{ x: (p.cx ?? 0) + r * Math.cos(a0), y: (p.cy ?? 0) + r * Math.sin(a0) },
				{ x: (p.cx ?? 0) + r * Math.cos(a0 + sw), y: (p.cy ?? 0) + r * Math.sin(a0 + sw) },
			];
		};
		for (const p of ps) {
			const e = endsOf(p);
			union(node(key(e[0].x, e[0].y, p.layer)), node(key(e[1].x, e[1].y, p.layer)));
		}
		for (const v of vs) union(node(key(v.x, v.y, 1)), node(key(v.x, v.y, 2)));
		for (const pd of pads) {
			const th = pd.layer === 12;
			const layers = th ? [1, 2] : [pd.layer];
			const body = pd.bodyRadius || pd.radius;
			const roots = layers.map(L => node(key(pd.x, pd.y, L)));
			for (let i = 1; i < roots.length; i++) union(roots[0], roots[i]);
			for (const p of ps) {
				if (!layers.includes(p.layer)) continue;
				for (const e of endsOf(p)) {
					if (Math.hypot(e.x - pd.x, e.y - pd.y) <= body) union(roots[0], node(key(e.x, e.y, p.layer)));
				}
			}
		}
		const roots = new Set<string>();
		for (const pd of pads) {
			const L = pd.layer === 12 ? 1 : pd.layer;
			roots.add(find(node(key(pd.x, pd.y, L))));
		}
		return roots.size <= 1;
	}

	/**
	 * 铜皮级连通复审（终审谓词）：并查集节点 = 走线/圆弧/过孔/焊盘，
	 * 按**铜皮实际搭接**合并 —— 走线端点搭接、走线中段跨过焊盘铜皮、
	 * 圆角接缝的 ≤2mil 端点错位（铜皮仍重叠）都算连通。与 EDA 的
	 * Connectivity 口径对齐；netIsConnected 的端点量化图在此会误杀
	 * 圆角化网络，只适合「宁可错杀」的拆除决策。
	 */
	private netCopperConnected(name: string, ps: RenderedPrim[], vs: PlannedVia[]): boolean {
		const pads = this.allPads.filter(p => p.net === name);
		if (pads.length <= 1) return true;
		const w = this.drc.traceWidthMil;
		const n = ps.length;
		const parent = new Map<number, number>();
		const find = (i: number): number => {
			let r = i;
			if (!parent.has(r)) parent.set(r, r);
			while (parent.get(r) !== r) r = parent.get(r)!;
			return r;
		};
		const node = (i: number) => { if (!parent.has(i)) parent.set(i, i); return i; };
		const union = (a: number, b: number) => { const ra = find(node(a)), rb = find(node(b)); if (ra !== rb) parent.set(ra, rb); };
		// 包围盒（含线宽的一半）用于快拒
		const box: Array<[number, number, number, number]> = ps.map(p => {
			if (p.kind === 'line') {
				return [Math.min(p.x1!, p.x2!) - w, Math.min(p.y1!, p.y2!) - w, Math.max(p.x1!, p.x2!) + w, Math.max(p.y1!, p.y2!) + w];
			}
			const r = (p.r ?? 0) + w;
			return [(p.cx ?? 0) - r, (p.cy ?? 0) - r, (p.cx ?? 0) + r, (p.cy ?? 0) + r];
		});
		// 弧采样为折线点（12°/步，足够表达铜皮搭接）
		const arcPts = (p: RenderedPrim): Array<[number, number]> => {
			const cx = p.cx ?? 0, cy = p.cy ?? 0, r = p.r ?? 0;
			const a0 = p.startAngle ?? 0, sw = p.sweep ?? 0;
			const steps = Math.max(2, Math.ceil(Math.abs(sw) / (Math.PI / 15)));
			const pts: Array<[number, number]> = [];
			for (let k = 0; k <= steps; k++) {
				const ang = a0 + (sw * k) / steps;
				pts.push([cx + r * Math.cos(ang), cy + r * Math.sin(ang)]);
			}
			return pts;
		};
		const primPts = (p: RenderedPrim): Array<[number, number]> =>
			p.kind === 'line' ? [[p.x1!, p.y1!], [(p.x1! + p.x2!) / 2, (p.y1! + p.y2!) / 2], [p.x2!, p.y2!]] : arcPts(p);
		// 图元两两铜皮搭接（端点/采样点到对方线段的距离 ≤ 线宽）
		for (let i = 0; i < n; i++) {
			for (let j = i + 1; j < n; j++) {
				const bi = box[i], bj = box[j];
				if (bi[2] < bj[0] || bj[2] < bi[0] || bi[3] < bj[1] || bj[3] < bi[1]) continue;
				if (ps[i].layer !== ps[j].layer) continue;
				const pi = ps[i], pj = ps[j];
				const ptsI = primPts(pi), ptsJ = primPts(pj);
				let touched = false;
				outer: for (const [ax, ay] of ptsI) {
					for (const [bx, by] of ptsJ) {
						if (pj.kind === 'line') {
							if (pointSegDist(ax, ay, pj.x1!, pj.y1!, pj.x2!, pj.y2!) <= w) { touched = true; break outer; }
						} else if (Math.hypot(ax - bx, ay - by) <= w) { touched = true; break outer; }
					}
				}
				if (!touched) {
					outer2: for (const [ax, ay] of ptsJ) {
						for (const [bx, by] of ptsI) {
							if (pi.kind === 'line') {
								if (pointSegDist(ax, ay, pi.x1!, pi.y1!, pi.x2!, pi.y2!) <= w) { touched = true; break outer2; }
							} else if (Math.hypot(ax - bx, ay - by) <= w) { touched = true; break outer2; }
						}
					}
				}
				if (touched) union(i, j);
			}
		}
		// 过孔：中心到图元 ≤ 铜环外半径即搭接（线端落在孔心的口径也被
		// EDA 接受，这里保持一致）；过孔-过孔同位叠放
		for (let vi = 0; vi < vs.length; vi++) {
			const v = vs[vi];
			const vR = (v.dia || 24) / 2;
			for (let i = 0; i < n; i++) {
				if (box[i][0] - vR > v.x || box[i][1] - vR > v.y || box[i][2] + vR < v.x || box[i][3] + vR < v.y) continue;
				if (ps[i].layer !== undefined && ps[i].layer !== 1 && ps[i].layer !== 2) continue;
				const pts = primPts(ps[i]);
				let ok = false;
				for (const [ax, ay] of pts) {
					if (ps[i].kind === 'line') {
						if (pointSegDist(v.x, v.y, ps[i].x1!, ps[i].y1!, ps[i].x2!, ps[i].y2!) <= vR) { ok = true; break; }
					} else if (Math.hypot(ax - v.x, ay - v.y) <= vR) { ok = true; break; }
				}
				if (ok) union(n + vi, i);
			}
			for (let vj = 0; vj < vs.length; vj++) {
				if (vj === vi) continue;
				if (Math.hypot(vs[vj].x - v.x, vs[vj].y - v.y) <= vR + (vs[vj].dia || 24) / 2) union(n + vi, n + vj);
			}
		}
		// 焊盘：铜皮与图元/过孔搭接（padCopperDistance 精确形状）。**不能只采
		// 3 个点**：长线中段跨越焊盘铜皮是电源网的主要连接形态（实测 +5V 等
		// 13 网被误杀）—— 取「焊盘中心在段上的最近投影点 + 两端点」三处，
		// 距离函数沿凸形焊盘近似单峰，投影点即最小值位置。
		const padNode = (pi: number) => n + vs.length + pi;
		for (let pi = 0; pi < pads.length; pi++) {
			const pd = pads[pi];
			const layers = pd.layer === 12 ? [1, 2] : [pd.layer];
			for (let i = 0; i < n; i++) {
				if (!layers.includes(ps[i].layer)) continue;
				const b = box[i];
				const reach = (pd.bodyRadius || pd.radius) + w;
				if (b[2] < pd.x - reach || b[0] > pd.x + reach || b[3] < pd.y - reach || b[1] > pd.y + reach) continue;
				const p = ps[i];
				const cands: Array<[number, number]> = [];
				if (p.kind === 'line') {
					const dx = p.x2! - p.x1!, dy = p.y2! - p.y1!;
					const len2 = dx * dx + dy * dy;
					const t = len2 > 0 ? Math.max(0, Math.min(1, ((pd.x - p.x1!) * dx + (pd.y - p.y1!) * dy) / len2)) : 0;
					cands.push([p.x1!, p.y1!], [p.x1! + dx * t, p.y1! + dy * t], [p.x2!, p.y2!]);
				} else {
					cands.push(...arcPts(p));
				}
				let ok = false;
				for (const [ax, ay] of cands) {
					if (this.padCopperDistance(pd, ax, ay) <= w / 2) { ok = true; break; }
				}
				if (ok) union(padNode(pi), i);
			}
			for (let vi = 0; vi < vs.length; vi++) {
				const v = vs[vi];
				if (this.padCopperDistance(pd, v.x, v.y) <= (v.dia || 24) / 2) union(padNode(pi), n + vi);
			}
		}
		// 全部焊盘同一分量？
		const root0 = find(padNode(0));
		for (let pi = 1; pi < pads.length; pi++) {
			if (find(padNode(pi)) !== root0) return false;
		}
		return true;
	}

	/**
	 * 网格 A* 兜底（单条连接）：可视图失败后接管。
	 *
	 * 网格求解把障碍光栅化成占据图，从焊盘内部多点起步，因此能处理
	 * 可视图的固有盲区：端点被邻盘净距圈覆盖、路径需要从任意点穿过。
	 * 返回的折线/过孔在生成时已按膨胀障碍避让，最后再做一次焊盘净距复核。
	 */
	/**
	 * 网格分辨率：按「走廊面积 + 焊盘密度」动态选择，让单次 A* 的格数可控。
	 *
	 * 旧实现固定 cell=4：1200mil 跨度走廊 ≈ 55 万格/层，实测单板 1321 次网格
	 * 调用累计 3.4 亿格、78s（占总时长 65%）。格数按面积反比于 cell²，所以
	 * 只要把「每格能表达的最小缝隙」交给焊盘密度决定，长走廊就能用粗格跑。
	 *
	 *  - byArea：按目标格数（默认 12 万/次）反解 cell；
	 *  - byDensity：走廊附近焊盘铜皮的最小缝隙（格距大于缝隙就表达不出通道），
	 *    夹在 [4, 12]；
	 * 最终 clamp 到 [4, 16]。真密集区（0.5mm 排）缝隙 ~4mil → 仍是 4。
	 */
	private gridCellFor(a: { x: number; y: number }, b: { x: number; y: number }): number {
		const env = Number((globalThis as any).__RB_GRID_CELL__);
		if (Number.isFinite(env) && env > 0) return env;
		const span = Math.hypot(b.x - a.x, b.y - a.y);
		const margin = span * 0.6 + 150;
		const w = Math.abs(b.x - a.x) + 2 * margin;
		const h = Math.abs(b.y - a.y) + 2 * margin;
		const target = Number((globalThis as any).__RB_GRID_TARGET_CELLS__) || 120_000;
		const byArea = Math.sqrt((w * h) / Math.max(10_000, target));
		// 走廊附近焊盘的铜皮最小缝隙（只看距直线 90mil 内的焊盘，控制 O(k²)）
		const near: EnginePad[] = [];
		for (const pd of this.allPads) {
			if (pointSegDist(pd.x, pd.y, a.x, a.y, b.x, b.y) < (pd.radius || 0) + 90) near.push(pd);
		}
		let gap = Infinity;
		const limit = Math.min(near.length, 120);
		for (let i = 0; i < limit; i++) {
			for (let j = i + 1; j < limit; j++) {
				const d = Math.hypot(near[i].x - near[j].x, near[i].y - near[j].y)
					- (near[i].radius || 0) - (near[j].radius || 0);
				if (d < gap) gap = d;
			}
		}
		const byDensity = Number.isFinite(gap) ? Math.max(4, Math.min(12, gap)) : 4;
		return Math.max(4, Math.min(16, Math.max(byArea, byDensity)));
	}

	/**
	 * 直连快判：两点之间的直线是否已经满足净距（障碍半径已含线宽/2 + 间距）。
	 * 用于跳过「网格 A* 布线 → 再优化」里最贵的一步：能直连的边占大多数，
	 * 单次网格要 10~300ms，而这里是 O(障碍数) 的纯几何判断（~0.1ms）。
	 */
	private directLineClear(a: { x: number; y: number }, b: { x: number; y: number }, layer: number, netName: string): boolean {
		const obs = this.layerObstacles(netName, layer, true);
		for (const o of obs) {
			if (o.soft) continue;
			if (o.net && o.net === netName) continue;
			if (o.layer !== undefined && o.layer !== layer) continue;
			if (o.kind === 'circle') {
				if (pointSegDist(o.x, o.y, a.x, a.y, b.x, b.y) < o.r) return false;
			} else {
				const ox1 = o.x1 ?? o.x, oy1 = o.y1 ?? o.y;
				const ox2 = o.x2 ?? ox1, oy2 = o.y2 ?? oy1;
				const steps = Math.min(48, Math.max(1, Math.ceil(Math.hypot(ox2 - ox1, oy2 - oy1) / Math.max(0.9 * o.r, 0.5))));
				for (let s = 0; s <= steps; s++) {
					const t = s / steps;
					if (pointSegDist(ox1 + (ox2 - ox1) * t, oy1 + (oy2 - oy1) * t, a.x, a.y, b.x, b.y) < o.r) return false;
				}
			}
		}
		return true;
	}

	private async gridFallbackEdge(
		src: EnginePad, dst: EnginePad, netName: string,
	): Promise<{ prims: RenderedPrim[]; vias: PlannedVia[] } | null> {
		if (this.checkCancelled()) return null;
		if (this.gridBudgetMs <= 0) return null;   // 兜底预算耗尽：不再尝试，留给主线
		const layers = this.hasBottomLayer ? [1, 2] : [1];
		const obstacles = [
			...this.staticObstacles,
			...this.padObstacles(this.allPads, netName),
			...this.dynamicObstacles,
		];
		const viaDia = Math.max(24, this.drc.traceWidthMil * 2);
		const viaHole = Math.max(12, this.drc.traceWidthMil);
		// 出线候选点起步（B 实验④）实测负收益：case9 58→53——候选点种子
		// 改变迷宫路径经济，粗路径提前占领走廊污染后续可视图精修（与实验
		// ①③同模式）。迷宫保持「焊盘中心起步」的兜底定位；stub 接口保留
		// 在 grid-router（不传即旧行为），留待真需要时再启用。
		const t0 = Date.now();
		const gridCell = this.gridCellFor({ x: src.x, y: src.y }, { x: dst.x, y: dst.y });
		const res = await gridRoute({
			src: { x: src.x, y: src.y },
			dst: { x: dst.x, y: dst.y },
			srcPad: { x: src.x, y: src.y, r: src.bodyRadius || src.radius, viaKeepoutR: this.padViaKeepout(src) },
			dstPad: { x: dst.x, y: dst.y, r: dst.bodyRadius || dst.radius, viaKeepoutR: this.padViaKeepout(dst) },
			srcReleases: this.padReleases(src, gridCell),
			dstReleases: this.padReleases(dst, gridCell),
			srcPadLayers: this.padLayers(src),
			dstPadLayers: this.padLayers(dst),
			obstacles,
			net: netName,
			layers,
			cell: gridCell,
			viaCostMil: Math.max(150, this.drc.traceWidthMil * 22),
			...this.gridViaExtras(),
			viaInPad: this.allowViaInPad,
			budgetMs: Math.min(2000, Math.max(400, this.gridBudgetMs)),
			isCancelled: () => this.checkCancelled(),
		});
		this.gridBudgetMs -= Date.now() - t0;
		if (!res.ok || !res.prims.length) {
			if ((globalThis as any).__RB_GRID_DEBUG__) {
				this.log(`[grid-debug] ${netName} ${src.id}(L${src.layer}→[${this.padLayers(src)}]) → ${dst.id}(L${dst.layer}→[${this.padLayers(dst)}]) 失败 ok=${res.ok} prims=${res.prims.length} cell=${res.cell} ${res.debug ?? ''}`, 'warn');
			}
			return null;
		}
		// 过孔落点复核（网格只保证线段避让，过孔铜环更大）。
		// 网格 viaBlocked 是按采样圆障碍（1.8r 间距）光栅化的，圆与圆之间有
		// 采样缝：网格合法的落点在精确口径（viaExactBad）下可能差 1~3mil。
		// 不整路否决 —— 先试微移落点（衔接线段端点同步平移，EDA 连通性要求
		// 线端点=过孔中心；平移后的线段逐条重过净距），移不动才放弃本路径。
		const vias: PlannedVia[] = [];
		// 本边已过孔对（同网）：EDA 的 Hole to Hole 按孔壁距判定（同网也查），
		// 中心距 ≥ 两孔半径 + hole2Hole 规则值
		const needVV = (viaHole || 12) + (this.drc.holeToHoleMil ?? 12) + 0.2;
		const viaOk = (x: number, y: number): boolean => {
			if (!this.viaPadClear(x, y, viaDia / 2, netName)) return false;
			if (!this.viaObstacleClear(x, y, obstacles, viaDia / 2, netName)) return false;
			// 本边先前落定的过孔还没进 plannedVias，viaExactBad 查不到，单独查
			for (const w of vias) if (Math.hypot(w.x - x, w.y - y) < needVV) return false;
			return true;
		};
		// 平移后线段复核：与网格终检同口径（异网障碍 r+0.4，同网跳过）
		const segClearObs = (x1: number, y1: number, x2: number, y2: number, layer: number): boolean => {
			const minx = Math.min(x1, x2), maxx = Math.max(x1, x2);
			const miny = Math.min(y1, y2), maxy = Math.max(y1, y2);
			for (const o of obstacles) {
				if (o.soft || o.net === netName) continue;
				if (o.layer !== undefined && o.layer !== layer) continue;
				const rr = o.r + 0.4;
				if (o.x + rr < minx || o.x - rr > maxx || o.y + rr < miny || o.y - rr > maxy) continue;
				if (o.kind === 'circle') {
					if (pointSegDist(o.x, o.y, x1, y1, x2, y2) < rr) return false;
				} else {
					const ox2 = o.x2 ?? o.x, oy2 = o.y2 ?? o.y;
					const steps = Math.min(64, Math.max(1, Math.ceil(Math.hypot(ox2 - o.x, oy2 - o.y) / Math.max(0.9 * o.r, 0.5))));
					for (let s = 0; s <= steps; s++) {
						const t = s / steps;
						if (pointSegDist(o.x + (ox2 - o.x) * t, o.y + (oy2 - o.y) * t, x1, y1, x2, y2) < rr) return false;
					}
				}
			}
			return true;
		};
		for (const v of res.vias) {
			let vx = v.x, vy = v.y;
			if (!viaOk(vx, vy)) {
				const endsNear = (p: GridPrim): boolean => {
					if (p.kind === 'line') {
						return Math.hypot((p.x1 ?? 0) - v.x, (p.y1 ?? 0) - v.y) < 1.0 ||
							Math.hypot((p.x2 ?? 0) - v.x, (p.y2 ?? 0) - v.y) < 1.0;
					}
					const a0 = p.startAngle ?? 0, sw = p.sweep ?? 0, r = p.r ?? 0;
					const sx = (p.cx ?? 0) + r * Math.cos(a0), sy = (p.cy ?? 0) + r * Math.sin(a0);
					const ex = (p.cx ?? 0) + r * Math.cos(a0 + sw), ey = (p.cy ?? 0) + r * Math.sin(a0 + sw);
					return Math.hypot(sx - v.x, sy - v.y) < 1.0 || Math.hypot(ex - v.x, ey - v.y) < 1.0;
				};
				const incident = res.prims.filter(endsNear);
				const inc = incident.filter((p): p is GridPrim & { kind: 'line' } => p.kind === 'line');
				// 弧端点无法平移（几何由圆心/半径/角度定义），遇到弧直接放弃本路径
				let moved = false;
				if (inc.length === incident.length) {
					for (const dist of [0.4, 0.8, 1.2, 1.8, 2.4, 3.0]) {
						for (let k = 0; k < 16 && !moved; k++) {
							const a = (k * Math.PI) / 8;
							const nx = v.x + Math.cos(a) * dist, ny = v.y + Math.sin(a) * dist;
							if (!viaOk(nx, ny)) continue;
							// 平移后的衔接线段必须仍然合法
							let ok = true;
							for (const p of inc) {
								const ax = Math.hypot((p.x1 ?? 0) - v.x, (p.y1 ?? 0) - v.y) < 1.0 ? nx : p.x1!;
								const ay = Math.hypot((p.x1 ?? 0) - v.x, (p.y1 ?? 0) - v.y) < 1.0 ? ny : p.y1!;
								const bx = Math.hypot((p.x2 ?? 0) - v.x, (p.y2 ?? 0) - v.y) < 1.0 ? nx : p.x2!;
								const by = Math.hypot((p.x2 ?? 0) - v.x, (p.y2 ?? 0) - v.y) < 1.0 ? ny : p.y2!;
								if (!segClearObs(ax, ay, bx, by, p.layer)) { ok = false; break; }
							}
							if (!ok) continue;
							for (const p of inc) {
								if (Math.hypot((p.x1 ?? 0) - v.x, (p.y1 ?? 0) - v.y) < 1.0) { p.x1 = nx; p.y1 = ny; }
								else if (Math.hypot((p.x2 ?? 0) - v.x, (p.y2 ?? 0) - v.y) < 1.0) { p.x2 = nx; p.y2 = ny; }
							}
							vx = nx; vy = ny; moved = true;
						}
						if (moved) break;
					}
				}
				if (!moved) {
					if ((globalThis as any).__RB_GRID_DEBUG__) this.log(`[grid-debug] ${netName} 网格结果过孔(${v.x.toFixed(0)},${v.y.toFixed(0)}) 精确净距不过且无法微移`, 'warn');
					return null;
				}
			}
			vias.push({ net: netName, x: vx, y: vy, dia: viaDia, hole: viaHole });
		}
		const prims: RenderedPrim[] = res.prims.map(p => p.kind === 'line'
			? { kind: 'line' as const, net: netName, layer: p.layer, x1: p.x1, y1: p.y1, x2: p.x2, y2: p.y2 }
			: {
				kind: 'arc' as const, net: netName, layer: p.layer,
				cx: p.cx, cy: p.cy, r: p.r, startAngle: p.startAngle, sweep: p.sweep,
			});
		this.log(`网络 ${netName}: ${src.id}→${dst.id} 由网格兜底布通（${prims.length} 段${vias.length ? `，${vias.length} 过孔` : ''}）`, 'info');
		return { prims, vias };
	}

	/** 多网快照（撕毁类动作回滚用）：图元、过孔、已完成边 */
	private snapshotNets(names: string[]): Map<string, { prims: RenderedPrim[]; vias: PlannedVia[]; doneEdges: Set<string> }> {
		const snap = new Map<string, { prims: RenderedPrim[]; vias: PlannedVia[]; doneEdges: Set<string> }>();
		for (const n of names) {
			snap.set(n, {
				prims: [...(this.netRendered.get(n) ?? [])],
				vias: this.plannedVias.filter(v => v.net === n),
				doneEdges: new Set(this.routedEdges.get(n) ?? []),
			});
		}
		return snap;
	}

	/** 按快照整体恢复（含过孔障碍重登与画布同步） */
	private restoreNets(
		snap: Map<string, { prims: RenderedPrim[]; vias: PlannedVia[]; doneEdges: Set<string> }>,
		savedOrder: string[],
		networks: Array<{ name: string; pads: EnginePad[] }>,
	): void {
		for (const n of snap.keys()) this.ripupNetInternal(n, true);
		for (const [n, sv] of snap) {
			if (sv.prims.length) {
				this.netRendered.set(n, [...sv.prims]);
				this.addRenderedToObstacles(mergePrims(sv.prims));
			}
			this.routedEdges.set(n, new Set(sv.doneEdges));
			this.plannedVias = this.plannedVias.filter(v => v.net !== n).concat(sv.vias);
			for (const v of sv.vias) {
				this.dynamicObstacles.push({
					id: `via${this.obstacleIdCounter++}`, kind: 'circle', x: v.x, y: v.y,
					r: v.dia / 2 + Math.max(this.drc.trackToTrackMil, this.drc.trackToPadMil) + this.drc.traceWidthMil / 2 + VIA_WALL_EXTRA,
					net: v.net, layer: undefined, group: `via:${v.net}:${v.x},${v.y}`, noPrune: true,
				});
			}
		}
		this.routedOrder = [...savedOrder];
		for (const [n, sv] of snap) {
			const nn = networks.find(x => x.name === n);
			this.callbacks.onNetUpdate?.(n, mergePrims(sv.prims), sv.vias, sv.prims.length > 0, nn ? this.padProgress(nn) : undefined);
		}
	}

	/**
	 * 拥挤协商终局（PathFinder 式）。
	 *
	 * 逐网单跑实测：case12 全部 40 个失败网、case9 全部 10 个失败网单独放在
	 * 洁净板上都能布通 —— 差的不是几何，是「谁占走廊」的联合收敛。顺序撕毁
	 * 的「绝不净损失已布网」约束让每个局部动作都滚不动全局（局部最小值）。
	 *
	 * 这里换成协商：失败网的缺失边在「异网铜皮只计罚分、不硬挡」的网格上
	 * 求最小代价路径，得到必须让路的网集合；撕掉它们 → 布本网 → 重布被撕网。
	 * 采纳条件是**整板布通数严格增加**（允许拿一个网换两个网，这是与撕毁
	 * 动作的本质区别）；否则整体回滚，并给被穿越网加历史罚分 —— 下一轮
	 * A* 会主动绕开反复冲突的网，逐轮把拥挤摊开。所有提交仍走 routeNetEdges
	 * 的标准链（含提交互检），DRC 口径不变。
	 */
	private async negotiatePhase(networks: Array<{ name: string; pads: EnginePad[] }>, deadline: number): Promise<void> {
		const hist = new Map<string, number>();
		const padCount = new Map(networks.map(n => [n.name, n.pads.length] as const));
		const maxIter = Number((globalThis as any).__RB_NEG_ITERS__) || 6;
		// 集群升级次数（阶段级，非每轮）：换位乒乓/零采纳轮触发，最多 2 次
		let clusterTried = 0;
		// 换位历史（阶段级）：换位事件的进出网计数。互锁往往是**时间性**的
		//（case9 实测：pending 网的让路集里只有已布网，冻结瞬间找不到互锁组，
		// 但 ±几轮内同一批网反复互相挤位）—— 这批「乒乓惯犯」就是集群的另一极。
		const swapHist = new Map<string, number>();
		// 阶段级失败记录：round 2+ 让新鲜网优先（round 1 的失败网重试价值低）
		const failedThisPhase = new Set<string>();
		// 协商重布依赖网格优先链：抢救阶段可能已把网格池烧光，这里保底
		this.gridBudgetMs = Math.max(this.gridBudgetMs, 60_000);
		for (let it = 0; it < maxIter; it++) {
			if (this.checkCancelled() || Date.now() > deadline) break;
			const pending = networks.filter(n => !this.routedOrder.includes(n.name));
			if (!pending.length) break;
			// 易网先走：缺失边少的排前；**本轮已失败过的网排到最后** ——
			// 76/98 实测 round 1 用尽窗口后，round 2 的 30s 被「重试已知失败」
			// 吃光，从未试的 12 个新鲜网永远轮不到。新鲜网转化率高于惯失败网。
			pending.sort((a, b) => {
				const fa = failedThisPhase.has(a.name) ? 1 : 0;
				const fb = failedThisPhase.has(b.name) ? 1 : 0;
				return fa - fb || this.netPendingEdges(a) - this.netPendingEdges(b) || (a.name < b.name ? -1 : 1);
			});
			const before = this.routedOrder.length;
			let tried = 0, accepted = 0, swaps = 0;
			// 让路集缓存（本轮内有效）：pairwise 尝试算过的直接复用，集群检测
			// 不再重复求解。任何动作被采纳 = 世界变了 → 整表作废。
			const ripCache = new Map<string, string[] | null>();
			for (const net of pending) {
				if (this.checkCancelled() || Date.now() > deadline) break;
				if (this.routedOrder.includes(net.name)) continue;
				tried++;
				const r = await this.negotiateNet(net, networks, hist, padCount, deadline, ripCache, swapHist);
				if (r) {
					accepted++;
					if (r === 2) swaps++;
					failedThisPhase.delete(net.name);
					ripCache.clear();
				} else {
					failedThisPhase.add(net.name);
				}
			}
			// 成对协商无增益 → 升级集群协商：互为让路对象的失败网
			//（A 布通挤掉 B、B 重来又挤掉 A 的死循环）整组撕掉重排。
			// 换位采纳（总数持平）是乒乓的信号 —— case9 实测单轮 4 次换位
			// 把时间烧光，集群始终排不上；出现 2 次就立刻升级，不再等零采纳轮。
			// 只在剩余时间够跑完一次集群动作时启动（成员+挡路者 × 单网上限），
			// 每阶段最多 2 次（失败回滚后历史罚分已变化，允许重试一次）。
			if (pending.length >= 2 && clusterTried < 2 && !this.checkCancelled() &&
				deadline - Date.now() > 40_000 && (globalThis as any).__RB_NEG_NO_CLUSTER__ !== 1 &&
				(!accepted || swaps >= 2)) {
				clusterTried++;
				// 成员选组：换位惯犯优先（乒乓主角），其次缺边少的 pending 网。
				// 不再要求「让路集含 pending 邻居」—— 互锁是时间性的，冻结瞬间
				// 让路集里常常只有已布网（case9 LCD/P0x 墙实测）。
				const memberPool = [...pending].sort((a, b) =>
					(swapHist.get(b.name) ?? 0) - (swapHist.get(a.name) ?? 0) ||
					this.netPendingEdges(a) - this.netPendingEdges(b) || (a.name < b.name ? -1 : 1));
				const clusterGroup = memberPool.slice(0, 4);
				// 换位历史的已布网强制入挡路者集合（时间互锁的另一极）
				const histBlockers = [...swapHist.keys()].filter(n => this.routedOrder.includes(n));
				tried++;
				if (await this.negotiateCluster(clusterGroup, networks, hist, padCount, deadline, ripCache, histBlockers)) accepted++;
			}
			this.log(`拥挤协商第 ${it + 1} 轮：尝试 ${tried}，采纳 ${accepted}，布通 ${before} → ${this.routedOrder.length}`, 'info');
			if (!accepted) break;
		}
	}

	/**
	 * 单边独走路径的让路集合：可视图在「只有静态障碍+焊盘」的世界里求本边
	 * 最短路（与 soloRoute 同口径，但只算这一条边），返回路径图元、过孔与
	 * 压到的异网名单。无独走路径返回 null。
	 * PathFinder 的意向路由复用同一世界口径（异网铜皮可压，清判再结算）。
	 */
	private async soloEdgeCrossing(src: EnginePad, dst: EnginePad, netName: string, soft: RBObstacle[]): Promise<{ prims: RenderedPrim[]; vias: PlannedVia[]; crossed: string[] } | null> {
		const layer = getRoutingLayer(src, dst);
		const attempt = async (ly: number) => {
			const obs = this.layerObstacles(netName, ly, false);
			return await this.directEdgeRoute(src, dst, netName, ly, obs) ??
				await this.candidateEdgeRoute(src, dst, netName, ly, obs);
		};
		let prims = await attempt(layer);
		if (!prims && layer === 1 && this.hasBottomLayer && bottomCapable(src) && bottomCapable(dst)) prims = await attempt(2);
		let vias: PlannedVia[] = [];
		// 单层都不通 → 过孔换层独走（case9 LCD 排：单网布通靠的正是这条链）。
		// 临时把异网走线/过孔移出动态障碍，让 viaEdgeCandidate 看到「洁净世界」，
		// 求完立即恢复（同步段内完成，不跨 await 外泄状态以外的副作用）。
		if (!prims && this.hasBottomLayer) {
			const saved = this.dynamicObstacles;
			const movable = (o: RBObstacle) => {
				const g = o.group ?? '';
				return !!o.net && o.net !== netName && (g.startsWith('trk:') || g.startsWith('via:'));
			};
			this.dynamicObstacles = saved.filter(o => !movable(o));
			const savedVias = this.plannedVias;
			this.plannedVias = savedVias.filter(v => v.net === netName);
			try {
				const vc = await this.viaEdgeCandidate(src, dst, netName);
				if (vc) {
					prims = vc.prims;
					vias = vc.vias;
					// 过孔本身也要占位：落点压到的异网铜皮同样需要让路
					for (const v of vc.vias) {
						prims = [...prims, { kind: 'line', net: netName, layer: 1, x1: v.x, y1: v.y, x2: v.x, y2: v.y } as RenderedPrim,
							{ kind: 'line', net: netName, layer: 2, x1: v.x, y1: v.y, x2: v.x, y2: v.y } as RenderedPrim];
					}
				}
			} finally {
				this.dynamicObstacles = saved;
				this.plannedVias = savedVias;
			}
		}
		if (!prims) return null;
		const out = new Set<string>();
		const probe: RenderedPrim[] = [...prims];
		for (const v of vias) {
			probe.push({ kind: 'line', net: netName, layer: 1, x1: v.x, y1: v.y, x2: v.x, y2: v.y } as RenderedPrim);
			probe.push({ kind: 'line', net: netName, layer: 2, x1: v.x, y1: v.y, x2: v.x, y2: v.y } as RenderedPrim);
		}
		for (const p of probe) {
			const pts = this.primPoints([p], 5);
			for (const o of soft) {
				if (!o.net || o.net === netName || out.has(o.net)) continue;
				if (o.layer !== undefined && o.layer !== p.layer) continue;
				for (const pt of pts) {
					if (Math.abs(o.x - pt.x) > o.r || Math.abs(o.y - pt.y) > o.r) continue;
					if (Math.hypot(o.x - pt.x, o.y - pt.y) < o.r) { out.add(o.net); break; }
				}
			}
		}
		return { prims, vias, crossed: [...out] };
	}

	/**
	 * 让路集合：缺失边在「异网铜皮只计罚分、不硬挡」的罚分网格上求
	 * 「撕谁最便宜」的路径（网格表示不了的边回退可视图独走路径），
	 * 返回需要让路的网络名（升序）。null = 某条缺失边在洁净世界里都
	 * 无路（不是拥挤问题，协商帮不上）。
	 */
	private async computeNetRipSet(
		net: { name: string; pads: EnginePad[] },
		hist: Map<string, number>,
		padCount: Map<string, number>,
		deadline: number,
	): Promise<string[] | null> {
		const __nd = !!(globalThis as any).__RB_NEG_DEBUG__;
		const edges = this.computeMST(net.pads);
		const done = this.routedEdges.get(net.name) ?? new Set<string>();
		const missing = edges.filter(([si, di]) => !done.has(`${net.pads[si].id}->${net.pads[di].id}`));
		if (!missing.length) return [];

		// 硬障碍 = 静态 + 焊盘 + 非走线类动态障碍；罚分障碍 = 异网走线/过孔
		const hard: RBObstacle[] = [...this.staticObstacles, ...this.padObstacles(this.allPads, net.name)];
		const soft: RBObstacle[] = [];
		for (const o of this.dynamicObstacles) {
			const g = o.group ?? '';
			if (o.net && o.net !== net.name && (g.startsWith('trk:') || g.startsWith('via:'))) soft.push(o);
			else hard.push(o);
		}
		const penaltyOf = (n: string): number => {
			const routed = this.routedOrder.includes(n);
			// 已布通网拆了要重布（代价随焊盘数涨）；残段网本来就是失败网，便宜
			//（h² 平方放大在 PF 里证伪：墙网走廊几何强制，梯度无效 —— 这里
			// 保持线性，回归验证过的 v0.0.125 形状）
			const base = routed ? 70 + 12 * (padCount.get(n) ?? 4) : 25;
			return base * (1 + (hist.get(n) ?? 0));
		};
		const layers = this.hasBottomLayer ? [1, 2] : [1];
		const gridCell = Number((globalThis as any).__RB_GRID_CELL__) || 4;
		const crossed = new Set<string>();
		for (const [si, di] of missing) {
			if (this.checkCancelled() || Date.now() > deadline) return null;
			const src = net.pads[si], dst = net.pads[di];
			const res = await gridRoute({
				src: { x: src.x, y: src.y },
				dst: { x: dst.x, y: dst.y },
				srcPad: { x: src.x, y: src.y, r: src.bodyRadius || src.radius, viaKeepoutR: this.padViaKeepout(src) },
				dstPad: { x: dst.x, y: dst.y, r: dst.bodyRadius || dst.radius, viaKeepoutR: this.padViaKeepout(dst) },
				srcReleases: this.padReleases(src, gridCell),
				dstReleases: this.padReleases(dst, gridCell),
				srcPadLayers: this.padLayers(src),
				dstPadLayers: this.padLayers(dst),
				obstacles: hard,
				penaltyObstacles: soft,
				penaltyOf,
				net: net.name,
				layers,
				cell: gridCell,
				viaCostMil: Math.max(150, this.drc.traceWidthMil * 22),
				...this.gridViaExtras(),
				viaInPad: this.allowViaInPad,
				budgetMs: 1500,
				isCancelled: () => this.checkCancelled(),
			});
			if (!res.ok) {
				// 网格表示不了的边（0.5mm 连接器排的出线、长焊盘端点）：洁净板
				// 逐网扫描实测这类边只有可视图链能解。改用「可视图独走路径」
				//（只避静态障碍+焊盘）求让路集合：路径压到的异网走线/过孔即需撕
				const solo = await this.soloEdgeCrossing(src, dst, net.name, soft);
				if (!solo) {
					if (__nd) this.log(`[neg] ${net.name} ${src.id}→${dst.id} 网格与独走都无路（${res.debug ?? ''}）`, 'info');
					return null;
				}
				for (const c of solo.crossed) crossed.add(c);
				continue;
			}
			for (const c of res.crossed ?? []) crossed.add(c);
		}
		return [...crossed].filter(n => n !== net.name).sort();
	}

	/**
	 * 单网协商动作。返回 0=拒绝（回滚），1=严格增益采纳，2=换位采纳
	 *（布通数持平、目标进一网掉一网 —— 乒乓死锁的信号，调用方据此升级
	 * 集群协商）。
	 */
	private async negotiateNet(
		net: { name: string; pads: EnginePad[] },
		networks: Array<{ name: string; pads: EnginePad[] }>,
		hist: Map<string, number>,
		padCount: Map<string, number>,
		deadline: number,
		ripCache?: Map<string, string[] | null>,
		swapHist?: Map<string, number>,
	): Promise<0 | 1 | 2> {
		const __nd = !!(globalThis as any).__RB_NEG_DEBUG__;
		// 让路集缓存（本轮内有效，采纳即作废由调用方清表）
		let rip0 = ripCache?.get(net.name);
		if (rip0 === undefined) {
			rip0 = await this.computeNetRipSet(net, hist, padCount, deadline);
			ripCache?.set(net.name, rip0);
		}
		if (rip0 === null) return 0;
		const rip = rip0;
		const maxRip = Number((globalThis as any).__RB_NEG_MAXRIP__) || 10;
		if (rip.length > maxRip) {
			if (__nd) this.log(`[neg] ${net.name} 需让路 ${rip.length} 网（>${maxRip}），跳过`, 'info');
			for (const c of rip) hist.set(c, (hist.get(c) ?? 0) + 0.5);
			return 0;
		}

		const savedOrder = [...this.routedOrder];
		const snap = this.snapshotNets([net.name, ...rip]);
		const before = this.routedOrder.length;
		for (const c of rip) this.ripupNet(c);
		// 单次协商限时：case12 实测一次动作吃 23s，整段协商只够试 1 个网。
		// 本网 + 每个被撕网各自封顶（__RB_NEG_NET_MS__，缺省 5s），超时即按
		// 「无净增益」回滚 —— 便宜地多试几个网，比在一个网上死磕划算。
		const perNet = Number((globalThis as any).__RB_NEG_NET_MS__) || 5000;
		const capAt = (): number => Math.min(deadline, Date.now() + perNet);
		await this.routeNetEdges(net, 'full', capAt());
		// 被撕网：先布焊盘少的（好安置），两焊盘网失败再试换层兜底
		const ripNets = rip.map(c => networks.find(x => x.name === c)).filter((x): x is { name: string; pads: EnginePad[] } => !!x)
			.sort((a, b) => a.pads.length - b.pads.length);
		for (const rn of ripNets) {
			if (this.checkCancelled() || Date.now() > deadline) break;
			const ok = await this.routeNetEdges(rn, 'full', capAt());
			if (!ok && rn.pads.length === 2 && Date.now() < deadline) await this.tryViaFallback(rn);
		}
		// 换位接受：总数持平但目标网布通（恰有一个被撕网掉出）也采纳 ——
		// PathFinder 式换位迭代。掉出的网带历史罚分，并**趁热立即重布**
		//（它的旧走廊刚被腾出来，此刻成功率最高；等下一轮世界又变了）。
		const targetGained = this.routedOrder.includes(net.name);
		const lost = savedOrder.filter(n => !this.routedOrder.includes(n));
		let after = this.routedOrder.length;
		if (after > before || (targetGained && after === before && lost.length === 1)) {
			if (after === before) {
				hist.set(lost[0], (hist.get(lost[0]) ?? 0) + 0.5);
				// 换位历史：进出双方都是时间互锁的参与者（集群升级的选组依据）
				if (swapHist) {
					swapHist.set(lost[0], (swapHist.get(lost[0]) ?? 0) + 1);
					swapHist.set(net.name, (swapHist.get(net.name) ?? 0) + 1);
				}
				const ln = networks.find(x => x.name === lost[0]);
				if (ln && !this.checkCancelled() && Date.now() < deadline) {
					// 趁热重布给双倍单网上限：掉出网救不回是换位的常态失败模式
					//（case9 实测 P01/+5V 掉出后 5s 不够），旧走廊刚腾出值得多等
					await this.routeNetEdges(ln, 'full', Math.min(deadline, Date.now() + perNet * 2));
					after = this.routedOrder.length;
				}
			}
			this.log(`拥挤协商：${net.name} 让路 [${rip.join(', ') || '无'}]，布通 ${before} → ${after}${lost.length ? `（掉出：${lost.join(', ')}${after > before ? '，已救回' : ''}）` : ''}`, 'success');
			return after === before ? 2 : 1;
		}
		// 回滚 + 历史罚分：反复卡在同一批网上时让 A* 学会绕开它们
		this.restoreNets(snap, savedOrder, networks);
		for (const c of rip) hist.set(c, (hist.get(c) ?? 0) + 1);
		if (__nd) this.log(`[neg] ${net.name} 让路 [${rip.join(', ')}] 无净增益（${before}→${after}），回滚`, 'info');
		return 0;
	}

	// ── PathFinder 终局（__RB_PF__=1）────────────────────
	// 与协商「全或无」接受的本质差异：失败网先**软提交**意向路径（重叠在
	// 阶段内合法存在），每个重叠独立解决，阶段末统一清判。实测前提：
	// case9/12 的失败网单独都能布通，卡点纯是「谁占走廊」的联合收敛。

	private pfTentative = new Set<string>();

	/**
	 * 软提交缺失边：硬障碍只剩静态+焊盘（异网铜皮全软化计罚分），过孔
	 * 只查焊盘净距与同网孔距（绝对 DRC），跨网重叠交给阶段末清判。
	 * 提交即入账 netRendered/plannedVias/动态障碍 —— 后续意向路径看得见
	 * 先行的重叠态，历史罚分逼惯犯走廊让位。
	 */
	private async pfTentativeRoute(
		net: { name: string; pads: EnginePad[] },
		hist: Map<string, number>,
		deadline: number,
	): Promise<boolean> {
		const edges = this.computeMST(net.pads);
		const done = this.routedEdges.get(net.name) ?? new Set<string>();
		const edgeKey = (si: number, di: number) => `${net.pads[si].id}->${net.pads[di].id}`;
		const missing = edges.filter(([si, di]) => !done.has(edgeKey(si, di)));
		if (!missing.length) return true;
		const gridCell = Number((globalThis as any).__RB_GRID_CELL__) || 4;
		const layers = this.hasBottomLayer ? [1, 2] : [1];
		// 历史平方放大（与协商罚分同款）：逼惯犯走廊让位的梯度
		const penaltyOf = (n: string): number => {
			const h = hist.get(n) ?? 0;
			return (40 + 8 * (this.allPads.filter(p => p.net === n).length)) * (1 + h) * (1 + h * 0.5);
		};
		let anyCommitted = false;
		for (const [si, di] of missing) {
			if (this.checkCancelled() || Date.now() > deadline) break;
			const src = net.pads[si], dst = net.pads[di];
			const t0 = Date.now();
			const res = await gridRoute({
				src: { x: src.x, y: src.y },
				dst: { x: dst.x, y: dst.y },
				srcPad: { x: src.x, y: src.y, r: src.bodyRadius || src.radius, viaKeepoutR: this.padViaKeepout(src) },
				dstPad: { x: dst.x, y: dst.y, r: dst.bodyRadius || dst.radius, viaKeepoutR: this.padViaKeepout(dst) },
				srcReleases: this.padReleases(src, gridCell),
				dstReleases: this.padReleases(dst, gridCell),
				srcPadLayers: this.padLayers(src),
				dstPadLayers: this.padLayers(dst),
				obstacles: [...this.staticObstacles, ...this.padObstacles(this.allPads, net.name)],
				penaltyObstacles: this.dynamicObstacles,
				penaltyOf,
				net: net.name,
				layers,
				cell: gridCell,
				viaCostMil: Math.max(150, this.drc.traceWidthMil * 22),
				...this.gridViaExtras(),
				viaInPad: this.allowViaInPad,
				budgetMs: Math.min(2500, Math.max(600, deadline - Date.now())),
				isCancelled: () => this.checkCancelled(),
			});
			this.gridBudgetMs -= Date.now() - t0;
			if (!res.ok || !res.prims.length) {
				// 网格无解的边（LCD 排：洁净板网格无解、可视图独走可解）——
				// 意向路由回退独走路径（同「洁净世界」口径，跨网重叠清判结算）
				const soft = this.dynamicObstacles;
				const solo = await this.soloEdgeCrossing(src, dst, net.name, soft);
				if (!solo) continue;
				const rds = this.netRendered.get(net.name) ?? [];
				rds.push(...solo.prims);
				this.netRendered.set(net.name, rds);
				this.addRenderedToObstacles(solo.prims);
				this.plannedVias.push(...solo.vias);
				for (const v of solo.vias) {
					this.dynamicObstacles.push({
						id: `via${this.obstacleIdCounter++}`, kind: 'circle', x: v.x, y: v.y,
						r: v.dia / 2 + Math.max(this.drc.trackToTrackMil, this.drc.trackToPadMil) + this.drc.traceWidthMil / 2 + VIA_WALL_EXTRA,
						net: net.name, layer: undefined, group: `via:${net.name}:${v.x},${v.y}`, noPrune: true,
					});
				}
				done.add(edgeKey(si, di));
				anyCommitted = true;
				for (const c of solo.crossed) hist.set(c, (hist.get(c) ?? 0) + 0.5);
				continue;
			}
			// 软提交：跨网重叠放行；焊盘净距与同网孔距仍是绝对口径
			const viaDia = Math.max(24, this.drc.traceWidthMil * 2);
			const viaHole = Math.max(12, this.drc.traceWidthMil);
			const needVV = viaHole + (this.drc.holeToHoleMil ?? 12) + 0.2;
			const newVias: PlannedVia[] = [];
			let viaOk = true;
			for (const v of res.vias) {
				if (!this.viaPadClear(v.x, v.y, viaDia / 2, net.name)) { viaOk = false; break; }
				for (const w of [...this.plannedVias.filter(x => x.net === net.name), ...newVias]) {
					if (Math.hypot(w.x - v.x, w.y - v.y) < needVV) { viaOk = false; break; }
				}
				if (!viaOk) break;
				newVias.push({ net: net.name, x: v.x, y: v.y, dia: viaDia, hole: viaHole });
			}
			if (!viaOk) continue;
			const prims: RenderedPrim[] = res.prims.map(p => p.kind === 'line'
				? { kind: 'line' as const, net: net.name, layer: p.layer, x1: p.x1, y1: p.y1, x2: p.x2, y2: p.y2 }
				: { kind: 'arc' as const, net: net.name, layer: p.layer, cx: p.cx, cy: p.cy, r: p.r, startAngle: p.startAngle, sweep: p.sweep });
			const rds = this.netRendered.get(net.name) ?? [];
			rds.push(...prims);
			this.netRendered.set(net.name, rds);
			this.addRenderedToObstacles(prims);
			this.plannedVias.push(...newVias);
			for (const v of newVias) {
				this.dynamicObstacles.push({
					id: `via${this.obstacleIdCounter++}`, kind: 'circle', x: v.x, y: v.y,
					r: v.dia / 2 + Math.max(this.drc.trackToTrackMil, this.drc.trackToPadMil) + this.drc.traceWidthMil / 2 + VIA_WALL_EXTRA,
					net: net.name, layer: undefined, group: `via:${net.name}:${v.x},${v.y}`, noPrune: true,
				});
			}
			done.add(edgeKey(si, di));
			anyCommitted = true;
			// 历史罚分：被本路径压到的网（含过孔落点）下轮更想绕开
			for (const c of res.crossed ?? []) hist.set(c, (hist.get(c) ?? 0) + 0.5);
		}
		this.routedEdges.set(net.name, done);
		this.pfTentative.add(net.name);
		return anyCommitted;
	}

	/** 跨网铜皮搭接检测（清判用）：本网任一图元与**已合法世界**的铜皮重叠即为脏 */
	private pfOverlapsOther(name: string, legalNets: Set<string>): boolean {
		const w = this.drc.traceWidthMil;
		const mine = this.netRendered.get(name) ?? [];
		const myVias = this.plannedVias.filter(v => v.net === name);
		for (const other of legalNets) {
			if (other === name) continue;
			const op = this.netRendered.get(other) ?? [];
			if (!op.length) continue;
			const ov = this.plannedVias.filter(v => v.net === other);
			for (const p of mine) {
				if (p.kind !== 'line') continue;
				const pminx = Math.min(p.x1!, p.x2!) - w, pmaxx = Math.max(p.x1!, p.x2!) + w;
				const pminy = Math.min(p.y1!, p.y2!) - w, pmaxy = Math.max(p.y1!, p.y2!) + w;
				const midX = (p.x1! + p.x2!) / 2, midY = (p.y1! + p.y2!) / 2;
				for (const q of op) {
					if (q.kind !== 'line' || q.layer !== p.layer) continue;
					if (pmaxx < Math.min(q.x1!, q.x2!) || pminx > Math.max(q.x1!, q.x2!)) continue;
					if (pmaxy < Math.min(q.y1!, q.y2!) || pminy > Math.max(q.y1!, q.y2!)) continue;
					if (pointSegDist(midX, midY, q.x1!, q.y1!, q.x2!, q.y2!) <= w) return true;
					if (pointSegDist(p.x1!, p.y1!, q.x1!, q.y1!, q.x2!, q.y2!) <= w) return true;
					if (pointSegDist(p.x2!, p.y2!, q.x1!, q.y1!, q.x2!, q.y2!) <= w) return true;
				}
				for (const v of ov) {
					if (v.x < pminx || v.x > pmaxx || v.y < pminy || v.y > pmaxy) continue;
					if (Math.hypot(v.x - midX, v.y - midY) <= (v.dia || 24) / 2 + w / 2) return true;
					if (Math.hypot(v.x - p.x1!, v.y - p.y1!) <= (v.dia || 24) / 2 + w / 2) return true;
					if (Math.hypot(v.x - p.x2!, v.y - p.y2!) <= (v.dia || 24) / 2 + w / 2) return true;
				}
			}
			for (const v of myVias) {
				for (const q of op) {
					if (q.kind !== 'line') continue;
					if (pointSegDist(v.x, v.y, q.x1!, q.y1!, q.x2!, q.y2!) <= (v.dia || 24) / 2 + w / 2) return true;
				}
			}
		}
		return false;
	}

	/**
	 * PathFinder 相位：软提交意向 → 迭代（历史罚分驱动换位）→ 清判。
	 * 清判按提交顺序贪心：与「已合法世界」无重叠的意向网转正；重叠网用
	 * 标准严格口径重布一次，仍不行则拆除（回到失败网，残段语义）。
	 */
	private async pathfinderPhase(networks: Array<{ name: string; pads: EnginePad[] }>, deadline: number): Promise<void> {
		const iters = Number((globalThis as any).__RB_PF_ITERS__) || 3;
		const hist = new Map<string, number>();
		this.gridBudgetMs = Math.max(this.gridBudgetMs, 60_000);
		// 快照整个合法世界：原合法网清判不达标时恢复原状 —— 相位绝不净损失
		const legalNames = [...this.routedOrder];
		const worldSnap = this.snapshotNets(legalNames);
		const savedOrder = [...this.routedOrder];
		this.pfTentative.clear();
		for (let it = 0; it < iters; it++) {
			if (this.checkCancelled() || Date.now() > deadline) break;
			// 全网络重布（真正的 PathFinder）：pending 优先，其后按历史罚分降序
			//（惯犯走廊先动）。合法网也参战 —— 只让失败网换道、占走廊者永远
			// 不动，墙就永远在（v0-v4 的核心教训）。
			const all = [...networks].sort((a, b) => {
				const pa = this.routedOrder.includes(a.name) || this.pfTentative.has(a.name) ? 1 : 0;
				const pb = this.routedOrder.includes(b.name) || this.pfTentative.has(b.name) ? 1 : 0;
				return pa - pb || (hist.get(b.name) ?? 0) - (hist.get(a.name) ?? 0) ||
					this.netPendingEdges(a) - this.netPendingEdges(b) || (a.name < b.name ? -1 : 1);
			});
			let rerouted = 0;
			for (const net of all) {
				if (this.checkCancelled() || Date.now() > deadline) break;
				this.ripupNetInternal(net.name, true);
				if (await this.pfTentativeRoute(net, hist, deadline)) rerouted++;
			}
			this.log(`PathFinder 第 ${it + 1} 轮：全网重布 ${rerouted}/${all.length}（tentative ${this.pfTentative.size}）`, 'info');
		}
		// ── 清判（两阶段）：原合法网先审（有快照兜底 + 走廊优先权），
		// pending 网后审（转正/重布/拆除）。严格口径 = EDA 真实要求。
		const legal = new Set<string>();
		let keptLegal = 0, restoredLegal = 0, promoted = 0, rerouted = 0, demoted = 0;
		for (const name of legalNames) {
			if (this.checkCancelled() || Date.now() > this.planDeadline - 5_000) break;
			const net = networks.find(n => n.name === name);
			if (!net) { legal.add(name); continue; }
			const prims = this.netRendered.get(name) ?? [];
			const vias = this.plannedVias.filter(v => v.net === name);
			const complete = (this.computeMST(net.pads)).every(([si, di]) =>
				(this.routedEdges.get(name) ?? new Set()).has(`${net.pads[si].id}->${net.pads[di].id}`));
			const routedSet = new Set(legal);
			const strictObs = this.dynamicObstacles.filter(o => !o.net || routedSet.has(o.net));
			let ok = complete && prims.length > 0 && this.primsClearOfDynamic(mergePrims(prims), name, strictObs);
			if (ok) {
				for (const v of vias) {
					if (this.viaPadClear(v.x, v.y, v.dia / 2, name) &&
						this.viaObstacleClear(v.x, v.y, strictObs, v.dia / 2, name)) continue;
					// 意向过孔落点微移复活（罚分网格的落点粗于引擎精细放置）：
					// 16 方向 × 0.4~3.0mil，衔接线端点同步平移（与提交链微移同款）
					let placed = false;
					const primsArr = this.netRendered.get(name) ?? [];
					const incident = primsArr.filter(p => p.kind === 'line' && this.primEndsNear(p, v.x, v.y, 1.0));
					if (incident.length && incident.every(p => p.kind === 'line')) {
						for (const dist of [0.4, 0.8, 1.2, 1.8, 2.4, 3.0]) {
							for (let k = 0; k < 16 && !placed; k++) {
								const a = (k * Math.PI) / 8;
								const nx = v.x + Math.cos(a) * dist, ny = v.y + Math.sin(a) * dist;
								if (!this.viaPadClear(nx, ny, v.dia / 2, name) ||
									!this.viaObstacleClear(nx, ny, strictObs, v.dia / 2, name)) continue;
								let swept = false;
								for (const p of incident) {
									if (p.kind !== 'line') continue;
									const ox = p.x1 ?? 0, oy = p.y1 ?? 0, ex = p.x2 ?? 0, ey = p.y2 ?? 0;
									const sx = Math.hypot(ox - v.x, oy - v.y) < 1.0 ? nx : ox;
									const sy = Math.hypot(ox - v.x, oy - v.y) < 1.0 ? ny : oy;
									const tx = Math.hypot(ex - v.x, ey - v.y) < 1.0 ? nx : ex;
									const ty = Math.hypot(ex - v.x, ey - v.y) < 1.0 ? ny : ey;
									if (!this.lineStillClear(sx, sy, tx, ty, p.layer, name)) { swept = true; break; }
								}
								if (swept) continue;
								for (const p of incident) {
									if (p.kind !== 'line') continue;
									if (Math.hypot((p.x1 ?? 0) - v.x, (p.y1 ?? 0) - v.y) < 1.0) { p.x1 = nx; p.y1 = ny; }
									else if (Math.hypot((p.x2 ?? 0) - v.x, (p.y2 ?? 0) - v.y) < 1.0) { p.x2 = nx; p.y2 = ny; }
								}
								v.x = nx; v.y = ny;
								placed = true;
							}
							if (placed) break;
						}
					}
					if (!placed) { ok = false; break; }
				}
			}
			if (!ok) {
				// 撕掉重布一次（严格口径、5s）；再不行恢复快照原状（原网必达标）
				this.ripupNetInternal(name, true);
				ok = await this.routeNetEdges(net, 'full', Math.min(this.planDeadline - 5_000, Date.now() + 5_000));
				if (!ok) {
					const sv = worldSnap.get(name);
					this.ripupNetInternal(name, true);
					if (sv && sv.prims.length) {
						this.netRendered.set(name, [...sv.prims]);
						this.addRenderedToObstacles(mergePrims(sv.prims));
					}
					this.routedEdges.set(name, new Set(sv?.doneEdges ?? []));
					this.plannedVias = this.plannedVias.filter(v => v.net !== name).concat(sv?.vias ?? []);
					for (const v of (sv?.vias ?? [])) {
						this.dynamicObstacles.push({
							id: `via${this.obstacleIdCounter++}`, kind: 'circle', x: v.x, y: v.y,
							r: v.dia / 2 + Math.max(this.drc.trackToTrackMil, this.drc.trackToPadMil) + this.drc.traceWidthMil / 2 + VIA_WALL_EXTRA,
							net: name, layer: undefined, group: `via:${name}:${v.x},${v.y}`, noPrune: true,
						});
					}
					restoredLegal++;
				} else {
					rerouted++;
				}
			} else {
				keptLegal++;
			}
			legal.add(name);
			if (!this.routedOrder.includes(name)) this.routedOrder.push(name);
		}
		// pending 网：与合法世界无冲突的意向转正，否则重布/拆除
		for (const name of [...this.pfTentative]) {
			if (this.checkCancelled() || Date.now() > this.planDeadline - 5_000) break;
			if (legal.has(name) || this.routedOrder.includes(name)) continue;
			const net = networks.find(n => n.name === name);
			if (!net) continue;
			const prims = this.netRendered.get(name) ?? [];
			const vias = this.plannedVias.filter(v => v.net === name);
			const complete = (this.computeMST(net.pads)).every(([si, di]) =>
				(this.routedEdges.get(name) ?? new Set()).has(`${net.pads[si].id}->${net.pads[di].id}`));
			const routedSet = new Set(legal);
			const strictObs = this.dynamicObstacles.filter(o => !o.net || routedSet.has(o.net));
			let ok = complete && prims.length > 0 && this.primsClearOfDynamic(mergePrims(prims), name, strictObs);
			if (ok) {
				for (const v of vias) {
					if (this.viaPadClear(v.x, v.y, v.dia / 2, name) &&
						this.viaObstacleClear(v.x, v.y, strictObs, v.dia / 2, name)) continue;
					// 意向过孔落点微移复活（罚分网格的落点粗于引擎精细放置）：
					// 16 方向 × 0.4~3.0mil，衔接线端点同步平移（与提交链微移同款）
					let placed = false;
					const primsArr = this.netRendered.get(name) ?? [];
					const incident = primsArr.filter(p => p.kind === 'line' && this.primEndsNear(p, v.x, v.y, 1.0));
					if (incident.length && incident.every(p => p.kind === 'line')) {
						for (const dist of [0.4, 0.8, 1.2, 1.8, 2.4, 3.0]) {
							for (let k = 0; k < 16 && !placed; k++) {
								const a = (k * Math.PI) / 8;
								const nx = v.x + Math.cos(a) * dist, ny = v.y + Math.sin(a) * dist;
								if (!this.viaPadClear(nx, ny, v.dia / 2, name) ||
									!this.viaObstacleClear(nx, ny, strictObs, v.dia / 2, name)) continue;
								let swept = false;
								for (const p of incident) {
									if (p.kind !== 'line') continue;
									const ox = p.x1 ?? 0, oy = p.y1 ?? 0, ex = p.x2 ?? 0, ey = p.y2 ?? 0;
									const sx = Math.hypot(ox - v.x, oy - v.y) < 1.0 ? nx : ox;
									const sy = Math.hypot(ox - v.x, oy - v.y) < 1.0 ? ny : oy;
									const tx = Math.hypot(ex - v.x, ey - v.y) < 1.0 ? nx : ex;
									const ty = Math.hypot(ex - v.x, ey - v.y) < 1.0 ? ny : ey;
									if (!this.lineStillClear(sx, sy, tx, ty, p.layer, name)) { swept = true; break; }
								}
								if (swept) continue;
								for (const p of incident) {
									if (p.kind !== 'line') continue;
									if (Math.hypot((p.x1 ?? 0) - v.x, (p.y1 ?? 0) - v.y) < 1.0) { p.x1 = nx; p.y1 = ny; }
									else if (Math.hypot((p.x2 ?? 0) - v.x, (p.y2 ?? 0) - v.y) < 1.0) { p.x2 = nx; p.y2 = ny; }
								}
								v.x = nx; v.y = ny;
								placed = true;
							}
							if (placed) break;
						}
					}
					if (!placed) { ok = false; break; }
				}
			}
			if (ok) {
				if (!this.routedOrder.includes(name)) this.routedOrder.push(name);
				legal.add(name);
				promoted++;
				continue;
			}
			this.ripupNetInternal(name, true);
			this.pfTentative.delete(name);
			if (await this.routeNetEdges(net, 'full', Math.min(this.planDeadline - 5_000, Date.now() + 5_000))) {
				legal.add(name);
				rerouted++;
			} else {
				demoted++;
			}
		}
		this.pfTentative.clear();
		this.log(`PathFinder 清判：合法保持 ${keptLegal}，恢复原状 ${restoredLegal}，转正 ${promoted}，重布 ${rerouted}，拆除 ${demoted}`, promoted || rerouted ? 'success' : 'info');
	}

	/**
	 * 冲突区限定版 PathFinder（__RB_PF_ZONE__=N 时替代全量版）：
	 * 只让「pending 网 + 被它们压到的合法网」进战区（cap N，实测 ~10-14 网），
	 * 战区内全网每轮重布（收敛力完整），区外冻结为合法背景。每轮成本
	 * 10-14s（对比全量版 40s）—— 4-5 轮装进 80s 窗口，结构性超支的解法。
	 * 原合法战区网有快照兜底：清判不达标恢复原状，相位绝不净损失。
	 */
	private async pfZonePhase(networks: Array<{ name: string; pads: EnginePad[] }>, deadline: number): Promise<void> {
		const iters = Number((globalThis as any).__RB_PF_ITERS__) || 5;
		const zoneCap = Number((globalThis as any).__RB_PF_ZONE__) || 14;
		const hist = new Map<string, number>();
		this.gridBudgetMs = Math.max(this.gridBudgetMs, 45_000);
		const pending0 = networks.filter(n => !this.routedOrder.includes(n.name) && this.allPads.filter(p => p.net === n.name).length > 1)
			.sort((a, b) => this.netPendingEdges(a) - this.netPendingEdges(b) || (a.name < b.name ? -1 : 1));
		if (!pending0.length) return;
		// 第 0 轮：只布 pending（软提交），用压越记录选出战区合法网
		this.pfTentative.clear();
		for (const net of pending0) {
			if (this.checkCancelled() || Date.now() > deadline) break;
			await this.pfTentativeRoute(net, hist, deadline);
		}
		const crossedLegal = [...hist.entries()].filter(([n]) => this.routedOrder.includes(n))
			.sort((a, b) => b[1] - a[1]).slice(0, Math.max(4, zoneCap - pending0.length)).map(([n]) => n);
		const zone = new Set<string>([...pending0.map(n => n.name), ...crossedLegal]);
		const zoneLegalNames = crossedLegal.filter(n => this.routedOrder.includes(n));
		const zoneSnap = this.snapshotNets(zoneLegalNames);
		const savedOrder = [...this.routedOrder];
		this.log(`[pf-zone] 战区 ${zone.size} 网（pending ${pending0.length} + 被压合法 ${crossedLegal.length}）：${[...zone].join(', ')}`, 'info');
		// 迭代：战区全网重布（pending 优先，其后按历史罚分）
		for (let it = 0; it < iters; it++) {
			if (this.checkCancelled() || Date.now() > deadline) break;
			const order = [...zone].map(n => networks.find(x => x.name === n)).filter((x): x is { name: string; pads: EnginePad[] } => !!x)
				.sort((a, b) => {
					const pa = this.routedOrder.includes(a.name) ? 1 : 0;
					const pb = this.routedOrder.includes(b.name) ? 1 : 0;
					return pa - pb || (hist.get(b.name) ?? 0) - (hist.get(a.name) ?? 0) ||
						this.netPendingEdges(a) - this.netPendingEdges(b) || (a.name < b.name ? -1 : 1);
				});
			let moved = 0;
			for (const net of order) {
				if (this.checkCancelled() || Date.now() > deadline) break;
				this.ripupNetInternal(net.name, true);
				if (await this.pfTentativeRoute(net, hist, deadline)) moved++;
			}
			this.log(`PathFinder(zone) 第 ${it + 1} 轮：战区重布 ${moved}/${zone.size}`, 'info');
		}
		// 清判：冻结世界自动合法；战区网按「原合法优先」贪心严格结算
		const legal = new Set<string>(this.routedOrder.filter(n => !zone.has(n)));
		let kept = 0, restored = 0, promoted = 0, rerouted = 0, demoted = 0;
		const settleOne = async (name: string, snapshot?: Map<string, { prims: RenderedPrim[]; vias: PlannedVia[]; doneEdges: Set<string> }>): Promise<void> => {
			if (this.checkCancelled() || Date.now() > this.planDeadline - 5_000) { legal.add(name); return; }
			const net = networks.find(n => n.name === name);
			if (!net) { legal.add(name); return; }
			const prims = this.netRendered.get(name) ?? [];
			const vias = this.plannedVias.filter(v => v.net === name);
			const complete = (this.computeMST(net.pads)).every(([si, di]) =>
				(this.routedEdges.get(name) ?? new Set()).has(`${net.pads[si].id}->${net.pads[di].id}`));
			const routedSet = new Set(legal);
			const strictObs = this.dynamicObstacles.filter(o => !o.net || routedSet.has(o.net));
			let ok = complete && prims.length > 0 && this.primsClearOfDynamic(mergePrims(prims), name, strictObs);
			if (ok) {
				for (const v of vias) {
					if (this.viaPadClear(v.x, v.y, v.dia / 2, name) &&
						this.viaObstacleClear(v.x, v.y, strictObs, v.dia / 2, name)) continue;
					// 意向过孔落点微移复活（罚分网格的落点粗于引擎精细放置）：
					// 16 方向 × 0.4~3.0mil，衔接线端点同步平移（与提交链微移同款）
					let placed = false;
					const primsArr = this.netRendered.get(name) ?? [];
					const incident = primsArr.filter(p => p.kind === 'line' && this.primEndsNear(p, v.x, v.y, 1.0));
					if (incident.length && incident.every(p => p.kind === 'line')) {
						for (const dist of [0.4, 0.8, 1.2, 1.8, 2.4, 3.0]) {
							for (let k = 0; k < 16 && !placed; k++) {
								const a = (k * Math.PI) / 8;
								const nx = v.x + Math.cos(a) * dist, ny = v.y + Math.sin(a) * dist;
								if (!this.viaPadClear(nx, ny, v.dia / 2, name) ||
									!this.viaObstacleClear(nx, ny, strictObs, v.dia / 2, name)) continue;
								let swept = false;
								for (const p of incident) {
									if (p.kind !== 'line') continue;
									const ox = p.x1 ?? 0, oy = p.y1 ?? 0, ex = p.x2 ?? 0, ey = p.y2 ?? 0;
									const sx = Math.hypot(ox - v.x, oy - v.y) < 1.0 ? nx : ox;
									const sy = Math.hypot(ox - v.x, oy - v.y) < 1.0 ? ny : oy;
									const tx = Math.hypot(ex - v.x, ey - v.y) < 1.0 ? nx : ex;
									const ty = Math.hypot(ex - v.x, ey - v.y) < 1.0 ? ny : ey;
									if (!this.lineStillClear(sx, sy, tx, ty, p.layer, name)) { swept = true; break; }
								}
								if (swept) continue;
								for (const p of incident) {
									if (p.kind !== 'line') continue;
									if (Math.hypot((p.x1 ?? 0) - v.x, (p.y1 ?? 0) - v.y) < 1.0) { p.x1 = nx; p.y1 = ny; }
									else if (Math.hypot((p.x2 ?? 0) - v.x, (p.y2 ?? 0) - v.y) < 1.0) { p.x2 = nx; p.y2 = ny; }
								}
								v.x = nx; v.y = ny;
								placed = true;
							}
							if (placed) break;
						}
					}
					if (!placed) { ok = false; break; }
				}
			}
			if (!ok) {
				this.ripupNetInternal(name, true);
				ok = await this.routeNetEdges(net, 'full', Math.min(this.planDeadline - 5_000, Date.now() + 4_000));
			}
			if (!ok && snapshot) {
				// 原合法战区网：恢复快照原状（走廊优先权，绝不净损失）
				this.ripupNetInternal(name, true);
				const sv = snapshot.get(name);
				if (sv && sv.prims.length) {
					this.netRendered.set(name, [...sv.prims]);
					this.addRenderedToObstacles(mergePrims(sv.prims));
				}
				this.routedEdges.set(name, new Set(sv?.doneEdges ?? []));
				this.plannedVias = this.plannedVias.filter(v => v.net !== name).concat(sv?.vias ?? []);
				for (const v of (sv?.vias ?? [])) {
					this.dynamicObstacles.push({
						id: `via${this.obstacleIdCounter++}`, kind: 'circle', x: v.x, y: v.y,
						r: v.dia / 2 + Math.max(this.drc.trackToTrackMil, this.drc.trackToPadMil) + this.drc.traceWidthMil / 2 + VIA_WALL_EXTRA,
						net: name, layer: undefined, group: `via:${name}:${v.x},${v.y}`, noPrune: true,
					});
				}
				ok = true;
				restored++;
			}
			if (ok) {
				if (!this.routedOrder.includes(name)) this.routedOrder.push(name);
				legal.add(name);
				if (snapshot) kept++; else promoted++;
			} else {
				demoted++;
			}
		};
		for (const name of zoneLegalNames) await settleOne(name, zoneSnap);
		for (const name of [...this.pfTentative]) {
			if (legal.has(name)) continue;
			await settleOne(name);
		}
		this.pfTentative.clear();
		this.log(`PathFinder(zone) 清判：合法保持 ${kept}，恢复 ${restored}，转正 ${promoted}，重布 ${rerouted}，拆除 ${demoted}`, promoted || rerouted ? 'success' : 'info');
	}

	/**
	 * 集群协商：互为让路对象的一批失败网（ pairwise 协商里 A 布通挤掉 B、
	 * B 重来又挤掉 A 的死循环，case9 LCD 排 / case12 DVP 排的实测形态）
	 * 整体撕掉，按「缺边少先走」重排，再重布被连带撕掉的已布网。
	 * 采纳条件 = 整板布通数严格增加；否则整体回滚 + 历史罚分。
	 */
	private async negotiateCluster(
		group: Array<{ name: string; pads: EnginePad[] }>,
		networks: Array<{ name: string; pads: EnginePad[] }>,
		hist: Map<string, number>,
		padCount: Map<string, number>,
		deadline: number,
		ripCache?: Map<string, string[] | null>,
		extraBlockers?: string[],
	): Promise<boolean> {
		const __nd = !!(globalThis as any).__RB_NEG_DEBUG__;
		// 成员让路集合并集（组内互让不算挡路者）
		const blockerHits = new Map<string, number>();
		// 换位历史的已布网强制入列（计数 2 优先于单次让路命中）：
		// 时间互锁的另一极 —— 本轮让路集里只有已布网时，互锁只能靠
		// 「谁被换出去过」还原（case9 LCD/P0x 墙的实测形态）
		for (const n of extraBlockers ?? []) {
			if (group.some(g => g.name === n)) continue;
			blockerHits.set(n, 2);
		}
		const members: Array<{ name: string; pads: EnginePad[] }> = [];
		for (const m of group) {
			if (this.checkCancelled() || Date.now() > deadline) break;
			let rs = ripCache?.get(m.name);
			if (rs === undefined) {
				rs = await this.computeNetRipSet(m, hist, padCount, deadline);
				ripCache?.set(m.name, rs);
			}
			if (rs === null) continue;   // 该成员独走都无路，不进集群
			members.push(m);
			for (const r of rs) {
				if (group.some(g => g.name === r)) continue;
				blockerHits.set(r, (blockerHits.get(r) ?? 0) + 1);
			}
		}
		if (members.length < 2) return false;
		const maxBlockers = Number((globalThis as any).__RB_NEG_MAXRIP__) || 10;
		// 挡路者：被多个成员压到的优先（撕一个解放多个），再按焊盘少优先
		const blockers = [...blockerHits.entries()]
			.sort((a, b) => b[1] - a[1] || (padCount.get(a[0]) ?? 9) - (padCount.get(b[0]) ?? 9) || (a[0] < b[0] ? -1 : 1))
			.slice(0, maxBlockers)
			.map(([n]) => n);
		// 重排顺序在撕毁前定（部分铜皮多的网缺边少，先走当骨架）
		const ordered = [...members].sort((a, b) => this.netPendingEdges(a) - this.netPendingEdges(b) || (a.name < b.name ? -1 : 1));

		const savedOrder = [...this.routedOrder];
		const names = [...members.map(m => m.name), ...blockers];
		const snap = this.snapshotNets(names);
		const before = this.routedOrder.length;
		for (const n of names) this.ripupNet(n);
		const perNet = Number((globalThis as any).__RB_NEG_NET_MS__) || 5000;
		const capAt = (): number => Math.min(deadline, Date.now() + perNet);
		for (const m of ordered) {
			if (this.checkCancelled() || Date.now() > deadline) break;
			await this.routeNetEdges(m, 'full', capAt());
		}
		const blockerNets = blockers.map(c => networks.find(x => x.name === c))
			.filter((x): x is { name: string; pads: EnginePad[] } => !!x)
			.sort((a, b) => a.pads.length - b.pads.length);
		for (const bn of blockerNets) {
			if (this.checkCancelled() || Date.now() > deadline) break;
			const ok = await this.routeNetEdges(bn, 'full', capAt());
			if (!ok && bn.pads.length === 2 && Date.now() < deadline) await this.tryViaFallback(bn);
		}
		const after = this.routedOrder.length;
		if (after > before) {
			this.log(`拥挤协商(集群)：[${members.map(m => m.name).join(', ')}] 让路 [${blockers.join(', ') || '无'}]，布通 ${before} → ${after}`, 'success');
			return true;
		}
		this.restoreNets(snap, savedOrder, networks);
		for (const c of blockers) hist.set(c, (hist.get(c) ?? 0) + 1);
		if (__nd) this.log(`[neg] 集群 [${members.map(m => m.name).join(', ')}] 无净增益（${before}→${after}），回滚`, 'info');
		return false;
	}

	/**
	 * 「借道」重排：挡路网只拆**本网独走路径上的那一段**，本网走过去后把
	 * 挡路网的断口在原地重新接上。
	 *
	 * 整网重排在密板上经常无处可去（长网要整体搬家，搬不动 → 整场回滚，
	 * 这是占满局部最小值的典型形态）；而断口重连只是「两个切口之间的局部
	 * 布线」，成功率高一个量级。安全约束与其它拆网动作一致：挡路网拆段后
	 * 接不回就整体回滚，绝不净损失已布铜皮；重连段过提交互检（含焊盘）。
	 */
	private async tryBypassReroute(
		net: { name: string; pads: EnginePad[] },
		blockerName: string,
		networks: Array<{ name: string; pads: EnginePad[] }>,
	): Promise<boolean> {
		const __bd = !!(globalThis as any).__RB_BYPASS_DEBUG__;
		const giveup = (why: string): boolean => {
			if (__bd) this.log(`[bypass] ${net.name}←${blockerName} 放弃: ${why}`, 'info');
			return false;
		};
		if (this.checkCancelled() || Date.now() > this.planDeadline) return giveup('截止');
		const blocker = networks.find(n => n.name === blockerName);
		if (!blocker || blocker.pads.length > 12) return giveup(`无网/焊盘多 ${blocker?.pads.length ?? '?'}`);
		const prims = this.netRendered.get(blockerName) ?? [];
		if (!prims.length) return giveup('挡路网无铜皮');

		// 本网独走路径（只避静态障碍+焊盘，不考虑其它网走线）
		const solo = await this.soloRoute(net);
		if (!solo || !solo.prims.length) return giveup('无独走路径');
		const soloPts = this.primPoints(solo.prims as unknown as RenderedPrim[], 5);
		const sep = this.drc.traceWidthMil + this.drc.trackToTrackMil;   // 平行走线中心线最小距

		// 挡路网压在独走路径上的图元。只借直线段：弧的切线衔接、跨层的
		// 过孔段（重连要换层）都留给整网撕毁处理。
		const blocked = prims.filter(p => {
			if (p.kind === 'arc') return false;
			const pts = this.primPoints([p], 4);
			for (const pt of pts) {
				for (const sp of soloPts) {
					if (Math.hypot(pt.x - sp.x, pt.y - sp.y) < sep) return true;
				}
			}
			return false;
		});
		if (!blocked.length) return giveup('挡路网不在独走路径上');
		const layers = new Set(blocked.map(p => p.layer));
		if (layers.size !== 1) {
			if ((globalThis as any).__RB_BYPASS_DEBUG__) this.log(`[bypass] ${net.name}←${blockerName} 跨层段 ${layers.size} 层，放弃`, 'info');
			return false;
		}
		const layer = [...layers][0];

		// 快照：挡路网全量 + 本网（可能带残段）
		const savedBlocker = {
			prims: [...prims],
			vias: this.plannedVias.filter(v => v.net === blockerName),
			doneEdges: new Set(this.routedEdges.get(blockerName) ?? []),
			order: this.routedOrder.includes(blockerName),
		};
		const savedSelf = {
			prims: [...(this.netRendered.get(net.name) ?? [])],
			vias: this.plannedVias.filter(v => v.net === net.name),
			doneEdges: new Set(this.routedEdges.get(net.name) ?? []),
		};
		const savedOrder = [...this.routedOrder];

		// 拆挡路网的走线障碍（过孔障碍保留——过孔不拆），重登保留段
		const blockedSet = new Set(blocked);
		const kept = prims.filter(p => !blockedSet.has(p));
		this.dynamicObstacles = this.dynamicObstacles.filter(o => !(o.net === blockerName && o.group?.startsWith('trk:')));
		if (kept.length) this.addRenderedToObstacles(mergePrims(kept));
		this.netRendered.set(blockerName, kept);

		// 本网在腾空的走廊上布线（残段续补，部分连通也算进展）
		const ok = await this.routeNetEdges(net, 'full', this.netSoft(this.planDeadline));

		// 切口 = 被拆段中与保留段相接、落在焊盘中心、或落在自身过孔上的端点
		//（过孔不动，是合法重连锚点；切口 1 的形态 = 段一端在过孔上）。
		// 切口按最近邻配对逐对重连；奇数个说明拆段边界有歧义，退回整网撕毁。
		const padKeys = new Set(blocker.pads.map(p => `${p.x.toFixed(1)},${p.y.toFixed(1)}`));
		const viaKeys = new Set(this.plannedVias.filter(v => v.net === blockerName).map(v => `${v.x.toFixed(1)},${v.y.toFixed(1)}`));
		const sharedWithKept = (px: number, py: number): boolean => {
			for (const q of kept) {
				if (Math.hypot((q.x1 ?? 0) - px, (q.y1 ?? 0) - py) < 0.6) return true;
				if (Math.hypot((q.x2 ?? 0) - px, (q.y2 ?? 0) - py) < 0.6) return true;
			}
			return false;
		};
		const freeEnds: Array<{ x: number; y: number }> = [];
		const seenEnd = new Set<string>();
		for (const p of blocked) {
			for (const [ex, ey] of [[p.x1 ?? 0, p.y1 ?? 0], [p.x2 ?? 0, p.y2 ?? 0]] as const) {
				const key = `${ex.toFixed(1)},${ey.toFixed(1)}`;
				if (seenEnd.has(key)) continue;
				seenEnd.add(key);
				// 连续拆段的端点：与保留链相接、或落在焊盘/自身过孔上，
				// 都是合法重连锚点（U 形拆段的两端接在保留臂上 —— 同样是切口）
				if (sharedWithKept(ex, ey) || padKeys.has(key) || viaKeys.has(key)) freeEnds.push({ x: ex, y: ey });
			}
		}
		let repaired = false;
		let reconnect: RenderedPrim[] = [];
		if (freeEnds.length >= 2 && freeEnds.length % 2 === 0 && freeEnds.length <= 6) {
			// 最近邻配对（同一连续拆段的两端互相配回）
			const pts = [...freeEnds];
			const pairs: Array<[{ x: number; y: number }, { x: number; y: number }]> = [];
			while (pts.length) {
				const a = pts.shift()!;
				let bi = 0, bd2 = Infinity;
				for (let i = 0; i < pts.length; i++) {
					const d = Math.hypot(pts[i].x - a.x, pts[i].y - a.y);
					if (d < bd2) { bd2 = d; bi = i; }
				}
				pairs.push([a, pts.splice(bi, 1)[0]]);
			}
			const obs = this.layerObstacles(blockerName, layer, true);
			let allOk = true;
			for (const [A, B] of pairs) {
				const r = await routeConnection(
					{ x: A.x, y: A.y }, { x: B.x, y: B.y },
					obs, blockerName, [150, 500], 2500,
					() => this.checkCancelled(),
				);
				if (!r.ok || !r.prims.length) {
					allOk = false;
					if (__bd) this.log(`[bypass] ${net.name}←${blockerName} 切口(${A.x.toFixed(0)},${A.y.toFixed(0)})→(${B.x.toFixed(0)},${B.y.toFixed(0)}) L${layer} 重连失败`, 'info');
					break;
				}
				reconnect.push(...r.prims.map(pr => pr.kind === 'line'
					? { kind: 'line' as const, net: blockerName, layer, x1: pr.x1, y1: pr.y1, x2: pr.x2, y2: pr.y2 }
					: { kind: 'arc' as const, net: blockerName, layer, cx: pr.cx, cy: pr.cy, r: pr.r, startAngle: pr.startAngle, sweep: pr.sweep }));
			}
			if (allOk && this.primsClearOfDynamic(reconnect, blockerName)) repaired = true;
			else if (allOk && __bd) this.log(`[bypass] ${net.name}←${blockerName} 重连段未过互检（${reconnect.length} 段）`, 'info');
		} else if (__bd) {
			this.log(`[bypass] ${net.name}←${blockerName} 切口数 ${freeEnds.length}（blocked=${blocked.length}）`, 'info');
		}

		if (ok && repaired) {
			const merged = this.smoothCorners(mergePrims([...kept, ...reconnect]), blockerName);
			this.netRendered.set(blockerName, merged);
			this.addRenderedToObstacles(merged);
			this.markAllEdgesDone(blocker);
			if (!savedBlocker.order && !this.routedOrder.includes(blockerName)) this.routedOrder.push(blockerName);
			if (!this.netIsConnected(blockerName, merged, this.plannedVias.filter(v => v.net === blockerName))) {
				// 防御：重连理应闭合，断开说明构造有洞 —— 按失败回滚
				repaired = false;
			}
		}
		if (ok && repaired) {
			this.log(`借道：${net.name} 从 ${blockerName} 的走廊通过，断口已重连`, 'success');
			this.callbacks.onNetUpdate?.(blockerName, mergePrims(this.netRendered.get(blockerName) ?? []), this.plannedVias.filter(v => v.net === blockerName), true, this.padProgress(blocker));
			return true;
		}

		// 回滚：本网新图元撤掉、挡路网整网恢复
		this.ripupNetInternal(net.name, true);
		if (savedSelf.prims.length) {
			this.netRendered.set(net.name, [...savedSelf.prims]);
			this.addRenderedToObstacles(mergePrims(savedSelf.prims));
		}
		this.routedEdges.set(net.name, new Set(savedSelf.doneEdges));
		this.plannedVias = this.plannedVias.filter(v => v.net !== net.name).concat(savedSelf.vias);
		this.ripupNetInternal(blockerName, true);
		if (savedBlocker.prims.length) {
			this.netRendered.set(blockerName, [...savedBlocker.prims]);
			this.addRenderedToObstacles(mergePrims(savedBlocker.prims));
		}
		this.routedEdges.set(blockerName, new Set(savedBlocker.doneEdges));
		this.plannedVias = this.plannedVias.filter(v => v.net !== blockerName).concat(savedBlocker.vias);
		for (const v of this.plannedVias) {
			if (v.net !== blockerName) continue;
			this.dynamicObstacles.push({
				id: `via${this.obstacleIdCounter++}`, kind: 'circle', x: v.x, y: v.y,
				r: v.dia / 2 + Math.max(this.drc.trackToTrackMil, this.drc.trackToPadMil) + this.drc.traceWidthMil / 2 + VIA_WALL_EXTRA,
				net: v.net, layer: undefined, group: `via:${v.net}:${v.x},${v.y}`, noPrune: true,
			});
		}
		this.routedOrder = savedOrder;
		this.callbacks.onNetUpdate?.(blockerName, mergePrims(savedBlocker.prims), savedBlocker.vias, savedBlocker.prims.length > 0, this.padProgress(blocker));
		this.callbacks.onNetUpdate?.(net.name, mergePrims(savedSelf.prims), savedSelf.vias, savedSelf.prims.length > 0, this.padProgress(net));
		return false;
	}

	/**
	 * hoist 重排：把沿本网 MST 走廊阻挡的已布网络挪到本网之后。
	 *
	 * 与「撕毁重布」的区别：后者按端点附近的封口挑对象（局部），hoist 按
	 * **整条走廊的遮挡量**挑对象（全局）——拥塞板上失败网常被中段的一两条
	 * 长线挡住，端点处却什么都没抓到。
	 *
	 * 安全约束与其它拆网动作一致：先快照，本网布通且被挪网全部布回才接受，
	 * 否则整体回滚，绝不净损失已布网络。被挪网最多 2 个、焊盘数 ≤12
	 * （重布代价与焊盘数成正比），全局尝试次数受限。
	 */
	private async tryHoistReroute(
		net: { name: string; pads: EnginePad[] },
		networks: Array<{ name: string; pads: EnginePad[] }>,
	): Promise<boolean> {
		if (this.checkCancelled() || Date.now() > this.planDeadline) return false;
		const corridor = this.drc.trackToTrackMil + this.drc.traceWidthMil + 20;
		const score = new Map<string, number>();
		for (const [ia, ib] of this.computeMST(net.pads)) {
			const pa = net.pads[ia], pb = net.pads[ib];
			for (const o of this.dynamicObstacles) {
				if (o.soft || !o.net || o.net === net.name) continue;
				if (!this.routedOrder.includes(o.net)) continue;
				if (pointSegDist(o.x, o.y, pa.x, pa.y, pb.x, pb.y) < o.r + corridor) {
					score.set(o.net, (score.get(o.net) ?? 0) + 1);
				}
			}
		}
		if (!score.size) return false;
		const picks = [...score.entries()].sort((a, b) => b[1] - a[1]).slice(0, Number((globalThis as any).__RB_HOIST_PICKS__) || 2)
			.map(([n]) => networks.find(x => x.name === n))
			.filter((n): n is { name: string; pads: EnginePad[] } => !!n)
			.filter(n => n.pads.length <= (Number((globalThis as any).__RB_HOIST_PADS__) || 12));
		if (!picks.length) return false;

		// 快照：被挪网 + 本网（本网可能带部分走线）
		const savedOrder = [...this.routedOrder];
		const saved = new Map<string, { prims: RenderedPrim[]; vias: PlannedVia[]; idx: number; doneEdges: Set<string> }>();
		for (const p of picks) {
			saved.set(p.name, {
				prims: [...(this.netRendered.get(p.name) ?? [])],
				vias: this.plannedVias.filter(v => v.net === p.name),
				idx: this.routedOrder.indexOf(p.name),
				doneEdges: new Set(this.routedEdges.get(p.name) ?? []),
			});
		}
		const savedSelf = {
			prims: [...(this.netRendered.get(net.name) ?? [])],
			vias: this.plannedVias.filter(v => v.net === net.name),
			doneEdges: new Set(this.routedEdges.get(net.name) ?? []),
		};
		for (const p of picks) this.ripupNet(p.name);

		const okNet = await this.routeNetEdges(net);
		let okAll = okNet;
		if (okNet) {
			// 被挪网按「先布的原顺序」补回（逆序撕裂，正序补回更容易贴合）
			for (let i = picks.length - 1; i >= 0; i--) {
				if (!(await this.routeNetEdges(picks[i]))) { okAll = false; break; }
			}
		}

		if (okNet && okAll) {
			this.log(`hoist：${net.name} 提前于 ${picks.map(p => p.name).join(', ')} 布通`, 'success');
			return true;
		}

		// 回滚：本网与被挪网全部恢复到动作前
		this.ripupNetInternal(net.name, true);
		for (const p of picks) this.ripupNetInternal(p.name, true);
		if (savedSelf.prims.length) {
			this.netRendered.set(net.name, [...savedSelf.prims]);
			this.addRenderedToObstacles(mergePrims(savedSelf.prims));
		}
		this.routedEdges.set(net.name, new Set(savedSelf.doneEdges));
		this.plannedVias = this.plannedVias.filter(v => v.net !== net.name).concat(savedSelf.vias);
		for (const p of picks) {
			const sv = saved.get(p.name)!;
			if (sv.prims.length) {
				this.netRendered.set(p.name, [...sv.prims]);
				this.addRenderedToObstacles(mergePrims(sv.prims));
			}
			this.plannedVias = this.plannedVias.filter(v => v.net !== p.name).concat(sv.vias);
			// 恢复被撕网**真实的 doneEdges**：部分走线网（残段参选）若在这里
			// 被标成全部 done，后续 routeNetEdges 会看到「无边可布」直接判
			// complete 进 routedOrder —— pad 悬空的布通网（P02 实测，EDA 判
			// Connection Error）。line 715 的老注释早已警告过这一坑。
			this.routedEdges.set(p.name, new Set(sv.doneEdges));
		}
		// 过孔障碍重新登记 + 顺序恢复
		for (const v of this.plannedVias) {
			if (v.net !== net.name && !picks.some(p => p.name === v.net)) continue;
			this.dynamicObstacles.push({
				id: `via${this.obstacleIdCounter++}`, kind: 'circle',
				x: v.x, y: v.y,
				r: v.dia / 2 + Math.max(this.drc.trackToTrackMil, this.drc.trackToPadMil) + this.drc.traceWidthMil / 2 + VIA_WALL_EXTRA,
				net: v.net, layer: undefined, group: `via:${v.net}:${v.x},${v.y}`, noPrune: true,
			});
		}
		this.routedOrder = savedOrder;
		// 画布同步
		for (const p of picks) {
			const sv = saved.get(p.name)!;
			this.callbacks.onNetUpdate?.(p.name, mergePrims(sv.prims), sv.vias, sv.prims.length > 0, this.padProgress(p));
		}
		this.callbacks.onNetUpdate?.(net.name, mergePrims(savedSelf.prims), savedSelf.vias, savedSelf.prims.length > 0, this.padProgress(net));
		return false;
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
		directOnly = false,
		budgetMs = 3000,
	): Promise<RenderedPrim[] | null> {
		const o = obs ?? this.layerObstacles(netName, layer, true);
		const r = await routeConnection(
			{ x: src.x, y: src.y }, { x: dst.x, y: dst.y },
			o, netName, [150, 500], budgetMs,
			() => this.checkCancelled(),
			directOnly,
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
		asDst = false,
	): Promise<RenderedPrim[] | null> {
		const vx = other.x - pad.x, vy = other.y - pad.y;
		const dist = Math.hypot(vx, vy);
		if (dist < 1) return null;
		// 排法向 = 排向旋转 90°；对侧焊盘几乎在排向上（贴排走线）时预留无意义
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
		// 预留侧：朝对侧焊盘方向冲出（asDst 时路径从对侧过来，同样面向来向）
		const side = (vx * nx + vy * ny) >= 0 ? 1 : -1;
		const filletR = Math.min(24, this.drc.traceWidthMil * 3);
		const mapPath = (prims: RenderedPrim[]): RenderedPrim[] => prims.map(pr => pr.kind === 'line'
			? { kind: 'line' as const, net: netName, layer, x1: pr.x1, y1: pr.y1, x2: pr.x2, y2: pr.y2 }
			: { kind: 'arc' as const, net: netName, layer, cx: pr.cx, cy: pr.cy, r: pr.r, startAngle: pr.startAngle, sweep: pr.sweep });
		const stubOf = (ex: number, ey: number): RenderedPrim =>
			asDst
				? { kind: 'line', net: netName, layer, x1: ex, y1: ey, x2: pad.x, y2: pad.y }
				: { kind: 'line', net: netName, layer, x1: pad.x, y1: pad.y, x2: ex, y2: ey };
		for (const s of [side, -side]) {
			const ex = pad.x + nx * s * L, ey = pad.y + ny * s * L;
			if (!stubFeasible(term, ex, ey, obs, netName)) continue;
			// asDst：对侧焊盘 → 冲出点；否则冲出点 → 对侧焊盘
			const r = asDst
				? await routeConnection({ x: other.x, y: other.y }, { x: ex, y: ey }, obs, netName, [150, 500], 3000, () => this.checkCancelled())
				: await routeConnection({ x: ex, y: ey }, { x: other.x, y: other.y }, obs, netName, [150, 500], 3000, () => this.checkCancelled());
			if (!r.ok || !r.prims.length) continue;
			const path = mapPath(r.prims as RenderedPrim[]);
			const stub = stubOf(ex, ey);
			// 拐点圆角（G1 切线过渡）：预留段与路径的接缝常呈大角度
			let prims: RenderedPrim[];
			if (asDst) {
				const last = path[path.length - 1];
				const head = last && last.kind === 'line'
					? fanoutFillet(
						{ x1: last.x1!, y1: last.y1!, x2: last.x2!, y2: last.y2! },
						{ x1: stub.x1!, y1: stub.y1!, x2: stub.x2!, y2: stub.y2! },
						filletR,
					)
					: null;
				// fanoutFillet 是纯几何：返回的图元不带 net/layer，必须在此补齐
				//（实测缺层图元进板 = 障碍横跨两层 + 层错乱渲染，审计还查不出层归属）
				prims = head
					? [...path.slice(0, -1), ...(head.prims as RenderedPrim[]).map(q => ({ ...q, net: netName, layer }))]
					: [...path, stub];
			} else {
				const head = path.length >= 1 && path[0].kind === 'line'
					? fanoutFillet(
						{ x1: stub.x1!, y1: stub.y1!, x2: stub.x2!, y2: stub.y2! },
						{ x1: path[0].x1!, y1: path[0].y1!, x2: path[0].x2!, y2: path[0].y2! },
						filletR,
					)
					: null;
				prims = head ? [...(head.prims as RenderedPrim[]).map(q => ({ ...q, net: netName, layer })), ...path.slice(1)] : [stub, ...path];
			}
			return prims;
		}
		return null;
	}

	/**
	 * 失败边诊断（__RB_BLOCKERS__=1 时启用）：本边两端扇出带与全走廊上
	 * 分别被哪些已布网络占据（按采样点命中率排序）。用于设计挡路网撕毁
	 * 策略——数据先行，不猜。
	 */
	private logEdgeBlockers(src: EnginePad, dst: EnginePad, netName: string): void {
		const segBlocked = (ax: number, ay: number, bx: number, by: number): { perNet: Map<string, number>; steps: number } => {
			const len = Math.hypot(bx - ax, by - ay);
			const steps = Math.max(2, Math.ceil(len / 6));
			const perNet = new Map<string, number>();
			for (let s = 0; s <= steps; s++) {
				const px = ax + (bx - ax) * (s / steps), py = ay + (by - ay) * (s / steps);
				for (const o of this.dynamicObstacles) {
					if (o.soft || !o.net || o.net === netName) continue;
					const g = o.group ?? '';
					if (!g.startsWith('trk:') && !g.startsWith('via:')) continue;
					if (Math.hypot(o.x - px, o.y - py) < o.r) perNet.set(o.net, (perNet.get(o.net) ?? 0) + 1);
				}
			}
			return { perNet, steps };
		};
		const fmt = (r: { perNet: Map<string, number>; steps: number }): string => {
			const total = r.steps + 1;
			return [...r.perNet.entries()].sort((a, b) => b[1] - a[1]).slice(0, 3)
				.map(([n, c]) => `${n}:${Math.round((100 * c) / total)}%`).join(',') || 'clear';
		};
		const parts: string[] = [];
		for (const [tag, pad] of [['src', src], ['dst', dst]] as const) {
			const row = this.rowInfo.get(pad.id);
			if (!row) continue;
			const L = row.gap * 1.5 + 40;
			const nx = -row.dy, ny = row.dx;
			for (const s of [1, -1] as const) {
				const r = segBlocked(pad.x, pad.y, pad.x + nx * s * L, pad.y + ny * s * L);
				parts.push(`${tag}-fan${s > 0 ? '+' : '-'}@${Math.round(L)}=${fmt(r)}`);
			}
		}
		parts.push(`corridor=${fmt(segBlocked(src.x, src.y, dst.x, dst.y))}`);
		this.log(`[blockers] ${netName} ${src.id}→${dst.id} ${parts.join(' ')}`, 'info');
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
	/**
	 * 过孔落点精确净距（与终检 repairViaClearance.badAt 完全同口径）：
	 * 走线按 netRendered 真实图元求距、钻孔-钻孔、过孔-过孔。
	 * 焊盘由 viaPadClear 负责（精确形状），不含在此。
	 *
	 * 为什么必须精确：提交链旧口径 viaObstacleClear 用的是采样圆障碍，
	 * 圆与圆之间有采样缝 —— 过孔从缝里漏过去（实测 case12 提交时 20~22mil
	 * 的过孔-走线对多达十几个），终检按真实几何一查一个准，拆孔断网，
	 * 拥挤协商赚到的 71~73 网被拆回 67。提交口径必须 ≥ 终检口径，
	 * 违规才会在**求解时**被拒（换个落点），而不是提交后被拆（整网损失）。
	 */
	private viaExactBad(x: number, y: number, netName: string, viaR: number, viaHole: number, skipVia?: PlannedVia): string | null {
		const drc = this.drc;
		// 阈值 = EDA 实测要求（viaR + trackToPad + halfW，case12 实测 22.98mil）
		// + 0.1 余量。必须与 repairViaClearance.badAt 完全一致：提交口径更松 →
		// 终检拆除级联（整网损失）；更紧 → 冤杀合法落点（case12 实测 +0.4 时
		// 主阶段基础布通 68 → 56）。
		const need = viaR + drc.trackToPadMil + drc.traceWidthMil / 2 + 0.1;
		for (const [nm, prims] of this.netRendered) {
			if (nm === netName) continue;
			for (const p of prims) {
				// 包围盒快拒（长走线板 netRendered 有数千图元，逐候选全量求距太贵）
				if (p.kind === 'line') {
					const minx = Math.min(p.x1!, p.x2!), maxx = Math.max(p.x1!, p.x2!);
					const miny = Math.min(p.y1!, p.y2!), maxy = Math.max(p.y1!, p.y2!);
					if (x + need < minx || x - need > maxx || y + need < miny || y - need > maxy) continue;
				} else {
					const r = (p.r ?? 0) + need;
					if (Math.abs((p.cx ?? 0) - x) > r || Math.abs((p.cy ?? 0) - y) > r) {
						// 圆弧外接盒不相交必安全；相交时内圈也要查（圆心距 < r - 弧半径 同样违规不可能，跳过）
						const inner = Math.abs(Math.hypot((p.cx ?? 0) - x, (p.cy ?? 0) - y) - (p.r ?? 0));
						if (inner >= need) continue;
					}
				}
				const d = this.primPointDist(p, x, y);
				if (d < need) return `${nm} 走线 ${d.toFixed(2)}mil`;
			}
		}
		// 钻孔-钻孔：EDA 的 Hole to Hole 规则按「孔壁到孔壁」判定
		//（otherClearance.hole2Hole，JLC 默认 0.3mm≈11.8mil），**同网也查**。
		// 旧口径 max(tt,tp)≈6mil 太松：case12 E2E 实测同网双过孔边距 11.02mil、
		// 过孔叠过孔 d=0 共 3 处 Hole to Hole 违规。
		const h2h = drc.holeToHoleMil ?? 12;
		const viaHoleR = (viaHole || 12) / 2;
		for (const pd of this.allPads) {
			if (pd.net === netName) continue;
			const cachedHole = Math.max(
				Number(this.padShapes[`${pd.x.toFixed(2)},${pd.y.toFixed(2)}`]?.hole?.[1]) || 0,
				Number(this.padShapes[`${pd.x.toFixed(2)},${pd.y.toFixed(2)}`]?.hole?.[2]) || 0);
			const holeD = (pd.holeDiameter ?? 0) || cachedHole;
			if (!(holeD > 0)) continue;
			const d = Math.hypot(pd.x - x, pd.y - y);
			const needHH = holeD / 2 + viaHoleR + h2h + 0.2;
			if (d < needHH) return `${pd.net} 钻孔 ${d.toFixed(2)}mil`;
		}
		// 过孔-过孔：异网按铜环间距（Safe Spacing Via↔Via + 环宽）；
		// 同网铜皮可相连但**钻孔仍要孔壁间距**（同网重叠过孔 = Hole to Hole d=0）
		for (const w of this.plannedVias) {
			if (w === skipVia) continue;
			const d = Math.hypot(w.x - x, w.y - y);
			const needVV = w.net === netName
				? viaHoleR + (w.hole || 12) / 2 + h2h + 0.2
				: viaR + w.dia / 2 + Math.max(drc.trackToTrackMil, drc.trackToPadMil) + 0.4;
			if (d < needVV) return `${w.net} 过孔 ${d.toFixed(2)}mil`;
		}
		return null;
	}

	private viaObstacleClear(x: number, y: number, obs: RBObstacle[], viaR: number, netName: string): boolean {
		// 焊盘净距用精确形状单独校验（下方循环里跳过圆近似的焊盘障碍，
		// 避免密排焊盘的外接圆冤杀合法落点）。**必须在这里做**：过孔落点
		// 校验的所有路径都经过本函数，之前只在部分调用点补 viaPadClear，
		// viaEdgeCandidate 的落点因此漏检 —— 实测过孔被放到距异网长焊盘
		// 17.94mil（EDA 判定差 0.04mil），且集中在 LED/LCD 连接器排。
		if (!this.viaPadClear(x, y, viaR, netName)) return false;
		// 精确口径（与终检一致）：采样圆障碍有缝，过孔能从缝里漏到距真实
		// 走线 20~22mil 的位置，终检再拆（case12 实测拆掉协商赚的 4~6 网）
		if (this.viaExactBad(x, y, netName, viaR, Math.max(12, this.drc.traceWidthMil))) return false;
		const viaExtra = viaR - this.drc.traceWidthMil / 2;
		for (const o of obs) {
			if (o.soft) continue;
			if (o.net === netName) continue;
			// 焊盘障碍已由 viaPadClear 按真实形状精确校验（圆外扩近似会冤杀
			// 1~5mil 的合法落点），这里只复查走线 / 静态 / 过孔障碍
			if (o.group?.startsWith('pad:')) continue;
			// EDA 的 Track↔Via 规则按焊盘间距执行（大于走线间距）
			if (Math.hypot(x - o.x, y - o.y) <
				o.r + viaExtra + Math.max(0, this.drc.trackToPadMil - this.drc.trackToTrackMil) + VIA_WALL_EXTRA) return false;
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
				if (this.checkCancelled() || Date.now() > this.planDeadline) return out;
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
			const good = grid.filter(s => s.kind !== 'router').slice(0, this.viaBreadth >= 12 ? 10 : 6);
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
					stub = f
						? (f.prims as RenderedPrim[]).map(q => ({ ...q, net: netName, layer: 1 }))   // fanoutFillet 纯几何：补 net/layer
						: [s1, s2];
				}
				out.push({ x: g.x, y: g.y, stub });
			}
			// 直线 / L 形全灭时落点仍可能可达（邻盘净距圈之间有斜向通道）：
			// 用可视图路由器铺顶层引出段，可绕开直线与单拐覆盖不到的形态。
			if (good.length === 0) {
				let routerOk = 0;
				for (const g of grid.filter(s => s.kind === 'router').slice(0, 6)) {
					if (this.checkCancelled() || Date.now() > this.planDeadline) return out;
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
		src: EnginePad, dst: EnginePad, netName: string, pending?: RenderedPrim[],
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
		if ((globalThis as any).__RB_GRID_DEBUG__) {
			this.log(`[via-debug] ${netName} ${src.id}(L${src.layer})→${dst.id}(L${dst.layer}) anchorsA=${anchorsA.length} anchorsB=${anchorsB.length}`, 'info');
		}
		if (!anchorsA.length || !anchorsB.length) {
			if ((globalThis as any).__RB_TIGHTEN_DIAG__) {
				this.log(`[via-diag] ${netName} ${src.id}(L${src.layer})→${dst.id}(L${dst.layer}) 锚点不足 A=${anchorsA.length} B=${anchorsB.length}`, 'info');
			}
			return null;
		}

		// 按「引出段总长 + 底层直线距离」升序逐对尝试落点：
		// 估值最优的一对底层未必走得通（会被先布网络的斜线封死），
		// 失败就换下一对，而不是直接放弃。
		// 形态惩罚（等效 mil）：只按长度挑落点会选出 45° 斜引出段，
		// ① 在过孔处与底层段折成锐角（视觉上的 V 形折返）；
		// ② 在焊盘处与本网已有出线夹成楔形锐角（酸角/难加工）。
		const existing = [...(this.netRendered.get(netName) ?? []), ...(pending ?? [])];
		const turnPenalty = (a: Anchor, b: Anchor): number => {
			let pen = 0;
			const bx = b.x - a.x, by = b.y - a.y;
			const bl = Math.hypot(bx, by);
			const turnAt = (d1: { x: number; y: number } | null, d2x: number, d2y: number): number => {
				if (!d1 || bl < 1) return 0;
				const c = (d1.x * d2x + d1.y * d2y) / bl;
				const deg = Math.acos(Math.max(-1, Math.min(1, c))) * 180 / Math.PI;
				return deg > 45 ? (deg - 45) * 1.5 : 0;
			};
			// A 端：引出段到达过孔的行进方向 → 底层 A→B 方向
			if (a.via) pen += turnAt(stubEndDir(a.stub), bx, by);
			// B 端：底层到达过孔后沿引出段反向走回焊盘
			if (b.via) {
				const d = stubEndDir(b.stub);
				pen += turnAt(d ? { x: -d.x, y: -d.y } : null, bx, by);
			}
			pen += wedgePenalty(src, a.stub) + wedgePenalty(dst, b.stub);
			return pen;
		};
		const wedgePenalty = (pad: EnginePad, stub: RenderedPrim[]): number => {
			if (!stub.length) return 0;
			const s = primDepartDir(stub[0], pad.x, pad.y);
			if (!s) return 0;
			let pen = 0;
			for (const p of existing) {
				const e = primDepartDir(p, pad.x, pad.y);
				if (!e) continue;
				const incl = Math.acos(Math.max(-1, Math.min(1, s.x * e.x + s.y * e.y))) * 180 / Math.PI;
				if (incl < 90) pen += (90 - incl) * 2;
			}
			return pen;
		};
		const pairs: Array<{ a: Anchor; b: Anchor; est: number }> = [];
		for (const a of anchorsA) {
			for (const b of anchorsB) {
				pairs.push({ a, b, est: a.stubLen + b.stubLen + Math.hypot(a.x - b.x, a.y - b.y) + turnPenalty(a, b) });
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
		let tried = 0;
		for (const { a, b } of pairs.slice(0, this.viaBreadth)) {
			if (this.checkCancelled()) return null;
			tried++;
			const bp = await bottomFor(a, b);
			if (bp) { pickA = a; pickB = b; bottomPrims = bp; break; }
			if ((globalThis as any).__RB_GRID_DEBUG__) {
				this.log(`[via-debug] ${netName} pair#${tried} A(${a.x.toFixed(0)},${a.y.toFixed(0)}) B(${b.x.toFixed(0)},${b.y.toFixed(0)}) 底层路径失败`, 'warn');
			}
		}
		if (!pickA || !pickB || !bottomPrims) {
			if ((globalThis as any).__RB_GRID_DEBUG__) {
				this.log(`[via-debug] ${netName} ${src.id}→${dst.id} 全部 ${tried} 对候选失败`, 'warn');
			}
			if ((globalThis as any).__RB_TIGHTEN_DIAG__) {
				this.log(`[via-diag] ${netName} ${src.id}(L${src.layer})→${dst.id}(L${dst.layer}) ${tried} 对候选全败（底层路径不通/微移失败）`, 'info');
			}
			return null;
		}

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
				if (this.checkCancelled() || Date.now() > this.planDeadline) return false;
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
						x: v.x, y: v.y, r: v.dia / 2 + Math.max(this.drc.trackToTrackMil, this.drc.trackToPadMil) + this.drc.traceWidthMil / 2 + VIA_WALL_EXTRA,
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
			// 精确口径（与终检一致）：采样圆障碍有缝，见 viaExactBad 注释
			if (this.viaExactBad(x, y, net.name, viaR, viaHole)) return false;
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
		for (const [endA, endB] of pairs.slice(0, this.viaBreadth >= 12 ? 10 : 4)) {
			if (this.checkCancelled() || Date.now() > this.planDeadline) return false;
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
			// 提交互检（与 routeNetEdges 同一道安全网）：兜底路径此前完全不校验，
			// 实测 Track to Via / SMD Pad to Via 违规来自这里
			if (!this.primsClearOfDynamic(mergedRender, net.name)) {
				this.log(`过孔兜底 ${net.name}：提交互检发现间距违规，换下一组候选`, 'warn');
				continue;
			}
			let viaBlocked = false;
			for (const v of vias) {
				if (!this.viaObstacleClear(v.x, v.y, this.dynamicObstacles, viaR, net.name)) { viaBlocked = true; break; }
			}
			if (viaBlocked) {
				this.log(`过孔兜底 ${net.name}：过孔落点与既有铜皮过近，换下一组候选`, 'warn');
				continue;
			}
			const rds = this.netRendered.get(net.name) ?? [];
			rds.push(...mergedRender);
			this.netRendered.set(net.name, rds);
			this.plannedVias.push(...vias);
			this.addRenderedToObstacles(mergedRender);
			for (const v of vias) {
				this.dynamicObstacles.push({
					id: `via${this.obstacleIdCounter++}`,
					kind: 'circle', x: v.x, y: v.y,
					r: viaR + Math.max(drc.trackToTrackMil, drc.trackToPadMil) + drc.traceWidthMil / 2 + VIA_WALL_EXTRA,
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
			if (Date.now() > this.optDeadline - 5_000) {
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
						x: v.x, y: v.y, r: v.dia / 2 + Math.max(this.drc.trackToTrackMil, this.drc.trackToPadMil) + this.drc.traceWidthMil / 2 + VIA_WALL_EXTRA,
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
			if (Date.now() > this.optDeadline - 5_000) {
				this.log('最终优化：时间预算用尽，提前收尾', 'warn');
				break;
			}
			const name = names[i];
			if (!this.routedOrder.includes(name)) continue;
			const net = networks.find(n => n.name === name);
			if (!net) continue;

			this.progress(
				this.routedPct(networks),
				`优化网络 ${i + 1}/${names.length}: ${name}（已布通 ${this.routedOrder.length}/${networks.length}）`,
			);
			const savedPrims = [...(this.netRendered.get(name) ?? [])];
			const savedVias = this.plannedVias.filter(v => v.net === name);
			const savedLen = renderedLength(savedPrims);

			// 快筛：已经贴着几何下界（MST 直线和）的网，重布不可能再短 2%
			//（采纳门槛也是 ≥2% 且 ≥8mil），直接跳过整网「撕毁 + 重解」。
			// 实测最终优化 11s 里大部分是这类零收益重布。
			const optLb = this.netLowerBound(net);
			if (optLb > 0 && savedLen <= optLb * 1.02 + 8) continue;

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
						r: v.dia / 2 + Math.max(this.drc.trackToTrackMil, this.drc.trackToPadMil) + this.drc.traceWidthMil / 2 + VIA_WALL_EXTRA,
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
		this.progress(this.routedPct(networks), `优化完成：已布通 ${this.routedOrder.length}/${networks.length}`);
	}

	/**
	 * J/H 引擎结果的后处理入口（多引擎架构）：外部引擎（内置自动布线等）
	 * 的走线读回后，登记障碍并逐网做拉紧（tauten）+ 圆润（smoothCorners）
	 * —— 让三个引擎的产物共享同一套形态优化管线。
	 *
	 * 输入的 prims 直接来自画布读回（pcb_PrimitiveLine/Arc.getAll 按 net
	 * 分组），vias 同理。处理在内存完成，调用方负责回写画布。
	 */
	public async postProcessImportedNets(
		entries: Array<{ name: string; prims: RenderedPrim[]; vias: PlannedVia[] }>,
		drc: DRCRules,
		allPads: EnginePad[],
		baseWorld?: RBObstacle[],
		onLog?: (message: string, level: string) => void,
		onNet?: (name: string, prims: RenderedPrim[], vias: PlannedVia[]) => void | Promise<void>,
		isCancelled?: () => boolean,
	): Promise<Array<{ name: string; prims: RenderedPrim[]; vias: PlannedVia[]; changed: boolean }>> {
		this.drc = drc;
		this.allPads = allPads;
		this.callbacks = { onLog, isCancelled };
		this.planOptions = { connectivityFirst: true, optimize: false };
		// 并排焊盘行信息：rowEscapeRoute（排内垂直出线预留）依赖。主线在
		// 排序期顺带构建，后处理独立补建——否则 0.5mm 排里的重解边全部
		// 无路（LED7 实测：走廊零障碍仍边无解 = 中心被邻盘净距圈吞没）
		this.rowInfo.clear();
		{
			const byNetPads = new Map<string, EnginePad[]>();
			for (const p of allPads) {
				const arr = byNetPads.get(p.net);
				if (arr) arr.push(p); else byNetPads.set(p.net, [p]);
			}
			this.prioritizeRowInteriorPads(
				[...byNetPads.values()].map(ps => ({ net: { name: ps[0].net, pads: ps }, tier: 1 })));
		}
		this.netRendered.clear();
		// 世界 = 全画布静态铜（其它网焊盘/走线/铺铜/板框，调用方收集）+ 导入铜皮。
		// 缺静态世界时重解路径会合法地压上排除网/邻网铜皮（case9 实测 SMD Pad
		// to Track 39 处的真凶）。
		// 坐标非法的障碍（画布多边形脏点 → undefined/NaN 圆心）直接剔除：
		// 去重键要做 toFixed，脏障碍会让整个拉紧流程崩掉（实测 case9）。
		const rawWorld = baseWorld ?? [];
		const world = rawWorld.filter(o => !!o && Number.isFinite(o.x) && Number.isFinite(o.y) && Number.isFinite(o.r));
		if (world.length !== rawWorld.length) {
			this.log(`静态世界剔除 ${rawWorld.length - world.length} 个坐标非法的障碍（画布几何脏点）`, 'warn');
		}
		this.dynamicObstacles = world;
		// 网格兜底预算池：主线 plan() 里按整轮预算切分，后处理独立成池
		//（gridFallbackEdge 见 0 直接放弃，重解的密集焊盘边就全靠它了）。
		// 拉紧菜单是用户显式等待的场景：池给足（J/H 画布实测 20s 池只够
		// ~10 条失败边，37 个边无解里大半是池耗尽假象）。
		this.gridBudgetMs = Number((globalThis as any).__RB_TIGHTEN_GRID_MS__) || 90000;
		this.plannedVias = [];
		this.routedEdges.clear();
		this.routedOrder = [];
		const out: Array<{ name: string; prims: RenderedPrim[]; vias: PlannedVia[]; changed: boolean }> = [];
		// 先登记全部铜皮为障碍（后处理的拉紧/圆润要看到其它网的铜皮），
		// 并按网记账——拉紧某网时把它的旧几何摘出世界（旧路不挡新路）。
		const obsByNet = new Map<string, RBObstacle[]>();
		for (const e of entries) {
			this.netRendered.set(e.name, e.prims);
			this.plannedVias.push(...e.vias);
			const mark = this.dynamicObstacles.length;
			if (e.prims.length) this.addRenderedToObstacles(mergePrims(e.prims));
			// 导入过孔也要入世界：后续网的重解若看不见异网过孔，走线会直接
			// 压上过孔铜环（拉紧菜单实测 Track to Via 违规的真凶之一）。
			for (const v of e.vias) this.dynamicObstacles.push(this.viaObstacleOf(v));
			obsByNet.set(e.name, this.dynamicObstacles.slice(mark));
		}
		// 从最短导线组开始拉直：短网纠缠少、先定形，长网在稳定世界上重解
		const order = [...entries].sort((a, b) => renderedLength(a.prims) - renderedLength(b.prims));
		let adoptedN = 0, tautenN = 0;
		const failHist = new Map<string, number>();
		for (const e of order) {
			if (this.checkCancelled()) break;
			const before = renderedLength(e.prims);
			let prims = e.prims;
			let vias = e.vias;
			try {
				// 1) 摘除本网旧几何与旧过孔
				const own = new Set(obsByNet.get(e.name) ?? []);
				this.dynamicObstacles = this.dynamicObstacles.filter(o => !own.has(o));
				this.plannedVias = this.plannedVias.filter(v => v.net !== e.name);
				// 2) 真拉紧：可视图逐边重解（跨层边走过孔换层链；
				//    更短且全量复验通过才采纳）
				let adopted: { prims: RenderedPrim[]; vias: PlannedVia[] } | null = null;
				try {
					adopted = await this.resolveImportedNet(e.name, e.vias);
				} catch { adopted = null; }
				if (adopted) {
					prims = adopted.prims;
					vias = adopted.vias;
					adoptedN++;
				} else {
					// 3) 退回隔点拉直（Manhattan 折线删得动的地方有限）
					this.tautenNet(e.name);
					prims = this.netRendered.get(e.name) ?? e.prims;
					tautenN++;
					failHist.set(this.resolveFail || '异常', (failHist.get(this.resolveFail || '异常') ?? 0) + 1);
				}
				// 4) 圆润（拐角切线圆弧）
				const smoothed = this.smoothCorners(mergePrims(prims), e.name);
				if (renderedLength(smoothed) <= renderedLength(prims) + 0.5) prims = smoothed;
				// 4.5) 终态守卫：段级门只校验候选段，合并/圆润之后必须整网
				// 复检——实测 +5V 重解后整网压上 55×53mil 的 GND 大焊盘
				//（EDA 报 SMD Pad to Track d=0），段级计数因形状/合并漏检。
				// 终态焊盘违规比原几何多 → 整个网回退原几何（宁可不拉紧）
				if (this.countPadViolations(prims, e.name) > this.countPadViolations(e.prims, e.name)) {
					prims = e.prims;
					vias = e.vias;
				}
			} catch { /* 后处理失败保留原几何（外部引擎产物已可用） */ }
			// 5) 登记最终形态为障碍（后续网的拉紧要看到本网的最终位置）
			this.netRendered.set(e.name, prims);
			if (prims.length) this.addRenderedToObstacles(mergePrims(prims));
			for (const v of vias) this.dynamicObstacles.push(this.viaObstacleOf(v));
			this.plannedVias.push(...vias);
			// 逐网回调：调用方实时上画布（拉紧过程动态可见）
			if (onNet) await onNet(e.name, prims, vias);
			out.push({ name: e.name, prims, vias, changed: renderedLength(prims) < before - 0.5 });
		}
		this.log(`后处理：真拉紧采纳 ${adoptedN} 网 / 隔点拉直 ${tautenN} 网（共 ${order.length}）`, 'info');
		this.log(`段级重解：尝试 ${this.runStats.tried} 段 / 采纳 ${this.runStats.adopted}（其中修复 ${this.runStats.repaired}）/ 近直线跳过 ${this.runStats.skipped} / 无解 ${this.runStats.noPath}`, 'info');
		if (failHist.size) {
			this.log(`重解未采纳：${[...failHist.entries()].map(([k, n]) => `${k}×${n}`).join('，')}`, 'info');
		}
		return out;
	}

	/**
	 * 导入网的可视图重解（真拉紧）：按 MST 逐边在「其它网铜皮」世界求
	 * 最短路。同层边中心直连（失败退避焊盘边界出线）；同层堵死且两端
	 * 都能底层接入时底层重试；跨层边（顶层 SMD ↔ 底层 SMD）走过孔
	 * 换层链 viaEdgeCandidate——原先带过孔的网整网跳过重解（拉紧菜单
	 * 「完全没有拉紧」的根源），现在逐边真实重布。更短（计过孔代价）
	 * 且 validatePrimsAgainst 全量复验通过才采纳，否则返回 null。
	 * 新落过孔即时入账 plannedVias（同网跨边 Hole to Hole 互查依赖），
	 * 未采纳在 finally 里整体回滚。
	 */
	/** 当前网重解未采纳的原因（resolveImportedNet 写入，诊断用） */
	private resolveFail = '';
	/** 段级重解计数（solveViaRuns 累计，诊断用）：近直线跳过/尝试/采纳/无解/修复 */
	private runStats = { skipped: 0, tried: 0, adopted: 0, noPath: 0, repaired: 0 };

	// ── 按段拉紧的公开接口（topological-router 逐段增量处理用） ──

	/** 初始化拉紧世界（全画布静态铜 + 焊盘形状上下文） */
	public initTightenWorld(
		drc: DRCRules, allPads: EnginePad[], baseWorld: RBObstacle[],
		padShapes?: Record<string, PadShape>,
	): void {
		this.drc = drc;
		this.allPads = allPads;
		this.padShapes = padShapes ?? {};
		this.callbacks = {};
		this.planOptions = { connectivityFirst: true, optimize: false };
		this.padObstacleCache.clear();
		this.padViolationGrid.clear();
		this.padExtentCache.clear();
		this.rowInfo.clear();
		const byNetPads = new Map<string, EnginePad[]>();
		for (const p of allPads) {
			const arr = byNetPads.get(p.net);
			if (arr) arr.push(p); else byNetPads.set(p.net, [p]);
		}
		this.prioritizeRowInteriorPads(
			[...byNetPads.values()].map(ps => ({ net: { name: ps[0].net, pads: ps }, tier: 1 })));
		this.dynamicObstacles = baseWorld.filter(o =>
			!!o && Number.isFinite(o.x) && Number.isFinite(o.y) && Number.isFinite(o.r));
	}

	/** 把一段铜皮登记进世界，返回其障碍组（撤销用） */
	public addRunToWorld(prims: RenderedPrim[]): string[] {
		const before = new Set<RBObstacle>(this.dynamicObstacles);
		this.addRenderedToObstacles(mergePrims(prims));
		const added = this.dynamicObstacles.filter(o => !before.has(o));
		return [...new Set(added.map(o => o.group ?? ''))].filter(g => !!g);
	}

	/** 登记过孔障碍 */
	public addViaToWorld(v: PlannedVia): void {
		this.dynamicObstacles.push(this.viaObstacleOf(v));
	}

	/** 撤销障碍组 */
	public removeWorldGroups(groups: string[]): void {
		if (!groups.length) return;
		const set = new Set(groups);
		this.dynamicObstacles = this.dynamicObstacles.filter(o => !set.has(o.group ?? ''));
	}

	/** 原走线按节点图切「连续导线段」（分支/过孔为边界） */
	public splitRunsForTighten(
		prims: RenderedPrim[], vias: PlannedVia[],
	): Array<{ prims: RenderedPrim[]; a: { x: number; y: number }; b: { x: number; y: number }; layer: number }> {
		const split = this.splitIntoRuns(prims, vias);
		return split.segments.map(s => ({ prims: s.prims, a: s.a, b: s.b, layer: s.layer }));
	}

	/** 单段求解：tier-1 焊盘为墙，tier-2 无墙（由调用方做焊盘零违规门） */
	public async solveRunSegment(		a: { x: number; y: number }, b: { x: number; y: number }, layer: number, netName: string,
	): Promise<RenderedPrim[] | null> {
		const obsAt = (ly: number) => this.dynamicObstacles.filter(o =>
			!(o.net && o.net === netName) && (o.layer === undefined || o.layer === ly));
		const run = async (obs: RBObstacle[]): Promise<RenderedPrim[] | null> => {
			// 走廊级别与预算：拉紧是「允许 10 分钟」的离线任务，300ms/60-150mil
			// 对真正绕路的段太紧（超时即无解 → 画布上表现为「拉紧没作用」）。
			const r = await routeConnection(a, b, obs, netName, [150, 500], 1200, () => false);
			if (!r.ok || !r.prims.length) return null;
			return r.prims.map(p => ({ ...(p as RenderedPrim), net: netName, layer }));
		};
		const padWalls = this.padObstacles(this.allPads, netName).filter(o =>
			o.layer === undefined || o.layer === layer);
		const t1 = await run([...padWalls, ...obsAt(layer)]);
		if (t1) return t1;
		return await run(obsAt(layer));
	}

	/**
	 * 整网拆线重布（拉紧菜单用）：按 MST 取焊盘对，逐边可视图重解。
	 *
	 * 逐「连续导线段」拉直只能删段内顶点——只要网里有一个 T 型分支，绕路
	 * 就横跨多个 run，单段求解永远够不到（用户实测：两个焊盘的网被布成
	 * 大兜圈，拉紧后只在钝角处加了圆弧）。整网按「两两最近焊盘」重布才能
	 * 把这个兜圈换成直连。
	 *
	 * 只处理「整网都在同一层、且所有焊盘都能落在该层」的网；有跨层焊盘
	 * 或已有过孔的网由调用方跳过（换层拓扑不在本函数职责内）。
	 */
	public async rerouteNetMST(netName: string, layer: number): Promise<RenderedPrim[] | null> {
		const all = this.allPads.filter(p => p.net === netName);
		if (all.length < 2) return null;
		const usable = all.filter(p => p.layer === 12 || p.padType === 'th' || p.layer === layer);
		if (usable.length !== all.length) return null;
		const edges = this.computeMST(usable);
		const out: RenderedPrim[] = [];
		for (const [i, j] of edges) {
			const seg = await this.solveRunSegment(usable[i], usable[j], layer, netName);
			if (!seg) return null;
			const merged = mergePrims(seg);
			const sm = this.smoothCorners(merged, netName);
			out.push(...(sm.length ? sm : merged));
		}
		return out.length ? out : null;
	}

	/** 焊盘违规计数（公开包装） */
	public padViolations(prims: RenderedPrim[], netName: string): number {
		return this.countPadViolations(prims, netName);
	}

	/** 外网铜皮校验（公开包装） */
	public async primsClearOfOthers(prims: RenderedPrim[], netName: string): Promise<boolean> {
		const layers = new Set(prims.map(p => p.layer ?? 1));
		// 焊盘也纳入——之前只查 dynamicObstacles（走线/过孔）不含焊盘，A\* 重解
		// 候选压到焊盘会过闸、再产出真 DRC 错误（用户实测）。
		const padObs = this.padObstacles(this.allPads, netName);
		for (const ly of layers) {
			const pr = prims.filter(p => (p.layer ?? 1) === ly);
			if (!pr.length) continue;
			const obs = [
				...this.dynamicObstacles.filter(o =>
					!(o.net && o.net === netName) && (o.layer === undefined || o.layer === ly)),
				...padObs.filter(o => o.layer === undefined || o.layer === ly),
			];
			if (!await validatePrimsAgainst(pr as unknown as RBPrimitive[], obs, netName)) return false;
		}
		return true;
	}

	/** 拐角圆弧圆润（公开包装） */
	public smoothRun(prims: RenderedPrim[], netName: string): RenderedPrim[] {
		return this.smoothCorners(mergePrims(prims), netName);
	}

	/**
	 * 段内橡皮筋拉紧（string-pulling）：同层连续直线链上，若「隔点直连」不侵入
	 * 任何障碍，就删掉中间顶点——45° 楼梯、兜底小折弯被拉成最紧折线。只删顶点、
	 * 不动端点与受保护顶点（焊盘/过孔/分支点），逐段净距复核，不产生 DRC 违规。
	 * 这是「两两导线段之间求最短距离」的几何实现；与 tautenNet 同构，只是
	 * 作用在任意一段 prims 上并返回结果（拉紧菜单逐段调用）。
	 */
	public tautenRun(prims: RenderedPrim[], netName: string): RenderedPrim[] {
		if (prims.length < 3) return prims;
		const obsCache = new Map<number, RBObstacle[]>();
		const obsFor = (layer: number): RBObstacle[] => {
			let o = obsCache.get(layer);
			if (!o) {
				o = this.layerObstacles(netName, layer, true);
				obsCache.set(layer, o);
			}
			return o;
		};
		const segClear = (x1: number, y1: number, x2: number, y2: number, layer: number): boolean => {
			const minx = Math.min(x1, x2), maxx = Math.max(x1, x2);
			const miny = Math.min(y1, y2), maxy = Math.max(y1, y2);
			for (const o of obsFor(layer)) {
				if (o.soft) continue;
				if (o.net && o.net === netName) continue;
				if (o.layer !== undefined && o.layer !== layer) continue;
				const rr = o.r + 0.2;
				if (o.x + rr < minx || o.x - rr > maxx || o.y + rr < miny || o.y - rr > maxy) continue;
				if (o.kind === 'circle') {
					if (pointSegDist(o.x, o.y, x1, y1, x2, y2) < rr) return false;
				} else {
					const ox1 = o.x1 ?? o.x, oy1 = o.y1 ?? o.y;
					const ox2 = o.x2 ?? ox1, oy2 = o.y2 ?? oy1;
					const steps = Math.min(64, Math.max(1, Math.ceil(Math.hypot(ox2 - ox1, oy2 - oy1) / Math.max(0.9 * o.r, 0.5))));
					for (let s = 0; s <= steps; s++) {
						const t = s / steps;
						if (pointSegDist(ox1 + (ox2 - ox1) * t, oy1 + (oy2 - oy1) * t, x1, y1, x2, y2) < rr) return false;
					}
				}
			}
			return true;
		};
		const merged = mergePrims(prims);
		// 受保护顶点：焊盘中心 / 过孔中心 / 分支点（≥3 段共享端点）
		const prot = new Set<string>();
		const kOf = (x: number, y: number) => `${Math.round(x * 2)}|${Math.round(y * 2)}`;
		for (const v of this.plannedVias) if (v.net === netName) prot.add(kOf(v.x, v.y));
		for (const pd of this.allPads) if (pd.net === netName) prot.add(kOf(pd.x, pd.y));
		const endCount = new Map<string, number>();
		for (const p of merged) {
			if (p.kind !== 'line') continue;
			const a = kOf(p.x1!, p.y1!), b = kOf(p.x2!, p.y2!);
			endCount.set(a, (endCount.get(a) ?? 0) + 1);
			endCount.set(b, (endCount.get(b) ?? 0) + 1);
		}
		for (const [k, c] of endCount) if (c >= 3) prot.add(k);
		const out: RenderedPrim[] = [];
		let i = 0;
		let changed = false;
		while (i < merged.length) {
			const cur = merged[i];
			if (cur.kind !== 'line') { out.push(cur); i++; continue; }
			const chain: RenderedPrim[] = [cur];
			let j = i + 1;
			while (j < merged.length) {
				const nxt = merged[j];
				if (nxt.kind !== 'line' || nxt.layer !== cur.layer) break;
				const tail = chain[chain.length - 1];
				if (Math.hypot((tail.x2 ?? 0) - (nxt.x1 ?? 0), (tail.y2 ?? 0) - (nxt.y1 ?? 0)) > 0.6) break;
				chain.push(nxt);
				j++;
			}
			if (chain.length >= 2) {
				const pts: Array<{ x: number; y: number }> = [{ x: chain[0].x1!, y: chain[0].y1! }];
				for (const c of chain) pts.push({ x: c.x2!, y: c.y2! });
				for (let pass = 0; pass < 4; pass++) {
					let removed = false;
					for (let k = 1; k < pts.length - 1; k++) {
						if (prot.has(kOf(pts[k].x, pts[k].y))) continue;
						if (segClear(pts[k - 1].x, pts[k - 1].y, pts[k + 1].x, pts[k + 1].y, cur.layer)) {
							pts.splice(k, 1);
							removed = true;
							changed = true;
							k--;
						}
					}
					if (!removed) break;
				}
				const dedup: Array<{ x: number; y: number }> = [pts[0]];
				for (const q of pts) {
					const l = dedup[dedup.length - 1];
					if (Math.hypot(q.x - l.x, q.y - l.y) > 0.05) dedup.push(q);
				}
				for (let k = 1; k < dedup.length; k++) {
					out.push({ kind: 'line', net: netName, layer: cur.layer, x1: dedup[k - 1].x, y1: dedup[k - 1].y, x2: dedup[k].x, y2: dedup[k].y });
				}
			} else {
				out.push(cur);
			}
			i = j;
		}
		if (!changed) return prims;
		if (renderedLength(out) >= renderedLength(merged) - 0.5) return prims;
		return out;
	}

	/**
	 * 带过孔网的真拉紧（逐段重解）：原走线链按过孔断点切成同层段，
	 * 过孔位置保持不动（不找新槽位——EDA 的过孔本就合法，且槽位搜索
	 * 在密排区必然失败），逐段在「其它网铜皮 + 本网其它段」世界里重解，
	 * 更短且复验通过才采纳，任一段失败保留原段——不整网回退。
	 * 返回 null = 链断裂/分支（调用方落到 MST 全网重解路径）。
	 */
	private async solveViaRuns(
		name: string, orig: RenderedPrim[], origVias: PlannedVia[],
	): Promise<{ prims: RenderedPrim[]; vias: PlannedVia[] } | null> {
		const split = this.splitIntoRuns(orig, origVias);
		if (!split) return null;
		const { segments, leftover } = split;
		if (!segments.length) return null;
		// 世界 = 其它网铜皮（dynamicObstacles，硬校验）。焊盘两级口径：
		// tier-1 带焊盘墙求完全合规路径（首选）；tier-2 焊盘不入世界 +
		// 「违规数不恶化」门（EDA 画布有存量焊盘违规，硬墙会让违规区
		// 整体不可重解；无墙最短路又会贴焊盘——由计数门兜住下限）
		const obsAt = (ly: number) => this.dynamicObstacles.filter(o =>
			!(o.net && o.net === name) && (o.layer === undefined || o.layer === ly));
		const padWalls = this.padObstacles(this.allPads, name);
		const obsWithPads = (ly: number) => [
			...padWalls, ...this.dynamicObstacles,
		].filter(o => !(o.net && o.net === name) && (o.layer === undefined || o.layer === ly));
		const solveSegment = async (a: { x: number; y: number }, b: { x: number; y: number }, ly: number): Promise<RenderedPrim[] | null> => {
			const run = async (obs: RBObstacle[]): Promise<RenderedPrim[] | null> => {
				const r = await routeConnection(a, b, obs, name,
					[150, 500], 3000, () => this.checkCancelled());
				if (!r.ok || !r.prims.length) return null;
				return r.prims.map(p => ({ ...(p as RenderedPrim), net: name, layer: ly }));
			};
			// tier-1：焊盘是墙（合法区首选，产物零新增焊盘违规）
			const t1 = await run(obsWithPads(ly));
			if (t1) return t1;
			// tier-2：焊盘不入世界，交给不恶化计数门
			return await run(obsAt(ly));
		};
		const temp: RBObstacle[] = [];
		const addTemp = (prims: RenderedPrim[]): string[] => {
			const before = new Set<RBObstacle>(this.dynamicObstacles);
			this.addRenderedToObstacles(prims);
			const added = this.dynamicObstacles.filter(o => !before.has(o));
			temp.push(...added);
			return [...new Set(added.map(o => o.group ?? ''))].filter(g => !!g);
		};
		const removeGroups = (groups: string[]): void => {
			const set = new Set(groups);
			this.dynamicObstacles = this.dynamicObstacles.filter(o => !set.has(o.group ?? ''));
		};
		try {
			// 原始段全部临时入世界：段间互见（重解段不得压本网其它段/分支残段）
			if (leftover.length) addTemp(leftover);
			for (const seg of segments) seg.groups = addTemp(seg.prims);
			const t0 = Date.now();
			for (const seg of segments) {
				if (this.checkCancelled()) break;
				const straight = Math.hypot(seg.b.x - seg.a.x, seg.b.y - seg.a.y);
				const oldLen = renderedLength(seg.prims);
				// 可回收余量不足 25mil（≈0.6mm，视觉可忽略）/ 单网预算耗尽：
				// 跳过重解。⚠ 用绝对余量而不是比例门限——长段上 160mil 的
				// 大绕路只占 12%，比例门限会把它跳过（LCD5 注入绕路实测）
				if (oldLen - straight < 25 || Date.now() - t0 > 10000) { this.runStats.skipped++; continue; }
				this.runStats.tried++;
				const origSegPrims = seg.prims;
				removeGroups(seg.groups);
				const cand: RenderedPrim[] | null = await solveSegment(seg.a, seg.b, seg.layer);
				const __diag = !!(globalThis as any).__RB_TIGHTEN_DIAG__;
				// tier-2（无焊盘墙）的采纳必须「新路径焊盘违规为 0」：计数指标
				// 可能漏检（采样/形状近似），要求 0 才能保证 EDA DRC 不新增。
				// 修复例外：原段已有焊盘违规、候选为 0 → 即使更长也采纳（只修不恶化）。
				const candPad = cand ? this.countPadViolations(cand, name) : 0;
				const origPad = this.countPadViolations(origSegPrims, name);
				const okShorter = !!cand && renderedLength(cand) < oldLen - 0.5 && candPad === 0;
				const okRepair = !!cand && candPad === 0 && origPad > 0;
				if (cand && (okShorter || okRepair) &&
					await validatePrimsAgainst(cand as unknown as RBPrimitive[], obsAt(seg.layer), name)) {
					seg.prims = cand;
					this.runStats.adopted++;
					if (okRepair) this.runStats.repaired = (this.runStats.repaired ?? 0) + 1;
				} else {
					this.runStats.noPath++;
					if (__diag) {
						this.log(`[段诊断] ${name} (${seg.a.x.toFixed(0)},${seg.a.y.toFixed(0)})→(${seg.b.x.toFixed(0)},${seg.b.y.toFixed(0)}) L${seg.layer} 原长${oldLen.toFixed(0)} 直距${straight.toFixed(0)} 新长${cand ? renderedLength(cand).toFixed(0) : '无路'}${cand && !okShorter && !okRepair ? `(不采纳 焊盘${candPad}/原${origPad})` : ''}`, 'info');
					}
				}
				// 本段当前形态（原始或新）回到世界
				seg.groups = addTemp(seg.prims);
			}
			return {
				prims: [...segments.flatMap(s => s.prims), ...leftover],
				vias: origVias.map(v => ({ ...v })),
			};
		} finally {
			// 临时障碍全部摘除：调用方以最终形态重新登记（step 5）
			const tset = new Set<RBObstacle>(temp);
			this.dynamicObstacles = this.dynamicObstacles.filter(o => !tset.has(o));
		}
	}

	/**
	 * 原走线按「节点图」切同层段：节点 = 图元端点聚类（0.8mil）；度≠2 的
	 * 节点与过孔位置为段边界。段 = 两个边界节点之间的图元链。EDA 产物
	 * 常有分支（T 型接续）——分支臂各自成段，独立重解；环（全度 2）进
	 * leftover 原样保留。过孔处强制切段（过孔位置保持不动）。
	 */
	private splitIntoRuns(
		prims: RenderedPrim[], vias: PlannedVia[],
	): { segments: Array<{ prims: RenderedPrim[]; a: { x: number; y: number }; b: { x: number; y: number }; layer: number; groups?: string[] }>; leftover: RenderedPrim[] } {
		const startOf = (p: RenderedPrim): { x: number; y: number } => p.kind === 'line'
			? { x: p.x1 ?? 0, y: p.y1 ?? 0 }
			: { x: (p.cx ?? 0) + (p.r ?? 0) * Math.cos(p.startAngle ?? 0), y: (p.cy ?? 0) + (p.r ?? 0) * Math.sin(p.startAngle ?? 0) };
		const endOf = (p: RenderedPrim): { x: number; y: number } => p.kind === 'line'
			? { x: p.x2 ?? 0, y: p.y2 ?? 0 }
			: { x: (p.cx ?? 0) + (p.r ?? 0) * Math.cos((p.startAngle ?? 0) + (p.sweep ?? 0)), y: (p.cy ?? 0) + (p.r ?? 0) * Math.sin((p.startAngle ?? 0) + (p.sweep ?? 0)) };
		// 端点聚类（0.8mil 内同节点）
		const eps: Array<{ x: number; y: number }> = [];
		const findNode = (p: { x: number; y: number }): number => {
			for (let i = 0; i < eps.length; i++) {
				if (Math.hypot(eps[i].x - p.x, eps[i].y - p.y) < 0.8) return i;
			}
			eps.push({ x: p.x, y: p.y });
			return eps.length - 1;
		};
		const ends: Array<[number, number]> = prims.map(p =>
			[findNode(startOf(p)), findNode(endOf(p))] as [number, number]);
		const deg = new Map<number, number>();
		for (const [a, b] of ends) {
			deg.set(a, (deg.get(a) ?? 0) + 1);
			deg.set(b, (deg.get(b) ?? 0) + 1);
		}
		const viaNodes = new Set<number>();
		for (const v of vias) {
			let best = -1, bd = 2;
			for (let i = 0; i < eps.length; i++) {
				const d = Math.hypot(eps[i].x - v.x, eps[i].y - v.y);
				if (d < bd) { bd = d; best = i; }
			}
			if (best >= 0) viaNodes.add(best);
		}
		const isBoundary = (n: number): boolean => (deg.get(n) ?? 0) !== 2 || viaNodes.has(n);
		const usedPrim = new Set<number>();
		// 从 prim i 的 node 端向外延伸（每步经度-2 节点接下一根图元），
		// 返回 [沿途图元, 终止节点]
		const walkOut = (i: number, node: number): { prims: RenderedPrim[]; end: number } => {
			const seq: RenderedPrim[] = [];
			let cur = node;
			let guard = prims.length + 4;
			while (!isBoundary(cur) && guard-- > 0) {
				let found = -1, next = -1;
				for (let j = 0; j < prims.length; j++) {
					if (usedPrim.has(j)) continue;
					const [ja, jb] = ends[j];
					if (ja === cur) { found = j; next = jb; break; }
					if (jb === cur) { found = j; next = ja; break; }
				}
				if (found < 0) break;
				usedPrim.add(found);
				seq.push(prims[found]);
				cur = next;
			}
			return { prims: seq, end: cur };
		};
		const segments: Array<{ prims: RenderedPrim[]; a: { x: number; y: number }; b: { x: number; y: number }; layer: number; groups?: string[] }> = [];
		const leftover: RenderedPrim[] = [];
		for (let i = 0; i < prims.length; i++) {
			if (usedPrim.has(i)) continue;
			usedPrim.add(i);
			const [pa, pb] = ends[i];
			const left = walkOut(i, pa);
			const right = walkOut(i, pb);
			const seq = [...left.prims, prims[i], ...right.prims];
			if (left.end === right.end) {
				// 独立环：也纳入拉紧（a≈b；worker 对 a≈b 跳过重解，只做拉直+圆润）。
				// 之前整环丢进 leftover 原样保留——GND 这类带环的网 52/84 段没动。
				const ln = eps[left.end];
				const loopLayer = seq[0].layer ?? 1;
				segments.push({ prims: orderRunChain(seq, ln.x, ln.y), a: { x: ln.x, y: ln.y }, b: { x: ln.x, y: ln.y }, layer: loopLayer });
				continue;
			}
			const na = eps[left.end], nb = eps[right.end];
			const layer = seq[0].layer;
			const ordered = orderRunChain(seq, na.x, na.y);
			if (ordered.some(q => q.layer !== ordered[0].layer)) {
				// 混层链（经 TH 焊盘换层、无 via 图元）：按层切段分别处理。
				// 之前整条丢 leftover —— P10/NET_45/P1 这类 100% 不拉紧。
				let curLayer = ordered[0].layer ?? 1;
				let grp: RenderedPrim[] = [];
				const flush = (): void => {
					if (!grp.length) return;
					const s = grp[0], e0 = grp[grp.length - 1];
					const sPt = s.kind === 'line'
						? { x: s.x1!, y: s.y1! }
						: { x: s.cx! + s.r! * Math.cos(s.startAngle!), y: s.cy! + s.r! * Math.sin(s.startAngle!) };
					const ePt = e0.kind === 'line'
						? { x: e0.x2!, y: e0.y2! }
						: { x: e0.cx! + e0.r! * Math.cos(e0.startAngle! + e0.sweep!), y: e0.cy! + e0.r! * Math.sin(e0.startAngle! + e0.sweep!) };
					segments.push({ prims: grp, a: sPt, b: ePt, layer: curLayer });
					grp = [];
				};
				for (const q of ordered) {
					if ((q.layer ?? 1) !== curLayer) { flush(); curLayer = q.layer ?? 1; }
					grp.push(q);
				}
				flush();
				continue;
			}
			segments.push({
				prims: ordered,
				a: { x: na.x, y: na.y },
				b: { x: nb.x, y: nb.y },
				layer: layer ?? 1,
			});
		}
		return { segments, leftover };
	}

	/**
	 * 图元采样点对异网焊盘（含钻孔）的净距违规计数（1.5mil 采样——4mil
	 * 会漏掉细条重叠，实测 +5V 重写后 EDA 报 7 处 d=0 而计数通过）。
	 * 「不恶化」口径的精确度前提；tier-2 采纳要求新路径计数为 0。
	 */
	private countPadViolations(prims: RenderedPrim[], netName: string): number {
		if (!prims.length) return 0;
		const samples: Array<{ x: number; y: number; layer: number }> = [];
		for (const p of prims) {
			if (p.kind === 'line') {
				const L = Math.hypot((p.x2 ?? 0) - (p.x1 ?? 0), (p.y2 ?? 0) - (p.y1 ?? 0));
				const n = Math.max(1, Math.ceil(L / 1.5));
				for (let k = 0; k <= n; k++) {
					const t = k / n;
					samples.push({
						x: (p.x1 ?? 0) + ((p.x2 ?? 0) - (p.x1 ?? 0)) * t,
						y: (p.y1 ?? 0) + ((p.y2 ?? 0) - (p.y1 ?? 0)) * t,
						layer: p.layer ?? 1,
					});
				}
			} else {
				const n = Math.max(2, Math.ceil(Math.abs(p.sweep ?? 0) / 0.3));
				for (let k = 0; k <= n; k++) {
					const a = (p.startAngle ?? 0) + ((p.sweep ?? 0) * k) / n;
					samples.push({
						x: (p.cx ?? 0) + (p.r ?? 0) * Math.cos(a),
						y: (p.cy ?? 0) + (p.r ?? 0) * Math.sin(a),
						layer: p.layer ?? 1,
					});
				}
			}
		}
		const halfW = this.drc.traceWidthMil / 2;
		const needCu = this.drc.trackToPadMil + halfW;
		// 焊盘空间网格：原实现每个采样点都要遍历全部焊盘做形状距离（264 个焊盘 ×
		// 上千采样点，实测单次 ~100ms，整板 38s）。cell=256 + 3×3 邻域足够覆盖
		// 最大焊盘（187mil）的角距影响（~142mil）。网格按排除网络名缓存。
		const CELL = 256;
		let grid = this.padViolationGrid.get(netName);
		if (!grid) {
			grid = new Map<string, EnginePad[]>();
			for (const pd of this.allPads) {
				if (pd.net === netName) continue;
				const k = `${Math.floor(pd.x / CELL)}|${Math.floor(pd.y / CELL)}`;
				const arr = grid.get(k);
				if (arr) arr.push(pd); else grid.set(k, [pd]);
			}
			this.padViolationGrid.set(netName, grid);
		}
		let bad = 0;
		for (const s of samples) {
			const gx = Math.floor(s.x / CELL), gy = Math.floor(s.y / CELL);
			let hit = false;
			for (let dx = -1; dx <= 1 && !hit; dx++) {
				for (let dy = -1; dy <= 1 && !hit; dy++) {
					const arr = grid.get(`${gx + dx}|${gy + dy}`);
					if (!arr) continue;
					for (const pd of arr) {
						const conductive = pd.layer === 12 || pd.padType === 'th' || pd.layer === s.layer;
						if (!conductive) continue;
						if (this.padCopperDistance(pd, s.x, s.y) < needCu) { hit = true; break; }
						if (pd.layer === 12 || pd.padType === 'th') {
							const holeD = Math.max(pd.holeDiameter, 0);
							if (holeD > 0 && Math.hypot(pd.x - s.x, pd.y - s.y) < holeD / 2 + needCu) { hit = true; break; }
						}
					}
				}
			}
			if (hit) bad++;
		}
		return bad;
	}

	private async resolveImportedNet(
		name: string, origVias: PlannedVia[],
	): Promise<{ prims: RenderedPrim[]; vias: PlannedVia[] } | null> {
		this.resolveFail = '';
		const fail = (why: string): null => { this.resolveFail = why; return null; };
		const orig0 = this.netRendered.get(name);
		// 带过孔网优先逐段重解（过孔不动、段级采纳/保留）；链断裂才落
		// MST 全网重解。段级复验已做，这里只看整网是否缩短。
		if (orig0 && orig0.length && origVias.length) {
			const runs = await this.solveViaRuns(name, orig0, origVias);
			if (runs) {
				if (renderedLength(runs.prims) < renderedLength(orig0) - 0.5) return runs;
				return fail(`不够短(${(renderedLength(orig0) - renderedLength(runs.prims)).toFixed(0)}mil,孔不变)`);
			}
			// null = 断链/分支 → MST 路径
		}
		const allP = this.allPads.filter(p => p.net === name);
		// 同坐标重复 pad（getAllPrimitivesByNet 枚举重复，case9 P02/P22/$1N50156
		// 实测）：MST 出零长度边，任一边无解即整网 bail 的口径下直接冤死 ——
		// 按 0.5mil 量化去重
		const pads: EnginePad[] = [];
		for (const p of allP) {
			if (!pads.some(q => Math.abs(q.x - p.x) < 0.5 && Math.abs(q.y - p.y) < 0.5)) pads.push(p);
		}
		if (pads.length < 2) return fail('单焊盘');
		const edges = this.computeMST(pads);
		// 该层世界：其它网铜皮（dynamicObstacles）。routeConnection 不自带同网
		// 过滤——这里必须先剔除本网障碍（自家焊盘/旧迹），否则端点落在自家
		// 焊盘中心直接判侵入。焊盘走「违规数不恶化」口径（见 countPadViolations）
		const obsAt = (ly: number) => this.dynamicObstacles.filter(o =>
			!(o.net && o.net === name) && (o.layer === undefined || o.layer === ly));
		const routeEdgeAt = async (a: EnginePad, b: EnginePad, ly: number): Promise<RenderedPrim[] | null> => {
			const obs = obsAt(ly);
			const obsPad = [...this.padObstacles(this.allPads, name).filter(o =>
				o.layer === undefined || o.layer === ly), ...obs];
			const run = async (o: RBObstacle[]): Promise<RenderedPrim[] | null> => {
				const r = await routeConnection(
					{ x: a.x, y: a.y }, { x: b.x, y: b.y }, o, name,
					[150, 500], 3000, () => this.checkCancelled(),
				);
				if (r.ok && r.prims.length) {
					return r.prims.map(p => ({ ...(p as RenderedPrim), net: name, layer: ly }));
				}
				return null;
			};
			// tier-1：焊盘是墙（合法区首选）；tier-2：无焊盘墙（不恶化门兜底）
			const t1 = await run(obsPad);
			if (t1) return t1;
			let prims = await run(obs);
			// 密集焊盘排里邻盘净距圈吞没焊盘中心：从焊盘轮廓边界出线退避
			if (!prims) prims = await this.candidateEdgeRoute(a, b, name, ly, obs);
			// 并排焊盘排（连接器/LED 排，0.5mm 间距）：排中焊盘唯一合法出线是
			// 排法向冲出（主线同款预留；rowInfo 在本入口开头补建）
			const srcRow = this.rowInfo.get(a.id);
			if (!prims && srcRow && srcRow.depth > 0.4) {
				prims = await this.rowEscapeRoute(a, b, srcRow, name, ly);
			}
			const dstRow = this.rowInfo.get(b.id);
			if (!prims && dstRow && dstRow.depth > 0.4) {
				prims = await this.rowEscapeRoute(b, a, dstRow, name, ly, true);
			}
			return prims;
		};
		const solveEdge = async (a: EnginePad, b: EnginePad): Promise<{ prims: RenderedPrim[]; vias: PlannedVia[] } | null> => {
			const layer = getRoutingLayer(a, b);
			const __diag = !!(globalThis as any).__RB_TIGHTEN_DIAG__;
			const stages: string[] = [];
			// 跨层对：一端只能顶层接入、另一端只能底层接入（getRoutingLayer
			// 恒返 1），任何同层直连在几何上都不存在。含通孔的对不在此列
			//——通孔可从顶层接入，先试同层，过孔链只作退避
			const crossLayer = bottomCapable(a) !== bottomCapable(b) && layer === 1 &&
				a.layer !== 12 && b.layer !== 12;
			let prims: RenderedPrim[] | null = null;
			let vias: PlannedVia[] = [];
			if (!crossLayer) {
				prims = await routeEdgeAt(a, b, layer);
				stages.push(`vis${layer}:${prims ? 'OK' : 'x'}`);
				// 同层堵死且两端都能底层接入 → 底层重试（TH/底层盘无需过孔）
				if (!prims && layer === 1 && this.hasBottomLayer &&
					bottomCapable(a) && bottomCapable(b)) {
					prims = await routeEdgeAt(a, b, 2);
					stages.push(`vis2:${prims ? 'OK' : 'x'}`);
				}
			} else {
				stages.push('cross');
			}
			// 可视图全败 → 网格兜底（焊盘内部多点起步，主管线同款；自带双层与过孔）
			if (!prims) {
				let g: { prims: RenderedPrim[]; vias: PlannedVia[] } | null | string = null;
				try { g = await this.gridFallbackEdge(a, b, name); } catch (e) { g = 'THROW:' + String((e as Error)?.message ?? e).slice(0, 90); }
				if (g && typeof g !== 'string' && g.prims.length) { prims = g.prims; vias = g.vias; }
				stages.push(`grid:${typeof g === 'string' ? g : g ? (g.prims.length ? 'OK' : 'x') : 'null'}`);
			}
			// 仍无路 → 过孔换层链。viaEdgeCandidate 用完整世界（静态+焊盘+
			// 其它网铜皮）并自带落点净距校验；拉紧场景世界固定，绝不撕其它网。
			if (!prims && this.hasBottomLayer) {
				let vc: { prims: RenderedPrim[]; vias: PlannedVia[]; length: number } | null | string = null;
				try { vc = await this.viaEdgeCandidate(a, b, name); } catch (e) { vc = 'THROW:' + String(e).slice(0, 60); }
				if (vc && typeof vc !== 'string') { prims = vc.prims; vias = vc.vias; }
				stages.push(`via:${typeof vc === 'string' ? vc : vc ? 'OK' : 'x'}`);
			}
			if (!prims && __diag) {
				// 诊断：走廊两侧 25mil 内的异网障碍按类别计数 + 端点邻域障碍数
				const near = obsAt(layer).filter(o => !o.soft &&
					pointSegDist(o.x, o.y, a.x, a.y, b.x, b.y) < o.r + 25);
				const byKind = new Map<string, number>();
				for (const o of near) {
					const k = o.group?.startsWith('pad:') ? 'pad'
						: o.group?.startsWith('via:') ? 'via'
						: o.group?.startsWith('fill') ? 'fill'
						: o.group === 'board' ? 'board' : 'trk';
					byKind.set(k, (byKind.get(k) ?? 0) + 1);
				}
				const epNear = (px: number, py: number) =>
					obsAt(layer).filter(o => !o.soft && Math.hypot(o.x - px, o.y - py) < o.r + 20).length;
				this.log(`[拉紧诊断] ${name} (${a.x.toFixed(0)},${a.y.toFixed(0)})L${a.layer}→(${b.x.toFixed(0)},${b.y.toFixed(0)})L${b.layer} L${layer} ${stages.join(' ')} 端点邻域 ${epNear(a.x, a.y)}/${epNear(b.x, b.y)} 走廊: ${[...byKind.entries()].map(([k, n]) => `${k}=${n}`).join(' ') || '无'}`, 'info');
			}
			return prims ? { prims, vias } : null;
		};
		const out: RenderedPrim[] = [];
		const outVias: PlannedVia[] = [];
		const savedVias = this.plannedVias;
		this.plannedVias = savedVias.filter(v => v.net !== name);
		try {
			for (const [ia, ib] of edges) {
				if (this.checkCancelled()) return fail('取消');
				const pa = pads[ia], pb = pads[ib];
				if (Math.hypot(pa.x - pb.x, pa.y - pb.y) < 1) continue;   // 重合 pad 退化边
				const solved = await solveEdge(pa, pb);
				if (!solved) return fail('边无解');   // 任一边解不出 → 整网回退隔点拉直
				const prims = solved.prims;
				const vias = solved.vias;
				if (prims.some(p => p.layer === undefined)) return fail('无层图元');   // 无层图元防御
				out.push(...prims);
				outVias.push(...vias);
				this.plannedVias.push(...vias);   // 跨边互见（同网 hole-to-hole）
			}
			const orig = this.netRendered.get(name);
			if (!orig || !orig.length) return fail('无原图元');
			// 采纳门槛计入过孔代价：铜皮缩短必须覆盖过孔数增量（+1 孔 ≈ 150mil），
			// 既不白加过孔，也允许「多绕铜皮换掉无谓扇出孔」的解
			const viaCost = Math.max(150, this.drc.traceWidthMil * 22);
			if (renderedLength(out) + outVias.length * viaCost >=
				renderedLength(orig) + origVias.length * viaCost - 0.5) {
				return fail(`不够短(${(renderedLength(orig) - renderedLength(out)).toFixed(0)}mil,孔${outVias.length}/${origVias.length})`);
			}
			// 全量复验：走线逐层 + 过孔净距（两层 + 精确口径）。防「重解路径
			// 合法地压上异网铜皮」——采纳门槛与主管线一致。焊盘走「违规数
			// 不恶化」口径（存量违规区允许不恶化的重解）
			for (const ly of this.hasBottomLayer ? [1, 2] : [1]) {
				const pr = out.filter(p => p.layer === ly);
				if (!pr.length) continue;
				if (!await validatePrimsAgainst(pr as unknown as RBPrimitive[], obsAt(ly), name)) return fail(`复验拒L${ly}`);
			}
			// 焊盘口径与逐段路径一致：新网焊盘违规必须为 0（计数指标可能
			// 漏检，只有 0 才能保证 EDA DRC 不新增；干净区的重解本就该零违规）
			if (this.countPadViolations(out, name) > 0) {
				return fail(`焊盘违规${this.countPadViolations(out, name)}`);
			}
			for (const v of outVias) {
				if (!this.viaObstacleClear(v.x, v.y, obsAt(1), v.dia / 2, name) ||
					!this.viaObstacleClear(v.x, v.y, obsAt(2), v.dia / 2, name)) return fail('过孔净距');
			}
			return { prims: out, vias: outVias };
		} finally {
			this.plannedVias = savedVias;
		}
	}
}
