/** 网络排序策略 */
export type NetOrder = 'auto' | 'river' | 'reverse' | 'max-x' | 'longest' | 'shortest';

/**
 * 布线参数（用户可在参数弹窗中调整）
 *
 * 这些字段全部直接参与布线算法，不做“摆设参数”。
 */
export interface RouterUiOptions {
	/** 走线宽度（mil） */
	traceWidthMil: number;
	/** 在 DRC 规则基础上额外增加的安全间距（mil） */
	extraClearanceMil: number;
	/** 出线额外余量（mil）：在焊盘边界之外再外扩的量，0 = 严格按最小距离出线 */
	escapeClearanceMil: number;
	/** 撕毁重布轮数（0 = 关闭） */
	ripupRounds: number;
	/** 兼容旧参数：仅对显式几何顺序生效，auto 策略自行决定电源次序 */
	powerNetsFirst: boolean;
	/** 整轮规划时间预算（毫秒，0 = 不限制） */
	budgetMs: number;
	/** 实时预览：每布完一个网络立即写入画布 */
	livePreview: boolean;
	/** 最后阶段做凸包拉紧/端帽/圆角优化（失败保留原走线） */
	finalOptimize: boolean;
	/** 失败网络允许过孔换层（底层兜底） */
	viaFallback: boolean;
	/** 允许过孔落在焊盘上（默认关闭：过孔铜环必须避开所有焊盘铜皮） */
	allowViaInPad: boolean;
	/** 允许布线的铜层 ID（1 = 顶层，2 = 底层；顶层始终可用） */
	routeLayers: number[];
	/** 布线前清除待布网络的既有走线（重新布线语义；默认开启） */
	clearExistingTraces: boolean;
	/** 不参与布线的网络名：保持画布现状，不做任何删改 */
	excludeNets: string[];
	/** 网络排序策略（auto = 按 PCB 布局专业规则自动规划） */
	netOrder: NetOrder;
}

export const DEFAULT_ROUTER_OPTIONS: RouterUiOptions = {
	traceWidthMil: 10,
	extraClearanceMil: 0,
	escapeClearanceMil: 0,
	ripupRounds: 3,
	powerNetsFirst: false,
	budgetMs: 1800000,
	// 实时预览：引擎（worker）只管输出结果，主线程写入调度器做「同网合并
	// 差量写入 + 30ms 限速」，写画布不反压引擎。旧实现的 FIFO 队列不合并
	// 不限速，EDA 写盘挤占 worker 算力，实测快扫 26s → 56s。
	livePreview: true,
	finalOptimize: true,
	viaFallback: true,
	allowViaInPad: false,
	routeLayers: [1, 2],
	clearExistingTraces: true,
	excludeNets: [],
	netOrder: 'auto',
};

const NET_ORDERS: NetOrder[] = ['auto', 'river', 'reverse', 'max-x', 'longest', 'shortest'];

export function normalizeRouterOptions(input: Partial<RouterUiOptions> | undefined): RouterUiOptions {
	const o = { ...DEFAULT_ROUTER_OPTIONS, ...(input ?? {}) };
	const clamp = (v: number, lo: number, hi: number, fallback: number): number => {
		const n = Number(v);
		if (!Number.isFinite(n)) return fallback;
		return Math.min(hi, Math.max(lo, n));
	};

	// 顶层是主布线层：无论如何都保留，避免出现「没有可用层」的空规划
	const layers = new Set<number>([1]);
	if (Array.isArray(o.routeLayers) && o.routeLayers.includes(2)) layers.add(2);

	const excluded = Array.isArray(o.excludeNets)
		? Array.from(new Set(o.excludeNets.filter(n => typeof n === 'string').map(n => n.trim()).filter(Boolean)))
		: [];

	return {
		traceWidthMil: clamp(o.traceWidthMil, 4, 60, DEFAULT_ROUTER_OPTIONS.traceWidthMil),
		extraClearanceMil: clamp(o.extraClearanceMil, 0, 40, DEFAULT_ROUTER_OPTIONS.extraClearanceMil),
		escapeClearanceMil: clamp(o.escapeClearanceMil, 0, 40, DEFAULT_ROUTER_OPTIONS.escapeClearanceMil),
		ripupRounds: Math.round(clamp(o.ripupRounds, 0, 6, DEFAULT_ROUTER_OPTIONS.ripupRounds)),
		powerNetsFirst: !!o.powerNetsFirst,
		budgetMs: clamp(o.budgetMs, 0, 3600000, DEFAULT_ROUTER_OPTIONS.budgetMs),
		livePreview: o.livePreview !== false,
		finalOptimize: o.finalOptimize !== false,
		viaFallback: o.viaFallback !== false,
		allowViaInPad: o.allowViaInPad === true,
		routeLayers: [...layers].sort((a, b) => a - b),
		clearExistingTraces: o.clearExistingTraces !== false,
		excludeNets: excluded,
		netOrder: NET_ORDERS.includes(o.netOrder) ? o.netOrder : 'auto',
	};
}

/** 参数弹窗需要的板级信息（层与网络清单） */
export interface BoardLayerChoice {
	id: number;
	name: string;
	/** 该层是否可用于布线（单面板时底层不可用） */
	available: boolean;
}

export interface BoardNetChoice {
	name: string;
	padCount: number;
	kind: 'signal' | 'power' | 'ground';
}

export interface BoardInspection {
	layers: BoardLayerChoice[];
	nets: BoardNetChoice[];
}

/** 进度弹窗展示的统计 */
export interface RouteStats {
	totalNets: number;
	routedNets: number;
	failedNets: number;
	/** 参与布线的焊盘总数（排除网络不计） */
	totalPads: number;
	/** 已消除全部飞线的焊盘数：该焊盘在 MST 中的所有连接都已布通 */
	routedPads: number;
	segments: number;
	vias: number;
	elapsedMs: number;
}

export interface RouteSummary extends RouteStats {
	cancelled: boolean;
	failed: string[];
}
