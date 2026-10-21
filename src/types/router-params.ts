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
	/** 扇出圆角半径（mil），0 = 直角 */
	filletRadiusMil: number;
	/** 焊盘逃逸线长度（mil） */
	escapeClearanceMil: number;
	/** 撕毁重布轮数（0 = 关闭） */
	ripupRounds: number;
	/** 电源网络优先布线 */
	powerNetsFirst: boolean;
	/** 整轮规划时间预算（毫秒，0 = 不限制） */
	budgetMs: number;
	/** 实时预览：每布完一个网络立即写入画布 */
	livePreview: boolean;
}

export const DEFAULT_ROUTER_OPTIONS: RouterUiOptions = {
	traceWidthMil: 10,
	extraClearanceMil: 0,
	filletRadiusMil: 40,
	escapeClearanceMil: 50,
	ripupRounds: 2,
	powerNetsFirst: true,
	budgetMs: 300000,
	livePreview: true,
};

export function normalizeRouterOptions(input: Partial<RouterUiOptions> | undefined): RouterUiOptions {
	const o = { ...DEFAULT_ROUTER_OPTIONS, ...(input ?? {}) };
	const clamp = (v: number, lo: number, hi: number, fallback: number): number => {
		const n = Number(v);
		if (!Number.isFinite(n)) return fallback;
		return Math.min(hi, Math.max(lo, n));
	};
	return {
		traceWidthMil: clamp(o.traceWidthMil, 4, 60, DEFAULT_ROUTER_OPTIONS.traceWidthMil),
		extraClearanceMil: clamp(o.extraClearanceMil, 0, 40, DEFAULT_ROUTER_OPTIONS.extraClearanceMil),
		filletRadiusMil: clamp(o.filletRadiusMil, 0, 200, DEFAULT_ROUTER_OPTIONS.filletRadiusMil),
		escapeClearanceMil: clamp(o.escapeClearanceMil, 10, 200, DEFAULT_ROUTER_OPTIONS.escapeClearanceMil),
		ripupRounds: Math.round(clamp(o.ripupRounds, 0, 6, DEFAULT_ROUTER_OPTIONS.ripupRounds)),
		powerNetsFirst: !!o.powerNetsFirst,
		budgetMs: clamp(o.budgetMs, 0, 1800000, DEFAULT_ROUTER_OPTIONS.budgetMs),
		livePreview: o.livePreview !== false,
	};
}

/** 进度弹窗展示的统计 */
export interface RouteStats {
	totalNets: number;
	routedNets: number;
	failedNets: number;
	segments: number;
	vias: number;
	elapsedMs: number;
}

export interface RouteSummary extends RouteStats {
	cancelled: boolean;
	failed: string[];
}
