/**
 * 拓扑自动布线扩展类型定义
 */

// ==================== 连接状态 ====================

/**
 * 路由器连接状态
 */
export type ConnectionStatus = 'disconnected' | 'connected';

// ==================== 布线选项 ====================

/**
 * 布线选项
 */
export interface RoutingOptions {
	/** 最大布线轮数 */
	maxPasses: number;
}

/**
 * 布线进度信息
 */
export interface RoutingProgress {
	/** 当前阶段 */
	stage: string;
	/** 进度百分比 (0-100) */
	percentage: number;
	/** 进度消息 */
	message: string;
	/** 拉紧菜单：已处理网数 / 总网数 / 缩短网数 / 真拉紧入库段数 */
	done?: number;
	total?: number;
	changed?: number;
	adopted?: number;
}

// ==================== IFrame 通信 ====================

/**
 * IFrame 发送到扩展的消息
 */
export interface IFrameMessage {
	/** 消息类型 */
	type: 'start' | 'cancel' | 'close';
	/** 布线选项 (仅 start 时需要) */
	options?: RoutingOptions;
}

// ==================== 默认值 ====================

/**
 * 快速布线默认选项
 */
export const QUICK_ROUTE_OPTIONS: RoutingOptions = {
	maxPasses: 10,
};

/**
 * 自定义布线默认选项
 */
export const CUSTOM_ROUTE_DEFAULT_OPTIONS: RoutingOptions = {
	maxPasses: 100,
};
