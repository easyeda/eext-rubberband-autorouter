/**
 * AutoRouterFacade - 统一布线门面
 * 串联 DRC 读取 → PCB 解析 → 路由计算 → 写入画布 的完整流程
 */

import { TopologicalRouter } from './router/TopologicalRouter';
import { showInfo, showSuccess, showError, showWarning } from './utils/toast';
import { Logger } from './utils/logger';
import { ProgressManager, ProgressPhase } from './progress/ProgressManager';

export interface RoutingOptions {
	/** 最大重试轮数（默认 3） */
	maxRetries?: number;
	/** 是否允许切换层（默认 true） */
	enableLayerSwitch?: boolean;
	/** 是否优化过孔数量（默认 true） */
	enableViaOptimize?: boolean;
}

export interface RoutingResult {
	success: boolean;
	tracesCreated: number;
	viasCreated: number;
	netsRouted: number;
	netsTotal: number;
	message: string;
}

const DEFAULT_OPTIONS: Required<RoutingOptions> = {
	maxRetries: 3,
	enableLayerSwitch: true,
	enableViaOptimize: true,
};

export class AutoRouterFacade {
	private router: TopologicalRouter | null = null;
	private logger = new Logger('[AutoRouter]');
	private progress = new ProgressManager();
	private cancelled = false;

	async route(options: RoutingOptions = {}): Promise<RoutingResult> {
		const opts = { ...DEFAULT_OPTIONS, ...options };
		this.cancelled = false;

		// 点击菜单后立刻给用户反馈（此时还没开始重计算）
		showInfo('橡皮筋自动布线：开始布线...');

		// 心跳 Toast：每 5 秒提示一次当前阶段与耗时，
		// 避免长时间计算期间界面"看起来没反应"
		const startedAt = Date.now();
		let lastMessage = '正在初始化...';
		const heartbeat = setInterval(() => {
			const sec = Math.round((Date.now() - startedAt) / 1000);
			showInfo(`布线中 ${sec}s：${lastMessage}`);
		}, 5000);

		this.progress.onUpdate((info) => {
			lastMessage = `[${info.percent}%] ${info.message}`;
			showInfo(lastMessage);
		});

		this.logger.info('开始布线...');
		this.progress.start(ProgressPhase.Parsing, '正在初始化...');

		try {
			const result = await this.runRouting(opts);
			this.progress.done();
			return result;
		} catch (err) {
			const msg = err instanceof Error ? err.message : String(err);
			this.logger.error('布线失败: ' + msg);
			showError('布线失败：' + msg + '（如已有走线写入，请按 Ctrl+Z 撤销）');
			return {
				success: false,
				tracesCreated: 0,
				viasCreated: 0,
				netsRouted: 0,
				netsTotal: 0,
				message: msg,
			};
		} finally {
			clearInterval(heartbeat);
		}
	}

	cancel(): void {
		this.cancelled = true;
		this.router?.cancel();
		this.logger.warn('用户取消布线');
	}

	private async runRouting(opts: Required<RoutingOptions>): Promise<RoutingResult> {
		let lastToastPct = -1;

		this.router = new TopologicalRouter(
			(progress) => {
				this.progress.update(progress.percentage, progress.message);
				// 每 5% 更新一次 toast
				const bucket = Math.floor(progress.percentage / 5) * 5;
				if (bucket !== lastToastPct) {
					lastToastPct = bucket;
					showInfo(`[${progress.percentage}%] ${progress.message}`);
				}
			},
			(message, level) => {
				if (level === 'error') this.logger.error(message);
				else if (level === 'warn') this.logger.warn(message);
				else if (level === 'success') this.logger.success(message);
				else this.logger.info(message);
			},
		);

		const success = await this.router.route({
			maxPasses: opts.maxRetries,
		});

		if (this.cancelled) {
			showWarning('布线已取消（如已有走线写入，请按 Ctrl+Z 撤销）');
			return {
				success: false,
				tracesCreated: 0,
				viasCreated: 0,
				netsRouted: 0,
				netsTotal: 0,
				message: '用户取消',
			};
		}

		return {
			success,
			tracesCreated: 0,
			viasCreated: 0,
			netsRouted: 0,
			netsTotal: 0,
			message: success ? '布线完成' : '布线未完全成功，部分网络降级直连',
		};
	}
}
