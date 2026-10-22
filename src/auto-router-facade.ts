/**
 * AutoRouterFacade - 布线门面
 *
 * 负责：
 *  - 参数弹窗（用户自定义布线参数）
 *  - 进度弹窗（实时日志 / 统计 / 手动停止）
 *  - 驱动 TopologicalRouter（EDA 读写 + Worker 规划）
 */

import * as extensionConfig from '../extension.json';
import { TopologicalRouter } from './router/topological-router';
import { fetchDRCRules, deriveDefaultOptions, DRCRules } from './router/drc-rules';
import { DialogBridge } from './ui/dialog-bridge';
import { Logger } from './utils/logger';
import { showInfo, showSuccess, showError, showWarning } from './utils/toast';
import {
	normalizeRouterOptions,
	RouterUiOptions,
	RouteStats,
	BoardInspection,
} from './types/router-params';

const PARAMS_ID = 'params' as const;
const PROGRESS_ID = 'progress' as const;

export class AutoRouterFacade {
	private router: TopologicalRouter | null = null;
	private bridge = new DialogBridge();
	private logger = new Logger('[AutoRouter]');
	private busy = false;
	private progressVisible = false;
	/** 最近一次使用的参数：重新布线 / 再次打开设置时复用 */
	private lastOptions: RouterUiOptions | null = null;

	isBusy(): boolean {
		return this.busy;
	}

	/** 菜单入口：读取 DRC 默认值并打开参数弹窗 */
	async openSettings(): Promise<void> {
		if (this.busy) {
			await this.showProgress();
			return;
		}

		let drcDefaults: RouterUiOptions;
		let drc: DRCRules;
		let inspection: BoardInspection | null = null;
		try {
			// DRC 与板级信息（层/网络清单）并行读取：参数窗打开即可展示可选项
			[drc, inspection] = await Promise.all([
				fetchDRCRules((message, level) => this.logLine(message, level)),
				this.ensureRouter().inspectBoard().catch(() => null),
			]);
			drcDefaults = deriveDefaultOptions(drc);
		} catch {
			drc = await fetchDRCRules();
			drcDefaults = deriveDefaultOptions(drc);
		}

		await new Promise<void>((resolve) => {
			void this.bridge.open(
				PARAMS_ID,
				'/iframe/params.html',
				880,
				680,
				'橡皮筋自动布线 · 参数',
				{
					onReady: (send) => {
						send({
							type: 'defaults',
							options: this.lastOptions ?? drcDefaults,
							drc: {
								traceWidthMil: drc.traceWidthMil,
								trackToTrackMil: drc.trackToTrackMil,
								trackToPadMil: drc.trackToPadMil,
								trackToBoardMil: drc.trackToBoardMil,
							},
							layers: inspection?.layers ?? null,
							nets: inspection?.nets ?? null,
						});
					},
					onMessage: (msg) => {
						if (msg.type === 'run') {
							const options = normalizeRouterOptions(msg.options);
							void this.bridge.close(PARAMS_ID);
							resolve();
							void this.startRouting(options);
						} else if (msg.type === 'cancel' || msg.type === 'close') {
							void this.bridge.close(PARAMS_ID);
							resolve();
						}
					},
					onBeforeClose: () => {
						this.bridge.forget(PARAMS_ID);
						resolve();
					},
				},
			).catch((err) => {
				showError('无法打开参数弹窗：' + (err instanceof Error ? err.message : String(err)));
				resolve();
			});
		});
	}

	/** 已打开进度窗口时，把窗口带到前台 */
	async showProgress(): Promise<void> {
		if (this.progressVisible) {
			await this.bridge.focus(PROGRESS_ID);
		}
	}

	cancel(): void {
		this.router?.cancel();
	}

	private async startRouting(options: RouterUiOptions): Promise<void> {
		if (this.busy) return;
		this.busy = true;
		this.lastOptions = options;
		showInfo('橡皮筋自动布线：开始布线');

		// 进度窗口已存在时直接复用（避免关闭重开导致运行中被 onBeforeClose 取消）
		if (this.progressVisible && this.bridge.isOpen(PROGRESS_ID)) {
			this.bridge.send(PROGRESS_ID, this.beginMessage(options));
			await this.bridge.focus(PROGRESS_ID);
		} else {
			await this.openProgress(options);
		}

		const router = this.ensureRouter();
		try {
			const summary = await router.route(options);
			this.bridge.send(PROGRESS_ID, { type: 'done', summary });
			if (summary.cancelled) {
				showWarning(`布线已停止：${summary.routedNets}/${summary.totalNets} 个网络`);
			} else if (summary.failedNets === 0) {
				showSuccess(`布线完成：${summary.routedNets}/${summary.totalNets} 个网络`);
			} else {
				showWarning(`布线完成：${summary.routedNets}/${summary.totalNets} 个网络，${summary.failedNets} 个失败`);
			}
		} catch (err) {
			const msg = err instanceof Error ? err.message : String(err);
			this.logger.error('布线失败: ' + msg);
			showError('布线失败：' + msg);
			this.bridge.send(PROGRESS_ID, { type: 'error', message: msg });
		} finally {
			this.busy = false;
		}
	}

	/**
	 * 路由器在门面生命周期内复用：它保存上一轮写入画布的图元 ID，
	 * 重新布线时先精确清理这些图元，避免把旧走线当成静态障碍。
	 */
	private ensureRouter(): TopologicalRouter {
		if (!this.router) {
			this.router = new TopologicalRouter(
				(progress) => {
					this.bridge.send(PROGRESS_ID, {
						type: 'progress',
						percentage: progress.percentage,
						message: progress.message,
						stage: progress.stage,
					});
				},
				(message, level) => {
					this.logLine(message, level);
					this.bridge.send(PROGRESS_ID, {
						type: 'log',
						level,
						message,
						time: Date.now(),
					});
				},
				(stats) => {
					this.bridge.send(PROGRESS_ID, { type: 'stats', stats });
				},
			);
		}
		return this.router;
	}

	private beginMessage(options: RouterUiOptions): Record<string, unknown> {
		return {
			type: 'begin',
			version: extensionConfig.version,
			options,
			startedAt: Date.now(),
			stats: this.emptyStats(),
		};
	}

	private async openProgress(options: RouterUiOptions): Promise<void> {
		this.progressVisible = true;
		try {
			await this.bridge.open(
				PROGRESS_ID,
				'/iframe/progress.html',
				620,
				560,
				'橡皮筋自动布线 · 进度',
				{
					onReady: (send) => {
						send(this.beginMessage(options));
					},
					onMessage: (msg) => {
						if (msg.type === 'cancel') {
							this.cancel();
						} else if (msg.type === 'close') {
							this.progressVisible = false;
							void this.bridge.close(PROGRESS_ID);
						} else if (msg.type === 'rerun') {
							if (!this.busy && this.lastOptions) {
								void this.startRouting(this.lastOptions);
							}
						} else if (msg.type === 'openSettings') {
							if (!this.busy) void this.openSettings();
						}
					},
					onBeforeClose: () => {
						this.progressVisible = false;
						this.bridge.forget(PROGRESS_ID);
						if (this.busy) this.cancel();
					},
				},
			);
		} catch (err) {
			this.progressVisible = false;
			// 进度窗口失败不阻断布线；用 Toast 兜底
			this.logger.warn('进度窗口打开失败: ' + (err instanceof Error ? err.message : String(err)));
		}
	}

	private emptyStats(): RouteStats {
		return {
			totalNets: 0,
			routedNets: 0,
			failedNets: 0,
			totalPads: 0,
			routedPads: 0,
			segments: 0,
			vias: 0,
			elapsedMs: 0,
		};
	}

	private logLine(message: string, level: string): void {
		if (level === 'error') this.logger.error(message);
		else if (level === 'warn') this.logger.warn(message);
		else if (level === 'success') this.logger.success(message);
		else this.logger.info(message);
	}
}
