/**
 * 简化的 PCB 自动布线器 - 修复版
 * 直接使用 TopologicalRouter，修复坐标单位问题
 */
import * as extensionConfig from '../extension.json';
import { QUICK_ROUTE_OPTIONS } from './types';
import { showInfo, showSuccess, showError } from './utils/toast';
import { TopologicalRouter } from './router/TopologicalRouter';

// IFrame 窗口 ID
const IFRAME_ID = 'routing-config';

// MessageBus 订阅任务
let iframeMessageTask: { cancel: () => void } | null = null;

// eslint-disable-next-line @typescript-eslint/no-unused-vars
export function activate(_status?: 'onStartupFinished', _arg?: string): void {}

export function about(): void {
    eda.sys_Dialog.showInformationMessage(
        eda.sys_I18n.text('橡皮筋自动布线 v' + extensionConfig.version, undefined, undefined, extensionConfig.version),
        eda.sys_I18n.text('关于'),
    );
}

/**
 * 快速自动布线 - 使用修复后的 TopologicalRouter
 */
export async function autoRoute(): Promise<void> {
    showInfo('正在启动拓扑自动布线...');
    
    const router = new TopologicalRouter(
        // 进度回调
        (progress) => {
            console.log(`[TopologicalRouter] ${progress.stage}: ${progress.percentage}%`);
            if (progress.stage && progress.percentage !== undefined) {
                showInfo(`${progress.stage}: ${progress.percentage}%`);
            }
        },
        // 日志回调
        (message, level) => {
            console.log(`[TopologicalRouter] [${level}] ${message}`);
            switch (level) {
                case 'error':
                    showError(message);
                    break;
                case 'success':
                    showSuccess(message);
                    break;
                case 'warn':
                    showInfo(message);
                    break;
            }
        },
        // 状态回调
        (status) => {
            console.log(`[TopologicalRouter] 状态：${status}`);
        },
    );

    try {
        const success = await router.route(QUICK_ROUTE_OPTIONS);
        if (success) {
            showSuccess('自动布线完成！');
        } else {
            showError('自动布线失败');
        }
    } catch (error) {
        console.error('[TopologicalRouter] 布线错误:', error);
        const message = error instanceof Error ? error.message : String(error);
        showError(`布线失败：${message}`);
    }
}

/**
 * 自定义布线
 */
export async function autoRouteCustom(): Promise<void> {
    showInfo('自定义布线功能开发中...');
}
