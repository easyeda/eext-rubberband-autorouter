/**
 * 橡皮筋自动布线2 - 入口
 * 保留 autoRoute() 和 about() 导出接口不变
 */
import * as extensionConfig from '../extension.json';
import { AutoRouterFacade } from './AutoRouterFacade';
import { showSuccess, showError } from './utils/toast';

// eslint-disable-next-line @typescript-eslint/no-unused-vars
export function activate(_status?: 'onStartupFinished', _arg?: string): void {}

export function about(): void {
	eda.sys_Dialog.showInformationMessage(
		eda.sys_I18n.text('橡皮筋自动布线 v' + extensionConfig.version, undefined, undefined, extensionConfig.version),
		eda.sys_I18n.text('关于'),
	);
}

export async function autoRoute(): Promise<void> {
	const facade = new AutoRouterFacade();
	const result = await facade.route();

	if (result.success) {
		showSuccess(`布线完成！${result.message}`);
	} else {
		showError(`布线失败：${result.message}`);
	}
}
