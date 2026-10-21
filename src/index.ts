/**
 * 橡皮筋自动布线 - 入口
 */
import * as extensionConfig from '../extension.json';
import { AutoRouterFacade } from './auto-router-facade';

// eslint-disable-next-line @typescript-eslint/no-unused-vars
export function activate(_status?: 'onStartupFinished', _arg?: string): void {}

export function about(): void {
	eda.sys_Dialog.showInformationMessage(
		eda.sys_I18n.text('橡皮筋自动布线 v' + extensionConfig.version, undefined, undefined, extensionConfig.version),
		eda.sys_I18n.text('关于'),
	);
}

// 菜单可能被重复点击：保持单例，布线中再次点击会聚焦进度窗口
let facade: AutoRouterFacade | null = null;

export async function autoRoute(): Promise<void> {
	if (!facade) facade = new AutoRouterFacade();
	await facade.openSettings();
}
