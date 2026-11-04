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

/** 拉紧已有布线：对当前 PCB 已完成的布线做真拉紧 + 圆弧圆润（独立菜单命令） */
export async function tightenExisting(): Promise<void> {
	const { TopologicalRouter } = await import('./router/topological-router');
	const router = new TopologicalRouter(
		() => {},
		(msg, level) => { console.log(`[Tighten][${level}] ${msg}`); if (level === 'error') eda.sys_Dialog.showErrorMessage(msg); },
		() => {},
	);
	try {
		const res = await router.tightenExistingRouting();
		eda.sys_Dialog.showInformationMessage(`拉紧完成：${res.changed}/${res.nets} 个网络有缩短`);
	} catch (err) {
		eda.sys_Dialog.showErrorMessage('拉紧失败：' + (err instanceof Error ? err.message : String(err)));
	}
}
