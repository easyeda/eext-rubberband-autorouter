/**
 * 橡皮筋自动布线 - 入口
 */
import * as extensionConfig from '../extension.json';
import { AutoRouterFacade } from './auto-router-facade';
import { DialogBridge } from './ui/dialog-bridge';
import { showError, showWarning } from './utils/toast';

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

/** 拉紧菜单：单例状态（进度窗 + 运行中的路由实例） */
let tightenBridge: DialogBridge | null = null;
let tightening = false;

/**
 * 拉紧已有布线：对当前 PCB 已完成的布线做真拉紧 + 圆弧圆润（独立菜单命令）。
 * 打开一个紧凑进度弹窗（关键信息：进度/当前网/缩短数），运行中关闭弹窗
 * 即停止拉紧（已写入部分保留）。
 */
export async function tightenExisting(): Promise<void> {
	if (facade?.isBusy()) {
		showWarning('布线正在进行中，请等待完成后再拉紧');
		return;
	}
	if (tightening) {
		// 重复点击：把进度窗带到前台
		await tightenBridge?.focus('tighten');
		return;
	}
	tightening = true;
	const bridge = tightenBridge ?? new DialogBridge();
	tightenBridge = bridge;

	const { TopologicalRouter } = await import('./router/topological-router');
	const router = new TopologicalRouter(
		(p) => bridge.send('tighten', { type: 'progress', ...p }),
		(msg, level) => {
			console.log(`[Tighten][${level}] ${msg}`);
			if (level === 'error') showError(msg);
		},
		() => { /* stats 不用 */ },
	);

	try {
		await bridge.open('tighten', '/iframe/tighten.html', 380, 188, '橡皮筋布线器 · 拉紧已有布线', {
			onReady: (send) => send({ type: 'begin', startedAt: Date.now() }),
			onMessage: (msg) => {
				if (msg.type === 'close') {
					// 运行中关闭 = 停止（协同取消，已写入的网保留）
					if (tightening) router.cancel();
					void bridge.close('tighten');
				}
			},
			onBeforeClose: () => {
				router.cancel();
				bridge.forget('tighten');
			},
		});

		// 完成结果只在进度弹窗里展示（用户要求不再弹完成提示窗）
		const res = await router.tightenExistingRouting();
		sendDone(bridge, res);
	} catch (err) {
		const msg = err instanceof Error ? err.message : String(err);
		bridge.send('tighten', { type: 'error', message: msg });
		showError('拉紧失败：' + msg);
	} finally {
		tightening = false;
	}
}

/** 把最终统计推给弹窗（弹窗转「关闭」态） */
function sendDone(bridge: DialogBridge, res: { nets: number; changed: number; savedMil: number }): void {
	bridge.send('tighten', { type: 'done', total: res.nets, changed: res.changed, savedMil: res.savedMil });
}
