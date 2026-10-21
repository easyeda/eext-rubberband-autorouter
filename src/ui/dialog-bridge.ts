/**
 * 扩展主线程 ↔ 内联框架弹窗的通信桥
 *
 * 通信方式：iframe 内调用 `window.parent.postMessage({ source: 'rb-ui', ... })`，
 * 主线程监听 window message，并用 `event.source` 把进度推回对应弹窗。
 * 这样无需额外文件加载或全局状态，也兼容 EDA 的 IFrame 沙箱。
 */

export type DialogName = 'params' | 'progress';

interface DialogHandler {
	onReady?: (send: (msg: Record<string, unknown>) => void) => void;
	onMessage?: (msg: any) => void;
	onBeforeClose?: () => void;
}

const HOST_SOURCE = 'rb-host';
const UI_SOURCE = 'rb-ui';
const DIALOG_ID_PREFIX = 'rb-dialog-';

export class DialogBridge {
	private installed = false;
	private handlers = new Map<DialogName, DialogHandler>();
	private sources = new Map<DialogName, any>();
	private queues = new Map<DialogName, Array<Record<string, unknown>>>();
	private opened = new Set<DialogName>();

	install(): void {
		if (this.installed) return;
		const host = globalThis as any;
		if (typeof host.addEventListener !== 'function') {
			throw new Error('当前环境不支持内联框架消息通信');
		}
		host.addEventListener('message', (event: any) => {
			const data = event?.data;
			if (!data || data.source !== UI_SOURCE) return;
			const dialog = data.dialog as DialogName;
			if (dialog !== 'params' && dialog !== 'progress') return;

			if (data.type === 'ready') {
				if (event.source) this.sources.set(dialog, event.source);
				this.flush(dialog);
				this.handlers.get(dialog)?.onReady?.(msg => this.send(dialog, msg));
				return;
			}
			this.handlers.get(dialog)?.onMessage?.(data);
		});
		this.installed = true;
	}

	async open(
		name: DialogName,
		url: string,
		width: number,
		height: number,
		title: string,
		handler: DialogHandler,
	): Promise<void> {
		this.install();
		const iframe = (eda as any).sys_IFrame;
		if (!iframe || typeof iframe.openIFrame !== 'function') {
			throw new Error('当前 EDA 版本不支持内联框架弹窗');
		}

		// 同名窗口已存在时先关闭，避免实例错乱
		if (this.opened.has(name)) {
			await this.close(name);
		}

		this.handlers.set(name, handler);
		this.queues.set(name, []);
		this.sources.delete(name);

		const id = DIALOG_ID_PREFIX + name;
		const ok = await iframe.openIFrame(url, width, height, id, {
			title,
			minimizeButton: true,
			maximizeButton: false,
			grayscaleMask: false,
			onBeforeCloseCallFn: () => {
				this.handlers.get(name)?.onBeforeClose?.();
				return true;
			},
		});
		if (!ok) {
			this.handlers.delete(name);
			this.queues.delete(name);
			throw new Error('打开参数窗口失败');
		}
		this.opened.add(name);
	}

	send(name: DialogName, msg: Record<string, unknown>): void {
		const source = this.sources.get(name);
		if (!source || typeof source.postMessage !== 'function') {
			const queue = this.queues.get(name) ?? [];
			queue.push(msg);
			this.queues.set(name, queue);
			return;
		}
		try {
			source.postMessage({ source: HOST_SOURCE, ...msg }, '*');
		} catch (err) {
			console.warn('[DialogBridge] 发送消息失败:', err);
		}
	}

	async close(name: DialogName): Promise<void> {
		const iframe = (eda as any).sys_IFrame;
		try {
			if (iframe?.closeIFrame) await iframe.closeIFrame(DIALOG_ID_PREFIX + name);
		} catch { /* ignore */ }
		this.handlers.delete(name);
		this.queues.delete(name);
		this.sources.delete(name);
		this.opened.delete(name);
	}

	async focus(name: DialogName): Promise<boolean> {
		const iframe = (eda as any).sys_IFrame;
		if (!iframe?.isIFrameAlreadyExist) return false;
		try {
			const id = DIALOG_ID_PREFIX + name;
			if (!(await iframe.isIFrameAlreadyExist(id))) return false;
			if (iframe.showIFrame) await iframe.showIFrame(id);
			return true;
		} catch {
			return false;
		}
	}

	isOpen(name: DialogName): boolean {
		return this.opened.has(name);
	}

	/** 用户从标题栏关闭窗口时清理本地状态（不再重复调用 closeIFrame） */
	forget(name: DialogName): void {
		this.handlers.delete(name);
		this.queues.delete(name);
		this.sources.delete(name);
		this.opened.delete(name);
	}

	private flush(name: DialogName): void {
		const queue = this.queues.get(name);
		if (!queue?.length) return;
		this.queues.set(name, []);
		for (const msg of queue) this.send(name, msg);
	}
}
