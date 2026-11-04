/**
 * 扩展主线程 ↔ 内联框架弹窗的通信桥
 *
 * 通信方式：iframe 内调用 `window.parent.postMessage({ source: 'rb-ui', ... })`，
 * 主线程监听 window message，并用 `event.source` 把进度推回对应弹窗。
 * 这样无需额外文件加载或全局状态，也兼容 EDA 的 IFrame 沙箱。
 */

export type DialogName = 'params' | 'progress' | 'tighten';

interface DialogHandler {
	onReady?: (send: (msg: Record<string, unknown>) => void) => void;
	onMessage?: (msg: any) => void;
	onBeforeClose?: () => void;
}

const HOST_SOURCE = 'rb-host';
const UI_SOURCE = 'rb-ui';
const DIALOG_ID_PREFIX = 'rb-dialog-';

/**
 * 右侧贴边定位：x/y 为弹窗左上角在 EDA 界面内的 CSS 像素坐标。
 *
 * 注意必须直接调用 `eda.sys_Window.getViewportSize()`：EDA 扩展沙箱里的
 * `eda` 是注入的局部变量，不一定挂在 `globalThis` 上（`globalThis.eda`
 * 在沙箱里可能为 undefined，回退到 window.innerWidth 会拿到错误尺寸，
 * 表现为 x/y 失效或弹窗仍出现在中间）。
 *
 * 已知 API bug（2026-10-23，客户端 4.1.84 实测）：openIFrame 的 props.x/y
 * 被客户端忽略——任意 id、任意 x/y（含 0,0）都落在同一级联默认位。
 * 这里照常传坐标：API 修复后贴边定位自动生效，无需再改代码。
 *
 * 取不到视口尺寸时返回空对象，让 EDA 使用默认居中位置。
 */
export function rightEdgePosition(width: number, height: number): { x?: number; y?: number } {
	let viewportWidth = 0;
	let viewportHeight = 0;
	try {
		const size = eda?.sys_Window?.getViewportSize?.();
		viewportWidth = Number(size?.width) || 0;
		viewportHeight = Number(size?.height) || 0;
	} catch { /* 旧版 EDA 没有该接口时回退 window */ }

	if (viewportWidth <= 0 || viewportHeight <= 0) {
		// 回退：扩展页面自身的窗口尺寸（仅当 EDA 视口接口不可用时）
		const host = globalThis as any;
		const doc = host.document;
		if (viewportWidth <= 0) viewportWidth = Number(host.innerWidth) || Number(doc?.documentElement?.clientWidth) || 0;
		if (viewportHeight <= 0) viewportHeight = Number(host.innerHeight) || Number(doc?.documentElement?.clientHeight) || 0;
	}
	if (viewportWidth <= 0) return {};

	const margin = 16;
	// 弹窗实际外框比 iframe 内容多出标题栏/边框，右侧留出的边距里扣除这部分余量，
	// 避免窗口右边被视口裁掉。高度按外框估算后纵向居中。
	const chromeWidth = 8;
	const chromeHeight = 36;
	return {
		x: Math.max(margin, Math.round(viewportWidth - width - chromeWidth - margin)),
		y: viewportHeight > 0
			? Math.max(margin, Math.round((viewportHeight - height - chromeHeight) / 2))
			: undefined,
	};
}

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
			if (dialog !== 'params' && dialog !== 'progress' && dialog !== 'tighten') return;

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
			// 参数窗与进度窗统一贴 EDA 界面右侧，纵向居中
			...rightEdgePosition(width, height),
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
