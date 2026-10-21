/**
 * Worker 源码内联与创建
 *
 * 构建时由 esbuild 插件 `inline-worker:planning` 把 Worker 入口打包成
 * 一段独立的 IIFE 字符串内联进主包，运行时通过 Blob URL 创建 Worker。
 * 这样扩展包只需要一个 dist/index.js，不依赖额外文件加载路径。
 */

import workerSource from 'inline-worker:planning';

export const PLANNING_WORKER_SOURCE: string = workerSource;

export function isWorkerSupported(): boolean {
	const g = globalThis as any;
	return typeof g.Worker === 'function' &&
		typeof g.Blob === 'function' &&
		typeof g.URL?.createObjectURL === 'function';
}

/**
 * 创建规划 Worker；环境不支持时返回 null（调用方回退到主线程计算）。
 */
export function createPlanningWorker(): Worker | null {
	if (!isWorkerSupported()) return null;
	let url: string | null = null;
	try {
		url = URL.createObjectURL(new Blob([PLANNING_WORKER_SOURCE], { type: 'text/javascript' }));
		const worker = new Worker(url);
		// Worker 已开始加载，延迟释放 Blob URL
		setTimeout(() => {
			try { if (url) URL.revokeObjectURL(url); } catch { /* ignore */ }
		}, 10000);
		return worker;
	} catch (err) {
		if (url) {
			try { URL.revokeObjectURL(url); } catch { /* ignore */ }
		}
		console.warn('[Router] Worker 创建失败，回退主线程计算:', err);
		return null;
	}
}
