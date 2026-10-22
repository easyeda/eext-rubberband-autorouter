/**
 * 布线规划 Worker
 *
 * 只依赖纯几何代码（PlanningEngine / VisibilityRouter / RubberbandEngine），
 * 不引用任何 `eda.*` API。主线程负责读取 PCB 数据与写入图元。
 */

import { PlanningEngine, PlanningInput, PlanningResult } from '../router/planning-engine';

interface PlanMessage {
	type: 'plan';
	input: PlanningInput;
}

interface CancelMessage {
	type: 'cancel';
}

type IncomingMessage = PlanMessage | CancelMessage;

// eslint-disable-next-line @typescript-eslint/no-explicit-any
const ctx = self as any;
const engine = new PlanningEngine();
let cancelled = false;

function safeError(err: unknown): string {
	if (err instanceof Error) return err.message;
	try {
		return String(err);
	} catch {
		return '未知错误';
	}
}

ctx.onmessage = async (event: MessageEvent<IncomingMessage>) => {
	const msg = event.data;
	if (!msg) return;

	if (msg.type === 'cancel') {
		cancelled = true;
		engine.cancel();
		return;
	}

	if (msg.type !== 'plan') return;

	cancelled = false;
	try {
		const result: PlanningResult = await engine.plan(msg.input, {
			onLog: (message, level) => ctx.postMessage({ type: 'log', message, level }),
			onProgress: (percentage, message) => ctx.postMessage({ type: 'progress', percentage, message }),
			onNetUpdate: (name, prims, vias, ok, pads) => ctx.postMessage({ type: 'net', name, prims, vias, ok, pads }),
			onRipup: (names) => ctx.postMessage({ type: 'ripup', names }),
			isCancelled: () => cancelled,
		});
		ctx.postMessage({ type: 'done', result });
	} catch (err) {
		ctx.postMessage({ type: 'error', message: safeError(err) });
	}
};
