/**
 * 布线规划 Worker
 *
 * 只依赖纯几何代码（PlanningEngine / VisibilityRouter / RubberbandEngine），
 * 不引用任何 `eda.*` API。主线程负责读取 PCB 数据与写入图元。
 *
 * 两类任务：
 *  - plan：完整布线规划（逐网结果流式回传，主线程差量写入）
 *  - tighten：拉紧已有布线（按连续导线段逐条求解，主线程逐段回写并刷新）
 */

import { PlanningEngine, PlanningInput, PlanningResult } from '../router/planning-engine';
import { runTightenJob, TightenJobInput } from '../router/tighten-job';

interface PlanMessage {
	type: 'plan';
	input: PlanningInput;
}

interface CancelMessage {
	type: 'cancel';
}

type TightenMessage = TightenJobInput;

type IncomingMessage = PlanMessage | CancelMessage | TightenMessage;

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

/**
 * 拉紧任务：几何全部在 tighten-job / tighten-core（与主线程回退共用同一份代码），
 * 这里只把事件转发给主线程。PlanningEngine 只为可选的「A* 重解 / 整网重布」提候选。
 */
async function runTighten(msg: TightenMessage): Promise<void> {
	await runTightenJob(msg, m => ctx.postMessage(m), () => cancelled, engine);
}

ctx.onmessage = async (event: MessageEvent<IncomingMessage>) => {
	const msg = event.data;
	if (!msg) return;

	if (msg.type === 'cancel') {
		cancelled = true;
		engine.cancel();
		return;
	}

	if (msg.type === 'tighten') {
		cancelled = false;
		try {
			await runTighten(msg);
		} catch (err) {
			ctx.postMessage({ type: 'error', message: safeError(err) });
		}
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
