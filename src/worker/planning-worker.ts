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

import {
	PlanningEngine, PlanningInput, PlanningResult,
	EnginePad, RenderedPrim, PadShape, PlannedVia, renderedLength,
} from '../router/planning-engine';
import { DRCRules } from '../router/drc-rules';
import { RBObstacle } from '../router/rubberband-engine';

interface PlanMessage {
	type: 'plan';
	input: PlanningInput;
}

interface CancelMessage {
	type: 'cancel';
}

interface TightenRunInput {
	id: number;
	net: string;
	layer: number;
	a: { x: number; y: number };
	b: { x: number; y: number };
	prims: RenderedPrim[];
}

interface TightenMessage {
	type: 'tighten';
	drc: DRCRules;
	allPads: EnginePad[];
	baseWorld: RBObstacle[];
	padShapes: Record<string, PadShape>;
	vias: PlannedVia[];
	runs: TightenRunInput[];
}

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

/** 拉紧主循环：逐段求解，每段结果立刻回传主线程（主线程写画布并刷新） */
async function runTighten(msg: TightenMessage): Promise<void> {
	engine.initTightenWorld(msg.drc, msg.allPads, msg.baseWorld, msg.padShapes);
	const runs = [...msg.runs];
	// 大余量优先：先处理能明显改善的段，画面上立即可见
	runs.sort((p, q) => {
		const sp = renderedLength(p.prims) - Math.hypot(p.b.x - p.a.x, p.b.y - p.a.y);
		const sq = renderedLength(q.prims) - Math.hypot(q.b.x - q.a.x, q.b.y - q.a.y);
		return sq - sp;
	});
	const groupsById = new Map<number, string[]>();
	for (const r of runs) groupsById.set(r.id, engine.addRunToWorld(r.prims));
	for (const v of msg.vias) engine.addViaToWorld(v);

	const total = runs.length;
	let adopted = 0, saved = 0, rounded = 0;
	for (const r of runs) {
		if (cancelled) break;
		const oldLen = renderedLength(r.prims);
		const slack = oldLen - Math.hypot(r.b.x - r.a.x, r.b.y - r.a.y);
		const origPad = engine.padViolations(r.prims, r.net);
		let next: RenderedPrim[] | null = null;
		let newLen = oldLen;
		let kind: 'solve' | 'smooth' | null = null;

		// 1) 真拉紧：可视图重解（焊盘墙优先），更短且零焊盘违规才采纳
		if (slack >= 5) {
			const cand = await engine.solveRunSegment(r.a, r.b, r.layer, r.net);
			if (cand) {
				const smoothed = engine.smoothRun(cand, r.net);
				const tryCand = smoothed.length && renderedLength(smoothed) <= renderedLength(cand) + 0.5 ? smoothed : cand;
				const candLen = renderedLength(tryCand);
				const candPad = engine.padViolations(tryCand, r.net);
				const okShort = candLen < oldLen - 0.5 && candPad === 0;
				const okRepair = candPad === 0 && origPad > 0;
				if ((okShort || okRepair) && await engine.primsClearOfOthers(tryCand, r.net)) {
					next = tryCand;
					newLen = candLen;
					kind = 'solve';
				}
			}
		}
		// 2) 退而求其次：只做拐角圆润（几何不动，只把尖角换成切线弧）
		if (!next) {
			const sm = engine.smoothRun(r.prims, r.net);
			const smLen = renderedLength(sm);
			if (sm.length && smLen < oldLen - 0.5 &&
				engine.padViolations(sm, r.net) <= origPad &&
				await engine.primsClearOfOthers(sm, r.net)) {
				next = sm;
				newLen = smLen;
				kind = 'smooth';
				rounded++;
			}
		}

		if (next) {
			engine.removeWorldGroups(groupsById.get(r.id) ?? []);
			groupsById.set(r.id, engine.addRunToWorld(next));
			adopted++;
			saved += Math.max(0, oldLen - newLen);
		}
		ctx.postMessage({
			type: 'tightenRun',
			id: r.id,
			net: r.net,
			prims: next,
			oldLen,
			newLen,
			kind,
		});
		if ((adopted & 7) === 0) await new Promise(res => setTimeout(res, 0));
	}
	ctx.postMessage({ type: 'tightenDone', total, adopted, saved, rounded });
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
