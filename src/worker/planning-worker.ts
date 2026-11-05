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
	let adopted = 0, saved = 0, rounded = 0, solved = 0;
	/** A* 重解总预算：整板需 10 分钟内完成，重解只占其中一部分（其余是写画布/刷新） */
	const SOLVE_BUDGET_MS = 180_000;
	const SOLVE_MAX_ATTEMPTS = 600;
	let solveSpent = 0, solveCount = 0;
	for (const r of runs) {
		if (cancelled) break;
		const oldLen = renderedLength(r.prims);
		const origPad = engine.padViolations(r.prims, r.net);
		let next: RenderedPrim[] | null = null;
		let newLen = oldLen;
		let kind: 'solve' | 'smooth' | null = null;

		// 1) 拉直 + 圆润（纯几何、便宜，对所有段常开）
		//    先 string-pulling：两两导线段之间求最短距离，隔点直连清晰就删中间
		//    顶点，把 45° 楼梯拉成最紧折线；再把剩余拐角换成切线弧。
		//    ⚠ 采纳闸门不能只卡「长度缩短≥0.3mil」：短腿 45° 角的小圆角只省
		//    0.1~0.2mil，会被误拒（实测 70 个可圆的内点因此没动）。只要「圆弧
		//    数增加」或「长度变短」且不变长，就采纳。
		const pulled = engine.tautenRun(r.prims, r.net);
		const pulledLen = renderedLength(pulled);
		const base = pulledLen < oldLen - 0.1 ? pulled : r.prims;
		const sm = engine.smoothRun(base, r.net);
		const candidate = renderedLength(sm) < renderedLength(base) - 0.1 ? sm : base;
		const candLen = renderedLength(candidate);
		const origArcN = r.prims.filter(p => p.kind === 'arc').length;
		const candArcN = candidate.filter(p => p.kind === 'arc').length;
		const improved = candArcN > origArcN || candLen < oldLen - 0.1;
		// ⚠ 不要再对整条 run 做 primsClearOfOthers：run 里只要有一处贴着焊盘
		//（原本 DRC 干净、未改动的段被 0.3mil 余量判越界），整条 run 被拒，
		// 连同一 run 里开阔处的可圆角也跟着不圆（实测 40/61 个角周围有
		// 16~80mil 空地却被挡）。圆角/拉直本身已逐弧 arcClear、逐线 segClear
		// 过了净距，这里只做「焊盘违规不恶化」，足够安全。
		if (improved && candLen <= oldLen + 0.5 &&
			engine.padViolations(candidate, r.net) <= origPad) {
			next = candidate;
			newLen = candLen;
			kind = 'smooth';
		}

		// 2) 拆线重布（可视图 A*）：只对「便宜拉直没把拓扑改短 + 明显绕路」的段。
		//    ⚠ 触发条件不能用 `!next`：第 1 阶段几乎每段都能加出圆角（= next
		//    非空），于是 A* 永远不跑，整板只剩「钝角处加个圆弧」而不改走线
		//    （用户实测：一段明显绕行的走线只被圆了角）。也不能只看「拉直省了
		//    多少」——实测 500mil 绕路拉直到 356.8mil（仍比直线 300mil 长 19%），
		//    这时 A* 还能压到 300mil。要看「候选还比直线长多少」。
		//    primsClearOfOthers 现已纳入焊盘，候选不会压焊盘 → 不产 DRC 错误。
		const straight = Math.hypot(r.b.x - r.a.x, r.b.y - r.a.y);
		const detour = oldLen - straight;
		const stillDetoured = candLen > straight + Math.max(15, straight * 0.08);
		if (detour >= 25 && straight > 5 && stillDetoured &&
			solveCount < SOLVE_MAX_ATTEMPTS && solveSpent < SOLVE_BUDGET_MS) {
			const tSolve = Date.now();
			const sc = await engine.solveRunSegment(r.a, r.b, r.layer, r.net);
			solveSpent += Date.now() - tSolve;
			solveCount++;
			if (sc) {
				const sm2 = engine.smoothRun(sc, r.net);
				const tryCand = sm2.length && renderedLength(sm2) <= renderedLength(sc) + 0.5 ? sm2 : sc;
				const tl = renderedLength(tryCand);
				const tp = engine.padViolations(tryCand, r.net);
				const okShort = tl < oldLen - 0.5 && tp === 0;
				const okRepair = tp === 0 && origPad > 0;
				// 与第 1 阶段候选择优：重解至少要再短 2mil 才值得改写画布
				const better = !next || tl < newLen - 2;
				if (better && (okShort || okRepair) && await engine.primsClearOfOthers(tryCand, r.net)) {
					next = tryCand;
					newLen = tl;
					kind = 'solve';
				}
			}
		}

		if (next) {
			engine.removeWorldGroups(groupsById.get(r.id) ?? []);
			groupsById.set(r.id, engine.addRunToWorld(next));
			adopted++;
			saved += Math.max(0, oldLen - newLen);
			if (kind === 'smooth') rounded++;
			if (kind === 'solve') solved++;
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
	ctx.postMessage({
		type: 'tightenDone', total, adopted, saved, rounded, solved,
		solveAttempts: solveCount, solveMs: Math.round(solveSpent),
	});
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
