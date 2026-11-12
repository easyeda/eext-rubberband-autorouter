/**
 * 「拉紧已有布线」离线夹具测试：驱动真实 worker 入口（src/worker/planning-worker.ts，消息 type:'tighten'），
 * 用 bench/tighten-metrics.mjs 的严格指标（实际线宽、真实焊盘形状、稠密采样）断言结果。
 *
 * 运行：npx tsx test/tighten-fixtures-test.ts
 * 环境变量：
 *   TIGHTEN_SRC_ROOT  被测源码根目录（默认本仓库）；指向修复前快照可复现旧代码问题
 *   TIGHTEN_WORLD     clean（默认，新契约：baseWorld 只含静态铜）| prod（旧主线程：baseWorld 还含现有走线副本）
 *   TIGHTEN_ONLY      只跑 id 以该前缀开头的夹具（逗号分隔多个，如 a-,c-through）
 *
 * 每个夹具断言：
 *   (1) 无遗留自由尖角（>=30°；拉紧后位于 run 端点/己网焊盘中心/过孔的角不计）
 *   (2) 长度不增加；有解析 taut 长度的夹具：结果 <= taut*(1+3%)，且不短于 taut（预言机自检）
 *   (3) 无「新」严格 DRC 违规（按实际线宽、真实焊盘形状；按位置+外部元件判定：附近没有同一元件上不更差的既有违规才算新）
 *   (4) 焊盘连通分区不变、线宽不变、悬空端点不增加（T 节点度数保持）
 *   (5) 二次拉紧幂等（总长变化 < 0.5mil、几何逐段一致（0.01mil）、且无新的自由尖角）
 * 超时（cancel）或 worker 没有发出 tightenDone 都算失败（断言 + 非零退出），不会静默通过。
 * 这些测试在修复前的旧代码上预期失败（先复现问题）。
 */
import * as path from 'path';

let passed = 0;
let failed = 0;
const failures: string[] = [];
let currentId = '';

function assert(condition: boolean, message: string): void {
	if (condition) {
		console.log(`  ✅ ${message}`);
		passed++;
	} else {
		console.error(`  ❌ ${message}`);
		failed++;
		failures.push(`${currentId}: ${message}`);
	}
}

// eslint-disable-next-line @typescript-eslint/no-explicit-any
type Any = any;
const f1 = (v: number) => (Math.round(v * 10) / 10).toString();
const f2 = (v: number) => (Math.round(v * 100) / 100).toString();

async function main(): Promise<void> {
	const B: Any = await import('../bench/tighten-boards.mjs');
	const M: Any = await import('../bench/tighten-metrics.mjs');
	const { buildFixtures } = await import('./tighten-fixtures');

	const root = path.resolve(process.env.TIGHTEN_SRC_ROOT || path.join(__dirname, '..'));
	const world = process.env.TIGHTEN_WORLD === 'prod' ? 'prod' : 'clean';
	const only = process.env.TIGHTEN_ONLY || '';
	console.log(`源码根: ${root}  world=${world}`);
	const code: string = await B.buildBundle(root);
	const { mod } = await B.loadFresh(code);

	const measure = (board: Any, nets: Any[], vias: Any[]) => {
		const ctx = B.metricsCtx(mod, board, nets, vias);
		return {
			ctx, counts: M.counts(ctx), sharp: M.sharpCorners(ctx, { minDeg: 30 }), drc: M.strictDrc(ctx),
			conn: M.connectivity(ctx), dangling: M.danglingEnds(ctx),
		};
	};
	/** 节点（端点聚类 0.8mil）的度数 */
	const degreeAt = (nets: Any[], net: string, x: number, y: number): number => {
		let d = 0;
		for (const n of nets) {
			if (n.name !== net) continue;
			for (const p of n.prims) {
				const e = p.kind === 'line'
					? [[p.x1, p.y1], [p.x2, p.y2]]
					: [[p.cx + p.r * Math.cos(p.startAngle), p.cy + p.r * Math.sin(p.startAngle)],
						[p.cx + p.r * Math.cos(p.startAngle + p.sweep), p.cy + p.r * Math.sin(p.startAngle + p.sweep)]];
				for (const q of e) if (Math.hypot(q[0] - x, q[1] - y) <= 0.8) d++;
			}
		}
		return d;
	};

	for (const fx of buildFixtures(B) as Any[]) {
		if (only && !only.split(',').some(x => fx.id.startsWith(x.trim()))) continue;
		currentId = fx.id;
		console.log(`\n▶ ${fx.id} — ${fx.title}${fx.note ? ` [${fx.note}]` : ''}`);
		const { board, nets, vias } = fx;

		// ── 夹具自检 ──
		const before = measure(board, nets, vias);
		if (fx.violatesBefore) {
			assert(before.drc.violations.length >= 1, `夹具有效：修复前存在既有违规（${before.drc.violations.length} 处，最差间距 ${f1(before.drc.violations[0]?.gap ?? 0)}）`);
		} else {
			assert(before.drc.violations.length === 0, `夹具有效：修复前按实际线宽严格 DRC 干净（${before.drc.violations.length} 处${before.drc.violations[0] ? `，最差 ${before.drc.violations[0].kind} ${f1(before.drc.violations[0].gap)}` : ''}）`);
		}
		const connBefore = Object.values(before.conn) as Any[];
		assert(connBefore.every(c => c.parts.length === 1 && c.floating === 0), `夹具有效：修复前每个网的焊盘全部连通（分区 ${JSON.stringify(connBefore.map(c => c.parts.length))}）`);
		if (fx.taut !== undefined) assert(fx.taut <= before.counts.lengthMil + 0.5,
			`夹具有效：解析 taut 长度 ${f1(fx.taut)} 不长于原走线 ${f1(before.counts.lengthMil)}`);

		// ── 拉紧 ──
		const p1 = await B.tightenPass({ code, board, nets, vias, world, timeoutMs: 120000 });
		if (p1.error) { assert(false, `worker 报错：${p1.error}`); continue; }
		if (p1.failure) { assert(false, `拉紧未正常结束：${p1.failure}`); continue; }
		const after = measure(board, p1.nets, vias);
		console.log(`  · 长度 ${f1(before.counts.lengthMil)} → ${f1(after.counts.lengthMil)}，弧 ${before.counts.arcs} → ${after.counts.arcs}，线 ${before.counts.lines} → ${after.counts.lines}，${p1.runs.length} 段 run，回写 ${p1.runsReported}，整网重布 ${p1.rewrittenNets.length}`);

		// (1) 自由尖角
		assert(after.sharp.free === 0,
			`(1) 无遗留自由尖角 ≥30°：拉紧后 ${after.sharp.free} 个（拉紧前 ${before.sharp.free}）${after.sharp.list.filter((c: Any) => !c.atFixed).slice(0, 3).map((c: Any) => ` [${c.type} ${c.angle}° @${f1(c.x)},${f1(c.y)}]`).join('')}`);

		// (2) 长度 / 够紧
		assert(after.counts.lengthMil <= before.counts.lengthMil + 0.01, `(2) 长度未增加：${f1(before.counts.lengthMil)} → ${f2(after.counts.lengthMil)}`);
		if (fx.taut !== undefined) {
			const tol = fx.tautTol ?? (fx.id.startsWith('a-') ? 0.02 : 0.03);
			assert(after.counts.lengthMil <= fx.taut * (1 + tol) + 0.5,
				`(2) 够紧：结果 ${f1(after.counts.lengthMil)} ≤ 解析 taut ${f1(fx.taut)} 的 ${Math.round(tol * 100)}% 之内（超出 ${f1((after.counts.lengthMil / fx.taut - 1) * 100)}%）`);
			assert(after.counts.lengthMil >= fx.taut - 1.0,
				`(2) 预言机自检：结果 ${f1(after.counts.lengthMil)} 不短于解析下限 ${f1(fx.taut)}（更短 = 抄近道穿过障碍/离开了必经焊盘）`);
		}

		if (fx.maxLen !== undefined) assert(after.counts.lengthMil <= fx.maxLen,
			`(2) 够紧：结果 ${f1(after.counts.lengthMil)} ≤ 上限 ${f1(fx.maxLen)}（仍须经过焊盘铜皮）`);

		// (3) 新增 DRC
		const nv = M.newViolations(before.drc, after.drc);
		assert(nv.length === 0,
			`(3) 无新增严格 DRC 违规：新增 ${nv.length} 处（拉紧后总计 ${after.drc.violations.length}）${nv.slice(0, 3).map((v: Any) => ` [${v.netA}/${v.netB} ${v.kind} 间距 ${f1(v.gap)}<${v.rule}${v.beforeGap === null ? '' : `，原 ${f1(v.beforeGap)}`} @${f1(v.x)},${f1(v.y)}]`).join('')}`);

		// (3b) 回写误差预算：新增的间距不得近于「规则 − 0.01」（旧口径只有 GAP_TOL=0.05 的宽容；求解器自己保证 ≥ 规则 + 0.15 − 0.008）
		const nv2 = M.newViolations(before.drc, after.drc, 0.01);
		assert(nv2.length === 0,
			`(3b) 无新增近于 规则−0.01 的间距：新增 ${nv2.length} 处${nv2.slice(0, 3).map((v: Any) => ` [${v.netA}/${v.netB} ${v.kind} 间距 ${v.gap.toFixed(3)}<${v.rule}@${f1(v.x)},${f1(v.y)}]`).join('')}`);
		if (fx.padDepth) {
			const pd = fx.padDepth;
			let dmin = Infinity;
			for (const n of p1.nets) for (const p of n.prims) {
				const L = M.primLength(p), k = Math.max(2, Math.ceil(L / 0.1));
				for (let i = 0; i <= k; i++) {
					const t = i / k;
					const x = p.kind === 'line' ? p.x1 + (p.x2 - p.x1) * t : p.cx + p.r * Math.cos(p.startAngle + p.sweep * t);
					const y = p.kind === 'line' ? p.y1 + (p.y2 - p.y1) * t : p.cy + p.r * Math.sin(p.startAngle + p.sweep * t);
					dmin = Math.min(dmin, Math.hypot(x - pd.x, y - pd.y));
				}
			}
			assert(dmin <= pd.r - pd.width / 2 + 0.02,
				`(3c) 穿过己网圆焊盘：中心线离圆心最近 ${f2(dmin)} ≤ r − w/2 = ${f2(pd.r - pd.width / 2)}（整个线宽压在铜上，不是只擦边）`);
		}

		// (4) 连通 / 线宽 / 悬空
		const cd = M.connectivityDiff(before.conn, after.conn);
		assert(cd.length === 0, `(4) 焊盘连通分区不变：${cd.length ? cd.map((x: Any) => `${x.net} ${x.kind} ${JSON.stringify(x.after)}`).join('; ') : '一致'}`);
		const wp = M.widthsPreserved(M.toNets(nets, (_n: Any, p: Any) => p.width), M.toNets(p1.nets, (_n: Any, p: Any) => p.width));
		assert(wp.ok, `(4) 线宽集合不变${wp.ok ? '' : `：${JSON.stringify(wp.bad)}`}`);
		assert(after.dangling.count <= before.dangling.count, `(4) 无新增悬空端点：${before.dangling.count} → ${after.dangling.count}${after.dangling.list[0] ? ` [@${f1(after.dangling.list[0].x)},${f1(after.dangling.list[0].y)}]` : ''}`);
		for (const kn of fx.keepNodes ?? []) {
			const dg = degreeAt(p1.nets, kn.net, kn.x, kn.y);
			assert(dg === kn.degree, `(4) 节点 (${kn.x},${kn.y}) 度数保持 ${kn.degree}：实际 ${dg}`);
		}

		// (5) 幂等
		const p2 = await B.tightenPass({ code, board, nets: p1.nets, vias, world, timeoutMs: 120000 });
		if (p2.error) { assert(false, `(5) 第二次拉紧 worker 报错：${p2.error}`); continue; }
		if (p2.failure) { assert(false, `(5) 第二次拉紧未正常结束：${p2.failure}`); continue; }
		const after2 = measure(board, p2.nets, vias);
		const dl = after2.counts.lengthMil - after.counts.lengthMil;
		assert(Math.abs(dl) < 0.5, `(5) 二次拉紧长度变化 < 0.5mil：${f2(dl)}（改动 ${p2.runsChanged} 段）`);
		const geo = M.sameGeometry(p1.nets, p2.nets, 0.01);
		assert(geo.equal, `(5) 二次拉紧几何不变（逐段 0.01mil）${geo.equal ? '' : `：${JSON.stringify(geo.diffs.slice(0, 3))}`}`);
		assert(after2.sharp.free <= after.sharp.free, `(5) 二次拉紧无新增自由尖角：${after.sharp.free} → ${after2.sharp.free}`);
	}

	console.log(`\n${'─'.repeat(50)}`);
	console.log(`测试结果：${passed} 通过，${failed} 失败`);
	if (failed > 0) {
		console.log('失败项：');
		for (const f of failures) console.log(`  - ${f}`);
	}
	process.exit(failed > 0 ? 1 : 0);
}

main().catch(err => {
	console.error('测试异常：', err);
	process.exit(1);
});
