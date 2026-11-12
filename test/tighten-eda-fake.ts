/**
 * 仿真 EDA 画布（只用于单测）：复现真机（lceda-pro 4.1.99）的线/弧行为。
 * 依据分三类（只仿记录过的行为）：
 *  - 【探测实证】.zplat/task-note.md 真机探测记录：第 1 条、线 delete 恒 true、第 3 条除「弧端点拆线」外的全部、第 4 条；
 *  - 【e2e 观察】真机端到端故障注入（时钟板 LCD3，见提交 0cd55c8 与 tighten-writeback restore() 注释）：
 *    弧端点落在同网同层线内部时线被拆开；「重叠只补剩余部分」（overlapKeepsRemainder，默认关）；
 *  - 【假设】弧 delete 对未知 id 返回 false（来自提交 6c62b77 的记录，探测记录只写了 delete 恒 true）；
 *    seedArc 与 createArc 的拆线行为一致（真机只观察到 create 时拆，seed 模拟「板上已有」的状态，取同一结果）。
 * 行为清单——
 *  1. create 对非法输入抛异常（NaN、非法层、零长线、扫角 0 / ±360）；< 0.1mil 的线返回 id 但不落盘；
 *  2. 线 delete 永远返回 true；弧 delete 对未知/已删 id 返回 false（两者都不可信）；
 *  3. create 时同网合并/拆分并换新 id：
 *     - 同网同层同线宽：重复线只留一根；共线首尾相接/部分重叠 → 合成一根（新 id）；新线是旧线子段 → 被吞；
 *     - 近共线（转角 < 1°）共端点 → 换成远端点间一根直线（丢公共顶点），对合并结果反复进行；
 *       顶点处已有第三个同网图元 / 过孔、线宽不同、或中间夹弧时不合并；
 *     - 同网交叉 / T 接（端点落在另一线内部）/ 新线穿过已有过孔 → 在该点拆开，所有碎片换新 id；
 *       先有线后加的过孔不拆；异网交叉不拆；
 *     - 重复弧：新弧被吞（旧 id 不变）；子弧不合并；同圆相邻弧不合并；
 *     - 弧端点落在同网同层线的内部 → 该线在此点拆开（真机 LCD3 实测：恢复时弧起点读回取整，落在线内部 0.028mil，线被拆出一根碎线）；
 *       线穿过已有弧端点同理；
 *  4. 弧读回端点按 0.1mil、扫角按 0.1° 取整（存储精确），|扫角| < 0.05° 读回 0；线读回不取整。
 */

import type { WritebackApi } from '../src/router/tighten-writeback';

export interface FLine { net: string; layer: number; width: number; sx: number; sy: number; ex: number; ey: number }
export interface FArc extends FLine { deg: number }
interface P { x: number; y: number }

const EPS = 1e-4;
const r1 = (v: number): number => Math.round(v * 10) / 10;
const d = (a: P, b: P): number => Math.hypot(a.x - b.x, a.y - b.y);

export class FakeEda {
	lines = new Map<string, FLine>();
	arcs = new Map<string, FArc>();
	vias: Array<{ net: string; x: number; y: number }> = [];
	private seq = 0;
	creates = 0;
	deletes = 0;
	reads = 0;
	/** 第 n 次（从 1 计）create 抛异常；failFrom: 从第 n 次起全部抛 */
	failAt?: number;
	failFrom?: number;
	log: string[] = [];
	/**
	 * 【e2e 观察】（拉紧故障注入）：新线与已有同网同层同宽的共线线部分重叠、且新线不被已有线包含时，已有线原样保留，
	 * 新线只补出「不重叠的剩余部分」（留下一小段碎线）——而不是合成一根。
	 * true = 仿这一行为（剩余部分直接落盘，不再参与合并/拆分）；false（默认）= 探测记录的「合成一根」。
	 */
	overlapKeepsRemainder = false;
	/** 读图元（listLines / listArcs）时抛异常的开关（测试终检读失败） */
	failReads = false;

	private nid(p: string): string { return `${p}${++this.seq}`; }

	private checkFault(): void {
		this.creates++;
		if (this.creates === this.failAt || (this.failFrom !== undefined && this.creates >= this.failFrom)) {
			throw new Error('fault injection');
		}
	}

	addVia(net: string, x: number, y: number): void { this.vias.push({ net, x, y }); }

	/** 直接放一根线（不触发合并/拆分：模拟板上已有的铜） */
	seedLine(net: string, layer: number, width: number, sx: number, sy: number, ex: number, ey: number): string {
		const id = this.nid('L');
		this.lines.set(id, { net, layer, width, sx, sy, ex, ey });
		return id;
	}
	/** 直接放一根弧（不做重复弧吞并；端点拆线与 createArc 一致，见 splitLinesAtArcEnds） */
	seedArc(net: string, layer: number, width: number, sx: number, sy: number, ex: number, ey: number, deg: number): string {
		const id = this.nid('A');
		this.arcs.set(id, { net, layer, width, sx, sy, ex, ey, deg });
		this.splitLinesAtArcEnds(net, layer, sx, sy, ex, ey);
		return id;
	}

	/** 【e2e 观察】弧端点落在同网同层线的内部 → 该线在此点拆成两根（全换新 id） */
	private splitLinesAtArcEnds(net: string, layer: number, sx: number, sy: number, ex: number, ey: number): void {
		for (const q of [{ x: sx, y: sy }, { x: ex, y: ey }]) {
			for (const [xid, X] of [...this.lines]) {
				if (X.net !== net || X.layer !== layer) continue;
				const C = { x: X.sx, y: X.sy }, D = { x: X.ex, y: X.ey };
				if (d(q, C) < EPS || d(q, D) < EPS || segDist(q, C, D) >= EPS) continue;
				this.lines.delete(xid);
				this.lines.set(this.nid('L'), { ...X, ex: q.x, ey: q.y });
				this.lines.set(this.nid('L'), { ...X, sx: q.x, sy: q.y });
			}
		}
	}

	createLine(net: string, layer: number, sx: number, sy: number, ex: number, ey: number, width: number): string {
		this.checkFault();
		if (![sx, sy, ex, ey, width].every(Number.isFinite)) throw new Error('invalid number');
		if (!Number.isInteger(layer) || layer < 1 || layer > 100) throw new Error('invalid layer');
		const len = Math.hypot(ex - sx, ey - sy);
		if (len < 1e-9) throw new Error('zero length');
		const id = this.nid('L');
		if (len < 0.1) return id;   // 静默丢弃
		this.lines.set(id, { net, layer, width, sx, sy, ex, ey });
		this.normalize(id);
		return id;
	}

	createArc(net: string, layer: number, sx: number, sy: number, ex: number, ey: number, deg: number, width: number): string {
		this.checkFault();
		if (![sx, sy, ex, ey, deg, width].every(Number.isFinite)) throw new Error('invalid number');
		if (!Number.isInteger(layer) || layer < 1 || layer > 100) throw new Error('invalid layer');
		if (deg === 0 || Math.abs(deg) >= 360) throw new Error('invalid arc angle');
		if (Math.hypot(ex - sx, ey - sy) < 1e-9) throw new Error('zero chord');
		const id = this.nid('A');
		for (const a of this.arcs.values()) {
			if (a.net !== net || a.layer !== layer || Math.abs(a.width - width) > 1e-9) continue;
			const same = Math.abs(a.sx - sx) < EPS && Math.abs(a.sy - sy) < EPS && Math.abs(a.ex - ex) < EPS && Math.abs(a.ey - ey) < EPS && Math.abs(a.deg - deg) < 1e-6;
			const rev = Math.abs(a.sx - ex) < EPS && Math.abs(a.sy - ey) < EPS && Math.abs(a.ex - sx) < EPS && Math.abs(a.ey - sy) < EPS && Math.abs(a.deg + deg) < 1e-6;
			if (same || rev) return id;   // 新弧被吞
		}
		this.arcs.set(id, { net, layer, width, sx, sy, ex, ey, deg });
		this.splitLinesAtArcEnds(net, layer, sx, sy, ex, ey);
		return id;
	}

	deleteLines(ids: string[]): boolean {
		this.deletes++;
		for (const id of ids) this.lines.delete(id);
		return true;
	}
	deleteArcs(ids: string[]): boolean {
		this.deletes++;
		let ok = true;
		for (const id of ids) if (!this.arcs.delete(id)) ok = false;
		return ok;
	}

	listLines(net?: string): any[] {
		this.reads++;
		if (this.failReads) throw new Error('read failure');
		return [...this.lines.entries()].filter(([, l]) => net === undefined || l.net === net).map(([id, l]) => ({
			net: l.net, layer: l.layer, startX: l.sx, startY: l.sy, endX: l.ex, endY: l.ey, lineWidth: l.width, primitiveId: id,
		}));
	}
	listArcs(net?: string): any[] {
		this.reads++;
		if (this.failReads) throw new Error('read failure');
		return [...this.arcs.entries()].filter(([, a]) => net === undefined || a.net === net).map(([id, a]) => {
			const deg = r1(a.deg);
			return {
				net: a.net, layer: a.layer, startX: r1(a.sx), startY: r1(a.sy), endX: r1(a.ex), endY: r1(a.ey),
				arcAngle: Object.is(deg, -0) ? 0 : deg, lineWidth: a.width, primitiveId: id,
			};
		});
	}

	api(): WritebackApi {
		return {
			createLine: async (n, ly, sx, sy, ex, ey, w) => this.createLine(n, ly, sx, sy, ex, ey, w),
			createArc: async (n, ly, sx, sy, ex, ey, deg, w) => this.createArc(n, ly, sx, sy, ex, ey, deg, w),
			deleteLines: async (ids) => this.deleteLines(ids),
			deleteArcs: async (ids) => this.deleteArcs(ids),
			listLines: async (n) => this.listLines(n),
			listArcs: async (n) => this.listArcs(n),
		};
	}

	// ── 合并 / 拆分 ──

	/** 点 q 处（除 except 外）同网同层的其它图元端点/内部 + 同网过孔数 */
	private occupancy(net: string, layer: number, q: P, except: Set<string>): number {
		let n = 0;
		for (const [id, l] of this.lines) {
			if (except.has(id) || l.net !== net || l.layer !== layer) continue;
			if (segDist(q, { x: l.sx, y: l.sy }, { x: l.ex, y: l.ey }) < EPS) n++;
		}
		for (const [id, a] of this.arcs) {
			if (except.has(id) || a.net !== net || a.layer !== layer) continue;
			if (d(q, { x: a.sx, y: a.sy }) < EPS || d(q, { x: a.ex, y: a.ey }) < EPS) n++;
		}
		for (const v of this.vias) if (v.net === net && d(q, v) < EPS) n++;
		return n;
	}

	private normalize(id0: string): void {
		let id = id0;
		// 1. 合并（只针对新线 / 合并结果）
		for (let guard = 0; guard < 100; guard++) {
			const L = this.lines.get(id);
			if (!L) return;
			let merged = false;
			for (const [xid, X] of this.lines) {
				if (xid === id || X.net !== L.net || X.layer !== L.layer || Math.abs(X.width - L.width) > 1e-9) continue;
				const A = { x: L.sx, y: L.sy }, B = { x: L.ex, y: L.ey };
				const C = { x: X.sx, y: X.sy }, D = { x: X.ex, y: X.ey };
				const len = d(A, B);
				const ux = (B.x - A.x) / len, uy = (B.y - A.y) / len;
				const off = (q: P): number => Math.abs((q.x - A.x) * uy - (q.y - A.y) * ux);
				const t = (q: P): number => (q.x - A.x) * ux + (q.y - A.y) * uy;
				if (off(C) < EPS && off(D) < EPS) {
					// 共线
					const [c0, c1] = [Math.min(t(C), t(D)), Math.max(t(C), t(D))];
					if (c1 < -EPS || c0 > len + EPS) continue;          // 不相交
					const touchOnly = Math.abs(c1) < EPS || Math.abs(c0 - len) < EPS;
					if (touchOnly) {
						const V = Math.abs(c1) < EPS ? A : B;
						if (this.occupancy(L.net, L.layer, V, new Set([id, xid])) > 0) continue;
					}
					if (c0 <= EPS && c1 >= len - EPS) {               // 新线是子段（含重复）→ 被吞
						this.lines.delete(id);
						return;
					}
					if (this.overlapKeepsRemainder && !touchOnly) {
						// 部分重叠：已有线原样保留，新线只留下不重叠的部分（可能两侧各一段）
						this.lines.delete(id);
						for (const [a, b] of [[0, c0], [c1, len]] as Array<[number, number]>) {
							if (b - a <= EPS) continue;
							this.lines.set(this.nid('L'), { ...L, sx: A.x + ux * a, sy: A.y + uy * a, ex: A.x + ux * b, ey: A.y + uy * b });
						}
						return;
					}
					const lo = Math.min(0, c0), hi = Math.max(len, c1);
					const nl: FLine = { ...L, sx: A.x + ux * lo, sy: A.y + uy * lo, ex: A.x + ux * hi, ey: A.y + uy * hi };
					this.lines.delete(id); this.lines.delete(xid);
					id = this.nid('L');
					this.lines.set(id, nl);
					merged = true;
					break;
				}
				// 近共线：共端点、转角 < 1°
				for (const [V, P1, Q1] of [[A, B, null], [B, A, null]] as Array<[P, P, null]>) {
					void Q1;
					let far: P | null = null;
					if (d(C, V) < EPS) far = D; else if (d(D, V) < EPS) far = C;
					if (!far) continue;
					// 从 X 的远端 → V → L 的远端 P1
					const ax = V.x - far.x, ay = V.y - far.y, bx = P1.x - V.x, by = P1.y - V.y;
					const th = Math.abs(Math.atan2(ax * by - ay * bx, ax * bx + ay * by)) * 180 / Math.PI;
					if (!(th > 0 && th < 1)) continue;
					if (this.occupancy(L.net, L.layer, V, new Set([id, xid])) > 0) continue;
					const nl: FLine = { ...L, sx: far.x, sy: far.y, ex: P1.x, ey: P1.y };
					this.lines.delete(id); this.lines.delete(xid);
					id = this.nid('L');
					this.lines.set(id, nl);
					merged = true;
					break;
				}
				if (merged) break;
			}
			if (!merged) break;
		}
		// 2. 拆分：交叉 / T 接 / 穿过孔
		const L = this.lines.get(id);
		if (!L) return;
		const A = { x: L.sx, y: L.sy }, B = { x: L.ex, y: L.ey };
		const cutsL: number[] = [];
		for (const [xid, X] of [...this.lines]) {
			if (xid === id || X.net !== L.net || X.layer !== L.layer) continue;
			const C = { x: X.sx, y: X.sy }, D = { x: X.ex, y: X.ey };
			const hit = segIntersect(A, B, C, D);
			if (!hit) continue;
			if (hit.u > EPS / d(C, D) && hit.u < 1 - EPS / d(C, D)) {   // X 内部被碰到 → 拆 X
				const Pp = { x: C.x + (D.x - C.x) * hit.u, y: C.y + (D.y - C.y) * hit.u };
				this.lines.delete(xid);
				this.lines.set(this.nid('L'), { ...X, ex: Pp.x, ey: Pp.y });
				this.lines.set(this.nid('L'), { ...X, sx: Pp.x, sy: Pp.y });
			}
			if (hit.t > EPS / d(A, B) && hit.t < 1 - EPS / d(A, B)) cutsL.push(hit.t);
		}
		for (const v of this.vias) {
			if (v.net !== L.net) continue;
			const len = d(A, B);
			const t = ((v.x - A.x) * (B.x - A.x) + (v.y - A.y) * (B.y - A.y)) / (len * len);
			if (t * len > EPS && (1 - t) * len > EPS && segDist(v, A, B) < EPS) cutsL.push(t);
		}
		for (const a of this.arcs.values()) {
			if (a.net !== L.net || a.layer !== L.layer) continue;
			const len = d(A, B);
			for (const q of [{ x: a.sx, y: a.sy }, { x: a.ex, y: a.ey }]) {
				const t = ((q.x - A.x) * (B.x - A.x) + (q.y - A.y) * (B.y - A.y)) / (len * len);
				if (t * len > EPS && (1 - t) * len > EPS && segDist(q, A, B) < EPS) cutsL.push(t);
			}
		}
		if (cutsL.length) {
			const ts = [0, ...[...new Set(cutsL)].sort((a, b) => a - b), 1];
			this.lines.delete(id);
			for (let i = 0; i + 1 < ts.length; i++) {
				const p0 = { x: A.x + (B.x - A.x) * ts[i], y: A.y + (B.y - A.y) * ts[i] };
				const p1 = { x: A.x + (B.x - A.x) * ts[i + 1], y: A.y + (B.y - A.y) * ts[i + 1] };
				if (d(p0, p1) > EPS) this.lines.set(this.nid('L'), { ...L, sx: p0.x, sy: p0.y, ex: p1.x, ey: p1.y });
			}
		}
	}
}

function segDist(q: P, a: P, b: P): number {
	const dx = b.x - a.x, dy = b.y - a.y;
	const L2 = dx * dx + dy * dy;
	if (L2 < 1e-18) return d(q, a);
	const t = Math.max(0, Math.min(1, ((q.x - a.x) * dx + (q.y - a.y) * dy) / L2));
	return Math.hypot(q.x - a.x - dx * t, q.y - a.y - dy * t);
}

/** 两线段交点参数（含端点接触；共线返回 null） */
function segIntersect(A: P, B: P, C: P, D: P): { t: number; u: number } | null {
	const rx = B.x - A.x, ry = B.y - A.y, sx = D.x - C.x, sy = D.y - C.y;
	const den = rx * sy - ry * sx;
	if (Math.abs(den) < 1e-12) return null;
	const t = ((C.x - A.x) * sy - (C.y - A.y) * sx) / den;
	const u = ((C.x - A.x) * ry - (C.y - A.y) * rx) / den;
	const tl = EPS / Math.hypot(rx, ry), ul = EPS / Math.hypot(sx, sy);
	if (t < -tl || t > 1 + tl || u < -ul || u > 1 + ul) return null;
	return { t, u };
}
