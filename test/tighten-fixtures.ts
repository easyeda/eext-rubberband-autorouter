/**
 * 拉紧（tighten）离线测试夹具 + 解析解（taut 长度）预言机。
 * 板/导线/焊盘用 bench/tighten-boards.mjs 的构造器；本文件只做数据，不依赖 src/ 算法。
 * 单位 mil；y 轴方向无关。每个夹具都是一块小板：按「实际线宽」严格 DRC 干净（k 号夹具除外，它故意带一处既有违规）。
 */

// eslint-disable-next-line @typescript-eslint/no-explicit-any
type Any = any;

export interface Fixture {
	id: string;
	title: string;
	board: Any;
	nets: Any[];
	vias: Any[];
	/** 被拉紧导线的解析 taut 总长（mil）；缺省 = 不检查「够紧」 */
	taut?: number;
	/** taut 允许的相对超出（默认 3%） */
	tautTol?: number;
	/** 本夹具故意带既有违规（k） */
	violatesBefore?: boolean;
	/** 结果总长上限（mil）：用于「必须仍然经过某焊盘」这类没有解析 taut 的夹具 */
	maxLen?: number;
	/** 节点度数必须保持（T 分支点）：端点聚类半径 0.8mil */
	keepNodes?: Array<{ net: string; x: number; y: number; degree: number }>;
	/** 备注（输出用） */
	note?: string;
	/**
	 * 必经的己网圆焊盘（圆心 x,y，半径 r）：结果中心线必须有一点离圆心 ≤ r − width/2（整个线宽都压在铜上，真正的搭接），
	 * 而不只是「贴着铜皮边缘」
	 */
	padDepth?: { x: number; y: number; r: number; width: number };
}

// ─────────────────────────────────────────────────────────────
// taut 预言机：外扩障碍（圆/胶囊，半径 rho）下的最短路径，
// 用「外切多边形节点 + 可见图 + Dijkstra」数值求解，和被测算法完全独立。
// ─────────────────────────────────────────────────────────────
/** disc / capsule: axis = (x1,y1)-(x2,y2); rect: axis-aligned box (x1,y1)-(x2,y2) = min/max corners. rho = clearance offset */
export interface Obs { kind: 'disc' | 'capsule' | 'rect'; x1: number; y1: number; x2: number; y2: number; rho: number }
export const disc = (x: number, y: number, rho: number): Obs => ({ kind: 'disc', x1: x, y1: y, x2: x, y2: y, rho });
export const capsule = (x1: number, y1: number, x2: number, y2: number, rho: number): Obs => ({ kind: 'capsule', x1, y1, x2, y2, rho });
export const rectObs = (x0: number, y0: number, x1: number, y1: number, rho: number): Obs => ({ kind: 'rect', x1: x0, y1: y0, x2: x1, y2: y1, rho });
/** foreign ELLIPSE/OVAL pad (w x h at x,y) inflated by `extra` (= rule + width/2) */
export const ovalObs = (x: number, y: number, w: number, h: number, extra: number): Obs => {
	const r = Math.min(w, h) / 2 + extra, l = Math.abs(w - h) / 2;
	return w >= h ? capsule(x - l, y, x + l, y, r) : capsule(x, y - l, x, y + l, r);
};
/** foreign RECT pad inflated by `extra` */
export const boxObs = (x: number, y: number, w: number, h: number, extra: number): Obs => rectObs(x - w / 2, y - h / 2, x + w / 2, y + h / 2, extra);

function ptSeg(px: number, py: number, x1: number, y1: number, x2: number, y2: number): number {
	const dx = x2 - x1, dy = y2 - y1, l2 = dx * dx + dy * dy;
	let t = l2 > 0 ? ((px - x1) * dx + (py - y1) * dy) / l2 : 0;
	t = Math.max(0, Math.min(1, t));
	return Math.hypot(px - x1 - t * dx, py - y1 - t * dy);
}
function segSeg(a: number[], b: number[], c: number[], d: number[]): number {
	const o = (p: number[], q: number[], r: number[]) => (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]);
	const d1 = o(c, d, a), d2 = o(c, d, b), d3 = o(a, b, c), d4 = o(a, b, d);
	if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) return 0;
	return Math.min(ptSeg(a[0], a[1], c[0], c[1], d[0], d[1]), ptSeg(b[0], b[1], c[0], c[1], d[0], d[1]),
		ptSeg(c[0], c[1], a[0], a[1], b[0], b[1]), ptSeg(d[0], d[1], a[0], a[1], b[0], b[1]));
}
function rectPt(o: Obs, px: number, py: number): number {
	return Math.hypot(Math.max(o.x1 - px, 0, px - o.x2), Math.max(o.y1 - py, 0, py - o.y2));
}
function rectSeg(o: Obs, p: number[], q: number[]): number {
	if (rectPt(o, p[0], p[1]) === 0 || rectPt(o, q[0], q[1]) === 0) return 0;
	const c = [[o.x1, o.y1], [o.x2, o.y1], [o.x2, o.y2], [o.x1, o.y2]];
	let d = Math.min(rectPt(o, p[0], p[1]), rectPt(o, q[0], q[1]));
	for (let i = 0; i < 4; i++) {
		if (segSeg(p, q, c[i], c[(i + 1) % 4]) === 0) return 0;
		d = Math.min(d, ptSeg(c[i][0], c[i][1], p[0], p[1], q[0], q[1]));
	}
	return d;
}
const distPt = (o: Obs, p: number[]) => (o.kind === 'rect' ? rectPt(o, p[0], p[1]) : ptSeg(p[0], p[1], o.x1, o.y1, o.x2, o.y2));
const distSeg = (o: Obs, p: number[], q: number[]) => (o.kind === 'rect' ? rectSeg(o, p, q) : segSeg(p, q, [o.x1, o.y1], [o.x2, o.y2]));

/** 两点间绕开所有障碍（中心线须与障碍轴线保持 >= rho）的最短路径长度 */
export function tautLength(A: number[], B: number[], obs: Obs[], N = 90): number {
	const eps = 1e-6;
	const nodes: number[][] = [A, B];
	for (const o of obs) {
		const R = o.rho / Math.cos(Math.PI / N);
		const centers = o.kind === 'disc' ? [[o.x1, o.y1]]
			: o.kind === 'capsule' ? [[o.x1, o.y1], [o.x2, o.y2]]
				: [[o.x1, o.y1], [o.x2, o.y1], [o.x2, o.y2], [o.x1, o.y2]];
		for (const c of centers) {
			for (let i = 0; i < N; i++) {
				const a = (2 * Math.PI * i) / N;
				nodes.push([c[0] + R * Math.cos(a), c[1] + R * Math.sin(a)]);
			}
		}
	}
	const clearPt = (p: number[]) => obs.every(o => distPt(o, p) >= o.rho - eps);
	if (!clearPt(A) || !clearPt(B)) throw new Error('taut oracle: end point inside an inflated obstacle');
	const V = nodes.filter((p, i) => i < 2 || clearPt(p));
	const visible = (p: number[], q: number[]) => obs.every(o => distSeg(o, p, q) >= o.rho - eps);
	const n = V.length;
	const dist = new Array(n).fill(Infinity), done = new Array(n).fill(false);
	dist[0] = 0;
	for (let it = 0; it < n; it++) {
		let u = -1;
		for (let i = 0; i < n; i++) if (!done[i] && (u < 0 || dist[i] < dist[u])) u = i;
		if (u < 0 || dist[u] === Infinity) break;
		if (u === 1) break;
		done[u] = true;
		for (let v = 0; v < n; v++) {
			if (done[v]) continue;
			const w = Math.hypot(V[u][0] - V[v][0], V[u][1] - V[v][1]);
			if (dist[u] + w >= dist[v]) continue;
			if (visible(V[u], V[v])) dist[v] = dist[u] + w;
		}
	}
	return dist[1];
}

// ─────────────────────────────────────────────────────────────
// 夹具
// ─────────────────────────────────────────────────────────────
export function buildFixtures(B: Any): Fixture[] {
	const RULE = 6;   // trackToTrack / trackToPad of the default board DRC
	const out: Fixture[] = [];
	const rect = (board: Any, net: string, id: string, x: number, y: number, layer = 1) =>
		B.makePad(board, { x, y, net, id, layer, kind: 'smd', w: 32, h: 36 });
	const round = (board: Any, net: string, id: string, x: number, y: number, d: number, layer = 1) =>
		B.makePad(board, { x, y, net, id, layer, kind: 'smd', w: d, h: d, shape: 'ELLIPSE' });
	const oval = (board: Any, net: string, id: string, x: number, y: number, w: number, h: number, layer = 1) =>
		B.makePad(board, { x, y, net, id, layer, kind: 'smd', w, h, shape: 'ELLIPSE' });
	const box = (board: Any, net: string, id: string, x: number, y: number, w: number, h: number, layer = 1) =>
		B.makePad(board, { x, y, net, id, layer, kind: 'smd', w, h });
	const wall = (board: Any, net: string, layer: number, lw: number, pts: number[][]) =>
		board.statics.tracks.push({ id: `${net}-track`, net, layer, lw, pts: pts.map(([x, y]) => ({ x, y })) });
	const poly = (net: string, layer: number, width: number, pts: number[][]) => B.polyPrims(net, layer, width, pts);
	const arcPrim = (net: string, layer: number, width: number, cx: number, cy: number, r: number, a0: number, sweep: number) =>
		({ kind: 'arc', net, layer, cx, cy, r, startAngle: a0, sweep, width });
	const arcPt = (cx: number, cy: number, r: number, a: number) => [cx + r * Math.cos(a), cy + r * Math.sin(a)];

	// a1..a4: staircase beside a foreign round pad / the end of a foreign fixed track, 20 mil and 6 mil traces
	const stair = [[100, 300], [150, 300], [150, 340], [250, 340], [250, 380], [350, 380], [350, 340], [450, 340], [450, 300], [500, 300]];
	for (const w of [20, 6, 30]) {
		{
			const board = B.newBoard();
			rect(board, 'A', 'A.1', 100, 300); rect(board, 'A', 'A.2', 500, 300);
			round(board, 'F1', 'F1.1', 300, 295, 30);
			const rho = 15 + RULE + w / 2;
			out.push({
				id: `a-pad-${w}`, title: `${w}mil staircase beside a foreign round pad (default width stays 10)`,
				board, nets: [{ name: 'A', prims: poly('A', 1, w, stair) }], vias: [],
				taut: tautLength([100, 300], [500, 300], [disc(300, 295, rho)]),
			});
		}
		{
			const board = B.newBoard();
			rect(board, 'A', 'A.1', 100, 300); rect(board, 'A', 'A.2', 500, 300);
			wall(board, 'F1', 1, 10, [[300, 100], [300, 292]]);
			const rho = 5 + RULE + w / 2;
			out.push({
				id: `a-trk-${w}`, title: `${w}mil staircase beside the end of a foreign fixed track`,
				board, nets: [{ name: 'A', prims: poly('A', 1, w, stair) }], vias: [],
				taut: tautLength([100, 300], [500, 300], [capsule(300, 100, 300, 292, rho)]),
			});
		}
	}

	// b: detour that must straighten alongside a LONG foreign track (capsule 320 mil): the chord must not cut its far part
	{
		const board = B.newBoard();
		rect(board, 'A', 'A.1', 100, 260); rect(board, 'A', 'A.2', 500, 260);
		wall(board, 'F1', 1, 10, [[300, 100], [300, 420]]);
		out.push({
			id: 'b-long-wall', title: 'U detour around the end of a 320mil foreign track',
			board, nets: [{ name: 'A', prims: poly('A', 1, 10, [[100, 260], [100, 40], [500, 40], [500, 260]]) }], vias: [],
			taut: tautLength([100, 260], [500, 260], [capsule(300, 100, 300, 420, 5 + RULE + 5)]),
		});
	}

	// c: trace passes THROUGH an intermediate own-net pad (degree-2 node at the pad centre). A shortcut that skips the pad
	//    (the foreign pad on the P0-P2 line makes the skipping route look attractive) would disconnect P1.
	{
		const board = B.newBoard();
		rect(board, 'C', 'C.1', 100, 100); rect(board, 'C', 'C.2', 300, 300); rect(board, 'C', 'C.3', 500, 100);
		round(board, 'F1', 'F1.1', 300, 100, 30);
		const pts1 = [[100, 100], [180, 100], [180, 160], [240, 160], [240, 300], [300, 300]];
		const pts2 = [[300, 300], [360, 300], [360, 160], [420, 160], [420, 100], [500, 100]];
		out.push({
			id: 'c-through-pad', title: 'trace through an intermediate own-net pad (V shape, foreign pad on the shortcut)',
			board, nets: [{ name: 'C', prims: [...poly('C', 1, 10, pts1), ...poly('C', 1, 10, pts2)] }], vias: [],
			taut: 2 * Math.hypot(200, 200),
			note: 'taut = P0-P1-P2 chords (P1 is a waypoint)',
		});
	}

	// d1: mild corner (14 deg, legs 400) with a foreign pad 30 mil inside the corner: a max-radius fillet (R > 1000) would cut it
	{
		const board = B.newBoard({ W: 1200, H: 800 });
		rect(board, 'D', 'D.1', 100, 400); rect(board, 'D', 'D.2', 900, 300);
		round(board, 'F1', 'F1.1', 500, 370, 30);
		out.push({
			id: 'd-mild-corner', title: 'mild corner, big fillet radius possible, foreign pad inside the corner',
			board, nets: [{ name: 'D', prims: poly('D', 1, 10, [[100, 400], [500, 400], [900, 300]]) }], vias: [],
			taut: tautLength([100, 400], [900, 300], [disc(500, 370, 15 + RULE + 5)]),
		});
	}
	// d2: input arc R=200 (70 deg) between two pads, foreign pad between the arc and its chord
	{
		const board = B.newBoard();
		const cx = 500, cy = 300, R = 200, a0 = Math.PI / 2, sw = -70 * Math.PI / 180;
		const S = arcPt(cx, cy, R, a0), E = arcPt(cx, cy, R, a0 + sw);
		rect(board, 'D', 'D.1', S[0], S[1]); rect(board, 'D', 'D.2', E[0], E[1]);
		const am = a0 + sw / 2;
		const P = arcPt(cx, cy, R - 30, am);
		round(board, 'F1', 'F1.1', Math.round(P[0] * 100) / 100, Math.round(P[1] * 100) / 100, 30);
		out.push({
			id: 'd-big-arc', title: 'R=200 arc between two pads, foreign pad between the arc and its chord',
			board, nets: [{ name: 'D', prims: [arcPrim('D', 1, 10, cx, cy, R, a0, sw)] }], vias: [],
			taut: tautLength(S, E, [disc(Math.round(P[0] * 100) / 100, Math.round(P[1] * 100) / 100, 15 + RULE + 5)]),
		});
	}

	// e1: 90 deg corner in open space; e2: 90 deg corner whose chord is blocked by a big foreign pad close to one end
	{
		const board = B.newBoard();
		rect(board, 'E', 'E.1', 200, 500); rect(board, 'E', 'E.2', 600, 200);
		out.push({
			id: 'e-corner-open', title: '90 degree corner in open space',
			board, nets: [{ name: 'E', prims: poly('E', 1, 10, [[200, 500], [600, 500], [600, 200]]) }], vias: [],
			taut: Math.hypot(400, 300),
			note: 'taut = straight chord; an arc-rounded corner is accepted only if it is no longer than 3% above it',
		});
	}
	{
		const board = B.newBoard();
		rect(board, 'E', 'E.1', 150, 500); rect(board, 'E', 'E.2', 550, 200);
		round(board, 'F1', 'F1.1', 230, 440, 80);
		const rho = 40 + RULE + 5;
		out.push({
			id: 'e-corner-pad', title: '90 degree corner, chord blocked by a big foreign pad near one end',
			board, nets: [{ name: 'E', prims: poly('E', 1, 10, [[150, 500], [550, 500], [550, 200]]) }], vias: [],
			taut: tautLength([150, 500], [550, 200], [disc(230, 440, rho)]),
		});
	}

	// f: >120 deg hairpin fold in open space
	{
		const board = B.newBoard();
		rect(board, 'H', 'H.1', 300, 300); rect(board, 'H', 'H.2', 360, 340);
		out.push({
			id: 'f-hairpin', title: 'hairpin fold (175 deg turn) in open space',
			board, nets: [{ name: 'H', prims: poly('H', 1, 10, [[300, 300], [800, 300], [360, 340]]) }], vias: [],
			taut: Math.hypot(60, 40),
		});
	}

	// g: non-tangent line-arc joint (34 deg kink) followed by a tangent arc-line joint
	{
		const board = B.newBoard();
		const J = [350, 400], r = 100, th0 = -Math.PI / 2 + 0.6, sw = 1.4;
		const c = [J[0] - r * Math.cos(th0), J[1] - r * Math.sin(th0)];
		const E = arcPt(c[0], c[1], r, th0 + sw);
		const a1 = th0 + sw;
		const Bp = [E[0] + 150 * -Math.sin(a1), E[1] + 150 * Math.cos(a1)];
		rect(board, 'G', 'G.1', 150, 400); rect(board, 'G', 'G.2', Math.round(Bp[0] * 100) / 100, Math.round(Bp[1] * 100) / 100);
		const Bq = [Math.round(Bp[0] * 100) / 100, Math.round(Bp[1] * 100) / 100];
		out.push({
			id: 'g-line-arc', title: 'non-tangent line-arc joint (34 deg) + tangent arc-line joint',
			board,
			nets: [{ name: 'G', prims: [...poly('G', 1, 10, [[150, 400], J]), arcPrim('G', 1, 10, c[0], c[1], r, th0, sw), ...poly('G', 1, 10, [E, Bq])] }],
			vias: [], taut: Math.hypot(Bq[0] - 150, Bq[1] - 400),
		});
	}

	// h: T-branch net, 3 runs meeting at N=(500,400)
	{
		const board = B.newBoard();
		rect(board, 'T', 'T.1', 200, 200); rect(board, 'T', 'T.2', 800, 200); rect(board, 'T', 'T.3', 500, 700);
		const armA = [[200, 200], [260, 200], [260, 300], [380, 300], [380, 400], [500, 400]];
		const armB = [[800, 200], [740, 200], [740, 300], [620, 300], [620, 400], [500, 400]];
		const armC = [[500, 400], [500, 460], [560, 460], [560, 560], [500, 560], [500, 700]];
		out.push({
			id: 'h-tee', title: 'T-branch net (3 runs, node (500,400) must stay)',
			board, nets: [{ name: 'T', prims: [...poly('T', 1, 10, armA), ...poly('T', 1, 10, armB), ...poly('T', 1, 10, armC)] }], vias: [],
			taut: Math.hypot(300, 200) * 2 + 300,
			keepNodes: [{ net: 'T', x: 500, y: 400, degree: 3 }],
		});
	}

	// i: 45 deg zig-zag threading the 10 mil wide window between two foreign pads; the taut path wraps the lower pad at clearance
	{
		const board = B.newBoard();
		rect(board, 'Z', 'Z.1', 100, 360); rect(board, 'Z', 'Z.2', 700, 300);
		round(board, 'F1', 'F1.1', 400, 259, 50); round(board, 'F2', 'F2.1', 400, 341, 50);
		const rho = 25 + RULE + 5;
		out.push({
			id: 'i-zigzag-gap', title: '45 deg zig-zag through the gap between two foreign pads',
			board,
			nets: [{ name: 'Z', prims: poly('Z', 1, 10, [[100, 360], [160, 360], [220, 300], [460, 300], [500, 340], [560, 340], [600, 300], [700, 300]]) }], vias: [],
			taut: tautLength([100, 360], [700, 300], [disc(400, 259, rho), disc(400, 341, rho)]),
		});
	}

	// j: chain with a 1.5 mil sliver segment
	{
		const board = B.newBoard();
		rect(board, 'J', 'J.1', 100, 300); rect(board, 'J', 'J.2', 700, 380);
		const pts = [[100, 300], [250, 300], [250, 340], [400, 340], [400, 341.5], [550, 341.5], [550, 380], [700, 380]];
		out.push({
			id: 'j-sliver', title: 'staircase containing a 1.5 mil sliver segment',
			board, nets: [{ name: 'J', prims: poly('J', 1, 10, pts) }], vias: [], taut: Math.hypot(600, 80),
		});
	}

	// k: pre-existing violation (3 mil gap, actually overlapping the pad's clearance) must not get worse
	{
		const board = B.newBoard();
		rect(board, 'K', 'K.1', 100, 300); rect(board, 'K', 'K.2', 500, 300);
		round(board, 'F1', 'F1.1', 300, 322, 30);
		out.push({
			id: 'k-existing-violation', title: 'staircase with a pre-existing clearance violation (-2 mil) against a foreign pad',
			board, nets: [{ name: 'K', prims: poly('K', 1, 10, [[100, 300], [200, 300], [200, 340], [400, 340], [400, 300], [500, 300]]) }], vias: [],
			violatesBefore: true,
			taut: tautLength([100, 300], [500, 300], [disc(300, 322, 15 + RULE + 5)]),
		});
	}

	// l: net with a via: top part 15 mil, bottom part 10 mil, a foreign pad on each layer next to the chords
	{
		const board = B.newBoard();
		rect(board, 'V', 'V.1', 100, 200, 1); rect(board, 'V', 'V.2', 700, 600, 2);
		round(board, 'F1', 'F1.1', 300, 310, 30, 1);
		round(board, 'F2', 'F2.1', 550, 500, 30, 2);
		const top = [[100, 200], [160, 200], [160, 260], [260, 260], [260, 340], [340, 340], [340, 400], [400, 400]];
		const bot = [[400, 400], [460, 400], [460, 460], [600, 460], [600, 560], [660, 560], [660, 600], [700, 600]];
		const via = { net: 'V', x: 400, y: 400, dia: 24, hole: 12 };
		out.push({
			id: 'l-via-two-layers', title: 'net with a via: 15mil top part + 10mil bottom part, foreign pad on each layer',
			board, nets: [{ name: 'V', prims: [...poly('V', 1, 15, top), ...poly('V', 2, 10, bot)] }], vias: [via],
			taut: tautLength([100, 200], [400, 400], [disc(300, 310, 15 + RULE + 7.5)]) + tautLength([400, 400], [700, 600], [disc(550, 500, 15 + RULE + 5)]),
		});
	}
	// m1/m2/n1/n2: "stuck" runs found by fuzzing the pre-fix code (it adopts nothing, or leaves slivers and kinks):
	// random-looking staircases through a small cluster of foreign pads (round / oval / rect), default 10 mil width.
	const X = 5 + RULE;   // inflation of a foreign pad for a 10 mil track
	{
		const board = B.newBoard();
		rect(board, 'A', 'A.1', 300, 407); rect(board, 'A', 'A.2', 737, 290);
		oval(board, 'F0', 'F0.1', 437, 378, 43, 53); oval(board, 'F1', 'F1.1', 456, 363, 27, 59);
		out.push({
			id: 'm1-oval-cluster', title: 'wandering staircase around two overlapping foreign ovals (pre-fix: nothing adopted)',
			board, nets: [{ name: 'A', prims: poly('A', 1, 10, [[300, 407], [374, 407], [374, 369], [468, 262], [514, 262], [514, 430], [591, 232], [680, 261], [737, 261], [737, 290]]) }], vias: [],
			taut: tautLength([300, 407], [737, 290], [ovalObs(437, 378, 43, 53, X), ovalObs(456, 363, 27, 59, X)]),
		});
	}
	{
		const board = B.newBoard();
		rect(board, 'A', 'A.1', 300, 447); rect(board, 'A', 'A.2', 796, 436);
		oval(board, 'F0', 'F0.1', 574, 403, 21, 58); box(board, 'F1', 'F1.1', 453, 449, 27, 63); oval(board, 'F2', 'F2.1', 670, 453, 31, 37);
		out.push({
			id: 'm2-mixed-cluster', title: 'staircase through three foreign pads (oval, rect, oval) (pre-fix: nothing adopted)',
			board, nets: [{ name: 'A', prims: poly('A', 1, 10, [[300, 447], [411, 447], [411, 356], [564, 470], [670, 499], [796, 499], [796, 436]]) }], vias: [],
			taut: tautLength([300, 447], [796, 436], [ovalObs(574, 403, 21, 58, X), boxObs(453, 449, 27, 63, X), ovalObs(670, 453, 31, 37, X)]),
		});
	}
	{
		const board = B.newBoard();
		rect(board, 'A', 'A.1', 300, 390); rect(board, 'A', 'A.2', 688, 380);
		oval(board, 'F0', 'F0.1', 574, 387, 24, 36);
		out.push({
			id: 'n1-single-oval', title: 'staircase around ONE foreign oval pad on the chord (pre-fix: nothing adopted, 6 kinks left)',
			board, nets: [{ name: 'A', prims: poly('A', 1, 10, [[300, 390], [390, 390], [390, 373], [445, 295], [558, 302], [627, 302], [627, 426], [688, 426], [688, 380]]) }], vias: [],
			taut: tautLength([300, 390], [688, 380], [ovalObs(574, 387, 24, 36, X)]),
		});
	}
	{
		const board = B.newBoard();
		rect(board, 'A', 'A.1', 300, 362); rect(board, 'A', 'A.2', 641, 343);
		box(board, 'F0', 'F0.1', 499, 319, 62, 42);
		out.push({
			id: 'n2-rect-sliver', title: 'staircase below a foreign rect pad (pre-fix: sliver + 99 deg kink left)',
			board, nets: [{ name: 'A', prims: poly('A', 1, 10, [[300, 362], [325, 362], [325, 371], [413, 371], [413, 260], [440, 260], [440, 369], [492, 357], [519, 357], [519, 364], [565, 417], [641, 417], [641, 343]]) }], vias: [],
			taut: tautLength([300, 362], [641, 343], [boxObs(499, 319, 62, 42, X)]),
		});
	}
	// o1..o3: wide traces (20 / 20 / 15 mil, default 10) beside one foreign oval pad: the pre-fix code re-solves with a
	// 10 mil assumption and the result breaks the clearance of the real width
	const wideCase = (id: string, w: number, A: number[], Bp: number[], pad: number[], pts: number[][]) => {
		const board = B.newBoard();
		rect(board, 'A', 'A.1', A[0], A[1]); rect(board, 'A', 'A.2', Bp[0], Bp[1]);
		oval(board, 'F0', 'F0.1', pad[0], pad[1], pad[2], pad[3]);
		out.push({
			id, title: `${w}mil staircase beside a foreign oval pad`,
			board, nets: [{ name: 'A', prims: poly('A', 1, w, pts) }], vias: [],
			taut: tautLength(A, Bp, [ovalObs(pad[0], pad[1], pad[2], pad[3], RULE + w / 2)]),
		});
	};
	wideCase('o1-20mil-oval', 20, [300, 374], [619, 356], [378, 376, 52, 23], [[300, 374], [406.1, 437.5], [481.6, 454.1], [539.9, 454.1], [539.9, 449.8], [619, 449.8], [619, 356]]);
	wideCase('o2-20mil-oval-zigzag', 20, [300, 432], [708, 532], [516, 494, 43, 26], [[300, 432], [334, 445.6], [391.1, 445.6], [391.1, 354.1], [477, 479.2], [527.2, 397.6], [595.1, 397.6], [595.1, 539.3], [677.2, 421.3], [708, 421.3], [708, 532]]);
	wideCase('o3-15mil-oval', 15, [300, 385], [643, 301], [428, 384, 59, 30], [[300, 385], [357.9, 448.3], [487.3, 448.3], [487.3, 384.8], [555.2, 273.1], [643, 273.1], [643, 301]]);

	// p1..p3: a run goes STRAIGHT through an own-net pad (no vertex at the pad, both run ends far outside it). The pad hangs 160 mil
	// off the P1-P3 chord, so a taut path that does not treat the pad as an attachment would simply leave it (pad disconnected).
	// Any valid result touches the pad copper: the shortest such path is <= the V through the pad centre (2*hypot(200,160) = 512.3).
	const V_THROUGH = 2 * Math.hypot(200, 160);
	{
		const board = B.newBoard();
		rect(board, 'P', 'P.1', 100, 100); rect(board, 'P', 'P.3', 500, 100);
		B.makePad(board, { x: 300, y: 260, net: 'P', id: 'P.2', layer: 1, kind: 'smd', w: 32, h: 36 });
		out.push({
			id: 'p1-through-rect-pad', title: 'straight run through the middle of an own-net RECT pad (no vertex there), pad off the chord',
			board, nets: [{ name: 'P', prims: poly('P', 1, 10, [[100, 100], [100, 260], [500, 260], [500, 100]]) }], vias: [],
			maxLen: V_THROUGH + 3,
			note: 'result must keep P.2 connected; length <= V through the pad centre + 3',
		});
	}
	{
		const board = B.newBoard();
		rect(board, 'P', 'P.1', 100, 500); rect(board, 'P', 'P.3', 500, 500);
		const hex = Array.from({ length: 6 }, (_, k) => [300 + 26 * Math.cos(Math.PI / 12 + k * Math.PI / 3), 660 + 26 * Math.sin(Math.PI / 12 + k * Math.PI / 3)].map(v => Math.round(v * 100) / 100));
		B.makePad(board, { x: 300, y: 660, net: 'P', id: 'P.2', layer: 1, kind: 'smd', w: 52, h: 52, polygon: hex });
		out.push({
			id: 'p2-through-polygon-pad', title: 'straight run through the middle of an own-net POLYGON (hexagon) pad (no vertex there), pad off the chord',
			board, nets: [{ name: 'P', prims: poly('P', 1, 10, [[100, 500], [100, 660], [500, 660], [500, 500]]) }], vias: [],
			maxLen: V_THROUGH + 3,
			note: 'POLYGON pad: absolute vertices, radius 26; result must keep P.2 connected',
		});
	}
	{
		// p3: the run crosses the RECT pad 14 mil off its centre (still well inside the 36 mil tall copper): only an edge-ish part of the pad is hit
		const board = B.newBoard();
		rect(board, 'P', 'P.1', 100, 100); rect(board, 'P', 'P.3', 500, 100);
		B.makePad(board, { x: 300, y: 260, net: 'P', id: 'P.2', layer: 1, kind: 'smd', w: 32, h: 36 });
		out.push({
			id: 'p3-through-rect-pad-offcentre', title: 'straight run through an own-net RECT pad 14 mil off its centre (no vertex there)',
			board, nets: [{ name: 'P', prims: poly('P', 1, 10, [[100, 100], [100, 274], [500, 274], [500, 100]]) }], vias: [],
			maxLen: 2 * Math.hypot(200, 174) + 3,
		});
	}
	{
		// p4: the run crosses an own-net ROUND TH pad (r = 25.6, like net RST on the clock board) 12 mil off its centre on the way
		// between two other pads. Pre-fix the anchor rule only demanded "centre-line on/inside the copper edge" and the pad chamfer only
		// 0.5 mil inside it, so the tightened track slid to 0.5 mil inside the edge (25.1 mil from the centre): a mere edge touch.
		const board = B.newBoard();
		rect(board, 'P', 'P.1', 100, 100); rect(board, 'P', 'P.3', 500, 100);
		B.makePad(board, { x: 300, y: 260, net: 'P', id: 'P.2', layer: 12, kind: 'th', w: 51.2, h: 51.2 });
		out.push({
			id: 'p4-through-round-pad-offcentre', title: 'straight run through an own-net ROUND TH pad (r 25.6) 12 mil off its centre (no vertex there)',
			board, nets: [{ name: 'P', prims: poly('P', 1, 10, [[100, 100], [100, 248], [500, 248], [500, 100]]) }], vias: [],
			maxLen: 2 * Math.hypot(200, 148) + 3,
			padDepth: { x: 300, y: 260, r: 25.6, width: 10 },
			note: 'centre-line must stay >= w/2 (5 mil) inside the copper edge, i.e. <= 20.6 mil from the pad centre',
		});
	}
	return out;
}
