#!/usr/bin/env node
/**
 * tighten-metrics.mjs - INDEPENDENT strict metrics for "tighten existing routing" results.
 *
 * Plain ESM, no dependency on src/ algorithms (no imports at all). Units: mil. Y axis direction is
 * irrelevant (all geometry is orientation free).
 *
 * CONTEXT OBJECT (every function takes it)
 *   {
 *     drc:       { traceWidthMil, trackToTrackMil, trackToPadMil, trackToBoardMil, trackToFillMil, ... },
 *     allPads:   [{ x, y, net, id, layer (1|2|12), radius, rotation?, index }],
 *     padShapes: { 'x.xx,y.yy': { pad: ['RECT'|'ROUNDRECT'|'ELLIPSE'|'OVAL'|'POLYGON', w, h], hole, rotation } },
 *     vias:      [{ net, x, y, dia, hole }]                    routed (own) vias
 *     statics:   { outline: [{x,y}...] (closed or open ring), tracks: [{ id, net, layer, lw, pts }],
 *                  vias: [{ net, x, y, dia, hole }], fills: [{ net?, layer?, lw?, pts }] }   UNINFLATED copper
 *     nets:      [{ name, tracks: [{ prim, width, layer? }] }]    prim = {kind:'line'|'arc', ...}
 *   POLYGON pads (padShapes pad: ['POLYGON', [x0,y0,'L',x1,y1,...]], absolute vertices) are measured with their REAL
 *   polygon (point-in-polygon / point-to-polygon distance implemented here); unparsable -> the pad radius circle.
 *   }
 *
 * EXPORTS
 *   sharpCorners(ctx, {minDeg=30})      joints (cluster 0.8 mil, same net+layer, degree exactly 2) between
 *                                       line-line / line-arc / arc-arc whose turn angle >= minDeg; the arc
 *                                       tangent at the joint is used. { count, free, fixed, list }
 *                                       fixed = joint within 1 mil of an own-net pad centre, inside an own-net pad's
 *                                       copper, or inside an own-net via; free = everything else
 *   counts(ctx)                         { lines, arcs, lengthMil }
 *   strictDrc(ctx)                      edge-to-edge clearance using each track's ACTUAL width, by dense
 *                                       centre-line sampling (<= 0.25 mil step, arcs along the true arc) and
 *                                       exact point-to-shape distances. Foreign nets only. Items checked:
 *                                       trk-trk, trk-pad (real pad shape + drill), trk-via, trk-static
 *                                       (static foreign copper), trk-fill, trk-board.
 *                                       LOCATION based: -> { violations[{item,netA,netB,kind,gap,rule,x,y,events}],
 *                                       events[...], pairs[...] }. A violation is one (moving net, foreign ITEM,
 *                                       location) with gap < rule - 0.05 (never collapsed per net pair); `events` are
 *                                       the raw per-(prim,item,4 mil stretch) gaps < rule; `pairs` is an extra per-net-pair
 *                                       summary only.
 *   newViolations(before, after)        violations of `after` that are NEW: a violating after event is excused only by a
 *                                       before event of the SAME moving net and SAME foreign item (foreign tracks: same
 *                                       net+layer) within max(20 mil, 2*width) along the geometry whose gap is
 *                                       <= after-gap + 0.05. Reported per item/location with beforeGap.
 *   connectivity(ctx)                   { net -> { parts: [[padId...]...], floating } } pads connected through
 *                                       touching copper (centre-line inside pad copper / via, two same-net same-layer
 *                                       tracks whose centre-lines come within (wA+wB)/2 ANYWHERE - ends, interiors,
 *                                       crossings, arcs -, via-pad overlap)
 *   connectivityDiff(before, after)     [{ net, kind: 'split'|'merge'|'other'|'floating', before, after }]
 *   danglingEnds(ctx)                   track ends (degree 1) that touch neither an own pad, own via nor another
 *                                       own track: { count, list }
 *   widthsPreserved(netsBefore, netsAfter)  per-net set of distinct widths unchanged
 *   summarize(ctx)                      one object with the numbers above (no DRC lists)
 *   sameGeometry(netsA, netsB, tol)     per net, the prim sets are equal within tol (default 0.01 mil), canonical
 *                                       matching (direction independent, order independent): { equal, diffs[] }
 *   toNets(netPrimsArray, widthOf)      helper: [{name, prims}] -> [{name, tracks}]
 *
 * SELF TEST   node bench/tighten-metrics.mjs --selftest
 */
import { pathToFileURL } from 'node:url';

export const SAMPLE_STEP = 0.25;
export const GAP_TOL = 0.05;
const TAU = Math.PI * 2;
const padKey = (x, y) => `${x.toFixed(2)},${y.toFixed(2)}`;

// ─────────────────────────────────────────────────────────────
// geometry
// ─────────────────────────────────────────────────────────────
function segDist(px, py, x1, y1, x2, y2) {
	const dx = x2 - x1, dy = y2 - y1, l2 = dx * dx + dy * dy;
	let t = l2 > 0 ? ((px - x1) * dx + (py - y1) * dy) / l2 : 0;
	t = t < 0 ? 0 : t > 1 ? 1 : t;
	return Math.hypot(px - x1 - t * dx, py - y1 - t * dy);
}
const arcPt = (p, a) => [p.cx + p.r * Math.cos(a), p.cy + p.r * Math.sin(a)];
export function primEnds(p) {
	if (p.kind === 'line') return [[p.x1, p.y1], [p.x2, p.y2]];
	return [arcPt(p, p.startAngle), arcPt(p, p.startAngle + p.sweep)];
}
export function primLength(p) {
	if (p.kind === 'line') return Math.hypot(p.x2 - p.x1, p.y2 - p.y1);
	return Math.abs(p.sweep) * p.r;
}
function primDist(px, py, p) {
	if (p.kind === 'line') return segDist(px, py, p.x1, p.y1, p.x2, p.y2);
	const dx = px - p.cx, dy = py - p.cy, d = Math.hypot(dx, dy);
	const s = p.sweep < 0 ? -1 : 1, L = Math.abs(p.sweep);
	let t = (Math.atan2(dy, dx) - p.startAngle) * s;
	t = ((t % TAU) + TAU) % TAU;
	if (d > 1e-12 && t <= L) return Math.abs(d - p.r);
	const [a, b] = primEnds(p);
	return Math.min(Math.hypot(px - a[0], py - a[1]), Math.hypot(px - b[0], py - b[1]));
}
function primBBox(p) {
	if (p.kind === 'line') return [Math.min(p.x1, p.x2), Math.min(p.y1, p.y2), Math.max(p.x1, p.x2), Math.max(p.y1, p.y2)];
	const [a, b] = primEnds(p);
	let x0 = Math.min(a[0], b[0]), x1 = Math.max(a[0], b[0]), y0 = Math.min(a[1], b[1]), y1 = Math.max(a[1], b[1]);
	const s = p.sweep < 0 ? -1 : 1, L = Math.abs(p.sweep);
	for (let k = 0; k < 4; k++) {
		let t = (k * Math.PI / 2 - p.startAngle) * s;
		t = ((t % TAU) + TAU) % TAU;
		if (t <= L) {
			const [x, y] = arcPt(p, k * Math.PI / 2);
			x0 = Math.min(x0, x); x1 = Math.max(x1, x); y0 = Math.min(y0, y); y1 = Math.max(y1, y);
		}
	}
	return [x0, y0, x1, y1];
}
/** sample the centre-line at <= step spacing (both end points included) */
function samplePrim(p, step, cb) {
	if (p.kind === 'line') {
		const n = Math.max(1, Math.ceil(Math.hypot(p.x2 - p.x1, p.y2 - p.y1) / step));
		for (let i = 0; i <= n; i++) cb(p.x1 + (p.x2 - p.x1) * i / n, p.y1 + (p.y2 - p.y1) * i / n);
		return;
	}
	const n = Math.max(2, Math.ceil(Math.abs(p.sweep) * p.r / step));
	for (let i = 0; i <= n; i++) {
		const a = p.startAngle + p.sweep * i / n;
		cb(p.cx + p.r * Math.cos(a), p.cy + p.r * Math.sin(a));
	}
}
const bbGap = (a, b) => Math.max(a[0] - b[2], b[0] - a[2], a[1] - b[3], b[1] - a[3], 0);
const bbGrow = (b, d) => [b[0] - d, b[1] - d, b[2] + d, b[3] + d];

/** signed distance to a closed polygon ring (negative inside) */
function polySigned(pts, x, y) {
	let d = Infinity, inside = false;
	const n = pts.length;
	for (let i = 0, j = n - 1; i < n; j = i++) {
		const a = pts[j], b = pts[i];
		d = Math.min(d, segDist(x, y, a.x, a.y, b.x, b.y));
		if ((b.y > y) !== (a.y > y) && x < (a.x - b.x) * (y - b.y) / (a.y - b.y) + b.x) inside = !inside;
	}
	return inside ? -d : d;
}
function ring(pts) {
	if (pts.length > 1) {
		const f = pts[0], l = pts[pts.length - 1];
		if (Math.hypot(f.x - l.x, f.y - l.y) < 1e-6) return pts.slice(0, -1);
	}
	return pts;
}

// ─────────────────────────────────────────────────────────────
// pad shapes -> signed distance closures
// ─────────────────────────────────────────────────────────────
/** returns { dist(x,y) (negative inside copper, hole counted), ext (circumscribed radius) } */
/** POLYGON pad vertices: shape = ['POLYGON', [x0, y0, 'L', x1, y1, ...]] (absolute coordinates; command markers skipped) */
export function polygonVertices(shape) {
	const raw = Array.isArray(shape?.[1]) ? shape[1] : null;
	if (!raw) return null;
	const pts = [];
	for (let i = 0; i + 1 < raw.length; i++) {
		if (typeof raw[i] === 'number' && typeof raw[i + 1] === 'number') { pts.push({ x: raw[i], y: raw[i + 1] }); i++; }
	}
	if (pts.length >= 2 && Math.hypot(pts[0].x - pts[pts.length - 1].x, pts[0].y - pts[pts.length - 1].y) < 0.01) pts.pop();
	return pts.length >= 3 ? pts : null;
}
function buildPad(pad, sh) {
	const circle = (r) => ({ dist: (x, y) => Math.hypot(x - pad.x, y - pad.y) - r, ext: r });
	let base;
	const s = sh && Array.isArray(sh.pad) ? sh.pad : null;
	if (!s) base = circle(pad.radius ?? 0);
	else if (String(s[0] ?? '').toUpperCase() === 'POLYGON') {
		// the REAL polygon (signed distance, negative inside); unparsable -> pad radius circle
		const pts = polygonVertices(s);
		if (!pts) base = circle(pad.radius ?? 0);
		else base = { ext: Math.max(...pts.map(q => Math.hypot(q.x - pad.x, q.y - pad.y))), dist: (x, y) => polySigned(pts, x, y) };
	} else {
		let name = String(s[0] ?? '').toUpperCase(), w = s[1], h = s[2], rot = sh.rotation ?? 0;
		if (!(w > 0 && h > 0)) base = circle(pad.radius ?? 0);
		else {
			const a = -rot * Math.PI / 180, c = Math.cos(a), sn = Math.sin(a);
			if (name === 'RECT' || name === 'ROUNDRECT') {
				base = {
					ext: Math.hypot(w / 2, h / 2),
					dist: (x, y) => {
						const dx = x - pad.x, dy = y - pad.y;
						const lx = dx * c - dy * sn, ly = dx * sn + dy * c;
						const ox = Math.abs(lx) - w / 2, oy = Math.abs(ly) - h / 2;
						return ox > 0 && oy > 0 ? Math.hypot(ox, oy) : Math.max(ox, oy);
					},
				};
			} else {
				// ELLIPSE / OVAL (and any unknown name): capsule along the longer axis (round when w == h)
				base = {
					ext: Math.max(w, h) / 2,
					dist: (x, y) => {
						const dx = x - pad.x, dy = y - pad.y;
						const lx = dx * c - dy * sn, ly = dx * sn + dy * c;
						if (w >= h) return Math.hypot(Math.max(Math.abs(lx) - (w - h) / 2, 0), ly) - h / 2;
						return Math.hypot(lx, Math.max(Math.abs(ly) - (h - w) / 2, 0)) - w / 2;
					},
				};
			}
		}
	}
	const hole = sh && Array.isArray(sh.hole) ? sh.hole : null;
	if (hole && String(hole[0]).toUpperCase() === 'ROUND' && hole[1] > 0) {
		const hr = hole[1] / 2, d0 = base.dist;
		return { ext: Math.max(base.ext, hr), dist: (x, y) => Math.min(d0(x, y), Math.hypot(x - pad.x, y - pad.y) - hr) };
	}
	return base;
}
function padList(ctx) {
	const out = [];
	for (const pad of ctx.allPads ?? []) {
		const b = buildPad(pad, ctx.padShapes?.[padKey(pad.x, pad.y)]);
		out.push({ pad, dist: b.dist, ext: b.ext, id: pad.id ?? `${pad.x},${pad.y}` });
	}
	return out;
}

// ─────────────────────────────────────────────────────────────
// track items + spatial grid
// ─────────────────────────────────────────────────────────────
function trackItems(ctx) {
	const items = [];
	for (const n of ctx.nets ?? []) {
		for (const t of n.tracks ?? []) {
			const prim = t.prim;
			if (!prim) continue;
			items.push({
				net: n.name, prim, layer: t.layer ?? prim.layer ?? 1,
				width: t.width ?? ctx.drc?.traceWidthMil ?? 10, bb: primBBox(prim),
			});
		}
	}
	return items;
}
class Grid {
	constructor(cell) { this.cell = cell; this.m = new Map(); }
	add(it, bb) {
		const c = this.cell;
		for (let i = Math.floor(bb[0] / c); i <= Math.floor(bb[2] / c); i++) {
			for (let j = Math.floor(bb[1] / c); j <= Math.floor(bb[3] / c); j++) {
				const k = i + ',' + j;
				let a = this.m.get(k);
				if (!a) this.m.set(k, a = []);
				a.push(it);
			}
		}
	}
	query(bb) {
		const c = this.cell, out = new Set();
		for (let i = Math.floor(bb[0] / c); i <= Math.floor(bb[2] / c); i++) {
			for (let j = Math.floor(bb[1] / c); j <= Math.floor(bb[3] / c); j++) {
				const a = this.m.get(i + ',' + j);
				if (a) for (const it of a) out.add(it);
			}
		}
		return out;
	}
}

// ─────────────────────────────────────────────────────────────
// sharp corners
// ─────────────────────────────────────────────────────────────
function leaveDir(p, which) {
	if (p.kind === 'line') {
		const dx = which ? p.x1 - p.x2 : p.x2 - p.x1, dy = which ? p.y1 - p.y2 : p.y2 - p.y1;
		const l = Math.hypot(dx, dy);
		return l > 1e-12 ? [dx / l, dy / l] : null;
	}
	const s = p.sweep < 0 ? -1 : 1;
	const a = which ? p.startAngle + p.sweep : p.startAngle;
	const f = which ? -s : s;           // direction leaving the joint along the prim
	return [f * -Math.sin(a), f * Math.cos(a)];
}
/** cluster end points (tol) of one net+layer group; returns { nodes, degenerate:Set } */
function clusterEnds(items, tol) {
	const nodes = [];
	const grid = new Map();
	const find = (x, y) => {
		const gi = Math.round(x / tol), gj = Math.round(y / tol);
		for (let i = -1; i <= 1; i++) for (let j = -1; j <= 1; j++) {
			const a = grid.get((gi + i) + ',' + (gj + j));
			if (a) for (const n of a) if (Math.hypot(n.x - x, n.y - y) <= tol) return n;
		}
		const n = { x, y, inc: [] };
		nodes.push(n);
		const k = gi + ',' + gj;
		let a = grid.get(k);
		if (!a) grid.set(k, a = []);
		a.push(n);
		return n;
	};
	items.forEach((it, idx) => {
		const [e0, e1] = primEnds(it.prim);
		const n0 = find(e0[0], e0[1]), n1 = find(e1[0], e1[1]);
		if (n0 === n1 && it.prim.kind === 'line') return;   // sliver collapsed inside one cluster: ignore
		n0.inc.push({ idx, which: 0 });
		n1.inc.push({ idx, which: 1 });
	});
	return nodes;
}
function groupByNetLayer(items) {
	const m = new Map();
	for (const it of items) {
		const k = it.net + '\u0000' + it.layer;
		let a = m.get(k);
		if (!a) m.set(k, a = []);
		a.push(it);
	}
	return m;
}
export function sharpCorners(ctx, opts = {}) {
	const minDeg = opts.minDeg ?? 30, tol = opts.tol ?? 0.8, fixedTol = opts.fixedTol ?? 1.0;
	const items = trackItems(ctx);
	const padsByNet = new Map();
	for (const p of ctx.allPads ?? []) {
		let a = padsByNet.get(p.net);
		if (!a) padsByNet.set(p.net, a = []);
		a.push(p);
	}
	const viaList = [...(ctx.vias ?? []), ...(ctx.statics?.vias ?? [])];
	const allPadShapes = padList(ctx);
	const list = [];
	for (const group of groupByNetLayer(items).values()) {
		const net = group[0].net, layer = group[0].layer;
		const nodes = clusterEnds(group, tol);
		for (const n of nodes) {
			if (n.inc.length !== 2) continue;
			const [a, b] = n.inc;
			if (a.idx === b.idx) continue;
			const pa = group[a.idx].prim, pb = group[b.idx].prim;
			const da = leaveDir(pa, a.which), db = leaveDir(pb, b.which);
			if (!da || !db) continue;
			const between = Math.acos(Math.max(-1, Math.min(1, da[0] * db[0] + da[1] * db[1]))) * 180 / Math.PI;
			const turn = 180 - between;
			if (turn < minDeg) continue;
			const atPad = (padsByNet.get(net) ?? []).some(p => Math.hypot(p.x - n.x, p.y - n.y) <= fixedTol);
			const atVia = viaList.some(v => v.net === net && Math.hypot(v.x - n.x, v.y - n.y) <= v.dia / 2);
			// a kink hidden inside the copper of an own-net pad has no electrical / visual meaning: classed fixed too
			const inPad = allPadShapes.some(P => P.pad.net === net && (P.pad.layer === 12 || P.pad.layer === layer) && P.dist(n.x, n.y) <= 0.01);
			list.push({
				net, layer, x: n.x, y: n.y, angle: Math.round(turn * 10) / 10,
				type: `${pa.kind}-${pb.kind}`, atFixed: atPad || atVia || inPad, atPad, atVia, inPad,
			});
		}
	}
	const fixed = list.filter(c => c.atFixed).length;
	return { minDeg, count: list.length, free: list.length - fixed, fixed, list };
}
export function counts(ctx) {
	let lines = 0, arcs = 0, lengthMil = 0;
	for (const it of trackItems(ctx)) {
		if (it.prim.kind === 'line') lines++; else arcs++;
		lengthMil += primLength(it.prim);
	}
	return { lines, arcs, lengthMil };
}

// ─────────────────────────────────────────────────────────────
// strict DRC (location based)
// ─────────────────────────────────────────────────────────────
/** an EVENT = the worst gap of one moving track prim against one foreign ITEM inside one BIN_MIL stretch of its centre-line */
export const BIN_MIL = 4;
/** events of the same (moving net, item) closer than this belong to one reported violation */
export const CLUSTER_MIL = 10;
/** coverage radius floor: a before-violation this close (along the geometry) excuses an after-violation */
export const COVER_MIL = 20;

/** single-link clusters (distance <= CLUSTER_MIL) of events; one entry per cluster, worst event as representative */
function clusterEvents(events, extra) {
	const groups = new Map();
	for (const e of events) {
		const k = e.netA + '\u0000' + e.item;
		let a = groups.get(k);
		if (!a) groups.set(k, a = []);
		a.push(e);
	}
	const out = [];
	for (const g of groups.values()) {
		const d = new DSU(g.length);
		for (let i = 0; i < g.length; i++) for (let j = i + 1; j < g.length; j++) {
			if (Math.hypot(g[i].x - g[j].x, g[i].y - g[j].y) <= CLUSTER_MIL) d.u(i, j);
		}
		const best = new Map();
		const cnt = new Map();
		g.forEach((e, i) => {
			const r = d.f(i);
			cnt.set(r, (cnt.get(r) ?? 0) + 1);
			const b = best.get(r);
			if (!b || e.gap < b.gap) best.set(r, e);
		});
		for (const [r, e] of best) out.push({ ...e, events: cnt.get(r), ...(extra ? extra(e, g, d, r) : {}) });
	}
	return out.sort((a, b) => a.gap - b.gap);
}

/**
 * Edge-to-edge DRC with ACTUAL widths. Returns
 *   events      every (moving prim, foreign item, 4 mil stretch) with gap < rule (the raw material of the comparison)
 *   violations  one per (moving net, foreign ITEM, location): events with gap < rule - 0.05, single-link clustered;
 *               { item, netA, netB, kind, gap, rule, x, y, events }  (item = 'P|<padId>' / 'V|x,y' / 'S|<staticId>' /
 *               'T|<foreignNet>|<layer>' / 'F|<fill#>' / 'B')
 *   pairs       per net pair summary (extra only): [{ netA, netB, kind, gap (min), rule, violations }]
 * Foreign track pairs are measured once (netA < netB, samples on A's centre-line).
 */
export function strictDrc(ctx) {
	const drc = ctx.drc ?? {};
	const rTT = drc.trackToTrackMil ?? drc.clearanceMil ?? 6;
	const rTP = drc.trackToPadMil ?? drc.clearanceMil ?? 6;
	const rTB = drc.trackToBoardMil ?? drc.clearanceMil ?? 6;
	const rTF = drc.trackToFillMil ?? drc.clearanceMil ?? 6;
	const st = ctx.statics ?? {};
	const events = [];
	const items = trackItems(ctx);
	let maxW = 0;
	for (const it of items) maxW = Math.max(maxW, it.width);
	const tgrid = new Grid(64);
	for (const it of items) tgrid.add(it, it.bb);
	const pads = padList(ctx);
	let maxExt = 0;
	const pgrid = new Grid(128);
	for (const p of pads) {
		maxExt = Math.max(maxExt, p.ext);
		pgrid.add(p, [p.pad.x - p.ext, p.pad.y - p.ext, p.pad.x + p.ext, p.pad.y + p.ext]);
	}
	const viaList = [...(ctx.vias ?? []), ...(st.vias ?? [])];
	const outline = st.outline && st.outline.length >= 3 ? ring(st.outline) : null;
	let outBB = null;
	if (outline) {
		const xs = outline.map(p => p.x), ys = outline.map(p => p.y);
		const isRect = outline.length === 4 && outline.every((p, i) => {
			const q = outline[(i + 1) % 4];
			return Math.abs(p.x - q.x) < 1e-6 || Math.abs(p.y - q.y) < 1e-6;
		});
		if (isRect) outBB = [Math.min(...xs), Math.min(...ys), Math.max(...xs), Math.max(...ys)];
	}
	const fills = (st.fills ?? []).map(f => ({ f, pts: ring(f.pts), bb: (() => {
		const xs = f.pts.map(p => p.x), ys = f.pts.map(p => p.y);
		return [Math.min(...xs), Math.min(...ys), Math.max(...xs), Math.max(...ys)];
	})() }));
	const stTracks = (st.tracks ?? []).map(t => {
		const xs = t.pts.map(p => p.x), ys = t.pts.map(p => p.y);
		return { t, bb: [Math.min(...xs), Math.min(...ys), Math.max(...xs), Math.max(...ys)] };
	});

	const xs = [], ys = [];
	for (const A of items) {
		const hwA = A.width / 2;
		xs.length = 0; ys.length = 0;
		samplePrim(A.prim, SAMPLE_STEP, (x, y) => { xs.push(x); ys.push(y); });
		const nS = xs.length;
		const spacing = nS > 1 ? Math.max(primLength(A.prim) / (nS - 1), 1e-6) : 1;
		const binN = Math.max(1, Math.round(BIN_MIL / spacing));
		/** scan A's samples against one foreign item; one event per BIN_MIL stretch whose worst gap is < rule */
		const scan = (distFn, offset, rule, item, netB, kind) => {
			let best = Infinity, bx = 0, by = 0;
			for (let i = 0; i < nS; i++) {
				const g = distFn(xs[i], ys[i]) - offset;
				if (g < best) { best = g; bx = xs[i]; by = ys[i]; }
				if ((i + 1) % binN === 0 || i === nS - 1) {
					if (best < rule) events.push({ netA: A.net, netB, kind, item, gap: best, rule, x: bx, y: by, w: A.width });
					best = Infinity;
				}
			}
		};
		// 1) foreign tracks (each unordered net pair once: A.net < B.net)
		for (const B of tgrid.query(bbGrow(A.bb, hwA + maxW / 2 + rTT + 1))) {
			if (B.net === A.net || !(A.net < B.net) || B.layer !== A.layer) continue;
			if (bbGap(A.bb, B.bb) > hwA + B.width / 2 + rTT + 1) continue;
			scan((px, py) => primDist(px, py, B.prim), hwA + B.width / 2, rTT, `T|${B.net}|${B.layer}`, B.net, 'trk-trk');
		}
		// 2) pads
		for (const P of pgrid.query(bbGrow(A.bb, hwA + rTP + 1))) {
			const pad = P.pad;
			if (pad.net && pad.net === A.net) continue;
			if (pad.layer !== 12 && pad.layer !== undefined && pad.layer !== A.layer) continue;
			if (bbGap(A.bb, [pad.x, pad.y, pad.x, pad.y]) > P.ext + hwA + rTP + 1) continue;
			scan(P.dist, hwA, rTP, `P|${P.id}`, pad.net ?? '', 'trk-pad');
		}
		// 3) vias (all layers)
		for (const v of viaList) {
			if (v.net && v.net === A.net) continue;
			if (bbGap(A.bb, [v.x, v.y, v.x, v.y]) > v.dia / 2 + hwA + rTP + 1) continue;
			scan((px, py) => Math.hypot(px - v.x, py - v.y), v.dia / 2 + hwA, rTP, `V|${v.x.toFixed(1)},${v.y.toFixed(1)}`, v.net ?? '', 'trk-via');
		}
		// 4) static foreign tracks
		for (const S of stTracks) {
			const t = S.t;
			if (t.net && t.net === A.net) continue;
			if (t.layer !== undefined && t.layer !== A.layer) continue;
			if (bbGap(A.bb, S.bb) > (t.lw ?? 10) / 2 + hwA + rTT + 1) continue;
			scan((px, py) => {
				let d = Infinity;
				for (let i = 0; i + 1 < t.pts.length; i++) d = Math.min(d, segDist(px, py, t.pts[i].x, t.pts[i].y, t.pts[i + 1].x, t.pts[i + 1].y));
				return d;
			}, (t.lw ?? 10) / 2 + hwA, rTT, `S|${t.id ?? t.pts[0].x + ',' + t.pts[0].y}`, t.net ?? '', 'trk-static');
		}
		// 5) fills / keepouts
		fills.forEach((F, fi) => {
			if (F.f.net && F.f.net === A.net) return;
			if (F.f.layer !== undefined && F.f.layer !== A.layer) return;
			if (bbGap(A.bb, F.bb) > hwA + rTF + 1) return;
			scan((px, py) => polySigned(F.pts, px, py), hwA + (F.f.lw ?? 0) / 2, rTF, `F|${fi}`, F.f.net || 'FILL', 'trk-fill');
		});
		// 6) board outline
		if (outline) {
			const far = outBB && A.bb[0] - hwA - rTB - 1 > outBB[0] && A.bb[1] - hwA - rTB - 1 > outBB[1] &&
				A.bb[2] + hwA + rTB + 1 < outBB[2] && A.bb[3] + hwA + rTB + 1 < outBB[3];
			if (!far) scan((px, py) => -polySigned(outline, px, py), hwA, rTB, 'B', 'BOARD', 'trk-board');
		}
	}
	const violations = clusterEvents(events.filter(e => e.gap < e.rule - GAP_TOL));
	const pm = new Map();
	for (const v of violations) {
		const k = `${v.netA}|${v.netB}|${v.kind}`;
		const p = pm.get(k);
		if (!p) pm.set(k, { netA: v.netA, netB: v.netB, kind: v.kind, gap: v.gap, rule: v.rule, violations: 1 });
		else { p.violations++; if (v.gap < p.gap) p.gap = v.gap; }
	}
	return { violations, events, pairs: [...pm.values()].sort((a, b) => a.gap - b.gap) };
}
/**
 * Violations of `after` that are NEW, decided by LOCATION and foreign ITEM (never per net pair).
 * An after event e (moving net, foreign item, point P, gap g, rule r) with g < r - tol is EXCUSED iff `before` has an
 * event with the same moving net and the SAME item (for foreign tracks: same foreign net+layer, so a foreign track that
 * itself moved still matches), located within R = max(COVER_MIL, 2 * width) of P, whose gap is <= g + tol.
 * Every other violating event is new; they are clustered again (single link) so one new violation is reported per
 * (net, item, location): { ...worst event, events, beforeGap } with beforeGap = worst before-gap of that item within R
 * (null = none near). Tracks change geometry, so there is no primitive identity to compare - only item + place.
 */
export function newViolations(before, after, tol = GAP_TOL) {
	const byItem = new Map();
	for (const b of before.events) {
		const k = b.netA + '\u0000' + b.item;
		let a = byItem.get(k);
		if (!a) byItem.set(k, a = []);
		a.push(b);
	}
	const nearest = (e) => {
		const R = Math.max(COVER_MIL, 2 * (e.w ?? 0));
		let worst = null, excused = false;
		for (const b of byItem.get(e.netA + '\u0000' + e.item) ?? []) {
			if (Math.hypot(b.x - e.x, b.y - e.y) > R) continue;
			if (worst === null || b.gap < worst) worst = b.gap;
			if (b.gap <= e.gap + tol) excused = true;
		}
		return { worst, excused };
	};
	const fresh = [];
	for (const e of after.events) {
		if (!(e.gap < e.rule - tol)) continue;
		const n = nearest(e);
		if (!n.excused) fresh.push({ ...e, beforeGap: n.worst });
	}
	return clusterEvents(fresh, (rep, g, d, r) => {
		let bg = null;
		g.forEach((e, i) => { if (d.f(i) === r && e.beforeGap !== null && (bg === null || e.beforeGap < bg)) bg = e.beforeGap; });
		return { beforeGap: bg };
	});
}

// ─────────────────────────────────────────────────────────────
// connectivity
// ─────────────────────────────────────────────────────────────
class DSU {
	constructor(n) { this.p = Array.from({ length: n }, (_, i) => i); }
	f(i) { while (this.p[i] !== i) { this.p[i] = this.p[this.p[i]]; i = this.p[i]; } return i; }
	u(a, b) { a = this.f(a); b = this.f(b); if (a !== b) this.p[a] = b; }
}
/** min centre-line distance of two prims (exact for crossing lines, dense sampling of both otherwise); early exit <= stop */
function primPrimDist(P, Q, stop) {
	if (P.kind === 'line' && Q.kind === 'line') {
		const o = (ax, ay, bx, by, cx, cy) => (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
		const d1 = o(Q.x1, Q.y1, Q.x2, Q.y2, P.x1, P.y1), d2 = o(Q.x1, Q.y1, Q.x2, Q.y2, P.x2, P.y2);
		const d3 = o(P.x1, P.y1, P.x2, P.y2, Q.x1, Q.y1), d4 = o(P.x1, P.y1, P.x2, P.y2, Q.x2, Q.y2);
		if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) return 0;
	}
	let best = Infinity;
	for (const [S, T] of [[P, Q], [Q, P]]) {
		samplePrim(S, SAMPLE_STEP, (x, y) => { if (best > stop) best = Math.min(best, primDist(x, y, T)); });
		if (best <= stop) return best;
	}
	return best;
}
export function connectivity(ctx) {
	const items = trackItems(ctx);
	const pads = padList(ctx);
	const viaAll = [...(ctx.vias ?? []), ...(ctx.statics?.vias ?? [])];
	const names = new Set(ctx.nets.map(n => n.name));
	for (const p of pads) if (p.pad.net) names.add(p.pad.net);
	const res = {};
	for (const net of names) {
		const T = items.filter(t => t.net === net);
		const P = pads.filter(p => p.pad.net === net);
		const V = viaAll.filter(v => v.net === net);
		if (!T.length && P.length < 1) continue;
		const nT = T.length, nP = P.length;
		const d = new DSU(nT + nP + V.length);
		const iT = (i) => i, iP = (i) => nT + i, iV = (i) => nT + nP + i;
		// track - pad
		for (let a = 0; a < nT; a++) for (let b = 0; b < nP; b++) {
			const t = T[a], p = P[b];
			if (p.pad.layer !== 12 && p.pad.layer !== undefined && p.pad.layer !== t.layer) continue;
			if (bbGap(t.bb, [p.pad.x, p.pad.y, p.pad.x, p.pad.y]) > p.ext + 0.5) continue;
			let hit = false;
			samplePrim(t.prim, SAMPLE_STEP, (x, y) => { if (!hit && p.dist(x, y) <= 0.01) hit = true; });
			if (hit) d.u(iT(a), iP(b));
		}
		// track - via
		for (let a = 0; a < nT; a++) for (let b = 0; b < V.length; b++) {
			if (primDist(V[b].x, V[b].y, T[a].prim) <= V[b].dia / 2 + 0.01) d.u(iT(a), iV(b));
		}
		// via - pad, via - via
		for (let b = 0; b < V.length; b++) {
			for (let c = 0; c < nP; c++) if (P[c].dist(V[b].x, V[b].y) <= V[b].dia / 2 + 0.01) d.u(iV(b), iP(c));
			for (let c = b + 1; c < V.length; c++) if (Math.hypot(V[b].x - V[c].x, V[b].y - V[c].y) <= (V[b].dia + V[c].dia) / 2) d.u(iV(b), iV(c));
		}
		// track - track: the copper overlaps anywhere (centre-line distance <= (wA+wB)/2), interiors / crossings included
		for (let a = 0; a < nT; a++) for (let b = a + 1; b < nT; b++) {
			const A = T[a], B = T[b];
			if (A.layer !== B.layer) continue;
			const reach = (A.width + B.width) / 2 + 0.01;
			if (bbGap(A.bb, B.bb) > reach) continue;
			if (primPrimDist(A.prim, B.prim, reach) <= reach) d.u(iT(a), iT(b));
		}
		const comp = new Map();
		const touch = (root, kind, v) => {
			let c = comp.get(root);
			if (!c) comp.set(root, c = { pads: [], other: 0 });
			if (kind === 'pad') c.pads.push(v); else c.other++;
		};
		for (let a = 0; a < nT; a++) touch(d.f(iT(a)), 't');
		for (let b = 0; b < nP; b++) touch(d.f(iP(b)), 'pad', P[b].id);
		for (let b = 0; b < V.length; b++) touch(d.f(iV(b)), 'v');
		const parts = [], floating = [];
		for (const c of comp.values()) {
			if (c.pads.length) parts.push(c.pads.slice().sort());
			else floating.push(c);
		}
		parts.sort((x, y) => (x[0] < y[0] ? -1 : x[0] > y[0] ? 1 : 0));
		res[net] = { parts, floating: floating.length };
	}
	return res;
}
export function connectivityDiff(before, after) {
	const out = [];
	for (const net of new Set([...Object.keys(before), ...Object.keys(after)])) {
		const b = before[net], a = after[net];
		if (!b || !a) { out.push({ net, kind: 'other', before: b, after: a }); continue; }
		const sb = JSON.stringify(b.parts), sa = JSON.stringify(a.parts);
		if (sb !== sa) {
			const kind = a.parts.length > b.parts.length ? 'split' : a.parts.length < b.parts.length ? 'merge' : 'other';
			out.push({ net, kind, before: b.parts, after: a.parts });
		} else if (a.floating > b.floating) out.push({ net, kind: 'floating', before: b.floating, after: a.floating });
	}
	return out;
}
export function danglingEnds(ctx, opts = {}) {
	const tol = opts.tol ?? 0.8;
	const items = trackItems(ctx);
	const pads = padList(ctx);
	const viaAll = [...(ctx.vias ?? []), ...(ctx.statics?.vias ?? [])];
	const list = [];
	for (const group of groupByNetLayer(items).values()) {
		const net = group[0].net, layer = group[0].layer;
		for (const n of clusterEnds(group, tol)) {
			if (n.inc.length !== 1) continue;
			const it = group[n.inc[0].idx];
			if (pads.some(p => p.pad.net === net && (p.pad.layer === 12 || p.pad.layer === undefined || p.pad.layer === layer) && p.dist(n.x, n.y) <= 0.01)) continue;
			if (viaAll.some(v => v.net === net && Math.hypot(v.x - n.x, v.y - n.y) <= v.dia / 2 + 0.01)) continue;
			if (group.some((o, i) => i !== n.inc[0].idx && primDist(n.x, n.y, o.prim) <= (o.width + it.width) / 2 + 0.01)) continue;
			list.push({ net, layer, x: n.x, y: n.y });
		}
	}
	return { count: list.length, list };
}
export function widthsPreserved(netsBefore, netsAfter) {
	const wset = (n) => [...new Set((n?.tracks ?? []).map(t => Math.round((t.width ?? 0) * 1000) / 1000))].sort((a, b) => a - b);
	const am = new Map(netsAfter.map(n => [n.name, n]));
	const bad = [];
	for (const nb of netsBefore) {
		const a = wset(nb), b = wset(am.get(nb.name));
		if (JSON.stringify(a) !== JSON.stringify(b)) bad.push({ net: nb.name, before: a, after: b });
	}
	return { ok: bad.length === 0, bad };
}
export function summarize(ctx, opts = {}) {
	const c = counts(ctx), s = sharpCorners(ctx, opts);
	const d = strictDrc(ctx);
	return {
		lines: c.lines, arcs: c.arcs, lengthMil: c.lengthMil,
		sharp: { count: s.count, free: s.free, fixed: s.fixed },
		drcViolations: d.violations.length,
		dangling: danglingEnds(ctx).count,
	};
}
/** two prims are the same copper centre-line within tol (direction independent) */
function primSame(a, b, tol) {
	if (a.kind !== b.kind) return false;
	const [a0, a1] = primEnds(a), [b0, b1] = primEnds(b);
	const d = (p, q) => Math.hypot(p[0] - q[0], p[1] - q[1]);
	const ends = (d(a0, b0) <= tol && d(a1, b1) <= tol) || (d(a0, b1) <= tol && d(a1, b0) <= tol);
	if (!ends) return false;
	if (a.kind === 'line') return true;
	return Math.hypot(a.cx - b.cx, a.cy - b.cy) <= tol && Math.abs(a.r - b.r) <= tol &&
		Math.abs(Math.abs(a.sweep) - Math.abs(b.sweep)) * a.r <= tol && (a.layer ?? 1) === (b.layer ?? 1);
}
export function sameGeometry(netsA, netsB, tol = 0.01) {
	const diffs = [];
	const mb = new Map(netsB.map(n => [n.name, n]));
	for (const na of netsA) {
		const nb = mb.get(na.name);
		const A = na.prims ?? na.tracks?.map(t => t.prim) ?? [], B = (nb?.prims ?? nb?.tracks?.map(t => t.prim) ?? []).slice();
		const used = new Array(B.length).fill(false);
		let unmatched = 0;
		for (const p of A) {
			const j = B.findIndex((q, i) => !used[i] && (p.layer ?? 1) === (q.layer ?? 1) && primSame(p, q, tol));
			if (j < 0) unmatched++; else used[j] = true;
		}
		const extra = used.filter(u => !u).length;
		if (unmatched || extra) diffs.push({ net: na.name, unmatched, extra, prims: [A.length, B.length] });
	}
	for (const nb of netsB) if (!netsA.some(n => n.name === nb.name) && (nb.prims?.length ?? 0)) diffs.push({ net: nb.name, unmatched: 0, extra: nb.prims.length, prims: [0, nb.prims.length] });
	return { equal: diffs.length === 0, diffs };
}
export function toNets(netPrims, widthOf) {
	return netPrims.map(n => ({
		name: n.name,
		tracks: n.prims.map(p => ({ prim: p, layer: p.layer ?? 1, width: typeof widthOf === 'function' ? widthOf(n, p) : (p.width ?? widthOf) })),
	}));
}

// ─────────────────────────────────────────────────────────────
// self test
// ─────────────────────────────────────────────────────────────
function selftest() {
	let fail = 0;
	const ok = (cond, msg) => { console.log(`${cond ? 'ok  ' : 'FAIL'} ${msg}`); if (!cond) fail++; };
	const drc = { traceWidthMil: 10, clearanceMil: 6, trackToTrackMil: 6, trackToPadMil: 6, trackToBoardMil: 12, trackToFillMil: 10 };
	const L = (net, x1, y1, x2, y2, layer = 1) => ({ kind: 'line', net, layer, x1, y1, x2, y2 });
	const ctxOf = (nets, extra = {}) => ({ drc, allPads: [], padShapes: {}, vias: [], statics: {}, nets, ...extra });
	const net = (name, width, ...prims) => ({ name, tracks: prims.map(prim => ({ prim, width, layer: prim.layer })) });

	// 1) two parallel 20 mil tracks 12 mil apart centre-to-centre: edge gap = 12 - 20 = -8
	{
		const r = strictDrc(ctxOf([net('A', 20, L('A', 0, 0, 100, 0)), net('B', 20, L('B', 0, 12, 100, 12))]));
		ok(r.violations.length === 1 && Math.abs(r.violations[0].gap + 8) < 0.05, `parallel 20mil tracks 12 apart -> 1 violation, gap -8 (got ${r.violations.length}, gap ${r.violations[0]?.gap?.toFixed(3)})`);
		// the lenient centre-line audit (distance 12 >= 6) would have passed it
	}
	// 1b) 30 centre-to-centre: gap 10 -> clean ; 26.0 -> gap 6 -> clean; 25.9 -> violation
	{
		const mk = (d) => strictDrc(ctxOf([net('A', 20, L('A', 0, 0, 100, 0)), net('B', 10, L('B', 0, d, 100, d))])).violations.length;
		ok(mk(30) === 0 && mk(21) === 0 && mk(20.5) === 1, `20mil vs 10mil tracks: clean at 30 and 21 centre distance (gap 6), violation at 20.5 (got ${mk(30)},${mk(21)},${mk(20.5)})`);
	}
	// 2) big arc (R=200) cutting a foreign RECT pad corner -> violation ; moving the pad away -> clean
	{
		const arc = { kind: 'arc', net: 'A', layer: 1, cx: 0, cy: 200, r: 200, startAngle: -Math.PI / 2 - 0.5, sweep: 1.0 };   // bottom of the arc touches (0,0)
		const pad = (x, y) => ({ x, y, net: 'B', id: 'B.1', layer: 1, radius: 14, index: 0 });
		const shapes = (x, y) => ({ [padKey(x, y)]: { pad: ['RECT', 20, 20], hole: null, rotation: 0 } });
		const bad = strictDrc(ctxOf([net('A', 10, arc)], { allPads: [pad(8, 0)], padShapes: shapes(8, 0) }));
		const good = strictDrc(ctxOf([net('A', 10, arc)], { allPads: [pad(8, -30)], padShapes: shapes(8, -30) }));
		ok(bad.violations.length === 1 && bad.violations[0].kind === 'trk-pad', `R=200 arc through a pad corner -> trk-pad violation (gap ${bad.violations[0]?.gap?.toFixed(2)})`);
		ok(good.violations.length === 0, `same arc, pad 30 away -> clean (${good.violations.length} violations)`);
	}
	// 2b) track passing the pad corner diagonally at exactly 6 + w/2 from the corner: clean; 1 mil closer: violation
	{
		const shapes = { [padKey(0, 0)]: { pad: ['RECT', 20, 20], hole: null, rotation: 0 } };
		const pad = { x: 0, y: 0, net: 'B', id: 'B.1', layer: 1, radius: 14.2, index: 0 };
		const diag = (d) => {   // line x + y = c with distance d from the corner (10,10): c = 20 + d*sqrt2
			const c = 20 + d * Math.SQRT2;
			return L('A', c + 50, -50, c - 50, 50);
		};
		const n = (d) => strictDrc(ctxOf([net('A', 10, diag(d))], { allPads: [pad], padShapes: shapes })).violations.length;
		ok(n(11.1) === 0 && n(10.5) === 1, `diagonal pass at corner distance 11.1 (6+5+0.1) clean / 10.5 violation (got ${n(11.1)},${n(10.5)})`);
	}
	// 3) clean case + connectivity + sharp corners
	{
		const a = net('A', 10, L('A', 0, 0, 100, 0), L('A', 100, 0, 100, 100));
		const b = net('B', 10, L('B', 0, 60, 100, 60));
		const pads = [
			{ x: 0, y: 0, net: 'A', id: 'A.1', layer: 1, radius: 16, index: 0 },
			{ x: 100, y: 100, net: 'A', id: 'A.2', layer: 1, radius: 16, index: 1 },
		];
		const shapes = {};
		for (const p of pads) shapes[padKey(p.x, p.y)] = { pad: ['RECT', 20, 20], hole: null, rotation: 0 };
		const ctx = ctxOf([a, b], { allPads: pads, padShapes: shapes });
		// net B crosses nothing of A? B at y=60 runs x 0..100 and A's vertical at x=100 from y 0..100 -> they touch! make it clean:
		ctx.nets[1].tracks[0].prim = L('B', 0, 60, 60, 60);
		const d = strictDrc(ctx);
		ok(d.violations.length === 0, `clean case -> no violation (got ${d.violations.length})`);
		const sc = sharpCorners(ctx);
		ok(sc.count === 1 && sc.free === 1 && Math.abs(sc.list[0].angle - 90) < 0.5, `L track: 1 free sharp corner of 90 deg (got ${sc.count} free ${sc.free} angle ${sc.list[0]?.angle})`);
		const cn = connectivity(ctx);
		ok(JSON.stringify(cn.A.parts) === '[["A.1","A.2"]]', `A.1/A.2 connected through the L track (got ${JSON.stringify(cn.A.parts)})`);
		const whole = ctxOf([a], { allPads: pads, padShapes: shapes });
		const broken = ctxOf([net('A', 10, L('A', 0, 0, 100, 0), L('A', 100, 30, 100, 100))], { allPads: pads, padShapes: shapes });
		const diff = connectivityDiff(connectivity(whole), connectivity(broken));
		ok(diff.length === 1 && diff[0].kind === 'split', `gap in the track splits the net (got ${JSON.stringify(diff.map(x => x.kind))})`);
		ok(danglingEnds(whole).count === 0 && danglingEnds(broken).count === 2, `dangling ends 0 vs 2 (got ${danglingEnds(whole).count}, ${danglingEnds(broken).count})`);
		const atPad = sharpCorners(ctxOf([net('A', 10, L('A', 0, 0, 100, 0), L('A', 100, 0, 100, 100))], { allPads: [{ x: 100, y: 0, net: 'A', id: 'A.m', layer: 1, radius: 16, index: 0 }] }));
		ok(atPad.count === 1 && atPad.fixed === 1, `corner on an own-net pad centre is classed fixed (fixed ${atPad.fixed})`);
	}
	// 4) tangent line-arc joint is not sharp, non-tangent is; compare helper
	{
		const arcTan = { kind: 'arc', net: 'A', layer: 1, cx: 100, cy: 50, r: 50, startAngle: -Math.PI / 2, sweep: Math.PI / 2 };   // starts (100,0) heading +x ... ends (150,50)
		const t1 = sharpCorners(ctxOf([net('A', 10, L('A', 0, 0, 100, 0), arcTan)]));
		const th0 = -Math.PI / 2 + 0.6;   // start angle whose tangent is 34.4 deg off +x
		const arcBad = { kind: 'arc', net: 'A', layer: 1, cx: 100 - 50 * Math.cos(th0), cy: -50 * Math.sin(th0), r: 50, startAngle: th0, sweep: Math.PI / 2 };
		const t2 = sharpCorners(ctxOf([net('A', 10, L('A', 0, 0, 100, 0), arcBad)]));
		ok(t1.count === 0, `tangent line-arc joint: no corner (got ${t1.count})`);
		ok(t2.count === 1 && Math.abs(t2.list[0].angle - 34.4) < 0.6, `non-tangent line-arc joint: 34.4 deg corner (got ${t2.list[0]?.angle})`);
		const stat = (d) => ctxOf([net('A', 10, L('A', 0, 0, 100, 0))], { statics: { tracks: [{ id: 's', net: 'Z', layer: 1, lw: 10, pts: [{ x: 0, y: d }, { x: 100, y: d }] }] } });
		const g = (d) => strictDrc(stat(d)).violations.length;
		ok(g(19) === 0 && g(16) === 0 && g(15) === 1, `static foreign 10mil track: centre distance 19 and 16 (edge gap 6) clean, 15 violation (got ${g(19)},${g(16)},${g(15)})`);
	}
	{
		// before-gap awareness: pre-existing 4 mil gap; after 4.0 -> not new; after 3.5 -> new; clean before, 5.5 after -> new
		const mk = (d) => strictDrc(ctxOf([net('A', 10, L('A', 0, 0, 100, 0))], { statics: { tracks: [{ id: 's', net: 'Z', layer: 1, lw: 10, pts: [{ x: 0, y: d }, { x: 100, y: d }] }] } }));
		const before = mk(14), a1 = mk(14), a2 = mk(13.5), a3 = mk(16);
		const nv = (x) => newViolations(before, x).length;
		ok(nv(a1) === 0 && nv(a2) === 1 && nv(a3) === 0, `newViolations: pre-existing gap 4 -> same 0 new, worse 1 new, repaired 0 new (got ${nv(a1)},${nv(a2)},${nv(a3)})`);
		const cleanBefore = mk(40);
		ok(newViolations(cleanBefore, mk(15.5)).length === 1, 'newViolations: clean before, 5.5 gap after -> new');
	}
	{
		// LOCATION based comparison. Net A: fixed 600 mil line at y=0 (w10). Net B (w10) is the moving net; rule 6.
		// centre distance d  ->  edge gap d - 10.  The LEGACY rule (min gap per net pair) is evaluated next to it.
		const A = L('A', 0, 0, 600, 0);
		const poly = (name, pts) => pts.slice(1).map((q, i) => L(name, pts[i][0], pts[i][1], q[0], q[1]));
		const run = (pts) => strictDrc(ctxOf([net('A', 10, A), net('B', 10, ...poly('B', pts))]));
		const pairMin = (r) => { const m = new Map(); for (const e of r.events) { const k = e.netA + '|' + e.netB; m.set(k, Math.min(m.get(k) ?? Infinity, e.gap)); } return m; };
		const legacyNew = (b, a) => { const mb = pairMin(b); let n = 0; for (const [k, g] of pairMin(a)) if (g < Math.min(6, mb.get(k) ?? Infinity) - GAP_TOL) n++; return n; };
		const cnt = (b, a) => newViolations(b, a).length;
		const tail = [[250, 100], [350, 100]];
		// (a) existing 4 mil gap at x<=150, afterwards ALSO a 1 mil gap far away (x 450..560)
		const b4 = run([[0, 14], [150, 14], [250, 100]]);
		const aNew1 = run([[0, 14], [150, 14], ...tail, [450, 11], [560, 11]]);
		ok(cnt(b4, aNew1) === 1 && aNew1.violations.length === 2, `4 mil gap kept + NEW 1 mil gap 300 mil away -> 1 new of ${aNew1.violations.length} (got ${cnt(b4, aNew1)})`);
		// (b) existing 1 mil gap (worse), kept; a NEW 3 mil gap elsewhere: legacy per-pair min misses it
		const b1 = run([[0, 11], [150, 11], [250, 100]]);
		const aNew3 = run([[0, 11], [150, 11], ...tail, [450, 13], [560, 13]]);
		ok(cnt(b1, aNew3) === 1 && legacyNew(b1, aNew3) === 0, `1 mil gap kept + NEW 3 mil gap elsewhere -> 1 new; the legacy per-pair rule says ${legacyNew(b1, aNew3)} (got ${cnt(b1, aNew3)})`);
		// (c) existing short (B on top of A, gap -10), kept; a NEW crossing elsewhere between the same two nets
		const bShort = run([[0, 0], [100, 0], [200, 100]]);
		const aCross = run([[0, 0], [100, 0], [200, 100], [350, 100], [450, -60], [450, -120]]);
		const nvC = newViolations(bShort, aCross);
		ok(nvC.length === 1 && legacyNew(bShort, aCross) === 0 && nvC[0].x > 300, `existing short kept + NEW crossing at x~${nvC[0]?.x?.toFixed(0)} -> 1 new (got ${nvC.length}); legacy says ${legacyNew(bShort, aCross)}`);
		// (d) the very same existing violation stays in place -> not new (tail changed, 10 mil shift of the corner)
		const sameSpot = run([[0, 14], [150, 14], [260, 120]]);
		const shifted = run([[0, 14], [160, 14], [250, 100]]);
		ok(cnt(b4, b4) === 0 && cnt(b4, sameSpot) === 0 && cnt(b4, shifted) === 0, `existing violation stays / tail rerouted / corner moved 10 mil -> 0 new (got ${cnt(b4, b4)},${cnt(b4, sameSpot)},${cnt(b4, shifted)})`);
		// (e) same place but worse -> new ; same place but better -> not new ; both found at the same spot
		ok(cnt(b4, run([[0, 12], [150, 12], [250, 100]])) === 1 && cnt(b4, run([[0, 15], [150, 15], [250, 100]])) === 0, 'same place: 4 -> 2 mil is new, 4 -> 5 mil is not');
		// (f) existing violation moved far away (> max(20, 2w)) with the same gap -> new (it is a different place)
		const moved = run([[0, 30], [250, 100], [350, 100], [400, 14], [560, 14]]);
		ok(cnt(b4, moved) === 1, `4 mil gap now 400 mil away from the original place -> new (got ${cnt(b4, moved)})`);
		// (g) report per location, not per pair: two separate violating places = 2 violations, one pair summary
		ok(aNew1.violations.length === 2 && aNew1.pairs.length === 1 && aNew1.pairs[0].violations === 2, `two places of one net pair -> 2 violations, pair summary has 1 entry with 2 (got ${aNew1.violations.length}/${aNew1.pairs.length}/${aNew1.pairs[0]?.violations})`);
		// (h) foreign static track (one item, long polyline): existing 1 mil near one end, new 3 mil at its other end
		const stat = (pts) => ({ statics: { tracks: [{ id: 'S1', net: 'Z', layer: 1, lw: 10, pts: pts.map(([x, y]) => ({ x, y })) }] } });
		const mover = (pts) => net('M', 10, ...poly('M', pts));
		const sBefore = strictDrc(ctxOf([mover([[0, 0], [600, 0]])], stat([[0, 11], [100, 11], [300, 200], [500, 200], [560, 13], [600, 13]])));
		const sAfter = strictDrc(ctxOf([mover([[0, 0], [600, 0]])], stat([[0, 11], [100, 11], [300, 200], [500, 200], [560, 13], [600, 13]])));
		ok(cnt(sBefore, sAfter) === 0 && sBefore.violations.length === 2, `same static item, 2 existing violating places, unchanged -> 0 new (violations ${sBefore.violations.length}, new ${cnt(sBefore, sAfter)})`);
		const sOnlyOne = strictDrc(ctxOf([mover([[0, 0], [600, 0]])], stat([[0, 11], [100, 11], [300, 200], [500, 200], [560, 40], [600, 40]])));
		ok(cnt(sOnlyOne, sAfter) === 1 && legacyNew(sOnlyOne, sAfter) === 0, `same static item: existing 1 mil at one end, NEW 3 mil at the other end -> 1 new (got ${cnt(sOnlyOne, sAfter)}; legacy ${legacyNew(sOnlyOne, sAfter)})`);
	}
	{
		// connectivity: same-layer same-net tracks crossing in their interiors are connected copper
		const pads = [['A.1', 0, 0], ['A.2', 100, 100], ['A.3', 0, 100], ['A.4', 100, 0]].map(([id, x, y], i) => ({ x, y, net: 'A', id, layer: 1, radius: 8, index: i }));
		const shapes = {};
		for (const p of pads) shapes[padKey(p.x, p.y)] = { pad: ['ELLIPSE', 12, 12], hole: null, rotation: 0 };
		const X = ctxOf([net('A', 10, L('A', 0, 0, 100, 100), L('A', 0, 100, 100, 0))], { allPads: pads, padShapes: shapes });
		ok(JSON.stringify(connectivity(X).A.parts) === '[["A.1","A.2","A.3","A.4"]]', `crossing X of two same-net tracks -> one part (got ${JSON.stringify(connectivity(X).A.parts)})`);
		const arcX = { kind: 'arc', net: 'A', layer: 1, cx: 50, cy: -50, r: 100, startAngle: Math.PI / 2 - 0.5, sweep: 1.0 };   // sags through (50,50), crossing the horizontal line y=50
		const arcPads = [['A.1', 0, 50], ['A.2', 100, 50]].map(([id, x, y], i) => ({ x, y, net: 'A', id, layer: 1, radius: 8, index: i }));
		const arcShapes = {};
		for (const p of arcPads) arcShapes[padKey(p.x, p.y)] = { pad: ['ELLIPSE', 12, 12], hole: null, rotation: 0 };
		const [ae0, ae1] = primEnds(arcX);
		const arcCtx = (second) => ctxOf([net('A', 10, L('A', -20, 50, 120, 50), second)], { allPads: arcPads, padShapes: arcShapes });
		ok(Math.abs(ae0[1] - 50) > 5 && JSON.stringify(connectivity(arcCtx(arcX)).A.parts) === '[["A.1","A.2"]]' && connectivity(arcCtx(arcX)).A.floating === 0, 'an arc crossing a line in its interior is connected (one component)');
		const apart = ctxOf([net('A', 10, L('A', 0, 0, 100, 0), L('A', 0, 30, 100, 30))], { allPads: [pads[0], { ...pads[3], y: 30, id: 'A.9' }].map((p, i) => ({ ...p, index: i })), padShapes: { ...shapes, [padKey(100, 30)]: shapes[padKey(100, 0)] } });
		ok(connectivity(apart).A.parts.length === 2, 'parallel tracks 30 mil apart are NOT connected');
		const diffX = connectivityDiff(connectivity(ctxOf([net('A', 10, L('A', 0, 0, 100, 100), L('A', 0, 100, 100, 0))], { allPads: pads, padShapes: shapes })), connectivity(X));
		ok(diffX.length === 0, 'connectivityDiff of identical crossing nets is empty');
		void ae1;
	}
	{
		// POLYGON pads are measured with their real polygon: an L shaped pad (arms along x=0..10 and y=0..10, 40 long)
		const verts = [[0, 0], [40, 0], [40, 10], [10, 10], [10, 40], [0, 40]];
		const flat = []; verts.forEach(([x, y], i) => { if (i) flat.push('L'); flat.push(x, y); });
		const pad = { x: 14.3, y: 14.3, net: 'P', id: 'P.1', layer: 1, radius: 20, index: 0 };
		const shapes = { [padKey(pad.x, pad.y)]: { pad: ['POLYGON', flat], hole: null, rotation: 0 } };
		const run = (y, x0) => strictDrc(ctxOf([net('A', 10, L('A', x0, y, 120, y))], { allPads: [pad], padShapes: shapes }));
		// y=30 from x=25: 15 from the left arm's edge (x=10), 20 from the bottom arm's edge (y=10): gap 10 -> clean (the bbox 40x40 / the radius circle would hit)
		ok(run(30, 25).violations.length === 0, `L polygon pad: track in the notch at gap 10 -> clean (got ${run(30, 25).violations.length})`);
		const near = run(30, 14);   // 4 from the left arm -> gap -1
		ok(near.violations.length === 1 && near.violations[0].kind === 'trk-pad' && Math.abs(near.violations[0].gap + 1) < 0.3, `L polygon pad: track 4 from the arm edge -> violation, gap ~ -1 (got ${near.violations[0]?.gap?.toFixed(2)})`);
		const inside = ctxOf([net('P', 10, L('P', 5, 35, 5, 200)), net('Q', 10, L('Q', 0, 0, 1, 0))], { allPads: [pad, { x: 5, y: 200, net: 'P', id: 'P.2', layer: 1, radius: 5, index: 1 }], padShapes: { ...shapes, [padKey(5, 200)]: { pad: ['ELLIPSE', 10, 10], hole: null, rotation: 0 } } });
		ok(JSON.stringify(connectivity(inside).P.parts) === '[["P.1","P.2"]]', 'track end inside the polygon arm (22 mil from the pad centre) connects to the POLYGON pad');
		ok(polygonVertices(['POLYGON', flat]).length === 6, 'polygonVertices skips L markers');
	}
	{
		// idempotence helper
		const n1 = [{ name: 'A', prims: [L('A', 0, 0, 100, 0), { kind: 'arc', layer: 1, cx: 100, cy: 50, r: 50, startAngle: -Math.PI / 2, sweep: Math.PI / 2 }] }];
		const n2 = [{ name: 'A', prims: [{ kind: 'arc', layer: 1, cx: 100, cy: 50, r: 50, startAngle: 0, sweep: -Math.PI / 2 }, L('A', 100.004, 0, 0, 0.004)] }];
		const n3 = [{ name: 'A', prims: [L('A', 0, 0, 100, 0), { kind: 'arc', layer: 1, cx: 100, cy: 50, r: 50, startAngle: -Math.PI / 2, sweep: Math.PI / 2 }].map(p => (p.kind === 'line' ? { ...p, y2: 0.05 } : p)) }];
		ok(sameGeometry(n1, n2).equal && !sameGeometry(n1, n3).equal && !sameGeometry(n1, [{ name: 'A', prims: n1[0].prims.slice(0, 1) }]).equal, 'sameGeometry: reversed / reordered within 0.01 equal, 0.05 off not, missing prim not');
	}
	{
		const w = widthsPreserved([net('A', 10, L('A', 0, 0, 1, 0)), net('B', 20, L('B', 0, 0, 1, 0))], [net('A', 10, L('A', 0, 0, 1, 0)), net('B', 10, L('B', 0, 0, 1, 0))]);
		ok(!w.ok && w.bad.length === 1 && w.bad[0].net === 'B', 'widthsPreserved flags the net whose width changed');
	}
	console.log(fail ? `\nSELFTEST FAILED (${fail})` : '\nSELFTEST OK');
	process.exit(fail ? 1 : 0);
}
if (process.argv[1] && import.meta.url === pathToFileURL(process.argv[1]).href && process.argv.includes('--selftest')) selftest();
