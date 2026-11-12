/**
 * tighten-boards.mjs - boards + offline harness shared by bench/tighten-bench.mjs and
 * test/tighten-fixtures-test.ts.
 *
 * The board generators and the static-obstacle builder are COPIED from scripts/bench-offline.mjs
 * (kept in sync by hand; bench-offline is protected). The harness drives the REAL production worker
 * entry (src/worker/planning-worker.ts, message type 'tighten') from an esbuild bundle of an arbitrary
 * source root (so the pristine pre-fix snapshot can be benchmarked with the very same code).
 *
 * Net model used here: nets = [{ name, prims: [{kind, net, layer, ..., width}] }] (width in mil per prim;
 * the worker output carries no width, the run's width is re-applied), vias = [{net,x,y,dia,hole}].
 */
import * as esbuild from 'esbuild';
import * as path from 'path';
import { toNets } from './tighten-metrics.mjs';

export function mulberry32(a) {
	return () => {
		a |= 0; a = (a + 0x6D2B79F5) | 0;
		let t = Math.imul(a ^ (a >>> 15), 1 | a);
		t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
		return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
	};
}

export const r1 = (v) => Math.round(v * 10) / 10;
export const r2 = (v) => Math.round(v * 100) / 100;

export const DRC = {
	traceWidthMil: 10, clearanceMil: 6,
	trackToTrackMil: 6, trackToLineMil: 6, trackToBoardMil: 11.81, trackToPadMil: 6,
	trackToFillMil: 10, holeToHoleMil: 11.81,
};
export const VIA = { dia: 24, hole: 12 };
export const padKey = (x, y) => `${x.toFixed(2)},${y.toFixed(2)}`;

/** pad factory — mirrors the EnginePad / PadShape population of scripts/eda-dump-driver.ts */
export function makePad(board, { x, y, net, id, layer, kind, w, h, shape, rotation = 0, polygon }) {
	// polygon: [[x,y],...] absolute vertices -> pad: ['POLYGON', [x0,y0,'L',x1,y1,...]] (w/h only size the pad radius)
	let shapeName = shape ? String(shape).toUpperCase() : 'RECT', hole = null, padType = 'smd', holeMil = 0, lay = layer;
	if (kind === 'th') { shapeName = 'ELLIPSE'; hole = ['ROUND', 36, 36]; holeMil = 36; padType = 'th'; lay = 12; }
	const pad = {
		x, y, net, id, layer: lay, padType, holeDiameter: holeMil,
		radius: Math.max(w / 2, h / 2, holeMil / 2),
		bodyRadius: Math.max(Math.min(w, h) / 2, holeMil / 2),
		rotation,
		index: board.allPads.length,
	};
	board.allPads.push(pad);
	board.padShapes[padKey(x, y)] = { pad: [shapeName, w, h], hole, rotation };
	if (polygon) {
		const flat = [];
		polygon.forEach(([vx, vy], i) => { if (i) flat.push('L'); flat.push(vx, vy); });
		board.padShapes[padKey(x, y)] = { pad: ['POLYGON', flat], hole, rotation: 0 };
		pad.radius = Math.max(...polygon.map(([vx, vy]) => Math.hypot(vx - x, vy - y)));
		pad.bodyRadius = pad.radius;
		pad.rotation = 0;
	}
	return pad;
}

export function buildStaticObstacles(mod, board, mode, extraTracks = [], extraVias = []) {
	const d = board.drc;
	const lineClear = mode === 'tighten' ? d.trackToTrackMil : Math.max(d.trackToLineMil, 8);
	const copperMargin = mode === 'tighten' ? 0.5 : 2;
	const out = [];
	const o = board.outline;
	if (o) {
		const r = 10 / 2 + d.trackToBoardMil + d.traceWidthMil / 2;
		out.push(...mod.buildPolylineObstacle(o, r, undefined, undefined, 'board', true));
	}
	for (const f of board.statics.fills) {
		const r = (f.lw ?? 0) / 2 + Math.max(d.trackToFillMil, 10) + d.traceWidthMil / 2 + 2;
		out.push(...mod.buildPolygonObstacle(f.pts, r, f.net || undefined, f.layer, `fill:${f.pts[0].x},${f.pts[0].y}`, true));
	}
	const trackObs = (t) => {
		const r = (t.lw ?? 10) / 2 + lineClear + d.traceWidthMil / 2 + copperMargin;
		out.push(...mod.buildPolylineObstacle(t.pts, r, t.net || undefined, t.layer, t.group ?? `line:${t.id}`));
	};
	for (const t of board.statics.tracks) trackObs(t);
	for (const t of extraTracks) trackObs(t);
	const viaClear = Math.max(d.trackToTrackMil, d.trackToPadMil);
	for (const v of [...board.statics.vias, ...extraVias]) {
		out.push({
			id: 'via' + out.length, kind: 'circle', x: v.x, y: v.y,
			r: v.dia / 2 + viaClear + d.traceWidthMil / 2 + 0.1,
			net: v.net || undefined, layer: undefined,
			group: `via:${v.net ?? ''}:${v.x.toFixed(1)},${v.y.toFixed(1)}`, noPrune: true,
		});
	}
	return out;
}

export function genRoutingBoard({ seed, nets, layers }) {
	const rng = mulberry32(seed);
	const rint = (a, b) => a + Math.floor(rng() * (b - a + 1));

	// 1) net sizes: GND (many pads), VCC, then signals 70% 2-pin / 15% 3 / 10% 4 / 5% 5
	const sizes = [];
	if (nets >= 4) {
		sizes.push({ name: 'GND', k: Math.min(12, Math.max(3, Math.round(nets / 8))) });
		sizes.push({ name: 'VCC', k: Math.min(6, Math.max(3, Math.round(nets / 14))) });
	}
	const nSig = nets - sizes.length;
	for (let i = 0; i < nSig; i++) {
		const r = rng();
		sizes.push({ name: `NET_${String(i + 1).padStart(3, '0')}`, k: r < 0.70 ? 2 : r < 0.85 ? 3 : r < 0.95 ? 4 : 5 });
	}
	const totalPads = sizes.reduce((s, n) => s + n.k, 0);

	// 2) component library (local coords, y down; pads {dx,dy,w,h,kind})
	const passive = (bottom) => ({ name: 'R', pads: [-38, 38].map(dx => ({ dx, dy: 0, w: 32, h: 36, kind: 'smd', bottom })) });
	const soic = (n) => {
		const pads = [];
		for (let i = 0; i < n / 2; i++) {
			const y = (i - (n / 2 - 1) / 2) * 50;
			pads.push({ dx: -120, dy: y, w: 62, h: 24, kind: 'smd' }, { dx: 120, dy: y, w: 62, h: 24, kind: 'smd' });
		}
		return { name: `SOIC${n}`, pads };
	};
	const qfp = (n) => {
		const per = n / 4, pitch = 19.7, rowHalf = (per - 1) * pitch / 2, ring = rowHalf + 60;
		const pads = [];
		for (let i = 0; i < per; i++) {
			const t = -rowHalf + i * pitch;
			pads.push({ dx: t, dy: -ring, w: 11, h: 59, kind: 'smd' }, { dx: t, dy: ring, w: 11, h: 59, kind: 'smd' },
				{ dx: -ring, dy: t, w: 59, h: 11, kind: 'smd' }, { dx: ring, dy: t, w: 59, h: 11, kind: 'smd' });
		}
		return { name: `QFP${n}`, pads };
	};
	const header = (n, pitch = 100) => ({
		name: `J${n}`, pads: Array.from({ length: n }, (_, i) => ({ dx: 0, dy: (i - (n - 1) / 2) * pitch, w: 62, h: 62, kind: 'th' })),
	});

	const comps = [];
	let remaining = totalPads;
	while (remaining > 0) {
		const r = rng();
		if (remaining >= 48 && r < 0.03) comps.push(qfp(48));
		else if (remaining >= 32 && r < 0.06) comps.push(qfp(32));
		else if (remaining >= 16 && r < 0.12) comps.push(soic(16));
		else if (remaining >= 8 && r < 0.20) comps.push(soic(8));
		else if (r < 0.24 || remaining < 2) comps.push(header(Math.min(remaining, rint(2, 8))));
		else comps.push(passive(layers >= 2 && rng() < 0.10));
		remaining -= comps[comps.length - 1].pads.length;
	}
	// pre-routed foreign nets (kept out of input.networks; their tracks/via are static obstacles)
	const preA = { ...header(2, 100), name: 'PRE_A', pre: 'PRE_A' };
	const preB = { ...header(2, 200), name: 'PRE_B', pre: 'PRE_B' };
	comps.push(preA, preB);
	// keepouts (fills, no pads)
	const nKeep = Math.max(1, Math.round(nets / 40));
	for (let i = 0; i < nKeep; i++) comps.push({ name: 'KEEPOUT', pads: [], keepout: { w: 320, h: 200, layer: i % 2 === 0 ? 1 : (layers >= 2 ? 2 : 1) } });

	// 3) orientation + bbox + placement
	const GAP = 70, EDGE = 150;
	for (const c of comps) {
		c.rot = c.keepout ? 0 : rint(0, 3);
		const odd = c.rot % 2 === 1;
		const cs = [1, 0, -1, 0][c.rot], sn = [0, 1, 0, -1][c.rot];
		for (const p of c.pads) {
			const x = p.dx * cs - p.dy * sn, y = p.dx * sn + p.dy * cs;
			p.dx = x; p.dy = y;
			if (odd) { const t = p.w; p.w = p.h; p.h = t; }
		}
		if (c.keepout) { c.hx = c.keepout.w / 2; c.hy = c.keepout.h / 2; }
		else {
			c.hx = Math.max(...c.pads.map(p => Math.abs(p.dx) + p.w / 2));
			c.hy = Math.max(...c.pads.map(p => Math.abs(p.dy) + p.h / 2));
		}
	}
	const area = comps.reduce((s, c) => s + (2 * c.hx + GAP) * (2 * c.hy + GAP), 0) * 3.0;
	let W = Math.sqrt(area * 1.5) + 2 * EDGE, H = area / Math.sqrt(area * 1.5) + 2 * EDGE;
	const order = [...comps].sort((a, b) => b.hx * b.hy - a.hx * a.hy);
	for (let attempt = 0; attempt < 40; attempt++) {
		const placed = [];
		let ok = true;
		for (const c of order) {
			let done = false;
			for (let t = 0; t < 600 && !done; t++) {
				const x = EDGE + c.hx + rng() * Math.max(1, W - 2 * EDGE - 2 * c.hx);
				const y = EDGE + c.hy + rng() * Math.max(1, H - 2 * EDGE - 2 * c.hy);
				if (placed.every(q => Math.abs(q.cx - x) >= q.hx + c.hx + GAP || Math.abs(q.cy - y) >= q.hy + c.hy + GAP)) {
					c.cx = Math.round(x * 2) / 2; c.cy = Math.round(y * 2) / 2; placed.push(c); done = true;
				}
			}
			if (!done) { ok = false; break; }
		}
		if (ok) break;
		W *= 1.08; H *= 1.08;
		if (attempt === 39) throw new Error('component placement failed');
	}
	W = Math.round(W); H = Math.round(H);

	const board = {
		name: 'routing', widthMil: W, heightMil: H, drc: { ...DRC }, copperLayers: layers,
		allPads: [], padShapes: {}, networks: [], components: comps.length,
		outline: [{ x: 0, y: 0 }, { x: W, y: 0 }, { x: W, y: H }, { x: 0, y: H }, { x: 0, y: 0 }],
		statics: { tracks: [], vias: [], fills: [] },
	};

	// 4) pads (net assigned later); PRE nets assigned now
	const free = [];
	let compNo = 0;
	for (const c of comps) {
		compNo++;
		if (c.keepout) {
			const { w, h, layer } = c.keepout;
			const pts = [[-w / 2, -h / 2], [w / 2, -h / 2], [w / 2, h / 2], [-w / 2, h / 2], [-w / 2, -h / 2]].map(([a, b]) => ({ x: c.cx + a, y: c.cy + b }));
			board.statics.fills.push({ layer, pts, lw: 0 });
			continue;
		}
		const made = [];
		c.pads.forEach((p, i) => {
			const x = Math.round((c.cx + p.dx) * 100) / 100, y = Math.round((c.cy + p.dy) * 100) / 100;
			const lay = p.bottom ? 2 : 1;
			const id = `${c.name}${compNo}-${i + 1}`;
			if (c.pre) made.push(makePad(board, { x, y, net: c.pre, id, layer: lay, kind: p.kind, w: p.w, h: p.h }));
			else free.push({ spec: { x, y, id, layer: lay, kind: p.kind, w: p.w, h: p.h }, comp: compNo });
		});
		if (c.pre === 'PRE_A') {
			board.statics.tracks.push({ id: 'preA', net: 'PRE_A', layer: 1, lw: 10, pts: made.map(p => ({ x: p.x, y: p.y })) });
		} else if (c.pre === 'PRE_B') {
			const mx = (made[0].x + made[1].x) / 2, my = (made[0].y + made[1].y) / 2;
			board.statics.tracks.push({ id: 'preB1', net: 'PRE_B', layer: 2, lw: 10, pts: [{ x: made[0].x, y: made[0].y }, { x: mx, y: my }] });
			board.statics.tracks.push({ id: 'preB2', net: 'PRE_B', layer: 1, lw: 10, pts: [{ x: mx, y: my }, { x: made[1].x, y: made[1].y }] });
			board.statics.vias.push({ net: 'PRE_B', x: mx, y: my, dia: VIA.dia, hole: VIA.hole });
		}
	}

	// 5) assign pads to nets by locality (anchor + random among the nearest free pads,
	//    preferring other components so nets are not intra-footprint shorts)
	const netsByName = new Map();
	const order2 = [...sizes].sort((a, b) => b.k - a.k);
	for (const ns of order2) {
		const anchorIdx = rint(0, free.length - 1);
		const anchor = free.splice(anchorIdx, 1)[0];
		const chosen = [anchor];
		for (let j = 1; j < ns.k && free.length; j++) {
			const dist = (f) => Math.hypot(f.spec.x - anchor.spec.x, f.spec.y - anchor.spec.y);
			let pool = free.map((f, i) => ({ f, i, d: dist(f) })).sort((a, b) => a.d - b.d);
			const diffComp = pool.filter(q => !chosen.some(c => c.comp === q.f.comp));
			if (diffComp.length >= 2) pool = diffComp;
			pool = pool.slice(0, Math.max(2, Math.min(pool.length, 4 * (ns.k - 1) + 2)));
			const pick = pool[rint(0, pool.length - 1)];
			chosen.push(free.splice(pick.i, 1)[0]);
		}
		netsByName.set(ns.name, chosen);
	}
	for (const ns of sizes) {
		const pads = (netsByName.get(ns.name) ?? []).map(f => makePad(board, { ...f.spec, net: ns.name }));
		if (pads.length) board.networks.push({ name: ns.name, pads });
	}
	// `networks` pad objects must be the same shape as allPads entries (they are the same objects)
	return board;
}

export function genBadBoard({ seed }) {
	const rng = mulberry32(seed * 7919 + 13);
	const rint = (a, b) => a + Math.floor(rng() * (b - a + 1));
	const CW = 760, CH = 460, COLS = 6, ROWS = 4, M = 200;
	const W = COLS * CW + 2 * M, H = ROWS * CH + 2 * M;
	const board = {
		name: 'bad-traces', widthMil: W, heightMil: H, drc: { ...DRC }, copperLayers: 2,
		allPads: [], padShapes: {}, networks: [],
		outline: [{ x: 0, y: 0 }, { x: W, y: 0 }, { x: W, y: H }, { x: 0, y: H }, { x: 0, y: 0 }],
		statics: { tracks: [], vias: [], fills: [] },
		traces: [], vias: [], cases: [],
	};
	const staircase = (x0, y0, nSteps, dx, dy) => {
		const pts = [{ x: x0, y: y0 }];
		for (let k = 0; k < nSteps; k++) {
			const l = pts[pts.length - 1];
			pts.push({ x: l.x + dx, y: l.y }, { x: l.x + dx, y: l.y + dy });
		}
		return pts;
	};
	const stairs45 = () => {
		const pts = [{ x: 60, y: 100 }];
		for (let k = 0; k < 5; k++) {
			const l = pts[pts.length - 1];
			pts.push({ x: l.x + 70, y: l.y }, { x: l.x + 120, y: l.y + 50 });
		}
		return pts;
	};
	const P = (x, y) => ({ x, y });
	const KEEP = { x0: 40, y0: 360, x1: 300, y1: 440 };
	// each type: pads (net's own), layerPts: [{layer, pts}], foreign pads, via, keepout?
	const types = [
		{ id: 'stairs90', pads: [[60, 60, 1], [640, 400, 1]], segs: [{ layer: 1, pts: staircase(60, 60, 5, 116, 68) }], foreign: [[620, 60, 1, 'smd']], keep: true },
		{ id: 'zigzag45', pads: [[60, 100, 1], [660, 350, 1]], segs: [{ layer: 1, pts: stairs45() }], foreign: [[60, 400, 1, 'smd']] },
		{ id: 'detourU', pads: [[60, 230, 1], [700, 230, 1]], segs: [{ layer: 1, pts: [P(60, 230), P(160, 230), P(160, 70), P(600, 70), P(600, 230), P(700, 230)] }], foreign: [[380, 230, 12, 'th']] },
		{ id: 'dogleg', pads: [[60, 60, 1], [700, 400, 1]], segs: [{ layer: 1, pts: [P(60, 60), P(700, 60), P(700, 400)] }], foreign: [[380, 230, 12, 'th']] },
		{ id: 'tee3', pads: [[60, 60, 1], [700, 60, 1], [380, 400, 1]],
			segs: [{ layer: 1, pts: [P(60, 60), P(60, 200), P(380, 200)] }, { layer: 1, pts: [P(380, 200), P(700, 200), P(700, 60)] }, { layer: 1, pts: [P(380, 200), P(380, 400)] }],
			foreign: [[700, 400, 1, 'smd']], keep: true },
		{ id: 'wiggle', pads: [[60, 230, 1], [700, 230, 1]], jitter: true, foreign: [[380, 60, 1, 'smd'], [380, 400, 1, 'smd']] },
		{ id: 'stairs90Bottom', pads: [[60, 60, 2], [640, 400, 2]], segs: [{ layer: 2, pts: staircase(60, 60, 5, 116, 68) }], foreign: [[620, 60, 2, 'smd']] },
		{ id: 'viaNet', pads: [[60, 100, 1], [700, 360, 2]],
			segs: [{ layer: 1, pts: [P(60, 100), P(220, 100), P(220, 230), P(380, 230)] }, { layer: 2, pts: [P(380, 230), P(540, 230), P(540, 360), P(700, 360)] }],
			via: [380, 230], foreign: [[380, 60, 1, 'smd'], [380, 400, 2, 'smd']] },
	];
	for (let ci = 0; ci < COLS * ROWS; ci++) {
		const t = types[ci % types.length];
		const col = ci % COLS, row = Math.floor(ci / COLS);
		const flipX = rng() < 0.5, flipY = rng() < 0.5;
		const T = (x, y) => ({ x: M + col * CW + (flipX ? CW - x : x), y: M + row * CH + (flipY ? CH - y : y) });
		const net = `B_${String(ci + 1).padStart(2, '0')}_${t.id}`;
		let segs = t.segs;
		if (t.jitter) {
			const pts = [P(60, 230)];
			let last = 0;
			for (let k = 1; k <= 7; k++) {
				let j; do { j = rint(-70, 70); } while (Math.abs(j - last) < 40);
				last = j; pts.push(P(60 + 80 * k, 230 + j));
			}
			pts.push(P(700, 230));
			// make it an orthogonal-ish wiggle: each vertex pair stays as given (arbitrary angles)
			segs = [{ layer: 1, pts }];
		}
		const myPads = t.pads.map(([x, y, layer], i) => {
			const q = T(x, y);
			return makePad(board, { x: q.x, y: q.y, net, id: `${net}.${i + 1}`, layer, kind: 'smd', w: 32, h: 36 });
		});
		const prims = [];
		segs.forEach((s, si) => {
			for (let i = 0; i + 1 < s.pts.length; i++) {
				const a = T(s.pts[i].x, s.pts[i].y), b = T(s.pts[i + 1].x, s.pts[i + 1].y);
				prims.push({ kind: 'line', net, layer: s.layer, x1: a.x, y1: a.y, x2: b.x, y2: b.y, seg: si });
			}
		});
		board.networks.push({ name: net, pads: myPads });
		board.traces.push({ name: net, prims });
		const caseInfo = { net, type: t.id, padSpanMil: myPads.length === 2 ? r1(Math.hypot(myPads[0].x - myPads[1].x, myPads[0].y - myPads[1].y)) : null };
		board.cases.push(caseInfo);
		if (t.via) {
			const q = T(t.via[0], t.via[1]);
			board.vias.push({ net, x: q.x, y: q.y, dia: VIA.dia, hole: VIA.hole });
		}
		(t.foreign ?? []).forEach(([x, y, layer, kind], i) => {
			const q = T(x, y);
			const fnet = `F_${String(ci + 1).padStart(2, '0')}_${i + 1}`;
			const p = kind === 'th'
				? makePad(board, { x: q.x, y: q.y, net: fnet, id: `${fnet}.1`, layer: 12, kind: 'th', w: 62, h: 62 })
				: makePad(board, { x: q.x, y: q.y, net: fnet, id: `${fnet}.1`, layer, kind: 'smd', w: 32, h: 36 });
			board.networks.push({ name: fnet, pads: [p] });
		});
		if (t.keep) {
			const a = T(KEEP.x0, KEEP.y0), b = T(KEEP.x1, KEEP.y1);
			const x0 = Math.min(a.x, b.x), x1 = Math.max(a.x, b.x), y0 = Math.min(a.y, b.y), y1 = Math.max(a.y, b.y);
			board.statics.fills.push({ layer: 1, lw: 0, pts: [P(x0, y0), P(x1, y0), P(x1, y1), P(x0, y1), P(x0, y0)] });
		}
	}
	// single-pad foreign nets are pads only (no tracks): not tighten inputs
	return board;
}

// ─────────────────────────────────────────────────────────────
// small board builder for hand-made fixtures
// ─────────────────────────────────────────────────────────────
export function newBoard({ W = 1200, H = 900, drc = {}, name = 'fixture' } = {}) {
	return {
		name, widthMil: W, heightMil: H, drc: { ...DRC, ...drc }, copperLayers: 2,
		allPads: [], padShapes: {}, networks: [],
		outline: [{ x: 0, y: 0 }, { x: W, y: 0 }, { x: W, y: H }, { x: 0, y: H }, { x: 0, y: 0 }],
		statics: { tracks: [], vias: [], fills: [] },
	};
}

// ─────────────────────────────────────────────────────────────
// prim helpers
// ─────────────────────────────────────────────────────────────
export function primsLength(prims) {
	let t = 0;
	for (const p of prims) t += p.kind === 'line' ? Math.hypot(p.x2 - p.x1, p.y2 - p.y1) : Math.abs(p.sweep ?? 0) * (p.r ?? 0);
	return t;
}
export function primMid(p) {
	if (p.kind === 'line') return { x: (p.x1 + p.x2) / 2, y: (p.y1 + p.y2) / 2 };
	const a = p.startAngle + p.sweep / 2;
	return { x: p.cx + p.r * Math.cos(a), y: p.cy + p.r * Math.sin(a) };
}
/** prim without bench-only extras (width / seg) — what the production worker receives */
export function stripPrim(p) {
	const { width, seg, ...rest } = p;
	void width; void seg;
	return { ...rest };
}
const q2 = (v) => Math.round((v ?? 0) * 100) / 100;
export function primsKey(prims) {
	return JSON.stringify(prims.map(p => p.kind === 'line'
		? ['l', p.layer, q2(p.x1), q2(p.y1), q2(p.x2), q2(p.y2)]
		: ['a', p.layer, q2(p.cx), q2(p.cy), q2(p.r), q2(p.startAngle), q2(p.sweep)]));
}
/** polyline -> line prims (net, layer, width stamped on every prim) */
export function polyPrims(net, layer, width, pts, extra = {}) {
	const out = [];
	for (let i = 0; i + 1 < pts.length; i++) {
		out.push({ kind: 'line', net, layer, x1: pts[i][0], y1: pts[i][1], x2: pts[i + 1][0], y2: pts[i + 1][1], width, ...extra });
	}
	return out;
}

// ─────────────────────────────────────────────────────────────
// bundling / worker instances
// ─────────────────────────────────────────────────────────────
/** esbuild-bundle the production worker + engine of source root `root` (in memory) */
export async function buildBundle(root) {
	const contents = `
		export * from './src/router/planning-engine.ts';
		export { buildPolylineObstacle, buildPolygonObstacle } from './src/router/rubberband-engine.ts';
		import './src/worker/planning-worker.ts';
	`;
	const built = await esbuild.build({
		stdin: { contents, resolveDir: path.resolve(root), loader: 'ts', sourcefile: 'tighten-entry.ts' },
		bundle: true, format: 'esm', write: false, platform: 'neutral', logLevel: 'error',
	});
	return built.outputFiles[0].text;
}
let freshCounter = 0;
/** import a fresh instance of the bundle. The worker grabs `self` at import time, so a collecting shim is
 * installed first; shim.onmessage is the real production worker handler. */
export async function loadFresh(code) {
	const shim = { messages: [], onmessage: undefined, postMessage(m) { shim.messages.push(m); } };
	globalThis.self = shim;
	const mod = await import('data:text/javascript;base64,' + Buffer.from(code + `\n//# tighten-instance ${++freshCounter}\n`).toString('base64'));
	if (typeof shim.onmessage !== 'function') throw new Error('planning-worker did not install self.onmessage');
	return { mod, shim };
}

/** routed prims -> polyline "tracks" (the OLD production tighten world also contained these) */
export function primsToTracks(nets, defaultWidth = 10) {
	const tracks = [];
	let k = 0;
	for (const n of nets) {
		for (const p of n.prims) {
			let pts;
			if (p.kind === 'line') pts = [{ x: p.x1, y: p.y1 }, { x: p.x2, y: p.y2 }];
			else {
				const steps = Math.max(6, Math.ceil(Math.abs(p.sweep ?? 0) / (Math.PI / 12)));
				pts = [];
				for (let i = 0; i <= steps; i++) {
					const a = p.startAngle + (p.sweep * i) / steps;
					pts.push({ x: p.cx + p.r * Math.cos(a), y: p.cy + p.r * Math.sin(a) });
				}
			}
			tracks.push({ id: `${n.name}#${k++}`, net: n.name, layer: p.layer ?? 1, lw: p.width ?? defaultWidth, pts });
		}
	}
	return tracks;
}
/** fixed foreign copper in the NEW `tracks` message field: exact, one entry per segment */
export function staticTracksExact(board) {
	const out = [];
	for (const t of board.statics.tracks) {
		for (let i = 0; i + 1 < t.pts.length; i++) {
			out.push({ net: t.net, layer: t.layer, width: t.lw ?? 10, prim: { kind: 'line', net: t.net, layer: t.layer, x1: t.pts[i].x, y1: t.pts[i].y, x2: t.pts[i + 1].x, y2: t.pts[i + 1].y } });
		}
	}
	return out;
}

/** metrics context (see tighten-metrics.mjs) for a state of the board */
export function metricsCtx(_mod, board, nets, vias) {
	return {
		drc: board.drc, allPads: board.allPads, padShapes: board.padShapes, vias,
		statics: { outline: board.outline, tracks: board.statics.tracks, vias: board.statics.vias, fills: board.statics.fills },
		nets: toNets(nets, (n, p) => p.width ?? board.drc.traceWidthMil),
	};   // POLYGON pads are measured by the metrics lib itself with their real polygon (no hook into src/)
}

/** production split (topological-router.tightenExistingRouting) with width carried per run */
export function splitRuns(mod, board, nets, vias) {
	const split = new mod.PlanningEngine();
	split.initTightenWorld(board.drc, board.allPads, [], board.padShapes);
	const viasByNet = new Map();
	for (const v of vias) { if (!viasByNet.has(v.net)) viasByNet.set(v.net, []); viasByNet.get(v.net).push(v); }
	const runs = [];
	let covered = 0, total = 0;
	for (const n of nets) {
		if (!n.prims.length) continue;
		total += n.prims.length;
		const mids = n.prims.map(p => ({ p, m: primMid(p) }));
		for (const seg of split.splitRunsForTighten(n.prims.map(stripPrim), viasByNet.get(n.name) ?? [])) {
			// width of a run = width of its first prim (matched back to the original prim by midpoint)
			const m = primMid(seg.prims[0]);
			let best = null, bd = Infinity;
			for (const c of mids) {
				if ((c.p.layer ?? 1) !== (seg.prims[0].layer ?? 1)) continue;
				const d = Math.hypot(c.m.x - m.x, c.m.y - m.y);
				if (d < bd) { bd = d; best = c.p; }
			}
			const width = best?.width ?? board.drc.traceWidthMil;
			covered += seg.prims.length;
			runs.push({ id: runs.length, net: n.name, layer: seg.layer, width, a: seg.a, b: seg.b, prims: seg.prims });
		}
	}
	return { runs, uncovered: total - covered };
}

/**
 * One tighten round through the real worker entry.
 *   world 'clean': baseWorld = static copper only; 'prod': + a copy of every existing track (old production main thread)
 * returns { nets (after, width per prim), runs, done, messages, wallMs, timedOut, missingDone, failure, runsReported, runsChanged, rewrittenNets, uncovered, error }
 * `failure` (string | null) is set when the pass timed out or the worker never sent `tightenDone`: callers MUST treat that as a failure.
 */
export async function tightenPass({ code, board, nets, vias, world = 'clean', timeoutMs = 300000 }) {
	const { mod, shim } = await loadFresh(code);
	const { runs, uncovered } = splitRuns(mod, board, nets, vias);
	const extra = world === 'prod' ? primsToTracks(nets, board.drc.traceWidthMil) : [];
	const baseWorld = buildStaticObstacles(mod, board, 'tighten', extra, []);
	const msg = {
		type: 'tighten', drc: board.drc, allPads: board.allPads, baseWorld, padShapes: board.padShapes,
		vias, tracks: staticTracksExact(board),
		runs: runs.map(r => ({ id: r.id, net: r.net, layer: r.layer, width: r.width, a: r.a, b: r.b, prims: r.prims })),
	};
	const t0 = Date.now();
	let timedOut = false;
	const timer = setTimeout(() => { timedOut = true; shim.onmessage({ data: { type: 'cancel' } }); }, timeoutMs);
	await shim.onmessage({ data: structuredClone(msg) });
	clearTimeout(timer);
	const wallMs = Date.now() - t0;
	const err = shim.messages.find(m => m.type === 'error');
	const cur = new Map(runs.map(r => [r.id, r.prims]));
	const reported = new Set();
	const override = new Map();
	let done = null;
	for (const m of shim.messages) {
		if (m.type === 'tightenRun') { if (m.prims && m.prims.length) { cur.set(m.id, m.prims); reported.add(m.id); } }
		else if (m.type === 'netRewrite') { if (m.prims && m.prims.length) override.set(m.net, m.prims); }
		else if (m.type === 'tightenDone') done = m;
	}
	const widthOfRun = new Map(runs.map(r => [r.id, r.width]));
	const after = nets.map(n => {
		const rs = runs.filter(r => r.net === n.name);
		if (override.has(n.name)) {
			const ws = new Set(rs.map(r => r.width));
			const w = ws.size === 1 ? rs[0].width : (rs[0]?.width ?? board.drc.traceWidthMil);
			return { name: n.name, prims: override.get(n.name).map(p => ({ ...p, width: w })) };
		}
		return { name: n.name, prims: rs.flatMap(r => cur.get(r.id).map(p => ({ ...p, width: widthOfRun.get(r.id) }))) };
	});
	let runsChanged = 0;
	for (const r of runs) if (!override.has(r.net) && primsKey(cur.get(r.id)) !== primsKey(r.prims)) runsChanged++;
	for (const net of override.keys()) runsChanged += runs.filter(r => r.net === net).length;
	return {
		nets: after, runs, done, messages: shim.messages, wallMs, timedOut, uncovered,
		runsReported: reported.size, runsChanged, rewrittenNets: [...override.keys()],
		error: err ? String(err.message) : null,
		missingDone: !done,
		failure: timedOut ? `tighten TIMED OUT after ${timeoutMs}ms (cancel sent)${done ? '' : ', no tightenDone message'}`
			: !done && !err ? 'worker finished without a tightenDone message' : null,
	};
}

/** route a generated routing board with PlanningEngine.plan (as bench-offline phase A) */
export async function routeBoard(code, board, { budgetMs = 60000, onLog } = {}) {
	const { mod } = await loadFresh(code);
	const clone = (o) => structuredClone(o);
	const input = {
		drc: clone(board.drc), allPads: clone(board.allPads), networks: clone(board.networks),
		staticObstacles: buildStaticObstacles(mod, board, 'plan'), padShapes: clone(board.padShapes),
		budgetMs, copperLayers: board.copperLayers,
		options: { ripupRounds: 5, optimize: true, allowViaFallback: true, allowViaInPad: false, netOrder: 'auto' },
	};
	const eng = new mod.PlanningEngine();
	const result = await eng.plan(input, { isCancelled: () => false, onNetUpdate: () => {}, onLog: (m, l) => { if (onLog) onLog(m, l); } });
	return {
		nets: result.nets.map(n => ({ name: n.name, prims: n.prims.map(p => ({ ...p, width: board.drc.traceWidthMil })) })),
		vias: result.vias.map(v => ({ ...v })),
	};
}

// ─────────────────────────────────────────────────────────────
// "wide" variant of the bad board
// ─────────────────────────────────────────────────────────────
export const WIDTH_LADDER = [30, 20, 15, 10, 6];
/**
 * Assign mixed actual widths (6/10/15/20/30) to the bad board's traces, then repeatedly narrow every net that
 * takes part in a strict-DRC violation until the board is clean with ACTUAL widths. Per (net, seg) widths so
 * the T net and the via net get different widths per arm / layer.
 * `strictDrc` is injected (tighten-metrics) to keep this file import-light.
 */
export function assignWideWidths(board, mod, strictDrc, vias) {
	const pattern = [20, 6, 15, 30, 10, 20, 6, 15];
	const widths = new Map();   // `${net}|${seg}` -> ladder index
	board.traces.forEach((tr, ti) => {
		for (const p of tr.prims) {
			const k = `${tr.name}|${p.seg ?? 0}`;
			if (!widths.has(k)) widths.set(k, WIDTH_LADDER.indexOf(pattern[(ti + (p.seg ?? 0)) % pattern.length]));
		}
	});
	const apply = () => {
		for (const tr of board.traces) for (const p of tr.prims) p.width = WIDTH_LADDER[widths.get(`${tr.name}|${p.seg ?? 0}`)];
	};
	const traceNames = new Set(board.traces.map(t => t.name));
	for (let it = 0; it < 8; it++) {
		apply();
		const d = strictDrc(metricsCtx(mod, board, board.traces, vias));
		const bad = new Set();
		for (const v of d.violations) { if (traceNames.has(v.netA)) bad.add(v.netA); if (traceNames.has(v.netB)) bad.add(v.netB); }
		if (!bad.size) return { clean: true, iterations: it };
		for (const [k, i] of widths) if (bad.has(k.split('|')[0])) widths.set(k, Math.min(WIDTH_LADDER.length - 1, i + 1));
	}
	apply();
	return { clean: false, iterations: 8 };
}
