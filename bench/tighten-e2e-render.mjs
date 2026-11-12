#!/usr/bin/env node
/**
 * tighten-e2e-render.mjs - SVG renderer of a canvas snapshot written by bench/tighten-e2e-run.mjs.
 *
 *   node bench/tighten-e2e-render.mjs <snapshot.json> --out file.svg [--box x0,y0,x1,y1] [--marks "x,y;x,y"] [--title text]
 *
 * Input  snapshot.json: { lines, arcs, vias, pads, polylines, fills, padShapes?, ... } (see tighten-e2e-entry.ts)
 * Output one SVG. Units: mil, DATA Y IS UP (the SVG is drawn in a y-flipped group; this matches the EDA canvas
 *        orientation by assumption, not verified against a screenshot). Dark background; top layer red,
 *        bottom layer blue, both semi-transparent (overlaps show darker/mixed); tracks at their TRUE width
 *        (lineWidth); pads with their real shape (RECT / ROUNDRECT / ELLIPSE / OVAL / POLYGON, rotation, drill);
 *        vias gold ring with a dark hole; board outline (layer 11) yellow; copper polylines/fills grey.
 *        --box clips the view to a window (zoom); --marks draws small white crosses (e.g. violations).
 *
 * Also exported: renderSnapshot(snap, { box, marks, title, widthPx }) -> svg string, boardBox(snap).
 */
import * as fs from 'node:fs';
import { pathToFileURL } from 'node:url';

const esc = (s) => String(s).replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
const f = (v) => (Math.round(v * 100) / 100).toString();

/** canvas arc (two-point form: start, end, arcAngle deg) -> centre form; same formula as canvasArcToPrim in topological-router.ts */
export function arcCentre(a) {
	const sx = a.startX, sy = a.startY, ex = a.endX, ey = a.endY;
	const sweep = (Number(a.arcAngle ?? 0) * Math.PI) / 180;
	const L = Math.hypot(ex - sx, ey - sy);
	if (!sweep || Math.abs(Math.sin(sweep / 2)) < 1e-6 || L < 1e-6) return null;
	const R = L / (2 * Math.abs(Math.sin(sweep / 2)));
	const sgn = sweep > 0 ? 1 : -1;
	const h = sgn * R * Math.cos(sweep / 2);
	const nx = -(ey - sy) / L, ny = (ex - sx) / L;
	const cx = (sx + ex) / 2 + h * nx, cy = (sy + ey) / 2 + h * ny;
	return { cx, cy, r: R, startAngle: Math.atan2(sy - cy, sx - cx), sweep };
}

export function boardBox(snap) {
	let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
	const add = (x, y) => { if (Number.isFinite(x) && Number.isFinite(y)) { x0 = Math.min(x0, x); y0 = Math.min(y0, y); x1 = Math.max(x1, x); y1 = Math.max(y1, y); } };
	let used = false;
	for (const p of snap.polylines ?? []) {
		if ((p.layer ?? p.layerId) !== 11) continue;
		for (const q of p.pts ?? []) { add(q.x, q.y); used = true; }
	}
	for (const l of snap.lines ?? []) if (l.layer === 11) { add(l.startX, l.startY); add(l.endX, l.endY); used = true; }
	if (!used) {
		for (const p of snap.pads ?? []) add(p.x, p.y);
		for (const l of snap.lines ?? []) { add(l.startX, l.startY); add(l.endX, l.endY); }
		for (const v of snap.vias ?? []) add(v.x, v.y);
	}
	if (!Number.isFinite(x0)) return [0, 0, 100, 100];
	return [x0, y0, x1, y1];
}

function polyVerts(shape) {
	const raw = Array.isArray(shape?.[1]) ? shape[1] : null;
	if (!raw) return null;
	const pts = [];
	for (let i = 0; i + 1 < raw.length; i++) if (typeof raw[i] === 'number' && typeof raw[i + 1] === 'number') { pts.push([raw[i], raw[i + 1]]); i++; }
	return pts.length >= 3 ? pts : null;
}

/** EDA reports ELLIPSE pad shapes with MIXED units (first value mm, second mil, e.g. ['ELLIPSE', 1.3, 51.2] = round 51.2 mil); same normalisation as tighten-e2e-run.mjs */
function normalizeEllipseUnits(shape) {
	if (!Array.isArray(shape) || String(shape[0] ?? '').toUpperCase() !== 'ELLIPSE') return shape;
	const a = shape[1], b = shape[2];
	if (!(a > 0) || !(b > 0)) return shape;
	if (Math.abs(a * 39.3701 - b) <= Math.max(0.5, b * 0.02)) return ['ELLIPSE', b, b];
	if (Math.abs(a - b) <= 0.01 && a < 1) return ['ELLIPSE', a * 39.3701, b * 39.3701];
	return shape;
}

function padSvg(p, fill, opacity) {
	const sh = Array.isArray(p.pad) ? p.pad : null;
	const x = p.x ?? 0, y = p.y ?? 0, rot = p.rotation ?? 0;
	const name = String(sh?.[0] ?? '').toUpperCase();
	let body = '';
	const attrs = `fill="${fill}" fill-opacity="${opacity}"`;
	if (name === 'POLYGON') {
		const pts = polyVerts(sh);
		if (pts) return `<polygon points="${pts.map(q => `${f(q[0])},${f(q[1])}`).join(' ')}" ${attrs}/>` + holeSvg(p, x, y);
	}
	const nsh = normalizeEllipseUnits(sh);
	const w = nsh?.[1] > 0 ? nsh[1] : 0, h = nsh?.[2] > 0 ? nsh[2] : 0;
	// the pad is drawn with its real shape/size, the drill as-is on top (a pad smaller than its drill is shown as such, no invented annulus);
	// a pad without a usable size gets only its drill drawn
	if (!w || !h) {
		body = '';
	} else if (name === 'RECT') {
		body = `<rect x="${f(-w / 2)}" y="${f(-h / 2)}" width="${f(w)}" height="${f(h)}" ${attrs}/>`;
	} else if (name === 'ROUNDRECT') {
		const rr = Math.min(w, h) * (typeof sh[3] === 'number' && sh[3] > 0 && sh[3] <= 100 ? sh[3] / 100 : 0.25);
		body = `<rect x="${f(-w / 2)}" y="${f(-h / 2)}" width="${f(w)}" height="${f(h)}" rx="${f(rr)}" ${attrs}/>`;
	} else if (name === 'ELLIPSE' || name === 'OVAL') {
		const rr = Math.min(w, h) / 2;
		body = `<rect x="${f(-w / 2)}" y="${f(-h / 2)}" width="${f(w)}" height="${f(h)}" rx="${f(rr)}" ${attrs}/>`;
	} else {
		body = `<rect x="${f(-w / 2)}" y="${f(-h / 2)}" width="${f(w)}" height="${f(h)}" ${attrs}/>`;
	}
	return (body ? `<g transform="translate(${f(x)} ${f(y)}) rotate(${f(rot)})">${body}</g>` : '') + holeSvg(p, x, y);
}
function holeSvg(p, x, y) {
	const hole = Array.isArray(p.hole) ? p.hole : null;
	if (!hole) return '';
	const d = hole[1] > 0 ? hole[1] : 0;
	if (!(d > 0)) return '';
	const w = hole[2] > 0 ? hole[2] : d;
	if (Math.abs(w - d) < 0.01) return `<circle cx="${f(x)}" cy="${f(y)}" r="${f(d / 2)}" fill="#0b0d12"/>`;
	const rot = (p.rotation ?? 0) + (p.holeRotation ?? 0);
	return `<g transform="translate(${f(x)} ${f(y)}) rotate(${f(rot)})"><rect x="${f(-d / 2)}" y="${f(-w / 2)}" width="${f(d)}" height="${f(w)}" rx="${f(Math.min(d, w) / 2)}" fill="#0b0d12"/></g>`;
}

const COL = { 1: '#ff4d4d', 2: '#4d8dff' };

function trackSvg(l, layerOpacity) {
	const col = COL[l.layer];
	if (!col) return '';
	const w = l.lineWidth > 0 ? l.lineWidth : 10;
	return `fill="none" stroke="${col}" stroke-opacity="${layerOpacity}" stroke-width="${f(w)}" stroke-linecap="round"`;
}

export function renderSnapshot(snap, opts = {}) {
	const bb = opts.box ?? (() => { const b = boardBox(snap), m = 40; return [b[0] - m, b[1] - m, b[2] + m, b[3] + m]; })();
	const [x0, y0, x1, y1] = bb;
	const W = x1 - x0, H = y1 - y0;
	const widthPx = opts.widthPx ?? 1600;
	const heightPx = Math.max(100, Math.round((widthPx * H) / W));
	const inBox = (ax, ay, bx, by, m = 40) => !(Math.max(ax, bx) < x0 - m || Math.min(ax, bx) > x1 + m || Math.max(ay, by) < y0 - m || Math.min(ay, by) > y1 + m);
	const out = [];
	out.push(`<svg xmlns="http://www.w3.org/2000/svg" viewBox="${f(x0)} ${f(-y1)} ${f(W)} ${f(H)}" width="${widthPx}" height="${heightPx}">`);
	out.push(`<rect x="${f(x0)}" y="${f(-y1)}" width="${f(W)}" height="${f(H)}" fill="#0b0d12"/>`);
	if (opts.title) out.push(`<title>${esc(opts.title)}</title>`);
	out.push('<g transform="scale(1,-1)">');
	// outline
	out.push('<g fill="none" stroke="#e6c84a" stroke-width="3" stroke-linejoin="round">');
	for (const p of snap.polylines ?? []) {
		if ((p.layer ?? p.layerId) !== 11 || !(p.pts?.length >= 2)) continue;
		out.push(`<polyline points="${p.pts.map(q => `${f(q.x)},${f(q.y)}`).join(' ')}"/>`);
	}
	for (const l of snap.lines ?? []) if (l.layer === 11) out.push(`<line x1="${f(l.startX)}" y1="${f(l.startY)}" x2="${f(l.endX)}" y2="${f(l.endY)}"/>`);
	out.push('</g>');
	// copper fills / polylines (netless) - grey
	for (const fl of snap.fills ?? []) {
		if ((fl.layer ?? fl.layerId) !== 1 && (fl.layer ?? fl.layerId) !== 2) continue;
		if (fl.pts?.length >= 3) out.push(`<polygon points="${fl.pts.map(q => `${f(q.x)},${f(q.y)}`).join(' ')}" fill="#8890a0" fill-opacity="0.25"/>`);
	}
	for (const p of snap.polylines ?? []) {
		const ly = p.layer ?? p.layerId;
		if ((ly !== 1 && ly !== 2) || !(p.pts?.length >= 2)) continue;
		out.push(`<polyline points="${p.pts.map(q => `${f(q.x)},${f(q.y)}`).join(' ')}" fill="none" stroke="${COL[ly]}" stroke-opacity="0.45" stroke-width="${f(p.lineWidth > 0 ? p.lineWidth : 10)}"/>`);
	}
	// tracks: bottom first, then top
	for (const layer of [2, 1]) {
		out.push(`<g data-layer="${layer}">`);
		for (const l of snap.lines ?? []) {
			if (l.layer !== layer || !inBox(l.startX, l.startY, l.endX, l.endY, 60)) continue;
			out.push(`<line x1="${f(l.startX)}" y1="${f(l.startY)}" x2="${f(l.endX)}" y2="${f(l.endY)}" ${trackSvg(l, 0.62)}/>`);
		}
		for (const a of snap.arcs ?? []) {
			if (a.layer !== layer) continue;
			const c = arcCentre(a);
			if (!c) continue;
			if (!inBox(c.cx - c.r, c.cy - c.r, c.cx + c.r, c.cy + c.r, 60)) continue;
			const large = Math.abs(c.sweep) > Math.PI ? 1 : 0;
			const sweepFlag = c.sweep > 0 ? 1 : 0;
			out.push(`<path d="M ${f(a.startX)} ${f(a.startY)} A ${f(c.r)} ${f(c.r)} 0 ${large} ${sweepFlag} ${f(a.endX)} ${f(a.endY)}" ${trackSvg(a, 0.62)}/>`);
		}
		out.push('</g>');
	}
	// pads (on top of tracks, semi-transparent so the track below stays visible)
	out.push('<g>');
	for (const p of snap.pads ?? []) {
		if (!inBox(p.x, p.y, p.x, p.y, 80)) continue;
		const ly = p.layer === 12 ? 12 : (p.layer === 2 ? 2 : 1);
		const fill = ly === 12 ? '#d8c060' : (ly === 2 ? '#8ab4ff' : '#ff9a9a');
		out.push(padSvg(p, fill, 0.55));
	}
	out.push('</g>');
	// vias
	out.push('<g>');
	for (const v of snap.vias ?? []) {
		if (!inBox(v.x, v.y, v.x, v.y, 40)) continue;
		const d = v.diameter > 0 ? v.diameter : 24, hd = v.holeDiameter > 0 ? v.holeDiameter : 12;
		out.push(`<circle cx="${f(v.x)}" cy="${f(v.y)}" r="${f(d / 2)}" fill="#d9b44a" fill-opacity="0.8"/><circle cx="${f(v.x)}" cy="${f(v.y)}" r="${f(hd / 2)}" fill="#0b0d12"/>`);
	}
	out.push('</g>');
	// marks
	if (opts.marks?.length) {
		const s = Math.max(6, W / 160);
		out.push(`<g stroke="#ffffff" stroke-width="${f(s / 4)}" fill="none">`);
		for (const m of opts.marks) out.push(`<circle cx="${f(m[0])}" cy="${f(m[1])}" r="${f(s)}"/><line x1="${f(m[0] - s)}" y1="${f(m[1])}" x2="${f(m[0] + s)}" y2="${f(m[1])}"/><line x1="${f(m[0])}" y1="${f(m[1] - s)}" x2="${f(m[0])}" y2="${f(m[1] + s)}"/>`);
		out.push('</g>');
	}
	out.push('</g>');
	// label (not flipped)
	if (opts.title) out.push(`<text x="${f(x0 + W * 0.01)}" y="${f(-y1 + W * 0.018)}" font-family="monospace" font-size="${f(W * 0.014)}" fill="#c8ccd4">${esc(opts.title)}</text>`);
	out.push('</svg>');
	return out.join('\n');
}

function main() {
	const argv = process.argv.slice(2);
	if (!argv.length || argv.includes('--help') || argv.includes('-h')) {
		console.log('usage: node bench/tighten-e2e-render.mjs <snapshot.json> --out file.svg [--box x0,y0,x1,y1] [--marks "x,y;x,y"] [--title text]');
		process.exit(argv.length ? 0 : 2);
	}
	const get = (k) => { const i = argv.indexOf(k); return i >= 0 ? argv[i + 1] : undefined; };
	const snap = JSON.parse(fs.readFileSync(argv[0], 'utf8'));
	const out = get('--out');
	if (!out) { console.error('--out required'); process.exit(2); }
	const box = get('--box')?.split(',').map(Number);
	const marks = get('--marks')?.split(';').filter(Boolean).map(s => s.split(',').map(Number));
	fs.writeFileSync(out, renderSnapshot(snap, { box: box?.length === 4 ? box : undefined, marks, title: get('--title') }), 'utf8');
	console.log('wrote', out);
}
if (import.meta.url === pathToFileURL(process.argv[1] ?? '').href) main();
