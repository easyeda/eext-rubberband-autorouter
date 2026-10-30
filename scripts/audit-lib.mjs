// 公共离线审计库：DRC 间距审计 + 接缝审计（batch-route-all 与 js-pcb 对测共用）

/** 旋转矩形/POLYGON 焊盘的符号距离（>0 在外部） */
export function padRectDistance(shape, x, y, polygonToRect) {
	const pad = Array.isArray(shape?.pad) ? shape.pad : null;
	if (!pad) return null;
	let w, h, rot;
	if (String(pad[0] ?? '').toUpperCase() === 'POLYGON') {
		const rect = polygonToRect ? polygonToRect(pad) : null;
		if (!rect) return null;
		w = rect.w; h = rect.h;
		rot = (rect.rotationDeg * Math.PI) / 180;
	} else {
		if (String(pad[0] ?? '').toUpperCase() === 'ELLIPSE') return null;
		w = (pad[1] ?? 0) / 2, h = (pad[2] ?? 0) / 2;
		if (!(w > 0) || !(h > 0)) return null;
		rot = ((shape.rotation ?? 0) * Math.PI) / 180;
	}
	const hw = w / 2, hh = h / 2;
	const dx = x - (shape.px ?? 0), dy = y - (shape.py ?? 0);
	const c = Math.cos(-rot), s = Math.sin(-rot);
	const lx = dx * c - dy * s, ly = dx * s + dy * c;
	const ox = Math.abs(lx) - hw, oy = Math.abs(ly) - hh;
	if (ox > 0 && oy > 0) return Math.hypot(ox, oy);
	return Math.max(ox, oy);
}

/** 离线 DRC 间距审计（圆/胶囊近似 + 焊盘形状精复核；最终口径以 EDA DRC 为准） */
export function drcAudit(input, result, polygonToRect) {
	const drc = input.drc;
	const needTT = drc.trackToTrackMil;
	const needTP = drc.trackToPadMil;
	const CELL = 96;
	const grid = new Map();
	const key = (x, y) => `${Math.floor(x / CELL)},${Math.floor(y / CELL)}`;
	const add = (x, y, item) => {
		const k = key(x, y);
		let b = grid.get(k);
		if (!b) { b = []; grid.set(k, b); }
		b.push({ x, y, ...item });
	};
	const STEP = 2.5;
	const sampleLine = (p, net) => {
		const d = Math.hypot((p.x2 ?? 0) - (p.x1 ?? 0), (p.y2 ?? 0) - (p.y1 ?? 0));
		const n = Math.max(1, Math.ceil(d / STEP));
		for (let i = 0; i <= n; i++) {
			const t = i / n;
			add((p.x1 ?? 0) + ((p.x2 ?? 0) - (p.x1 ?? 0)) * t, (p.y1 ?? 0) + ((p.y2 ?? 0) - (p.y1 ?? 0)) * t, { kind: 'trk', net, layer: p.layer });
		}
	};
	const sampleArc = (p, net) => {
		const r = p.r ?? 0, sw = p.sweep ?? 0;
		const n = Math.max(2, Math.ceil(Math.abs(sw) * r / STEP));
		for (let i = 0; i <= n; i++) {
			const a = (p.startAngle ?? 0) + (sw * i) / n;
			add((p.cx ?? 0) + r * Math.cos(a), (p.cy ?? 0) + r * Math.sin(a), { kind: 'trk', net, layer: p.layer });
		}
	};
	for (const n of result.nets) {
		for (const p of n.prims) {
			if (p.kind === 'line') sampleLine(p, n.name); else sampleArc(p, n.name);
		}
	}
	for (const p of input.allPads) {
		add(p.x, p.y, { kind: 'pad', net: p.net, r: p.radius, layer: p.layer === 12 ? undefined : p.layer, shape: input.padShapes[`${p.x.toFixed(2)},${p.y.toFixed(2)}`] ?? null });
	}
	for (const v of result.vias ?? []) {
		add(v.x, v.y, { kind: 'via', net: v.net, r: v.dia / 2, layer: undefined });
	}
	const pairMin = new Map();
	const consider = (na, nb, d, x, y, ka, kb) => {
		if (na === nb) return;
		if (ka === 'pad' && kb === 'pad') return;
		const k = na < nb ? `${na}|${nb}` : `${nb}|${na}`;
		const kinds = na < nb ? `${ka}-${kb}` : `${kb}-${ka}`;
		const cur = pairMin.get(k);
		if (!cur || d < cur.min) pairMin.set(k, { min: d, x, y, kinds });
	};
	for (const b of grid.values()) {
		for (let i = 0; i < b.length; i++) {
			const A = b[i];
			for (let j = i + 1; j < b.length; j++) checkPair(A, b[j]);
			const gx = Math.floor(A.x / CELL), gy = Math.floor(A.y / CELL);
			for (const [dx, dy] of [[1, 0], [0, 1], [1, 1], [1, -1]]) {
				const nb2 = grid.get(`${gx + dx},${gy + dy}`);
				if (nb2) for (const B of nb2) checkPair(A, B);
			}
		}
	}
	function checkPair(A, B) {
		if (A.net === B.net) return;
		if (A.layer !== undefined && B.layer !== undefined && A.layer !== B.layer) return;
		let d = Math.hypot(A.x - B.x, A.y - B.y) - (A.kind === 'trk' ? 0 : A.r) - (B.kind === 'trk' ? 0 : B.r);
		if (d > needTP + 2) return;
		if (A.kind === 'pad' && A.shape) {
			const exact = padRectDistance({ ...A.shape, px: A.x, py: A.y }, B.x, B.y, polygonToRect);
			if (exact !== null) d = exact;
		} else if (B.kind === 'pad' && B.shape) {
			const exact = padRectDistance({ ...B.shape, px: B.x, py: B.y }, A.x, A.y, polygonToRect);
			if (exact !== null) d = exact;
		}
		consider(A.net, B.net, d, (A.x + B.x) / 2, (A.y + B.y) / 2, A.kind, B.kind);
	}
	const violations = [];
	for (const [k, v] of pairMin) {
		const [na, nb] = k.split('|');
		const bothTrk = v.kinds === 'trk-trk';
		const need = bothTrk ? needTT : needTP;
		if (v.min < need - 0.5) violations.push({ netA: na, netB: nb, kind: v.kinds, min: Math.round(v.min * 10) / 10, need: Math.round(need * 100) / 100, x: Math.round(v.x), y: Math.round(v.y) });
	}
	violations.sort((a, b) => a.min - b.min);
	return { violations: violations.slice(0, 80), violationCount: violations.length };
}

/** 接缝审计（开口/锐角/近整圆弧） */
export function jointAudit(result) {
	const TAU = Math.PI * 2;
	const ptOf = (p, end) => {
		if (p.kind === 'line') return end ? { x: p.x2, y: p.y2 } : { x: p.x1, y: p.y1 };
		const a = (p.startAngle ?? 0) + (end ? (p.sweep ?? 0) : 0);
		return { x: (p.cx ?? 0) + (p.r ?? 0) * Math.cos(a), y: (p.cy ?? 0) + (p.r ?? 0) * Math.sin(a) };
	};
	const tanOf = (p, end) => {
		if (p.kind === 'line') {
			const dx = (p.x2 ?? 0) - (p.x1 ?? 0), dy = (p.y2 ?? 0) - (p.y1 ?? 0);
			const l = Math.hypot(dx, dy) || 1;
			return { x: dx / l, y: dy / l };
		}
		const a = (p.startAngle ?? 0) + (end ? (p.sweep ?? 0) : 0);
		const s = (p.sweep ?? 0) >= 0 ? 1 : -1;
		return s > 0 ? { x: -Math.sin(a), y: Math.cos(a) } : { x: Math.sin(a), y: -Math.cos(a) };
	};
	const angDiff = (a, b) => {
		let d = Math.abs(Math.atan2(a.y, a.x) - Math.atan2(b.y, b.x)) % TAU;
		if (d > Math.PI) d = TAU - d;
		return d * 180 / Math.PI;
	};
	const issues = [];
	let joints = 0, gaps = 0, sharp = 0, bigArcs = 0;
	for (const net of result.nets) {
		const prims = net.prims;
		if (prims.length < 2) continue;
		for (let i = 0; i < prims.length - 1; i++) {
			const a = prims[i], b = prims[i + 1];
			const ae = ptOf(a, true), bs = ptOf(b, false);
			const gap = Math.hypot(ae.x - bs.x, ae.y - bs.y);
			if (gap > 15) continue;
			joints++;
			if (gap > 0.5) { gaps++; issues.push({ net: net.name, type: 'gap', mil: Math.round(gap * 10) / 10, at: [Math.round(ae.x), Math.round(ae.y)] }); }
			const turn = angDiff(tanOf(a, true), tanOf(b, false));
			if (turn > 30) { sharp++; issues.push({ net: net.name, type: 'sharp', deg: Math.round(turn), at: [Math.round(ae.x), Math.round(ae.y)] }); }
		}
		for (const p of prims) {
			if (p.kind === 'arc' && Math.abs((p.sweep ?? 0) * 180 / Math.PI) > 350) { bigArcs++; issues.push({ net: net.name, type: 'nearFullCircle' }); }
		}
	}
	return { joints, gaps, sharp, bigArcs, issues: issues.slice(0, 60) };
}
