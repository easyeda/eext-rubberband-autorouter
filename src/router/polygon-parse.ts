/**
 * EasyEDA 多边形源（TPCB_PolygonSourceArray）→ 世界坐标点列。
 *
 * 格式（easyeda-api-skill format/pcb/shape.md）：
 *  - `X Y L X Y ...`            直线路径
 *  - `X Y ARC angle X Y`        圆弧段（弧角 + 终点）
 *  - `X Y CARC angle X Y`       圆心圆弧段
 *  - `X Y C c1x c1y c2x c2y x y` 三次贝塞尔
 *  - `["R", X, Y, w, h, rot, round]` 旋转矩形
 *  - `["CIRCLE", cx, cy, r, isCCW]`  圆
 *
 * ⚠ 历史 bug（2026-10-29）：旧实现处理了 "R" 但**忽略旋转角**，把旋转 90°
 * 的板框放到板外（y -2719..-189），真实板边无障碍 → 导线飞出板框、过孔贴边。
 * 本模块是 E2E 收集器与离线 dump 驱动的唯一实现（口径必须一致）。
 */
export function parsePolygonPath(polygon: any[]): Array<{ x: number; y: number }> {
	if (!Array.isArray(polygon) || polygon.length < 3) return [];
	const head = typeof polygon[0] === 'string' ? String(polygon[0]).toUpperCase() : '';

	// 旋转矩形：局部角点 (0,0),(w,0),(w,-h),(0,-h) 绕 (X,Y) 按 rot 逆时针旋转
	if (head === 'R' && polygon.length >= 5) {
		const X = Number(polygon[1]), Y = Number(polygon[2]);
		const w = Number(polygon[3]), h = Number(polygon[4]);
		const rot = ((Number(polygon[5]) || 0) * Math.PI) / 180;
		const c = Math.cos(rot), s = Math.sin(rot);
		const toWorld = (dx: number, dy: number) => ({ x: X + dx * c - dy * s, y: Y + dx * s + dy * c });
		const pts = [toWorld(0, 0), toWorld(w, 0), toWorld(w, -h), toWorld(0, -h)];
		pts.push({ ...pts[0] });
		return pts;
	}
	if (head === 'CIRCLE' && polygon.length >= 4) {
		const cx = Number(polygon[1]), cy = Number(polygon[2]), r = Number(polygon[3]);
		const pts: Array<{ x: number; y: number }> = [];
		for (let k = 0; k <= 24; k++) {
			const a = (k / 24) * Math.PI * 2;
			pts.push({ x: cx + r * Math.cos(a), y: cy + r * Math.sin(a) });
		}
		return pts;
	}

	// 路径模式
	const pts: Array<{ x: number; y: number }> = [];
	const ARC_STEP = Math.PI / 12;   // 弧采样 ≤15°/段
	let i = 0;
	let curX = 0, curY = 0;
	while (i < polygon.length) {
		const tok = polygon[i];
		if (typeof tok === 'number' && typeof polygon[i + 1] === 'number') {
			curX = tok; curY = polygon[i + 1];
			pts.push({ x: curX, y: curY });
			i += 2;
			continue;
		}
		const kind = typeof tok === 'string' ? tok.toUpperCase() : '';
		if (kind === 'L') { i += 1; continue; }
		if (kind === 'C') {
			const c1x = Number(polygon[i + 1]), c1y = Number(polygon[i + 2]);
			const c2x = Number(polygon[i + 3]), c2y = Number(polygon[i + 4]);
			const ex = Number(polygon[i + 5]), ey = Number(polygon[i + 6]);
			const sx = curX, sy = curY;
			for (let k = 1; k <= 8; k++) {
				const t = k / 8, u = 1 - t;
				pts.push({
					x: u * u * u * sx + 3 * u * u * t * c1x + 3 * u * t * t * c2x + t * t * t * ex,
					y: u * u * u * sy + 3 * u * u * t * c1y + 3 * u * t * t * c2y + t * t * t * ey,
				});
			}
			curX = ex; curY = ey;
			i += 7;
			continue;
		}
		if (kind === 'ARC' || kind === 'CARC') {
			const angle = ((Number(polygon[i + 1]) || 0) * Math.PI) / 180;
			const ex = Number(polygon[i + 2]), ey = Number(polygon[i + 3]);
			const sx = curX, sy = curY;
			const chord = Math.hypot(ex - sx, ey - sy);
			if (chord > 1e-6 && Math.abs(Math.sin(angle / 2)) > 1e-6) {
				// 由弦长 + 扫角反推圆心，沿扫向采样
				const R = chord / (2 * Math.abs(Math.sin(angle / 2)));
				const sgn = angle > 0 ? 1 : -1;
				const hh = sgn * R * Math.cos(angle / 2);
				const nx = -(ey - sy) / chord, ny = (ex - sx) / chord;
				const ccx = (sx + ex) / 2 + hh * nx, ccy = (sy + ey) / 2 + hh * ny;
				const a0 = Math.atan2(sy - ccy, sx - ccx);
				const steps = Math.max(2, Math.ceil(Math.abs(angle) / ARC_STEP));
				for (let k = 1; k <= steps; k++) {
					const a = a0 + (angle * k) / steps;
					pts.push({ x: ccx + R * Math.cos(a), y: ccy + R * Math.sin(a) });
				}
			} else {
				pts.push({ x: ex, y: ey });
			}
			curX = ex; curY = ey;
			i += 4;
			continue;
		}
		i += 1;
	}
	return pts;
}
