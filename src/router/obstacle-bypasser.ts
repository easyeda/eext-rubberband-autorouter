/**
 * 障碍物绕行算法
 * 生成绕过障碍物的路径点
 */

interface Point {
	x: number;
	y: number;
	arcAngle?: number;
}

interface Obstacle {
	id?: string;
	x: number;
	y: number;
	radius: number;
	type: 'pad' | 'via' | 'track' | 'keepout';
	net?: string;
	layer?: number;
	// 线段障碍物（用于走线）
	startX?: number;
	startY?: number;
	endX?: number;
	endY?: number;
}

interface PathSegment {
	start: Point;
	end: Point;
	viaObstacle?: Obstacle; // 如果经过障碍物附近，记录该障碍物
}

/**
 * 障碍物绕行器
 * 在直线路径被阻挡时，生成绕行点
 */
export class ObstacleBypasser {
	private clearanceMil: number;
	private trackWidthMil: number;

	constructor(clearanceMil: number, trackWidthMil: number) {
		this.clearanceMil = clearanceMil;
		this.trackWidthMil = trackWidthMil;
	}

	/**
	 * 检查线段是否与障碍物相交
	 */
	private segmentIntersectsObstacle(
		start: Point,
		end: Point,
		obstacle: Obstacle,
		margin: number = 0
	): boolean {
		const requiredGap = obstacle.radius + margin;

		// 如果是线段障碍物（走线），使用线段到线段距离
		if (obstacle.startX !== undefined && obstacle.startY !== undefined &&
			obstacle.endX !== undefined && obstacle.endY !== undefined) {
			
			const dist = this.segmentToSegmentDistance(
				start.x, start.y, end.x, end.y,
				obstacle.startX, obstacle.startY,
				obstacle.endX, obstacle.endY
			);
			
			return dist < requiredGap;
		}
		
		// 否则使用点到线段距离（圆形障碍物）
		const dist = this.pointToSegmentDistance(
			obstacle.x, obstacle.y,
			start.x, start.y,
			end.x, end.y
		);

		return dist < requiredGap;
	}

	/**
	 * 计算两条线段之间的最短距离
	 */
	private segmentToSegmentDistance(
		x1: number, y1: number, x2: number, y2: number,
		x3: number, y3: number, x4: number, y4: number
	): number {
		// 检查线段是否相交
		if (this.segmentsIntersect(x1, y1, x2, y2, x3, y3, x4, y4)) {
			return 0;
		}

		// 计算各端点到另一条线段的距离，取最小值
		const d1 = this.pointToSegmentDistance(x1, y1, x3, y3, x4, y4);
		const d2 = this.pointToSegmentDistance(x2, y2, x3, y3, x4, y4);
		const d3 = this.pointToSegmentDistance(x3, y3, x1, y1, x2, y2);
		const d4 = this.pointToSegmentDistance(x4, y4, x1, y1, x2, y2);

		return Math.min(d1, d2, d3, d4);
	}

	/**
	 * 检查两条线段是否相交
	 */
	private segmentsIntersect(
		x1: number, y1: number, x2: number, y2: number,
		x3: number, y3: number, x4: number, y4: number
	): boolean {
		const ccw = (ax: number, ay: number, bx: number, by: number, cx: number, cy: number): number => {
			return (cy - ay) * (bx - ax) - (by - ay) * (cx - ax);
		};

		const d1 = ccw(x3, y3, x4, y4, x1, y1);
		const d2 = ccw(x3, y3, x4, y4, x2, y2);
		const d3 = ccw(x1, y1, x2, y2, x3, y3);
		const d4 = ccw(x1, y1, x2, y2, x4, y4);

		if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
			((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) {
			return true;
		}

		// 检查共线情况
		const onSegment = (ax: number, ay: number, bx: number, by: number, cx: number, cy: number): boolean => {
			return Math.min(ax, cx) <= bx && bx <= Math.max(ax, cx) &&
				Math.min(ay, cy) <= by && by <= Math.max(ay, cy);
		};

		if (d1 === 0 && onSegment(x3, y3, x4, y4, x1, y1)) return true;
		if (d2 === 0 && onSegment(x3, y3, x4, y4, x2, y2)) return true;
		if (d3 === 0 && onSegment(x1, y1, x2, y2, x3, y3)) return true;
		if (d4 === 0 && onSegment(x1, y1, x2, y2, x4, y4)) return true;

		return false;
	}

	/**
	 * 找到与线段相交的障碍物
	 */
	private findIntersectingObstacles(
		start: Point,
		end: Point,
		obstacles: Obstacle[],
		excludeNet?: string
	): Obstacle[] {
		const intersecting: Obstacle[] = [];
		const margin = this.trackWidthMil / 2;

		// 获取线段的层号
		const startLayer = (start as any).layer;
		const endLayer = (end as any).layer;

		for (const obs of obstacles) {
			if (excludeNet && obs.net === excludeNet) continue;

			// 层号检查：只检查同层的障碍物
			if (obs.layer !== undefined) {
				if (startLayer !== undefined && obs.layer !== startLayer && obs.layer !== 12 && startLayer !== 12) {
					continue;
				}
				if (endLayer !== undefined && obs.layer !== endLayer && obs.layer !== 12 && endLayer !== 12) {
					continue;
				}
			}

			if (this.segmentIntersectsObstacle(start, end, obs, margin)) {
				intersecting.push(obs);
			}
		}

		return intersecting;
	}

	/**
	 * 生成绕行点
	 * 优先使用切线+圆弧绕行，失败时回退到直角绕行
	 */
	private generateBypassPoints(
		start: Point,
		end: Point,
		obstacle: Obstacle,
		direction: 'left' | 'right',
		offsetMultiplier: number = 1
	): Point[] {
		// 优先尝试切线+圆弧绕行
		const tangentResult = this.generateTangentBypass(start, end, obstacle, direction);
		if (tangentResult && tangentResult.length > 0) {
			return tangentResult;
		}

		// 回退到直角绕行
		if (obstacle.startX !== undefined && obstacle.startY !== undefined &&
			obstacle.endX !== undefined && obstacle.endY !== undefined) {
			return this.generateSegmentBypassPoints(start, end, obstacle, direction, offsetMultiplier);
		}
		return this.generateCircleBypassPoints(start, end, obstacle, direction);
	}

	/**
	 * 切线+圆弧绕行算法
	 * 圆形障碍物：计算外切线切点 + 圆弧
	 * 线段障碍物：绕过端点，切线必须清除整条线段
	 */
	private generateTangentBypass(
		start: Point,
		end: Point,
		obstacle: Obstacle,
		direction: 'left' | 'right'
	): Point[] | null {
		const margin = this.trackWidthMil / 2 + 3;

		if (obstacle.startX !== undefined && obstacle.endX !== undefined) {
			return this.generateSegmentTangentBypass(start, end, obstacle, direction);
		}

		// 圆形障碍物
		const cx = obstacle.x;
		const cy = obstacle.y;
		const R = obstacle.radius + margin;

		return this.computeTangentArc(start, end, cx, cy, R, direction);
	}

	/**
	 * 线段障碍物的切线绕行
	 * 线段障碍物形成"胶囊体"：两条平行偏移线 + 两个端点半圆
	 * 路径：起点 → 切线到偏移线 → 沿偏移线到端点 → 圆弧绕端点 → 沿偏移线 → 切线到终点
	 */
	private generateSegmentTangentBypass(
		start: Point,
		end: Point,
		obstacle: Obstacle,
		direction: 'left' | 'right'
	): Point[] | null {
		const margin = this.trackWidthMil / 2 + 3;
		const sx = obstacle.startX!, sy = obstacle.startY!;
		const ex = obstacle.endX!, ey = obstacle.endY!;
		const R = obstacle.radius + margin;

		const segDx = ex - sx, segDy = ey - sy;
		const segLen = Math.sqrt(segDx * segDx + segDy * segDy);
		if (segLen < 0.001) return null;
		const segUx = segDx / segLen, segUy = segDy / segLen;
		const nx = -segUy, ny = segUx;

		// 判断起点和终点在线段的哪一侧
		const startSide = (start.x - sx) * nx + (start.y - sy) * ny;
		const endSide = (end.x - sx) * nx + (end.y - sy) * ny;

		if ((startSide > 0) === (endSide > 0)) {
			// 同侧：沿法向偏移线直接通过
			const sign = startSide > 0 ? 1 : -1;
			const osx = sx + nx * sign * R, osy = sy + ny * sign * R;
			const oex = ex + nx * sign * R, oey = ey + ny * sign * R;
			const t1 = this.projectOntoLine(start.x, start.y, osx, osy, oex, oey);
			const t2 = this.projectOntoLine(end.x, end.y, osx, osy, oex, oey);
			return [
				{ x: osx + t1 * (oex - osx), y: osy + t1 * (oey - osy) },
				{ x: osx + t2 * (oex - osx), y: osy + t2 * (oey - osy) },
			];
		}

		// 不同侧：需要绕过端点
		// 起点先水平/垂直到偏移线，沿偏移线到端点，圆弧绕过，再回到终点
		const startSign = startSide > 0 ? 1 : -1;
		const endSign = endSide > 0 ? 1 : -1;

		let bestResult: Point[] | null = null;
		let bestLen = Infinity;

		for (const ep of [
			{ tipX: sx, tipY: sy, awayX: -segUx, awayY: -segUy },
			{ tipX: ex, tipY: ey, awayX: segUx, awayY: segUy },
		]) {
			// 起点侧偏移线：平行于线段，偏移 R
			const offSX = nx * startSign * R;
			const offSY = ny * startSign * R;
			// 终点侧偏移线
			const offEX = nx * endSign * R;
			const offEY = ny * endSign * R;

			// 切入点：从起点垂直投影到起点侧偏移线
			const entryX = start.x + offSX - (start.x - sx) * (1 - Math.abs(nx));
			const entryY = start.y + offSY - (start.y - sy) * (1 - Math.abs(ny));
			// 简化：对竖线 nx=1,ny=0 → entry=(sx+R*sign, start.y)
			// 对横线 nx=0,ny=1 → entry=(start.x, sy+R*sign)
			const eX = sx + offSX + segUx * ((start.x - sx) * segUx + (start.y - sy) * segUy);
			const eY = sy + offSY + segUy * ((start.x - sx) * segUx + (start.y - sy) * segUy);

			// 切出点：从终点垂直投影到终点侧偏移线
			const xX = sx + offEX + segUx * ((end.x - sx) * segUx + (end.y - sy) * segUy);
			const xY = sy + offEY + segUy * ((end.x - sx) * segUx + (end.y - sy) * segUy);

			// 圆弧起点/终点
			const arcSX = ep.tipX + offSX;
			const arcSY = ep.tipY + offSY;
			const arcEX = ep.tipX + offEX;
			const arcEY = ep.tipY + offEY;

			// 圆弧角度
			const a1 = Math.atan2(arcSY - ep.tipY, arcSX - ep.tipX);
			const a2 = Math.atan2(arcEY - ep.tipY, arcEX - ep.tipX);
			let arcSpan = a2 - a1;
			while (arcSpan > Math.PI) arcSpan -= 2 * Math.PI;
			while (arcSpan < -Math.PI) arcSpan += 2 * Math.PI;

			const arcMidAngle = a1 + arcSpan / 2;
			const arcMidX = ep.tipX + R * Math.cos(arcMidAngle);
			const arcMidY = ep.tipY + R * Math.sin(arcMidAngle);
			const midToAway = (arcMidX - ep.tipX) * ep.awayX + (arcMidY - ep.tipY) * ep.awayY;
			if (midToAway < 0) {
				arcSpan = arcSpan > 0 ? arcSpan - 2 * Math.PI : arcSpan + 2 * Math.PI;
			}
			if (Math.abs(arcSpan) > Math.PI * 1.1) continue;

			const points: Point[] = [
				{ x: eX, y: eY },
				{ x: arcSX, y: arcSY },
				{ x: arcEX, y: arcEY, arcAngle: arcSpan },
				{ x: xX, y: xY },
			];

			const fullPath = [start, ...points, end];
			const len = this.calculatePathLength(fullPath);
			if (len < bestLen) {
				bestLen = len;
				bestResult = points;
			}
		}

		return bestResult;
	}

	/**
	 * 计算点到线段的投影参数 t (0-1)
	 */
	private projectOntoLine(px: number, py: number, x1: number, y1: number, x2: number, y2: number): number {
		const dx = x2 - x1, dy = y2 - y1;
		const len2 = dx * dx + dy * dy;
		if (len2 === 0) return 0;
		return Math.max(0, Math.min(1, ((px - x1) * dx + (py - y1) * dy) / len2));
	}

	/**
	 * 通用切线+圆弧计算
	 * 从 start 和 end 到圆 (cx, cy, R) 的外切线，切点间用圆弧连接
	 */
	private computeTangentArc(
		start: Point,
		end: Point,
		cx: number, cy: number, R: number,
		direction: 'left' | 'right'
	): Point[] | null {
		const dx1 = start.x - cx, dy1 = start.y - cy;
		const d1 = Math.sqrt(dx1 * dx1 + dy1 * dy1);
		const dx2 = end.x - cx, dy2 = end.y - cy;
		const d2 = Math.sqrt(dx2 * dx2 + dy2 * dy2);

		if (d1 <= R || d2 <= R) return null;

		const alpha1 = Math.asin(R / d1);
		const baseAngle1 = Math.atan2(cy - start.y, cx - start.x);
		const alpha2 = Math.asin(R / d2);
		const baseAngle2 = Math.atan2(cy - end.y, cx - end.x);

		const sign = direction === 'left' ? -1 : 1;

		// 切点 T1（起点侧）
		const t1Angle = baseAngle1 + sign * alpha1 + Math.PI + sign * (Math.PI / 2 - alpha1);
		const t1x = cx + R * Math.cos(t1Angle);
		const t1y = cy + R * Math.sin(t1Angle);

		// 切点 T2（终点侧）
		const t2Angle = baseAngle2 - sign * alpha2 + Math.PI - sign * (Math.PI / 2 - alpha2);
		const t2x = cx + R * Math.cos(t2Angle);
		const t2y = cy + R * Math.sin(t2Angle);

		// 圆弧角度
		let arcSpan = t2Angle - t1Angle;
		while (arcSpan > Math.PI) arcSpan -= 2 * Math.PI;
		while (arcSpan < -Math.PI) arcSpan += 2 * Math.PI;
		if (sign > 0 && arcSpan < 0) arcSpan += 2 * Math.PI;
		if (sign < 0 && arcSpan > 0) arcSpan -= 2 * Math.PI;

		if (Math.abs(arcSpan) > Math.PI) return null;

		const distT1 = Math.sqrt((t1x - start.x) ** 2 + (t1y - start.y) ** 2);
		const distT2 = Math.sqrt((t2x - end.x) ** 2 + (t2y - end.y) ** 2);
		if (distT1 < 1 || distT2 < 1) return null;

		return [
			{ x: t1x, y: t1y },
			{ x: t2x, y: t2y, arcAngle: arcSpan },
		];
	}

	/**
	 * 线段障碍物绕行：选择绕过更近的端点，沿垂直方向偏移
	 */
	private generateSegmentBypassPoints(
		start: Point,
		end: Point,
		obstacle: Obstacle,
		direction: 'left' | 'right',
		offsetMultiplier: number = 1
	): Point[] {
		const margin = this.trackWidthMil / 2 + 5;

		const sx = obstacle.startX!, sy = obstacle.startY!;
		const ex = obstacle.endX!, ey = obstacle.endY!;

		const segDx = ex - sx;
		const segDy = ey - sy;
		const segLen = Math.sqrt(segDx * segDx + segDy * segDy);
		if (segLen < 0.001) return [];

		const segUx = segDx / segLen;
		const segUy = segDy / segLen;

		const offset = (obstacle.radius + margin) * offsetMultiplier;

		// "left" 绕过起点端，"right" 绕过终点端
		let tipX: number, tipY: number, extDirX: number, extDirY: number;
		if (direction === 'left') {
			tipX = sx; tipY = sy;
			extDirX = -segUx; extDirY = -segUy;
		} else {
			tipX = ex; tipY = ey;
			extDirX = segUx; extDirY = segUy;
		}

		// 端点外侧的 y/x 坐标（绕行基准线）
		const beyondX = tipX + extDirX * offset;
		const beyondY = tipY + extDirY * offset;

		// 生成两个绕行点：
		// 点1：从起点垂直/水平移动到绕行基准线
		// 点2：从绕行基准线移动到终点方向
		// 这样路径不会穿过线段
		const isVertical = Math.abs(segDx) < Math.abs(segDy);

		if (isVertical) {
			// 竖线：绕行点在端点上方或下方，保持各自的 x 坐标
			return [
				{ x: start.x, y: beyondY },
				{ x: end.x, y: beyondY },
			];
		} else {
			// 横线：绕行点在端点左侧或右侧，保持各自的 y 坐标
			return [
				{ x: beyondX, y: start.y },
				{ x: beyondX, y: end.y },
			];
		}
	}

	/**
	 * 圆形障碍物绕行（原有逻辑）
	 */
	private generateCircleBypassPoints(
		start: Point,
		end: Point,
		obstacle: Obstacle,
		direction: 'left' | 'right'
	): Point[] {
		const margin = this.trackWidthMil / 2 + 3;
		const bypassRadius = obstacle.radius + margin;

		// 计算从起点到障碍物中心的向量
		const dx = obstacle.x - start.x;
		const dy = obstacle.y - start.y;
		const dist = Math.sqrt(dx * dx + dy * dy);

		if (dist < 0.001) return [];

		// 单位向量
		const ux = dx / dist;
		const uy = dy / dist;

		// 计算入射角
		const angle = Math.atan2(uy, ux);

		// 绕行角度（45 度）
		const bypassAngle = Math.PI / 4;

		// 根据方向选择左侧或右侧绕行
		const finalAngle = direction === 'left' 
			? angle - bypassAngle 
			: angle + bypassAngle;

		// 计算绕行点
		const bypassDist = bypassRadius / Math.cos(bypassAngle);
		const bypassX = start.x + Math.cos(finalAngle) * bypassDist;
		const bypassY = start.y + Math.sin(finalAngle) * bypassDist;

		// 计算出射点（对称）
		const dx2 = end.x - obstacle.x;
		const dy2 = end.y - obstacle.y;
		const dist2 = Math.sqrt(dx2 * dx2 + dy2 * dy2);

		if (dist2 < 0.001) return [{ x: bypassX, y: bypassY }];

		const ux2 = dx2 / dist2;
		const uy2 = dy2 / dist2;
		const angle2 = Math.atan2(uy2, ux2);

		const exitAngle = direction === 'left'
			? angle2 + bypassAngle
			: angle2 - bypassAngle;

		const exitX = obstacle.x + Math.cos(exitAngle) * bypassRadius;
		const exitY = obstacle.y + Math.sin(exitAngle) * bypassRadius;

		return [
			{ x: bypassX, y: bypassY },
			{ x: exitX, y: exitY }
		];
	}

	/**
	 * 计算点到线段的最短距离
	 */
	private pointToSegmentDistance(
		px: number, py: number,
		x1: number, y1: number,
		x2: number, y2: number
	): number {
		const dx = x2 - x1;
		const dy = y2 - y1;
		const len2 = dx * dx + dy * dy;

		if (len2 === 0) {
			return Math.sqrt((px - x1) ** 2 + (py - y1) ** 2);
		}

		let t = ((px - x1) * dx + (py - y1) * dy) / len2;
		t = Math.max(0, Math.min(1, t));

		const projX = x1 + t * dx;
		const projY = y1 + t * dy;

		return Math.sqrt((px - projX) ** 2 + (py - projY) ** 2);
	}

	/**
	 * 检查线段是否安全（无碰撞）
	 */
	isSegmentSafe(
		start: Point,
		end: Point,
		obstacles: Obstacle[],
		excludeNet?: string
	): boolean {
		const intersecting = this.findIntersectingObstacles(start, end, obstacles, excludeNet);
		return intersecting.length === 0;
	}

	/**
	 * 生成绕行路径
	 * @param start 起点
	 * @param end 终点
	 * @param obstacles 障碍物列表
	 * @param excludeNet 排除的网络
	 * @returns 绕行路径点列表，如果无法绕行返回 null
	 */
	generateBypassPath(
		start: Point,
		end: Point,
		obstacles: Obstacle[],
		excludeNet?: string
	): Point[] | null {
		console.log(`[Bypass] 开始生成绕行路径：(${start.x.toFixed(0)}, ${start.y.toFixed(0)}) -> (${end.x.toFixed(0)}, ${end.y.toFixed(0)})`);
		
		// 首先检查直接路径是否可行
		if (this.isSegmentSafe(start, end, obstacles, excludeNet)) {
			console.log('[Bypass] 直接路径安全，无需绕行');
			return [start, end];
		}

		// 找到阻挡的障碍物
		const blocking = this.findIntersectingObstacles(start, end, obstacles, excludeNet);
		console.log(`[Bypass] 找到 ${blocking.length} 个阻挡障碍物`);
		
		if (blocking.length === 0) {
			console.log('[Bypass] 无障碍物，返回直接路径');
			return [start, end];
		}
		
		// 记录阻挡的障碍物信息
		blocking.forEach((obs, i) => {
			if (obs.startX !== undefined) {
				console.log(`  障碍物 ${i + 1}: 线段 (${obs.startX.toFixed(0)},${(obs.startY ?? 0).toFixed(0)}) -> (${(obs.endX ?? 0).toFixed(0)},${(obs.endY ?? 0).toFixed(0)})`);
			} else {
				console.log(`  障碍物 ${i + 1}: 圆形 (${obs.x.toFixed(0)},${obs.y.toFixed(0)}) r=${obs.radius.toFixed(0)}`);
			}
		});

		// 尝试 1: 切线+圆弧绕行（橡皮筋效果）
		const mainObstacle = blocking[0];
		let bestTangentPath: Point[] | null = null;
		let bestTangentLen = Infinity;

		for (const dir of ['left', 'right'] as const) {
			const tangentPoints = this.generateTangentBypass(start, end, mainObstacle, dir);
			if (tangentPoints && tangentPoints.length > 0) {
				const tangentPath = [start, ...tangentPoints, end];
				if (this.isPathSafe(tangentPath, obstacles, excludeNet)) {
					const len = this.calculatePathLength(tangentPath);
					if (len < bestTangentLen) {
						bestTangentLen = len;
						bestTangentPath = tangentPath;
					}
				}
			}
		}

		if (bestTangentPath) {
			console.log('[Bypass] 切线+圆弧绕行成功');
			return bestTangentPath;
		}

		// 尝试 2: 直角绕行 + 扩展
		for (const dir of ['left', 'right'] as const) {
			console.log(`[Bypass] 尝试${dir === 'left' ? '左' : '右'}侧绕行...`);
			const isSegment = mainObstacle.startX !== undefined && mainObstacle.endX !== undefined;
			const bypassFn = isSegment
				? () => this.generateSegmentBypassPoints(start, end, mainObstacle, dir)
				: () => this.generateCircleBypassPoints(start, end, mainObstacle, dir);
			const points = bypassFn();
			if (points.length > 0) {
				const path = [start, ...points, end];
				if (this.isPathSafe(path, obstacles, excludeNet)) {
					console.log(`[Bypass] ${dir === 'left' ? '左' : '右'}侧绕行成功`);
					return path;
				}
				const expanded = this.expandBypassToAvoidAll(path, obstacles, excludeNet, mainObstacle, dir);
				if (expanded && this.isPathSafe(expanded, obstacles, excludeNet)) {
					console.log(`[Bypass] ${dir === 'left' ? '左' : '右'}侧扩展绕行成功`);
					return expanded;
				}
				console.log(`[Bypass] ${dir === 'left' ? '左' : '右'}侧绕行失败：路径不安全`);
			}
		}

		// 如果单点绕行失败，尝试多点绕行
		console.log('[Bypass] 尝试多点绕行...');
		const multiPath = this.generateMultiPointBypass(start, end, obstacles, excludeNet);
		if (multiPath) {
			console.log('[Bypass] 多点绕行成功');
			return multiPath;
		}

		// 最后手段：U 型大幅绕行
		console.log('[Bypass] 尝试 U 型大幅绕行...');
		const uPath = this.generateUShapeBypass(start, end, obstacles, excludeNet);
		if (uPath) {
			console.log('[Bypass] U 型绕行成功');
			return uPath;
		}

		console.log('[Bypass] 所有绕行尝试失败，返回 null');
		return null;
	}

	/**
	 * 检查整条路径是否安全
	 * 支持圆弧段：对有 arcAngle 的点，采样弧上的点检查
	 */
	private isPathSafe(
		path: Point[],
		obstacles: Obstacle[],
		excludeNet?: string
	): boolean {
		for (let i = 0; i < path.length - 1; i++) {
			const p1 = path[i];
			const p2 = path[i + 1];

			if (p2.arcAngle) {
				// 圆弧段：采样弧上的点检查
				if (!this.isArcSafe(p1, p2, p2.arcAngle, obstacles, excludeNet)) {
					return false;
				}
			} else {
				if (!this.isSegmentSafe(p1, p2, obstacles, excludeNet)) {
					return false;
				}
			}
		}
		return true;
	}

	/**
	 * 检查圆弧段是否安全
	 * 将圆弧采样为多段小线段检查
	 */
	private isArcSafe(
		start: Point,
		end: Point,
		arcAngle: number,
		obstacles: Obstacle[],
		excludeNet?: string
	): boolean {
		const samples = this.sampleArc(start, end, arcAngle, 8);
		for (let i = 0; i < samples.length - 1; i++) {
			if (!this.isSegmentSafe(samples[i], samples[i + 1], obstacles, excludeNet)) {
				return false;
			}
		}
		return true;
	}

	/**
	 * 将圆弧采样为点序列
	 */
	private sampleArc(start: Point, end: Point, arcAngle: number, numSamples: number): Point[] {
		// 从起点、终点和弧度角反算圆心
		const mx = (start.x + end.x) / 2;
		const my = (start.y + end.y) / 2;
		const dx = end.x - start.x;
		const dy = end.y - start.y;
		const chordLen = Math.sqrt(dx * dx + dy * dy);

		if (chordLen < 0.001 || Math.abs(arcAngle) < 0.001) {
			return [start, end];
		}

		// 弦的中点到圆心的距离
		const halfAngle = arcAngle / 2;
		const R = (chordLen / 2) / Math.sin(Math.abs(halfAngle));
		const d = R * Math.cos(Math.abs(halfAngle));

		// 弦的法向量（指向圆心方向）
		const nx = -dy / chordLen;
		const ny = dx / chordLen;
		const sign = arcAngle > 0 ? 1 : -1;

		const cx = mx + nx * d * sign;
		const cy = my + ny * d * sign;

		// 起点角度
		const startAngle = Math.atan2(start.y - cy, start.x - cx);

		const points: Point[] = [];
		for (let i = 0; i <= numSamples; i++) {
			const t = i / numSamples;
			const angle = startAngle + arcAngle * t;
			points.push({
				x: cx + R * Math.cos(angle),
				y: cy + R * Math.sin(angle),
			});
		}

		return points;
	}

	/**
	 * U 型大幅绕行
	 * 当所有精细绕行失败时，沿垂直方向大幅偏移生成 U 型路径
	 */
	private generateUShapeBypass(
		start: Point,
		end: Point,
		obstacles: Obstacle[],
		excludeNet?: string
	): Point[] | null {
		const dx = end.x - start.x;
		const dy = end.y - start.y;
		const len = Math.sqrt(dx * dx + dy * dy);
		if (len < 0.001) return null;

		// 法向量（垂直方向）
		const nx = -dy / len;
		const ny = dx / len;

		// 计算所有障碍物的最大半径，决定偏移量
		const blocking = this.findIntersectingObstacles(start, end, obstacles, excludeNet);
		const maxRadius = blocking.reduce((m, o) => Math.max(m, o.radius), 0);
		const baseOffset = Math.max(maxRadius * 2, len * 0.5, this.clearanceMil * 10);

		// 尝试递增偏移量，两个方向
		const offsets = [baseOffset, baseOffset * 1.5, baseOffset * 2, baseOffset * 3];

		for (const offset of offsets) {
			for (const dir of [1, -1]) {
				const mid1: Point = {
					x: start.x + dx * 0.2 + nx * dir * offset,
					y: start.y + dy * 0.2 + ny * dir * offset,
				};
				const mid2: Point = {
					x: end.x - dx * 0.2 + nx * dir * offset,
					y: end.y - dy * 0.2 + ny * dir * offset,
				};
				const path = [start, mid1, mid2, end];
				if (this.isPathSafe(path, obstacles, excludeNet)) {
					return path;
				}
			}
		}

		return null;
	}

	/**
	 * 生成多点绕行路径（更复杂的绕行）
	 */
	private generateMultiPointBypass(
		start: Point,
		end: Point,
		obstacles: Obstacle[],
		excludeNet?: string
	): Point[] | null {
		const margin = this.trackWidthMil / 2 + 5;

		// 找到所有阻挡的障碍物
		const directPath = [start, end];
		const blocking = this.findIntersectingObstacles(start, end, obstacles, excludeNet);

		if (blocking.length === 0) {
			return directPath;
		}

		// 对障碍物按距离起点的距离排序
		blocking.sort((a, b) => {
			const distA = Math.sqrt((a.x - start.x) ** 2 + (a.y - start.y) ** 2);
			const distB = Math.sqrt((b.x - start.x) ** 2 + (b.y - start.y) ** 2);
			return distA - distB;
		});

		// 尝试为每个障碍物生成绕行点
		const path: Point[] = [start];
		let current = start;

		for (let i = 0; i < blocking.length; i++) {
			const obs = blocking[i];
			const next = i === blocking.length - 1 ? end : 
				{ x: blocking[i + 1].x, y: blocking[i + 1].y };

			// 尝试左侧或右侧绕行
			const leftPoints = this.generateBypassPoints(current, next, obs, 'left');
			const rightPoints = this.generateBypassPoints(current, next, obs, 'right');

			let chosenPoints: Point[] = [];

			// 选择更短的路径
			if (leftPoints.length > 0 && rightPoints.length > 0) {
				const leftDist = this.calculatePathLength([current, ...leftPoints, next]);
				const rightDist = this.calculatePathLength([current, ...rightPoints, next]);
				chosenPoints = leftDist < rightDist ? leftPoints : rightPoints;
			} else if (leftPoints.length > 0) {
				chosenPoints = leftPoints;
			} else if (rightPoints.length > 0) {
				chosenPoints = rightPoints;
			} else {
				// 都无法绕行，尝试直接连接
				continue;
			}

			path.push(...chosenPoints);
			current = chosenPoints[chosenPoints.length - 1] || next;
		}

		path.push(end);

		// 验证最终路径
		if (this.isPathSafe(path, obstacles, excludeNet)) {
			return path;
		}

		// 如果复杂绕行也失败，返回 null
		return null;
	}

	/**
	 * 计算路径总长度
	 */
	private calculatePathLength(path: Point[]): number {
		let length = 0;
		for (let i = 1; i < path.length; i++) {
			const dx = path[i].x - path[i - 1].x;
			const dy = path[i].y - path[i - 1].y;
			length += Math.sqrt(dx * dx + dy * dy);
		}
		return length;
	}

	/**
	 * 扩展绕行路径以避开所有障碍物
	 * 找到阻挡绕行路径的障碍物，计算需要的最小偏移量
	 */
	private expandBypassToAvoidAll(
		path: Point[],
		obstacles: Obstacle[],
		excludeNet: string | undefined,
		mainObstacle: Obstacle,
		direction: 'left' | 'right'
	): Point[] | null {
		if (!(mainObstacle.startX !== undefined && mainObstacle.endX !== undefined)) return null;

		const sx = mainObstacle.startX!, sy = mainObstacle.startY!;
		const ex = mainObstacle.endX!, ey = mainObstacle.endY!;
		const segDx = ex - sx, segDy = ey - sy;
		const segLen = Math.sqrt(segDx * segDx + segDy * segDy);
		if (segLen < 0.001) return null;

		const segUx = segDx / segLen, segUy = segDy / segLen;
		const isVertical = Math.abs(segDx) < Math.abs(segDy);
		const margin = this.trackWidthMil / 2;

		const start = path[0], end = path[path.length - 1];

		let maxNeeded = 0;
		for (const obs of obstacles) {
			if (excludeNet && obs.net === excludeNet) continue;
			for (let i = 0; i < path.length - 1; i++) {
				if (!this.isSegmentSafe(path[i], path[i + 1], [obs])) {
					const obsRadius = obs.radius + margin + 5;
					if (isVertical) {
						if (direction === 'left') {
							const needed = (obs.y + obsRadius) - sy;
							if (needed > maxNeeded) maxNeeded = needed;
						} else {
							const needed = ey - (obs.y - obsRadius);
							if (needed > maxNeeded) maxNeeded = needed;
						}
					} else {
						if (direction === 'left') {
							const needed = (obs.x + obsRadius) - sx;
							if (needed > maxNeeded) maxNeeded = needed;
						} else {
							const needed = ex - (obs.x - obsRadius);
							if (needed > maxNeeded) maxNeeded = needed;
						}
					}
				}
			}
		}

		if (maxNeeded <= 0) return null;

		// 用计算出的偏移量生成新路径
		const offset = maxNeeded + margin + 10;
		let tipX: number, tipY: number, extDirX: number, extDirY: number;
		if (direction === 'left') {
			tipX = sx; tipY = sy; extDirX = -segUx; extDirY = -segUy;
		} else {
			tipX = ex; tipY = ey; extDirX = segUx; extDirY = segUy;
		}

		const beyondX = tipX + extDirX * offset;
		const beyondY = tipY + extDirY * offset;

		let bypassPoints: Point[];
		if (isVertical) {
			bypassPoints = [{ x: start.x, y: beyondY }, { x: end.x, y: beyondY }];
		} else {
			bypassPoints = [{ x: beyondX, y: start.y }, { x: beyondX, y: end.y }];
		}

		const expanded = [start, ...bypassPoints, end];

		// 检查扩展路径是否安全，如果不安全（可能碰到板框），尝试缩小偏移量找到安全间隙
		if (this.isPathSafe(expanded, obstacles, excludeNet)) {
			return expanded;
		}

		// 二分搜索安全偏移量
		let lo = mainObstacle.radius + margin + 5;
		let hi = offset;
		let bestPath: Point[] | null = null;

		for (let iter = 0; iter < 10; iter++) {
			const mid = (lo + hi) / 2;
			const bx = tipX + extDirX * mid;
			const by = tipY + extDirY * mid;
			let pts: Point[];
			if (isVertical) {
				pts = [start, { x: start.x, y: by }, { x: end.x, y: by }, end];
			} else {
				pts = [start, { x: bx, y: start.y }, { x: bx, y: end.y }, end];
			}
			if (this.isPathSafe(pts, obstacles, excludeNet)) {
				bestPath = pts;
				hi = mid;
			} else {
				lo = mid;
			}
		}

		return bestPath;
	}

	/**
	 * 优化路径（移除不必要的点）
	 */
	optimizePath(path: Point[], obstacles: Obstacle[], excludeNet?: string): Point[] {
		if (path.length <= 2) return path;

		const optimized: Point[] = [path[0]];
		let currentIndex = 0;

		while (currentIndex < path.length - 1) {
			// 尝试跳到最远的点
			let furthestIndex = currentIndex + 1;

			for (let j = currentIndex + 2; j <= path.length; j++) {
				const target = j === path.length ? path[path.length - 1] : path[j];
				
				if (this.isSegmentSafe(optimized[optimized.length - 1], target, obstacles, excludeNet)) {
					furthestIndex = j;
				} else {
					break;
				}
			}

			if (furthestIndex >= path.length) {
				optimized.push(path[path.length - 1]);
				break;
			} else {
				optimized.push(path[furthestIndex]);
				currentIndex = furthestIndex;
			}
		}

		return optimized;
	}
}
