/**
 * 障碍物绕行算法
 * 生成绕过障碍物的路径点
 */

interface Point {
	x: number;
	y: number;
}

interface Obstacle {
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
		const margin = this.clearanceMil + this.trackWidthMil / 2;

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
	 * 在障碍物两侧生成切线路径点
	 */
	private generateBypassPoints(
		start: Point,
		end: Point,
		obstacle: Obstacle,
		direction: 'left' | 'right'
	): Point[] {
		const margin = this.clearanceMil + this.trackWidthMil / 2 + 3; // 额外 3mil 余量
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

		// 尝试左侧和右侧绕行
		const mainObstacle = blocking[0]; // 使用第一个阻挡的障碍物

		// 尝试左侧绕行
		console.log('[Bypass] 尝试左侧绕行...');
		const leftPoints = this.generateBypassPoints(start, end, mainObstacle, 'left');
		if (leftPoints.length > 0) {
			const leftPath = [start, ...leftPoints, end];
			if (this.isPathSafe(leftPath, obstacles, excludeNet)) {
				console.log('[Bypass] 左侧绕行成功');
				return leftPath;
			} else {
				console.log('[Bypass] 左侧绕行失败：路径不安全');
			}
		}

		// 尝试右侧绕行
		console.log('[Bypass] 尝试右侧绕行...');
		const rightPoints = this.generateBypassPoints(start, end, mainObstacle, 'right');
		if (rightPoints.length > 0) {
			const rightPath = [start, ...rightPoints, end];
			if (this.isPathSafe(rightPath, obstacles, excludeNet)) {
				console.log('[Bypass] 右侧绕行成功');
				return rightPath;
			} else {
				console.log('[Bypass] 右侧绕行失败：路径不安全');
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
	 */
	private isPathSafe(
		path: Point[],
		obstacles: Obstacle[],
		excludeNet?: string
	): boolean {
		for (let i = 0; i < path.length - 1; i++) {
			if (!this.isSegmentSafe(path[i], path[i + 1], obstacles, excludeNet)) {
				return false;
			}
		}
		return true;
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
		const margin = this.clearanceMil + this.trackWidthMil / 2 + 5;

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
