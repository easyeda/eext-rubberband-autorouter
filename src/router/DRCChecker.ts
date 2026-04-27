/**
 * DRC 碰撞检测模块
 * 用于检测走线与障碍物之间的间距，避免 DRC 错误
 */

interface Point {
	x: number;
	y: number;
}

interface Obstacle {
	id?: string;
	x: number;
	y: number;
	radius: number; // 排斥半径 = 物体半径 + clearance
	type: 'pad' | 'via' | 'track' | 'keepout';
	net?: string;
	layer?: number;
	startX?: number;
	startY?: number;
	endX?: number;
	endY?: number;
}

interface TrackSegment {
	start: Point;
	end: Point;
	width: number;
	net: string;
	layer: number;
}

/**
 * DRC 碰撞检测器
 */
export class DRCChecker {
	private clearanceMil: number;
	private trackWidthMil: number;

	constructor(clearanceMil: number, trackWidthMil: number) {
		this.clearanceMil = clearanceMil;
		this.trackWidthMil = trackWidthMil;
	}

	/**
	 * 检查线段是否与障碍物碰撞
	 * @param start 线段起点
	 * @param end 线段终点
	 * @param obstacles 障碍物列表
	 * @param excludeNet 排除的网络（同一网络不检查）
	 * @returns 是否安全（无碰撞）
	 */
	isSegmentClear(
		start: Point,
		end: Point,
		obstacles: Obstacle[],
		excludeNet?: string
	): boolean {
		const requiredGap = this.clearanceMil + this.trackWidthMil / 2;

		for (const obs of obstacles) {
			// 跳过同一网络的障碍物
			if (excludeNet && obs.net === excludeNet) continue;

			// 计算点到线段的最短距离
			const dist = this.pointToSegmentDistance(
				obs.x, obs.y,
				start.x, start.y,
				end.x, end.y
			);

			// 检查距离是否足够
			if (dist < obs.radius + requiredGap) {
				return false;
			}
		}

		return true;
	}

	/**
	 * 检查两条线段是否相交或间距不足
	 */
	isSegmentClearFromTrack(
		start: Point,
		end: Point,
		tracks: TrackSegment[],
		excludeNet?: string
	): boolean {
		const requiredGap = this.clearanceMil;

		for (const track of tracks) {
			// 跳过同一网络
			if (excludeNet && track.net === excludeNet) continue;

			// 检查是否在同一层
			if (track.layer !== undefined) {
				// 计算两条线段之间的最短距离
				const dist = this.segmentToSegmentDistance(
					start.x, start.y, end.x, end.y,
					track.start.x, track.start.y,
					track.end.x, track.end.y
				);

				const minRequired = (this.trackWidthMil + track.width) / 2 + requiredGap;
				if (dist < minRequired) {
					return false;
				}
			}
		}

		return true;
	}

	/**
	 * 计算点到线段的最短距离
	 */
	pointToSegmentDistance(
		px: number, py: number,
		x1: number, y1: number,
		x2: number, y2: number
	): number {
		const dx = x2 - x1;
		const dy = y2 - y1;
		const len2 = dx * dx + dy * dy;

		if (len2 === 0) {
			// 线段退化为点
			return Math.sqrt((px - x1) ** 2 + (py - y1) ** 2);
		}

		// 计算投影点
		let t = ((px - x1) * dx + (py - y1) * dy) / len2;
		t = Math.max(0, Math.min(1, t));

		const projX = x1 + t * dx;
		const projY = y1 + t * dy;

		return Math.sqrt((px - projX) ** 2 + (py - projY) ** 2);
	}

	/**
	 * 计算两条线段之间的最短距离
	 */
	segmentToSegmentDistance(
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
	segmentsIntersect(
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
	 * 添加安全余量后的检查
	 * @param margin 额外的安全余量（mils）
	 */
	isSegmentClearWithMargin(
		start: Point,
		end: Point,
		obstacles: Obstacle[],
		excludeNet?: string,
		margin: number = 5
	): boolean {
		const requiredGap = this.clearanceMil + this.trackWidthMil / 2 + margin;

		for (const obs of obstacles) {
			if (excludeNet && obs.net === excludeNet) continue;

			const dist = this.pointToSegmentDistance(
				obs.x, obs.y,
				start.x, start.y,
				end.x, end.y
			);

			if (dist < obs.radius + requiredGap) {
				return false;
			}
		}

		return true;
	}

	/**
	 * 获取更新后的 clearance 和 trackWidth
	 */
	getRules() {
		return {
			clearanceMil: this.clearanceMil,
			trackWidthMil: this.trackWidthMil
		};
	}

	/**
	 * 更新规则
	 */
	updateRules(clearanceMil: number, trackWidthMil: number) {
		this.clearanceMil = clearanceMil;
		this.trackWidthMil = trackWidthMil;
	}
}
