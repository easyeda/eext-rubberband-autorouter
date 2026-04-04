/**
 * 路径优化器模块
 * 提供多种路径优化算法：
 * 1. 橡皮筋优化 - 减少不必要的拐点
 * 2. 平滑处理 - 优化走线角度
 * 3. 过孔优化 - 减少过孔数量
 */

interface Point {
	x: number;
	y: number;
	layer?: number;
}

interface OptimizationOptions {
	/** 是否启用橡皮筋优化 */
	enableRubberband: boolean;
	/** 是否启用过孔优化 */
	enableViaOptimization: boolean;
	/** 是否启用平滑处理 */
	enableSmoothing: boolean;
	/** 最小线段长度（mils） */
	minSegmentLength: number;
	/** 过孔成本权重 */
	viaCostWeight: number;
}

const DEFAULT_OPTIONS: OptimizationOptions = {
	enableRubberband: true,
	enableViaOptimization: true,
	enableSmoothing: false,
	minSegmentLength: 5,
	viaCostWeight: 1.5,
};

export class PathOptimizer {
	private options: OptimizationOptions;

	constructor(options?: Partial<OptimizationOptions>) {
		this.options = { ...DEFAULT_OPTIONS, ...options };
	}

	/**
	 * 优化路径
	 */
	optimize(path: Point[]): Point[] {
		if (path.length <= 2) return path;

		let result = [...path];

		// 1. 橡皮筋优化 - 移除不必要的拐点
		if (this.options.enableRubberband) {
			result = this.rubberbandOptimize(result);
		}

		// 2. 过孔优化 - 减少层变化
		if (this.options.enableViaOptimization) {
			result = this.viaOptimize(result);
		}

		// 3. 平滑处理 - 优化角度
		if (this.options.enableSmoothing) {
			result = this.smoothPath(result);
		}

		return result;
	}

	/**
	 * 橡皮筋优化
	 * 原理：尝试跳过中间点，直接连接更远的点，如果连线不违反 DRC
	 */
	private rubberbandOptimize(path: Point[]): Point[] {
		if (path.length <= 2) return path;

		const optimized: Point[] = [path[0]];
		let currentIndex = 0;

		while (currentIndex < path.length - 1) {
			// 寻找可以跳到的最远点
			let furthestIndex = currentIndex + 1;

			for (let j = currentIndex + 2; j < path.length; j++) {
				if (this.isLineValid(path[currentIndex], path[j], path, currentIndex, j)) {
					furthestIndex = j;
				} else {
					break;
				}
			}

			if (furthestIndex > currentIndex + 1) {
				// 可以跳过中间点
				optimized.push(path[furthestIndex]);
				currentIndex = furthestIndex;
			} else {
				// 不能跳过，添加下一个点
				optimized.push(path[currentIndex + 1]);
				currentIndex++;
			}
		}

		return optimized;
	}

	/**
	 * 过孔优化
	 * 尝试调整路径以减少层变化
	 */
	private viaOptimize(path: Point[]): Point[] {
		if (path.length <= 2) return path;

		// 找出所有过孔位置
		const viaIndices: number[] = [];
		for (let i = 1; i < path.length - 1; i++) {
			if (path[i].layer !== path[i - 1].layer || path[i].layer !== path[i + 1].layer) {
				viaIndices.push(i);
			}
		}

		if (viaIndices.length === 0) return path;

		// 尝试移除不必要的过孔
		const result = [...path];

		for (const viaIdx of viaIndices.reverse()) {
			// 检查是否可以移除这个过孔
			const prev = result[viaIdx - 1];
			const curr = result[viaIdx];
			const next = result[viaIdx + 1];

			// 如果前后点在同一层，尝试直接连接
			if (prev.layer === next.layer) {
				if (this.isLineValid(prev, next, result, viaIdx - 1, viaIdx + 1)) {
					// 移除过孔点
					result.splice(viaIdx, 1);
				}
			}
		}

		// 合并相邻的重复层切换（A→B→A 模式 → 直接 A）
		return this.mergeRedundantLayerSwitches(result);
	}

	/**
	 * 合并冗余的层切换
	 * 例如：layer1 → layer2 → layer1 在同一位置 → 直接 layer1
	 */
	private mergeRedundantLayerSwitches(path: Point[]): Point[] {
		if (path.length <= 3) return path;

		const result: Point[] = [path[0]];

		for (let i = 1; i < path.length - 1; i++) {
			const prev = result[result.length - 1];
			const curr = path[i];
			const next = path[i + 1];

			// 如果当前点是一个层切换中间点，且前后层相同，跳过
			if (prev.layer !== undefined && next.layer !== undefined &&
				prev.layer === next.layer && curr.layer !== prev.layer) {
				// 检查是否在同一位置（过孔点）
				const distToNext = Math.sqrt((curr.x - next.x) ** 2 + (curr.y - next.y) ** 2);
				if (distToNext < 1) {
					// 跳过这个冗余的层切换点
					continue;
				}
			}

			result.push(curr);
		}

		result.push(path[path.length - 1]);
		return result;
	}

	/**
	 * 平滑处理
	 * 优化走线角度，避免锐角
	 */
	private smoothPath(path: Point[]): Point[] {
		if (path.length <= 2) return path;

		const result = [...path];
		const minAngle = Math.PI / 4; // 最小角度 45 度

		for (let i = 1; i < result.length - 1; i++) {
			const prev = result[i - 1];
			const curr = result[i];
			const next = result[i + 1];

			const angle = this.calculateAngle(prev, curr, next);

			if (angle < minAngle) {
				// 角度太小，尝试调整
				const newPoint = this.findSmoothPoint(prev, curr, next, minAngle);
				if (newPoint) {
					result[i] = newPoint;
				}
			}
		}

		return result;
	}

	/**
	 * 计算两条线段之间的夹角
	 */
	private calculateAngle(p1: Point, p2: Point, p3: Point): number {
		const dx1 = p1.x - p2.x;
		const dy1 = p1.y - p2.y;
		const dx2 = p3.x - p2.x;
		const dy2 = p3.y - p2.y;

		const dot = dx1 * dx2 + dy1 * dy2;
		const len1 = Math.sqrt(dx1 * dx1 + dy1 * dy1);
		const len2 = Math.sqrt(dx2 * dx2 + dy2 * dy2);

		if (len1 === 0 || len2 === 0) return Math.PI;

		const cosAngle = dot / (len1 * len2);
		return Math.acos(Math.max(-1, Math.min(1, cosAngle)));
	}

	/**
	 * 寻找平滑点
	 */
	private findSmoothPoint(p1: Point, p2: Point, p3: Point, minAngle: number): Point | null {
		// 简单的实现：沿角平分线移动
		const dx1 = p1.x - p2.x;
		const dy1 = p1.y - p2.y;
		const dx2 = p3.x - p2.x;
		const dy2 = p3.y - p2.y;

		const len1 = Math.sqrt(dx1 * dx1 + dy1 * dy1);
		const len2 = Math.sqrt(dx2 * dx2 + dy2 * dy2);

		if (len1 === 0 || len2 === 0) return null;

		const ux1 = dx1 / len1;
		const uy1 = dy1 / len1;
		const ux2 = dx2 / len2;
		const uy2 = dy2 / len2;

		// 角平分线方向
		const bx = ux1 + ux2;
		const by = uy1 + uy2;
		const bl = Math.sqrt(bx * bx + by * by);

		if (bl < 0.001) return null;

		// 沿角平分线移动一定距离
		const moveDistance = Math.min(len1, len2) * 0.3;
		return {
			x: p2.x + (bx / bl) * moveDistance,
			y: p2.y + (by / bl) * moveDistance,
			layer: p2.layer,
		};
	}

	/**
	 * 检查线段是否有效（不违反 DRC）
	 */
	private isLineValid(
		p1: Point,
		p2: Point,
		path: Point[],
		skipStart: number,
		skipEnd: number
	): boolean {
		// 检查线段长度
		const dist = Math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2);
		if (dist < this.options.minSegmentLength) return false;

		// 检查是否与路径的其他部分相交
		for (let i = 0; i < path.length - 1; i++) {
			if (i >= skipStart && i <= skipEnd) continue;

			const q1 = path[i];
			const q2 = path[i + 1];

			if (this.segmentsIntersect(p1, p2, q1, q2)) {
				return false;
			}
		}

		return true;
	}

	/**
	 * 检查两条线段是否相交
	 */
	private segmentsIntersect(p1: Point, p2: Point, q1: Point, q2: Point): boolean {
		const ccw = (a: Point, b: Point, c: Point): number => {
			return (c.y - a.y) * (b.x - a.x) - (b.y - a.y) * (c.x - a.x);
		};

		const d1 = ccw(q1, q2, p1);
		const d2 = ccw(q1, q2, p2);
		const d3 = ccw(p1, p2, q1);
		const d4 = ccw(p1, p2, q2);

		if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
			((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) {
			return true;
		}

		// 检查共线情况
		const onSegment = (a: Point, b: Point, c: Point): boolean => {
			return Math.min(a.x, c.x) <= b.x && b.x <= Math.max(a.x, c.x) &&
				Math.min(a.y, c.y) <= b.y && b.y <= Math.max(a.y, c.y);
		};

		if (d1 === 0 && onSegment(q1, q2, p1)) return true;
		if (d2 === 0 && onSegment(q1, q2, p2)) return true;
		if (d3 === 0 && onSegment(p1, p2, q1)) return true;
		if (d4 === 0 && onSegment(p1, p2, q2)) return true;

		return false;
	}

	/**
	 * 计算路径的过孔数量
	 */
	countVias(path: Point[]): number {
		let count = 0;
		for (let i = 1; i < path.length; i++) {
			if (path[i].layer !== path[i - 1].layer) {
				count++;
			}
		}
		return count;
	}

	/**
	 * 计算路径总长度
	 */
	calculateLength(path: Point[]): number {
		let length = 0;
		for (let i = 1; i < path.length; i++) {
			const dx = path[i].x - path[i - 1].x;
			const dy = path[i].y - path[i - 1].y;
			length += Math.sqrt(dx * dx + dy * dy);
		}
		return length;
	}
}
