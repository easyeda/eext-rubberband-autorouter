/**
 * 凸包算法类
 */
export class ConvexHull {
  /**
   * Andrew单调链算法计算凸包
   * @param {Array} points - 点数组 [{x, y}, ...]
   * @returns {Array} 凸包顶点数组
   */
  static compute(points) {
    if (points.length < 3) return points;

    // 按x坐标排序，x相同则按y排序
    const sorted = [...points].sort((a, b) => {
      if (a.x !== b.x) return a.x - b.x;
      return a.y - b.y;
    });

    // 构建下凸包
    const lower = [];
    for (const p of sorted) {
      while (lower.length >= 2 &&
             ConvexHull._cross(lower[lower.length - 2], lower[lower.length - 1], p) <= 0) {
        lower.pop();
      }
      lower.push(p);
    }

    // 构建上凸包
    const upper = [];
    for (let i = sorted.length - 1; i >= 0; i--) {
      const p = sorted[i];
      while (upper.length >= 2 &&
             ConvexHull._cross(upper[upper.length - 2], upper[upper.length - 1], p) <= 0) {
        upper.pop();
      }
      upper.push(p);
    }

    // 移除重复的首尾点
    lower.pop();
    upper.pop();

    return lower.concat(upper);
  }

  /**
   * 叉积计算
   */
  static _cross(o, a, b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
  }

  /**
   * 计算圆形对象的凸包（简化版Apollonius图）
   * @param {Array} circles - 圆形数组 [{x, y, r}, ...]
   * @returns {Array} 触及凸包的圆形索引
   */
  static circlesConvexHull(circles) {
    if (circles.length < 3) return circles.map((_, i) => i);

    // 提取圆心
    const centers = circles.map(c => ({ x: c.x, y: c.y }));
    const hull = ConvexHull.compute(centers);

    // 找出在凸包上的圆形
    const hullIndices = [];
    hull.forEach(hp => {
      const index = circles.findIndex(c => c.x === hp.x && c.y === hp.y);
      if (index !== -1) hullIndices.push(index);
    });

    return hullIndices;
  }
}
