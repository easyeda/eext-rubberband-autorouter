/**
 * 几何计算工具类
 */
export class GeometryUtils {
  /**
   * 2D叉积 - 判断从o看，a到b是左转还是右转
   * @returns {boolean} true=左转, false=右转
   */
  static crossProduct2D(ax, ay, bx, by, ox, oy) {
    const vax = ax - ox;
    const vay = ay - oy;
    const vbx = bx - ox;
    const vby = by - oy;

    const cross = vax * vby - vay * vbx;
    if (cross !== 0) return cross > 0;

    // 共线情况，确保确定性结果
    return vax !== vbx ? vax < vbx : vay < vby;
  }

  /**
   * 点到直线的距离
   */
  static distanceLinePoint(x1, y1, x2, y2, x0, y0) {
    const dx = x2 - x1;
    const dy = y2 - y1;
    return Math.abs(dx * (y1 - y0) - (x1 - x0) * dy) / Math.hypot(dx, dy);
  }

  /**
   * 点到线段的距离平方
   */
  static distanceLineSegmentPointSquared(bx, by, cx, cy, px, py) {
    const mx = cx - bx;
    const my = cy - by;
    let hx = px - bx;
    let hy = py - by;

    const t0 = (mx * hx + my * hy) / (mx * mx + my * my);

    if (t0 <= 0) {
      // 最近点是b
    } else if (t0 < 1) {
      hx -= t0 * mx;
      hy -= t0 * my;
    } else {
      hx -= mx;
      hy -= my;
    }

    return hx * hx + hy * hy;
  }

  /**
   * 两直线交点
   * @returns {Array|null} [x, y, ua, ub] 或 null（平行）
   */
  static lineLineIntersection(x1, y1, x2, y2, x3, y3, x4, y4) {
    const dx1 = x2 - x1;
    const dy1 = y2 - y1;
    const d = (y4 - y3) * dx1 - (x4 - x3) * dy1;

    if (d === 0) return null; // 平行

    const ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / d;
    const ub = (dx1 * (y1 - y3) - dy1 * (x1 - x3)) / d;

    return [x1 + ua * dx1, y1 + ua * dy1, ua, ub];
  }

  /**
   * 线段相交检测
   */
  static lineSegmentsIntersect(x1, y1, x2, y2, x3, y3, x4, y4) {
    const result = GeometryUtils.lineLineIntersection(x1, y1, x2, y2, x3, y3, x4, y4);
    if (!result) return false;

    const [, , ua, ub] = result;
    return ua >= 0 && ua <= 1 && ub >= 0 && ub <= 1;
  }
}
