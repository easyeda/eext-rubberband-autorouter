/**
 * 切线计算类
 */
export class Tangents {
  /**
   * 计算两个圆之间的切线
   * @param {number} x1, y1 - 第一个圆的圆心
   * @param {number} r1 - 第一个圆的半径
   * @param {boolean} left1 - 第一个圆的左/右切线
   * @param {number} x2, y2 - 第二个圆的圆心
   * @param {number} r2 - 第二个圆的半径
   * @param {boolean} left2 - 第二个圆的左/右切线
   * @returns {Array} [x1', y1', x2', y2'] 切线端点
   */
  static getTangents(x1, y1, r1, left1, x2, y2, r2, left2) {
    const d = Math.hypot(x1 - x2, y1 - y2);
    const vx = (x2 - x1) / d;
    const vy = (y2 - y1) / d;

    // 根据左右切线调整半径
    const r2Adjusted = r2 * (left1 === left2 ? 1 : -1);
    const c = (r1 - r2Adjusted) / d;

    let h = 1 - c * c;
    if (h < 0) h = 0; // 处理数值误差

    h = Math.sqrt(h) * (left1 ? -1 : 1);

    const nx = vx * c - h * vy;
    const ny = vy * c + h * vx;

    return [
      x1 + r1 * nx,
      y1 + r1 * ny,
      x2 + r2Adjusted * nx,
      y2 + r2Adjusted * ny
    ];
  }

  /**
   * 计算两个圆的所有四条切线
   * @returns {Object} { ll, lr, rl, rr } 四条切线
   */
  static getAllTangents(x1, y1, r1, x2, y2, r2) {
    return {
      ll: Tangents.getTangents(x1, y1, r1, true, x2, y2, r2, true),
      lr: Tangents.getTangents(x1, y1, r1, true, x2, y2, r2, false),
      rl: Tangents.getTangents(x1, y1, r1, false, x2, y2, r2, true),
      rr: Tangents.getTangents(x1, y1, r1, false, x2, y2, r2, false)
    };
  }
}
