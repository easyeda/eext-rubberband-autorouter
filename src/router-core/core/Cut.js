import { MBD } from '../utils/Constants.js';

/**
 * Cut类 - 边容量/切割
 */
export class Cut {
  constructor(v1, v2) {
    this.v1 = v1;
    this.v2 = v2;
    this.cap = Math.hypot(v1.x - v2.x, v1.y - v2.y); // 顶点间距离
    this.freeCap = this.cap; // 可用容量
    this.cl = []; // 通过的走线间隙数组
  }

  /**
   * 计算通过此切割的"挤压强度"
   * @param {number} traceWidth - 走线宽度
   * @param {number} traceClearance - 走线间隙
   * @returns {number} 挤压强度，MBD表示无可用空间
   */
  squeezeStrength(traceWidth, traceClearance) {
    // 计算所需的总宽度
    const requiredWidth = traceWidth + 2 * traceClearance;

    // 计算已使用的容量
    let usedCap = 0;
    for (const clearance of this.cl) {
      usedCap += clearance;
    }

    // 计算剩余容量
    const remainingCap = this.cap - usedCap;

    // 如果剩余容量不足，返回MBD（阻塞）
    if (remainingCap < requiredWidth) {
      return MBD;
    }

    // 返回挤压强度（值越小越好）
    return requiredWidth / remainingCap;
  }

  /**
   * 使用此切割（添加走线）
   * @param {number} traceClearance - 走线间隙
   */
  use(traceClearance) {
    this.cl.push(traceClearance);
    this.freeCap -= traceClearance;
  }

  /**
   * 释放此切割（移除走线）
   * @param {number} traceClearance - 走线间隙
   */
  release(traceClearance) {
    const index = this.cl.indexOf(traceClearance);
    if (index !== -1) {
      this.cl.splice(index, 1);
      this.freeCap += traceClearance;
    }
  }
}
