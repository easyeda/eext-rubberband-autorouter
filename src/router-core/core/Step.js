/**
 * Step类 - 路径步骤/网络段
 */
export class Step {
  constructor(prev, next, pathId) {
    this.id = pathId; // 网络ID
    this.vertex = null; // 当前顶点
    this.prev = prev; // 前一个顶点
    this.next = next; // 下一个顶点
    this.pstep = null; // 前一个步骤
    this.nstep = null; // 下一个步骤
    this.radius = 0; // 弧半径
    this.rgt = false; // 左/右车道标志
    this.outer = false; // 是否使用外车道
    this.netDesc = null; // 网络描述
    this.xt = false; // 是否可以生成凸弧
  }

  /**
   * 获取步骤的方向向量
   */
  getDirection() {
    if (!this.next) return { dx: 0, dy: 0 };
    const dx = this.next.x - this.vertex.x;
    const dy = this.next.y - this.vertex.y;
    const len = Math.hypot(dx, dy);
    return len > 0 ? { dx: dx / len, dy: dy / len } : { dx: 0, dy: 0 };
  }
}
