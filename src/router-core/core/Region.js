/**
 * Region类 - 表示路由区域
 */
export class Region {
  constructor(vertex) {
    this.vertex = vertex;
    this.neighbors = []; // 邻接区域
    this.incident = true; // 是否为关联区域
    this.outer = false; // 是否使用外车道
    this.g = 1; // 权重
    this.ox = 0; // 偏移向量x
    this.oy = 0; // 偏移向量y
    this.rx = vertex.x; // 区域x坐标
    this.ry = vertex.y; // 区域y坐标
    this.idirs = []; // 内方向约束
    this.odirs = []; // 外方向约束
  }

  /**
   * 计算到另一个区域的距离
   */
  distanceTo(other) {
    return Math.hypot(
      this.vertex.x - other.vertex.x,
      this.vertex.y - other.vertex.y
    );
  }

  /**
   * 获取区域的实际坐标（包含偏移）
   */
  get actualX() {
    return this.rx + this.ox;
  }

  get actualY() {
    return this.ry + this.oy;
  }
}
