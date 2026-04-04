/**
 * NetDesc类 - 网络描述
 */
export class NetDesc {
  constructor(t1Name, t2Name, traceWidth, traceClearance, style = 'default') {
    this.t1Name = t1Name; // 起始终端名称
    this.t2Name = t2Name; // 结束终端名称
    this.traceWidth = traceWidth; // 走线宽度
    this.traceClearance = traceClearance; // 走线间隙
    this.style = style; // 路由样式名称
    this.pri = 0; // 优先级（用于排序）
  }

  /**
   * 获取走线的总宽度（宽度 + 间隙）
   */
  get totalWidth() {
    return this.traceWidth + this.traceClearance;
  }
}
