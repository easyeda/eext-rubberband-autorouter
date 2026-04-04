/**
 * Vertex类 - 表示PCB上的顶点/终端
 */
export class Vertex {
  static idCounter = 0;
  static cidCounter = 0;

  constructor(x = 0, y = 0, radius = 0, clearance = 0) {
    this.id = Vertex.idCounter++;
    this.cid = -1; // cluster id
    this.x = x;
    this.y = y;
    this.core = radius;
    this.radius = radius;
    this.separation = clearance;
    this.clearance = clearance;
    this.name = '';
    this.neighbors = []; // Delaunay三角剖分中的邻接顶点
    this.incidentNets = []; // 关联到此终端的网络
    this.attachedNets = []; // 附加到此终端的网络
    this.via = false;
    this.numInets = 0;
  }

  /**
   * 重置类级别的计数器
   */
  static resetClass() {
    Vertex.idCounter = 0;
    Vertex.cidCounter = 0;
  }

  /**
   * 开始新的集群
   */
  static beginNewCluster() {
    Vertex.cidCounter++;
  }

  /**
   * 将此顶点添加到当前集群
   */
  addToCurrentCluster() {
    this.cid = Vertex.cidCounter;
  }

  /**
   * 重置为初始大小
   */
  resetInitialSize() {
    this.radius = this.core;
    this.separation = this.clearance;
  }

  /**
   * 计算到另一个顶点的距离
   */
  distanceTo(other) {
    return Math.hypot(this.x - other.x, this.y - other.y);
  }

  /**
   * 获取走线半径（radius + separation）
   */
  get tradius() {
    return this.radius + this.separation;
  }
}
