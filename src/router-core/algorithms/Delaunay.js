import Delaunator from 'delaunator';

/**
 * Delaunay三角剖分封装类
 */
export class DelaunayTriangulation {
  constructor() {
    this.vertices = [];
    this.delaunay = null;
  }

  /**
   * 插入顶点
   */
  insert(vertex) {
    this.vertices.push(vertex);
  }

  /**
   * 执行三角剖分
   */
  triangulate() {
    // delaunator需要点数组格式 [[x1, y1], [x2, y2], ...]
    const points = this.vertices.map(v => [v.x, v.y]);

    this.delaunay = Delaunator.from(points);
    this._buildNeighborGraph();
  }

  /**
   * 构建邻接图
   */
  _buildNeighborGraph() {
    const { triangles, halfedges } = this.delaunay;
    const neighbors = new Map();

    for (let i = 0; i < this.vertices.length; i++) {
      neighbors.set(i, new Set());
    }

    // 遍历所有三角形的边
    for (let e = 0; e < triangles.length; e++) {
      const p1 = triangles[e];
      const p2 = triangles[this._nextHalfedge(e)];

      neighbors.get(p1).add(p2);
      neighbors.get(p2).add(p1);
    }

    // 将邻居信息附加到顶点
    this.vertices.forEach((v, i) => {
      const neighborSet = neighbors.get(i);
      if (neighborSet) {
        v.neighbors = Array.from(neighborSet).map(idx => this.vertices[idx]);
      } else {
        v.neighbors = [];
      }
    });
  }

  _nextHalfedge(e) {
    return (e % 3 === 2) ? e - 2 : e + 1;
  }

  /**
   * 遍历所有顶点
   */
  each(callback) {
    this.vertices.forEach(callback);
  }
}
