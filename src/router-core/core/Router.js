import { DelaunayTriangulation } from '../algorithms/Delaunay.js';
import { FibonacciQueue } from '../algorithms/FibonacciQueue.js';
import { Vertex } from './Vertex.js';
import { Region } from './Region.js';
import { Cut } from './Cut.js';
import { Step } from './Step.js';
import { MBD } from '../utils/Constants.js';
import { GeometryUtils } from '../geometry/GeometryUtils.js';

/**
 * 主路由器类
 */
export class Router {
  constructor(b1x, b1y, b2x, b2y) {
    Vertex.resetClass();
    this.bounds = { b1x, b1y, b2x, b2y };
    this.cdt = new DelaunayTriangulation();
    this.cdtHash = new Map();
    this.vertices = [];
    this.regions = [];
    this.netlist = [];
    this.newcuts = new Map();
    this.pathID = 0;
  }

  /**
   * 插入顶点
   */
  insertVertex(name, x, y, thickness, clearance) {
    const key = `${x},${y}`;
    if (this.cdtHash.has(key)) return;

    const v = new Vertex(x, y, thickness, clearance);
    v.name = name;
    this.cdtHash.set(key, v);
    this.cdt.insert(v);
  }

  /**
   * 完成初始化
   */
  finishInit() {
    this.cdt.triangulate();

    this.vertices = [];
    this.regions = [];

    this.cdt.each(v => {
      this.vertices.push(v);
      this.regions.push(new Region(v));
    });

    // 构建cuts
    this.vertices.forEach(v => {
      v.neighbors.forEach(n => {
        const key = this._getCutKey(v, n);
        if (!this.newcuts.has(key)) {
          this.newcuts.set(key, new Cut(v, n));
        }
      });
    });
  }

  /**
   * 路由单个网络
   */
  route(netId, maxDetourFactor = 2) {
    const netDesc = this.netlist[netId];
    const startNode = this.regions.find(r =>
      r.incident && r.vertex.name === netDesc.t1Name
    );

    if (!startNode) {
      console.error(`Start node not found: ${netDesc.t1Name}`);
      return false;
    }

    const path = this.dijkstra(startNode, netDesc.t2Name, netDesc, maxDetourFactor);

    if (!path) {
      console.error(`No path found from ${netDesc.t1Name} to ${netDesc.t2Name}`);
      return false;
    }

    // 分割区域
    this._splitRegionsAlongPath(path, netDesc);

    return true;
  }

  /**
   * 生成键的字符串表示（避免循环引用）
   */
  _keyToString(key) {
    if (Array.isArray(key)) {
      return key.map(item => {
        if (item && typeof item === 'object') {
          if (item.vertex) {
            // Region对象
            return `r${item.vertex.id}`;
          } else if (item.id !== undefined) {
            // Vertex或其他有id的对象
            return `v${item.id}`;
          }
          return String(item);
        }
        return String(item);
      }).join('|');
    }
    return String(key);
  }

  /**
   * Dijkstra最短路径算法
   */
  dijkstra(startNode, endNodeName, netDesc, maxDetourFactor = 2) {
    const q = new FibonacciQueue(-1, Infinity);
    const distances = new Map();
    const parents = new Map();

    const endVertex = this.vertices.find(v => v.name === endNodeName);
    if (!endVertex) return null;

    const maxDistance = maxDetourFactor * Math.hypot(
      startNode.vertex.x - endVertex.x,
      startNode.vertex.y - endVertex.y
    );

    // 初始化：从起始节点的所有邻接区域开始
    startNode.vertex.neighbors.forEach(neighbor => {
      const neighborRegion = this.regions.find(r => r.vertex === neighbor);
      if (neighborRegion) {
        const key = [neighborRegion, startNode, false];
        const dist = Math.hypot(
          neighbor.x - startNode.vertex.x,
          neighbor.y - startNode.vertex.y
        );
        q.inc(key, dist);
        distances.set(this._keyToString(key), dist);
      }
    });

    // 主循环
    while (q.length > 0) {
      const [min, oldDistance] = q.pop();
      if (!min) break;

      const [currentRegion, prevRegion, rgt] = min;

      // 检查是否到达目标
      if (currentRegion.vertex.name === endNodeName && currentRegion.incident) {
        // 回溯构建路径
        const path = [];
        let p = min;
        while (p) {
          path.unshift(p[0]);
          p = parents.get(this._keyToString(p));
        }
        return path;
      }

      // 检查距离限制
      if (oldDistance > maxDistance) continue;

      // 探索邻接区域
      currentRegion.vertex.neighbors.forEach(neighbor => {
        const neighborRegion = this.regions.find(r => r.vertex === neighbor);
        if (!neighborRegion) return;

        const newKey = [neighborRegion, currentRegion, false];
        const keyStr = this._keyToString(newKey);

        if (distances.has(keyStr)) return;

        // 计算新距离
        const edgeDist = Math.hypot(
          neighbor.x - currentRegion.vertex.x,
          neighbor.y - currentRegion.vertex.y
        );
        const newDistance = oldDistance + edgeDist;

        // 检查容量
        const cut = this.newcuts.get(this._getCutKey(currentRegion.vertex, neighbor));
        if (cut) {
          const squeeze = cut.squeezeStrength(netDesc.traceWidth, netDesc.traceClearance);
          if (squeeze >= MBD) return; // 阻塞
        }

        if (q.inc(newKey, newDistance)) {
          distances.set(keyStr, newDistance);
          parents.set(keyStr, min);
        }
      });
    }

    return null; // 未找到路径
  }

  /**
   * 沿路径分割区域
   */
  _splitRegionsAlongPath(path, netDesc) {
    // 创建Step对象记录路径
    let pstep = null;
    for (let i = 0; i < path.length; i++) {
      const cur = path[i];
      const prev = i > 0 ? path[i - 1].vertex : null;
      const next = i < path.length - 1 ? path[i + 1].vertex : null;

      const step = new Step(prev, next, this.pathID);
      step.vertex = cur.vertex;
      step.netDesc = netDesc;
      cur.vertex.attachedNets.push(step);

      if (pstep) {
        pstep.nstep = step;
        step.pstep = pstep;
      }

      pstep = step;
    }

    this.pathID++;
  }

  /**
   * 获取Cut的键
   */
  _getCutKey(v1, v2) {
    return v1.id < v2.id ? `${v1.id},${v2.id}` : `${v2.id},${v1.id}`;
  }

  /**
   * 生成网络列表
   */
  generateNetlist(netArray) {
    this.netlist = netArray;
  }
}
