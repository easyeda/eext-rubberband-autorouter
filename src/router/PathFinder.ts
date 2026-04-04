/**
 * 路径搜索器
 * 实现 A*算法和橡皮筋绕行逻辑
 */

import { PadData } from './PCBParser';
import { DRCRules } from './DRCManager';
import { ObstacleMap } from './ObstacleMap';

export interface PathPoint {
    x: number;
    y: number;
    layer: number;
}

/**
 * A* 算法节点
 */
interface AStarNode {
    x: number;
    y: number;
    layer: number;
    g: number; // 从起点到当前点的实际代价
    h: number; // 从当前点到终点的启发式代价
    f: number; // g + h
    parent?: AStarNode;
}

/**
 * 路径搜索器类
 */
export class PathFinder {
    private pcbData: any;
    private rules: DRCRules;
    private obstacleMap: ObstacleMap;
    
    constructor(pcbData: any, rules: DRCRules, obstacleMap: ObstacleMap) {
        this.pcbData = pcbData;
        this.rules = rules;
        this.obstacleMap = obstacleMap;
    }
    
    /**
     * 查找从起点到终点的路径
     */
    async findPath(fromPad: PadData, toPad: PadData, net: string): Promise<PathPoint[]> {
        // 1. 尝试直接连接
        const directPath = this.tryDirectPath(fromPad, toPad, net);
        if (directPath) {
            return directPath;
        }
        
        // 2. 使用 A*算法搜索路径
        const astarPath = await this.aStarSearch(fromPad, toPad, net);
        if (astarPath && astarPath.length > 0) {
            return astarPath;
        }
        
        // 3. 如果 A*失败，尝试绕行算法
        const bypassPath = this.generateBypassPath(fromPad, toPad, net);
        if (bypassPath && bypassPath.length > 0) {
            return bypassPath;
        }
        
        // 4. 最后手段：直接连接（可能产生 DRC 错误）
        return [
            { x: fromPad.x, y: fromPad.y, layer: fromPad.layer },
            { x: toPad.x, y: toPad.y, layer: toPad.layer }
        ];
    }
    
    /**
     * 尝试直接路径
     */
    private tryDirectPath(fromPad: PadData, toPad: PadData, net: string): PathPoint[] | null {
        // 检查是否在同一层
        if (fromPad.layer !== toPad.layer) {
            return null;
        }
        
        // 检查直接路径是否安全
        const isClear = this.obstacleMap.isSegmentClear(
            fromPad.x, fromPad.y,
            toPad.x, toPad.y,
            net
        );
        
        if (isClear) {
            return [
                { x: fromPad.x, y: fromPad.y, layer: fromPad.layer },
                { x: toPad.x, y: toPad.y, layer: toPad.layer }
            ];
        }
        
        return null;
    }
    
    /**
     * A* 路径搜索
     */
    private async aStarSearch(fromPad: PadData, toPad: PadData, net: string): Promise<PathPoint[] | null> {
        const openSet: AStarNode[] = [];
        const closedSet = new Set<string>();
        
        // 起始节点
        const startNode: AStarNode = {
            x: fromPad.x,
            y: fromPad.y,
            layer: fromPad.layer,
            g: 0,
            h: this.heuristic(fromPad.x, fromPad.y, toPad.x, toPad.y),
            f: 0,
            parent: undefined
        };
        startNode.f = startNode.g + startNode.h;
        
        openSet.push(startNode);
        
        const maxIterations = 1000;
        let iterations = 0;
        
        while (openSet.length > 0 && iterations < maxIterations) {
            iterations++;
            
            // 找到 f 值最小的节点
            let currentIndex = 0;
            let minF = openSet[0].f;
            
            for (let i = 1; i < openSet.length; i++) {
                if (openSet[i].f < minF) {
                    minF = openSet[i].f;
                    currentIndex = i;
                }
            }
            
            const current = openSet[currentIndex];
            
            // 检查是否到达终点
            if (this.isSamePoint(current, toPad)) {
                return this.reconstructPath(current);
            }
            
            // 移动到关闭集合
            openSet.splice(currentIndex, 1);
            closedSet.add(this.nodeKey(current));
            
            // 探索邻接节点（8 方向）
            const neighbors = this.getNeighbors(current, toPad);
            
            for (const neighbor of neighbors) {
                if (closedSet.has(this.nodeKey(neighbor))) {
                    continue;
                }
                
                // 检查是否安全
                if (!this.isNodeClear(neighbor, net)) {
                    continue;
                }
                
                // 计算代价
                const tentativeG = current.g + this.distance(current, neighbor);
                
                // 检查是否已在开放集合中
                const existingNode = openSet.find(n => this.nodeKey(n) === this.nodeKey(neighbor));
                
                if (!existingNode) {
                    // 新节点
                    neighbor.g = tentativeG;
                    neighbor.h = this.heuristic(neighbor.x, neighbor.y, toPad.x, toPad.y);
                    neighbor.f = neighbor.g + neighbor.h;
                    neighbor.parent = current;
                    openSet.push(neighbor);
                } else if (tentativeG < existingNode.g) {
                    // 找到更好的路径
                    existingNode.g = tentativeG;
                    existingNode.f = existingNode.g + existingNode.h;
                    existingNode.parent = current;
                }
            }
        }
        
        // 未找到路径
        return null;
    }
    
    /**
     * 启发式函数：曼哈顿距离
     */
    private heuristic(x1: number, y1: number, x2: number, y2: number): number {
        return Math.abs(x1 - x2) + Math.abs(y1 - y2);
    }
    
    /**
     * 获取邻接节点
     */
    private getNeighbors(node: AStarNode, target: PadData): AStarNode[] {
        const neighbors: AStarNode[] = [];
        const stepSize = this.rules.physics.traceWidth * 2; // 步长
        
        // 8 方向
        const directions = [
            { dx: stepSize, dy: 0 },
            { dx: -stepSize, dy: 0 },
            { dx: 0, dy: stepSize },
            { dx: 0, dy: -stepSize },
            { dx: stepSize, dy: stepSize },
            { dx: -stepSize, dy: stepSize },
            { dx: stepSize, dy: -stepSize },
            { dx: -stepSize, dy: -stepSize }
        ];
        
        for (const dir of directions) {
            const newX = node.x + dir.dx;
            const newY = node.y + dir.dy;
            
            // 检查是否在板框内
            if (!this.isInBoard(newX, newY)) {
                continue;
            }
            
            neighbors.push({
                x: newX,
                y: newY,
                layer: node.layer,
                g: 0,
                h: 0,
                f: 0
            });
        }
        
        return neighbors;
    }
    
    /**
     * 检查节点是否安全
     */
    private isNodeClear(node: AStarNode, net: string): boolean {
        return this.obstacleMap.isPointClear(node.x, node.y, net);
    }
    
    /**
     * 检查点是否在板框内
     */
    private isInBoard(x: number, y: number): boolean {
        const bbox = this.pcbData.boardOutline?.boundingBox;
        if (!bbox) return true; // 没有板框限制
        
        const margin = this.rules.clearance.viaToBoard;
        return x >= bbox.minX + margin && 
               x <= bbox.maxX - margin && 
               y >= bbox.minY + margin && 
               y <= bbox.maxY - margin;
    }
    
    /**
     * 计算两点间距离
     */
    private distance(a: AStarNode, b: AStarNode): number {
        const dx = a.x - b.x;
        const dy = a.y - b.y;
        return Math.sqrt(dx * dx + dy * dy);
    }
    
    /**
     * 检查是否是同一点
     */
    private isSamePoint(node: AStarNode, pad: PadData): boolean {
        const tolerance = this.rules.physics.traceWidth / 2;
        return Math.abs(node.x - pad.x) < tolerance && 
               Math.abs(node.y - pad.y) < tolerance &&
               node.layer === pad.layer;
    }
    
    /**
     * 节点键值
     */
    private nodeKey(node: AStarNode): string {
        return `${node.x.toFixed(1)},${node.y.toFixed(1)},${node.layer}`;
    }
    
    /**
     * 重建路径
     */
    private reconstructPath(node: AStarNode): PathPoint[] {
        const path: PathPoint[] = [];
        let current: AStarNode | undefined = node;
        
        while (current) {
            path.unshift({
                x: current.x,
                y: current.y,
                layer: current.layer
            });
            current = current.parent;
        }
        
        return path;
    }
    
    /**
     * 生成绕行路径
     */
    private generateBypassPath(fromPad: PadData, toPad: PadData, net: string): PathPoint[] | null {
        // 简化实现：尝试在障碍物周围生成绕行点
        const midX = (fromPad.x + toPad.x) / 2;
        const midY = (fromPad.y + toPad.y) / 2;
        
        // 尝试 4 个绕行方向
        const offsets = [
            { dx: -50, dy: -50 },
            { dx: 50, dy: -50 },
            { dx: -50, dy: 50 },
            { dx: 50, dy: 50 }
        ];
        
        for (const offset of offsets) {
            const bypassX = midX + offset.dx;
            const bypassY = midY + offset.dy;
            
            // 检查绕行点是否安全
            if (this.obstacleMap.isPointClear(bypassX, bypassY, net) &&
                this.obstacleMap.isPointClear(bypassX + offset.dx, bypassY + offset.dy, net)) {
                
                const path: PathPoint[] = [
                    { x: fromPad.x, y: fromPad.y, layer: fromPad.layer },
                    { x: bypassX, y: bypassY, layer: fromPad.layer },
                    { x: bypassX + offset.dx, y: bypassY + offset.dy, layer: fromPad.layer },
                    { x: toPad.x, y: toPad.y, layer: toPad.layer }
                ];
                
                // 验证整条路径
                let isSafe = true;
                for (let i = 0; i < path.length - 1; i++) {
                    if (!this.obstacleMap.isSegmentClear(
                        path[i].x, path[i].y,
                        path[i + 1].x, path[i + 1].y,
                        net
                    )) {
                        isSafe = false;
                        break;
                    }
                }
                
                if (isSafe) {
                    return path;
                }
            }
        }
        
        return null;
    }
}
