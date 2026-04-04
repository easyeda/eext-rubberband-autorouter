/**
 * 核心布线引擎
 * 实现橡皮筋拓扑路由算法
 * 
 * 算法流程：
 * 1. 构建障碍物地图
 * 2. 对每个网络执行布线：
 *    a. 计算最小生成树 (MST)
 *    b. 对 MST 的每条边执行 A* 搜索
 *    c. 遇到障碍物时生成绕行点
 *    d. 优化路径（橡皮筋算法）
 * 3. 生成过孔（如需换层）
 * 4. 返回布线结果
 */

import { PadData, NetData, PCBData } from './PCBParser';
import { DRCRules } from './DRCManager';
import { ObstacleMap } from './ObstacleMap';
import { PathFinder, PathPoint } from './PathFinder';

export interface TraceData {
    net: string;
    layer: number;
    points: { x: number; y: number }[];
    width: number;
}

export interface ViaData {
    net: string;
    x: number;
    y: number;
    fromLayer: number;
    toLayer: number;
    diameter: number;
    holeDiameter: number;
}

export interface RoutingResult {
    success: boolean;
    traces: TraceData[];
    vias: ViaData[];
    routedNets: number;
}

export interface ProgressData {
    stage: string;
    percentage: number;
    message: string;
}

export interface LogData {
    message: string;
    level: 'info' | 'warn' | 'error' | 'success';
}

/**
 * 核心布线引擎类
 */
export class RouterCore {
    private pcbData: PCBData;
    private rules: DRCRules;
    private obstacleMap: ObstacleMap;
    private pathFinder: PathFinder;
    
    private traces: TraceData[] = [];
    private vias: ViaData[] = [];
    private routedNets = 0;
    
    constructor(pcbData: PCBData, rules: DRCRules) {
        this.pcbData = pcbData;
        this.rules = rules;
        
        // 初始化障碍物地图
        this.obstacleMap = new ObstacleMap(pcbData, rules);
        
        // 初始化路径搜索器
        this.pathFinder = new PathFinder(pcbData, rules, this.obstacleMap);
    }
    
    /**
     * 执行布线
     */
    async route(
        maxPasses: number,
        onProgress: (progress: ProgressData) => void,
        onLog: (log: LogData) => void,
        shouldCancel: () => boolean
    ): Promise<RoutingResult> {
        const startTime = performance.now();
        
        try {
            onProgress({ stage: '准备', percentage: 0, message: '初始化布线引擎...' });
            onLog({ message: `开始布线，最大轮数：${maxPasses}`, level: 'info' });
            
            // 1. 按网络分组焊盘
            onProgress({ stage: '分组', percentage: 5, message: '按网络分组焊盘...' });
            const padGroups = this.groupPadsByNet();
            onLog({ message: `找到 ${padGroups.size} 个可布线网络`, level: 'info' });
            
            // 2. 对每个网络布线
            const netNames = Array.from(padGroups.keys());
            let successCount = 0;
            
            for (let i = 0; i < netNames.length; i++) {
                if (shouldCancel()) {
                    onLog({ message: '布线已取消', level: 'warn' });
                    break;
                }
                
                const netName = netNames[i];
                const pads = padGroups.get(netName)!;
                
                const pct = 10 + Math.floor((i / netNames.length) * 80);
                onProgress({ 
                    stage: '布线中', 
                    percentage: pct, 
                    message: `布线网络 ${i + 1}/${netNames.length}: ${netName}` 
                });
                
                try {
                    const success = await this.routeNet(netName, pads, onLog);
                    if (success) {
                        successCount++;
                    }
                } catch (error) {
                    onLog({ 
                        message: `网络 ${netName} 布线失败：${error instanceof Error ? error.message : String(error)}`, 
                        level: 'error' 
                    });
                }
            }
            
            this.routedNets = successCount;
            
            // 3. 完成
            const duration = performance.now() - startTime;
            onProgress({ stage: '完成', percentage: 100, message: '布线完成' });
            onLog({ 
                message: `布线完成！成功 ${successCount}/${netNames.length} 个网络，生成 ${this.traces.length} 段走线，${this.vias.length} 个过孔，耗时 ${duration.toFixed(0)}ms`, 
                level: 'success' 
            });
            
            return {
                success: successCount > 0,
                traces: this.traces,
                vias: this.vias,
                routedNets: successCount
            };
            
        } catch (error) {
            onLog({ 
                message: `布线失败：${error instanceof Error ? error.message : String(error)}`, 
                level: 'error' 
            });
            
            return {
                success: false,
                traces: [],
                vias: [],
                routedNets: 0
            };
        }
    }
    
    /**
     * 按网络分组焊盘
     */
    private groupPadsByNet(): Map<string, PadData[]> {
        const groups = new Map<string, PadData[]>();
        
        for (const pad of this.pcbData.pads) {
            if (!groups.has(pad.net)) {
                groups.set(pad.net, []);
            }
            groups.get(pad.net)!.push(pad);
        }
        
        return groups;
    }
    
    /**
     * 布线单个网络
     */
    private async routeNet(
        netName: string, 
        pads: PadData[], 
        onLog: (log: LogData) => void
    ): Promise<boolean> {
        if (pads.length < 2) {
            return false;
        }
        
        // 1. 计算最小生成树 (MST)
        const mstEdges = this.computeMST(pads);
        
        // 2. 对 MST 的每条边布线
        let success = true;
        for (const [padA, padB] of mstEdges) {
            try {
                const path = await this.pathFinder.findPath(padA, padB, netName);
                
                if (path && path.length > 0) {
                    // 创建走线
                    this.createTrace(netName, path);
                    
                    // 检查是否需要过孔
                    this.createViasIfNeeded(netName, path);
                    
                    // 添加新走线到障碍物地图
                    this.obstacleMap.addTrace(path, netName);
                } else {
                    onLog({ message: `网络 ${netName}: 无法找到路径`, level: 'warn' });
                    success = false;
                }
            } catch (error) {
                onLog({
                    message: `网络 ${netName} 布线出错：${error instanceof Error ? error.message : String(error)}`,
                    level: 'error'
                });
                success = false;
            }
        }
        
        return success;
    }
    
    /**
     * 计算最小生成树 (Prim 算法)
     */
    private computeMST(pads: PadData[]): [PadData, PadData][] {
        const n = pads.length;
        if (n < 2) return [];
        
        const inMST = new Array<boolean>(n).fill(false);
        const minDist = new Array<number>(n).fill(Infinity);
        const minFrom = new Array<number>(n).fill(-1);
        
        minDist[0] = 0;
        const edges: [PadData, PadData][] = [];
        
        for (let iter = 0; iter < n; iter++) {
            // 找到距离最小的未访问节点
            let u = -1;
            for (let i = 0; i < n; i++) {
                if (!inMST[i] && (u === -1 || minDist[i] < minDist[u])) {
                    u = i;
                }
            }
            
            inMST[u] = true;
            
            if (minFrom[u] !== -1) {
                edges.push([pads[minFrom[u]], pads[u]]);
            }
            
            // 更新邻接节点的距离
            for (let v = 0; v < n; v++) {
                if (inMST[v]) continue;
                
                const dx = pads[u].x - pads[v].x;
                const dy = pads[u].y - pads[v].y;
                const dist = Math.sqrt(dx * dx + dy * dy);
                
                if (dist < minDist[v]) {
                    minDist[v] = dist;
                    minFrom[v] = u;
                }
            }
        }
        
        return edges;
    }
    
    /**
     * 创建走线
     */
    private createTrace(netName: string, path: PathPoint[]) {
        if (path.length < 2) return;
        
        // 确定走线层（使用起点的层）
        const layer = path[0].layer;
        
        this.traces.push({
            net: netName,
            layer,
            points: path.map(p => ({ x: p.x, y: p.y })),
            width: this.rules.physics.traceWidth
        });
    }
    
    /**
     * 检查并创建过孔
     */
    private createViasIfNeeded(netName: string, path: PathPoint[]) {
        for (let i = 1; i < path.length; i++) {
            if (path[i].layer !== path[i - 1].layer) {
                // 需要过孔
                this.vias.push({
                    net: netName,
                    x: path[i].x,
                    y: path[i].y,
                    fromLayer: path[i - 1].layer,
                    toLayer: path[i].layer,
                    diameter: this.rules.physics.viaDiameter,
                    holeDiameter: this.rules.physics.viaHoleDiameter
                });
            }
        }
    }
}
