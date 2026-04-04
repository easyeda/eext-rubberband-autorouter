/**
 * 动态障碍物地图 - bbox 版本
 * 在布线过程中动态维护，每次布线后更新障碍物
 */

// 日志前缀
const LOG_PREFIX = '[Router2]';

export interface Obstacle {
    type: 'pad' | 'trace' | 'via';
    net?: string;
    layer?: number;
    // 圆形障碍物（焊盘、过孔）
    x?: number;
    y?: number;
    radius?: number;
    // 线段障碍物（走线）
    x1?: number;
    y1?: number;
    x2?: number;
    y2?: number;
    width?: number;
    // bbox 字段（用于快速碰撞检测）
    left?: number;
    right?: number;
    top?: number;
    bottom?: number;
}

/**
 * 动态障碍物地图类
 */
export class DynamicObstacleMap {
    private obstacles: Obstacle[] = [];
    private clearanceMil: number;
    
    constructor(clearanceMil: number) {
        this.clearanceMil = clearanceMil;
    }
    
    /**
     * 初始化障碍物（焊盘）
     */
    initialize(pads: any[]) {
        this.obstacles = [];
        
        // 添加焊盘障碍物
        for (const pad of pads) {
            this.addPad(pad);
        }
    }
    
    /**
     * 添加焊盘障碍物（使用 bbox）
     */
    addPad(pad: any) {
        // 焊盘尺寸（mil）
        const width = pad.width || 0;
        const height = pad.height || 0;
        
        // 计算 bbox（焊盘中心 ± 半尺寸 + 安全间距）
        const halfWidth = width / 2;
        const halfHeight = height / 2;
        
        this.obstacles.push({
            type: 'pad',
            net: pad.net,
            layer: pad.layer,
            x: pad.x,
            y: pad.y,
            left: pad.x - halfWidth - this.clearanceMil,
            right: pad.x + halfWidth + this.clearanceMil,
            top: pad.y - halfHeight - this.clearanceMil,
            bottom: pad.y + halfHeight + this.clearanceMil,
            radius: Math.max(halfWidth, halfHeight) + this.clearanceMil  // 圆形近似
        });
    }
    
    /**
     * 添加走线障碍物（动态）
     */
    addTrace(net: string, layer: number, points: { x: number; y: number }[], width: number) {
        for (let i = 0; i < points.length - 1; i++) {
            const p1 = points[i];
            const p2 = points[i + 1];

            this.obstacles.push({
                type: 'trace',
                net,
                layer,
                x1: p1.x,
                y1: p1.y,
                x2: p2.x,
                y2: p2.y,
                width: width + this.clearanceMil  // 走线宽度 + 单侧间距
            });
        }
    }
    
    /**
     * 添加过孔障碍物（动态）
     */
    addVia(net: string, x: number, y: number, diameter: number) {
        this.obstacles.push({
            type: 'via',
            net,
            x,
            y,
            radius: diameter / 2 + this.clearanceMil
        });
    }
    
    /**
     * 检查点是否与障碍物冲突（使用 bbox）
     */
    isPointClear(x: number, y: number, excludeNet?: string): boolean {
        for (const obs of this.obstacles) {
            if (excludeNet && obs.net === excludeNet) continue;
            
            if (obs.type === 'trace') {
                // 线段障碍物
                if (obs.x1 !== undefined && obs.y1 !== undefined && 
                    obs.x2 !== undefined && obs.y2 !== undefined) {
                    const dist = this.pointToSegmentDistance(
                        x, y,
                        obs.x1, obs.y1, obs.x2, obs.y2
                    );
                    if (dist < (obs.width || 0) / 2) {
                        return false;
                    }
                }
            } else {
                // 圆形/矩形障碍物
                // 先用 bbox 快速检测
                if (obs.left !== undefined && obs.right !== undefined && 
                    obs.top !== undefined && obs.bottom !== undefined) {
                    if (x >= obs.left && x <= obs.right && y >= obs.top && y <= obs.bottom) {
                        return false;  // 点在 bbox 内
                    }
                }
                // 再用圆形精确检测
                if (obs.x !== undefined && obs.y !== undefined && obs.radius !== undefined) {
                    const dx = x - obs.x;
                    const dy = y - obs.y;
                    const dist = Math.sqrt(dx * dx + dy * dy);
                    if (dist < obs.radius) {
                        return false;
                    }
                }
            }
        }
        return true;
    }
    
    /**
     * 检查线段是否与障碍物冲突（使用 bbox）
     */
    isSegmentClear(
        x1: number, y1: number,
        x2: number, y2: number,
        excludeNet?: string
    ): boolean {
        for (const obs of this.obstacles) {
            if (excludeNet && obs.net === excludeNet) continue;
            
            // 跳过起点和终点附近的障碍物（避免焊盘自身被检测为障碍）
            const distToStart = obs.x !== undefined && obs.y !== undefined 
                ? Math.sqrt((obs.x - x1) ** 2 + (obs.y - y1) ** 2) 
                : Infinity;
            const distToEnd = obs.x !== undefined && obs.y !== undefined
                ? Math.sqrt((obs.x - x2) ** 2 + (obs.y - y2) ** 2)
                : Infinity;
            
            // 如果障碍物距离起点或终点很近（< 焊盘半径），跳过
            if (distToStart < 25 || distToEnd < 25) {
                continue;
            }
            
            let dist: number;
            
            if (obs.type === 'trace' && obs.x1 !== undefined && obs.y1 !== undefined) {
                // 线段到线段距离
                dist = this.segmentToSegmentDistance(
                    x1, y1, x2, y2,
                    obs.x1, obs.y1, obs.x2 ?? obs.x1, obs.y2 ?? obs.y1
                );
                if (dist < (obs.width || 0) / 2) {
                    console.log(`${LOG_PREFIX} [Obstacle] 线段障碍冲突：距离=${dist.toFixed(2)}, 需要>${(obs.width || 0) / 2}`);
                    return false;
                }
            } else if (obs.x !== undefined && obs.y !== undefined && obs.radius !== undefined) {
                // 先用 bbox 快速检测线段是否与 bbox 相交
                if (obs.left !== undefined && obs.right !== undefined && 
                    obs.top !== undefined && obs.bottom !== undefined) {
                    if (this.segmentIntersectsBBox(x1, y1, x2, y2, obs.left, obs.right, obs.top, obs.bottom)) {
                        // 再用圆形精确检测
                        dist = this.pointToSegmentDistance(
                            obs.x, obs.y,
                            x1, y1, x2, y2
                        );
                        if (dist < obs.radius) {
                            console.log(`${LOG_PREFIX} [Obstacle] 圆形障碍冲突：障碍 (${obs.x.toFixed(1)},${obs.y.toFixed(1)}), r=${obs.radius.toFixed(1)}, 距离=${dist.toFixed(2)}`);
                            return false;
                        }
                    }
                }
            }
        }
        return true;
    }
    
    /**
     * 检查线段是否与 bbox 相交
     */
    private segmentIntersectsBBox(
        x1: number, y1: number, x2: number, y2: number,
        left: number, right: number, top: number, bottom: number
    ): boolean {
        // 检查线段是否与 bbox 的四条边相交
        return this.segmentsIntersect(x1, y1, x2, y2, left, top, right, top) ||  // 上边
               this.segmentsIntersect(x1, y1, x2, y2, right, top, right, bottom) ||  // 右边
               this.segmentsIntersect(x1, y1, x2, y2, right, bottom, left, bottom) ||  // 下边
               this.segmentsIntersect(x1, y1, x2, y2, left, bottom, left, top);  // 左边
    }
    
    /**
     * 计算点到线段的距离
     */
    private pointToSegmentDistance(
        px: number, py: number,
        x1: number, y1: number, x2: number, y2: number
    ): number {
        const dx = x2 - x1;
        const dy = y2 - y1;
        const len2 = dx * dx + dy * dy;
        
        if (len2 === 0) {
            return Math.sqrt((px - x1) ** 2 + (py - y1) ** 2);
        }
        
        let t = ((px - x1) * dx + (py - y1) * dy) / len2;
        t = Math.max(0, Math.min(1, t));
        
        const projX = x1 + t * dx;
        const projY = y1 + t * dy;
        
        return Math.sqrt((px - projX) ** 2 + (py - projY) ** 2);
    }
    
    /**
     * 计算两条线段之间的距离
     */
    private segmentToSegmentDistance(
        x1: number, y1: number, x2: number, y2: number,
        x3: number, y3: number, x4: number, y4: number
    ): number {
        // 检查是否相交
        if (this.segmentsIntersect(x1, y1, x2, y2, x3, y3, x4, y4)) {
            return 0;
        }
        
        // 计算端点到另一条线段的最小距离
        const d1 = this.pointToSegmentDistance(x1, y1, x3, y3, x4, y4);
        const d2 = this.pointToSegmentDistance(x2, y2, x3, y3, x4, y4);
        const d3 = this.pointToSegmentDistance(x3, y3, x1, y1, x2, y2);
        const d4 = this.pointToSegmentDistance(x4, y4, x1, y1, x2, y2);
        
        return Math.min(d1, d2, d3, d4);
    }
    
    /**
     * 检查两条线段是否相交
     */
    private segmentsIntersect(
        x1: number, y1: number, x2: number, y2: number,
        x3: number, y3: number, x4: number, y4: number
    ): boolean {
        const ccw = (ax: number, ay: number, bx: number, by: number, cx: number, cy: number): number => {
            return (cy - ay) * (bx - ax) - (by - ay) * (cx - ax);
        };
        
        const d1 = ccw(x3, y3, x4, y4, x1, y1);
        const d2 = ccw(x3, y3, x4, y4, x2, y2);
        const d3 = ccw(x1, y1, x2, y2, x3, y3);
        const d4 = ccw(x1, y1, x2, y2, x4, y4);
        
        if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
            ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) {
            return true;
        }
        
        return false;
    }
    
    /**
     * 获取所有障碍物
     */
    getAllObstacles(): Obstacle[] {
        return [...this.obstacles];
    }
    
    /**
     * 清除动态障碍物（保留焊盘）
     */
    clearDynamic() {
        this.obstacles = this.obstacles.filter(obs => obs.type === 'pad');
    }
}
