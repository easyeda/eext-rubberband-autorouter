/**
 * 障碍物地图
 * 管理所有障碍物（焊盘、走线、过孔、板框）
 */

import { PCBData } from './PCBParser';
import { DRCRules } from './DRCManager';
import { PathPoint } from './PathFinder';

export interface Obstacle {
    type: 'pad' | 'trace' | 'via' | 'board';
    net?: string;
    layer?: number;
    x: number;
    y: number;
    radius: number; // 圆形障碍物的半径
    x1?: number;    // 线段起点 X
    y1?: number;    // 线段起点 Y
    x2?: number;    // 线段终点 X
    y2?: number;    // 线段终点 Y
}

/**
 * 障碍物地图类
 */
export class ObstacleMap {
    private obstacles: Obstacle[] = [];
    private rules: DRCRules;
    
    constructor(pcbData: PCBData, rules: DRCRules) {
        this.rules = rules;
        this.initialize(pcbData);
    }
    
    /**
     * 初始化障碍物
     */
    private initialize(pcbData: PCBData) {
        // 1. 添加焊盘障碍物
        for (const pad of pcbData.pads) {
            const radius = Math.max(pad.width, pad.height) / 2 + this.rules.clearance.padToPad;
            this.obstacles.push({
                type: 'pad',
                net: pad.net,
                layer: pad.layer,
                x: pad.x,
                y: pad.y,
                radius
            });
        }
        
        // 2. 添加板框障碍物
        for (const segment of pcbData.boardOutline.segments) {
            this.obstacles.push({
                type: 'board',
                x: (segment.startX + segment.endX) / 2,
                y: (segment.startY + segment.endY) / 2,
                radius: this.rules.clearance.viaToBoard,
                x1: segment.startX,
                y1: segment.startY,
                x2: segment.endX,
                y2: segment.endY
            });
        }
    }
    
    /**
     * 添加走线障碍物
     */
    addTrace(path: PathPoint[], net: string) {
        for (let i = 0; i < path.length - 1; i++) {
            const p1 = path[i];
            const p2 = path[i + 1];
            
            this.obstacles.push({
                type: 'trace',
                net,
                layer: p1.layer,
                x: (p1.x + p2.x) / 2,
                y: (p1.y + p2.y) / 2,
                radius: this.rules.clearance.traceToTrace,
                x1: p1.x,
                y1: p1.y,
                x2: p2.x,
                y2: p2.y
            });
        }
    }
    
    /**
     * 添加过孔障碍物
     */
    addVia(x: number, y: number, net: string) {
        this.obstacles.push({
            type: 'via',
            net,
            x,
            y,
            radius: this.rules.clearance.viaToVia
        });
    }
    
    /**
     * 检查点是否与障碍物冲突
     */
    isPointClear(x: number, y: number, excludeNet?: string): boolean {
        for (const obs of this.obstacles) {
            if (excludeNet && obs.net === excludeNet) continue;
            
            const dx = x - obs.x;
            const dy = y - obs.y;
            const dist = Math.sqrt(dx * dx + dy * dy);
            
            if (dist < obs.radius) {
                return false;
            }
        }
        return true;
    }
    
    /**
     * 检查线段是否与障碍物冲突
     */
    isSegmentClear(
        x1: number, y1: number, 
        x2: number, y2: number, 
        excludeNet?: string
    ): boolean {
        for (const obs of this.obstacles) {
            if (excludeNet && obs.net === excludeNet) continue;
            
            // 检查线段与圆形障碍物的距离
            let dist: number;
            
            if (obs.x1 !== undefined && obs.y1 !== undefined && 
                obs.x2 !== undefined && obs.y2 !== undefined) {
                // 线段障碍物：计算线段间距离
                dist = this.segmentToSegmentDistance(
                    x1, y1, x2, y2,
                    obs.x1, obs.y1, obs.x2, obs.y2
                );
            } else {
                // 圆形障碍物：计算点到线段距离
                dist = this.pointToSegmentDistance(
                    obs.x, obs.y,
                    x1, y1, x2, y2
                );
            }
            
            if (dist < obs.radius) {
                return false;
            }
        }
        return true;
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
        // 简化实现：计算端点到另一条线段的最小距离
        const d1 = this.pointToSegmentDistance(x1, y1, x3, y3, x4, y4);
        const d2 = this.pointToSegmentDistance(x2, y2, x3, y3, x4, y4);
        const d3 = this.pointToSegmentDistance(x3, y3, x1, y1, x2, y2);
        const d4 = this.pointToSegmentDistance(x4, y4, x1, y1, x2, y2);
        
        return Math.min(d1, d2, d3, d4);
    }
    
    /**
     * 获取所有障碍物（用于调试）
     */
    getAllObstacles(): Obstacle[] {
        return [...this.obstacles];
    }
}
