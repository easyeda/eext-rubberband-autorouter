/**
 * DRC 规则管理器
 * 读取和管理 PCB 设计规则
 */

export interface DRCRules {
    clearance: ClearanceRules;
    physics: PhysicsRules;
}

export interface ClearanceRules {
    traceToTrace: number;   // 导线 - 导线间距 (mil)
    traceToPad: number;     // 导线 - 焊盘间距 (mil)
    traceToVia: number;     // 导线 - 过孔间距 (mil)
    viaToVia: number;       // 过孔 - 过孔间距 (mil)
    viaToPad: number;       // 过孔 - 焊盘间距 (mil)
    viaToBoard: number;     // 过孔 - 板框间距 (mil)
    padToPad: number;       // 焊盘 - 焊盘间距 (mil)
}

export interface PhysicsRules {
    traceWidth: number;         // 导线宽度 (mil)
    viaDiameter: number;        // 过孔直径 (mil)
    viaHoleDiameter: number;    // 过孔钻孔直径 (mil)
    minTraceLength: number;     // 最小导线长度 (mil)
}

/**
 * DRC 规则管理器类
 */
export class DRCManager {
    private readonly MM_TO_MIL = 39.3701;
    private rules: DRCRules;
    
    // 默认规则
    private readonly DEFAULT_RULES: DRCRules = {
        clearance: {
            traceToTrace: 6,
            traceToPad: 6,
            traceToVia: 6,
            viaToVia: 6,
            viaToPad: 6,
            viaToBoard: 10,
            padToPad: 6
        },
        physics: {
            traceWidth: 8,
            viaDiameter: 20,
            viaHoleDiameter: 10,
            minTraceLength: 0
        }
    };
    
    constructor(drcConfig?: any) {
        this.rules = this.DEFAULT_RULES;
        
        if (drcConfig) {
            this.parseRules(drcConfig);
        }
    }
    
    /**
     * 解析 DRC 配置
     */
    private parseRules(config: any) {
        try {
            const cfg = config.config || config;
            
            // 1. 解析间距规则
            if (cfg.Spacing && cfg.Spacing['Safe Spacing']) {
                this.parseSpacingRules(cfg.Spacing['Safe Spacing']);
            }
            
            // 2. 解析物理规则
            if (cfg.Physics) {
                this.parsePhysicsRules(cfg.Physics);
            }
            
        } catch (error) {
            console.error('[DRCManager] 解析规则失败，使用默认值:', error);
        }
    }
    
    /**
     * 解析间距规则
     */
    private parseSpacingRules(spacing: any) {
        try {
            const tableKey = Object.keys(spacing)[0];
            const tbl = spacing[tableKey];
            const content: number[][] = tbl?.tables?.['1']?.content ?? [];
            const cols: string[] = tbl?.column ?? [];
            const rows: string[] = tbl?.row ?? [];
            
            // 查找 Track 列索引
            const trackColIdx = cols.indexOf('Track');
            
            if (trackColIdx === -1) {
                console.warn('[DRCManager] 未找到 Track 列');
                return;
            }
            
            // 读取各种间距值
            const clearances: number[] = [];
            
            for (const padType of ['SMD Pad', 'TH Pad', 'Via', 'Track']) {
                const rowIdx = rows.indexOf(padType);
                if (rowIdx === -1) continue;
                
                const val = content[rowIdx]?.[trackColIdx];
                if (typeof val === 'number' && val > 0) {
                    clearances.push(val * this.MM_TO_MIL);
                }
            }
            
            if (clearances.length > 0) {
                const minClearance = Math.min(...clearances);
                this.rules.clearance.traceToTrace = minClearance;
                this.rules.clearance.traceToPad = minClearance;
                this.rules.clearance.traceToVia = minClearance;
                this.rules.clearance.viaToVia = minClearance;
                this.rules.clearance.viaToPad = minClearance;
                this.rules.clearance.padToPad = minClearance;
            }
            
        } catch (error) {
            console.warn('[DRCManager] 解析间距规则失败:', error);
        }
    }
    
    /**
     * 解析物理规则
     */
    private parsePhysicsRules(physics: any) {
        try {
            // 1. 解析走线宽度
            if (physics.Track) {
                const trackRules = physics.Track;
                const keys = Object.keys(trackRules);
                const defaultKey = keys.find(k => trackRules[k].isSetDefault) || keys[0];
                
                if (defaultKey && trackRules[defaultKey]?.form?.data?.['1']?.defaultValue) {
                    const tw = trackRules[defaultKey].form.data['1'].defaultValue;
                    this.rules.physics.traceWidth = tw * this.MM_TO_MIL;
                }
            }
            
            // 2. 解析过孔规则
            if (physics.Via) {
                const viaRules = physics.Via;
                const keys = Object.keys(viaRules);
                const defaultKey = keys.find(k => viaRules[k].isSetDefault) || keys[0];
                
                if (defaultKey && viaRules[defaultKey]) {
                    const via = viaRules[defaultKey];
                    console.log('[Router2] DRC过孔规则原始数据:', JSON.stringify(via.form?.data, null, 2));
                    if (via.form?.data?.['1']?.defaultValue) {
                        this.rules.physics.viaDiameter = via.form.data['1'].defaultValue * this.MM_TO_MIL;
                        console.log(`[Router2] 过孔外径: ${via.form.data['1'].defaultValue}mm = ${this.rules.physics.viaDiameter.toFixed(2)}mil`);
                    }
                    if (via.form?.data?.['2']?.defaultValue) {
                        this.rules.physics.viaHoleDiameter = via.form.data['2'].defaultValue * this.MM_TO_MIL;
                        console.log(`[Router2] 过孔内径: ${via.form.data['2'].defaultValue}mm = ${this.rules.physics.viaHoleDiameter.toFixed(2)}mil`);
                    }
                }
            }
            
        } catch (error) {
            console.warn('[DRCManager] 解析物理规则失败:', error);
        }
    }
    
    /**
     * 获取规则
     */
    getRules(): DRCRules {
        return { ...this.rules };
    }
    
    /**
     * 获取特定间距
     */
    getClearance(type1: string, type2: string): number {
        const key = `${type1}To${type2}` as keyof ClearanceRules;
        return this.rules.clearance[key] || this.rules.clearance.traceToTrace;
    }
    
    /**
     * 验证间距是否合规
     */
    validateClearance(distance: number, type1: string, type2: string): boolean {
        const required = this.getClearance(type1, type2);
        return distance >= required;
    }
    
    /**
     * 打印规则（用于调试）
     */
    printRules() {
        console.log('[DRCManager] 当前规则:');
        console.log('  间距规则 (mil):');
        console.log(`    导线 - 导线：${this.rules.clearance.traceToTrace}`);
        console.log(`    导线 - 焊盘：${this.rules.clearance.traceToPad}`);
        console.log(`    导线 - 过孔：${this.rules.clearance.traceToVia}`);
        console.log(`    过孔 - 过孔：${this.rules.clearance.viaToVia}`);
        console.log(`    过孔 - 焊盘：${this.rules.clearance.viaToPad}`);
        console.log(`    过孔 - 板框：${this.rules.clearance.viaToBoard}`);
        console.log('  物理规则 (mil):');
        console.log(`    导线宽度：${this.rules.physics.traceWidth}`);
        console.log(`    过孔直径：${this.rules.physics.viaDiameter}`);
        console.log(`    过孔孔径：${this.rules.physics.viaHoleDiameter}`);
    }
}
