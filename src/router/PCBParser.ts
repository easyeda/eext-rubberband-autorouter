/**
 * PCB 数据解析器
 * 读取 PCB 文档的边框、元件、焊盘、网络等信息
 */

export interface PadData {
    uuid: string;
    padNumber: string;
    x: number;        // mil
    y: number;        // mil
    width: number;    // mil
    height: number;   // mil
    rotation: number; // 度
    net: string;
    layer: number;
    padType: 'smd' | 'th' | 'via';
    componentUuid: string;
    holeDiameter?: number; // mil (TH 焊盘)
}

export interface NetData {
    name: string;
    uuid: string;
}

export interface BoardOutline {
    segments: OutlineSegment[];
    boundingBox: {
        minX: number;
        maxX: number;
        minY: number;
        maxY: number;
    };
}

export interface OutlineSegment {
    startX: number;
    startY: number;
    endX: number;
    endY: number;
}

export interface PCBData {
    boardOutline: BoardOutline;
    pads: PadData[];
    nets: NetData[];
    layers: number;
}

/**
 * PCB 数据解析器类
 */
export class PCBParser {
    private readonly MM_TO_MIL = 39.3701;
    
    /**
     * 解析 PCB 数据
     */
    async parse(pcbData: any): Promise<PCBData> {
        try {
            // 1. 获取板框
            const boardOutline = await this.parseBoardOutline();
            
            // 2. 获取所有网络
            const nets = await this.parseNets();
            
            // 3. 获取所有焊盘
            const pads = await this.parsePads(nets);
            
            // 4. 获取层数
            const layers = await this.getLayerCount();
            
            return {
                boardOutline,
                pads,
                nets,
                layers
            };
        } catch (error) {
            console.error('[PCBParser] 解析失败:', error);
            throw new Error(`PCB 数据解析失败：${error instanceof Error ? error.message : String(error)}`);
        }
    }
    
    /**
     * 解析板框
     */
    private async parseBoardOutline(): Promise<BoardOutline> {
        try {
            // 使用 dmt_Board 获取板框信息
            const boardInfo = await eda.dmt_Board.getCurrentBoardInfo();
            
            const segments: OutlineSegment[] = [];
            const boundingBox = {
                minX: Infinity,
                maxX: -Infinity,
                minY: Infinity,
                maxY: -Infinity
            };
            
            // 如果获取到板框信息
            const boardInfoAny = boardInfo as any;
            if (boardInfoAny && boardInfoAny.boardOutline) {
                const outline = boardInfoAny.boardOutline;
                
                // 尝试从板框获取线段
                if (outline.segments) {
                    for (const segment of outline.segments) {
                        const seg: OutlineSegment = {
                            startX: (segment.startX || 0) * this.MM_TO_MIL,
                            startY: (segment.startY || 0) * this.MM_TO_MIL,
                            endX: (segment.endX || 0) * this.MM_TO_MIL,
                            endY: (segment.endY || 0) * this.MM_TO_MIL
                        };
                        segments.push(seg);
                        
                        boundingBox.minX = Math.min(boundingBox.minX, seg.startX, seg.endX);
                        boundingBox.maxX = Math.max(boundingBox.maxX, seg.startX, seg.endX);
                        boundingBox.minY = Math.min(boundingBox.minY, seg.startY, seg.endY);
                        boundingBox.maxY = Math.max(boundingBox.maxY, seg.startY, seg.endY);
                    }
                }
            }
            
            // 如果没有找到板框，使用默认值
            if (segments.length === 0) {
                console.warn('[PCBParser] 未找到板框，使用默认值');
                return {
                    segments: [],
                    boundingBox: {
                        minX: 0,
                        maxX: 100 * this.MM_TO_MIL,
                        minY: 0,
                        maxY: 100 * this.MM_TO_MIL
                    }
                };
            }
            
            return { segments, boundingBox };
            
        } catch (error) {
            console.warn('[PCBParser] 解析板框失败，使用默认值:', error);
            // 返回默认板框
            return {
                segments: [],
                boundingBox: {
                    minX: 0,
                    maxX: 100 * this.MM_TO_MIL,
                    minY: 0,
                    maxY: 100 * this.MM_TO_MIL
                }
            };
        }
    }
    
    /**
     * 解析网络列表
     */
    private async parseNets(): Promise<NetData[]> {
        try {
            const netNames = await eda.pcb_Net.getAllNetsName();
            
            if (!netNames || netNames.length === 0) {
                return [];
            }
            
            return netNames.map((name: string) => ({
                name,
                uuid: name // 简化处理，实际可能需要获取真实 UUID
            }));
            
        } catch (error) {
            console.error('[PCBParser] 解析网络失败:', error);
            throw error;
        }
    }
    
    /**
     * 解析焊盘
     */
    private async parsePads(nets: NetData[]): Promise<PadData[]> {
        const pads: PadData[] = [];
        
        try {
            // 遍历所有网络，获取每个网络的焊盘
            for (const net of nets) {
                const primitives = await eda.pcb_Net.getAllPrimitivesByNet(net.name);
                
                if (!primitives || primitives.length === 0) {
                    continue;
                }
                
                for (const prim of primitives) {
                    const pad = this.parsePadPrimitive(prim, net.name);
                    if (pad) {
                        pads.push(pad);
                    }
                }
            }
            
            return pads;
            
        } catch (error) {
            console.error('[PCBParser] 解析焊盘失败:', error);
            throw error;
        }
    }
    
    /**
     * 解析单个焊盘图元
     */
    private parsePadPrimitive(prim: any, netName: string): PadData | null {
        try {
            const anyPrim = prim as any;

            // 获取坐标 - EDA API 返回的坐标单位是 mil
            const center = anyPrim.center || { x: 0, y: 0 };
            const x = center.x || anyPrim.x || anyPrim.startX || 0;  // 不转换，直接使用
            const y = center.y || anyPrim.y || anyPrim.startY || 0;

            // 获取尺寸 - EDA API 返回的尺寸单位是 mm，需要转换为 mil
            const width = (anyPrim.topWidth || anyPrim.width || anyPrim.w || 0) * this.MM_TO_MIL;
            const height = (anyPrim.topHeight || anyPrim.height || anyPrim.h || 0) * this.MM_TO_MIL;

            // 获取层
            const layer = anyPrim.layerId ?? anyPrim.layer ?? 1;

            // 判断焊盘类型
            const holeRadius = anyPrim.holeRadius || 0;
            const padType: 'smd' | 'th' | 'via' = holeRadius > 0 ? 'th' : (anyPrim.isVia ? 'via' : 'smd');
            
            return {
                uuid: anyPrim.globalIndex || anyPrim.uuid || `${netName}_${Math.random().toString(36).slice(2)}`,
                padNumber: anyPrim.num || anyPrim.padNumber || '',
                x,
                y,
                width,
                height,
                rotation: anyPrim.rotation || 0,
                net: netName,
                layer,
                padType,
                componentUuid: anyPrim.parentId || '',
                holeDiameter: holeRadius > 0 ? holeRadius * 2 * this.MM_TO_MIL : undefined
            };
            
        } catch (error) {
            console.warn('[PCBParser] 解析单个焊盘失败:', error);
            return null;
        }
    }
    
    /**
     * 获取层数
     */
    private async getLayerCount(): Promise<number> {
        try {
            // 获取所有层
            const layers = await eda.pcb_Layer.getAllLayers();
            return layers ? layers.length : 2; // 默认双层板
        } catch (error) {
            console.warn('[PCBParser] 获取层数失败，使用默认值 2');
            return 2;
        }
    }
}
