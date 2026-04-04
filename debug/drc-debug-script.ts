/**
 * DRC 调试脚本 - 通过 API Gateway 获取实际 DRC 检查结果
 * 
 * 使用方法：
 * 1. 在嘉立创 EDA 中打开 PCB 文档
 * 2. 运行自动布线
 * 3. 运行此调试脚本
 * 4. 查看控制台输出
 */

async function debugDRC() {
    console.log('=== DRC 调试开始 ===');
    console.log('时间:', new Date().toLocaleString());
    
    try {
        // 1. 检查当前文档
        const currentDoc = eda.dmt_SelectControl.getCurrentDocumentInfo();
        console.log('\n【1】当前文档:', currentDoc);
        
        if (!currentDoc || currentDoc.documentType !== 3) {
            console.error('❌ 错误：请在 PCB 文档中运行此脚本');
            return;
        }
        
        // 2. 获取所有网络
        const allNets = await eda.pcb_Net.getAllNetsName();
        console.log('\n【2】网络列表:', allNets);
        console.log('网络数量:', allNets?.length || 0);
        
        // 3. 获取所有走线
        const allLines = await eda.pcb_PrimitiveLine.getAll();
        console.log('\n【3】当前走线数量:', allLines?.length || 0);
        
        if (allLines && allLines.length > 0) {
            console.log('\n走线详情:');
            for (let i = 0; i < allLines.length; i++) {
                const line = allLines[i] as any;
                console.log(`  走线 ${i + 1}:`, {
                    net: line.net,
                    layer: line.layer,
                    startX: line.startX,
                    startY: line.startY,
                    endX: line.endX,
                    endY: line.endY,
                    lineWidth: line.lineWidth
                });
            }
        }
        
        // 4. 获取所有过孔
        const allVias = await eda.pcb_PrimitiveVia.getAll();
        console.log('\n【4】过孔数量:', allVias?.length || 0);
        
        // 5. 运行 DRC 检查
        console.log('\n【5】运行 DRC 检查...');
        const drcPassed = await eda.pcb_Drc.check(true, true, false);
        console.log('DRC 检查结果:', drcPassed ? '✅ 通过' : '❌ 失败');
        
        // 6. 获取 DRC 配置
        try {
            const drcConfig = await eda.pcb_Drc.getCurrentRuleConfiguration();
            console.log('\n【6】DRC 配置:');
            const cfg = (drcConfig as any)?.config;
            
            if (cfg?.Spacing?.['Safe Spacing']) {
                const spacing = cfg.Spacing['Safe Spacing'];
                const tableKey = Object.keys(spacing)[0];
                const tbl = spacing[tableKey];
                const content = tbl?.tables?.['1']?.content ?? [];
                console.log('  Safe Spacing 矩阵:', content);
            }
            
            if (cfg?.Physics?.Track) {
                const trackRules = cfg.Physics.Track;
                const defaultKey = Object.keys(trackRules).find(k => trackRules[k].isSetDefault) ?? Object.keys(trackRules)[0];
                const tw = trackRules[defaultKey]?.form?.data?.['1']?.defaultValue;
                console.log('  默认走线宽度:', tw, 'mm');
            }
        } catch (err) {
            console.error('获取 DRC 配置失败:', err);
        }
        
        // 7. 检查走线交叉
        console.log('\n【7】检查走线交叉...');
        let crossCount = 0;
        
        if (allLines && allLines.length > 1) {
            for (let i = 0; i < allLines.length; i++) {
                for (let j = i + 1; j < allLines.length; j++) {
                    const line1 = allLines[i] as any;
                    const line2 = allLines[j] as any;
                    
                    // 检查是否在同一层
                    if (line1.layer === line2.layer) {
                        // 检查是否相交
                        if (segmentsIntersect(
                            line1.startX, line1.startY, line1.endX, line1.endY,
                            line2.startX, line2.startY, line2.endX, line2.endY
                        )) {
                            crossCount++;
                            console.log(`  ⚠️ 交叉 ${crossCount}: 走线 ${i + 1} (${line1.net}) 与 走线 ${j + 1} (${line2.net}) 交叉`);
                        }
                    }
                }
            }
        }
        
        console.log('\n交叉总数:', crossCount);
        
        // 8. 检查间距违规
        console.log('\n【8】检查间距违规...');
        let violationCount = 0;
        const minClearanceMil = 8; // 默认最小间距
        const minClearanceMm = minClearanceMil / 39.3701;
        
        if (allLines && allLines.length > 1) {
            for (let i = 0; i < allLines.length; i++) {
                for (let j = i + 1; j < allLines.length; j++) {
                    const line1 = allLines[i] as any;
                    const line2 = allLines[j] as any;
                    
                    // 检查是否在同一层且不是同一网络
                    if (line1.layer === line2.layer && line1.net !== line2.net) {
                        const dist = segmentToSegmentDistance(
                            line1.startX, line1.startY, line1.endX, line1.endY,
                            line2.startX, line2.startY, line2.endX, line2.endY
                        );
                        
                        // 转换为 mm
                        const distMm = dist / 39.3701;
                        
                        if (distMm < minClearanceMm) {
                            violationCount++;
                            console.log(`  ⚠️ 违规 ${violationCount}: 走线 ${i + 1} (${line1.net}) 与 走线 ${j + 1} (${line2.net}) 间距 ${distMm.toFixed(3)}mm < ${minClearanceMm.toFixed(3)}mm`);
                        }
                    }
                }
            }
        }
        
        console.log('\n间距违规总数:', violationCount);
        
        // 9. 总结
        console.log('\n=== 调试总结 ===');
        console.log('走线数量:', allLines?.length || 0);
        console.log('交叉数量:', crossCount);
        console.log('间距违规数量:', violationCount);
        console.log('DRC 检查:', drcPassed ? '✅ 通过' : '❌ 失败');
        
        if (crossCount > 0 || violationCount > 0) {
            console.log('\n❌ 检测到布线问题，需要修复绕行算法');
        } else {
            console.log('\n✅ 未检测到明显问题');
        }
        
    } catch (error) {
        console.error('调试过程出错:', error);
    }
    
    console.log('\n=== 调试结束 ===');
}

// 辅助函数：检查两条线段是否相交
function segmentsIntersect(
    x1: number, y1: number, x2: number, y2: number,
    x3: number, y3: number, x4: number, y4: number
): boolean {
    const ccw = (ax: number, ay: number, bx: number, by: number, cx: number, cy: number): number => {
        return (cy - ay) * (bx - ax) - (by - ay) * (cx - ax);
    };

    const d1 = ccw(x3, y3, x4, x4, x1, y1);
    const d2 = ccw(x3, y3, x4, y4, x2, y2);
    const d3 = ccw(x1, y1, x2, y2, x3, y3);
    const d4 = ccw(x1, y1, x2, y2, x4, y4);

    if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
        ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) {
        return true;
    }

    return false;
}

// 辅助函数：计算两条线段之间的最短距离
function segmentToSegmentDistance(
    x1: number, y1: number, x2: number, y2: number,
    x3: number, y3: number, x4: number, y4: number
): number {
    const pointToSegmentDistance = (
        px: number, py: number,
        x1: number, y1: number, x2: number, y2: number
    ): number => {
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
    };

    // 检查是否相交
    if (segmentsIntersect(x1, y1, x2, y2, x3, y3, x4, y4)) {
        return 0;
    }

    // 计算各端点到另一条线段的距离
    const d1 = pointToSegmentDistance(x1, y1, x3, y3, x4, y4);
    const d2 = pointToSegmentDistance(x2, y2, x3, y3, x4, y4);
    const d3 = pointToSegmentDistance(x3, y3, x1, y1, x2, y2);
    const d4 = pointToSegmentDistance(x4, y4, x1, y1, x2, y2);

    return Math.min(d1, d2, d3, d4);
}

// 运行调试
debugDRC();
