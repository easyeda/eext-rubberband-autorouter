/**
 * 调试脚本 - 用于诊断绕行算法问题
 * 
 * 使用方法：
 * 1. 在嘉立创 EDA 中打开 PCB 文档
 * 2. 运行此调试脚本
 * 3. 查看控制台输出
 */

async function debugAutorouter() {
    console.log('=== 自动布线调试开始 ===');
    
    try {
        // 1. 检查当前文档类型
        const currentDoc = eda.dmt_SelectControl.getCurrentDocumentInfo();
        console.log('当前文档:', currentDoc);
        
        if (!currentDoc || currentDoc.documentType !== 3) {
            console.error('错误：请在 PCB 文档中运行此脚本');
            return;
        }
        
        // 2. 获取所有网络
        const allNets = await eda.pcb_Net.getAllNetsName();
        console.log('网络列表:', allNets);
        
        // 3. 获取第一个网络的基元
        if (allNets && allNets.length > 0) {
            const netName = allNets[0];
            console.log('\n分析网络:', netName);
            
            const primitives = await eda.pcb_Net.getAllPrimitivesByNet(netName);
            console.log('网络基元数量:', primitives?.length || 0);
            
            if (primitives && primitives.length > 0) {
                console.log('第一个基元:', JSON.stringify(primitives[0], null, 2));
                
                // 获取焊盘信息
                for (const prim of primitives) {
                    const p = prim as any;
                    console.log('焊盘位置:', {
                        x: p.center?.x,
                        y: p.center?.y,
                        layer: p.layerId,
                        net: p.net
                    });
                }
            }
        }
        
        // 4. 获取所有走线
        const allLines = await eda.pcb_PrimitiveLine.getAll();
        console.log('\n当前走线数量:', allLines?.length || 0);
        
        if (allLines && allLines.length > 0) {
            console.log('前 3 条走线信息:');
            for (let i = 0; i < Math.min(3, allLines.length); i++) {
                const line = allLines[i];
                console.log(`走线 ${i + 1}:`, {
                    net: (line as any).net,
                    startX: (line as any).startX,
                    startY: (line as any).startY,
                    endX: (line as any).endX,
                    endY: (line as any).endY,
                    layer: (line as any).layer,
                    lineWidth: (line as any).lineWidth
                });
            }
        }
        
        // 5. 运行 DRC 检查
        console.log('\n运行 DRC 检查...');
        const drcPassed = await eda.pcb_Drc.check(true, true, false);
        console.log('DRC 检查结果:', drcPassed ? '通过' : '失败');
        
        // 6. 获取 DRC 规则
        const drcConfig = await eda.pcb_Drc.getCurrentRuleConfiguration();
        console.log('DRC 配置:', JSON.stringify(drcConfig, null, 2));
        
        console.log('\n=== 调试完成 ===');
        
    } catch (error) {
        console.error('调试过程出错:', error);
    }
}

// 运行调试
debugAutorouter();
