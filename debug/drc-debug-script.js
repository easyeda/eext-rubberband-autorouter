/**
 * DRC 调试脚本 - 修复版
 * 在嘉立创 EDA Console 中执行
 */

(async () => {
    console.log('=== DRC 调试开始 ===');
    console.log('时间:', new Date().toLocaleString());
    
    // 1. 检查 eda 对象
    if (typeof eda === 'undefined') {
        console.error('❌ eda 未定义');
        console.log('提示：请在嘉立创 EDA 环境中运行此脚本');
        return;
    }
    console.log('✅ eda 对象可用');
    
    // 2. 检查当前文档 - 详细诊断
    try {
        console.log('\n检查当前文档...');
        const doc = eda.dmt_SelectControl.getCurrentDocumentInfo();
        console.log('当前文档完整对象:', doc);
        console.log('当前文档 JSON:', JSON.stringify(doc));
        console.log('当前文档所有属性:', Object.keys(doc || {}));
        
        if (!doc) {
            console.error('❌ 未找到当前文档 (返回 null/undefined)');
            console.log('提示：请先打开一个 PCB 文档');
            return;
        }
        
        // 尝试多种可能的属性名
        const docType = doc.documentType || doc.type || doc.docType || doc.document_type;
        console.log('文档类型值:', docType);
        
        if (docType === 3 || docType === '3' || docType === 'PCB') {
            console.log('✅ 当前是 PCB 文档，可以继续调试');
        } else {
            console.log('文档类型不是 PCB (3)，继续尝试获取 PCB 特定 API...');
        }
        
        // 尝试直接调用 PCB API 来验证
        try {
            console.log('\n尝试调用 PCB API 验证...');
            const testLines = await eda.pcb_PrimitiveLine.getAll();
            console.log('✅ PCB API 可用，走线数量:', testLines ? testLines.length : 0);
        } catch (err) {
            console.error('❌ PCB API 调用失败:', err.message);
            console.log('提示：可能不在 PCB 文档中');
            return;
        }
        
    } catch (err) {
        console.error('获取文档信息失败:', err);
        console.log('尝试直接执行 DRC 检查...');
    }
    
    // 3. 获取所有网络
    try {
        console.log('\n获取网络列表...');
        const allNets = await eda.pcb_Net.getAllNetsName();
        console.log('网络数量:', allNets ? allNets.length : 0);
        if (allNets && allNets.length > 0) {
            console.log('前 5 个网络:', allNets.slice(0, 5));
        }
    } catch (err) {
        console.error('获取网络失败:', err.message);
    }
    
    // 4. 获取所有走线
    try {
        console.log('\n获取走线...');
        const allLines = await eda.pcb_PrimitiveLine.getAll();
        console.log('走线数量:', allLines ? allLines.length : 0);
        
        if (allLines && allLines.length > 0) {
            console.log('前 5 条走线详情:');
            for (let i = 0; i < Math.min(5, allLines.length); i++) {
                const line = allLines[i];
                console.log('  ' + (i+1) + ': net=' + line.net + 
                           ', layer=' + line.layer + 
                           ', start=(' + line.startX + ',' + line.startY + ')' +
                           ', end=(' + line.endX + ',' + line.endY + ')');
            }
        }
    } catch (err) {
        console.error('获取走线失败:', err.message);
    }
    
    // 5. 运行 DRC 检查
    try {
        console.log('\n运行 DRC 检查 (eda.pcb_Drc.check)...');
        const drcPassed = await eda.pcb_Drc.check(true, true, false);
        console.log('DRC 检查结果:', drcPassed ? '✅ 通过' : '❌ 失败');
    } catch (err) {
        console.error('DRC 检查失败:', err.message);
    }
    
    // 6. 检查走线交叉
    try {
        console.log('\n检查走线交叉...');
        const allLines = await eda.pcb_PrimitiveLine.getAll();
        let crossCount = 0;
        
        if (allLines && allLines.length > 1) {
            for (let i = 0; i < allLines.length; i++) {
                for (let j = i + 1; j < allLines.length; j++) {
                    const l1 = allLines[i];
                    const l2 = allLines[j];
                    
                    if (l1.layer === l2.layer && l1.net !== l2.net) {
                        const x1=l1.startX, y1=l1.startY, x2=l1.endX, y2=l1.endY;
                        const x3=l2.startX, y3=l2.startY, x4=l2.endX, y4=l2.endY;
                        
                        const d = (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4);
                        if (d !== 0) {
                            const t = ((x1-x3)*(y3-y4)-(y1-y3)*(x3-x4))/d;
                            const u = -((x1-x2)*(y1-y3)-(y1-y2)*(x1-x3))/d;
                            if (t>0 && t<1 && u>0 && u<1) {
                                crossCount++;
                                console.log('  ⚠️ 交叉 ' + crossCount + ': ' + l1.net + ' 与 ' + l2.net);
                            }
                        }
                    }
                }
            }
        }
        
        console.log('交叉总数:', crossCount);
    } catch (err) {
        console.error('交叉检查失败:', err.message);
    }
    
    // 7. 总结
    console.log('\n=== DRC 调试总结 ===');
    console.log('请将以上日志发送给开发者分析');
    console.log('=== DRC 调试结束 ===');
})();
