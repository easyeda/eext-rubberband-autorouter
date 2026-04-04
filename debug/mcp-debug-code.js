/**
 * MCP 调试脚本 - 通过 Run API Gateway 执行
 * 
 * 使用方法：
 * 1. 确保 Run API Gateway 已连接（端口 49620）
 * 2. 通过 MCP 工具执行此代码
 * 3. 获取日志和结果
 */

// 调试代码 - 通过 MCP 执行
(async () => {
    console.log('=== MCP 调试开始 ===');
    console.log('时间:', new Date().toLocaleString());
    
    try {
        // 1. 检查 eda 对象
        console.log('\n【1】检查 EDA 环境...');
        if (typeof eda === 'undefined') {
            console.error('❌ eda 对象未定义');
            return;
        }
        console.log('✅ eda 对象可用');
        
        // 2. 检查当前文档
        console.log('\n【2】检查当前文档...');
        const currentDoc = eda.dmt_SelectControl.getCurrentDocumentInfo();
        console.log('当前文档:', currentDoc);
        
        if (!currentDoc || currentDoc.documentType !== 3) {
            console.error('❌ 请在 PCB 文档中运行 (documentType=3, 当前=' + (currentDoc?.documentType || 'null') + ')');
            return;
        }
        console.log('✅ 当前是 PCB 文档');
        
        // 3. 获取所有网络
        console.log('\n【3】获取网络列表...');
        const allNets = await eda.pcb_Net.getAllNetsName();
        console.log('网络数量:', allNets ? allNets.length : 0);
        console.log('网络列表:', allNets);
        
        // 4. 获取所有走线
        console.log('\n【4】获取走线...');
        const allLines = await eda.pcb_PrimitiveLine.getAll();
        console.log('走线数量:', allLines ? allLines.length : 0);
        
        if (allLines && allLines.length > 0) {
            console.log('前 5 条走线:');
            for (let i = 0; i < Math.min(5, allLines.length); i++) {
                const line = allLines[i];
                console.log('  ' + (i+1) + ': net=' + line.net + ', layer=' + line.layer + 
                           ', start=(' + line.startX + ',' + line.startY + ')' +
                           ', end=(' + line.endX + ',' + line.endY + ')');
            }
        }
        
        // 5. 运行 DRC 检查
        console.log('\n【5】运行 DRC 检查...');
        const drcPassed = await eda.pcb_Drc.check(true, true, false);
        console.log('DRC 结果:', drcPassed ? '✅ 通过' : '❌ 失败');
        
        // 6. 检查走线交叉
        console.log('\n【6】检查走线交叉...');
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
                                console.log('  交叉 ' + crossCount + ': ' + l1.net + ' 与 ' + l2.net);
                            }
                        }
                    }
                }
            }
        }
        
        console.log('\n交叉总数:', crossCount);
        
        // 7. 总结
        console.log('\n=== 调试总结 ===');
        console.log('走线数量:', allLines ? allLines.length : 0);
        console.log('交叉数量:', crossCount);
        console.log('DRC 检查:', drcPassed ? '✅ 通过' : '❌ 失败');
        
        if (crossCount > 0) {
            console.log('\n❌ 检测到走线交叉问题');
        } else {
            console.log('\n✅ 未检测到交叉问题');
        }
        
    } catch (error) {
        console.error('调试出错:', error);
    }
    
    console.log('\n=== 调试结束 ===');
})();
