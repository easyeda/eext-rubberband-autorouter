/**
 * MCP 调试脚本 - 完整版本
 * 通过 Run API Gateway 执行
 */

(async () => {
    console.log('=== MCP 调试开始 ===');
    console.log('时间:', new Date().toLocaleString());
    
    try {
        // 1. 检查 EDA 环境
        console.log('\n【1】检查 EDA 环境...');
        if (typeof eda === 'undefined') {
            console.error('❌ eda 对象未定义');
            return;
        }
        console.log('✅ eda 对象可用');
        
        // 2. 检查当前文档
        console.log('\n【2】检查当前文档...');
        const currentDoc = eda.dmt_SelectControl.getCurrentDocumentInfo();
        console.log('当前文档:', JSON.stringify(currentDoc, null, 2));
        
        if (!currentDoc || currentDoc.documentType !== 3) {
            console.error('❌ 请在 PCB 文档中运行 (documentType=3, 当前=' + (currentDoc?.documentType || 'null') + ')');
            return;
        }
        console.log('✅ 当前是 PCB 文档');
        
        // 3. 获取所有网络
        console.log('\n【3】获取网络列表...');
        const allNets = await eda.pcb_Net.getAllNetsName();
        console.log('网络数量:', allNets ? allNets.length : 0);
        console.log('网络列表:', JSON.stringify(allNets));
        
        // 4. 获取所有走线
        console.log('\n【4】获取走线...');
        const allLines = await eda.pcb_PrimitiveLine.getAll();
        console.log('走线数量:', allLines ? allLines.length : 0);
        
        if (allLines && allLines.length > 0) {
            console.log('前 10 条走线详情:');
            for (let i = 0; i < Math.min(10, allLines.length); i++) {
                const line = allLines[i];
                console.log('  ' + (i+1) + ': net=' + line.net + ', layer=' + line.layer + 
                           ', start=(' + line.startX.toFixed(1) + ',' + line.startY.toFixed(1) + ')' +
                           ', end=(' + line.endX.toFixed(1) + ',' + line.endY.toFixed(1) + ')' +
                           ', width=' + line.lineWidth);
            }
        }
        
        // 5. 运行 DRC 检查
        console.log('\n【5】运行 DRC 检查...');
        const drcPassed = await eda.pcb_Drc.check(true, true, false);
        console.log('DRC 结果:', drcPassed ? '✅ 通过' : '❌ 失败');
        
        // 6. 检查走线交叉
        console.log('\n【6】检查走线交叉...');
        let crossCount = 0;
        const crossDetails = [];
        
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
                                const detail = '交叉 ' + crossCount + ': ' + l1.net + ' 与 ' + l2.net;
                                crossDetails.push(detail);
                                console.log('  ' + detail);
                            }
                        }
                    }
                }
            }
        }
        
        console.log('\n交叉总数:', crossCount);
        
        // 7. 获取 DRC 配置
        console.log('\n【7】获取 DRC 配置...');
        try {
            const drcConfig = await eda.pcb_Drc.getCurrentRuleConfiguration();
            const cfg = drcConfig.config;
            
            if (cfg && cfg.Spacing && cfg.Spacing['Safe Spacing']) {
                const spacing = cfg.Spacing['Safe Spacing'];
                const tableKey = Object.keys(spacing)[0];
                const tbl = spacing[tableKey];
                const content = (tbl.tables && tbl.tables['1'] && tbl.tables['1'].content) || [];
                console.log('Safe Spacing 矩阵:', JSON.stringify(content));
            }
            
            if (cfg && cfg.Physics && cfg.Physics.Track) {
                const trackRules = cfg.Physics.Track;
                const keys = Object.keys(trackRules);
                const defaultKey = keys.find(k => trackRules[k].isSetDefault) || keys[0];
                const tw = trackRules[defaultKey].form.data['1'].defaultValue;
                console.log('默认走线宽度:', tw, 'mm');
            }
        } catch (err) {
            console.error('获取 DRC 配置失败:', err);
        }
        
        // 8. 总结
        console.log('\n=== 调试总结 ===');
        console.log('走线数量:', allLines ? allLines.length : 0);
        console.log('交叉数量:', crossCount);
        console.log('DRC 检查:', drcPassed ? '✅ 通过' : '❌ 失败');
        
        if (crossCount > 0) {
            console.log('\n❌ 检测到走线交叉问题，需要修复绕行算法');
            console.log('交叉详情:', JSON.stringify(crossDetails));
        } else {
            console.log('\n✅ 未检测到交叉问题');
        }
        
    } catch (error) {
        console.error('调试出错:', error);
        console.error('错误堆栈:', error.stack);
    }
    
    console.log('\n=== 调试结束 ===');
})();
