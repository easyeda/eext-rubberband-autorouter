/**
 * 通过 eda.pcb_Drc.check() 进行调试
 */
(async () => {
    console.log('=== DRC 调试开始 ===');
    console.log('时间:', new Date().toLocaleString());
    
    // 1. 检查 eda 对象
    if (typeof eda === 'undefined') {
        console.error('❌ eda 未定义');
        return;
    }
    console.log('✅ eda 对象可用');
    
    // 2. 检查当前文档
    try {
        const doc = eda.dmt_SelectControl.getCurrentDocumentInfo();
        console.log('当前文档:', JSON.stringify(doc));
        
        if (!doc || doc.documentType !== 3) {
            console.error('❌ 请在 PCB 文档中运行 (当前=' + (doc?.documentType || 'null') + ')');
            return;
        }
        console.log('✅ 当前是 PCB 文档');
    } catch (err) {
        console.error('获取文档信息失败:', err);
    }
    
    // 3. 获取所有走线
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
                           ', start=(' + line.startX.toFixed(1) + ',' + line.startY.toFixed(1) + ')' +
                           ', end=(' + line.endX.toFixed(1) + ',' + line.endY.toFixed(1) + ')' +
                           ', width=' + line.lineWidth);
            }
        }
    } catch (err) {
        console.error('获取走线失败:', err);
    }
    
    // 4. 运行 DRC 检查
    try {
        console.log('\n运行 DRC 检查...');
        const drcPassed = await eda.pcb_Drc.check(true, true, false);
        console.log('DRC 检查结果:', drcPassed ? '✅ 通过' : '❌ 失败');
    } catch (err) {
        console.error('DRC 检查失败:', err);
    }
    
    // 5. 获取 DRC 配置
    try {
        console.log('\n获取 DRC 配置...');
        const drcConfig = await eda.pcb_Drc.getCurrentRuleConfiguration();
        console.log('DRC 配置:', JSON.stringify(drcConfig));
        
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
    
    // 6. 检查走线交叉
    try {
        console.log('\n检查走线交叉...');
        const allLines = await eda.pcb_PrimitiveLine.getAll();
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
        
        console.log('交叉总数:', crossCount);
        if (crossDetails.length > 0) {
            console.log('交叉详情:', JSON.stringify(crossDetails));
        }
    } catch (err) {
        console.error('交叉检查失败:', err);
    }
    
    // 7. 总结
    console.log('\n=== DRC 调试总结 ===');
    console.log('请查看以上日志并发送给开发者分析');
    console.log('=== DRC 调试结束 ===');
})();
