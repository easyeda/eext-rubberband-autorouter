/**
 * 调试脚本 - 请在嘉立创 EDA Console 中执行
 * 
 * 使用方法：
 * 1. 按 F12 打开开发者工具
 * 2. 切换到 Console 标签
 * 3. 复制粘贴此脚本全部内容
 * 4. 按回车执行
 * 5. 截图或复制输出结果
 */

(async () => {
    console.log('=== 调试开始 ===');
    console.log('时间:', new Date().toLocaleString());
    
    // 检查 eda 对象
    if (typeof eda === 'undefined') {
        console.error('❌ eda 未定义');
        return;
    }
    console.log('✅ eda 可用');
    
    // 检查当前文档
    try {
        const doc = eda.dmt_SelectControl.getCurrentDocumentInfo();
        console.log('当前文档:', JSON.stringify(doc));
        
        if (!doc || doc.documentType !== 3) {
            console.error('❌ 请在 PCB 文档中运行 (当前=' + (doc?.documentType || 'null') + ')');
            return;
        }
        console.log('✅ 是 PCB 文档');
    } catch (err) {
        console.error('获取文档信息失败:', err);
    }
    
    // 获取网络
    try {
        const nets = await eda.pcb_Net.getAllNetsName();
        console.log('网络数量:', nets ? nets.length : 0);
    } catch (err) {
        console.error('获取网络失败:', err);
    }
    
    // 获取走线
    try {
        const lines = await eda.pcb_PrimitiveLine.getAll();
        console.log('走线数量:', lines ? lines.length : 0);
        
        if (lines && lines.length > 0) {
            console.log('前 5 条走线:');
            for (let i = 0; i < Math.min(5, lines.length); i++) {
                const l = lines[i];
                console.log('  ' + (i+1) + ': net=' + l.net + ', layer=' + l.layer + 
                           ', start=(' + l.startX.toFixed(1) + ',' + l.startY.toFixed(1) + ')' +
                           ', end=(' + l.endX.toFixed(1) + ',' + l.endY.toFixed(1) + ')');
            }
        }
    } catch (err) {
        console.error('获取走线失败:', err);
    }
    
    // 运行 DRC 检查
    try {
        console.log('运行 DRC 检查...');
        const drc = await eda.pcb_Drc.check(true, true, false);
        console.log('DRC 结果:', drc ? '✅ 通过' : '❌ 失败');
    } catch (err) {
        console.error('DRC 检查失败:', err);
    }
    
    // 检查走线交叉
    try {
        console.log('检查走线交叉...');
        const lines = await eda.pcb_PrimitiveLine.getAll();
        let crossCount = 0;
        
        if (lines && lines.length > 1) {
            for (let i = 0; i < lines.length; i++) {
                for (let j = i + 1; j < lines.length; j++) {
                    const l1 = lines[i], l2 = lines[j];
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
        
        console.log('交叉数量:', crossCount);
    } catch (err) {
        console.error('交叉检查失败:', err);
    }
    
    console.log('=== 调试结束 ===');
    console.log('请将以上日志截图或复制发送给开发者');
})();
