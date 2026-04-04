/**
 * 通过 MCP 执行调试代码
 */
(async () => {
    console.log('=== MCP 调试开始 ===');
    
    // 检查 eda 对象
    if (typeof eda === 'undefined') {
        console.error('❌ eda 未定义');
        return;
    }
    console.log('✅ eda 可用');
    
    // 检查当前文档
    const doc = eda.dmt_SelectControl.getCurrentDocumentInfo();
    console.log('当前文档:', JSON.stringify(doc));
    
    if (!doc || doc.documentType !== 3) {
        console.error('❌ 请在 PCB 文档中运行 (当前=' + (doc?.documentType || 'null') + ')');
        return;
    }
    console.log('✅ 是 PCB 文档');
    
    // 获取网络
    const nets = await eda.pcb_Net.getAllNetsName();
    console.log('网络数量:', nets ? nets.length : 0);
    
    // 获取走线
    const lines = await eda.pcb_PrimitiveLine.getAll();
    console.log('走线数量:', lines ? lines.length : 0);
    
    // 运行 DRC
    const drc = await eda.pcb_Drc.check(true, true, false);
    console.log('DRC:', drc ? '✅ 通过' : '❌ 失败');
    
    // 检查交叉
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
                        if (t>0 && t<1 && u>0 && u<1) crossCount++;
                    }
                }
            }
        }
    }
    console.log('交叉数量:', crossCount);
    console.log('=== 调试结束 ===');
})();
