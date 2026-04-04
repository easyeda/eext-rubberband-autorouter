/**
 * 检查网络基元数据结构
 */
(async () => {
    console.log('=== 检查网络基元数据 ===');
    
    if (typeof eda === 'undefined') {
        console.error('❌ eda 未定义');
        return;
    }
    
    // 获取所有网络
    const allNets = await eda.pcb_Net.getAllNetsName();
    console.log('网络数量:', allNets ? allNets.length : 0);
    console.log('网络列表:', allNets);
    
    // 检查每个网络的基元
    for (const netName of allNets) {
        console.log('\n=== 网络:', netName, '===');
        
        try {
            const primitives = await eda.pcb_Net.getAllPrimitivesByNet(netName);
            console.log('基元数量:', primitives ? primitives.length : 0);
            
            if (primitives && primitives.length > 0) {
                console.log('前 3 个基元详情:');
                for (let i = 0; i < Math.min(3, primitives.length); i++) {
                    const p = primitives[i];
                    console.log('  基元 ' + (i+1) + ':');
                    console.log('    类型:', p.type || 'unknown');
                    console.log('    所有属性:', Object.keys(p));
                    
                    // 检查常用属性
                    const anyP = p as any;
                    console.log('    center:', anyP.center);
                    console.log('    topWidth:', anyP.topWidth);
                    console.log('    topHeight:', anyP.topHeight);
                    console.log('    holeRadius:', anyP.holeRadius);
                    console.log('    layerId:', anyP.layerId);
                    console.log('    globalIndex:', anyP.globalIndex);
                    console.log('    net:', anyP.net);
                    console.log('    isVia:', anyP.isVia);
                }
            }
        } catch (err) {
            console.error('获取基元失败:', err.message);
        }
    }
    
    console.log('\n=== 检查完成 ===');
})();
