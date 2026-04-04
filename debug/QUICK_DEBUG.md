# 快速调试指南

## 方法 1: 查看 Console 日志（最简单）

1. **打开 PCB 文档**

2. **按 F12** 打开开发者工具

3. **切换到 Console 标签**

4. **运行自动布线**
   - 菜单栏 → 橡皮筋自动布线 → 开始自动布线

5. **查看 Console 输出**
   - 查找 `[Bypass]` 开头的日志
   - 截图并发给我

## 方法 2: 运行调试脚本

1. **打开 PCB 文档**

2. **按 F12** 打开开发者工具

3. **切换到 Console 标签**

4. **复制粘贴以下内容到 Console 并按回车**:

```javascript
(async () => {
    console.log('=== DRC 调试开始 ===');
    
    const allLines = await eda.pcb_PrimitiveLine.getAll();
    console.log('走线数量:', allLines ? allLines.length : 0);
    
    let crossCount = 0;
    if (allLines && allLines.length > 1) {
        for (let i = 0; i < allLines.length; i++) {
            for (let j = i + 1; j < allLines.length; j++) {
                const l1 = allLines[i], l2 = allLines[j];
                if (l1.layer === l2.layer && l1.net !== l2.net) {
                    // 简单交叉检查
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
    
    const drcPassed = await eda.pcb_Drc.check(true, true, false);
    console.log('交叉数量:', crossCount);
    console.log('DRC 检查:', drcPassed ? '通过' : '失败');
    console.log('=== 调试结束 ===');
})();
```

5. **发送输出结果**

## 需要提供的信息

请提供以下**任意一项**:

### 选项 A: Console 日志（推荐）
```
[Bypass] 开始生成绕行路径：...
[Bypass] 找到 X 个阻挡障碍物
...
```

### 选项 B: 调试脚本输出
```
=== DRC 调试开始 ===
走线数量：X
交叉数量：X
DRC 检查：失败
```

### 选项 C: 截图
- 布线后的 PCB 截图（显示交叉）
- Console 日志截图

---

**版本**: v2.7.3  
**构建产物**: `build/dist/rubberband-autorouter_v2.7.3.eext`
