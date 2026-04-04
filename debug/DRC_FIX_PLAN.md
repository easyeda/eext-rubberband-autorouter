# DRC 间距错误修复方案

## 问题分析

DRC 错误显示：
```
间距错误 贴片焊盘 到 导线 距离为 0mil，应该>= 6mil
间距错误 导线 到 导线 距离为 0mil，应该>= 4mil
```

**根本原因**：
1. 走线直接从焊盘中心开始
2. 没有考虑焊盘本身的尺寸（125x134mil）
3. 走线路径穿过其他焊盘区域

## 修复方案

### 方案 1: 在焊盘边缘开始/结束走线

计算从焊盘中心到边缘的方向向量，在边缘处开始走线：

```typescript
// 计算焊盘边缘的起点/终点
function getPadEdgePoint(pad: PadInfo, targetPoint: Point): Point {
    const dx = targetPoint.x - pad.x;
    const dy = targetPoint.y - pad.y;
    const dist = Math.sqrt(dx * dx + dy * dy);
    
    if (dist < 0.001) return { x: pad.x, y: pad.y };
    
    // 焊盘半径 + 安全间距
    const padRadius = pad.radius + clearanceMil;
    
    // 在焊盘边缘的点
    return {
        x: pad.x + (dx / dist) * padRadius,
        y: pad.y + (dy / dist) * padRadius
    };
}
```

### 方案 2: 添加路径优化，避免穿过其他焊盘

在生成路径后，检查并优化穿过其他焊盘的线段。

### 方案 3: 使用 A* 寻路时考虑焊盘排斥区域

修改 `findPath` 函数，增加对非目标焊盘的排斥力，避免路径穿过。

## 建议实施

先实施**方案 1**，这是最简单有效的修复方法。

---

**版本**: v2.7.6 (待发布)
