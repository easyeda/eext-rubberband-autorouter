# 橡皮筋自动布线  (eext-rubberband-autorouter)

## 项目概述

**橡皮筋自动布线 ** 是一个为嘉立创 EDA 专业版开发的 PCB 自动布线扩展，使用纯 JavaScript/TypeScript 实现拓扑橡皮筋算法。该扩展能够在 PCB 设计中自动完成导线连接，支持智能避障、过孔生成和 DRC 检查。

### 核心特性

- ✅ **本地算法** - 纯 TypeScript 实现，无需外部服务
- ✅ **拓扑路由** - 基于 Delaunay 三角剖分的拓扑橡皮筋算法
- ✅ **智能避障** - 动态障碍物检测，自动绕行
- ✅ **过孔生成** - 自动检测层变化并创建过孔
- ✅ **DRC 检查** - 走线间距验证，避免 DRC 错误
- ✅ **路径优化** - 橡皮筋算法优化路径，减少拐点
- ✅ **多层焊盘支持** - 智能处理 TH、SMD、Via 多种焊盘类型

### 当前版本

- **版本号**: v2.10.17
- **发布日期**: 2026-03-26
- **主要功能**: 多层焊盘智能换层、动态障碍物地图、DRC 详细日志

---

## 项目结构

```
eext-rubberband-autorouter/
├── src/
│   ├── router/                    # 路由器模块
│   │   ├── DynamicObstacleMap.ts  # 动态障碍物地图
│   │   ├── DRCChecker.ts          # DRC 检查器
│   │   ├── ObstacleBypasser.ts    # 障碍物绕行器
│   │   ├── PathOptimizer.ts       # 路径优化器
│   │   └── TopologicalRouter.ts   # 拓扑路由器（主路由器）
│   ├── router-core/               # 核心算法
│   │   ├── core/                  # Router, Vertex, Region...
│   │   ├── algorithms/            # Delaunay, A* 算法
│   │   ├── geometry/              # 几何计算工具
│   │   └── parsers/               # PCBParser, StyleParser
│   ├── types/
│   │   └── index.ts               # TypeScript 类型定义
│   ├── utils/
│   │   ├── ErrorHandler.ts        # 错误处理器
│   │   └── toast.ts               # 吐司通知工具
│   └── index.ts                   # 扩展主入口
├── iframe/
│   └── routing.html               # 配置界面 HTML
├── build/                         # 构建输出目录
├── dist/                          # 编译输出目录
├── extension.json                 # EDA 扩展配置
├── package.json                   # NPM 项目配置
├── tsconfig.json                  # TypeScript 配置
└── README.md                      # 项目说明文档
```

---

## 技术栈

| 类别 | 技术 | 版本 |
|------|------|------|
| **语言** | TypeScript | ^5.0.0 |
| **运行时** | JavaScript (ESNext) | - |
| **构建工具** | esbuild | ^0.19.0 |
| **核心库** | delaunator | ^5.0.0 |
| **工具库** | jszip, fs-extra | ^3.10.0, ^11.0.0 |
| **目标平台** | 嘉立创 EDA 专业版 | ^3.2.0 |

---

## 构建与运行

### 环境要求

- Node.js >= 20.0.0
- npm >= 9.0.0
- 嘉立创 EDA 专业版 >= 3.2.0

### 安装依赖

```bash
npm install
```

### 构建命令

```bash
# 开发模式（监听文件变化）
npm run watch

# 生产构建
npm run build

# 打包扩展 (.eext 文件)
npm run package
```

### 安装扩展

#### 方法 1：从 .eext 文件安装

1. 运行 `npm run package` 生成 `.eext` 文件
2. 打开嘉立创 EDA 专业版
3. 进入 **扩展管理** (Extensions)
4. 点击 **从本地安装**
5. 选择 `build/dist/rubberband-autorouter_v{version}.eext`
6. 安装完成

#### 方法 2：开发模式安装

1. 在嘉立创 EDA 中选择 **从本地文件夹安装**
2. 选择项目根目录
3. 修改代码后运行 `npm run build` 重新构建
4. 在 EDA 中重新加载扩展

---

## 使用方法

### 快速布线

1. 打开 PCB 文档
2. 菜单栏 → **橡皮筋自动布线** → **开始自动布线**
3. 等待布线完成

### 查看版本

1. 菜单栏 → **橡皮筋自动布线** → **关于**
2. 显示当前版本号（如 v2.10.17）

### 布线日志

按 **F12** 打开浏览器开发者工具，切换到 **Console** 标签，查看实时日志：

```
[Router2] [DRC] 最终设计规则：线宽=10.00mil, 间距=5.98mil
[Router2] [Pad] 总焊盘数：70
[Router2] [Router] 焊盘 A 层=12, 出线层=1; 焊盘 B 层=1, 出线层=1, 需要过孔=false
[Router2] [Router] 检查路径：GND, 有障碍：true
[Router2] [Router] 生成绕行路径...
[Router2] [Router] 绕行点坐标：(1203.2,885.5)
[Router2] [Router] 创建过孔：1 -> 2 @ (x,y)
```

---

## 核心算法

### 三级寻路策略

```
1. A* 寻路（Delaunay 图）
   ↓ 失败
2. 障碍物绕行算法
   ↓ 失败
3. 降级直连（最后手段）
```

#### 1. A* 寻路

基于 Delaunay 三角剖分生成的拓扑图，使用改进的 A* 算法：
- 改进的启发式函数
- 动态障碍物检测
- 边代价优化（异网络 +50%，异层 +20%，朝向目标奖励 30%）

#### 2. 障碍物绕行算法

当 A* 失败时，使用橡皮筋绕行算法：
- 检测阻挡的障碍物
- 在障碍物两侧生成绕行点
- 尝试左侧和右侧绕行
- 多点绕行（复杂障碍物）
- 多距离尝试（50%-200% 连线长度，10-50 倍间距）

#### 3. 降级直连

当前两种方法都失败时，尝试直接连接。

### 层处理规则

| 焊盘 A | 焊盘 B | 布线层 | 过孔 |
|--------|--------|--------|------|
| 顶层 (1) | 顶层 (1) | 顶层 | 否 |
| 底层 (2) | 底层 (2) | 底层 | 否 |
| 顶层 (1) | 底层 (2) | 顶→底 | 是 ✅ |
| 底层 (2) | 顶层 (1) | 底→顶 | 是 ✅ |
| 多层 (12) | 顶层 (1) | 顶层 | 否 |
| 多层 (12) | 底层 (2) | 底层 | 否 |
| 多层 (12) | 多层 (12) | 顶层 | 否 |

**注意**: layerId=12 表示多层焊盘（通孔），可以在顶层或底层连接，但**不能在 12 层走线**。

### 算法流程

```
PCB 数据解析
    ↓
DRC 规则读取
    ↓
焊盘收集 + 障碍物构建
    ↓
Delaunay 三角剖分
    ↓
三级寻路（A* → 绕行 → 直连）
    ↓
路径优化
    ↓
过孔生成
    ↓
走线创建
    ↓
DRC 检查
```

---

## 核心模块

| 模块 | 文件 | 功能 |
|------|------|------|
| **主路由器** | `TopologicalRouter.ts` | 协调布线流程 |
| **动态障碍物地图** | `DynamicObstacleMap.ts` | 动态维护障碍物 |
| **DRC 检查器** | `DRCChecker.ts` | 碰撞检测 |
| **障碍物绕行器** | `ObstacleBypasser.ts` | 生成绕行路径 |
| **路径优化器** | `PathOptimizer.ts` | 优化路径质量 |
| **错误处理器** | `ErrorHandler.ts` | 错误分类和重试 |

---

## 开发规范

### 代码风格

- 使用 TypeScript 严格模式
- 遵循 ESLint 配置 (`.eslintrc.js`)
- 代码格式化使用 Prettier (`.prettierrc.js`)

### 提交规范

- 功能新增：`feat: 描述`
- Bug 修复：`fix: 描述`
- 文档更新：`docs: 描述`
- 代码重构：`refactor: 描述`

### 版本管理

遵循语义化版本 (SemVer)：
- **主版本号**: 不兼容的 API 变更
- **次版本号**: 向后兼容的功能新增
- **修订号**: 向后兼容的问题修正

---

## 常见问题

### Q: 布线时遇到障碍物怎么办？

A: v2.10.17 会自动使用绕行算法：
1. 首先尝试 A* 寻路
2. 失败后自动生成绕行路径
3. 日志会提示"生成绕行路径..."

### Q: 为什么有时还是会直连？

A: 当绕行算法也无法找到安全路径时，会降级为直连：
- 障碍物过于密集
- 起点和终点被完全包围
- 建议：增加最大轮数或手动预布关键走线

### Q: 如何提高布线成功率？

A: 建议：
1. 增加最大轮数（如 200-500）
2. 简化 PCB 布局，减少元件密度
3. 手动预布一些关键走线（差分对、时钟线等）
4. 合理设置 DRC 规则

### Q: 支持多层板吗？

A: v2.10.17 已支持过孔自动生成，可以处理双层板布线。多层板支持正在进一步优化中。

### Q: 日志提示"添加过孔换层"是什么意思？

A: 表示检测到起点和终点在不同层，自动在终点位置创建过孔进行换层连接。

---

## 许可证

Apache-2.0

---

## 相关链接

- [嘉立创 EDA 官网](https://lceda.cn/)
- [扩展开发文档](https://prodocs.lceda.cn/cn/api/guide/)
- [问题反馈](https://github.com/easyeda/eext-rubberband-autorouter/issues)
- [更新日志](./CHANGELOG.md)

---

## 贡献

欢迎提交 Issue 和 Pull Request！

---

## 支持

如有问题，请：
1. 查看 [常见问题](#常见问题)
2. 查看 [更新日志](./CHANGELOG.md)
3. 提交 [Issue](https://github.com/easyeda/eext-rubberband-autorouter/issues)
4. 联系嘉立创技术支持
