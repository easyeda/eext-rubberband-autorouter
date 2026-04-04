# 橡皮筋自动布线

基于拓扑橡皮筋算法的 PCB 自动布线工具，使用纯 JavaScript 实现，无需外部服务。

## 特性

- ✅ **本地算法** - 纯 JavaScript 实现，无需启动外部服务
- ✅ **拓扑路由** - 基于 Delaunay 三角剖分的拓扑橡皮筋算法
- ✅ **实时反馈** - 进度条和日志实时更新
- ✅ **简洁界面** - 简化的配置面板，易于使用
- ✅ **高性能** - 直接执行，无通信延迟
- ✅ **增强焊盘识别** - 支持 TH、SMD、Via 多种焊盘类型
- ✅ **过孔生成** - 自动检测层变化并创建过孔
- ✅ **路径优化** - 橡皮筋算法优化路径，减少拐点
- ✅ **智能避障** - 动态障碍物检测，自动绕行
- ✅ **错误处理** - 完善的错误分类和自动重试机制
- ✅ **DRC 检查** - 走线间距验证，避免 DRC 错误

## 安装

### 方法 1：从 .eext 文件安装

1. 下载 `rubberband-autorouter_v2.7.0.eext` 文件
2. 打开嘉立创 EDA 专业版
3. 进入 **扩展管理** (Extensions)
4. 点击 **从本地安装** (Install from local)
5. 选择下载的 .eext 文件
6. 安装完成

### 方法 2：从源码安装（开发模式）

```bash
# 克隆项目
git clone https://github.com/easyeda/eext-rubberband-autorouter.git
cd eext-rubberband-autorouter

# 安装依赖
npm install

# 构建项目
npm run build

# 打包扩展
npm run package
```

然后在嘉立创 EDA 中选择 **从本地文件夹安装**，选择项目根目录。

---

## 使用方法

### 快速布线

1. 打开 PCB 文档
2. 菜单栏 → **橡皮筋自动布线** → **开始自动布线**
3. 等待布线完成

使用默认参数（10 轮），直接开始布线。

### 自定义布线

1. 打开 PCB 文档
2. 菜单栏 → **橡皮筋自动布线** → **关于** 查看版本信息
3. 在配置面板中设置参数：
   - **最大轮数**：布线的最大迭代次数（1-1000）
4. 点击 **开始布线**
5. 查看实时日志和进度条
6. 等待布线完成

---

## 核心算法

### 三级寻路策略

本扩展使用三级寻路策略，确保最佳布线效果：

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
- 在障碍物两侧 45° 方向生成绕行点
- 尝试左侧和右侧绕行
- 多点绕行（复杂障碍物）
- 路径优化（移除不必要的点）

#### 3. 降级直连

当前两种方法都失败时，尝试直接连接。

---

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
```

---

## 技术架构

```
EDA 扩展
    ↓
TopologicalRouter (主路由器)
    ├── DRCChecker (DRC 检查器)
    ├── ObstacleBypasser (障碍物绕行器)
    ├── PathOptimizer (路径优化器)
    └── ErrorHandler (错误处理器)
    ↓
router-core (核心算法)
    ├── Delaunay 三角剖分
    ├── A* 路径搜索
    └── 几何计算工具
    ↓
布线结果导入
```

### 核心模块

| 模块 | 文件 | 功能 |
|------|------|------|
| **主路由器** | `TopologicalRouter.ts` | 协调布线流程 |
| **DRC 检查器** | `DRCChecker.ts` | 碰撞检测 |
| **障碍物绕行器** | `ObstacleBypasser.ts` | 生成绕行路径 |
| **路径优化器** | `PathOptimizer.ts` | 优化路径质量 |
| **错误处理器** | `ErrorHandler.ts` | 错误分类和重试 |

---

## 项目结构

```
eext-rubberband-autorouter/
├── src/
│   ├── router/
│   │   ├── TopologicalRouter.ts    # 主路由器
│   │   ├── DRCChecker.ts           # DRC 检查器
│   │   ├── ObstacleBypasser.ts     # 障碍物绕行器
│   │   └── PathOptimizer.ts        # 路径优化器
│   ├── router-core/                # 核心算法
│   │   ├── core/                   # Router, Vertex, Region...
│   │   ├── algorithms/             # Delaunay, FibonacciQueue
│   │   ├── geometry/               # GeometryUtils, Tangents...
│   │   ├── parsers/                # PCBParser, StyleParser
│   │   └── utils/                  # Constants, Units
│   ├── types/
│   │   └── index.ts                # 类型定义
│   ├── utils/
│   │   ├── ErrorHandler.ts         # 错误处理
│   │   └── toast.ts                # 吐司通知
│   └── index.ts                    # 主入口
├── iframe/
│   └── routing.html                # 配置界面
├── extension.json                  # 扩展配置
├── package.json                    # 项目配置
└── tsconfig.json                   # TS 配置
```

---

## 开发

### 构建命令

```bash
# 安装依赖
npm install

# 开发模式（监听文件变化）
npm run watch

# 构建项目
npm run build

# 打包扩展
npm run package
```

### 调试

1. 在嘉立创 EDA 中以开发模式安装扩展
2. 修改源代码
3. 运行 `npm run build` 重新构建
4. 在 EDA 中重新加载扩展

---

## 依赖

### 运行时依赖

- **delaunator** ^5.0.0 - Delaunay 三角剖分库

### 开发依赖

- **TypeScript** ^5.0.0 - 类型安全的 JavaScript
- **esbuild** ^0.19.0 - 快速的 JavaScript 打包工具
- **tsx** ^4.0.0 - TypeScript 执行器
- **jszip** ^3.10.0 - ZIP 文件处理
- **fs-extra** ^11.0.0 - 文件操作

---

## 常见问题

### Q: 布线时遇到障碍物怎么办？

A: v2.7.0 会自动使用绕行算法：
1. 首先尝试 A* 寻路
2. 失败后自动生成绕行路径
3. 日志会提示"使用绕行算法"

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

A: v2.7.0 已支持过孔自动生成，可以处理双层板布线。多层板支持正在进一步优化中。

### Q: 与 FreeRouting 有什么区别？

A: 主要区别：
- **本地执行**：无需启动外部服务
- **纯 JavaScript**：更易于集成和维护
- **简化界面**：更简洁的配置选项
- **算法不同**：使用拓扑橡皮筋算法
- **v2.7.0 增强**：障碍物绕行、DRC 检查、路径优化

### Q: 日志提示"使用绕行算法"是什么意思？

A: 表示 A* 寻路失败，但绕行算法成功生成了绕行路径。这是正常现象，说明算法正在工作。

---

## 许可证

Apache-2.0

---

## 致谢

- 原 FreeRouting 集成项目
- Ruby PCB Router 核心算法
- 嘉立创 EDA 团队
- Delaunator 库作者

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
