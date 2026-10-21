# 橡皮筋自动布线

基于拓扑橡皮筋算法的 PCB 自动布线工具，使用纯 JavaScript 实现，无需外部服务。

## 特性

- ✅ **本地算法** - 纯 JavaScript 实现，无需启动外部服务
- ✅ **拓扑橡皮筋** - 可视图 + Dijkstra 求精确切线圆弧路径，凸包拉紧效果
- ✅ **Worker 并行** - 规划在 Web Worker 中执行，不阻塞 EDA 界面
- ✅ **参数弹窗** - 线宽、间距、逃逸长度、圆角、撕毁轮数等可视化调整
- ✅ **实时进度** - 进度、统计、日志实时刷新，可随时手动停止
- ✅ **实时写入** - 每个网络完成后立即刷新到 PCB 画布
- ✅ **圆润扇出** - 根据焊盘长宽/旋转从端帽出线，切线圆角过渡
- ✅ **增强焊盘识别** - 支持 TH、SMD、Via 多种焊盘类型
- ✅ **过孔生成** - 自动检测层变化并创建过孔
- ✅ **智能避障** - 圆形障碍群 + 动态走线障碍，严格满足 DRC 间距
- ✅ **撕毁重布** - 失败网络自动拆除封路网络后重试

## 安装

### 方法 1：从 .eext 文件安装

1. 下载 `rubberband-autorouter_v0.0.39.eext` 文件
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

1. 打开 PCB 文档
2. 菜单栏 → **橡皮筋自动布线** → **开始自动布线**
3. 在弹出的参数窗口中调整参数，点击 **开始布线**
4. 进度窗口会实时显示进度、统计与日志；可随时点击 **停止布线**
5. 布线过程中每个网络完成后立即写入画布，实时可见

### 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| 走线宽度 | DRC 默认 | 同时影响障碍外扩半径 |
| 额外安全间距 | 0 | 在 DRC 规则之上增加的余量 |
| 逃逸线长度 | Pad 间距 + 线宽 + 20mil | 焊盘出线后的直行距离 |
| 扇出圆角半径 | min(40, 线宽×4) | 出线段与路径之间的切线圆角，0 = 直角 |
| 撕毁重布轮数 | 2 | 失败网络拆除封路网络后重试的轮数 |
| 时间预算 | 300 秒 | 超时后停止重试；可在进度窗口手动停止 |
| 电源网络优先 | 开 | GND/VCC/VDD 等网络先布 |
| 实时预览 | 开 | 每布完一个网络立即刷新 PCB |

---

## 核心算法

### 拓扑橡皮筋（可视图 + Dijkstra）

1. 焊盘/走线/填充/板框转为圆形障碍群（胶囊、多边形角点、内部阻挡网格）
2. 构建圆与圆之间的公切线、圆自身的圆弧边，形成可视图
3. 在可视图上运行 Dijkstra，得到严格满足间距的最短路径
4. 路径输出为精确图元（直线 + 圆弧），形成“绳子拉紧”的凸包效果

### 焊盘扇出

- `padExitModel` 根据焊盘长宽与旋转，从端帽沿长轴垂直出线
- 出线段与路径首/末段之间插入 `fanoutFillet` 切线圆角
- 每个焊盘的逃逸通道注册为网络专属预约，避免其他网络横穿

### 撕毁重布

失败网络会识别封路的已布网络，拆除后按外向优先顺序重布，再重布被拆网络；
支持多轮迭代与最终成组重布。

### Worker 并行

- 纯几何规划（MST、逃逸点、可视图、圆角、撕毁重布）在 Web Worker 中执行
- 主线程只负责 EDA API 读写、画布刷新与统计
- Worker 不可用时自动回退到主线程（协作式让出，界面仍可响应）

---

## 技术架构

```
主线程 (EDA 宿主)
  ├── 读取 DRC / 焊盘 / 静态障碍
  ├── 逐网络写入图元（实时刷新）与删除（撕毁）
  └── 参数弹窗 / 进度弹窗（postMessage 通信）
        │  结构化数据
        ▼
Web Worker
  ├── PlanningEngine（MST / 逃逸点 / 撕毁重布）
  ├── VisibilityRouter（可视图 + Dijkstra）
  └── RubberbandEngine（切线 / 圆群 / 圆角）
```

### 核心模块

| 模块 | 文件 | 功能 |
|------|------|------|
| **EDA 路由宿主** | `src/router/topological-router.ts` | 数据收集、Worker 调度、实时写入/删除、统计 |
| **规划引擎** | `src/router/planning-engine.ts` | 纯几何规划（零 EDA 依赖） |
| **可视图路由** | `src/router/visibility-router.ts` | 切线图 + Dijkstra |
| **几何原语** | `src/router/rubberband-engine.ts` | 障碍圆群、逃逸点、圆角 |
| **Worker 入口** | `src/worker/planning-worker.ts` | 规划消息协议与进度 |
| **弹窗桥** | `src/ui/dialog-bridge.ts` | 主线程 ↔ iframe 消息 |

---

## 项目结构

```
eext-rubberband-autorouter/
├── src/
│   ├── router/
│   │   ├── topological-router.ts   # EDA 读写 + Worker 调度 + 实时写入
│   │   ├── planning-engine.ts      # 纯几何规划引擎（零 EDA 依赖）
│   │   ├── rubberband-engine.ts    # 切线/障碍圆群/圆角等几何原语
│   │   ├── visibility-router.ts    # 可视图 + Dijkstra
│   │   ├── drc-rules.ts            # DRC 规则读取与参数覆盖
│   │   └── worker-source.ts        # Worker 源码内联与创建
│   ├── worker/
│   │   └── planning-worker.ts      # Worker 入口（规划与进度消息）
│   ├── ui/
│   │   └── dialog-bridge.ts        # 主线程 ↔ 内联框架消息桥
│   ├── types/
│   │   ├── index.ts                # 基础类型
│   │   └── router-params.ts        # 用户可调参数与统计
│   ├── utils/
│   │   ├── error-handler.ts        # 错误处理
│   │   ├── logger.ts               # 日志
│   │   └── toast.ts                # 吐司通知
│   ├── auto-router-facade.ts       # 参数弹窗 / 进度弹窗 / 布线调度
│   └── index.ts                    # 主入口
├── iframe/
│   ├── params.html                 # 参数设置弹窗
│   └── progress.html               # 实时进度与日志弹窗
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
