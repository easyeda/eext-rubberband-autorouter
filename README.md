<p align="center"><img src="images/logo.png" width="112" alt="橡皮筋自动布线器" /></p>

# 橡皮筋自动布线

基于拓扑橡皮筋算法的嘉立创 EDA 专业版 PCB 自动布线扩展：纯本地算法，无需外部服务，支持多引擎与自动拉紧圆润。

**中文** · [English](#english)

---

## 目录

**用户指南**

- [特性](#特性)
- [安装](#安装)
- [快速开始](#快速开始)
- [布线引擎选择](#布线引擎选择)
- [参数说明](#参数说明)
- [网络排序策略](#网络排序策略)
- [常见问题](#常见问题)

**开发者文档**

- [核心算法](#核心算法)
- [技术架构](#技术架构)
- [核心模块](#核心模块)
- [项目结构](#项目结构)
- [构建与调试](#构建与调试)
- [依赖](#依赖)
- [版本与提交规范](#版本与提交规范)

**English**

- [Features](#features)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Routing Engines](#routing-engines)
- [Parameters](#parameters)
- [FAQ](#faq)
- [For Developers](#for-developers)

---

# 用户指南

## 特性

- ✅ **本地算法** — 纯 JavaScript 实现，无需启动外部服务，设计数据不出本机
- ✅ **三种布线引擎** — RRouter（插件自带拓扑橡皮筋）/ JRouter / HRouter（EDA 内置引擎），一键切换
- ✅ **拓扑橡皮筋** — 可视图 + Dijkstra 求精确切线圆弧路径，走线像绳子一样拉紧
- ✅ **自动拉紧圆润** — 内置引擎（J/H）布完后自动做橡皮筋拉紧与切线圆角优化
- ✅ **实时进度** — 进度、布通率、飞线布通率、统计、日志实时刷新，可随时停止
- ✅ **边布线边圆润** — RRouter 每布完一个网络立即写入画布并做圆润处理，过程实时可见
- ✅ **自动网络排序** — 短连接 → 一般信号 → 电源 → 地；地网络始终最后布
- ✅ **单层优先** — 优先同层走通，绕不过去才换层；过孔自动校验与相邻焊盘/走线的 DRC 间距
- ✅ **部分连通保留** — 网络中某条连接失败时，已布通的连接照常保留，不整网丢弃
- ✅ **撕毁重布** — 失败网络自动拆除封路网络后重试，支持多轮
- ✅ **增强焊盘识别** — TH、SMD、POLYGON、椭圆/圆角矩形（含旋转）焊盘精确建模
- ✅ **DRC 严格口径** — 全部图元终检，任何一段不过间距即弃边，宁可少布不产违规

## 安装

### 方法 1：从 .eext 文件安装（推荐）

1. 从 [Releases](https://github.com/easyeda/eext-rubberband-autorouter/releases) 下载最新的 `rubberband-autorouter_v*.eext`
2. 打开嘉立创 EDA 专业版
3. 进入 **扩展管理** (Extensions) → **从本地安装**
4. 选择下载的 .eext 文件
5. 安装完成，菜单栏出现 **橡皮筋布线器**

### 方法 2：从源码安装（开发模式）

```bash
git clone https://github.com/easyeda/eext-rubberband-autorouter.git
cd eext-rubberband-autorouter
npm install
npm run build
```

然后在嘉立创 EDA 中选择 **扩展管理 → 从本地文件夹安装**，选择项目根目录。

## 快速开始

1. 打开 PCB 文档
2. 菜单栏 → **橡皮筋布线器** → **开始布线...**
3. 参数窗口贴着 EDA 界面右侧弹出；按需调整参数后点击 **开始布线**
4. 进度窗口实时显示进度、**布通率**（按网络）与**飞线布通率**（按焊盘）；可随时 **停止布线**
5. 布线过程中每个网络完成后立即写入画布，实时可见
6. 完成后可 **重新布线**（按上次参数重跑）或 **布线设置**（改参数再跑）

## 布线引擎选择

参数窗口**顶部**可并排选择三种引擎，问号图标悬停可查看引擎说明：

| 引擎 | 来源 | 适用场景 | 行为 |
|------|------|----------|------|
| **RRouter**（默认） | 插件自带 | 常规板，追求走线形态 | 拓扑橡皮筋引擎，边布线边圆润，支持全部参数 |
| **JRouter** | EDA 内置 | 密排连接器板，布通率优先 | 调用内置自动布线 API，完成后插件自动做拉紧 + 圆润优化 |
| **HRouter** | EDA 内置 | 追求布线速度 | 内置引擎速度优先（FASTER）→ 插件后处理拉紧圆润 |

- 选择 JRouter / HRouter 时，仅适用于 RRouter 的参数（走线宽度、网络排序、撕毁重布、实时预览、凸包优化、过孔兜底）会自动隐藏——内置引擎使用自身规则
- RRouter 布通率偏低（<85%）时，日志会建议切换到 JRouter / HRouter
- J/H 引擎的走线写入画布后，插件会对结果做橡皮筋拉紧与切线圆角，再刷新画布

## 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| 布线引擎 | RRouter | 见[布线引擎选择](#布线引擎选择) |
| 走线宽度 | 10 mil | **仅 RRouter**。铜线宽度，同时作为障碍外扩基准；越宽越难穿插。 |
| 额外安全间距 | 0 mil | 在 DRC 最小间距之上**再加**的余量，0 = 严格按 DRC 执行。 |
| 出线额外余量 | 0 mil | 焊盘边缘到走线中心线在 DRC 之外再保留的距离，0 = 按最小间距贴焊盘出线。 |
| 撕毁重布轮数 | 3 轮 | **仅 RRouter**。网络布不通时，允许拆除挡路的已布网络后重试的轮数；0 = 关闭。 |
| 时间预算 | 30 分钟 | 全部网络首轮布线、撕毁重布、优化、冲突消除与过孔兜底共用的规划总时长。到期保留已布走线。引擎内部上限 60 分钟。 |
| 网络排序 | 自动规划 | **仅 RRouter**。见[网络排序策略](#网络排序策略)。 |
| 实时预览 | 开 | **仅 RRouter**。每布完一个网络立即写画布；关闭后规划完统一写入，界面更流畅。 |
| 最后凸包优化 | 开 | **仅 RRouter**。先保证连通，再逐网重求最短路，只有更短才替换。 |
| 过孔换层兜底 | 开 | **仅 RRouter**。同层绕不过去时自动打孔换层；关闭底层后自动失效。 |
| 布线层 | 顶层 + 底层 | 按当前 PCB 实际可用铜层生成；顶层固定启用，单面板自动禁用底层。 |
| 排除网络 / 地网络 | 默认排除 GND | 勾掉不参与布线的网络（保持画布现状，焊盘与走线仍作为障碍）。地网络默认不参与布线。 |

## 网络排序策略

默认「自动规划」按 PCB Layout 通行做法分档：

1. **短连接**（两焊盘且跨度 ≤ 板对角线 22%）——先处理，占用少、成功率高
2. **一般信号**——按河流方向排序，同向网络成束，减少交叉
3. **电源网络**（VCC/VDD/VBAT/±5V 等）——线宽/电流大，放在信号之后
4. **地网络**（GND/VSS/0V/EARTH 等）——**永远最后布**：地焊盘最多、连接最像"网"，
   先布会把板面切碎；最后布则可利用剩余空间就近打孔绕行

地网络判定覆盖 `GND`、`AGND`、`DGND`、`PGND`、`VSS`、`0V`、`EARTH` 等常见命名；
手动选择其他排序模式时，地网络仍强制最后布（硬规则）。

「自动规划」与「河流」模式下还会识别**并排焊盘**（连接器/排针/IC 引脚共线排）：
排内网络改为**中间 → 两端**出线，避免外侧走线绕成包围圈封死中间焊盘的唯一出口。

## 常见问题

### Q: 布通率不理想怎么办？

A: 依次尝试：
1. 看日志——RRouter 布通率 <85% 时会明确建议切换引擎，切到 **JRouter / HRouter** 试试（密排连接器板内置引擎明显更强）
2. 增加「撕毁重布轮数」（默认 3）
3. 保持「过孔换层兜底」开启（双面板几乎必开）
4. 简化布局密度 / 手工预布关键走线（差分对、时钟线）

### Q: 走线为什么会绕远路？

A: 绕路有两个来源，都已针对性处理：
1. **出线方向被预设**——现在主模型是焊盘中心直连最短路，出线方向即最短路起始方向
2. **顶层通道被占**——优化与冲突消除阶段做逐网/成组重排；换层只作兜底，不随意打孔

### Q: 画布上多出的小圆点是什么？是测试点吗？

A: 不是。插件没有放置测试点的逻辑，小圆点是**过孔**（换层连接）。过孔只会落在焊盘之外，并按 DRC 间距校验与相邻焊盘/走线的距离。

### Q: 支持多层板吗？

A: 双面板（顶层 + 底层）完整验证过；当前算法只使用顶/底两层，内层尚未参与布线。单面板自动禁用底层走线。

### Q: 与 FreeRouting 有什么区别？

A: **本地执行**（无需外部服务）、**纯 JavaScript**（易集成维护）、**拓扑橡皮筋算法**（切线圆弧路径，形态自然）、**深度集成 EDA**（直接读写画布、读 DRC 规则、可切换 EDA 内置引擎）。

---

# 开发者文档

## 核心算法

### RRouter：中心到中心 + 单层优先

1. **主模型**：焊盘中心 → 可视图最短路 → 焊盘中心。本网焊盘不作为障碍，
   出线方向就是最短路的起始方向——走线天然切线连续；直线可连时总长恰等于两焊盘直线距离。
2. **单层优先**：每条连接先在当前层求解；同层确实绕不过去才打孔换层。
   顶层绕路阈值 1.55× 下界：绕到对侧省的长度抵不过两个过孔的代价时宁可换层。
3. **密集排退避**：邻盘外扩间距圈吞没本焊盘中心时，退回「焊盘轮廓出线候选」模型，
   引出段与路径之间切线圆角过渡；完全落在自家铜皮内的侵入直接豁免。
4. **过孔兜底**：按「两端引出段 + 跨层直线距离」给锚点对排序，逐对尝试
   「引出 → 过孔 → 对层路径 → 过孔 → 引入」；焊盘外过孔按原始形状（矩形/椭圆/圆角矩形含旋转）精确校验 DRC。
5. **网格 A\* 兜底**：可视图盲区（端点被邻盘净距圈覆盖、必经点不在切点图）用网格 A\* 兜底，
   输出经 string-pull 拉直 + 逐段终检后入库。

### JRouter / HRouter 委托管线

```
purge 待布网络旧走线 → autoRouting BETA API（engine 属性透传）
  → 结果落盘画布 → 回读 Line/Arc/Via 几何
  → postProcessImportedNets（登记障碍 → 逐网拉紧 + 圆润）
  → 整批清除原几何 → 逐网回写 → DRC 复核与统计
```

- **先落盘再优化再刷新**：内置路由器直接写画布、无插入前钩子，因此后处理在其完成后进行
- 内置引擎「成功」语义宽于全连通：DRC 复核会把仍有 Connection Error 的网降级为失败
- BETA API 批量过大时可能返回 undefined（已加防护，静默跳过不崩溃）

### 优化、冲突消除与收尾对账

- **优化阶段**：逐网拆除重布，只有总长更短才替换；被否决的尝试连引擎状态与画布一起完整回滚
- **冲突消除**：对瓶颈走廊做成组重排（全走廊探测、内向优先）
- **收尾对账**：规划结束后按最终规划逐网清写画布，保证画布与引擎终态一致
- **部分连通保留**：某条 MST 连接失败时其余连接立即提交保留，重试只补未完成的边

### 拓扑橡皮筋（可视图 + Dijkstra）

1. 焊盘/走线/填充/板框转为圆形障碍群（胶囊、多边形角点、内部阻挡网格）
2. 构建圆与圆之间的公切线、圆自身的圆弧边，形成可视图
3. 可视图上运行 Dijkstra，得到严格满足间距的最短路径
4. 输出精确图元（直线 + 圆弧），换向处自动插入切线圆弧（G1 连续）

### 撕毁重布与协商

失败网络识别封路的已布网络，拆除后按外向优先重布，再重布被拆网络；
多轮迭代采用协商式拥塞控制（历史拥挤代价 + 阶段化预算纪律，尾部阶段保底预算）。

## 技术架构

```
主线程 (EDA 宿主)
  ├── 读取 DRC / 焊盘 / 静态障碍
  ├── 逐网络写入图元（差量写入调度：同网合并 + 30ms 限速）
  ├── J/H 引擎委托：autoRouting → 回读 → 后处理 → 回写
  └── 参数弹窗 / 进度弹窗（postMessage 通信）
        │  结构化数据
        ▼
Web Worker
  ├── PlanningEngine（MST / 可视图最短路 / 撕毁重布 / 拉紧圆润）
  ├── VisibilityRouter（可视图 + Dijkstra）
  └── GridRouter（网格 A* 兜底）
```

## 核心模块

| 模块 | 文件 | 功能 |
|------|------|------|
| **EDA 路由宿主** | `src/router/topological-router.ts` | 数据收集、Worker 调度、实时写入/删除、多引擎委托与后处理回写 |
| **规划引擎** | `src/router/planning-engine.ts` | 纯几何规划（零 EDA 依赖）；`postProcessImportedNets` 为三引擎通用的拉紧圆润公开入口 |
| **可视图路由** | `src/router/visibility-router.ts` | 切线图 + Dijkstra（旋转感知状态机） |
| **网格兜底** | `src/router/grid-router.ts` | 八连通 A* + 跨层语义（SMD 限自身层、过孔禁孔圈） |
| **几何原语** | `src/router/rubberband-engine.ts` | 障碍圆群、切线、圆角 |
| **Worker 入口** | `src/worker/planning-worker.ts` | 规划消息协议与进度 |
| **弹窗桥** | `src/ui/dialog-bridge.ts` | 主线程 ↔ iframe 消息 |

## 项目结构

```
eext-rubberband-autorouter/
├── src/
│   ├── router/
│   │   ├── topological-router.ts   # EDA 读写 + Worker 调度 + 多引擎委托
│   │   ├── planning-engine.ts      # 纯几何规划引擎（零 EDA 依赖）
│   │   ├── visibility-router.ts    # 可视图 + Dijkstra
│   │   ├── grid-router.ts          # 网格 A* 兜底
│   │   ├── rubberband-engine.ts    # 切线/障碍圆群/圆角等几何原语
│   │   ├── edge-router.ts          # 焊盘出线几何：出线候选/退避走线/间距判定
│   │   ├── drc-rules.ts            # DRC 规则读取与参数覆盖
│   │   └── worker-source.ts        # Worker 源码内联与创建
│   ├── worker/
│   │   └── planning-worker.ts      # Worker 入口（规划与进度消息）
│   ├── ui/
│   │   └── dialog-bridge.ts        # 主线程 ↔ 内联框架消息桥
│   ├── types/
│   │   ├── index.ts                # 基础类型
│   │   └── router-params.ts        # 用户可调参数与统计
│   ├── utils/                      # 错误处理 / 日志 / 吐司
│   ├── auto-router-facade.ts       # 参数弹窗 / 进度弹窗 / 布线调度
│   └── index.ts                    # 主入口
├── iframe/
│   ├── params.html                 # 参数设置弹窗（含引擎选择）
│   └── progress.html               # 实时进度与日志弹窗
├── build/                          # 打包输出（.eext）
├── extension.json                  # 扩展配置
└── package.json
```

## 构建与调试

```bash
npm install        # 安装依赖
npm run watch      # 开发模式（监听文件变化）
npm run build      # 构建（自动递增补丁版本号）+ 打包
npm run package    # 仅打包 .eext
```

调试流程：

1. 在嘉立创 EDA 中以开发模式安装扩展（从本地文件夹）
2. 修改源代码后 `npm run build`
3. 在 EDA 中重新加载扩展；按 F12 查看引擎日志（`[Router2]` 前缀）

## 依赖

**运行时**：delaunator ^5.0.0（Delaunay 三角剖分）

**开发**：TypeScript ^5、esbuild ^0.19、tsx ^4、jszip ^3.10、fs-extra ^11

## 版本与提交规范

- 语义化版本（SemVer）；`npm run build` 自动递增补丁号
- 提交信息：`feat: 描述` / `fix: 描述` / `docs: 描述` / `refactor: 描述`
- 引擎隔离原则：RRouter 的引擎管线与 J/H 的委托后处理相互独立；
  拉紧/圆润通过 `postProcessImportedNets` 适配三引擎数据

---

# English

**中文** · **English**

This is a PCB auto-routing extension for EasyEDA Pro (嘉立创 EDA 专业版) built on the topological rubberband algorithm: fully local computation, no external service, multiple routing engines with automatic tautening and corner smoothing.

## Features

- ✅ **Local algorithm** — pure JavaScript, design data never leaves your machine
- ✅ **Three routing engines** — RRouter (built-in topological rubberband) / JRouter / HRouter (EasyEDA built-in engines), one-click switch
- ✅ **Topological rubberband** — visibility graph + Dijkstra produces exact tangent line/arc paths that tighten like a pulled string
- ✅ **Automatic post-processing** — built-in engines (J/H) get rubberband tautening + tangent fillets applied by the extension
- ✅ **Live progress** — progress, completion rate, rats-nest completion, stats and logs in real time; stop any time
- ✅ **Route-and-smooth** — RRouter writes each net to the canvas as soon as it is solved, already smoothed
- ✅ **Smart net ordering** — short nets → signals → power → ground; ground always last
- ✅ **Same-layer first** — only change layers when a net truly cannot route on one layer; via placements are DRC-checked against neighboring pads/traces
- ✅ **Partial connectivity preserved** — failed MST branches don't discard the already-routed branches
- ✅ **Rip-up and reroute** — failed nets automatically tear blocking nets and retry, multiple rounds
- ✅ **Rich pad modeling** — TH, SMD, POLYGON, ellipse/rounded-rect (with rotation) pads modeled exactly
- ✅ **Strict DRC discipline** — every primitive is re-checked before commit; a segment that fails clearance is dropped

## Installation

1. Download the latest `rubberband-autorouter_v*.eext` from [Releases](https://github.com/easyeda/eext-rubberband-autorouter/releases)
2. Open EasyEDA Pro → **Extensions** → **Install from local**
3. Select the .eext file — a **Rubberband Router** menu appears

From source (development mode):

```bash
git clone https://github.com/easyeda/eext-rubberband-autorouter.git
cd eext-rubberband-autorouter
npm install
npm run build
```

Then in EasyEDA Pro choose **Extensions → Install from local folder** and pick the project root.

## Quick Start

1. Open a PCB document
2. Menu → **Rubberband Router** → **Start Routing...**
3. The parameter panel docks to the right of the EDA window; adjust and click **Start**
4. Watch live progress, completion rate and rats-nest completion; stop any time
5. Routed nets appear on the canvas immediately as they complete
6. After finishing: **Re-route** (same parameters) or **Routing Settings** (change and run)

## Routing Engines

Pick an engine at the **top** of the parameter panel; hover the "?" icon for descriptions:

| Engine | Source | Best for | Behavior |
|--------|--------|----------|----------|
| **RRouter** (default) | Extension | Typical boards, best trace aesthetics | Topological rubberband engine, routes and smooths live, all parameters available |
| **JRouter** | EasyEDA built-in | Dense connector boards, completion-rate first | Calls the built-in auto-router API, then the extension applies tautening + smoothing |
| **HRouter** | EasyEDA built-in | Speed-first routing | Built-in engine with FASTER optimization → extension post-processing |

- When JRouter/HRouter is selected, RRouter-only parameters (trace width, net ordering, rip-up rounds, live preview, final optimization, via fallback) are hidden — built-in engines use their own rules
- If RRouter completion drops below 85%, the log suggests switching to JRouter/HRouter
- Built-in engine results are written to the canvas first, then tautened/smoothed by the extension, then refreshed

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| Routing engine | RRouter | See [Routing Engines](#routing-engines) |
| Trace width | 10 mil | **RRouter only**. Copper width; also widens obstacle inflation. |
| Extra clearance | 0 mil | Added on top of the DRC minimum; 0 = strict DRC. |
| Escape margin | 0 mil | Extra pad-edge-to-trace-center distance; 0 = escape at minimum clearance. |
| Rip-up rounds | 3 | **RRouter only**. Rounds of tearing blocking nets to retry a failed net; 0 = off. |
| Time budget | 30 min | Shared planning budget for first pass, rip-up, optimization, conflict resolution and via fallback. Engine hard cap: 60 min. |
| Net ordering | Auto | **RRouter only**. Short nets → signals → power → ground (ground last). |
| Live preview | On | **RRouter only**. Write each net to the canvas as soon as it is solved. |
| Final optimization | On | **RRouter only**. After connectivity, re-solve each net; keep only shorter results. |
| Via layer fallback | On | **RRouter only**. Switch layers when a net cannot route on one layer. |
| Routing layers | Top + Bottom | Based on the board's available copper layers; single-sided boards disable bottom automatically. |
| Excluded nets / ground | GND excluded by default | Unchecked nets keep their current state and act as obstacles. Ground nets are excluded by default. |

## FAQ

### Q: The completion rate is low. What can I do?

A: In order: (1) check the log — below 85% RRouter explicitly suggests switching engines; try **JRouter/HRouter**; (2) raise rip-up rounds; (3) keep via layer fallback enabled on double-sided boards; (4) hand-route critical nets (differential pairs, clocks) first.

### Q: Why do traces take detours?

A: Two causes, both addressed: escape direction now follows the pad-center shortest path; congested corridors are re-solved per-net and in groups. Layer changes are a fallback, not a shortcut trade.

### Q: What are the small dots on the canvas? Test points?

A: No. The extension never places test points — the dots are **vias** (layer changes). Vias are always placed outside pads and DRC-checked against neighboring pads and traces.

### Q: Are 4+ layer boards supported?

A: Double-sided (top + bottom) is fully verified. Inner layers are not used yet. Single-sided boards disable the bottom layer automatically.

### Q: How is this different from FreeRouting?

A: Fully local (no external service), pure JavaScript, topological rubberband algorithm with natural tangent-arc aesthetics, and deep EasyEDA integration — reads DRC rules, writes the canvas directly, and can delegate to EasyEDA's built-in engines.

<a id="for-developers"></a>
## For Developers

**Architecture**: the main thread collects DRC rules / pads / static obstacles, schedules a Web Worker (pure geometry, zero EDA dependency), and applies worker output to the canvas through a diff-write scheduler (same-net merge + 30ms throttle). Multi-engine delegation (`topological-router.ts`) calls the `autoRouting` BETA API with an `engine` property, reads the resulting geometry back, post-processes it via `PlanningEngine.postProcessImportedNets` (obstacle registration → per-net tauten + smooth), then purges and rewrites the nets.

**Core modules**: `topological-router.ts` (EDA host, delegation, write-back) · `planning-engine.ts` (pure planning engine; shared tauten/smooth entry for all three engines) · `visibility-router.ts` (tangent graph + Dijkstra with rotation-aware state) · `grid-router.ts` (grid A* fallback with per-layer pad semantics) · `rubberband-engine.ts` (obstacle circle groups, tangents, fillets).

**Pipeline**: pads/traces/fills/board-outline → circle obstacle groups → visibility graph (common tangents + arcs) → Dijkstra → exact line/arc primitives → fillet insertion → per-segment DRC re-check before commit. Failed nets go through negotiated-congestion rip-up and reroute, then grid-A* and via-fallback rescue, all under phase-budget discipline.

**Build**:

```bash
npm install
npm run watch     # dev, watch mode
npm run build     # build + bump patch version + package
npm run package   # package .eext only
```

**Conventions**: SemVer (patch auto-bumped on build); commit prefixes `feat:` / `fix:` / `docs:` / `refactor:`; engine isolation — the RRouter pipeline and the J/H delegate pipeline stay independent, sharing only `postProcessImportedNets` for geometry post-processing.

---

## 许可证 License

Apache-2.0

## 致谢 Acknowledgements

- 原FreeRouting 集成项目 / Original FreeRouting integration project
- Ruby PCB Router 核心算法 / Ruby PCB Router core algorithm
- 嘉立创 EDA 团队 / The EasyEDA team
- Delaunator 库作者 / The Delaunator author

## 相关链接 Links

- [嘉立创 EDA 官网 EasyEDA Official Site](https://lceda.cn/)
- [扩展开发文档 Extension Dev Docs](https://prodocs.lceda.cn/cn/api/guide/)
- [问题反馈 Issues](https://github.com/easyeda/eext-rubberband-autorouter/issues)
- [更新日志 Changelog](./CHANGELOG.md)
