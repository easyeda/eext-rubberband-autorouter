# 橡皮筋布线器（eext-rubberband-autorouter）PRD

> 嘉立创EDA专业版扩展 · 拓扑橡皮筋自动布线
> 本文档覆盖当前全部功能点与近期新增的需求/策略，供后续维护与 AI 协作开发对齐口径。
> 更新：2026-10-23（v0.0.82）

## 1. 产品定位

在嘉立创EDA专业版内一键完成 PCB 自动布线：基于可视图 Dijkstra 的橡皮筋算法，
输出「中心到中心最短路 + 绕障圆弧拉紧」的走线，目标布通率 ≥95%、DRC 零违规、
拐角 G1 连续（切线圆角过渡）。

## 2. 布线引擎（src/router/planning-engine.ts）

### 2.1 规划主流程（plan）
1. **网络排序**（sortNetworks，auto 四层）：短小两焊盘连接 → 一般信号 → 电源 → 地（永远最后，防围墙）。
2. **第一遍·线段级快扫**（quick sweep）：每条 MST 边只试最便宜的单层直连，失败立即放下一条边——
   全网「一布就通」的边最先全部落位，不追求单网完整再下一网（2026-10-23 策略，实测 F407 隔离板 8/11 → 9/11）。
3. **撕毁重布轮**（ripupRounds，默认 5）：
   - 无封口的遗留网每轮补一次完整链尝试（快扫没给过昂贵退避）；
   - 有封口的网走「拆墙重布」：快照被撕网原走线 → 重布失败网 → 逆序重布被撕网 →
     **「已救回」判定以 routedOrder 为准**（残留半成品不算救回，杜绝净损失，2026-10-23 修复）→
     全部救回才接受，否则整体回滚。
4. **成组重布**：以失败网「焊盘中心连线」为走廊，拆除走廊上其它网络（最多 8 个），
   失败网按内向优先（maxX 升序）重布，被拆网重布失败先试两焊盘底层兜底，
   「已救回」口径同上，不满足则整体回滚。
5. **最终优化**（optimize）：逐网重布取更短，显著更短（≥2% 且 ≥8mil）且不新增过孔才采纳。
6. **冲突消除**（resolveContention）：绕路倍率最高的网先「独走最优」评估，组内两种排序都试，
   接受条件从严（总长显著缩短 + 过孔不增加）。
7. **过孔换层兜底**：部分网逐边重试 → 零图元网走 `tryViaFallback`（首尾焊盘出线 + 底层最短路），
   过孔落点被单一网络独占封死时走 `tryViaSlotEviction`（guarded：全局限 6 次、预算余量 <120s 停用、
   解锁 <2 槽不拆、>8 盘不拆）。

### 2.2 单边求解链（routeNetEdges，full 模式）
- 顶层直连（两级走廊 150/500mil 一次求解）→ 焊盘边界出线候选（routePadEdge + 切线圆角）；
- 顶层绕路 >1.55×下界视为「绕不过」，与换层方案比长度后择优（换层须省 50mil 以上）；
- 顶层 SMD↔底层 SMD 走「焊盘外过孔」路径（默认禁焊盘打孔，allowViaInPad 显式开启才允许）；
- 两层都走不通才用过孔换层候选（viaEdgeCandidate：两端多落点 × 底层最短路，估价值升序试 8 对）。

### 2.3 并排焊盘出线预留（2026-10-23 新增）
- 等间距、等大小的共线焊盘排（连接器/IC 引脚排/排阻）由 `prioritizeRowInteriorPads` 检测，
  记录每焊盘的行几何（排向/中位间距/排内深度），并按「中间 → 两端」重排布线顺序；
- 排中部焊盘（depth>0.4）若直连出线方向**贴着排向滑出**（会封死侧边焊盘唯一垂直出口），
  改走 `rowEscapeRoute`：沿排法向冲出引脚区（长度 ≈ 焊盘包络 + 间距余量）再交最短路；
- 冲出段与路径的拐点做 G1 切线圆角（fanoutFillet，R ≤ 线宽×3）；出线本就垂直的直连保持原样。

### 2.4 完整性/合法性保障
- **无网络焊盘 DRC 避让**：`collectPads` 补齐 NC 脚/安装孔/屏蔽框脚等空网焊盘（net=''），
  只进 allPads 当障碍不进 padMap；走线与过孔照查间距（实测 0mil 踩线 17→0）。
- **POLYGON 焊盘归一化**：顶点数组 → 质心回零 → 最长边主轴 → 轴对齐包围盒 + 旋转，
  padShapeToCircles/padShapeDistance/padExitModel 三处统一（修复 LQFP 焊盘几乎不设障）。
- **板框层甄别**（triageBoardObstacles）：拼版 V-CUT/网格填充线会把板面切成不连通条带，
  只保留距整体包围盒边缘 ≤60mil 的外轮廓圈，内部圈按非电气障碍忽略。
- **TH 判定只认 layer 12**：层 1/2 SMD 带封装库假孔（如 LQFP hole=8.5mil）不误判通孔。
- **单面板支持**：铜层数按 `getAllLayers() + layerStatus===1` 探测（停用内层过滤），
  单面板禁用底层走线与过孔换层。
- **快照回滚不净损失**：所有拆网动作（撕毁/成组/驱逐/冲突消除）先快照原走线与过孔，
  失败即恢复并重新注册过孔障碍与 routedEdges。

## 3. 性能（visibility-router.ts / planning-engine.ts）

- **圆对距离截断**（2026-10-23）：x 序扫描，相距 > r1+r2+800mil 的圆对不建公切线
  （实测公切线构建占单次求解 70~90%，Dijkstra≈0；同时消除横穿空地的长切线大斜线）。
- **过孔落点空间索引**：viaPadClear 走 64mil 焊盘网格（3×3 邻域）；过孔槽位扫描只查
  焊盘 R+120mil 邻域障碍子集（密集板从「万级网格点 × 全量障碍」降两个数量级）。
- **协作式让出**：唯一可用原语是单例 MessageChannel（scheduler.yield 在该 Electron 版本为
  假让出；rAF 被节流）；配对循环/弧边循环/Dijkstra 每 16ms 让出 + 单次求解预算 3s。
- **停止布线即时生效**（2026-10-23）：取消探针贯穿 routeConnection/routeBetweenEndpoints/
  routePadEdge 全链路，在全部让出点轮询（响应 10~30s → ~50ms）。
- **预算纪律**：规划截止时间贯穿快扫/撕毁轮/兜底/冲突消除/驱逐（轮头与逐网检查），
  防止收尾阶段顶穿预算。

## 4. UI / 交互（iframe/ + src/ui/）

- **参数弹窗**（params.html）：布线设置双栏、网络顺序策略、DRC 参数（线宽/间距）覆盖、
  排除网络、ripupRounds（默认 5）。
- **进度弹窗**（progress.html）：实时布通率（飞线口径 = 焊盘级 MST 完成度）、逐网日志、
  「关闭窗口」按钮**全程常显**（关窗不取消布线，2026-10-23 需求）、停止布线即时生效。
- **Toast**：枚举口径 `ESYS_ToastMessageType.WARNING='warn'`（'warning' 是错值，背景色会退化）。
- **弹窗定位**：openIFrame props 传 x/y（右侧贴边 + getViewportSize 视口居中）；
  已知 EDA 4.1.84 API bug：props.x/y 被客户端忽略（等修复，代码照常传，修复后自动生效）。

## 5. 部署与构建

- 构建：`npm run build`（bump-version → esbuild → package）→ `build/dist/*.eext` → EDA 手动导入。
- `.edaignore` 控制包内容（src/build/config/case/tmp/server/debug/scripts 等开发文件全部排除）。

## 6. 离线测试工具链（scripts/，2026-10-23 新增）

- **批量导出** `node scripts/batch-dump-all.mjs [--force] [--filter]`：
  遍历 case/ 全部工程 → EDA CLI headless 采集 PlanningInput（复用生产源码构建的
  `eda-dump-bundle.mjs`，零逻辑漂移）→ `tmp/dumps/<case>-dump.json`。
  `STRUCT_VERSION` 版本缓存：PlanningInput 结构变化时 +1，自动识别过期并重导。
- **批量布线** `node scripts/batch-route-all.mjs [--budget 毫秒] [--filter] [--limit] [--redump]`：
  纯 Node 跑引擎（不启动 EDA），每工程输出两个文件：
  - `digest.json`（**AI 只读这个**，<5KB）：布通率/失败网/部分网/DRC top10/接缝缺陷；
  - `result.json`（全量图元，仅供脚本/写回，禁止整读进 AI 上下文）。
  内置离线审计：间距（走线↔走线/焊盘/过孔，网格采样圆近似）+ 接缝（开口/锐角折角/近整圆弧）。
  汇总 `tmp/results/_summary.json` 按布通率升序。
- **专项诊断**：tmp/edge-debug.mts（失败网隔离 + VR 分相计时）、tmp/audit-joints.mjs（接缝审计）、
  tmp/case-repro.mjs（全板离线复现）、tmp/kill-leaks.ps1（清理后台计算，只匹配计算特征——
  路径宽匹配会误杀 EDA 文档服务，已记事故教训）。
- **已知坑**（详见项目记忆）：EDA 弧角度约定与数学正方向一致；沙箱无 window/document；
  后台 setTimeout 节流 ~15ms 需按墙钟补齐；headless 文档加载依赖客户端工程服务状态。

## 7. 质量基线（2026-10-23）

| 板 | 布通 | 说明 |
|---|---|---|
| F407 隔离板（11 网子集） | **9/11** | 历史天花板 8；失败网 = RESET(5/6)、USB_D+(1/2) |
| F407 全板（156 网） | 81 → 94/156 | 线段级快扫 +13；圆对截断后待批量库复测 |
| 电流表（48 网） | 30/48 | netless 0mil 违规清零；合法性优先策略 |

目标：≥95% 布通 + DRC 0 违规（布通率与合法性是两个独立硬指标，round-2 实测二者互相拉扯，
策略上**先合法后布通**）。
