# MCP 调试指南

## 前提条件

1. **Run API Gateway 已连接**
   - 端口：49620
   - 状态：已连接

2. **EDA 已打开 PCB 文档**

## 调试步骤

### 步骤 1: 安装调试版本

```
build/dist/rubberband-autorouter_v2.7.3.eext
```

### 步骤 2: 通过 MCP 执行调试代码

使用以下 MCP 工具调用：

#### 方法 A: 使用 dev_plugin + get_console_logs

```
1. 调用 dev_plugin 导入插件
   pluginPath: "d:\教程\easyeda-docs\github-eext\eext-rubberband-autorouter\build\dist\rubberband-autorouter_v2.7.3.eext"

2. 运行自动布线（手动）
   菜单栏 → 橡皮筋自动布线 → 开始自动布线

3. 调用 get_console_logs 获取日志
   filter: "[Bypass]"
```

#### 方法 B: 直接执行调试代码

```
调用 execute_on_eda:
code: <mcp-debug-code.js 的内容>
```

### 步骤 3: 分析日志

期望的日志输出：

```
=== MCP 调试开始 ===
【1】检查 EDA 环境...
✅ eda 对象可用
【2】检查当前文档...
当前文档：{ documentType: 3, ... }
✅ 当前是 PCB 文档
【3】获取网络列表...
网络数量：3
【4】获取走线...
走线数量：6
【5】运行 DRC 检查...
DRC 结果：❌ 失败
【6】检查走线交叉...
  交叉 1: GND 与 VCC
交叉总数：1

=== 调试总结 ===
走线数量：6
交叉数量：1
DRC 检查：❌ 失败
❌ 检测到走线交叉问题
```

### 步骤 4: 查看绕行日志

在 Console 中查找 `[Bypass]` 日志：

```
[Bypass] 开始生成绕行路径：(100, 200) -> (500, 600)
[Bypass] 找到 2 个阻挡障碍物
  障碍物 1: 线段 (200,300) -> (400,300)
[Bypass] 尝试左侧绕行...
[Bypass] 左侧绕行失败：路径不安全
[Bypass] 尝试右侧绕行...
[Bypass] 右侧绕行成功
```

## MCP 工具列表

### dev_plugin

导入插件并启用日志监控。

```json
{
  "pluginPath": "d:\\教程\\easyeda-docs\\github-eext\\eext-rubberband-autorouter\\build\\dist\\rubberband-autorouter_v2.7.3.eext"
}
```

### get_console_logs

获取 Console 日志。

```json
{
  "filter": "[Bypass]"
}
```

### execute_on_eda

在 EDA 环境中执行代码。

```json
{
  "code": "<JavaScript code>"
}
```

## 自动化调试脚本

可以创建以下自动化流程：

1. 调用 `dev_plugin` 导入插件
2. 等待插件加载
3. 调用 `execute_on_eda` 运行自动布线
4. 等待布线完成
5. 调用 `execute_on_eda` 运行调试代码
6. 调用 `get_console_logs` 获取日志
7. 分析结果

---

**版本**: v2.7.3  
**日期**: 2026-03-24
