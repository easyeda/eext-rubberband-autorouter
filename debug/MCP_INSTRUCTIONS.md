# MCP 调试指令

## 请执行以下步骤

### 步骤 1: 导入调试版本插件

使用 MCP 工具 `dev_plugin`:

```json
{
  "tool": "dev_plugin",
  "arguments": {
    "pluginPath": "d:\\\\教程\\\\easyeda-docs\\\\github-eext\\\\eext-rubberband-autorouter\\\\build\\\\dist\\\\rubberband-autorouter_v2.7.3.eext"
  }
}
```

### 步骤 2: 等待插件加载

等待约 2-3 秒

### 步骤 3: 执行调试代码

使用 MCP 工具 `execute_on_eda`:

```json
{
  "tool": "execute_on_eda",
  "arguments": {
    "code": "<复制 mcp-full-debug.js 的完整内容>"
  }
}
```

### 步骤 4: 获取 Console 日志

使用 MCP 工具 `get_console_logs`:

```json
{
  "tool": "get_console_logs",
  "arguments": {
    "filter": ""
  }
}
```

### 步骤 5: 分析结果

期望的输出：

```
=== MCP 调试开始 ===
【1】检查 EDA 环境...
✅ eda 对象可用
【2】检查当前文档...
当前文档：{ documentType: 3, ... }
✅ 当前是 PCB 文档
【3】获取网络列表...
网络数量：X
【4】获取走线...
走线数量：X
【5】运行 DRC 检查...
DRC 结果：❌ 失败
【6】检查走线交叉...
  交叉 1: GND 与 VCC
交叉总数：X
【7】获取 DRC 配置...
默认走线宽度：X mm
【8】总结...
交叉数量：X
DRC 检查：❌ 失败
❌ 检测到走线交叉问题
```

### 步骤 6: 获取绕行日志

使用 MCP 工具 `get_console_logs` 并过滤 `[Bypass]`:

```json
{
  "tool": "get_console_logs",
  "arguments": {
    "filter": "[Bypass]"
  }
}
```

期望的绕行日志：

```
[Bypass] 开始生成绕行路径：...
[Bypass] 找到 X 个阻挡障碍物
  障碍物 1: 线段 (...)
[Bypass] 尝试左侧绕行...
[Bypass] 左侧绕行成功/失败
```

---

## 快速执行脚本

如果你想一次性执行所有步骤，可以使用以下脚本：

```javascript
// 1. 导入插件
await dev_plugin({ pluginPath: "d:\\\\教程\\\\easyeda-docs\\\\github-eext\\\\eext-rubberband-autorouter\\\\build\\\\dist\\\\rubberband-autorouter_v2.7.3.eext" });

// 2. 等待
await sleep(3000);

// 3. 执行调试代码
const debugCode = await readFile("debug/mcp-full-debug.js", "utf-8");
await execute_on_eda({ code: debugCode });

// 4. 获取日志
await sleep(2000);
const logs = await get_console_logs({ filter: "" });
console.log(logs);

// 5. 获取绕行日志
const bypassLogs = await get_console_logs({ filter: "[Bypass]" });
console.log(bypassLogs);
```

---

请执行以上指令并提供日志结果，我将根据日志分析并修复问题。
