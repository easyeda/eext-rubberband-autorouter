# MCP 调试工作流

## 当前状态

- ✅ Run API Gateway 已连接（端口 49620）
- ✅ MCP 工具可用
- 🔄 需要执行调试代码

## 调试流程

### 1. 导入调试版本插件

```json
{
  "tool": "dev_plugin",
  "arguments": {
    "pluginPath": "d:\\\\教程\\\\easyeda-docs\\\\github-eext\\\\eext-rubberband-autorouter\\\\build\\\\dist\\\\rubberband-autorouter_v2.7.3.eext"
  }
}
```

### 2. 执行自动布线

```json
{
  "tool": "execute_on_eda",
  "arguments": {
    "code": "(async () => { eda.sys_HeaderMenu.clickHeaderMenu('pcb', 'TopologicalRouter', 'AutoRoute'); })();"
  }
}
```

### 3. 等待布线完成（约 5-10 秒）

### 4. 获取 Console 日志

```json
{
  "tool": "get_console_logs",
  "arguments": {
    "filter": "[Bypass]"
  }
}
```

### 5. 执行 DRC 检查

```json
{
  "tool": "execute_on_eda",
  "arguments": {
    "code": "(async () => { const r = await eda.pcb_Drc.check(true,true,false); console.log('DRC:',r?'通过':'失败'); })();"
  }
}
```

### 6. 获取完整日志

```json
{
  "tool": "get_console_logs",
  "arguments": {}
}
```

## 预期日志

```
[Bypass] 开始生成绕行路径：...
[Bypass] 找到 X 个阻挡障碍物
[Bypass] 尝试左侧绕行...
[Bypass] 左侧绕行成功/失败
[Bypass] 尝试右侧绕行...
[Bypass] 右侧绕行成功/失败
```

## 分析要点

1. **绕行是否被调用**: 查找 `[Bypass]` 日志
2. **障碍物检测**: 查看找到的障碍物数量
3. **绕行方向**: 左侧/右侧是否成功
4. **失败原因**: 如果失败，查看错误信息

---

请执行以上流程并提供日志结果。
