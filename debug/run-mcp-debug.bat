/**
 * MCP 调试执行脚本
 * 通过 curl 执行
 */

// 1. 先检查桥接服务
echo "=== 检查桥接服务 ==="
curl http://localhost:49620/health

// 2. 执行调试代码
echo "\n=== 执行调试代码 ==="
curl -X POST http://localhost:49620/execute ^
  -H "Content-Type: application/json" ^
  -d "{\"code\": \"(async () => { console.log('=== MCP 调试开始 ==='); if (typeof eda === 'undefined') { console.error('eda 未定义'); return; } console.log('eda 可用'); const doc = eda.dmt_SelectControl.getCurrentDocumentInfo(); console.log('文档:', doc); if (!doc || doc.documentType !== 3) { console.error('不是 PCB 文档'); return; } console.log('是 PCB 文档'); const nets = await eda.pcb_Net.getAllNetsName(); console.log('网络:', nets); const lines = await eda.pcb_PrimitiveLine.getAll(); console.log('走线:', lines ? lines.length : 0); const drc = await eda.pcb_Drc.check(true, true, false); console.log('DRC:', drc ? '通过' : '失败'); console.log('=== 调试结束 ==='); })();\"}"

// 3. 获取日志
echo "\n=== 获取日志 ==="
curl -X GET http://localhost:49620/logs
