@echo off
chcp 65001 >nul
echo ========================================
echo MCP 调试脚本
echo ========================================
echo.

REM 1. 检查桥接服务
echo [1/3] 检查桥接服务...
curl -s http://localhost:49620/health
echo.
echo.

REM 2. 执行调试代码
echo [2/3] 执行调试代码...
echo.

set CODE=(async () => { console.log('=== MCP 调试开始 ==='); if (typeof eda === 'undefined') { console.error('❌ eda 未定义'); return; } console.log('✅ eda 可用'); const doc = eda.dmt_SelectControl.getCurrentDocumentInfo(); console.log('文档:', JSON.stringify(doc)); if (!doc || doc.documentType !== 3) { console.error('❌ 请在 PCB 文档中运行'); return; } console.log('✅ 是 PCB 文档'); const nets = await eda.pcb_Net.getAllNetsName(); console.log('网络数量:', nets ? nets.length : 0); const lines = await eda.pcb_PrimitiveLine.getAll(); console.log('走线数量:', lines ? lines.length : 0); const drc = await eda.pcb_Drc.check(true, true, false); console.log('DRC:', drc ? '✅ 通过' : '❌ 失败'); console.log('=== 调试结束 ==='); })();

curl -X POST http://localhost:49620/execute -H "Content-Type: application/json" -d "{\"code\": \"%CODE%\"}"
echo.
echo.

REM 3. 等待并获取日志
echo [3/3] 等待 3 秒后获取日志...
timeout /t 3 /nobreak >nul
echo.
curl -s http://localhost:49620/logs
echo.
echo.

echo ========================================
echo 调试完成
echo ========================================
pause
