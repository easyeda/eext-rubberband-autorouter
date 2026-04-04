@echo off
chcp 65001 >nul
echo ==================================================
echo WebSocket 文件监控服务 - 快速启动
echo ==================================================
echo.

:: 检查 Node.js
where node >nul 2>nul
if %errorlevel% neq 0 (
    echo [错误] 未检测到 Node.js，请先安装 Node.js
    pause
    exit /b 1
)

echo [信息] Node.js 版本:
node --version
echo.

:: 检查依赖
echo [信息] 检查依赖包...
if not exist "node_modules\ws" (
    echo [警告] 缺少 ws 依赖，正在安装...
    call npm install
) else (
    echo [信息] 依赖包已就绪
)
echo.

:: 检查 SSL 证书
if not exist "ssl\key.pem" (
    echo [警告] 未找到 SSL 证书，正在生成...
    call npm run generate-ssl
    timeout /t 3 /nobreak >nul
) else (
    echo [信息] SSL 证书已存在
)
echo.

:: 启动服务器
echo ==================================================
echo 正在启动 WebSocket 服务器...
echo ==================================================
echo.
echo 服务地址:
echo   - WSS: wss://localhost:8443
echo   - WS:  ws://localhost:8080
echo.
echo 按 Ctrl+C 可停止服务器
echo ==================================================
echo.

node server.js

pause
