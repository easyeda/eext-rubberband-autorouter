const { spawn } = require('child_process');
const path = require('path');

const mcpPath = path.join(__dirname, '..', 'extension-dev-mcp-tools', 'dist', 'index.js');
const pluginPath = path.join(__dirname, 'build', 'dist', 'rubberband-autorouter_v2.10.17.eext');

const mcp = spawn('node', [mcpPath]);

const request = {
    jsonrpc: '2.0',
    id: 3,
    method: 'tools/call',
    params: {
        name: 'dev_plugin',
        arguments: {
            pluginPath: pluginPath,
            timeout: 300
        }
    }
};

mcp.stdout.on('data', (data) => {
    console.log(data.toString());
});

mcp.stderr.on('data', (data) => {
    console.error(data.toString());
});

setTimeout(() => {
    mcp.stdin.write(JSON.stringify(request) + '\n');
}, 2000);

setTimeout(() => {
    mcp.kill();
}, 310000);
