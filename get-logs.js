const { spawn } = require('child_process');
const path = require('path');

const mcpPath = path.join(__dirname, '..', 'extension-dev-mcp-tools', 'dist', 'index.js');

const mcp = spawn('node', [mcpPath]);

const request = {
    jsonrpc: '2.0',
    id: 2,
    method: 'tools/call',
    params: {
        name: 'get_console_logs',
        arguments: {
            count: 100
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
}, 10000);
