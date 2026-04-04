const { exec } = require('child_process');
const fs = require('fs');
const path = require('path');

const sslDir = path.join(__dirname, 'ssl');

// 创建 ssl 目录
if (!fs.existsSync(sslDir)) {
	fs.mkdirSync(sslDir, { recursive: true });
	console.log(`已创建 ssl 目录：${sslDir}`);
}

// 生成自签名证书的 OpenSSL 命令
const opensslCommand = `openssl req -x509 -newkey rsa:4096 -keyout "${path.join(sslDir, 'key.pem')}" -out "${path.join(sslDir, 'cert.pem')}" -days 365 -nodes -subj "/C=CN/ST=State/L=City/O=Organization/CN=localhost"`;

console.log('正在生成 SSL 证书...');
exec(opensslCommand, (error, stdout, stderr) => {
	if (error) {
		console.error('生成 SSL 证书失败:', error.message);
		console.log('\n请手动安装 OpenSSL 并运行以下命令:');
		console.log(opensslCommand);
		return;
	}
	
	console.log('SSL 证书生成成功！');
	console.log(`私钥文件：${path.join(sslDir, 'key.pem')}`);
	console.log(`证书文件：${path.join(sslDir, 'cert.pem')}`);
	console.log('\n现在可以启动 WSS 服务器了');
});
