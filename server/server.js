"use strict";

const WebSocket = require('ws');
const chokidar = require('chokidar');
const fs = require('fs');
const path = require('path');
const crypto = require('crypto');

// dist 目录路径
const DIST_DIR = path.join(__dirname, '../dist');
const WS_HOST = '127.0.0.1';
// WebSocket 服务器端口（HTTP）
const WS_PORT = 8080;
// WebSocket Secure 服务器端口（HTTPS）
const WSS_PORT = 8443;
// SSL 证书路径（用于 WSS）
const SSL_KEY_PATH = path.join(__dirname, 'ssl', 'key.pem');
const SSL_CERT_PATH = path.join(__dirname, 'ssl', 'cert.pem');

/**
 * @typedef {Object} FileNode
 * @property {string} id - 唯一标识符（文件哈希值）
 * @property {string} name - 文件名/文件夹名，无路径
 * @property {string} path - 完整路径（相对于 dist 目录）
 * @property {number} deep - 层级深度 (0-based)
 * @property {'file' | 'folder'} type - 节点类型
 * @property {string} hash - 文件哈希值
 * @property {boolean} show - 是否显示
 * @property {boolean} expanded - 展开状态
 * @property {'' | 'add' | 'modify' | 'delete'} [status] - 节点状态
 */

// 存储当前的文件树
let currentFileTree = [];
let currentFileTreeHash = '';
// WebSocket 客户端集合，Map 结构：clientId -> ws
const clients = new Map();
// 客户端计数器，用于生成唯一 ID
let clientCounter = 0;

/**
 * 获取格式化的时间字符串（时：分：秒）
 * @returns {string} 格式化的时间，例如 "14:30:45"
 */
function getFormattedTime() {
	const now = new Date();
	const hours = String(now.getHours()).padStart(2, '0');
	const minutes = String(now.getMinutes()).padStart(2, '0');
	const seconds = String(now.getSeconds()).padStart(2, '0');
	return `${hours}:${minutes}:${seconds}`;
}

/**
 * 带时间戳的日志输出
 * @param {...any} args - 要输出的参数
 */
function logWithTime(...args) {
	const timestamp = getFormattedTime();
	console.log(`[${timestamp}]`, ...args);
}

/**
 * 带时间戳的警告输出
 * @param {...any} args - 要输出的参数
 */
function warnWithTime(...args) {
	const timestamp = getFormattedTime();
	console.warn(`[${timestamp}]`, ...args);
}

/**
 * 生成客户端唯一 ID
 * @returns {string} 客户端唯一标识
 */
function generateClientId() {
	clientCounter++;
	const random = Math.random().toString(36).toUpperCase().substring(2, 9);
	return `Client_${clientCounter}_${random}`;
}

/**
 * 计算文件的哈希值（使用 MD5，速度较快）
 * @param {string} filePath - 文件完整路径
 * @returns {string} 文件哈希值
 */
function calculateFileHash(filePath) {
	try {
		const content = fs.readFileSync(filePath);
		return crypto.createHash('md5').update(content).digest('hex');
	} catch (error) {
		console.error(`计算文件哈希失败 ${filePath}:`, error.message);
		return '';
	}
}

/**
 * 计算文件夹的哈希值（基于相对路径的 MD5）
 * @param {string} relativePath - 文件夹相对于 dist 目录的路径
 * @returns {string} 文件夹哈希值
 */
function calculateFolderHash(relativePath) {
    try {
        // 直接使用相对路径 + '/' 进行 MD5 编码
        const pathString = relativePath + '/';
        return crypto.createHash('md5').update(pathString).digest('hex');
    } catch (error) {
        console.error(`计算文件夹哈希失败 ${relativePath}:`, error.message);
        return '';
    }
}

/**
 * @description 文件树排序
 * @param a
 * @param b
 * @returns
 */
const correctFileTreeSort = (a, b) => {
	const pathA = a.path;
	const pathB = b.path;
	const isFolderA = a.type === 'folder';
	const isFolderB = b.type === 'folder';

	// 拆分路径为层级数组
	const partsA = pathA.split('/').filter(Boolean);
	const partsB = pathB.split('/').filter(Boolean);

	// 规则1：所有 非根目录项(deep>0) 优先于 根目录文件(deep=0+file)
	const isRootFileA = a.deep === 0 && a.type === 'file';
	const isRootFileB = b.deep === 0 && b.type === 'file';
	if (isRootFileA && !isRootFileB) return 1;
	if (!isRootFileA && isRootFileB) return -1;

	// 规则2：逐层级对比，父目录优先，同层级文件夹优先
	const minLen = Math.min(partsA.length, partsB.length);
	for (let i = 0; i < minLen; i++) {
		if (partsA[i] !== partsB[i]) {
			return partsA[i].localeCompare(partsB[i]);
		}
	}

	// 规则3：短路径（父目录）排在前
	if (partsA.length !== partsB.length) {
		return partsA.length - partsB.length;
	}

	// 规则4：同层级 文件夹 > 文件
	if (isFolderA && !isFolderB) return -1;
	if (!isFolderA && isFolderB) return 1;

	return 0;
};

/**
 * @description 计算文件列表的哈希值
 * @param file
 * @returns
 */
const calculateListHash = (fileList = []) => {
	// 提取所有文件的 hash 值
	const hashList = fileList.map(file => `${file.hash}:${file.status}`).sort((a, b) => a.localeCompare(b));
	// 将排序后的 hash 列表拼接成字符串并计算 MD5 哈希值
	const listString = hashList.join('|');
	return crypto.createHash('md5').update(listString).digest('hex');
};


/**
 * 递归扫描目录，构建文件树
 * @param {string} dir - 目录路径
 * @param {number} depth - 当前深度
 * @param {string} relativePath - 相对于 dist 的路径
 * @returns {FileNode[]} 文件树节点数组
 */
function scanDirectory(dir, depth = 0, relativePath = '') {
	const nodes = [];

	try {
		const items = fs.readdirSync(dir);
		
		for (const item of items) {
			// 忽略隐藏文件和系统文件
			if (item === 'Thumbs.db') {
				continue;
			}
			
			const fullPath = path.join(dir, item);
			const itemRelativePath = relativePath ? path.posix.join(relativePath, item) : item;
			const stat = fs.statSync(fullPath);
			
			if (stat.isDirectory()) {
				// 文件夹节点
				const children = scanDirectory(fullPath, depth + 1, itemRelativePath);
				const folderHash = calculateFolderHash(itemRelativePath);
				
				nodes.push({
					id: folderHash,
					name: item,
					path: itemRelativePath +'/',
					deep: depth,
					type: 'folder',
					hash: folderHash,
					show: true,
					expanded: true,
					status: ''
				});
				// 添加子节点
				nodes.push(...children);
			} else if (stat.isFile()) {
				// 文件节点
				const fileHash = calculateFileHash(fullPath);
				
				nodes.push({
					id: fileHash,
					name: item,
					path: itemRelativePath,
					deep: depth,
					type: 'file',
					hash: fileHash,
					show: true,
					expanded: true,
					status: ''
				});
			}
		}
	} catch (error) {
		console.error(`扫描目录失败 ${dir}:`, error.message);
	}

	return nodes;
}

/**
 * 构建完整的文件树
 * @returns {FileNode[]} 文件树节点数组
 */
function buildFileTree() {
	if (!fs.existsSync(DIST_DIR)) {
		warnWithTime('dist 目录不存在');
		return [];
	}
	const fileTree = scanDirectory(DIST_DIR);
	
	return fileTree;
}

/**
 * 比较新旧文件树，标记变化状态（基于哈希值对比）
 * @param {FileNode[]} oldTree - 旧文件树
 * @param {FileNode[]} newTree - 新文件树
 * @returns {FileNode[]} 标记了状态的新文件树
 */
function markFileChanges(oldTree, newTree) {
	const oldMap = new Map(oldTree.map(node => [node.path, node]));
	const newMap = new Map(newTree.map(node => [node.path, node]));

	// 标记新增和修改（通过哈希值对比）
	for (const node of newTree) {
		node.status = '';
		const oldNode = oldMap.get(node.path);
		if (!oldNode) {
			// 旧文件中不存在，标记为新增
			node.status = 'add';
		} else if (node.hash !== oldNode.hash) {
			// 哈希值不同，标记为修改
			node.status = 'modify';
			// 可选：记录详细日志
			// console.log(`文件内容变化：${node.path}`);
			// console.log(`  旧哈希：${oldNode.hash}`);
			// console.log(`  新哈希：${node.hash}`);
		}
		// 哈希值相同，说明文件未变化，status 保持为空字符串
	}
	
	// 标记删除（在旧文件中存在，但在新文件中不存在）
	for (const node of oldTree) {
		if (!newMap.has(node.path)) {
			node.status = 'delete';
			newTree.push(node);
		}
	}
	
	return newTree;
}

/**
 * 统计文件树变化数量（新增、修改、删除）
 * @param {FileNode[]} fileTree - 文件树节点数组
 * @returns {{total: number, add: number, modify: number, delete: number}} 文件树节点总数（包含文件和文件夹），以及各类变化数量
 */
function countFileTreeNodes(fileTree) {
	let addCount = 0;
	let modifyCount = 0;
	let deleteCount = 0;
	fileTree.forEach(node => {
		if (node.status === 'add') addCount++;
		else if (node.status === 'modify') modifyCount++;
		else if (node.status === 'delete') deleteCount++;
	});
	
	return {
		'total': addCount + modifyCount + deleteCount,
		addCount,
		modifyCount,
		deleteCount
	}
}

/**
 * 处理客户端发送的消息
 * @param {WebSocket} ws - WebSocket 连接实例
 * @param {string} clientId - 客户端唯一 ID
 * @param {Object} message - 客户端发送的消息对象
 */
function handleClientMessage(ws, clientId, message) {
	if (!message || !message.action) {
		console.warn(`[${clientId}] 收到无效消息:`, message);
		return;
	}
	
	switch (message.action) {
		case 'getUpdatedFileTree':
			handleGetUpdatedFileTreeRequest(ws, clientId, message.fileList);
			break;
		
		case 'updateFile':
			handleUpdateFileRequest(ws, clientId, message.file);
			break;
		
		default:
			warnWithTime(`[${clientId}] 未知消息类型:`, message.action);
	}
}

/**
 * 处理客户端获取更新文件树的请求
 * @param {WebSocket} ws - WebSocket 连接实例
 * @param {string} clientId - 客户端唯一 ID
 * @param {FileNode[]} clientFileList - 客户端传来的文件列表
 */
function handleGetUpdatedFileTreeRequest(ws, clientId, clientFileList) {
	logWithTime(`[${clientId}] 请求获取更新的文件树`);
	
	// 确保客户端传来的是数组
	if (!Array.isArray(clientFileList)) {
		warnWithTime(`[${clientId}] 客户端传来的 fileList 不是数组`);
		clientFileList = [];
	}
	
	// 获取服务端最新的文件树
	const newFileTree = currentFileTree.map(node => ({ ...node }));
	
	// 对比客户端文件列表和服务端最新文件树，标记变化状态
	const updatedFileTree = markFileChanges(clientFileList, newFileTree).sort(correctFileTreeSort);
	
	// 只发送给请求的客户端
	ws.send(JSON.stringify({
		action: 'getUpdatedFileTree_Response',
		fileList: updatedFileTree,
		hash: calculateListHash(updatedFileTree),
	}));
	const { addCount,modifyCount,deleteCount } = countFileTreeNodes(updatedFileTree);
	logWithTime(`[${clientId}] 已发送更新的文件树，共 ${updatedFileTree.length} 个节点 | 新增：${addCount}, 修改：${modifyCount}, 删除：${deleteCount}`);
}

/**
 * 处理客户端更新文件的请求
 * @param {WebSocket} ws - WebSocket 连接实例
 * @param {string} clientId - 客户端唯一 ID
 * @param {{path: string, id: string, hash: string}} file - 客户端传来的文件信息
 */
function handleUpdateFileRequest(ws, clientId, file) {
	logWithTime(`[${clientId}] 请求更新文件：${file?.path || '未知'}`);
	
	// 验证文件信息
	if (!file || !file.path) {
		warnWithTime(`[${clientId}] 无效的文件信息:`, file);
		ws.send(JSON.stringify({
			action: 'updateFile_Response',
			success: false,
			error: '无效的文件信息，缺少 path 字段'
		}));
		return;
	}
	
	// 构建完整的文件路径
	const fullPath = path.join(DIST_DIR, file.path);
	
	// 检查文件是否存在
	if (!fs.existsSync(fullPath)) {
		warnWithTime(`[${clientId}] 文件不存在：${fullPath}`);
		ws.send(JSON.stringify({
			action: 'updateFile_Response',
			success: false,
			error: `文件不存在：${file.path}`,
			path: file.path
		}));
		return;
	}
	
	try {
		// 读取文件内容
		const content = fs.readFileSync(fullPath);
		
		// 计算当前文件的哈希值
		const currentHash = calculateFileHash(fullPath);
		
		// 获取文件统计信息
		const stat = fs.statSync(fullPath);
		logWithTime(`[${clientId}] 成功读取文件：${file.path}[${currentHash}] | 最后修改：${stat.mtime.toISOString()}`);
		
		// 发送文件内容和元数据给客户端
		// 使用 base64 编码传输二进制数据（正确处理 UTF-8）
		ws.send(JSON.stringify({
			action: 'updateFile_Response',
			success: true,
			file: {
				path: file.path,
				id: file.id,
				hash: currentHash,
				size: stat.size,
				mtime: stat.mtime.getTime(),
				content: content.toString('base64') // Node.js 的 toString('base64') 正确处理 UTF-8
			}
		}));
		
		logWithTime(`[${clientId}] 文件已发送：${file.path} (${stat.size} bytes)`);
		
	} catch (error) {
		console.error(`[${clientId}] 读取文件失败：${file.path}`, error.message);
		ws.send(JSON.stringify({
			action: 'updateFile_Response',
			success: false,
			error: `读取文件失败：${error.message}`,
			path: file.path
		}));
	}
}

/**
 * 向所有连接的客户端广播消息
 * @param {Object} message - 要发送的消息对象
 */
function broadcast(message) {
	const messageStr = JSON.stringify(message);
	clients.forEach((ws, clientId) => {
		if (ws.readyState === WebSocket.OPEN) {
			ws.send(messageStr);
		}
	});
}

/**
 * 处理 dist 目录变化
 */
function handleDistChange() {
	logWithTime('检测到 dist 目录变化，正在更新文件列表...');
	
	const newFileTree = buildFileTree();
	currentFileTree = newFileTree.sort(correctFileTreeSort);
	currentFileTreeHash = calculateListHash(newFileTree);

	// 比较新旧文件树，标记变化状态
	const updatedFileTree = markFileChanges(currentFileTree, newFileTree);
	
	const { addCount,modifyCount,deleteCount } = countFileTreeNodes(updatedFileTree);
	logWithTime(`文件列表已更新，共 ${updatedFileTree.length} 个节点 | 新增：${addCount}, 修改：${modifyCount}, 删除：${deleteCount}`);
	
	// 广播更新消息给所有客户端（暂时不使用主动推送）
	// broadcast({
	// 	action: 'update',
	// 	fileList: updatedFileTree
	// });
	
}

/**
 * 初始化并启动 WebSocket 服务器
 * @param {boolean} useSSL - 是否使用 SSL（WSS 协议）
 * @returns {WebSocket.Server} WebSocket 服务器实例
 */
function initWebSocketServer(useSSL = false) {
	let options = { host: WS_HOST, port: useSSL ? WSS_PORT : WS_PORT };
	
	// 如果使用 SSL，加载证书
	if (useSSL && fs.existsSync(SSL_KEY_PATH) && fs.existsSync(SSL_CERT_PATH)) {
		options = {
			...options,
			key: fs.readFileSync(SSL_KEY_PATH),
			cert: fs.readFileSync(SSL_CERT_PATH)
		};
		logWithTime('已加载 SSL 证书，启用 WSS 协议');
	} else if (useSSL) {
		warnWithTime(`未找到 SSL 证书文件，请确保以下文件存在:`);
		warnWithTime(`  - ${SSL_KEY_PATH}`);
		warnWithTime(`  - ${SSL_CERT_PATH}`);
		warnWithTime('将回退到 WS 协议运行');
		useSSL = false;
		options = { host: WS_HOST ,port: WS_PORT };
	}
	
	const protocol = useSSL ? 'wss' : 'ws';
	const port = useSSL ? WSS_PORT : WS_PORT;
	const host = WS_HOST;
	const wss = new WebSocket.Server(options);
	
	wss.on('listening', () => {
		logWithTime(`${protocol.toUpperCase()} 服务器已启动，监听端口：${port}`);
		logWithTime(`客户端连接地址：${protocol}://${host}:${port}`);
	});
	
	wss.on('connection', (ws) => {
		// 为客户端生成唯一 ID
		const clientId = generateClientId();
		clients.set(clientId, ws);
		
		logWithTime(`[${clientId}] 客户端已连接`);
		logWithTime(`当前在线客户端数：${clients.size}`);
		
		// 注意：连接成功后不主动发送文件列表，等待客户端主动请求
		
		ws.on('message', (data) => {
			try {
				const message = JSON.parse(data.toString());
				handleClientMessage(ws, clientId, message);
			} catch (error) {
				console.error(`[${clientId}] 消息解析失败:`, error.message);
			}
		});
		
		ws.on('close', () => {
			logWithTime(`[${clientId}] 客户端已断开连接`);
			clients.delete(clientId);
			logWithTime(`当前在线客户端数：${clients.size}`);
		});
		
		ws.on('error', (error) => {
			console.error(`[${clientId}] WebSocket 错误:`, error.message);
		});
	});
	
	wss.on('error', (error) => {
		console.error('WebSocket 服务器错误:', error.message);
	});
}

/**
 * 初始化目录监控
 */
function initWatcher() {
	// 使用 chokidar 监控 dist 目录
	const watcher = chokidar.watch(DIST_DIR, {
		ignored: /(^|[\\\/])(\.git|\.vscode|node_modules)([\\\/]|$)/, // 忽略特定目录
		persistent: true,
		ignoreInitial: true, // 忽略初始添加事件
		awaitWriteFinish: {
			stabilityThreshold: 1000, // 等待文件写入完成的稳定时间
			pollInterval: 500 // 轮询间隔
		}
	});
	
	// 防抖处理
	let debounceTimer = null;
	const debouncedHandle = () => {
		if (debounceTimer) {
			clearTimeout(debounceTimer);
		}
		debounceTimer = setTimeout(handleDistChange, 2000);
	};
	
	watcher
		.on('add', debouncedHandle)
		.on('change', debouncedHandle)
		.on('unlink', debouncedHandle)
		.on('addDir', debouncedHandle)
		.on('unlinkDir', debouncedHandle)
		.on('error', (error) => {
			console.error('文件监控错误:', error.message);
		});
	
	logWithTime(`开始监控 dist 目录：${DIST_DIR}`);
	
	return watcher;
}

/**
 * 主函数
 */
async function main() {
	console.log('='.repeat(50));
	console.log('嘉立创EDA扩展开发助手服务端');
	console.log('文件监控 WebSocket 服务器启动中...')
	console.log('='.repeat(50));
	
	// 检查 dist 目录是否存在
	if (!fs.existsSync(DIST_DIR)) {
		logWithTime('创建 dist 目录...');
		fs.mkdirSync(DIST_DIR, { recursive: true });
	}
	
	// 初次构建文件树
	logWithTime('初次扫描 dist 目录...');
	currentFileTree = buildFileTree().sort(correctFileTreeSort);
	logWithTime(`初始文件树包含 ${currentFileTree.length} 个节点`);
	// 初始化 WebSocket 服务器（支持 WSS）
	// 优先尝试启动 WSS，如果 SSL 证书不存在则回退到 WS
	// initWebSocketServer(true);
	
	// 同时启动 WS 服务器以兼容非加密连接
	initWebSocketServer(false);
	
	// 初始化文件监控
	const watcher = initWatcher();
	
	// 优雅关闭
	process.on('SIGINT', () => {
		console.log('\n正在关闭服务器...');
		watcher.close();
		
		clients.forEach((ws, clientId) => {
			console.log(`[${clientId}] 强制断开连接`);
			ws.close();
		});
		
		logWithTime('所有客户端已断开');
		
		setTimeout(() => {
			logWithTime('服务器已完全关闭');
			process.exit(0);
		}, 500);
	});
}

// 启动服务
main().catch(error => {
	console.error('启动失败:', error);
	process.exit(1);
});
