/**
 * 核心算法单元测试
 * 运行方式：npx tsx test/core-test.ts
 */

// ─── 简单断言工具 ───────────────────────────────────────────────────────────

let passed = 0;
let failed = 0;

function assert(condition: boolean, message: string): void {
	if (condition) {
		console.log(`  ✅ ${message}`);
		passed++;
	} else {
		console.error(`  ❌ ${message}`);
		failed++;
	}
}

function describe(name: string, fn: () => void): void {
	console.log(`\n▶ ${name}`);
	fn();
}

// ─── RoutingOrchestrator 测试 ───────────────────────────────────────────────

import { RoutingOrchestrator } from '../src/router/routing-orchestrator';

describe('RoutingOrchestrator.sortNetworks', () => {
	const orch = new RoutingOrchestrator();

	const makePad = (net: string, i: number) => ({
		x: i * 100, y: 0, net, layer: 1, radius: 20, index: i,
	});

	const nets = [
		{ name: 'SIG1', pads: [makePad('SIG1', 0), makePad('SIG1', 1)] },
		{ name: 'GND',  pads: [makePad('GND', 2), makePad('GND', 3), makePad('GND', 4)] },
		{ name: 'VCC',  pads: [makePad('VCC', 5), makePad('VCC', 6)] },
		{ name: 'SIG2', pads: [makePad('SIG2', 7)] },
	];

	const sorted = orch.sortNetworks(nets);

	assert(sorted[0].name === 'GND' || sorted[0].name === 'VCC',
		'电源/地网络排在最前');
	assert(sorted[sorted.length - 1].name === 'SIG2',
		'单焊盘网络排在最后');
});

describe('RoutingOrchestrator.buildMST', () => {
	const orch = new RoutingOrchestrator();

	const pads = [
		{ x: 0,   y: 0,   net: 'A', layer: 1, radius: 10, index: 0 },
		{ x: 100, y: 0,   net: 'A', layer: 1, radius: 10, index: 1 },
		{ x: 200, y: 0,   net: 'A', layer: 1, radius: 10, index: 2 },
		{ x: 100, y: 100, net: 'A', layer: 1, radius: 10, index: 3 },
	];

	const edges = orch.buildMST(pads);

	assert(edges.length === pads.length - 1,
		`MST 边数 = 焊盘数 - 1 (期望 ${pads.length - 1}, 实际 ${edges.length})`);

	// 验证所有焊盘都被连接（连通性）
	const connected = new Set<number>();
	connected.add(0);
	for (const e of edges) {
		connected.add(e.from.index);
		connected.add(e.to.index);
	}
	assert(connected.size === pads.length, 'MST 连通所有焊盘');
});

// ─── PathOptimizer 测试 ────────────────────────────────────────────────────

import { PathOptimizer } from '../src/router/path-optimizer';

describe('PathOptimizer.countVias', () => {
	const opt = new PathOptimizer();

	const noVia = [
		{ x: 0, y: 0, layer: 1 },
		{ x: 100, y: 0, layer: 1 },
		{ x: 200, y: 0, layer: 1 },
	];
	assert(opt.countVias(noVia) === 0, '无层切换时过孔数为 0');

	const oneVia = [
		{ x: 0, y: 0, layer: 1 },
		{ x: 100, y: 0, layer: 2 },
		{ x: 200, y: 0, layer: 2 },
	];
	assert(opt.countVias(oneVia) === 1, '一次层切换时过孔数为 1');

	const twoVias = [
		{ x: 0, y: 0, layer: 1 },
		{ x: 100, y: 0, layer: 2 },
		{ x: 200, y: 0, layer: 1 },
	];
	assert(opt.countVias(twoVias) === 2, '两次层切换时过孔数为 2');
});

describe('PathOptimizer.optimize - 共线点合并', () => {
	const opt = new PathOptimizer({ enableRubberband: true, enableViaOptimization: false, enableSmoothing: false });

	// 三点共线，中间点应被橡皮筋优化移除
	const collinear = [
		{ x: 0,   y: 0, layer: 1 },
		{ x: 50,  y: 0, layer: 1 },
		{ x: 100, y: 0, layer: 1 },
	];
	const result = opt.optimize(collinear);
	assert(result.length <= collinear.length, '橡皮筋优化不增加点数');
});

// ─── ObstacleBypasser 测试 ─────────────────────────────────────────────────

// ObstacleBypasser 使用内部 interface，通过动态 require 测试
// 由于 TypeScript 编译限制，这里直接测试公开方法

describe('ObstacleBypasser - 直接路径安全时返回直线', () => {
	// 动态导入避免类型问题
	const { ObstacleBypasser } = require('../src/router/obstacle-bypasser');
	const bypasser = new ObstacleBypasser(6, 8);

	const start = { x: 0, y: 0 };
	const end   = { x: 500, y: 0 };
	const obstacles: unknown[] = []; // 无障碍物

	const path = bypasser.generateBypassPath(start, end, obstacles);
	assert(path !== null, '无障碍物时返回路径');
	assert(path?.length === 2, '无障碍物时返回直线（2点）');
});

describe('ObstacleBypasser - 有障碍物时绕行', () => {
	const { ObstacleBypasser } = require('../src/router/obstacle-bypasser');
	const bypasser = new ObstacleBypasser(6, 8);

	const start = { x: 0, y: 0 };
	const end   = { x: 500, y: 0 };
	// 在路径中间放一个大障碍物
	const obstacles = [{ x: 250, y: 0, radius: 80, type: 'pad', net: 'OTHER' }];

	const path = bypasser.generateBypassPath(start, end, obstacles, 'NET_A');
	assert(path !== null, '有障碍物时能找到绕行路径');
	if (path) {
		assert(path.length > 2, '绕行路径包含中间点');
		assert(path[0].x === start.x && path[0].y === start.y, '路径起点正确');
		assert(path[path.length - 1].x === end.x && path[path.length - 1].y === end.y, '路径终点正确');
	}
});

// ─── 结果汇总 ──────────────────────────────────────────────────────────────

console.log(`\n${'─'.repeat(50)}`);
console.log(`测试结果：${passed} 通过，${failed} 失败`);
if (failed > 0) {
	process.exit(1);
}
