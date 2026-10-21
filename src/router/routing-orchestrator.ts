/**
 * RoutingOrchestrator - 布线编排器
 * 负责网络排序（关键网络优先）和失败网络重试队列
 */

export interface PadInfo {
	x: number;
	y: number;
	net: string;
	layer: number;
	radius: number;
	index: number;
}

export interface MSTEdge {
	from: PadInfo;
	to: PadInfo;
	net: string;
}

export interface Network {
	name: string;
	pads: PadInfo[];
}

export interface RouteResult {
	net: string;
	success: boolean;
	edges: MSTEdge[];
}

export class RoutingOrchestrator {
	/**
	 * 按优先级排序网络
	 * 规则：连接数多的网络优先（关键网络），电源网络次之，普通网络最后
	 */
	sortNetworks(nets: Network[]): Network[] {
		return [...nets].sort((a, b) => {
			// 电源/地网络优先
			const isPowerA = this.isPowerNet(a.name);
			const isPowerB = this.isPowerNet(b.name);
			if (isPowerA && !isPowerB) return -1;
			if (!isPowerA && isPowerB) return 1;

			// 连接数多的优先
			return b.pads.length - a.pads.length;
		});
	}

	/**
	 * 判断是否为电源/地网络
	 */
	private isPowerNet(name: string): boolean {
		const upper = name.toUpperCase();
		return upper === 'GND' || upper === 'VCC' || upper === 'VDD' ||
			upper.startsWith('GND') || upper.startsWith('VCC') ||
			upper.startsWith('VDD') || upper.startsWith('PWR') ||
			upper.startsWith('+') || upper.startsWith('-');
	}

	/**
	 * Prim 算法构建最小生成树
	 */
	buildMST(pads: PadInfo[]): MSTEdge[] {
		const n = pads.length;
		if (n < 2) return [];

		const inMST = new Array<boolean>(n).fill(false);
		const minDist = new Array<number>(n).fill(Infinity);
		const minFrom = new Array<number>(n).fill(-1);
		minDist[0] = 0;

		const edges: MSTEdge[] = [];

		for (let iter = 0; iter < n; iter++) {
			let u = -1;
			for (let i = 0; i < n; i++) {
				if (!inMST[i] && (u === -1 || minDist[i] < minDist[u])) u = i;
			}
			inMST[u] = true;

			if (minFrom[u] !== -1) {
				edges.push({
					from: pads[minFrom[u]],
					to: pads[u],
					net: pads[u].net,
				});
			}

			for (let v = 0; v < n; v++) {
				if (inMST[v]) continue;
				const dx = pads[u].x - pads[v].x;
				const dy = pads[u].y - pads[v].y;
				const d = Math.sqrt(dx * dx + dy * dy);
				if (d < minDist[v]) {
					minDist[v] = d;
					minFrom[v] = u;
				}
			}
		}

		return edges;
	}

	/**
	 * 执行带重试的布线调度
	 * 第一轮：按优先级顺序布线所有网络
	 * 第二轮：重试第一轮失败的网络（障碍物更多，可能需要更大绕行）
	 */
	async scheduleWithRetry<T>(
		networks: Network[],
		routeFn: (net: Network, attempt: number) => Promise<T | null>,
		maxRetries: number = 2,
		onProgress?: (done: number, total: number, netName: string) => void
	): Promise<{ results: Map<string, T>; failed: string[] }> {
		const sorted = this.sortNetworks(networks);
		const results = new Map<string, T>();
		let failedNets = sorted.map(n => n.name);

		for (let attempt = 0; attempt < maxRetries; attempt++) {
			const retryList = sorted.filter(n => failedNets.includes(n.name));
			const nextFailed: string[] = [];

			for (let i = 0; i < retryList.length; i++) {
				const net = retryList[i];
				onProgress?.(results.size, sorted.length, net.name);

				try {
					const result = await routeFn(net, attempt);
					if (result !== null) {
						results.set(net.name, result);
					} else {
						nextFailed.push(net.name);
					}
				} catch {
					nextFailed.push(net.name);
				}
			}

			failedNets = nextFailed;
			if (failedNets.length === 0) break;

			if (attempt < maxRetries - 1) {
				console.log(`[Orchestrator] 第 ${attempt + 1} 轮完成，${failedNets.length} 个网络失败，准备重试...`);
			}
		}

		return { results, failed: failedNets };
	}
}
