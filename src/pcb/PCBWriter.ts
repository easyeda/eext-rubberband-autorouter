/**
 * PCBWriter - EDA API 写入解耦层
 * 集中管理所有写入操作，记录写入数量，失败时提示用户撤销
 */

export interface LineData {
	net: string;
	layer: number;
	x1: number;
	y1: number;
	x2: number;
	y2: number;
	width: number;
}

export interface ViaData {
	net: string;
	x: number;
	y: number;
	diameter: number;
	holeDiameter: number;
}

export interface WriteStats {
	linesWritten: number;
	linesFailed: number;
	viasWritten: number;
	viasFailed: number;
}

export class PCBWriter {
	private linesWritten = 0;
	private linesFailed = 0;
	private viasWritten = 0;
	private viasFailed = 0;

	/**
	 * 写入单条线段
	 */
	async writeLine(line: LineData): Promise<void> {
		await eda.pcb_PrimitiveLine.create(
			line.net,
			line.layer,
			line.x1, line.y1,
			line.x2, line.y2,
			line.width,
			false
		);
	}

	/**
	 * 放置过孔
	 */
	async writeVia(via: ViaData): Promise<void> {
		await eda.pcb_PrimitiveVia.create(
			via.net,
			via.x, via.y,
			via.holeDiameter,
			via.diameter
		);
	}

	/**
	 * 批量写入线段，失败时跳过并记录
	 */
	async writeLines(lines: LineData[]): Promise<{ written: number; failed: number }> {
		let written = 0;
		let failed = 0;
		for (const line of lines) {
			try {
				await this.writeLine(line);
				written++;
				this.linesWritten++;
			} catch (err) {
				console.warn('[PCBWriter] 写入线段失败:', err);
				failed++;
				this.linesFailed++;
			}
		}
		return { written, failed };
	}

	/**
	 * 批量写入过孔（自动去重），失败时跳过并记录
	 */
	async writeVias(vias: ViaData[]): Promise<{ written: number; failed: number }> {
		// 按位置去重
		const unique = new Map<string, ViaData>();
		for (const via of vias) {
			const key = `${via.x.toFixed(2)}_${via.y.toFixed(2)}_${via.net}`;
			if (!unique.has(key)) unique.set(key, via);
		}

		let written = 0;
		let failed = 0;
		for (const via of unique.values()) {
			try {
				await this.writeVia(via);
				written++;
				this.viasWritten++;
			} catch (err) {
				console.warn('[PCBWriter] 写入过孔失败:', err);
				failed++;
				this.viasFailed++;
			}
		}
		return { written, failed };
	}

	/**
	 * 获取写入统计
	 */
	getStats(): WriteStats {
		return {
			linesWritten: this.linesWritten,
			linesFailed: this.linesFailed,
			viasWritten: this.viasWritten,
			viasFailed: this.viasFailed,
		};
	}

	/**
	 * 获取已写入的元素总数（用于判断是否需要提示用户撤销）
	 */
	getTotalWritten(): number {
		return this.linesWritten + this.viasWritten;
	}

	/**
	 * 重置计数器
	 */
	reset(): void {
		this.linesWritten = 0;
		this.linesFailed = 0;
		this.viasWritten = 0;
		this.viasFailed = 0;
	}
}
