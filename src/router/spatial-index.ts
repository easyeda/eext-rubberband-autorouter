/**
 * spatial-index — 均匀网格桶空间索引（纯加速，不改变任何判定结果）
 *
 * 约定：条目 = 轴对齐包围盒 [minx,miny,maxx,maxy] + 可选层号。query 返回的
 * 恰好是「包围盒与查询盒相交（闭区间）且层兼容」的全部条目 id，等价于对全部
 * 条目做一遍线性扫描 + 同一个盒相交判定。调用方在候选上再跑原来的精确谓词，
 * 因此只要「谓词为真 ⇒ 盒相交」成立（包围盒是真实几何的上界），索引前后结果完全一致。
 *
 * 层语义：条目 layer === undefined 表示所有层；查询 layer === undefined 表示不限层。
 * 非有限包围盒（NaN/Infinity）的条目视为匹配一切的盒，进入 always 列表，任何查询都会返回
 * （保守：交给精确谓词裁决，与线性扫描等价）。
 */

/** 网格单元边长（mil）：走线障碍圆半径 ~10-20，焊盘圆更大，40 是实测折中 */
export const DEFAULT_CELL = 40;

/** 单条目最多占用的格数；更大的条目走 always 列表 */
const MAX_CELLS_PER_ITEM = 400;
const KEY_OFF = 1 << 15;
const KEY_MUL = 1 << 16;

function cellKey(ix: number, iy: number): number {
	return (ix + KEY_OFF) * KEY_MUL + (iy + KEY_OFF);
}

export class SpatialGrid {
	private readonly cell: number;
	private readonly cells = new Map<number, number[]>();
	/** id → [minx,miny,maxx,maxy,layer(-1=all)] */
	private readonly boxes = new Map<number, number[]>();
	/** 不入格的条目（非有限 / 超大）：每次查询逐个按盒判定 */
	private readonly always = new Set<number>();
	/** 查询去重戳（id 按非负整数使用） */
	private stamps: Int32Array = new Int32Array(64);
	private stampNo = 0;

	constructor(cell = DEFAULT_CELL) {
		this.cell = cell > 0 ? cell : DEFAULT_CELL;
	}

	get size(): number { return this.boxes.size; }

	has(id: number): boolean { return this.boxes.has(id); }

	clear(): void {
		this.cells.clear();
		this.boxes.clear();
		this.always.clear();
	}

	private range(min: number, max: number): [number, number] | null {
		const a = Math.floor(min / this.cell), b = Math.floor(max / this.cell);
		// 超出键空间（±32768 格）的条目退化为 always，避免键冲突
		if (a < -KEY_OFF + 1 || b > KEY_OFF - 2) return null;
		return [a, b];
	}

	/** 插入条目；同 id 已存在则先移除（等价于替换） */
	insert(id: number, minx: number, miny: number, maxx: number, maxy: number, layer?: number): void {
		if (this.boxes.has(id)) this.remove(id);
		// 任一坐标非有限 → 视为「匹配一切」的盒（保守：交给调用方的精确谓词裁决）
		if (!(Number.isFinite(minx) && Number.isFinite(miny) && Number.isFinite(maxx) && Number.isFinite(maxy))) {
			minx = miny = -Infinity;
			maxx = maxy = Infinity;
		}
		this.boxes.set(id, [minx, miny, maxx, maxy, layer === undefined ? -1 : layer]);
		const rx = Number.isFinite(minx) && Number.isFinite(maxx) ? this.range(minx, maxx) : null;
		const ry = Number.isFinite(miny) && Number.isFinite(maxy) ? this.range(miny, maxy) : null;
		// 非有限 / 超出键空间 / 横跨过多格的条目进 always 列表：不占格，每次查询都当候选，
		// 仍按其真实盒做相交判定（非有限盒的比较恒假 → 视为相交，保守）
		if (!rx || !ry || (rx[1] - rx[0] + 1) * (ry[1] - ry[0] + 1) > MAX_CELLS_PER_ITEM) {
			this.always.add(id);
			return;
		}
		for (let ix = rx[0]; ix <= rx[1]; ix++) {
			for (let iy = ry[0]; iy <= ry[1]; iy++) {
				const k = cellKey(ix, iy);
				const b = this.cells.get(k);
				if (b) b.push(id); else this.cells.set(k, [id]);
			}
		}
	}

	remove(id: number): boolean {
		const box = this.boxes.get(id);
		if (!box) return false;
		this.boxes.delete(id);
		if (this.always.delete(id)) return true;
		const rx = this.range(box[0], box[2])!, ry = this.range(box[1], box[3])!;
		for (let ix = rx[0]; ix <= rx[1]; ix++) {
			for (let iy = ry[0]; iy <= ry[1]; iy++) {
				const k = cellKey(ix, iy);
				const b = this.cells.get(k);
				if (!b) continue;
				const p = b.indexOf(id);
				if (p >= 0) {
					b[p] = b[b.length - 1];
					b.pop();
				}
				if (b.length === 0) this.cells.delete(k);
			}
		}
		return true;
	}

	/**
	 * 返回包围盒与 [minx,maxx]×[miny,maxy] 相交（闭区间）且层兼容的条目 id。
	 * 返回顺序未定义；需要原始顺序时用 querySorted。
	 */
	query(minx: number, miny: number, maxx: number, maxy: number, layer?: number, out: number[] = []): number[] {
		if (this.always.size) {
			for (const id of this.always) {
				const bx = this.boxes.get(id)!;
				if (bx[2] < minx || bx[0] > maxx || bx[3] < miny || bx[1] > maxy) continue;
				if (layer !== undefined && bx[4] !== -1 && bx[4] !== layer) continue;
				out.push(id);
			}
		}
		if (!(minx <= maxx) || !(miny <= maxy)) {
			// 非有限/反向查询盒：无法按格检索 → 全表回退（仍精确）
			if (!Number.isFinite(minx) || !Number.isFinite(maxx) || !Number.isFinite(miny) || !Number.isFinite(maxy)) {
				return this.queryAll(minx, miny, maxx, maxy, layer, out);
			}
			return out;
		}
		const rx = this.range(minx, maxx), ry = this.range(miny, maxy);
		// 查询盒覆盖的格数多于现有格数（超大查询）→ 直接扫全部条目，避免遍历海量空格
		if (!rx || !ry || (rx[1] - rx[0] + 1) * (ry[1] - ry[0] + 1) > this.cells.size + 64) {
			return this.queryAll(minx, miny, maxx, maxy, layer, out);
		}
		const sn = ++this.stampNo;
		for (let ix = rx[0]; ix <= rx[1]; ix++) {
			for (let iy = ry[0]; iy <= ry[1]; iy++) {
				const b = this.cells.get(cellKey(ix, iy));
				if (!b) continue;
				for (let i = 0; i < b.length; i++) {
					const id = b[i];
					if (id >= this.stamps.length) {
						const ns = new Int32Array(Math.max(id + 1, this.stamps.length * 2));
						ns.set(this.stamps);
						this.stamps = ns;
					}
					if (this.stamps[id] === sn) continue;
					this.stamps[id] = sn;
					const bx = this.boxes.get(id)!;
					if (bx[2] < minx || bx[0] > maxx || bx[3] < miny || bx[1] > maxy) continue;
					if (layer !== undefined && bx[4] !== -1 && bx[4] !== layer) continue;
					out.push(id);
				}
			}
		}
		return out;
	}

	/** 同 query，但按 id 升序返回（用于「首个违规者」依赖遍历顺序的场合） */
	querySorted(minx: number, miny: number, maxx: number, maxy: number, layer?: number): number[] {
		const r = this.query(minx, miny, maxx, maxy, layer);
		r.sort((a, b) => a - b);
		return r;
	}

	/** 全表回退（查询盒超出键空间时） */
	private queryAll(minx: number, miny: number, maxx: number, maxy: number, layer: number | undefined, out: number[]): number[] {
		const seen = new Set(out);
		for (const [id, bx] of this.boxes) {
			if (seen.has(id)) continue;
			if (bx[2] < minx || bx[0] > maxx || bx[3] < miny || bx[1] > maxy) continue;
			if (layer !== undefined && bx[4] !== -1 && bx[4] !== layer) continue;
			out.push(id);
		}
		return out;
	}
}

/** 带坐标/半径的圆（RBObstacle 的子集） */
export interface IndexedCircle {
	x: number;
	y: number;
	r: number;
	kind?: string;
	x1?: number; y1?: number; x2?: number; y2?: number;
	layer?: number;
}

/** 障碍包围盒（圆：x±r；胶囊：两端点包络 ± r；半径取 |r|+0.05，覆盖各调用点的 r-0.02 等微调） */
export function obstacleBox(o: IndexedCircle): [number, number, number, number] {
	const r = Math.abs(o.r) + 0.05;
	if (o.kind === 'capsule') {
		const x1 = o.x1 ?? o.x, y1 = o.y1 ?? o.y;
		const x2 = o.x2 ?? x1, y2 = o.y2 ?? y1;
		return [
			Math.min(o.x, x1, x2) - r, Math.min(o.y, y1, y2) - r,
			Math.max(o.x, x1, x2) + r, Math.max(o.y, y1, y2) + r,
		];
	}
	return [o.x - r, o.y - r, o.x + r, o.y + r];
}

/**
 * 障碍数组的空间索引：条目 id = 数组下标。
 * 注意：索引是**数组的快照**——数组内容（长度/元素）变化后必须重建。
 * 用 ObstacleIndex.of(arr) 取缓存时会校验 长度；调用方保证快照数组不被原地改写。
 */
export class ObstacleIndex<T extends IndexedCircle> {
	readonly grid: SpatialGrid;
	readonly list: T[];
	readonly length: number;

	constructor(list: T[], cell = DEFAULT_CELL) {
		this.list = list;
		this.length = list.length;
		this.grid = new SpatialGrid(cell);
		for (let i = 0; i < list.length; i++) {
			const o = list[i];
			if (!o) continue;
			const b = obstacleBox(o);
			this.grid.insert(i, b[0], b[1], b[2], b[3]);
		}
	}

	/** 候选障碍（盒相交，层不过滤）；顺序未定义 */
	near(minx: number, miny: number, maxx: number, maxy: number): T[] {
		const ids = this.grid.query(minx, miny, maxx, maxy);
		const out: T[] = new Array(ids.length);
		for (let i = 0; i < ids.length; i++) out[i] = this.list[ids[i]];
		return out;
	}

	private static cache = new WeakMap<object, ObstacleIndex<any>>();

	/** 取（或建）数组的缓存索引；长度变化即重建 */
	static of<U extends IndexedCircle>(list: U[]): ObstacleIndex<U> {
		const hit = ObstacleIndex.cache.get(list) as ObstacleIndex<U> | undefined;
		if (hit && hit.length === list.length) return hit;
		const idx = new ObstacleIndex<U>(list);
		ObstacleIndex.cache.set(list, idx);
		return idx;
	}
}
