/**
 * 简化的Fibonacci优先队列（使用二叉堆实现）
 */
export class FibonacciQueue {
  constructor(direction = -1, infinity = Infinity) {
    this.direction = direction; // -1 for min, 1 for max
    this.infinity = infinity;
    this.heap = [];
    this.keyMap = new Map();
  }

  /**
   * 生成键的字符串表示（处理循环引用）
   */
  _keyToString(key) {
    if (Array.isArray(key)) {
      return key.map(item => {
        if (item && typeof item === 'object') {
          if (item.vertex) {
            // Region 对象，用 vertex.id 区分
            return `r${item.vertex.id}`;
          }
          if (item.id !== undefined) {
            return `v${item.id}`;
          }
          return String(item);
        }
        return String(item);
      }).join('|');
    }
    return String(key);
  }

  /**
   * 插入或更新键值
   * @param {*} key - 键（可以是任意类型）
   * @param {number} value - 值
   * @returns {boolean} 是否成功插入或更新
   */
  inc(key, value) {
    const keyStr = this._keyToString(key);

    if (this.keyMap.has(keyStr)) {
      const oldValue = this.keyMap.get(keyStr);
      if (this.direction * value < this.direction * oldValue) {
        this.keyMap.set(keyStr, value);
        this._rebuildHeap();
        return true;
      }
      return false;
    }

    this.keyMap.set(keyStr, value);
    this.heap.push({ key, value });
    this._bubbleUp(this.heap.length - 1);
    return true;
  }

  /**
   * 弹出最小/最大元素
   * @returns {Array} [key, value]
   */
  pop() {
    if (this.heap.length === 0) return [null, null];

    const min = this.heap[0];
    const last = this.heap.pop();

    if (this.heap.length > 0) {
      this.heap[0] = last;
      this._bubbleDown(0);
    }

    this.keyMap.delete(this._keyToString(min.key));
    return [min.key, min.value];
  }

  get length() {
    return this.heap.length;
  }

  _bubbleUp(index) {
    while (index > 0) {
      const parentIndex = Math.floor((index - 1) / 2);
      if (this._compare(this.heap[index].value, this.heap[parentIndex].value) >= 0) break;

      [this.heap[index], this.heap[parentIndex]] = [this.heap[parentIndex], this.heap[index]];
      index = parentIndex;
    }
  }

  _bubbleDown(index) {
    while (true) {
      let minIndex = index;
      const leftChild = 2 * index + 1;
      const rightChild = 2 * index + 2;

      if (leftChild < this.heap.length &&
          this._compare(this.heap[leftChild].value, this.heap[minIndex].value) < 0) {
        minIndex = leftChild;
      }

      if (rightChild < this.heap.length &&
          this._compare(this.heap[rightChild].value, this.heap[minIndex].value) < 0) {
        minIndex = rightChild;
      }

      if (minIndex === index) break;

      [this.heap[index], this.heap[minIndex]] = [this.heap[minIndex], this.heap[index]];
      index = minIndex;
    }
  }

  _compare(a, b) {
    return this.direction * (a - b);
  }

  _rebuildHeap() {
    // 重建堆时保持原有的key对象
    const newHeap = [];
    for (const item of this.heap) {
      const keyStr = this._keyToString(item.key);
      if (this.keyMap.has(keyStr)) {
        newHeap.push({ key: item.key, value: this.keyMap.get(keyStr) });
      }
    }
    this.heap = newHeap;

    for (let i = Math.floor(this.heap.length / 2) - 1; i >= 0; i--) {
      this._bubbleDown(i);
    }
  }
}
