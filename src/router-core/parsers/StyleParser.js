/**
 * 样式CSV解析器
 */
export class StyleParser {
  constructor() {
    this.styles = new Map();
  }

  /**
   * 解析CSV文件内容
   * @param {string} content - CSV文件内容
   * @returns {Map} 样式映射表
   */
  parse(content) {
    const lines = content.split('\n').filter(line => line.trim());

    if (lines.length === 0) return this.styles;

    // 解析表头
    const headers = lines[0].split(',').map(h => h.trim());

    // 解析数据行
    for (let i = 1; i < lines.length; i++) {
      const values = lines[i].split(',').map(v => v.trim());

      if (values.length < headers.length) continue;

      const style = {};
      headers.forEach((header, index) => {
        style[header] = values[index];
      });

      if (style.name) {
        this.styles.set(style.name, style);
      }
    }

    return this.styles;
  }

  /**
   * 获取指定样式
   * @param {string} name - 样式名称
   * @returns {Object|null} 样式对象
   */
  getStyle(name) {
    return this.styles.get(name) || null;
  }

  /**
   * 获取所有样式名称
   * @returns {Array} 样式名称数组
   */
  getStyleNames() {
    return Array.from(this.styles.keys());
  }
}
