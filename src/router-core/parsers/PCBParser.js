/**
 * PCB文件解析器（简化版）
 */
export class PCBParser {
  constructor() {
    this.elements = [];
    this.netlists = [];
    this.fileVersion = '';
    this.pcbInfo = {};
  }

  /**
   * 解析PCB文件内容
   * @param {string} content - PCB文件内容
   * @returns {Object} 解析结果
   */
  parse(content) {
    const lines = content.split('\n');
    let i = 0;

    while (i < lines.length) {
      const line = lines[i].trim();

      if (line.startsWith('FileVersion[')) {
        this.fileVersion = this._extractBracketContent(line);
      } else if (line.startsWith('PCB[')) {
        this.pcbInfo = this._parsePCBLine(line);
      } else if (line.startsWith('Element[')) {
        const element = this._parseElement(lines, i);
        if (element) {
          this.elements.push(element);
          i = element.endIndex;
        }
      } else if (line.startsWith('NetList')) {
        const netlist = this._parseNetlist(lines, i);
        if (netlist) {
          this.netlists.push(netlist);
          i = netlist.endIndex;
        }
      }

      i++;
    }

    return {
      fileVersion: this.fileVersion,
      pcbInfo: this.pcbInfo,
      elements: this.elements,
      netlists: this.netlists
    };
  }

  /**
   * 提取方括号内的内容
   */
  _extractBracketContent(line) {
    const match = line.match(/\[([^\]]+)\]/);
    return match ? match[1] : '';
  }

  /**
   * 解析PCB行
   */
  _parsePCBLine(line) {
    const match = line.match(/PCB\["([^"]*)"\s+(\d+)\s+(\d+)\]/);
    if (match) {
      return {
        name: match[1],
        width: parseInt(match[2]),
        height: parseInt(match[3])
      };
    }
    return {};
  }

  /**
   * 解析Element（简化版）
   */
  _parseElement(lines, startIndex) {
    const line = lines[startIndex];
    const match = line.match(/Element\[([^\]]+)\]/);

    if (!match) return null;

    const parts = match[1].split(/\s+/);
    const element = {
      flags: parts[0] || '',
      desc: this._unquote(parts[1] || ''),
      name: this._unquote(parts[2] || ''),
      value: this._unquote(parts[3] || ''),
      mx: parseInt(parts[4]) || 0,
      my: parseInt(parts[5]) || 0,
      pins: [],
      pads: [],
      endIndex: startIndex
    };

    // 查找Element的结束位置
    let depth = 0;
    for (let i = startIndex; i < lines.length; i++) {
      const l = lines[i].trim();
      if (l.includes('(')) depth++;
      if (l.includes(')')) {
        depth--;
        if (depth === 0) {
          element.endIndex = i;
          break;
        }
      }

      // 解析Pin
      if (l.startsWith('Pin[')) {
        const pin = this._parsePin(l);
        if (pin) element.pins.push(pin);
      }

      // 解析Pad
      if (l.startsWith('Pad[')) {
        const pad = this._parsePad(l);
        if (pad) element.pads.push(pad);
      }
    }

    return element;
  }

  /**
   * 解析Pin
   */
  _parsePin(line) {
    const match = line.match(/Pin\[([^\]]+)\]/);
    if (!match) return null;

    const parts = match[1].split(/\s+/);
    return {
      rx: parseInt(parts[0]) || 0,
      ry: parseInt(parts[1]) || 0,
      thickness: parseInt(parts[2]) || 0,
      clearance: parseInt(parts[3]) || 0,
      mask: parseInt(parts[4]) || 0,
      drill: parseInt(parts[5]) || 0,
      name: this._unquote(parts[6] || ''),
      number: this._unquote(parts[7] || '')
    };
  }

  /**
   * 解析Pad
   */
  _parsePad(line) {
    const match = line.match(/Pad\[([^\]]+)\]/);
    if (!match) return null;

    const parts = match[1].split(/\s+/);
    return {
      rx1: parseInt(parts[0]) || 0,
      ry1: parseInt(parts[1]) || 0,
      rx2: parseInt(parts[2]) || 0,
      ry2: parseInt(parts[3]) || 0,
      thickness: parseInt(parts[4]) || 0,
      clearance: parseInt(parts[5]) || 0,
      mask: parseInt(parts[6]) || 0,
      name: this._unquote(parts[7] || ''),
      number: this._unquote(parts[8] || '')
    };
  }

  /**
   * 解析Netlist（简化版）
   */
  _parseNetlist(lines, startIndex) {
    const netlist = {
      nets: [],
      endIndex: startIndex
    };

    let depth = 0;
    let currentNet = null;

    for (let i = startIndex; i < lines.length; i++) {
      const line = lines[i].trim();

      if (line.includes('(')) depth++;
      if (line.includes(')')) {
        depth--;
        if (depth === 0) {
          netlist.endIndex = i;
          break;
        }
      }

      // 解析Net
      if (line.startsWith('Net(')) {
        const match = line.match(/Net\("([^"]+)"\s+"([^"]+)"\)/);
        if (match) {
          if (currentNet) netlist.nets.push(currentNet);
          currentNet = {
            name: match[1],
            style: match[2],
            connects: []
          };
        }
      }

      // 解析Connect
      if (line.startsWith('Connect(') && currentNet) {
        const match = line.match(/Connect\("([^"]+)"\)/);
        if (match) {
          currentNet.connects.push(match[1]);
        }
      }
    }

    if (currentNet) netlist.nets.push(currentNet);

    return netlist;
  }

  /**
   * 移除引号
   */
  _unquote(str) {
    return str.replace(/^"(.*)"$/, '$1');
  }
}
