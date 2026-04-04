// 单位转换工具
import {
  MIL_TO_CMIL,
  MM_TO_CMIL,
  UM_TO_CMIL,
  NM_TO_CMIL,
  CMIL_TO_CMIL
} from './Constants.js';

// 单位映射表
const UNIT_MAP = {
  '': CMIL_TO_CMIL,
  'mil': MIL_TO_CMIL,
  'nm': NM_TO_CMIL,
  'um': UM_TO_CMIL,
  'mm': MM_TO_CMIL
};

/**
 * 应用单位转换
 * @param {string} valueWithUnit - 带单位的值，如 "10mil", "0.5mm"
 * @returns {number} 转换后的值（单位：cmil）
 */
export function applyUnit(valueWithUnit) {
  const match = String(valueWithUnit).match(/([+-]?\d+(?:\.\d+)?)(\w*)/);
  if (!match) {
    throw new Error(`Invalid unit format: ${valueWithUnit}`);
  }

  const [, numStr, unit] = match;
  const factor = UNIT_MAP[unit];

  if (factor === undefined) {
    throw new Error(`Invalid unit: ${unit}`);
  }

  return parseFloat(numStr) * factor;
}

/**
 * 将cmil转换为其他单位
 * @param {number} cmil - cmil值
 * @param {string} targetUnit - 目标单位
 * @returns {number} 转换后的值
 */
export function fromCmil(cmil, targetUnit = 'mil') {
  const factor = UNIT_MAP[targetUnit];
  if (factor === undefined) {
    throw new Error(`Invalid target unit: ${targetUnit}`);
  }
  return cmil / factor;
}
