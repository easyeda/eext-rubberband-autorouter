// 常量定义
// 内部单位: 0.01 mil (cmil)

// 最大板对角线长度 - 用作"无穷大"值
export const MBD = 2 ** 30 - 2 ** 24;

// 平均过孔直径 (50 mil)
export const AVD = 5000;

// 平均走线宽度 (10 mil)
export const ATW = 1000;

// 单位转换系数
export const INCH_TO_NM = 25.4e6;
export const MIL_TO_NM = 25.4e3;
export const CMIL_TO_NM = 25.4e1;

export const NM_TO_CMIL = 1.0 / CMIL_TO_NM;
export const UM_TO_CMIL = 1e3 / CMIL_TO_NM;
export const MM_TO_CMIL = 1e6 / CMIL_TO_NM;
export const MIL_TO_CMIL = 100;
export const CMIL_TO_CMIL = 1;

// 测试用常数
export const PCB_SIZE = 140000;
export const POINTS = 64;
export const PIN_RADIUS = 1000;
export const TRACE_WIDTH = 1200;
export const CLEARANCE = 800;
export const MIN_CUT_SIZE = 6000;
