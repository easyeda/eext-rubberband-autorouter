/**
 * DRC 间距规则读取与用户参数覆盖
 */

import { DEFAULT_ROUTER_OPTIONS, RouterUiOptions } from '../types/router-params';

/** PCB 单位：1mil = 0.0254mm */
export const MM_TO_MIL = 39.3701;

export interface DRCRules {
	traceWidthMil: number;
	clearanceMil: number;
	/** Track↔Track 间距 */
	trackToTrackMil: number;
	/** Track↔Line 间距 */
	trackToLineMil: number;
	/** Track↔Board Outline 间距 */
	trackToBoardMil: number;
	/** Track↔SMD/TH Pad、Via 间距 */
	trackToPadMil: number;
	/** Track↔Fill/Zone 间距 */
	trackToFillMil: number;
}

// DRC 回退默认值
const FALLBACK_TRACE_WIDTH_MIL = 8;
const FALLBACK_CLEARANCE_MIL = 8;

export const FALLBACK_DRC_RULES: DRCRules = {
	traceWidthMil: FALLBACK_TRACE_WIDTH_MIL,
	clearanceMil: FALLBACK_CLEARANCE_MIL,
	trackToTrackMil: FALLBACK_CLEARANCE_MIL,
	trackToLineMil: FALLBACK_CLEARANCE_MIL,
	trackToBoardMil: 12,
	trackToPadMil: FALLBACK_CLEARANCE_MIL,
	trackToFillMil: FALLBACK_CLEARANCE_MIL,
};

export type DRCLog = (message: string, level: string) => void;

/** 从当前 PCB 的 DRC 配置读取走线宽度与各类间距 */
export async function fetchDRCRules(onLog?: DRCLog): Promise<DRCRules> {
	try {
		const config = await eda.pcb_Drc.getCurrentRuleConfiguration();
		const cfg = (config as any)?.config;

		const spacing = cfg?.Spacing?.['Safe Spacing'];
		let trackToTrackMm = 0.1016;
		let trackToPadMm = 0.1524;
		let trackToLineMm = 0.1524;
		let trackToBoardMm = 0.3;
		let trackToFillMm = 0.254;

		if (spacing) {
			const tableKey = Object.keys(spacing).find(k => spacing[k].isSetDefault) ?? Object.keys(spacing)[0];
			const tbl = spacing[tableKey];
			const content: number[][] = tbl?.tables?.['1']?.content ?? [];
			const cols: string[] = tbl?.column ?? [];
			const rows: string[] = tbl?.row ?? [];
			const trackColIdx = cols.indexOf('Track');

			const readSpacing = (rowName: string): number | null => {
				const rowIdx = rows.indexOf(rowName);
				if (rowIdx < 0) return null;
				const val = content[rowIdx]?.[Math.min(trackColIdx, content[rowIdx]?.length - 1)];
				return typeof val === 'number' && val > 0 ? val : null;
			};

			trackToTrackMm = readSpacing('Track') ?? trackToTrackMm;
			const padVal = Math.max(
				readSpacing('SMD Pad') ?? 0,
				readSpacing('TH Pad') ?? 0,
				readSpacing('Via') ?? 0,
			);
			if (padVal > 0) trackToPadMm = padVal;
			trackToLineMm = readSpacing('Line') ?? trackToLineMm;
			trackToBoardMm = readSpacing('Board Outline') ?? trackToBoardMm;
			trackToFillMm = readSpacing('Copper/Plane Zone') ?? readSpacing('Fill Region/Teardrop') ?? trackToFillMm;
		}

		// 读取默认走线宽度
		const trackRules = cfg?.Physics?.Track;
		let traceWidthMm = 0.254;
		if (trackRules) {
			const defaultKey =
				Object.keys(trackRules).find(k => trackRules[k].isSetDefault) ??
				Object.keys(trackRules)[0];
			const tw = trackRules[defaultKey]?.form?.data?.['1']?.defaultValue;
			if (typeof tw === 'number' && tw > 0) traceWidthMm = tw;
		}

		const rules: DRCRules = {
			traceWidthMil: traceWidthMm * MM_TO_MIL,
			clearanceMil: Math.max(trackToTrackMm, trackToPadMm) * MM_TO_MIL,
			trackToTrackMil: trackToTrackMm * MM_TO_MIL,
			trackToLineMil: trackToLineMm * MM_TO_MIL,
			trackToBoardMil: trackToBoardMm * MM_TO_MIL,
			trackToPadMil: trackToPadMm * MM_TO_MIL,
			trackToFillMil: trackToFillMm * MM_TO_MIL,
		};

		onLog?.(
			`DRC 规则：线宽 ${rules.traceWidthMil.toFixed(2)}mil，Track 间距 ${rules.trackToTrackMil.toFixed(2)}mil，` +
			`Pad 间距 ${rules.trackToPadMil.toFixed(2)}mil，板框间距 ${rules.trackToBoardMil.toFixed(2)}mil`,
			'info',
		);
		return rules;
	} catch (err) {
		console.warn('[DRCRules] 获取 DRC 规则失败，使用默认值:', err);
		onLog?.('读取 DRC 规则失败，使用默认值', 'warn');
		return { ...FALLBACK_DRC_RULES };
	}
}

/** 把用户在参数弹窗里的设置叠加到 DRC 规则上 */
export function applyRouterOptionsToDRC(drc: DRCRules, options: RouterUiOptions): DRCRules {
	const extra = options.extraClearanceMil;
	return {
		...drc,
		traceWidthMil: options.traceWidthMil,
		clearanceMil: Math.max(drc.trackToTrackMil + extra, drc.trackToPadMil + extra),
		trackToTrackMil: drc.trackToTrackMil + extra,
		trackToLineMil: drc.trackToLineMil + extra,
		trackToBoardMil: drc.trackToBoardMil + extra,
		trackToPadMil: drc.trackToPadMil + extra,
		trackToFillMil: drc.trackToFillMil + extra,
	};
}

/** 参数弹窗的推荐默认值（由 DRC 推导） */
export function deriveDefaultOptions(drc: DRCRules): RouterUiOptions {
	return {
		...DEFAULT_ROUTER_OPTIONS,
		traceWidthMil: Math.round(drc.traceWidthMil * 10) / 10,
		escapeClearanceMil: 0,   // 默认严格按最小距离出线
	};
}
