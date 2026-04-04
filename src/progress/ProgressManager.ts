/**
 * 进度管理器
 * 定义布线阶段，统一进度回调通知
 */

export enum ProgressPhase {
	Parsing = 'parsing',
	Routing = 'routing',
	Optimizing = 'optimizing',
	Writing = 'writing',
	Done = 'done',
}

export interface ProgressInfo {
	phase: ProgressPhase;
	percent: number;
	message: string;
}

type UpdateCallback = (info: ProgressInfo) => void;

export class ProgressManager {
	private phase: ProgressPhase = ProgressPhase.Parsing;
	private callback?: UpdateCallback;

	onUpdate(cb: UpdateCallback): void {
		this.callback = cb;
	}

	start(phase: ProgressPhase, message: string): void {
		this.phase = phase;
		this.notify(0, message);
	}

	update(percent: number, message: string): void {
		this.notify(percent, message);
	}

	done(): void {
		this.phase = ProgressPhase.Done;
		this.notify(100, '布线完成');
	}

	private notify(percent: number, message: string): void {
		this.callback?.({ phase: this.phase, percent, message });
	}
}
