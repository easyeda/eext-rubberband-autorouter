/**
 * 分级日志工具
 */

export type LogLevel = 'info' | 'warn' | 'error' | 'success';

export class Logger {
	constructor(private prefix: string) {}

	info(msg: string): void {
		console.log(`${this.prefix} ${msg}`);
	}

	warn(msg: string): void {
		console.warn(`${this.prefix} ${msg}`);
	}

	error(msg: string): void {
		console.error(`${this.prefix} ${msg}`);
	}

	success(msg: string): void {
		console.log(`${this.prefix} ✅ ${msg}`);
	}
}
