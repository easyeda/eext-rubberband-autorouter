/**
 * 错误处理和重试机制模块
 * 提供：
 * 1. 错误分类和详细错误信息
 * 2. 自动重试机制（指数退避）
 * 3. 错误恢复策略
 * 4. 错误日志和报告
 */

/**
 * 错误类型枚举
 */
export enum ErrorType {
	/** 网络错误 */
	NETWORK = 'NETWORK',
	/** 数据解析错误 */
	PARSE_ERROR = 'PARSE_ERROR',
	/** DRC 规则错误 */
	DRC_ERROR = 'DRC_ERROR',
	/** 布线算法错误 */
	ROUTING_ERROR = 'ROUTING_ERROR',
	/** PCB API 错误 */
	PCB_API_ERROR = 'PCB_API_ERROR',
	/** 资源不足错误 */
	RESOURCE_ERROR = 'RESOURCE_ERROR',
	/** 用户取消 */
	CANCELLED = 'CANCELLED',
	/** 未知错误 */
	UNKNOWN = 'UNKNOWN',
}

/**
 * 错误严重程度
 */
export enum ErrorSeverity {
	/** 信息性错误，可忽略 */
	INFO = 'info',
	/** 警告，可能影响结果 */
	WARNING = 'warning',
	/** 错误，操作失败 */
	ERROR = 'error',
	/** 严重错误，无法继续 */
	CRITICAL = 'critical',
}

/**
 * 错误详情接口
 */
export interface ErrorDetails {
	/** 错误类型 */
	type: ErrorType;
	/** 严重程度 */
	severity: ErrorSeverity;
	/** 错误消息 */
	message: string;
	/** 详细原因 */
	cause?: string;
	/** 建议的解决方案 */
	suggestion?: string;
	/** 相关上下文 */
	context?: Record<string, unknown>;
	/** 原始错误 */
	originalError?: Error;
	/** 时间戳 */
	timestamp: number;
	/** 重试次数 */
	retryCount?: number;
}

/**
 * 重试配置
 */
export interface RetryConfig {
	/** 最大重试次数 */
	maxRetries: number;
	/** 初始延迟（毫秒） */
	initialDelay: number;
	/** 最大延迟（毫秒） */
	maxDelay: number;
	/** 退避因子 */
	backoffFactor: number;
	/** 可重试的错误类型 */
	retryableErrorTypes: ErrorType[];
}

const DEFAULT_RETRY_CONFIG: RetryConfig = {
	maxRetries: 3,
	initialDelay: 100,
	maxDelay: 5000,
	backoffFactor: 2,
	retryableErrorTypes: [
		ErrorType.NETWORK,
		ErrorType.PCB_API_ERROR,
		ErrorType.RESOURCE_ERROR,
	],
};

/**
 * 错误日志回调
 */
export type ErrorLogCallback = (details: ErrorDetails) => void;

/**
 * 错误处理器类
 */
export class ErrorHandler {
	private errorLog: ErrorDetails[] = [];
	private onLog?: ErrorLogCallback;
	private retryConfig: RetryConfig;

	constructor(onLog?: ErrorLogCallback, retryConfig?: Partial<RetryConfig>) {
		this.onLog = onLog;
		this.retryConfig = { ...DEFAULT_RETRY_CONFIG, ...retryConfig };
	}

	/**
	 * 创建错误详情
	 */
	createError(
		type: ErrorType,
		severity: ErrorSeverity,
		message: string,
		options?: {
			cause?: string;
			suggestion?: string;
			context?: Record<string, unknown>;
			originalError?: Error;
		}
	): ErrorDetails {
		const details: ErrorDetails = {
			type,
			severity,
			message,
			cause: options?.cause,
			suggestion: options?.suggestion,
			context: options?.context,
			originalError: options?.originalError,
			timestamp: Date.now(),
		};

		this.errorLog.push(details);
		this.onLog?.(details);

		return details;
	}

	/**
	 * 判断错误是否可重试
	 */
	isRetryable(error: ErrorDetails): boolean {
		return this.retryConfig.retryableErrorTypes.includes(error.type);
	}

	/**
	 * 计算重试延迟（指数退避）
	 */
	calculateRetryDelay(retryCount: number): number {
		const delay = this.retryConfig.initialDelay * 
			Math.pow(this.retryConfig.backoffFactor, retryCount);
		return Math.min(delay, this.retryConfig.maxDelay);
	}

	/**
	 * 执行带重试的操作
	 */
	async withRetry<T>(
		operation: () => Promise<T>,
		errorType: ErrorType,
		context?: Record<string, unknown>
	): Promise<T> {
		let lastError: ErrorDetails | undefined = undefined;

		for (let attempt = 0; attempt <= this.retryConfig.maxRetries; attempt++) {
			try {
				return await operation();
			} catch (error) {
				const details = this.createError(
					errorType,
					ErrorSeverity.ERROR,
					error instanceof Error ? error.message : String(error),
					{
						originalError: error instanceof Error ? error : undefined,
						context: { ...context, attempt: attempt + 1, retryCount: attempt },
					}
				);

				lastError = details;

				// 检查是否应该重试
				if (attempt < this.retryConfig.maxRetries && this.isRetryable(details)) {
					const delay = this.calculateRetryDelay(attempt);
					console.log(`[ErrorHandler] 重试 ${attempt + 1}/${this.retryConfig.maxRetries}，延迟 ${delay}ms`);
					await this.sleep(delay);
				} else {
					break;
				}
			}
		}

		// 所有重试都失败了
		throw new RoutingError(
			lastError?.message || '操作失败',
			errorType,
			lastError?.severity || ErrorSeverity.ERROR,
			lastError
		);
	}

	/**
	 * 获取错误日志
	 */
	getErrorLog(): ErrorDetails[] {
		return [...this.errorLog];
	}

	/**
	 * 清除错误日志
	 */
	clearErrorLog(): void {
		this.errorLog = [];
	}

	/**
	 * 生成错误报告
	 */
	generateReport(): string {
		if (this.errorLog.length === 0) {
			return '无错误记录';
		}

		const lines: string[] = [
			`错误报告（共 ${this.errorLog.length} 条）`,
			'='.repeat(50),
		];

		for (const error of this.errorLog) {
			lines.push(
				`[${error.severity.toUpperCase()}] ${error.type}`,
				`  消息：${error.message}`,
				`  时间：${new Date(error.timestamp).toLocaleString()}`,
			);

			if (error.cause) {
				lines.push(`  原因：${error.cause}`);
			}

			if (error.suggestion) {
				lines.push(`  建议：${error.suggestion}`);
			}

			if (error.context) {
				lines.push(`  上下文：${JSON.stringify(error.context)}`);
			}

			lines.push('-'.repeat(50));
		}

		return lines.join('\n');
	}

	private sleep(ms: number): Promise<void> {
		return new Promise(resolve => setTimeout(resolve, ms));
	}
}

/**
 * 布线错误类
 */
export class RoutingError extends Error {
	constructor(
		message: string,
		public type: ErrorType,
		public severity: ErrorSeverity,
		public details?: ErrorDetails
	) {
		super(message);
		this.name = 'RoutingError';
	}

	/**
	 * 转换为友好的错误消息
	 */
	toFriendlyMessage(): string {
		const prefix = this.getSeverityPrefix();
		const suggestion = this.details?.suggestion 
			? ` 建议：${this.details.suggestion}` 
			: '';
		return `${prefix}${this.message}${suggestion}`;
	}

	private getSeverityPrefix(): string {
		switch (this.severity) {
			case ErrorSeverity.INFO:
				return '[提示] ';
			case ErrorSeverity.WARNING:
				return '[警告] ';
			case ErrorSeverity.ERROR:
				return '[错误] ';
			case ErrorSeverity.CRITICAL:
				return '[严重错误] ';
			default:
				return '';
		}
	}
}

/**
 * 预定义的错误消息模板
 */
export const ErrorMessages = {
	[ErrorType.NETWORK]: {
		message: '网络连接失败',
		suggestion: '请检查网络连接或稍后重试',
	},
	[ErrorType.PARSE_ERROR]: {
		message: '数据解析失败',
		suggestion: '请检查 PCB 数据格式是否正确',
	},
	[ErrorType.DRC_ERROR]: {
		message: 'DRC 规则读取失败',
		suggestion: '将使用默认 DRC 规则，可能影响布线质量',
	},
	[ErrorType.ROUTING_ERROR]: {
		message: '布线算法执行失败',
		suggestion: '尝试减少布线轮数或简化 PCB 布局',
	},
	[ErrorType.PCB_API_ERROR]: {
		message: 'PCB API 调用失败',
		suggestion: '请确保在 PCB 编辑器中操作',
	},
	[ErrorType.RESOURCE_ERROR]: {
		message: '资源不足',
		suggestion: '关闭其他应用程序后重试',
	},
	[ErrorType.CANCELLED]: {
		message: '操作已取消',
		suggestion: '',
	},
	[ErrorType.UNKNOWN]: {
		message: '发生未知错误',
		suggestion: '请查看详细日志或联系技术支持',
	},
};

/**
 * 创建标准错误
 */
export function createStandardError(
	type: ErrorType,
	severity: ErrorSeverity = ErrorSeverity.ERROR,
	originalError?: Error,
	context?: Record<string, unknown>
): RoutingError {
	const template = ErrorMessages[type];
	return new RoutingError(
		template.message,
		type,
		severity,
		{
			type,
			severity,
			message: template.message,
			suggestion: template.suggestion,
			originalError,
			context,
			timestamp: Date.now(),
		}
	);
}
