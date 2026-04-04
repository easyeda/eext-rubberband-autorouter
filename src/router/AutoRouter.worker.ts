/**
 * PCB 自动布线 Worker
 * 在独立线程中运行，避免阻塞主线程
 */

import { PCBParser } from './PCBParser';
import { DRCManager } from './DRCManager';
import { RouterCore } from './RouterCore';

// Worker 消息类型
interface WorkerMessage {
    type: 'start' | 'cancel' | 'progress' | 'complete' | 'error' | 'log';
    data?: any;
}

// 布线请求
interface RoutingRequest {
    maxPasses: number;
}

// 布线结果
interface RoutingResult {
    success: boolean;
    traces: TraceData[];
    vias: ViaData[];
    stats: RoutingStats;
}

interface TraceData {
    net: string;
    layer: number;
    points: { x: number; y: number }[];
    width: number;
}

interface ViaData {
    net: string;
    x: number;
    y: number;
    fromLayer: number;
    toLayer: number;
    diameter: number;
    holeDiameter: number;
}

interface RoutingStats {
    totalNets: number;
    routedNets: number;
    failedNets: number;
    totalTraces: number;
    totalVias: number;
    duration: number;
}

// 全局状态
let isRunning = false;
let shouldCancel = false;
let routerCore: RouterCore | null = null;

// 消息处理
self.onmessage = async (event: MessageEvent<WorkerMessage>) => {
    const message = event.data;
    
    try {
        switch (message.type) {
            case 'start':
                await handleStart(message.data);
                break;
            case 'cancel':
                handleCancel();
                break;
        }
    } catch (error) {
        sendMessage({
            type: 'error',
            data: {
                message: error instanceof Error ? error.message : String(error),
                stack: error instanceof Error ? error.stack : undefined
            }
        });
    }
};

/**
 * 处理布线开始请求
 */
async function handleStart(request: RoutingRequest & { pcbData: any; drcRules: any }) {
    if (isRunning) {
        throw new Error('布线正在进行中');
    }
    
    isRunning = true;
    shouldCancel = false;
    const startTime = performance.now();
    
    try {
        sendMessage({ type: 'log', data: { message: '开始初始化布线器...', level: 'info' } });
        
        // 1. 解析 PCB 数据
        sendMessage({ type: 'log', data: { message: '正在解析 PCB 数据...', level: 'info' } });
        const parser = new PCBParser();
        const pcbData = await parser.parse(request.pcbData);
        
        sendMessage({ 
            type: 'log', 
            data: { message: `解析完成：${pcbData.pads.length} 个焊盘，${pcbData.nets.length} 个网络`, level: 'success' } 
        });
        
        // 2. 初始化 DRC 管理器
        sendMessage({ type: 'log', data: { message: '正在加载设计规则...', level: 'info' } });
        const drcManager = new DRCManager(request.drcRules);
        const rules = drcManager.getRules();
        
        sendMessage({ 
            type: 'log', 
            data: { 
                message: `规则加载完成：线宽 ${rules.physics.traceWidth}mil, 间距 ${rules.clearance.traceToTrace}mil`, 
                level: 'success' 
            } 
        });
        
        // 3. 初始化布线器
        sendMessage({ type: 'log', data: { message: '正在初始化布线引擎...', level: 'info' } });
        routerCore = new RouterCore(pcbData, rules);
        
        // 4. 执行布线
        sendMessage({ type: 'log', data: { message: `开始布线（最大轮数：${request.maxPasses}）...`, level: 'info' } });
        const result = await routerCore.route(request.maxPasses, (progress) => {
            sendMessage({ type: 'progress', data: progress });
        }, (log) => {
            sendMessage({ type: 'log', data: log });
        }, () => shouldCancel);
        
        // 5. 返回结果
        const duration = performance.now() - startTime;
        const routingResult: RoutingResult = {
            success: result.success,
            traces: result.traces,
            vias: result.vias,
            stats: {
                totalNets: pcbData.nets.length,
                routedNets: result.routedNets,
                failedNets: pcbData.nets.length - result.routedNets,
                totalTraces: result.traces.length,
                totalVias: result.vias.length,
                duration
            }
        };
        
        sendMessage({ 
            type: 'log', 
            data: { 
                message: `布线完成！成功 ${result.routedNets}/${pcbData.nets.length} 个网络，耗时 ${duration.toFixed(0)}ms`, 
                level: 'success' 
            } 
        });
        
        sendMessage({ type: 'complete', data: routingResult });
        
    } catch (error) {
        const duration = performance.now() - startTime;
        sendMessage({ 
            type: 'log', 
            data: { 
                message: `布线失败：${error instanceof Error ? error.message : String(error)} (耗时 ${duration.toFixed(0)}ms)`, 
                level: 'error' 
            } 
        });
        throw error;
    } finally {
        isRunning = false;
        routerCore = null;
    }
}

/**
 * 处理取消请求
 */
function handleCancel() {
    shouldCancel = true;
    sendMessage({ type: 'log', data: { message: '正在取消布线...', level: 'warn' } });
}

/**
 * 发送消息到主线程
 */
function sendMessage(message: WorkerMessage) {
    self.postMessage(message);
}

// 启动日志
sendMessage({ 
    type: 'log', 
    data: { message: 'PCB 自动布线 Worker 已就绪 (v2.8.0)', level: 'success' } 
});
