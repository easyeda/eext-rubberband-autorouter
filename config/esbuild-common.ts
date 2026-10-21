import esbuild from 'esbuild';
import path from 'path';

/**
 * 把 Worker 入口单独打成 IIFE 字符串并内联进主包。
 * 主包运行时用 Blob URL 创建 Worker，无需额外文件。
 */
const inlinePlanningWorker: esbuild.Plugin = {
	name: 'inline-planning-worker',
	setup(build) {
		build.onResolve({ filter: /^inline-worker:planning$/ }, args => ({
			path: args.path,
			namespace: 'inline-worker',
		}));
		build.onLoad({ filter: /.*/, namespace: 'inline-worker' }, async () => {
			const result = await esbuild.build({
				entryPoints: [path.resolve(process.cwd(), 'src/worker/planning-worker.ts')],
				bundle: true,
				write: false,
				format: 'iife',
				platform: 'browser',
				target: 'es2020',
				minify: false,
				sourcemap: false,
				logLevel: 'silent',
			});
			return { contents: result.outputFiles[0].text, loader: 'text' };
		});
	},
};

export default {
	entryPoints: {
		'index': './src/index',
	},
	entryNames: '[name]',
	assetNames: '[name]',
	bundle: true, // 用于内部方法调用，请勿修改
	minify: false, // 用于内部方法调用，请勿修改
	loader: {},
	outdir: './dist/',
	sourcemap: undefined,
	platform: 'browser', // 用于内部方法调用，请勿修改
	format: 'iife', // 用于内部方法调用，请勿修改
	globalName: 'edaEsbuildExportName', // 用于内部方法调用，请勿修改
	treeShaking: true,
	ignoreAnnotations: true,
	define: {},
	external: [],
	plugins: [inlinePlanningWorker],
} satisfies Parameters<(typeof esbuild)['build']>[0];
