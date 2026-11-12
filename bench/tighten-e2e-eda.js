// Code file for `lceda-pro invoke --code-file` (async function BODY: top-level return/await are legal; NOT runnable by node).
// Reads __CLI__.args:
//   { cmd:'load', bundlePath }            import the bundle once (cached on globalThis), setup(eda), ensurePcbTab()
//   { cmd:'probe'|'start'|'status'|'chunk', ... }   forwarded to the bundle's handle(args) (see bench/tighten-e2e-entry.ts)
// Long jobs (route / tighten / drc / snapshot) are started in the background by `start` and polled with `status`.
const args = (typeof __CLI__ !== 'undefined' && __CLI__.args) || {};

async function loadBundle(bundlePath) {
	if (globalThis.__RB_E2E_MOD__ && globalThis.__RB_E2E_MOD_PATH__ === bundlePath && !args.reload) return globalThis.__RB_E2E_MOD__;
	const file = await eda.sys_FileSystem.readFileFromFileSystem(bundlePath);
	if (!file) throw new Error('cannot read bundle: ' + bundlePath);
	const buf = new Uint8Array(await file.arrayBuffer());
	let bin = '';
	for (let i = 0; i < buf.length; i += 0x8000) bin += String.fromCharCode.apply(null, buf.subarray(i, i + 0x8000));
	const mod = await import('data:text/javascript;charset=utf-8;base64,' + btoa(bin));
	globalThis.__RB_E2E_MOD__ = mod;
	globalThis.__RB_E2E_MOD_PATH__ = bundlePath;
	return mod;
}

if (args.cmd === 'load') {
	const mod = await loadBundle(args.bundlePath);
	mod.setup(eda);
	await mod.ensurePcbTab();
	return { loaded: true, exports: Object.keys(mod) };
}
const mod = globalThis.__RB_E2E_MOD__;
if (!mod) throw new Error('bundle not loaded in this session (run cmd:"load" first)');
return await mod.handle(args);
