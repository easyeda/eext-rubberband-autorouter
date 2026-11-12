#!/usr/bin/env bash
# 无头 Linux 服务器上的嘉立创 EDA 专业版（lceda-pro）客户端 CLI 封装
#
# 用法:
#   scripts/lceda-env.sh start            确保虚拟显示 + EDA 服务端已就绪（幂等），打印一行状态
#   scripts/lceda-env.sh stop             停服务端（lceda-pro stop），再杀掉本脚本管理的 Xvfb.local
#   scripts/lceda-env.sh status           打印 doctor JSON（connected=true 时退出码 0，否则 1）
#   scripts/lceda-env.sh                  无参数: 打印用法并以退出码 2 退出，不启动任何东西（-h/--help 打印用法、退出码 0）
#   scripts/lceda-env.sh <CLI 参数…>      先静默确保就绪，再 exec 客户端并在参数末尾追加 --no-sandbox
#                                         （stdout 原样透传，stderr 不过滤，退出码即 CLI 的退出码）
#   例: scripts/lceda-env.sh open --path X.eprj3 --headless true
#
# 环境变量（均可选）:
#   LCEDA_PRO_HOME   安装根目录，默认 $HOME/tools/lceda-pro
#                    二进制 = $LCEDA_PRO_HOME/lceda-pro/lceda-pro，sysroot = $LCEDA_PRO_HOME/sysroot
#   LCEDA_DISPLAY    使用/启动的 X 显示号，默认 :99
#                    未设置 LCEDA_DISPLAY 但环境里已有 DISPLAY 时，直接用现有 DISPLAY，不启动 Xvfb
#
# 为什么需要它（本机无 sudo、无系统显示）:
#   1. 缺的系统库（GTK/NSS/X11 等）用 `apt-get download <包>` 下载 .deb，再 `dpkg-deb -x X.deb $LCEDA_PRO_HOME/sysroot`
#      解包到用户目录（不安装），运行时靠 LD_LIBRARY_PATH=$SYSROOT/usr/lib/x86_64-linux-gnu 找到它们。
#   2. 虚拟显示用 sysroot 里解出的 Xvfb。解包出的 Xvfb 硬编码了系统路径，故另存了一份打过补丁的
#      $SYSROOT/usr/bin/Xvfb.local；它运行时以相对路径调用 ./xkbcomp，所以【必须】以 cwd=$SYSROOT/usr/bin 启动。
#   3. 客户端 CLI 会自动拉起服务端（owner），但在本机 GPU 初始化阶段 15s 超时失败；
#      因此改为手动分离启动服务端: lceda-pro --no-sandbox --disable-gpu（约 20s 就绪）。
#   4. --no-sandbox 必须放在子命令及其选项【之后】，放在子命令之前会破坏子命令解析。
#   5. 客户端需要激活文件（向所有者索取）；未激活时 open 会返回 BRIDGE_NOT_READY。
#
# 日志: $LCEDA_PRO_HOME/xvfb.log、$LCEDA_PRO_HOME/server.log
# 服务端异常退出会残留 $TMPDIR/<endpoint>.sock，导致之后再也起不来，本脚本启动前会自动清理。
# 进程名恰为 lceda-pro / Xvfb.local；本脚本只用 pgrep -x / pkill -x，绝不用 pkill -f（会误杀调用方 shell）。
set -euo pipefail

usage() {
	cat <<'EOF'
用法: scripts/lceda-env.sh <start|stop|status|CLI 参数…>
  start     确保虚拟显示 + EDA 服务端已就绪（幂等）
  stop      停服务端并杀掉本脚本管理的 Xvfb.local
  status    打印 doctor JSON（connected=true 时退出码 0）
  其它参数  静默确保就绪后转发给客户端 CLI（自动追加 --no-sandbox）
无参数不会启动任何东西，只打印本说明并以退出码 2 退出。
EOF
}
# 无参数 / -h：只打印用法，绝不启动 Xvfb 或服务端（先于任何环境检查）
if [ $# -eq 0 ]; then usage >&2; exit 2; fi
case "$1" in -h|--help) usage; exit 0 ;; esac

LCEDA_PRO_HOME="${LCEDA_PRO_HOME:-$HOME/tools/lceda-pro}"
BIN="$LCEDA_PRO_HOME/lceda-pro/lceda-pro"
SYSROOT="$LCEDA_PRO_HOME/sysroot"
XVFB_LOG="$LCEDA_PRO_HOME/xvfb.log"
SERVER_LOG="$LCEDA_PRO_HOME/server.log"
SERVER_PID="$LCEDA_PRO_HOME/server.pid"
READY_TIMEOUT=90

die() { echo "[lceda-env] 错误: $*" >&2; exit 1; }
note() { [ "${QUIET:-0}" = 1 ] || echo "[lceda-env] $*" >&2; }

[ -x "$BIN" ] || die "找不到客户端二进制: $BIN（期望位置；可用环境变量 LCEDA_PRO_HOME 指定安装根目录）"

# 显示号决策: LCEDA_DISPLAY > 现有 DISPLAY（外部显示，不管 Xvfb） > :99
MANAGE_XVFB=1
if [ -n "${LCEDA_DISPLAY:-}" ]; then
	DISP="$LCEDA_DISPLAY"
elif [ -n "${DISPLAY:-}" ]; then
	DISP="$DISPLAY"
	MANAGE_XVFB=0
else
	DISP=":99"
fi

HAS_SYSROOT=0
[ -d "$SYSROOT" ] && HAS_SYSROOT=1
if [ "$HAS_SYSROOT" = 0 ] && [ "$MANAGE_XVFB" = 1 ]; then
	die "找不到 sysroot: $SYSROOT（期望位置；可用 LCEDA_PRO_HOME 覆盖）。若已有可用的 X 显示，请先设置 DISPLAY 后重试"
fi
if [ "$HAS_SYSROOT" = 1 ]; then
	export LD_LIBRARY_PATH="$SYSROOT/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
export DISPLAY="$DISP"

# 显示号 :N → 套接字 /tmp/.X11-unix/XN
disp_socket() { echo "/tmp/.X11-unix/X${DISP#:}"; }

# doctor 的 JSON 行（最后一行以 { 开头的 stdout）；失败返回空
doctor_json() {
	local out
	out="$(timeout 40 "$BIN" doctor --no-sandbox 2>/dev/null || true)"
	printf '%s\n' "$out" | grep '^[[:space:]]*{' | tail -n 1 || true
}
is_connected() { local j; j="$(doctor_json)"; grep -q '"connected":[[:space:]]*true' <<<"$j"; }

ensure_xvfb() {
	[ "$MANAGE_XVFB" = 1 ] || return 0
	[ -S "$(disp_socket)" ] && return 0
	local xvfb="$SYSROOT/usr/bin/Xvfb.local"
	[ -x "$xvfb" ] || die "找不到补丁版 Xvfb: $xvfb（期望位置；可用 LCEDA_PRO_HOME 覆盖）"
	note "启动 Xvfb $DISP ..."
	# （9>&- 防止后台进程继承启动锁的文件描述符）
	# 必须 cwd=$SYSROOT/usr/bin：Xvfb.local 以相对路径调用 ./xkbcomp
	( cd "$SYSROOT/usr/bin"; setsid nohup ./Xvfb.local "$DISP" -screen 0 1920x1080x24 -nolisten tcp \
		-xkbdir /usr/share/X11/xkb >>"$XVFB_LOG" 2>&1 < /dev/null 9>&- & )
	local i
	for i in $(seq 1 30); do
		[ -S "$(disp_socket)" ] && return 0
		sleep 0.5
	done
	die "Xvfb 未能在 15s 内就绪（显示 $DISP），见日志 $XVFB_LOG"
}

# 清理残留的 unix 套接字: 服务端异常退出会留下 $TMPDIR/<endpoint>.sock，
# 之后新进程 listen 报 EADDRINUSE、connect 又被拒，只能傻等 15s 后报 ENDPOINT_UNAVAILABLE
clean_stale_socket() {
	pgrep -x -u "$(id -u)" lceda-pro >/dev/null && return 0
	local ep sock
	ep="$(doctor_json | sed -n 's/.*"endpoint":"\([^"]*\)".*/\1/p')"
	[ -n "$ep" ] || return 0
	sock="${TMPDIR:-/tmp}/$ep.sock"
	if [ -S "$sock" ]; then
		note "清理残留套接字 $sock"
		rm -f "$sock"
	fi
}

launch_server() {
	clean_stale_socket
	note "启动 EDA 服务端（日志 $SERVER_LOG）..."
	setsid nohup "$BIN" --no-sandbox --disable-gpu >>"$SERVER_LOG" 2>&1 < /dev/null 9>&- &
	echo $! >"$SERVER_PID"
	disown 2>/dev/null || true
}

ensure_server() {
	if is_connected; then return 0; fi
	local pid="" launches=0
	[ -f "$SERVER_PID" ] && pid="$(cat "$SERVER_PID" 2>/dev/null || true)"
	if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
		note "服务端进程 $pid 已存在，等待就绪 ..."
	else
		launch_server; launches=1
		pid="$(cat "$SERVER_PID")"
	fi
	local t0=$SECONDS
	while [ $((SECONDS - t0)) -lt "$READY_TIMEOUT" ]; do
		sleep 3
		if is_connected; then return 0; fi
		# 刚 stop 过时旧实例可能还在退出，新进程会因单实例锁直接退出；进程没了就重拉一次
		if ! kill -0 "$pid" 2>/dev/null && [ "$launches" -lt 3 ] && ! pgrep -x -u "$(id -u)" lceda-pro >/dev/null; then
			launch_server; launches=$((launches + 1))
			pid="$(cat "$SERVER_PID")"
		fi
	done
	die "EDA 服务端 ${READY_TIMEOUT}s 内未就绪（doctor 未返回 connected:true），见日志 $SERVER_LOG"
}

# 并发调用（如多个批处理同时跑）时串行化启动过程
ensure_started() {
	mkdir -p "$LCEDA_PRO_HOME"
	if command -v flock >/dev/null 2>&1; then
		exec 9>"$LCEDA_PRO_HOME/start.lock"
		flock 9
	fi
	ensure_xvfb
	ensure_server
	if command -v flock >/dev/null 2>&1; then flock -u 9; exec 9>&-; fi
}

cmd="$1"
case "$cmd" in
	start)
		ensure_started
		echo "[lceda-env] 就绪: 显示 $DISP，服务端 connected:true"
		;;
	stop)
		"$BIN" stop --no-sandbox 2>/dev/null || true
		rm -f "$SERVER_PID"
		# 等服务端进程全部退出（最多 20s），否则紧接着的 start 会撞上旧实例
		for _ in $(seq 1 40); do
			pgrep -x -u "$(id -u)" lceda-pro >/dev/null || break
			sleep 0.5
		done
		if [ "$MANAGE_XVFB" = 1 ]; then
			# 只杀本显示号对应的 Xvfb.local（按 /proc/PID/cmdline 匹配显示号）
			for p in $(pgrep -x Xvfb.local || true); do
				if tr '\0' ' ' <"/proc/$p/cmdline" 2>/dev/null | grep -q -- " $DISP "; then
					kill "$p" 2>/dev/null || true
				fi
			done
		fi
		echo "[lceda-env] 已停止"
		;;
	status)
		j="$(doctor_json)"
		if [ -z "$j" ]; then echo '{"connected":false,"error":"doctor 无 JSON 输出"}'; exit 1; fi
		echo "$j"
		grep -q '"connected":[[:space:]]*true' <<<"$j"
		;;
	*)
		QUIET=1 ensure_started
		exec "$BIN" "$@" --no-sandbox
		;;
esac
